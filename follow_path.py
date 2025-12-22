import math
import time
from typing import List, Tuple

from map_loader import load_map, cell_to_world
from plan_test import plan_path_for_goal  # 경로 생성은 기존 함수 재사용

try:
    from jetbot import Robot   # JETANK 기본 제어 클래스
except ImportError:
    Robot = None  # PC 테스트용


# ==========================
# JETANK 모터 제어 래퍼
# ==========================

class JetankRobot:
    """JETANK를 (v, omega) 명령으로 제어하기 위한 래퍼.

    - v_norm   : -1.0 ~ 1.0  (정규화된 전후진 속도)
    - omega_norm: -1.0 ~ 1.0 (정규화된 회전 속도, +는 반시계 방향)

    내부적으로는 jetbot.Robot.set_motors(left, right)를 사용한다.
    """

    def __init__(self):
        if Robot is None:
            print("[WARN] jetbot.Robot 모듈을 찾을 수 없습니다. 모터 제어는 비활성화됩니다.")
            self.robot = None
        else:
            self.robot = Robot()

    def set_velocity(self, v_norm: float, omega_norm: float):
        """정규화된 선속도/각속도를 좌우 바퀴 속도로 변환해서 모터에 전달."""
        # 클램프
        v_norm = max(-1.0, min(1.0, v_norm))
        omega_norm = max(-1.0, min(1.0, omega_norm))

        # 간단한 차동구동 모델
        k_omega = 0.7  # 회전 기여 비율 (기본 샘플에서 잘 동작하던 값)

        left = v_norm - k_omega * omega_norm
        right = v_norm + k_omega * omega_norm

        # 다시 클램프
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        if self.robot is not None:
            self.robot.set_motors(left, right)
        else:
            print(f"[MOTOR] (dry-run) left={left:.2f}, right={right:.2f}")

    def stop(self):
        if self.robot is not None:
            self.robot.set_motors(0.0, 0.0)
        else:
            print("[MOTOR] (dry-run) stop")


# ==========================
# 단순 Pose 추정 (옵션)
# ==========================

class PoseEstimator:
    """(선속도, 각속도, 시간)을 적분해서 (x, y, theta)를 추정.

    - x, y   : 미터 단위, 맵 기준 좌표
    - theta  : 라디안, [-pi, pi]
    """

    def __init__(self, x0: float, y0: float, theta0: float = 0.0):
        self.x = x0
        self.y = y0
        self.theta = theta0

    def update(self, v_mps: float, omega_rad: float, dt: float):
        # dt 동안의 평균 속도/각속도로 적분
        self.x += v_mps * math.cos(self.theta) * dt
        self.y += v_mps * math.sin(self.theta) * dt
        self.theta += omega_rad * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    def set_heading(self, theta: float):
        self.theta = (theta + math.pi) % (2 * math.pi) - math.pi

    def get_pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.theta


# ==========================
# 경로 따라가기 (open-loop, 상수 속도)
# ==========================

def follow_path_constant_speed(
    waypoints_world: List[Tuple[float, float]],
    is_enemy_goal: bool,
    v_cruise_norm: float = 0.35,
    v_charge_norm: float = 0.8,
    omega_turn_norm: float = 0.5,
    scale_v_mps: float = 0.15,
    scale_omega_rad: float = 0.85,
):
    """정해진 경로(웨이포인트)를 따라가는 단순 open-loop 제어.

    - 모든 직선 구간:
        * 적이 아닌 일반 구간: v = v_cruise_norm, omega = 0
        * 적 돌진 마지막 직선: v = v_charge_norm, omega = 0
    - 모든 회전:
        * v = 0, |omega| = omega_turn_norm (항상 동일한 회전 속도)
    - 각 segment마다 "얼마나 오래" 동작할지만 거리/각도에 따라 계산.
    """

    if len(waypoints_world) < 2:
        print("[FOLLOW] Waypoints too short, nothing to do.")
        return

    robot = JetankRobot()

    # Pose 추정은 선택적이지만, 서버 전송이나 디버깅용으로 유지
    x0, y0 = waypoints_world[0]
    pose_est = PoseEstimator(x0, y0, theta0=0.0)

    # 현재 추정 heading (초기에는 +x 방향 가정)
    theta_est = 0.0

    num_segments = len(waypoints_world) - 1

    try:
        for i in range(num_segments):
            sx, sy = waypoints_world[i]
            ex, ey = waypoints_world[i + 1]

            dx = ex - sx
            dy = ey - sy
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                continue

            target_heading = math.atan2(dy, dx)

            # 1) 먼저, 현재 heading에서 target_heading까지 회전 (제자리 회전)
            # ------------------------------------------------------------
            # 회전 각도
            angle_diff = target_heading - theta_est
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_diff) > 1e-3:
                # 회전 방향
                turn_dir = 1.0 if angle_diff > 0 else -1.0
                # 필요한 시간 = 각도 / (실제 각속도)
                omega_real = omega_turn_norm * scale_omega_rad   # rad/s
                turn_time = abs(angle_diff) / max(omega_real, 1e-6)

                print(f"[FOLLOW] Segment {i}: TURN angle={math.degrees(angle_diff):.1f}deg, time={turn_time:.2f}s")

                robot.set_velocity(0.0, turn_dir * omega_turn_norm)
                time.sleep(turn_time)
                robot.stop()

                # pose/heading 업데이트 (이론값 기준)
                pose_est.update(0.0, turn_dir * omega_real, turn_time)
                theta_est = target_heading
            else:
                print(f"[FOLLOW] Segment {i}: TURN skipped (small angle).")

            # 2) 다음, 직선 이동
            # ------------------------------------------------------------
            # 어떤 속도로 갈지 결정 (적 돌진이면 마지막 segment에 대해 v_charge 사용)
            if is_enemy_goal and i == num_segments - 1:
                v_cmd = v_charge_norm
            else:
                v_cmd = v_cruise_norm

            v_real = v_cmd * scale_v_mps  # m/s
            drive_time = seg_len / max(v_real, 1e-6)

            print(f"[FOLLOW] Segment {i}: DRIVE len={seg_len:.3f}m, v={v_cmd:.2f}, time={drive_time:.2f}s")

            robot.set_velocity(v_cmd, 0.0)
            time.sleep(drive_time)
            robot.stop()

            pose_est.update(v_real, 0.0, drive_time)

            # 잠깐 쉬어주기 (기계적 진동/슬립 정리)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("[FOLLOW] Interrupted by user.")
    finally:
        robot.stop()

    x, y, theta = pose_est.get_pose()
    print(f"[FOLLOW] Finished. est pose = ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}deg)")


# ==========================
# 메인: 맵 로딩 + 경로 계획 + 경로 따라가기
# ==========================

def main():
    # 1) 맵 로딩
    print("[MAIN] Loading map ...")
    # TODO: 여기서 사용하는 맵 이름만 네 상황에 맞게 바꿔줘
    m_orig = load_map("map_smallroom_60x60.json")

    if m_orig.start is None:
        raise RuntimeError("Start(S) not found in map")
    start_cell = m_orig.start

    if not m_orig.enemies:
        raise RuntimeError("Enemy(E) not found in map")
    goal_cell = m_orig.enemies[0]

    # 서버에서 적 여부 플래그를 받을 예정이라면 이 값만 바꾸면 됨
    is_enemy_goal = True  # 테스트용

    print(f"[MAIN] Start cell: {start_cell}, Goal cell: {goal_cell}, is_enemy_goal={is_enemy_goal}")

    # 2) 셀 기반 경로 계획
    path_cells, used_enemy_mode = plan_path_for_goal(
        m_orig,
        start_cell=start_cell,
        goal_cell=goal_cell,
        is_enemy_goal=is_enemy_goal,
        inflate_radius=2,
        min_charge_cells=8,
    )

    if path_cells is None:
        print("[MAIN] No path found, abort.")
        return

    print("[MAIN] Planned cells:", path_cells)
    print("[MAIN] Used enemy mode:", used_enemy_mode)

    # 3) 셀 -> 월드 좌표(미터)로 변환
    waypoints_world: List[Tuple[float, float]] = []
    for (cx, cy) in path_cells:
        wx, wy = cell_to_world(cx, cy, m_orig.resolution)
        waypoints_world.append((wx, wy))

    print("[MAIN] Waypoints (world):")
    for i, w in enumerate(waypoints_world):
        print(f"  [{i}] {w}")

    # 4) 상수 속도 경로 따라가기
    follow_path_constant_speed(
        waypoints_world,
        is_enemy_goal=is_enemy_goal,
        v_cruise_norm=0.35,   # 일반 구간 직진 속도 (상수)
        v_charge_norm=0.8,    # 적 돌진 마지막 직선 속도 (상수, 최고속)
        omega_turn_norm=0.5,  # 회전 속도 (상수, 제자리 회전)
        scale_v_mps=0.15,     # 캘리브레이션한 실제 m/s 스케일
        scale_omega_rad=0.85, # 캘리브레이션한 실제 rad/s 스케일
    )


if __name__ == "__main__":
    main()
