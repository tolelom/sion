#!/usr/bin/env python3
import threading
import time
import random
from typing import Optional, Tuple

import cv2

from map_loader import load_map, cell_to_world, world_to_cell
from plan_test import plan_path_for_goal
from follow_path import follow_path_constant_speed


# ============ 설정값 ============

MAP_FILE = "map_smallroom_60x60.json"   # 쓰는 맵 파일 이름
AUTO_INTERVAL = 5.0                     # 자동 모드에서 연속 이동 사이 대기 시간 (초)
STATUS_PERIOD = 0.5                     # 서버로 상태 전송 주기 (초)

# 이동/회전 속도 설정 (follow_path와 맞춰 줄 것)
V_CRUISE = 0.35
V_CHARGE = 0.8
OMEGA_TURN = 0.5
SCALE_V_MPS = 0.15
SCALE_OMEGA_RAD = 0.85


# ============ 글로벌 상태 ============

mode_lock = threading.Lock()
current_mode = "auto"  # "auto" 또는 "manual"

movement_lock = threading.Lock()
movement_thread: Optional[threading.Thread] = None
movement_busy = False

stop_flag = False

# 현재 위치/목표는 간단한 상태만 공유 (정확한 pose는 나중에)
current_target_cell: Optional[Tuple[int, int]] = None
last_auto_time = 0.0


# ============ 서버 통신 (나중에 구현할 부분) ============

def get_mode_and_manual_target_from_server() -> Tuple[str, Optional[Tuple[float, float]]]:
    """
    서버로부터 현재 모드와 수동 모드일 때의 목표 좌표를 받아오는 함수.

    return:
        mode: "auto" 또는 "manual"
        manual_target_world: (wx, wy) [미터 단위 월드 좌표] 또는 None

    TODO:
        - HTTP / WebSocket 등으로 서버와 통신하도록 구현
        - 지금은 예시로 global current_mode만 사용, 좌표는 항상 None 반환
    """
    global current_mode

    # TODO: 여기에서 실제 서버 통신 구현
    # 예: mode, x, y = 요청 결과
    # 일단은 current_mode만 보고 manual 좌표는 None 리턴
    return current_mode, None


def send_status_to_server(status: dict):
    """
    현재 상태(모드, 이동 상태, 목표 등)를 서버에 주기적으로 전송하는 함수.

    status 예시:
        {
          "mode": "auto",
          "moving": True,
          "target_cell": [gx, gy],
          "timestamp": ...
        }

    TODO:
        - requests.post(...) 혹은 WebSocket 등으로 전송
        - 지금은 print만 수행
    """
    print("[STATUS]", status)


# ============ 카메라 스레드 ============

def camera_loop():
    """
    항상 카메라를 켜두고 프레임을 읽는 루프.
    - 나중에 여기에서 적 인식, 스트리밍 등을 붙이면 됨.
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[CAM] Failed to open camera")
        return

    print("[CAM] Camera loop started")
    while not stop_flag:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        # TODO: 여기에서 프레임 처리 / 서버 전송 / 적 인식 등을 수행
        # 지금은 아무것도 안 하고 버림

        # CPU 사용률 줄이려고 살짝 쉼
        time.sleep(0.02)

    cap.release()
    print("[CAM] Camera loop stopped")


# ============ 유틸 함수들 ============

def pick_random_reachable_goal_cell(m, start_cell: Tuple[int, int], is_enemy_goal: bool = False) -> Optional[Tuple[int, int]]:
    """
    맵 전체에서 랜덤 셀을 뽑아서, 거기로 경로가 존재하면 goal로 사용.
    - 장애물 여부는 plan_path_for_goal 결과로 대신 판단 (경로 없으면 실패)
    """
    max_try = 50
    for _ in range(max_try):
        gx = random.randint(0, m.width - 1)
        gy = random.randint(0, m.height - 1)
        goal = (gx, gy)
        if goal == start_cell:
            continue

        path_cells, _ = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal,
            is_enemy_goal=is_enemy_goal,
            inflate_radius=2,
            min_charge_cells=8,
        )
        if path_cells is not None:
            return goal

    return None


def world_to_cell_safe(m, wx: float, wy: float) -> Optional[Tuple[int, int]]:
    """월드 좌표 -> 셀 좌표 변환 (맵 범위 체크 포함)."""
    cx, cy = world_to_cell(wx, wy, m.resolution)
    if 0 <= cx < m.width and 0 <= cy < m.height:
        return (cx, cy)
    return None


# ============ 이동 스레드 ============

def movement_worker(
    m,
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
    is_enemy_goal: bool,
):
    """
    실제로 경로를 만들고 follow_path_constant_speed를 호출하는 작업.
    별도 스레드에서 동작.
    """
    global movement_busy, current_target_cell

    with movement_lock:
        movement_busy = True
        current_target_cell = goal_cell

    print(f"[MOVE] New task: start={start_cell}, goal={goal_cell}, enemy={is_enemy_goal}")

    # 1) 셀 기반 경로 계획
    path_cells, used_enemy_mode = plan_path_for_goal(
        m,
        start_cell=start_cell,
        goal_cell=goal_cell,
        is_enemy_goal=is_enemy_goal,
        inflate_radius=2,
        min_charge_cells=8,
    )

    if path_cells is None:
        print("[MOVE] No path found, abort this task.")
        with movement_lock:
            movement_busy = False
        return

    print("[MOVE] Path cells:", path_cells)
    print("[MOVE] Used enemy mode:", used_enemy_mode)

    # 2) 셀 -> 월드 좌표 변환
    waypoints_world = []
    for (cx, cy) in path_cells:
        wx, wy = cell_to_world(cx, cy, m.resolution)
        waypoints_world.append((wx, wy))

    print("[MOVE] Waypoints(world):", waypoints_world)

    # 3) 경로 따라가기 (open-loop)
    follow_path_constant_speed(
        waypoints_world,
        is_enemy_goal=is_enemy_goal,
        v_cruise_norm=V_CRUISE,
        v_charge_norm=V_CHARGE,
        omega_turn_norm=OMEGA_TURN,
        scale_v_mps=SCALE_V_MPS,
        scale_omega_rad=SCALE_OMEGA_RAD,
    )

    print("[MOVE] Task finished.")

    with movement_lock:
        movement_busy = False


# ============ 메인 루프 ============

def main():
    global stop_flag, last_auto_time

    print("[MAIN] Loading map ...")
    m = load_map(MAP_FILE)

    if m.start is None:
        raise RuntimeError("Start(S) not found in map")
    current_cell = m.start  # 현재 위치를 맵 상의 셀로 추정 (초기에는 S 위치)

    # 카메라 스레드 시작
    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()

    print("[MAIN] Enter main loop (Ctrl+C to stop)")
    last_status_time = 0.0
    last_auto_time = time.time()

    try:
        while True:
            now = time.time()

            # 1) 서버에서 모드/수동 목표 좌표 가져오기
            mode, manual_target_world = get_mode_and_manual_target_from_server()
            with mode_lock:
                # 서버 응답으로 모드 갱신
                global current_mode
                current_mode = mode

            # 2) 이동 상태 확인
            with movement_lock:
                busy = movement_busy
                goal_cell = current_target_cell

            # 3) 수동 모드: 수동 목표가 있고, 현재 이동 중이 아니라면 즉시 이동 시작
            if mode == "manual" and not busy and manual_target_world is not None:
                c = world_to_cell_safe(m, manual_target_world[0], manual_target_world[1])
                if c is not None:
                    goal_cell = c
                    t = threading.Thread(
                        target=movement_worker,
                        args=(m, current_cell, goal_cell, False),  # 수동 모드는 기본적으로 enemy=False로 가정
                        daemon=True,
                    )
                    t.start()
                    with movement_lock:
                        movement_thread = t
                        movement_busy = True
                        current_target_cell = goal_cell

                    current_cell = goal_cell
                    last_auto_time = now
                else:
                    print("[MAIN] Manual target out of map range, ignore.")

            # 4) 자동 모드: 일정 시간마다 랜덤 목표로 이동 (idle일 때만)
            if mode == "auto" and not busy and (now - last_auto_time) > AUTO_INTERVAL:
                goal = pick_random_reachable_goal_cell(m, current_cell, is_enemy_goal=False)
                if goal is not None:
                    t = threading.Thread(
                        target=movement_worker,
                        args=(m, current_cell, goal, False),
                        daemon=True,
                    )
                    t.start()
                    with movement_lock:
                        movement_thread = t
                        movement_busy = True
                        current_target_cell = goal

                    current_cell = goal
                    last_auto_time = now
                else:
                    print("[MAIN] Failed to pick reachable random goal.")

            # 5) 서버로 상태 전송
            if (now - last_status_time) > STATUS_PERIOD:
                status = {
                    "mode": mode,
                    "moving": busy,
                    "target_cell": list(goal_cell) if goal_cell is not None else None,
                    "timestamp": now,
                }
                send_status_to_server(status)
                last_status_time = now

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("[MAIN] KeyboardInterrupt, stopping...")
    finally:
        stop_flag = True
        time.sleep(0.5)  # 카메라 스레드 정리 시간
        print("[MAIN] Exit.")


if __name__ == "__main__":
    main()
