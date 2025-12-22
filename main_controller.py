#!/usr/bin/env python3
import time
import threading
import random
from typing import Optional, Tuple, Dict, Any

import cv2

from map_loader import load_map, cell_to_world, world_to_cell
from plan_test import plan_path_for_goal
from follow_path import follow_path_constant_speed


# =========================
# CONFIG
# =========================

MAP_FILE = "map_smallroom_60x60.json"

# 자동 모드에서 다음 랜덤 목표를 생성하는 최소 간격(초)
AUTO_INTERVAL_SEC = 6.0

# 상태 전송 주기(초)
STATUS_PERIOD_SEC = 0.5

# 서버 폴링 주기(초) - 서버에서 모드/목표를 가져오기
SERVER_POLL_SEC = 0.2

# 카메라 캡처 프레임 주기(초)
CAM_PERIOD_SEC = 0.03

# follow_path 속도 파라미터 (이미 캘리브레이션 해둔 값 사용)
V_CRUISE = 0.35
V_CHARGE = 0.8
OMEGA_TURN = 0.5
SCALE_V_MPS = 0.15
SCALE_OMEGA_RAD = 0.85

# 플래너 파라미터
INFLATE_RADIUS = 2
MIN_CHARGE_CELLS = 8


# =========================
# Shared State (thread-safe)
# =========================

stop_event = threading.Event()

state_lock = threading.Lock()
shared_state: Dict[str, Any] = {
    "mode": "auto",                 # "auto" | "manual"
    "goal_cell": None,              # (x,y) or None
    "is_enemy_goal": False,         # bool
    "pose_est_world": None,         # (x,y,theta) or None (TODO)
    "last_cmd_ts": 0.0,             # 최근 목표 수신 시각
    "last_plan_ok": False,          # 마지막 플랜 성공 여부
    "last_error": "",               # 마지막 에러 문자열
}


# =========================
# Server I/O (TODO stub)
# =========================

def server_poll_command() -> Optional[Dict[str, Any]]:
    """
    서버에서 최신 명령을 가져온다 (폴링).
    반환 예시(권장):
      {
        "mode": "manual" or "auto",
        "goal": {"type": "cell", "x": 30, "y": 20}  # or type:"world"
        "is_enemy_goal": true/false
      }

    TODO:
      - requests.get(...) 또는 websocket으로 대체
      - 실패 시 None 반환
    """
    # TODO: 실제 서버 통신 구현
    return None


def server_send_status(payload: Dict[str, Any]) -> None:
    """
    서버로 상태를 전송한다.
    payload 예시:
      {
        "mode": "auto",
        "moving": true,
        "target_cell": [x,y],
        "pose_world": [x,y,theta],
        "ts": 123.45
      }

    TODO:
      - requests.post(...) 또는 websocket send
    """
    # TODO: 실제 서버 전송 구현
    # 지금은 디버그 출력만
    print("[STATUS]", payload)


# =========================
# Camera thread
# =========================

def camera_thread_main():
    """
    카메라는 항상 켜두되,
    OpenCV/GStreamer가 실패하더라도 메인이 죽지 않도록 재시도한다.
    """
    print("[CAM] thread start")
    cap = None

    while not stop_event.is_set():
        if cap is None:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("[CAM] open failed. retry in 1s")
                cap.release()
                cap = None
                time.sleep(1.0)
                continue
            print("[CAM] opened")

        ret, frame = cap.read()
        if not ret:
            # 카메라 스트림이 깨졌을 때 재오픈
            print("[CAM] read failed. reopen in 0.5s")
            cap.release()
            cap = None
            time.sleep(0.5)
            continue

        # TODO: 여기서 적 탐지/스트리밍/프레임 압축 등을 수행
        # 예: enemy_found = detect_enemy(frame)
        # if enemy_found: shared_state["is_enemy_goal"]=True 같은 인터럽트 로직 추가 가능

        time.sleep(CAM_PERIOD_SEC)

    if cap is not None:
        cap.release()

    print("[CAM] thread stop")


# =========================
# Planning helpers
# =========================

def clamp_goal_cell(m, cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    x, y = cell
    if 0 <= x < m.width and 0 <= y < m.height:
        return (x, y)
    return None


def pick_random_goal_cell(m, start_cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    """
    자동모드 랜덤 목표 선택:
    - 장애물/비가용은 plan_path_for_goal로 걸러낸다
    """
    for _ in range(60):
        gx = random.randint(0, m.width - 1)
        gy = random.randint(0, m.height - 1)
        goal = (gx, gy)
        if goal == start_cell:
            continue

        path, _ = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal,
            is_enemy_goal=False,
            inflate_radius=INFLATE_RADIUS,
            min_charge_cells=MIN_CHARGE_CELLS,
        )
        if path is not None:
            return goal
    return None


def goal_from_command(m, cmd: Dict[str, Any]) -> Tuple[Optional[Tuple[int, int]], bool, str]:
    """
    서버 명령 dict를 받아 goal_cell, is_enemy_goal을 결정한다.
    아직 서버 프로토콜이 미정이므로 최대한 유연하게 처리.
    """
    try:
        mode = cmd.get("mode", None)
        if mode not in ("auto", "manual"):
            mode = None

        is_enemy = bool(cmd.get("is_enemy_goal", False))
        goal = cmd.get("goal", None)
        if goal is None:
            return None, is_enemy, "no goal in command"

        gtype = goal.get("type", "cell")

        if gtype == "cell":
            gx = int(goal["x"])
            gy = int(goal["y"])
            c = clamp_goal_cell(m, (gx, gy))
            if c is None:
                return None, is_enemy, "goal cell out of range"
            return c, is_enemy, ""

        if gtype == "world":
            wx = float(goal["x"])
            wy = float(goal["y"])
            cx, cy = world_to_cell(wx, wy, m.resolution)
            c = clamp_goal_cell(m, (cx, cy))
            if c is None:
                return None, is_enemy, "goal world->cell out of range"
            return c, is_enemy, ""

        return None, is_enemy, f"unknown goal type: {gtype}"

    except Exception as e:
        return None, False, f"parse command error: {e}"


# =========================
# Movement worker
# =========================

def move_worker(m, start_cell: Tuple[int, int], goal_cell: Tuple[int, int], is_enemy_goal: bool):
    """
    플랜 -> 웨이포인트 변환 -> follow_path 실행
    """
    try:
        with state_lock:
            shared_state["last_error"] = ""
            shared_state["last_plan_ok"] = False

        path_cells, used_enemy_mode = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal_cell,
            is_enemy_goal=is_enemy_goal,
            inflate_radius=INFLATE_RADIUS,
            min_charge_cells=MIN_CHARGE_CELLS,
        )

        if path_cells is None:
            with state_lock:
                shared_state["last_plan_ok"] = False
                shared_state["last_error"] = "No path found"
            print("[MOVE] No path found")
            return

        waypoints_world = [cell_to_world(x, y, m.resolution) for (x, y) in path_cells]
        with state_lock:
            shared_state["last_plan_ok"] = True

        print(f"[MOVE] start={start_cell} goal={goal_cell} enemy={is_enemy_goal} used_enemy_mode={used_enemy_mode}")
        print(f"[MOVE] waypoints={waypoints_world}")

        follow_path_constant_speed(
            waypoints_world=waypoints_world,
            is_enemy_goal=is_enemy_goal,
            v_cruise_norm=V_CRUISE,
            v_charge_norm=V_CHARGE,
            omega_turn_norm=OMEGA_TURN,
            scale_v_mps=SCALE_V_MPS,
            scale_omega_rad=SCALE_OMEGA_RAD,
        )

    except Exception as e:
        with state_lock:
            shared_state["last_plan_ok"] = False
            shared_state["last_error"] = f"move_worker exception: {e}"
        print("[MOVE] Exception:", e)


# =========================
# MAIN LOOP
# =========================

def main():
    print("[MAIN] Loading map ...")
    m = load_map(MAP_FILE)

    if m.start is None:
        raise RuntimeError("Start(S) not found in map")

    # "현재 위치"는 지금은 open-loop라서 완벽하진 않음.
    # 여기서는 “마지막으로 목표로 삼았던 셀”을 현재 셀로 간주하는 단순 모델로 시작.
    # TODO: 나중에 pose_est를 붙이면 start_cell을 추정 pose 기반으로 바꾸기.
    current_cell = m.start

    # 카메라 스레드 시작 (실패해도 main 유지)
    cam_th = threading.Thread(target=camera_thread_main, daemon=True)
    cam_th.start()

    move_th: Optional[threading.Thread] = None
    last_auto = time.time()
    last_poll = 0.0
    last_status = 0.0

    print("[MAIN] Enter loop (Ctrl+C to stop)")

    try:
        while True:
            now = time.time()

            # (A) 서버 폴링
            if now - last_poll >= SERVER_POLL_SEC:
                cmd = server_poll_command()
                last_poll = now

                if cmd is not None:
                    goal_cell, is_enemy, err = goal_from_command(m, cmd)

                    with state_lock:
                        if "mode" in cmd and cmd["mode"] in ("auto", "manual"):
                            shared_state["mode"] = cmd["mode"]
                        shared_state["is_enemy_goal"] = is_enemy
                        shared_state["last_cmd_ts"] = now
                        if err:
                            shared_state["last_error"] = err

                    if goal_cell is not None:
                        with state_lock:
                            shared_state["goal_cell"] = goal_cell

            # (B) 이동 중 여부 판단 (전역 movement_busy 같은 거 안 씀!)
            moving = (move_th is not None) and move_th.is_alive()

            # (C) 모드에 따라 목표 결정
            with state_lock:
                mode = shared_state["mode"]
                goal_cell = shared_state["goal_cell"]
                is_enemy_goal = shared_state["is_enemy_goal"]

            # 자동 모드: 일정 주기마다 랜덤 목표를 세팅(이동 중이면 세팅만 미룸)
            if mode == "auto" and (not moving) and (now - last_auto >= AUTO_INTERVAL_SEC):
                goal = pick_random_goal_cell(m, current_cell)
                if goal is not None:
                    with state_lock:
                        shared_state["goal_cell"] = goal
                        shared_state["is_enemy_goal"] = False
                    goal_cell = goal
                    is_enemy_goal = False
                    last_auto = now

            # (D) 이동 시작 조건:
            # - 이동 중이 아니고
            # - 목표가 존재하면
            if (not moving) and (goal_cell is not None):
                move_th = threading.Thread(
                    target=move_worker,
                    args=(m, current_cell, goal_cell, is_enemy_goal),
                    daemon=True,
                )
                move_th.start()

                # open-loop 단순 모델: 목표를 곧 “현재 셀”로 갱신
                current_cell = goal_cell

                # 목표는 한 번 소비하면 지움 (같은 명령 반복 실행 방지)
                with state_lock:
                    shared_state["goal_cell"] = None

            # (E) 상태 전송
            if now - last_status >= STATUS_PERIOD_SEC:
                with state_lock:
                    payload = {
                        "ts": now,
                        "mode": shared_state["mode"],
                        "moving": moving,
                        "last_plan_ok": shared_state["last_plan_ok"],
                        "last_error": shared_state["last_error"],
                        "current_cell": list(current_cell),
                        # TODO: pose 추정 붙이면 pose_world도 포함
                        # "pose_world": [x, y, theta],
                    }
                server_send_status(payload)
                last_status = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("[MAIN] KeyboardInterrupt -> stopping")
    finally:
        stop_event.set()
        time.sleep(0.2)
        print("[MAIN] Exit")


if __name__ == "__main__":
    main()
