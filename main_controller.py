#!/usr/bin/env python3
"""
AGV 메인 컨트롤러 - asyncio 기반

WebSocket을 통해 백엔드 서버와 실시간 통신하는 AGV 제어 시스템
- 경로 계획 (A* 알고리즘)
- 이동 제어
- 카메라 피드 처리
- 실시간 서버 통신
"""

import asyncio
import time
import threading
import random
import logging
from typing import Optional, Tuple, Dict, Any

import cv2

from map_loader import load_map, cell_to_world, world_to_cell
from plan_test import plan_path_for_goal
from follow_path import follow_path_constant_speed
from agv_websocket import AGVWebSocketClient, StatusPayload, PositionData

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)
logger = logging.getLogger('MAIN')


# =========================
# CONFIG
# =========================

MAP_FILE = "map_smallroom_60x60.json"

# AGV 식별자
AGV_ID = "agv-001"

# 자동 모드에서 다음 랜덤 목표를 생성하는 최소 간격(초)
AUTO_INTERVAL_SEC = 6.0

# 상태 전송 주기(초)
STATUS_PERIOD_SEC = 0.5

# 카메라 캡처 프레임 주기(초)
CAM_PERIOD_SEC = 0.03

# follow_path 속도 파라미터
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

stop_event = asyncio.Event()

state_lock = asyncio.Lock()
shared_state: Dict[str, Any] = {
    "mode": "auto",                 # "auto" | "manual"
    "goal_cell": None,              # (x,y) or None
    "is_enemy_goal": False,         # bool
    "pose_est_world": (0.0, 0.0, 0.0),  # (x,y,theta) 초기값
    "last_cmd_ts": 0.0,             # 최근 목표 수신 시각
    "last_plan_ok": False,          # 마지막 플랜 성공 여부
    "last_error": "",               # 마지막 에러 문자열
    "current_pos_world": (0.0, 0.0, 0.0),  # 현재 월드 좌표
    "current_battery": 100.0,       # 배터리 %
    "speed": 0.0,                  # m/s
    "moving": False,                # 이동 중 여부
}


# =========================
# WebSocket 명령 핸들러
# =========================

async def handle_websocket_command(cmd: Dict[str, Any]):
    """
WebSocket으로부터 받은 명령 처리

지원 명령:
- set_goal: 목표 위치 설정
- set_mode: 모드 변경 (auto/manual)
- stop: 이동 중지
"""
    global m  # 맵 객체 전역 참조
    
    try:
        cmd_type = cmd.get("type")
        logger.info(f"[Command] Received: {cmd_type}")
        
        if cmd_type == "set_goal":
            # 목표 위치 설정
            target_pos = cmd.get("target_pos")
            if target_pos:
                wx, wy = target_pos.get("x"), target_pos.get("y")
                # 월드 좌표 → 셀 좌표 변환
                cx, cy = world_to_cell(wx, wy, m.resolution)
                
                # 범위 확인
                if 0 <= cx < m.width and 0 <= cy < m.height:
                    async with state_lock:
                        shared_state["goal_cell"] = (cx, cy)
                        shared_state["is_enemy_goal"] = cmd.get("is_enemy_goal", False)
                        shared_state["last_cmd_ts"] = time.time()
                    logger.info(f"[Command] Goal set: ({cx}, {cy})")
                else:
                    logger.warning(f"[Command] Goal out of range: ({cx}, {cy})")
        
        elif cmd_type == "set_mode":
            mode = cmd.get("mode")
            if mode in ("auto", "manual"):
                async with state_lock:
                    shared_state["mode"] = mode
                logger.info(f"[Command] Mode set to: {mode}")
        
        elif cmd_type == "stop":
            async with state_lock:
                shared_state["goal_cell"] = None
            logger.info("[Command] Stop received")
        
    except Exception as e:
        logger.error(f"[Command] Error: {e}")


# =========================
# 상태 전송
# =========================

async def send_status_to_server(ws_client: AGVWebSocketClient):
    """
주기적으로 서버에 상태 전송

STATUS_PERIOD_SEC마다 AGV 상태를 서버로 전송
"""
    logger.info("[StatusTask] Started")
    
    while not stop_event.is_set():
        try:
            async with state_lock:
                pos = shared_state.get("pose_est_world", (0, 0, 0))
                
                payload = StatusPayload(
                    agv_id=AGV_ID,
                    position=PositionData(
                        x=pos[0],
                        y=pos[1],
                        angle=pos[2],
                        timestamp=time.time()
                    ),
                    mode=shared_state["mode"],
                    state="moving" if shared_state["moving"] else "idle",
                    battery=shared_state["current_battery"],
                    speed=shared_state["speed"],
                    detected_enemies=[]
                )
            
            if ws_client.is_connected():
                result = await ws_client.send_status(payload)
                if not result:
                    logger.warning("[StatusTask] Failed to send status")
            
            await asyncio.sleep(STATUS_PERIOD_SEC)
        
        except Exception as e:
            logger.error(f"[StatusTask] Error: {e}")
            await asyncio.sleep(STATUS_PERIOD_SEC)
    
    logger.info("[StatusTask] Stopped")


# =========================
# 카메라 스레드
# =========================

def camera_thread_main():
    """
카메라 피드 처리 (스레드)

OpenCV로 카메라를 열고 프레임을 읽음
실패 시 자동으로 재시도
"""
    logger.info("[Camera] Thread started")
    cap = None

    while not stop_event.is_set():
        if cap is None:
            try:
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    logger.warning("[Camera] Open failed, retrying in 1s...")
                    cap.release()
                    cap = None
                    time.sleep(1.0)
                    continue
                logger.info("[Camera] Opened")
            except Exception as e:
                logger.error(f"[Camera] Error opening camera: {e}")
                time.sleep(1.0)
                continue

        try:
            ret, frame = cap.read()
            if not ret:
                logger.warning("[Camera] Read failed, reopening in 0.5s...")
                cap.release()
                cap = None
                time.sleep(0.5)
                continue

            # TODO: 여기서 적 탐지/스트리밍 등을 수행
            # enemy_found = detect_enemy(frame)
            # if enemy_found:
            #     asyncio.run_coroutine_threadsafe(
            #         update_enemy_goal(),
            #         loop
            #     )

            time.sleep(CAM_PERIOD_SEC)
        
        except Exception as e:
            logger.error(f"[Camera] Error: {e}")
            time.sleep(0.5)

    if cap is not None:
        cap.release()

    logger.info("[Camera] Thread stopped")


# =========================
# 경로 계획 헬퍼
# =========================

def clamp_goal_cell(m, cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    """목표 셀이 맵 범위 내인지 확인"""
    x, y = cell
    if 0 <= x < m.width and 0 <= y < m.height:
        return (x, y)
    return None


def pick_random_goal_cell(m, start_cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
    """
자동모드 랜덤 목표 선택

60회 시도하여 경로 계획이 가능한 목표를 선택
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


# =========================
# 이동 워커
# =========================

def move_worker_sync(m, start_cell: Tuple[int, int], goal_cell: Tuple[int, int], is_enemy_goal: bool):
    """
경로 계획 및 이동 실행 (동기 함수, asyncio.to_thread에서 실행)
"""
    try:
        asyncio.run_coroutine_threadsafe(
            _set_moving(True),
            asyncio.get_event_loop()
        )
        
        path_cells, used_enemy_mode = plan_path_for_goal(
            m,
            start_cell=start_cell,
            goal_cell=goal_cell,
            is_enemy_goal=is_enemy_goal,
            inflate_radius=INFLATE_RADIUS,
            min_charge_cells=MIN_CHARGE_CELLS,
        )

        if path_cells is None:
            asyncio.run_coroutine_threadsafe(
                _set_plan_failed("No path found"),
                asyncio.get_event_loop()
            )
            logger.warning("[Move] No path found")
            return

        waypoints_world = [cell_to_world(x, y, m.resolution) for (x, y) in path_cells]
        logger.info(f"[Move] Planning success: {len(waypoints_world)} waypoints")

        follow_path_constant_speed(
            waypoints_world=waypoints_world,
            is_enemy_goal=is_enemy_goal,
            v_cruise_norm=V_CRUISE,
            v_charge_norm=V_CHARGE,
            omega_turn_norm=OMEGA_TURN,
            scale_v_mps=SCALE_V_MPS,
            scale_omega_rad=SCALE_OMEGA_RAD,
        )
        
        logger.info("[Move] Movement complete")

    except Exception as e:
        logger.error(f"[Move] Exception: {e}")
        asyncio.run_coroutine_threadsafe(
            _set_plan_failed(f"Exception: {e}"),
            asyncio.get_event_loop()
        )
    finally:
        asyncio.run_coroutine_threadsafe(
            _set_moving(False),
            asyncio.get_event_loop()
        )


async def _set_moving(value: bool):
    """이동 상태 설정 (스레드 안전)"""
    async with state_lock:
        shared_state["moving"] = value


async def _set_plan_failed(error_msg: str):
    """계획 실패 설정"""
    async with state_lock:
        shared_state["last_plan_ok"] = False
        shared_state["last_error"] = error_msg


# =========================
# 메인 비동기 루프
# =========================

async def main_loop(m, start_cell, ws_client: AGVWebSocketClient):
    """
비동기 메인 루프

주요 작업:
1. WebSocket 통신
2. 자동/수동 모드 처리
3. 경로 계획 및 이동
4. 상태 전송
"""
    logger.info("[MainLoop] Started")
    
    current_cell = start_cell
    last_auto = time.time()
    move_task = None
    
    try:
        while not stop_event.is_set():
            now = time.time()
            
            # 이동 중 여부 확인
            if move_task:
                moving = not move_task.done()
            else:
                moving = False
            
            async with state_lock:
                shared_state["moving"] = moving
                mode = shared_state["mode"]
                goal_cell = shared_state["goal_cell"]
                is_enemy_goal = shared_state["is_enemy_goal"]
            
            # 자동 모드: 랜덤 목표 생성
            if mode == "auto" and not moving and (now - last_auto >= AUTO_INTERVAL_SEC):
                goal = pick_random_goal_cell(m, current_cell)
                if goal:
                    async with state_lock:
                        shared_state["goal_cell"] = goal
                    goal_cell = goal
                    last_auto = now
                    logger.debug(f"[MainLoop] Auto goal generated: {goal}")
            
            # 이동 시작
            if not moving and goal_cell:
                logger.info(f"[MainLoop] Starting move from {current_cell} to {goal_cell}")
                move_task = asyncio.create_task(
                    asyncio.to_thread(
                        move_worker_sync, m, current_cell, goal_cell, is_enemy_goal
                    )
                )
                current_cell = goal_cell
                async with state_lock:
                    shared_state["goal_cell"] = None
            
            await asyncio.sleep(0.01)
    
    except Exception as e:
        logger.error(f"[MainLoop] Error: {e}")
    finally:
        logger.info("[MainLoop] Stopped")


# =========================
# 메인 함수
# =========================

async def main():
    """
AGV 컨트롤러 메인 함수

초기화:
1. 맵 로드
2. WebSocket 연결
3. 카메라 스레드 시작
4. 상태 전송 태스크 시작
5. 메인 루프 실행
"""
    global m  # 맵 객체 전역 저장
    
    logger.info("=" * 50)
    logger.info("AGV Controller (asyncio) Starting...")
    logger.info("=" * 50)
    
    try:
        # 맵 로드
        logger.info(f"[Init] Loading map: {MAP_FILE}")
        m = load_map(MAP_FILE)
        
        if m.start is None:
            raise RuntimeError("Start(S) not found in map")
        
        logger.info(f"[Init] Map loaded: {m.width}x{m.height}, Start: {m.start}")
        
        # WebSocket 클라이언트 생성
        ws_client = AGVWebSocketClient(
            agv_id=AGV_ID,
            server_url="ws://localhost:3000/websocket/agv",
            on_command_received=handle_websocket_command
        )
        
        # WebSocket 연결 시도 (백그라운드 재연결)
        logger.info("[Init] Starting WebSocket connection...")
        ws_connect_task = asyncio.create_task(ws_client.connect())
        
        # 카메라 스레드 시작 (실패해도 계속 진행)
        logger.info("[Init] Starting camera thread...")
        cam_thread = threading.Thread(target=camera_thread_main, daemon=True)
        cam_thread.start()
        
        # 상태 전송 태스크 시작
        logger.info("[Init] Starting status sender...")
        status_task = asyncio.create_task(send_status_to_server(ws_client))
        
        # 메인 루프 시작
        logger.info("[Init] Starting main loop...")
        loop_task = asyncio.create_task(main_loop(m, m.start, ws_client))
        
        logger.info("[Init] ✅ All systems ready")
        logger.info("[Init] Press Ctrl+C to stop")
        
        # 모든 태스크가 완료될 때까지 대기
        await asyncio.gather(
            ws_connect_task,
            status_task,
            loop_task,
            return_exceptions=True
        )
    
    except KeyboardInterrupt:
        logger.info("\n[Main] KeyboardInterrupt - Shutting down...")
    except Exception as e:
        logger.error(f"[Main] Fatal error: {e}", exc_info=True)
    finally:
        logger.info("[Main] Cleanup...")
        stop_event.set()
        await asyncio.sleep(0.2)
        logger.info("[Main] Exit")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
