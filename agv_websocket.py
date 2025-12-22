#!/usr/bin/env python3
"""
AGV WebSocket 클라이언트
백엔드 서버(http://localhost:3000)와 실시간 양방향 통신

사용 예시:
    client = AGVWebSocketClient(agv_id="agv-001")
    await client.connect()
    status = StatusPayload(...)
    await client.send_status(status)
"""

import asyncio
import json
import logging
import time
from typing import Optional, Dict, Any, Callable
from dataclasses import dataclass, asdict

# logging 설정
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)
logger = logging.getLogger('AGVWebSocket')

try:
    import websockets
    from websockets.exceptions import ConnectionClosed
except ImportError:
    logger.error("❌ websockets 라이브러리가 없습니다. pip install websockets 실행하세요.")
    raise


@dataclass
class PositionData:
    """위치 데이터"""
    x: float
    y: float
    angle: float
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


@dataclass
class StatusPayload:
    """AGV 상태 페이로드"""
    agv_id: str
    position: PositionData
    mode: str  # "auto" | "manual"
    state: str  # "idle" | "moving" | "charging"
    battery: float  # 0-100
    speed: float  # m/s
    detected_enemies: list = None
    
    def to_dict(self):
        """딕셔너리로 변환"""
        data = {
            'agv_id': self.agv_id,
            'position': {
                'x': self.position.x,
                'y': self.position.y,
                'angle': self.position.angle,
                'timestamp': self.position.timestamp or time.time()
            },
            'mode': self.mode,
            'state': self.state,
            'battery': self.battery,
            'speed': self.speed,
            'detected_enemies': self.detected_enemies or [],
            'timestamp': time.time()
        }
        return data


class AGVWebSocketClient:
    """AGV ↔ Backend 양방향 통신 클라이언트"""
    
    def __init__(
        self,
        agv_id: str = "agv-001",
        server_url: str = "ws://localhost:3000/websocket/agv",
        on_command_received: Optional[Callable] = None,
    ):
        self.agv_id = agv_id
        self.server_url = server_url
        self.ws = None
        self.connected = False
        self.on_command_received = on_command_received or self._default_handler
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2  # seconds
        self.receive_task = None
        logger.info(f"[Init] AGV ID: {self.agv_id}")
        
    async def connect(self):
        """서버에 연결"""
        try:
            logger.info(f"[Connect] Connecting to {self.server_url}...")
            self.ws = await asyncio.wait_for(
                websockets.connect(self.server_url),
                timeout=5.0
            )
            self.connected = True
            self.reconnect_attempts = 0
            logger.info(f"[Connect] ✅ Connected!")
            
            # 수신 태스크 시작
            self.receive_task = asyncio.create_task(self.receive_loop())
            
        except asyncio.TimeoutError:
            logger.error("[Connect] ❌ Connection timeout")
            self.connected = False
            await self._handle_reconnect()
        except Exception as e:
            logger.error(f"[Connect] ❌ Connection failed: {e}")
            self.connected = False
            await self._handle_reconnect()
    
    async def send_status(self, payload: StatusPayload) -> bool:
        """상태 메시지 전송"""
        if not self.connected or self.ws is None:
            logger.warning("[Status] ⚠️  Not connected, cannot send status")
            return False
        
        try:
            message = {
                "type": "status",
                "agent_id": self.agv_id,
                "data": payload.to_dict(),
                "timestamp": int(time.time() * 1000)
            }
            await asyncio.wait_for(
                self.ws.send(json.dumps(message)),
                timeout=2.0
            )
            return True
        except asyncio.TimeoutError:
            logger.error("[Status] ❌ Send timeout")
            self.connected = False
            return False
        except Exception as e:
            logger.error(f"[Status] ❌ Send error: {e}")
            self.connected = False
            return False
    
    async def receive_loop(self):
        """수신 루프 (명령 대기)"""
        try:
            async for message in self.ws:
                try:
                    data = json.loads(message)
                    if data.get("type") == "command":
                        logger.info(f"[Command] Received: {data.get('data', {}).get('type')}")
                        await self._handle_command(data.get("data"))
                    else:
                        logger.debug(f"[Message] Unknown type: {data.get('type')}")
                except json.JSONDecodeError:
                    logger.error(f"[Message] Invalid JSON: {message}")
        except ConnectionClosed:
            logger.warning("[Receive] Connection closed by server")
            self.connected = False
            await self._handle_reconnect()
        except asyncio.CancelledError:
            logger.info("[Receive] Task cancelled")
        except Exception as e:
            logger.error(f"[Receive] Error: {e}")
            self.connected = False
            await self._handle_reconnect()
    
    async def _handle_command(self, command_data: Dict[str, Any]):
        """명령 처리"""
        try:
            if asyncio.iscoroutinefunction(self.on_command_received):
                await self.on_command_received(command_data)
            else:
                self.on_command_received(command_data)
        except Exception as e:
            logger.error(f"[Command] Handler error: {e}")
    
    async def _handle_reconnect(self):
        """자동 재연결"""
        if self.reconnect_attempts < self.max_reconnect_attempts:
            self.reconnect_attempts += 1
            delay = self.reconnect_delay * self.reconnect_attempts
            logger.info(
                f"[Reconnect] Retrying in {delay}s "
                f"({self.reconnect_attempts}/{self.max_reconnect_attempts})..."
            )
            await asyncio.sleep(delay)
            await self.connect()
        else:
            logger.error(
                f"[Reconnect] ❌ Max reconnection attempts ({self.max_reconnect_attempts}) reached"
            )
    
    async def disconnect(self):
        """연결 종료"""
        if self.receive_task:
            self.receive_task.cancel()
            try:
                await self.receive_task
            except asyncio.CancelledError:
                pass
        
        if self.ws:
            await self.ws.close()
            self.connected = False
            logger.info("[Disconnect] Closed")
    
    def is_connected(self) -> bool:
        """연결 상태 확인"""
        return self.connected
    
    @staticmethod
    def _default_handler(command: Dict[str, Any]):
        """기본 명령 핸들러"""
        logger.info(f"[Default] Unhandled command: {command.get('type')}")


# 테스트용 코드
if __name__ == "__main__":
    async def test_client():
        """WebSocket 클라이언트 테스트"""
        
        async def handle_command(cmd):
            logger.info(f"[Test] Command handler called with: {cmd}")
        
        client = AGVWebSocketClient(
            agv_id="agv-001",
            on_command_received=handle_command
        )
        
        try:
            # 서버 연결 시도 (서버가 없으면 재연결 시도)
            await asyncio.wait_for(client.connect(), timeout=3.0)
            
            if client.is_connected():
                # 상태 메시지 전송
                status = StatusPayload(
                    agv_id="agv-001",
                    position=PositionData(x=10.5, y=15.2, angle=1.57),
                    mode="auto",
                    state="moving",
                    battery=85.0,
                    speed=0.35
                )
                result = await client.send_status(status)
                logger.info(f"[Test] Status send result: {result}")
                
                # 수신 대기 (10초)
                await asyncio.sleep(10)
            else:
                logger.warning("[Test] Could not connect to server")
        
        except asyncio.TimeoutError:
            logger.warning("[Test] Connection attempt timed out")
        except Exception as e:
            logger.error(f"[Test] Error: {e}")
        finally:
            await client.disconnect()
    
    # 실행
    try:
        asyncio.run(test_client())
    except KeyboardInterrupt:
        logger.info("[Test] Interrupted by user")
