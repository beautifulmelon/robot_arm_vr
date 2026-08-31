"""Quest WebXR 양방향 서버.

지금까지는 SpesRobotics `teleop` 패키지를 전송 계층으로 썼는데, 로봇 상태를 헤드셋으로
되돌려 보낼 수 없다는 한계가 있었다. 조작자는 헤드셋을 쓰면 Mac 화면을 못 보므로
관절이 한계에 걸렸는지, 워크스페이스를 벗어났는지를 VR 안에서 봐야 한다.

그래서 필요한 부분만 직접 구현한다. 부수적으로 아래 문제들도 함께 사라진다.
  · 패키지 동봉 인증서가 2025-07-28 만료라 모듈 전역 THIS_DIR 을 갈아끼우던 우회
  · move=False 동안 previous_pose 를 갱신하지 않아 grip 을 잡을 때마다 나던
    "Pose jump detected" 로 인한 1프레임 손실
  · 폰 기준 -45° pitch 보정이 컨트롤러에 부적절하게 적용되던 것

라우트
    GET  /            Quest 브라우저용 WebXR 페이지 (web/index.html)
    GET  /dashboard   Mac 브라우저용 상태 대시보드 (web/dashboard.html)
    GET  /robot_view.js  VR/대시보드 공용 3D 렌더링 모듈
    GET  /mesh/{arm,hand}/*  실제 형상 STL
    GET  /vendor/*    three.js 등 정적 자원
    WS   /ws          양방향 — 수신: 컨트롤러 입력 / 송신: 로봇 상태
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from pathlib import Path
from typing import Callable

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

WEB_DIR = Path(__file__).resolve().parents[2] / "web"

logger = logging.getLogger("xr_server")


class XRServer:
    """WebXR 페이지 서빙 + 입력 수신 + 로봇 상태 송신."""

    def __init__(
        self,
        cert_file: Path,
        key_file: Path,
        host: str = "0.0.0.0",
        port: int = 4443,
        web_dir: Path | None = None,
        state_hz: float = 15.0,
        arm_mesh_dir: Path | None = None,
    ):
        self.host = host
        self.port = port
        self.web_dir = web_dir or WEB_DIR
        # /mesh/arm 은 지금 쓰는 URDF 옆의 meshes/ 를 가리켜야 한다. 임시 팔처럼
        # 다른 폴더의 URDF 를 쓰면 assets/robot_arm/meshes 고정으로는 404 가 난다.
        self.arm_mesh_dir = Path(arm_mesh_dir) if arm_mesh_dir else None
        self.cert_file = cert_file
        self.key_file = key_file
        self.state_period = 1.0 / state_hz

        self._app = FastAPI()
        self._on_message: Callable[[dict], None] | None = None
        self._on_command: Callable[[dict], None] | None = None
        self._clients: set[WebSocket] = set()
        self._state: dict = {}
        self._state_lock = threading.Lock()
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._setup_routes()

    # ── 공개 API ──────────────────────────────────────────────────────
    def subscribe(self, callback: Callable[[dict], None]) -> None:
        """컨트롤러 입력 메시지 콜백. WebSocket 스레드에서 호출된다."""
        self._on_message = callback

    def subscribe_command(self, callback: Callable[[dict], None]) -> None:
        """대시보드/HUD 버튼 콜백 (예: 트립 해제). WebSocket 스레드에서 호출된다."""
        self._on_command = callback

    def publish_state(self, state: dict) -> None:
        """로봇 상태를 브라우저로 내보낸다 (최신값만 유지, latest-wins).

        제어 루프에서 매 프레임 불러도 되도록 여기서는 저장만 하고,
        실제 전송은 별도 태스크가 state_hz 로 솎아서 보낸다. 90 Hz 를 그대로
        WebSocket 에 밀어넣으면 헤드셋 브라우저가 렌더링에 쓸 시간을 뺏긴다.
        """
        with self._state_lock:
            self._state = state

    def start(self) -> None:
        if self._thread is not None:
            return
        config = uvicorn.Config(
            self._app,
            host=self.host,
            port=self.port,
            ssl_certfile=str(self.cert_file),
            ssl_keyfile=str(self.key_file),
            log_level="warning",
        )
        self._server = uvicorn.Server(config)
        self._thread = threading.Thread(target=self._server.run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._server = None

    @property
    def client_count(self) -> int:
        return len(self._clients)

    # ── 내부 ──────────────────────────────────────────────────────────
    def _setup_routes(self) -> None:
        app = self._app

        # three.js 등 정적 자원. Quest 브라우저가 여기서 받아간다.
        vendor = self.web_dir / "vendor"
        vendor.mkdir(parents=True, exist_ok=True)
        app.mount("/vendor", StaticFiles(directory=str(vendor)), name="vendor")

        @app.get("/")
        async def index():
            # ★ no-cache: 페이지를 고쳐도 헤드셋이 옛 화면을 계속 띄우는 일이
            #   실제로 있었다. 재검증만 시키므로 비용은 거의 없다.
            return FileResponse(self.web_dir / "index.html",
                                headers={"Cache-Control": "no-cache"})

        @app.get("/dashboard")
        async def dashboard():
            path = self.web_dir / "dashboard.html"
            if not path.exists():
                return JSONResponse({"error": "dashboard.html 없음"}, status_code=404)
            return FileResponse(path)

        # 실제 형상 렌더링용 STL. 대시보드는 기본으로 쓰고, VR 은 옵션이다.
        assets = self.web_dir.parent / "assets"
        dirs = {
            "arm": self.arm_mesh_dir or (assets / "robot_arm/meshes"),
            "hand": assets / "hand/meshes",
        }
        for tag, d in dirs.items():
            if d.is_dir():
                app.mount(f"/mesh/{tag}", StaticFiles(directory=str(d)), name=f"mesh_{tag}")
            else:
                logger.warning("메시 폴더 없음 — /mesh/%s 는 404: %s", tag, d)

        @app.get("/robot_view.js")
        async def robot_view():
            # VR 페이지와 대시보드가 공유하는 렌더링 모듈
            return FileResponse(self.web_dir / "robot_view.js",
                                media_type="application/javascript",
                                headers={"Cache-Control": "no-cache"})

        @app.get("/state")
        async def state():
            with self._state_lock:
                return JSONResponse(self._state)

        @app.websocket("/ws")
        async def ws(websocket: WebSocket):
            await websocket.accept()
            self._clients.add(websocket)
            self._loop = asyncio.get_running_loop()
            logger.info("client connected (%d)", len(self._clients))

            sender = asyncio.create_task(self._state_sender(websocket))
            try:
                while True:
                    raw = await websocket.receive_text()
                    msg = json.loads(raw)
                    if msg.get("type") == "pose" and self._on_message:
                        self._on_message(msg.get("data") or {})
                    elif msg.get("type") == "cmd" and self._on_command:
                        self._on_command(msg.get("data") or {})
                    elif msg.get("type") == "log":
                        logger.info("client: %s", msg.get("data"))
            except WebSocketDisconnect:
                pass
            except Exception as exc:  # 클라이언트가 이상한 걸 보내도 서버는 살아있어야 한다
                logger.warning("ws error: %s", exc)
            finally:
                sender.cancel()
                self._clients.discard(websocket)
                logger.info("client disconnected (%d)", len(self._clients))

    async def _state_sender(self, websocket: WebSocket) -> None:
        """로봇 상태를 state_hz 로 솎아서 보낸다."""
        while True:
            await asyncio.sleep(self.state_period)
            with self._state_lock:
                state = self._state
            if not state:
                continue
            try:
                await websocket.send_text(json.dumps({"type": "state", "data": state}))
            except Exception:
                return


def build_joint_state(cfg, q, q_target=None, clamped: bool = False,
                      near_limit_frac: float = 0.9) -> dict:
    """관절각을 한계 대비 정보와 함께 대시보드용 dict 로 만든다.

    Args:
        cfg: ArmConfig
        q: 현재 관절각 (rad)
        q_target: IK 목표 관절각 (rad). 있으면 지령-실제 차이를 표시
        clamped: 워크스페이스 클램프가 걸렸는지
        near_limit_frac: 가동범위의 이 비율을 넘으면 '한계 근접'으로 표시
    """
    import numpy as np

    q = np.asarray(q, dtype=float)
    lo = np.asarray(cfg.lower, dtype=float)
    hi = np.asarray(cfg.upper, dtype=float)
    span = hi - lo
    # 중앙을 0 으로 두고 -1~+1 로 정규화. 한계 근접을 한눈에 보기 위함.
    # np.where 는 양쪽 분기를 모두 계산하므로 고정 관절(span=0)에서 0 나눗셈이
    # 실제로 일어난다. 분모를 먼저 안전하게 만든 뒤 나눈다.
    center = (hi + lo) / 2.0
    half = np.where(span > 1e-9, span / 2.0, 1.0)
    norm = np.where(span > 1e-9, (q - center) / half, 0.0)

    joints = []
    for i, name in enumerate(cfg.joint_names):
        a = float(abs(norm[i]))
        status = "limit" if a >= 0.995 else ("near" if a >= near_limit_frac else "ok")
        joints.append({
            "name": name,
            "deg": float(np.degrees(q[i])),
            "lower_deg": float(np.degrees(lo[i])),
            "upper_deg": float(np.degrees(hi[i])),
            "norm": float(norm[i]),
            "status": status,
            "target_deg": float(np.degrees(q_target[i])) if q_target is not None else None,
        })
    return {
        "joints": joints,
        "clamped": bool(clamped),
        "n_near": int(sum(1 for j in joints if j["status"] != "ok")),
        "t": time.time(),
    }
