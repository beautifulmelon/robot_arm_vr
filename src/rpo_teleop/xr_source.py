"""Quest 2 WebXR 입력 소스.

전송 계층은 [xr_server.XRServer](xr_server.py) 를 쓴다. 프론트엔드는
[web/index.html](../../web/index.html) 이며 양손 컨트롤러·손 트래킹 전체를 받는다.

초기에는 SpesRobotics `teleop` 패키지를 썼으나 자체 서버로 교체했다. 이유:
  · 로봇 상태를 헤드셋으로 되돌려 보낼 수 없었다. 조작자는 헤드셋을 쓰면 Mac 화면을
    못 보므로 관절 한계·워크스페이스 상태를 VR 안에서 봐야 한다.
  · 동봉 인증서가 2025-07-28 만료라 모듈 전역 THIS_DIR 을 갈아끼우는 우회가 필요했다.
  · move=False 동안 previous_pose 를 갱신하지 않아 grip 을 잡을 때마다
    "Pose jump detected" 로 1프레임을 버렸다 (실측 확인).
  · 폰 기준 -45° pitch 보정이 컨트롤러에 부적절했다.

측정 근거는 docs/01_quest_mapping.md 참고.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .certs import ensure_cert, get_local_ip
from .xr_server import XRServer

WEB_DIR = Path(__file__).resolve().parents[2] / "web"


@dataclass(frozen=True)
class ControllerState:
    """한쪽 Touch 컨트롤러의 상태. 좌표는 WebXR local-floor(RUB) 기준."""

    position: np.ndarray  # (3,) m
    quat: np.ndarray  # (4,) xyzw
    trigger: float  # 0.0~1.0 검지
    grip: float  # 0.0~1.0 중지
    primary: bool  # 오른손 A / 왼손 X
    secondary: bool  # 오른손 B / 왼손 Y
    stick: np.ndarray  # (2,) -1~1
    stick_press: bool

    @property
    def rotation_matrix(self) -> np.ndarray:
        x, y, z, w = self.quat
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])


@dataclass(frozen=True)
class QuestFrame:
    """한 XR 프레임에서 받은 전체 입력."""

    browser_time: float  # 브라우저 XR 타임스탬프 (초)
    recv_time: float  # 서버 수신 시각 (time.time())
    right: ControllerState | None
    left: ControllerState | None
    hands: dict = field(default_factory=dict)  # {"right": {joint: {p,q,r}}, ...}
    raw: dict = field(default_factory=dict)

    def pinch_distance(self, side: str = "right") -> float:
        """엄지-검지 끝 거리 (m). 손 트래킹이 없으면 inf."""
        hand = self.hands.get(side)
        if not hand:
            return float("inf")
        a, b = hand.get("thumb-tip"), hand.get("index-finger-tip")
        if not a or not b:
            return float("inf")
        return float(np.linalg.norm(np.array(a["p"]) - np.array(b["p"])))


def _parse_controller(entry: dict | None, use_grip_space: bool = True) -> ControllerState | None:
    """페이로드의 컨트롤러 엔트리 → ControllerState.

    use_grip_space=True 이면 gripSpace 를 쓴다. targetRaySpace 는 컨트롤러에서
    앞아래로 정확히 48.15° 기울어진 포인팅 레이라(실측 std 0.00°) 로봇 EE 자세로는
    부자연스럽다. docs/01_quest_mapping.md §5 참고.
    """
    if not entry:
        return None
    pose = entry.get("grip") if use_grip_space else entry.get("targetRay")
    pose = pose or entry.get("targetRay") or entry.get("grip")
    gp = entry.get("gamepad")
    if not pose or not gp:
        return None
    p, o = pose["position"], pose["orientation"]
    stick = gp.get("stick") or [0.0, 0.0]
    return ControllerState(
        position=np.array([p["x"], p["y"], p["z"]], dtype=float),
        quat=np.array([o["x"], o["y"], o["z"], o["w"]], dtype=float),
        trigger=float(gp.get("trigger", 0.0)),
        grip=float(gp.get("grip", 0.0)),
        primary=bool(gp.get("primary", False)),
        secondary=bool(gp.get("secondary", False)),
        stick=np.array(stick, dtype=float),
        stick_press=bool(gp.get("stickPress", False)),
    )


class QuestXRSource:
    """Quest 2 WebXR 입력 서버. 백그라운드 스레드에서 돌고 최신 프레임을 노출한다."""

    def __init__(
        self,
        port: int = 4443,
        ip: str | None = None,
        use_grip_space: bool = True,
        web_dir: Path | None = None,
        arm_mesh_dir: Path | None = None,
    ):
        self.port = port
        self.ip = ip or get_local_ip()
        self.use_grip_space = use_grip_space
        self.web_dir = web_dir or WEB_DIR
        self.arm_mesh_dir = arm_mesh_dir   # 지금 쓰는 URDF 옆의 meshes/

        self._server: XRServer | None = None
        self._on_command = None
        self._lock = threading.Lock()
        self._latest: QuestFrame | None = None
        self._count = 0
        self._first_recv: float | None = None

    def subscribe_command(self, callback) -> None:
        """대시보드 버튼 콜백. start() 전에 등록해야 한다."""
        self._on_command = callback
        if self._server is not None:
            self._server.subscribe_command(callback)

    # ── 수명주기 ──────────────────────────────────────────────────────
    @property
    def url(self) -> str:
        return f"https://{self.ip}:{self.port}"

    def start(self) -> None:
        if self._server is not None:
            return
        cert_file, key_file = ensure_cert(self.ip)
        self._server = XRServer(cert_file, key_file, port=self.port, web_dir=self.web_dir,
                                arm_mesh_dir=self.arm_mesh_dir)
        self._server.subscribe(self._on_message)
        if self._on_command is not None:
            self._server.subscribe_command(self._on_command)
        self._server.start()

    def stop(self) -> None:
        if self._server is not None:
            self._server.stop()
        self._server = None

    def publish_state(self, state: dict) -> None:
        """로봇 상태를 브라우저(Quest HUD / 대시보드)로 내보낸다."""
        if self._server is not None:
            self._server.publish_state(state)

    def _on_message(self, message: dict) -> None:
        if "right" not in message and "left" not in message:
            return  # 예상 밖의 페이로드 (구 버전 페이지가 붙은 경우 등)
        now = time.time()
        frame = QuestFrame(
            browser_time=float(message.get("t", 0.0)),
            recv_time=now,
            right=_parse_controller(message.get("right"), self.use_grip_space),
            left=_parse_controller(message.get("left"), self.use_grip_space),
            hands=message.get("hands") or {},
            raw=message,
        )
        with self._lock:
            self._latest = frame
            self._count += 1
            if self._first_recv is None:
                self._first_recv = now

    # ── 조회 ──────────────────────────────────────────────────────────
    @property
    def latest(self) -> QuestFrame | None:
        with self._lock:
            return self._latest

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._count

    @property
    def is_connected(self) -> bool:
        """최근 1초 안에 프레임을 받았는가."""
        frame = self.latest
        return frame is not None and (time.time() - frame.recv_time) < 1.0

    def wait_for_connection(self, timeout: float = 300.0, poll: float = 0.1) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_connected:
                return True
            time.sleep(poll)
        return False
