"""모터 백엔드 추상화 + 안전 계층.

IK 가 뽑아낸 관절각을 실제 모터로 내보내는 마지막 구간. 여기서 틀리면 로봇이
부서지거나 사람이 다치므로, 명령을 그대로 흘리지 않고 반드시 SafetyLayer 를 통과시킨다.

    ArmIK → q_target
      → SafetyLayer   관절 한계 · 속도 한계 · 소프트 스타트 · 워치독
      → MotorBackend  MockBackend(시뮬) 또는 RoboPartyCANBackend(실기)

백엔드를 갈아끼울 수 있게 만든 이유는 두 가지다. 하드웨어 없이 전체 파이프라인을
검증하기 위해서이고, 실기 시운전 때 "명령이 이상한 건지 모터가 이상한 건지"를
분리해서 볼 수 있어야 하기 때문이다.
"""

from __future__ import annotations

import abc
import time
from dataclasses import dataclass, field

import numpy as np


# ──────────────────────────────────────────────────────────────────────
# 백엔드 인터페이스
# ──────────────────────────────────────────────────────────────────────
class MotorBackend(abc.ABC):
    """관절각 지령 / 상태 읽기의 최소 인터페이스."""

    @property
    @abc.abstractmethod
    def n_joints(self) -> int: ...

    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    @abc.abstractmethod
    def enable(self) -> None:
        """모터 여자(勵磁). 이 시점부터 로봇이 힘을 냅니다."""

    @abc.abstractmethod
    def disable(self) -> None:
        """모터 소자. 팔이 중력에 의해 떨어질 수 있음에 주의."""

    @abc.abstractmethod
    def read_positions(self) -> np.ndarray:
        """현재 관절각 (rad). 로봇 관절 좌표계 기준."""

    @abc.abstractmethod
    def write_positions(self, q: np.ndarray, dq: np.ndarray | None = None) -> None:
        """목표 관절각 (rad) 지령.

        Args:
            q:  목표 관절각
            dq: 목표 관절속도 (rad/s). ★ 임피던스 제어에서 중요하다.

        ★ 왜 dq 가 필요한가 — MIT/임피던스 모드의 토크는

              τ = kp·(p_des − p) + kd·(v_des − v) + τ_ff

          인데, 예전 인터페이스는 q 만 받아서 구현체가 **v_des 를 알 수 없어
          0 을 넣었다.** 그러면 kd 항이 `kd·(0 − v)` 가 되어 **로봇이 자기
          움직임에 제동을 건다.** 실기 실측: kd 2.0, 지령속도 1.131 rad/s 에서
          -2.26 N·m — 이 팔의 브레이크어웨이(0.5~0.8 N·m)보다 큰 제동이었다.

          MockBackend 는 v_des 를 안 쓰므로 **시뮬에서는 절대 안 드러난다.**
          실물에서 만나기 전까지 아무도 몰랐다.
          (젯슨 팀 실기 브링업에서 발견 — handoff/24_JETSON_RESPONSE5.txt §3-2)

          ★★ 다만 **여기 들어온 dq 를 그대로 v_des 에 넣으면 안 된다.** ★★
            상위(Mac)가 주는 dq 는 안전계층을 통과하기 **전**의 의도다.
            안전계층이 위치를 클램프하면 낼 수 있는 속도도 깎이는데, v_des 에
            깎이기 전 값을 넣으면 kd 가 그만큼 되밀어 **안전계층을 우회한다.**
            v_des 는 **자기가 클램프한 지령**의 차분을 써야 한다.
            (실측 근거: 30_AGREED A-14, handoff/26_JETSON_RESPONSE6.txt §3)

          그럼 상위의 dq 는 어디에 쓰나 — "사람이 움직이려 하는가" 를 아는
          유일한 신호다. 클램프된 속도는 정지마찰 구간에서 0 이 되지만
          (이격 제한이 지령을 고정하므로) 이 값은 0 이 아니다.
        """

    def read_velocities(self) -> np.ndarray:
        return np.zeros(self.n_joints)

    def read_temperatures(self) -> np.ndarray:
        return np.full(self.n_joints, np.nan)

    def read_errors(self) -> list[int]:
        return [0] * self.n_joints


# ──────────────────────────────────────────────────────────────────────
# 안전 계층
# ──────────────────────────────────────────────────────────────────────
@dataclass
class SafetyLimits:
    lower: np.ndarray  # rad
    upper: np.ndarray
    max_velocity: np.ndarray  # rad/s (URDF 값)

    velocity_scale: float = 0.3
    """URDF 속도 한계 대비 실제 허용 비율. 시운전은 0.1~0.3 으로 시작한다.

    ★★ 이 값은 속도만 줄이는 게 아니라 **사실상 토크 상한을 만든다.** ★★

      max_step = max_velocity × velocity_scale × dt
      max_lag  = max(max_step × 5, 0.05)        ← 이격 제한(§5-5)
      → 지령-실제 오차가 max_lag 을 못 넘는다
      → 임피던스 제어의 토크는 τ ≈ kp × 오차
      ∴ **낼 수 있는 최대 토크 ∝ velocity_scale**

      실측 (URDF 3.77 rad/s, dt=1/30, kp=100 가정)

          scale   max_lag   정지→출발 도약   최대 토크
          0.1     0.0628        3.6°         6.3 N·m
          0.3     0.1885       10.8°        18.9 N·m
          0.5     0.3142       18.0°        31.4 N·m

    ★ 그래서 **마찰이 큰 팔은 scale 0.1 에서 아예 안 움직인다.**
      그때 kp 를 올리는 건 소용없다 (실기에서 8→100 으로 12배를 올려도
      증상이 그대로였고 scale 한 번에 해결됐다).
      **안 움직이면 kp 가 아니라 velocity_scale 을 먼저 의심할 것.**

    ★ 반대로 scale 을 올리면 정지에서 출발할 때의 **도약이 커진다.**
      마찰 밴드를 넘느라 오차가 max_lag 까지 쌓였다가 한 번에 풀리기 때문이다.
      부드러운 추종과 작은 도약은 맞바꾸기다.

    ※ scale 0.0796 이하에서는 0.05 rad 바닥에 걸려 더 안 줄어든다.
    ※ 이것은 텔레옵 스케일(사람 손 → 로봇 변위 배율)과 **다른 변수**다.
    """

    soft_start_s: float = 2.0
    """기동 직후 이 시간 동안 속도 한계를 0 에서 서서히 올린다.

    전원을 넣은 순간 팔의 실제 위치와 IK 의 목표가 다르면 첫 명령에서 최대 속도로
    튄다. 시운전에서 가장 흔한 사고 원인이라 반드시 둔다.
    """

    watchdog_s: float = 1.0
    """이 시간 동안 새 명령이 없으면 트립시킨다 (텔레옵 링크 끊김 대비).

    ★ 0.5 → 3.0 → 1.0. 처음 3.0 으로 늘린 근거는 "핫스팟은 0.5초 끊김이
    일상적" 이었는데, 젯슨 팀 실측에서 그 전제가 무너졌다 (10분 3000패킷 중
    100ms 초과 0개, 최대 37.7ms). 게다가 링크가 끊기면 lost(500ms)에서 이미
    RUN 에서 내려와 멈추므로 3초를 끌 이유가 없다. 링크가 Wi-Fi 핫스팟이라 0.5초 끊김이 일상적으로
    일어나는데, 그때마다 트립되면 사람이 매번 손으로 해제해야 해서 운용이
    불가능하다. 대신 아래 2단계를 앞에 두어 그 구간에서는 토크를 유지한 채
    지령만 얼린다 (freeze). 트립은 '정말 죽었다'고 볼 수 있을 때만 건다.
    """

    watchdog_freeze_s: float = 0.10
    """이 시간 동안 새 명령이 없으면 지령을 얼린다. 토크는 유지."""

    watchdog_linklost_s: float = 0.50
    """이 시간을 넘으면 '링크 끊김'으로 표시한다. 아직 트립은 아니다."""

    limit_margin: float = 0.05
    """관절 한계에서 이만큼(rad) 안쪽까지만 허용. 하드 스톱 충돌 방지."""


class SafetyLayer:
    """명령을 물리적으로 안전한 범위로 조인다.

    IK 는 기구학만 보므로 "지금 팔이 어디 있는지"와 무관하게 목표를 뱉는다.
    여기서 현재 위치 기준으로 한 스텝 이동량을 제한해야 실제 모터가 따라갈 수 있다.
    """

    def __init__(self, limits: SafetyLimits, dt: float = 1.0 / 30.0):
        self.limits = limits
        self.dt = dt
        self._q_cmd: np.ndarray | None = None
        self._start_time: float | None = None
        self._last_update: float | None = None      # clamp() 호출 시각 (구 워치독)
        self._last_command_at: float | None = None  # 지령 도착 시각 (link_stage 용)
        self.tripped: str | None = None

    def reset(self, q_current: np.ndarray) -> None:
        """현재 실제 위치에서 지령을 다시 시작한다. 기동 시 반드시 호출."""
        self._q_cmd = np.asarray(q_current, dtype=float).copy()
        self._start_time = time.monotonic()
        self._last_update = self._start_time
        self._last_command_at = self._start_time
        self.tripped = None

    @property
    def velocity_limit(self) -> np.ndarray:
        """소프트 스타트를 반영한 현재 속도 한계 (rad/s)."""
        v = self.limits.max_velocity * self.limits.velocity_scale
        if self._start_time is None:
            return v * 0.0
        elapsed = time.monotonic() - self._start_time
        if elapsed >= self.limits.soft_start_s:
            return v
        return v * (elapsed / self.limits.soft_start_s)

    def trip(self, reason: str) -> None:
        """비상 정지 — 현재 지령을 얼려둔다."""
        self.tripped = reason

    def clear_trip(self) -> None:
        """트립 해제. 소프트스타트를 처음부터 다시 건다.

        호출 조건(모드가 DISABLED/HOLD 일 것, 사람이 명시적으로 눌렀을 것)은
        상위 상태머신이 판단한다. 여기서는 해제 동작만 한다.
        """
        self.tripped = None
        self._start_time = time.monotonic()
        self._last_update = self._start_time
        self._last_command_at = self._start_time

    def note_command(self) -> None:
        """원격에서 지령 패킷이 도착했음을 알린다. link_stage() 의 기준 시각.

        ★ clamp() 와 반드시 분리해야 한다. clamp() 는 제어 주기마다 불리는데,
          링크가 끊겨도 마지막 지령을 계속 흘려보내느라 계속 불린다. 그래서
          clamp() 안의 시계로 링크를 판단하면 **영원히 끊긴 걸 모른다.**
          "지령이 왔다"와 "제어 스텝을 돈다"는 다른 사건이다.
        """
        self._last_command_at = time.monotonic()

    @property
    def link_age_s(self) -> float:
        """마지막 지령 수신 후 경과 시간 (초). note_command() 기준."""
        if self._last_command_at is None:
            return 0.0
        return time.monotonic() - self._last_command_at

    def link_stage(self) -> str:
        """링크 상태 3단계. 핫스팟처럼 자주 끊기는 링크를 전제로 한다.

        ★ note_command() 를 지령 수신 시점에 불러줘야 동작한다.

            "ok"        정상
            "stale"     freeze 구간 — 지령을 얼린다. 토크는 살아있다.
            "lost"      링크 끊김 표시 구간. 아직 트립 아님.
            "trip"      트립해야 하는 구간.
        """
        age = self.link_age_s
        if age > self.limits.watchdog_s:
            return "trip"
        if age > self.limits.watchdog_linklost_s:
            return "lost"
        if age > self.limits.watchdog_freeze_s:
            return "stale"
        return "ok"

    def clamp(self, q_target: np.ndarray, q_current: np.ndarray | None = None) -> np.ndarray:
        """목표 관절각을 안전 범위로 조인다.

        ★ 이 함수는 **트립을 걸지 않는다.** 링크 감시는 note_command() 로
          알리고 link_stage() 로 판정한다. 그 이유는 아래 주석 참고.

        Args:
            q_target:  IK 가 낸 목표
            q_current: 실제 관절각. 주면 지령이 실제에서 너무 벌어지지 않게 한다.
        """
        q_target = np.asarray(q_target, dtype=float)
        if self._q_cmd is None:
            self.reset(q_current if q_current is not None else q_target)

        # ★ 여기에 워치독을 두면 안 된다 (예전에 있었고, 실기에서 터졌다).
        #
        #   옛 코드는 clamp() 안에서 _last_update 를 갱신하고 그 간격으로
        #   트립을 걸었다. 그건 "지령이 왔는가" 가 아니라 **"clamp 가 불렸는가"**
        #   를 재는 것이다. 제어 루프가 한 번이라도 밀리면(GC, CPU 경합, CAN
        #   읽기 지연, 로깅 블로킹) 지령은 멀쩡히 오는데도 트립되고,
        #   그 뒤로는 아무리 호출해도 지령이 **영원히 얼어붙었다.**
        #
        #   증상이 지독하다 — 실기에서는 "팔이 그냥 멈췄다" 로만 보이고,
        #   link_stage() 는 "ok" 라고 말한다(진짜 워치독은 정상이므로).
        #   두 워치독이 서로 다른 말을 하는 상태가 된다.
        #   젯슨 팀이 실기 시험 두 시간을 여기에 썼다.
        #
        #   진짜 워치독은 note_command() / link_stage() 다. 지령을 수신한
        #   시점에만 시계를 건드린다. 두 개가 공존할 이유가 없다.
        self._last_update = time.monotonic()

        if self.tripped:
            # 트립 자체는 여전히 지령을 얼린다. 트립을 **거는** 판단만 밖으로
            # 옮긴 것이지, 걸린 뒤의 동작은 그대로다.
            return self._q_cmd.copy()

        lo = np.asarray(self.limits.lower) + self.limits.limit_margin
        hi = np.asarray(self.limits.upper) - self.limits.limit_margin
        q = np.clip(q_target, lo, hi)

        # 한 스텝 이동량 제한 — 실제 모터가 따라갈 수 있는 속도로
        max_step = self.velocity_limit * self.dt
        delta = np.clip(q - self._q_cmd, -max_step, max_step)
        self._q_cmd = self._q_cmd + delta

        # 지령이 실제 위치에서 너무 앞서가면(모터가 못 따라오는 상황) 끌어당긴다.
        # 이걸 안 하면 오차가 계속 쌓이다가 장애물이 치워지는 순간 튄다.
        if q_current is not None:
            lag = self._q_cmd - np.asarray(q_current, dtype=float)
            max_lag = np.maximum(max_step * 5.0, 0.05)
            self._q_cmd = np.asarray(q_current, dtype=float) + np.clip(lag, -max_lag, max_lag)

        return self._q_cmd.copy()


# ──────────────────────────────────────────────────────────────────────
# Mock 백엔드 — 하드웨어 없이 전체 파이프라인 검증용
# ──────────────────────────────────────────────────────────────────────
@dataclass
class MockBackend(MotorBackend):
    """1차 지연으로 실제 모터의 추종 지연을 흉내내는 가짜 백엔드."""

    n: int
    tau: float = 0.05
    """추종 시정수 (초). 실제 모터의 응답 지연을 대략 흉내낸다."""

    q: np.ndarray = field(init=False)
    q_cmd: np.ndarray = field(init=False)
    _connected: bool = field(default=False, init=False)
    _enabled: bool = field(default=False, init=False)
    _last: float | None = field(default=None, init=False)

    def __post_init__(self):
        self.q = np.zeros(self.n)
        self.q_cmd = np.zeros(self.n)

    @property
    def n_joints(self) -> int:
        return self.n

    def connect(self) -> None:
        self._connected = True
        self._last = time.monotonic()

    def disconnect(self) -> None:
        self._connected = False

    def enable(self) -> None:
        self._enabled = True

    def disable(self) -> None:
        self._enabled = False

    def read_positions(self) -> np.ndarray:
        now = time.monotonic()
        dt = 0.0 if self._last is None else max(0.0, now - self._last)
        self._last = now
        if self._enabled and dt > 0:
            alpha = 1.0 - np.exp(-dt / max(self.tau, 1e-6))
            self.q = self.q + alpha * (self.q_cmd - self.q)
        return self.q.copy()

    def write_positions(self, q: np.ndarray, dq: np.ndarray | None = None) -> None:
        # ★ 1차 지연 목이라 dq 를 안 쓴다. 그래서 이 목으로는 §3-2 의 감쇠
        #   문제가 **구조적으로 안 드러난다.** 실물/Isaac 물리에서만 보인다.
        self.q_cmd = np.asarray(q, dtype=float).copy()

    def set_state(self, q: np.ndarray) -> None:
        """테스트용 — 실제 위치를 직접 설정."""
        self.q = np.asarray(q, dtype=float).copy()
        self.q_cmd = self.q.copy()
