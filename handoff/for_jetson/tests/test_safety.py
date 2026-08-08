"""안전 계층 검증.

실기 사고는 대부분 여기서 막아야 하는 것들이다. 각 방어 장치가 실제로 동작하는지
수치로 못박아 둔다.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rpo_teleop.motor_backend import MockBackend, SafetyLayer, SafetyLimits  # noqa: E402

N = 5


def make_layer(**kw) -> SafetyLayer:
    limits = SafetyLimits(
        lower=np.full(N, -np.pi),
        upper=np.full(N, np.pi),
        max_velocity=np.full(N, 3.14),
        **kw,
    )
    return SafetyLayer(limits, dt=1 / 30)


def test_joint_limits_enforced_with_margin():
    layer = make_layer(soft_start_s=0.0, velocity_scale=1.0)
    layer.reset(np.zeros(N))
    for _ in range(200):
        q = layer.clamp(np.full(N, 10.0), np.zeros(N))
    assert np.all(q <= np.pi - layer.limits.limit_margin + 1e-9), "한계 안쪽 마진까지만 허용"


def test_velocity_limit_caps_step():
    layer = make_layer(soft_start_s=0.0, velocity_scale=0.3)
    layer.reset(np.zeros(N))
    q = layer.clamp(np.full(N, 3.0), np.zeros(N))
    max_step = 3.14 * 0.3 * (1 / 30)
    assert np.all(np.abs(q) <= max_step + 1e-9), f"한 스텝 {np.abs(q).max():.4f} > {max_step:.4f}"


def test_soft_start_ramps_from_zero():
    """기동 직후에는 속도 한계가 0에서 시작해야 한다.

    전원 투입 순간 실제 위치와 목표가 다르면 첫 명령에서 최대 속도로 튄다.
    시운전 사고의 가장 흔한 원인.
    """
    layer = make_layer(soft_start_s=1.0, velocity_scale=1.0)
    layer.reset(np.zeros(N))
    assert np.allclose(layer.velocity_limit, 0.0, atol=1e-3), "기동 직후 속도 한계는 0"

    first = layer.clamp(np.full(N, 3.0), np.zeros(N))
    assert np.all(np.abs(first) < 1e-2), f"첫 스텝이 커서는 안 된다: {np.abs(first).max():.4f}"

    time.sleep(0.3)
    mid = layer.velocity_limit
    assert np.all(mid > 0) and np.all(mid < 3.14), "램프 중간값이어야 한다"


def test_watchdog_measures_commands_not_clamp_calls():
    """★ 워치독은 "지령이 왔는가" 를 재야 한다. "clamp 가 불렸는가" 가 아니다.

    예전에는 clamp() 안에서 시계를 갱신하고 그 간격으로 트립을 걸었다.
    제어 루프가 한 번이라도 밀리면(GC, CPU 경합, CAN 읽기 지연) 지령은 멀쩡히
    오는데도 트립되고, 그 뒤로는 아무리 호출해도 지령이 **영원히 얼어붙었다.**
    실기에서 "팔이 그냥 멈췄다" 로만 보였고 원인을 찾는 데 두 시간이 걸렸다.
    (젯슨 팀 실기 브링업에서 발견 — handoff/22_JETSON_RESPONSE4.txt §2)
    """
    layer = make_layer(soft_start_s=0.0, watchdog_s=0.05, velocity_scale=1.0)
    layer.reset(np.zeros(N))
    layer.note_command()

    q = np.zeros(N)
    for _ in range(5):
        q = layer.clamp(np.full(N, 2.0), q)

    time.sleep(0.12)                 # ★ 제어 루프가 밀렸다. 지령은 계속 온다.
    layer.note_command()
    for _ in range(60):
        layer.note_command()
        q = layer.clamp(np.full(N, 2.0), q)

    assert layer.tripped is None, f"루프가 밀렸다고 트립하면 안 된다: {layer.tripped}"
    assert np.all(q > 1.9), f"지령이 얼어붙었다: {q}"


def test_link_stage_still_catches_real_command_loss():
    """진짜로 지령이 끊기면 여전히 잡아야 한다. 감시 자체를 없앤 게 아니다."""
    layer = make_layer(soft_start_s=0.0, watchdog_s=0.05,
                       watchdog_freeze_s=0.01, watchdog_linklost_s=0.02)
    layer.reset(np.zeros(N))
    layer.note_command()
    assert layer.link_stage() == "ok"
    time.sleep(0.08)                 # note_command() 를 안 부른다 = 지령 끊김
    assert layer.link_stage() == "trip", "지령이 끊겼는데 못 잡았다"


def test_clamp_still_freezes_once_tripped():
    """트립을 **거는** 판단만 밖으로 옮겼다. 걸린 뒤의 동작은 그대로다."""
    layer = make_layer(soft_start_s=0.0, velocity_scale=1.0)
    layer.reset(np.zeros(N))
    layer.note_command()
    q = np.zeros(N)
    for _ in range(5):
        q = layer.clamp(np.full(N, 2.0), q)
    before = q.copy()
    layer.trip("사람이 비상정지")
    for _ in range(30):
        q = layer.clamp(np.full(N, 2.0), q)
    assert np.allclose(q, before), "트립됐는데 계속 움직였다"


def test_trip_freezes_command():
    layer = make_layer(soft_start_s=0.0)
    layer.reset(np.zeros(N))
    q1 = layer.clamp(np.full(N, 1.0), np.zeros(N))
    layer.trip("사용자 비상정지")
    for _ in range(10):
        q2 = layer.clamp(np.full(N, 3.0), np.zeros(N))
    assert np.allclose(q1, q2), "트립되면 더 이상 움직이면 안 된다"


def test_command_lag_bounded():
    """모터가 못 따라올 때 지령이 앞서 나가 쌓이면 안 된다.

    쌓인 오차는 장애물이 치워지는 순간 한꺼번에 풀리며 급발진으로 나타난다.
    """
    layer = make_layer(soft_start_s=0.0, velocity_scale=1.0)
    layer.reset(np.zeros(N))
    stuck = np.zeros(N)  # 실제 관절은 막혀서 안 움직인다고 가정
    for _ in range(300):
        q = layer.clamp(np.full(N, 3.0), stuck)
    max_step = 3.14 * 1.0 * (1 / 30)
    assert np.all(np.abs(q - stuck) <= max(max_step * 5.0, 0.05) + 1e-9), \
        f"지령-실제 괴리가 {np.abs(q - stuck).max():.4f} 로 너무 크다"


def test_reset_starts_from_actual_position():
    """기동 시 지령은 반드시 '지금 팔이 있는 위치'에서 시작해야 한다."""
    layer = make_layer(soft_start_s=0.0)
    actual = np.array([0.3, -0.5, 1.0, 0.2, -0.1])
    layer.reset(actual)
    q = layer.clamp(actual, actual)
    assert np.allclose(q, actual, atol=1e-9), "리셋 직후엔 현재 위치를 그대로 지령"


# ── Mock 백엔드 ────────────────────────────────────────────────────────
def test_mock_backend_does_not_move_when_disabled():
    be = MockBackend(n=N)
    be.connect()
    be.write_positions(np.ones(N))
    for _ in range(20):
        q = be.read_positions()
        time.sleep(0.005)
    assert np.allclose(q, 0.0), "소자 상태에서는 움직이면 안 된다"


def test_mock_backend_converges_when_enabled():
    be = MockBackend(n=N, tau=0.02)
    be.connect()
    be.enable()
    be.write_positions(np.ones(N))
    for _ in range(60):
        be.read_positions()
        time.sleep(0.005)
    assert np.allclose(be.read_positions(), 1.0, atol=0.05), "여자 상태에서는 목표로 수렴"


def test_safety_layer_with_mock_end_to_end():
    """안전 계층 + 백엔드를 붙여 30 Hz 루프를 돌려본다."""
    be = MockBackend(n=N, tau=0.02)
    be.connect()
    be.enable()
    layer = make_layer(soft_start_s=0.2, velocity_scale=0.5)
    layer.reset(be.read_positions())

    target = np.array([0.5, -0.4, 0.3, 0.6, -0.2])
    speeds = []
    for _ in range(200):
        q_actual = be.read_positions()
        q_cmd = layer.clamp(target, q_actual)
        be.write_positions(q_cmd)
        speeds.append(q_cmd.copy())
        time.sleep(1 / 200)

    steps = np.abs(np.diff(np.array(speeds), axis=0))
    assert steps.max() <= 3.14 * 0.5 * (1 / 30) + 1e-6, "속도 한계 초과"
    assert np.allclose(be.read_positions(), target, atol=0.05), "결국 목표에 도달해야 한다"
