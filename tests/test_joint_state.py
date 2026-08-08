"""관절 한계 표시 로직 검증.

조작자는 헤드셋을 쓰면 Mac 화면을 못 본다. 관절이 한계에 걸렸다는 신호를 놓치면
계속 밀어붙이다 하드 스톱에 부딪히므로, 이 판정이 틀리면 안 된다.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rpo_teleop.xr_server import build_joint_state  # noqa: E402


@dataclass
class FakeCfg:
    """ArmConfig 중 build_joint_state 가 쓰는 필드만."""
    joint_names: list
    lower: list
    upper: list


def make_cfg(n=3, lo=-np.pi, hi=np.pi) -> FakeCfg:
    return FakeCfg(
        joint_names=[f"joint{i+1}" for i in range(n)],
        lower=[lo] * n,
        upper=[hi] * n,
    )


def test_center_is_zero_norm():
    """가동범위 한가운데면 norm=0, 상태 ok."""
    cfg = make_cfg()
    st = build_joint_state(cfg, np.zeros(3))
    for j in st["joints"]:
        assert abs(j["norm"]) < 1e-12
        assert j["status"] == "ok"
    assert st["n_near"] == 0


def test_norm_maps_limits_to_plus_minus_one():
    cfg = make_cfg(n=2, lo=-1.0, hi=3.0)  # 중앙 +1.0, 반폭 2.0
    st = build_joint_state(cfg, np.array([-1.0, 3.0]))
    assert np.isclose(st["joints"][0]["norm"], -1.0)
    assert np.isclose(st["joints"][1]["norm"], +1.0)


def test_asymmetric_limits():
    """상하한이 비대칭이어도 중앙 기준으로 정규화되어야 한다."""
    cfg = make_cfg(n=1, lo=-1.745, hi=1.745)  # joint2 스타일 (±100°)
    st = build_joint_state(cfg, np.array([1.745 * 0.5]))
    assert np.isclose(st["joints"][0]["norm"], 0.5)

    cfg2 = FakeCfg(joint_names=["j"], lower=[0.0], upper=[2.0])  # 중앙 1.0
    st2 = build_joint_state(cfg2, np.array([1.5]))
    assert np.isclose(st2["joints"][0]["norm"], 0.5)


def test_near_limit_detected():
    cfg = make_cfg(n=3)
    q = np.array([0.0, np.pi * 0.92, -np.pi * 0.95])  # 2개가 90% 초과
    st = build_joint_state(cfg, q, near_limit_frac=0.9)
    statuses = [j["status"] for j in st["joints"]]
    assert statuses[0] == "ok"
    assert statuses[1] == "near"
    assert statuses[2] == "near"
    assert st["n_near"] == 2


def test_at_limit_detected():
    cfg = make_cfg(n=2)
    st = build_joint_state(cfg, np.array([np.pi, 0.0]))
    assert st["joints"][0]["status"] == "limit"
    assert st["joints"][1]["status"] == "ok"
    assert st["n_near"] == 1


def test_degrees_reported():
    cfg = make_cfg(n=1, lo=-np.pi / 2, hi=np.pi / 2)
    st = build_joint_state(cfg, np.array([np.pi / 4]))
    j = st["joints"][0]
    assert np.isclose(j["deg"], 45.0)
    assert np.isclose(j["lower_deg"], -90.0)
    assert np.isclose(j["upper_deg"], 90.0)


def test_target_included_when_given():
    cfg = make_cfg(n=2)
    st = build_joint_state(cfg, np.zeros(2), q_target=np.array([0.5, -0.5]))
    assert np.isclose(st["joints"][0]["target_deg"], np.degrees(0.5))
    assert np.isclose(st["joints"][1]["target_deg"], np.degrees(-0.5))


def test_target_none_when_absent():
    cfg = make_cfg(n=2)
    st = build_joint_state(cfg, np.zeros(2))
    assert all(j["target_deg"] is None for j in st["joints"])


def test_clamped_flag_passthrough():
    cfg = make_cfg(n=1)
    assert build_joint_state(cfg, np.zeros(1), clamped=True)["clamped"] is True
    assert build_joint_state(cfg, np.zeros(1), clamped=False)["clamped"] is False


def test_zero_span_joint_does_not_divide_by_zero():
    """고정된(상하한이 같은) 관절이 섞여도 죽지 않아야 한다."""
    cfg = FakeCfg(joint_names=["fixed", "normal"], lower=[1.0, -1.0], upper=[1.0, 1.0])
    st = build_joint_state(cfg, np.array([1.0, 0.5]))
    assert st["joints"][0]["norm"] == 0.0
    assert np.isfinite(st["joints"][1]["norm"])


def test_real_config_roundtrip():
    """실제 config/arm.json 으로도 동작하는지."""
    from rpo_teleop.arm_config import ArmConfig

    path = Path(__file__).resolve().parents[1] / "config" / "arm.json"
    if not path.exists():
        return  # 설정이 아직 없는 환경에서는 건너뛴다
    cfg = ArmConfig.load(path)
    st = build_joint_state(cfg, cfg.home)
    assert len(st["joints"]) == cfg.dof
    # 홈 자세는 특이점 회피용으로 고른 것이라 한계에 붙어 있으면 안 된다
    assert st["n_near"] == 0, f"홈 자세가 관절 한계에 근접해 있다: {st['joints']}"
