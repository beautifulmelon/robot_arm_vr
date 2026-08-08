"""좌표 변환·클러치 로직 검증.

좌표계 버그는 실기로 확인하기 전까지 드러나지 않고, 드러났을 때는 로봇이 엉뚱한
방향으로 튀는 형태라 위험하다. 여기서 방향 감각을 수치로 못박아 둔다.

실행: .venv/bin/python -m pytest tests/ -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rpo_teleop.transforms import (  # noqa: E402
    ClutchState,
    R_WEBXR_TO_ROBOT,
    rotation_to_rotvec,
    rotvec_to_rotation,
    webxr_pos_to_robot,
    webxr_rot_to_robot,
)


# ── 방향 감각 ──────────────────────────────────────────────────────────
@pytest.mark.parametrize(
    "webxr, robot, desc",
    [
        ([0, 0, -1], [1, 0, 0], "손을 앞으로(-z) → 로봇 앞(+x)"),
        ([1, 0, 0], [0, -1, 0], "손을 오른쪽(+x) → 로봇 오른쪽(-y)"),
        ([0, 1, 0], [0, 0, 1], "손을 위로(+y) → 로봇 위(+z)"),
        ([0, 0, 1], [-1, 0, 0], "손을 뒤로(+z) → 로봇 뒤(-x)"),
        ([-1, 0, 0], [0, 1, 0], "손을 왼쪽(-x) → 로봇 왼쪽(+y)"),
        ([0, -1, 0], [0, 0, -1], "손을 아래로(-y) → 로봇 아래(-z)"),
    ],
)
def test_direction_mapping(webxr, robot, desc):
    assert np.allclose(webxr_pos_to_robot(np.array(webxr, float)), robot), desc


def test_transform_is_proper_rotation():
    """반사가 섞이면 회전 방향이 뒤집힌다 — det=+1 이어야 한다."""
    assert np.isclose(np.linalg.det(R_WEBXR_TO_ROBOT), 1.0)
    assert np.allclose(R_WEBXR_TO_ROBOT @ R_WEBXR_TO_ROBOT.T, np.eye(3))


def test_rotation_similarity_preserves_angle():
    """좌표계를 바꿔도 회전 각도 자체는 보존되어야 한다."""
    rng = np.random.default_rng(0)
    for _ in range(50):
        rotvec = rng.normal(0, 1, 3)
        rot = rotvec_to_rotation(rotvec)
        rot_robot = webxr_rot_to_robot(rot)
        assert np.isclose(np.trace(rot), np.trace(rot_robot), atol=1e-9)


# ── 회전벡터 왕복 ──────────────────────────────────────────────────────
def test_rotvec_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(200):
        axis = rng.normal(0, 1, 3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(0, np.pi - 1e-3)
        rotvec = axis * angle
        back = rotation_to_rotvec(rotvec_to_rotation(rotvec))
        assert np.allclose(back, rotvec, atol=1e-7)


def test_rotvec_near_180_degrees():
    """θ→π 에서 sin(θ)≈0 이라 일반식이 무너지는 구간."""
    for axis in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]),
                 np.array([1.0, 1.0, 0]) / np.sqrt(2)):
        rotvec = axis * (np.pi - 1e-9)
        rot = rotvec_to_rotation(rotvec)
        back = rotation_to_rotvec(rot)
        # 축의 부호는 ±가 모두 같은 회전이므로 회전행렬로 되돌려 비교한다
        assert np.allclose(rotvec_to_rotation(back), rot, atol=1e-6)


def test_rotvec_identity():
    assert np.allclose(rotation_to_rotvec(np.eye(3)), np.zeros(3))


# ── 클러치 ────────────────────────────────────────────────────────────
def test_clutch_disengaged_holds_last_command():
    """grip 을 놓으면 로봇은 마지막 자세를 유지해야 한다 (0으로 튀면 안 됨)."""
    clutch = ClutchState()
    eye = np.eye(3)

    clutch.update(np.zeros(3), eye, grip_value=1.0)  # latch
    engaged, delta, _ = clutch.update(np.array([0.0, 0.0, -0.1]), eye, grip_value=1.0)
    assert engaged
    moved = delta.copy()
    assert np.linalg.norm(moved) > 0

    engaged, delta_off, _ = clutch.update(np.array([0.5, 0.5, 0.5]), eye, grip_value=0.0)
    assert not engaged
    assert np.allclose(delta_off, moved), "놓은 동안 명령이 유지되어야 한다"


def test_clutch_regrip_continues_from_last():
    """뗐다 다시 잡으면 이어서 움직여야 한다 (재기준). 이게 clutch 의 핵심."""
    clutch = ClutchState(max_step_m=1e9)  # 레이트 리밋 배제
    eye = np.eye(3)

    clutch.update(np.zeros(3), eye, 1.0)
    _, d1, _ = clutch.update(np.array([0.0, 0.0, -0.2]), eye, 1.0)  # 앞으로 0.2 m
    clutch.update(np.array([0.0, 0.0, -0.2]), eye, 0.0)  # 놓기

    # 손을 원위치로 되돌린 뒤 다시 잡고 또 앞으로
    clutch.update(np.zeros(3), eye, 1.0)  # 재latch (여기가 새 기준)
    _, d2, _ = clutch.update(np.array([0.0, 0.0, -0.2]), eye, 1.0)

    assert np.isclose(d2[0], 2 * d1[0], rtol=1e-6), "재그립 후 변위가 누적되어야 한다"


def test_clutch_scale_applied():
    clutch = ClutchState(position_scale=0.76, max_step_m=1e9)
    eye = np.eye(3)
    clutch.update(np.zeros(3), eye, 1.0)
    _, delta, _ = clutch.update(np.array([0.0, 0.0, -1.0]), eye, 1.0)  # 손 1 m 전진
    assert np.isclose(delta[0], 0.76), "스케일 0.76 이 적용되어야 한다"


def test_clutch_rate_limit():
    """트래킹이 튀어도 한 프레임 변위가 max_step_m 을 넘지 않아야 한다."""
    clutch = ClutchState(max_step_m=0.05)
    eye = np.eye(3)
    clutch.update(np.zeros(3), eye, 1.0)
    _, delta, _ = clutch.update(np.array([0.0, 0.0, -10.0]), eye, 1.0)  # 10 m 순간이동
    assert np.linalg.norm(delta) <= 0.05 + 1e-9


def test_clutch_rotation_relative_to_reference():
    """기준 자세에서의 상대 회전이 나와야 한다."""
    clutch = ClutchState()
    r0 = rotvec_to_rotation(np.array([0.0, 0.3, 0.0]))
    clutch.update(np.zeros(3), r0, 1.0)  # latch (자세 r0)

    _, _, rotvec = clutch.update(np.zeros(3), r0, 1.0)  # 같은 자세 유지
    assert np.allclose(rotvec, np.zeros(3), atol=1e-9), "안 돌렸으면 상대 회전 0"

    extra = rotvec_to_rotation(np.array([0.0, 0.0, 0.5]))
    _, _, rotvec2 = clutch.update(np.zeros(3), extra @ r0, 1.0)
    assert np.linalg.norm(rotvec2) > 0.4, "회전시켰으면 상대 회전이 잡혀야 한다"
    assert np.isclose(np.linalg.norm(rotvec2), 0.5, atol=1e-6)


def test_clutch_reset():
    clutch = ClutchState()
    clutch.update(np.zeros(3), np.eye(3), 1.0)
    clutch.update(np.array([0.0, 0.0, -0.1]), np.eye(3), 1.0)
    clutch.reset()
    assert not clutch.engaged
    _, delta, rotvec = clutch.update(np.zeros(3), np.eye(3), 0.0)
    assert np.allclose(delta, 0) and np.allclose(rotvec, 0)


# ── 워크스페이스 클램프 + dead zone ────────────────────────────────────
def _sphere_clamp(max_r: float):
    """원점 기준 반경 max_r 로 제한하는 clamp_fn."""
    def fn(delta):
        r = float(np.linalg.norm(delta))
        return delta if r <= max_r else delta * (max_r / r)
    return fn


def test_clamp_limits_delta():
    clutch = ClutchState(position_scale=1.0, max_step_m=1e9)
    eye = np.eye(3)
    clutch.update(np.zeros(3), eye, 1.0)
    _, delta, _ = clutch.update(np.array([0.0, 0.0, -5.0]), eye, 1.0, clamp_fn=_sphere_clamp(0.3))
    assert np.isclose(np.linalg.norm(delta), 0.3), "클램프 반경을 넘으면 안 된다"


def test_clamp_no_dead_zone_on_return():
    """도달 범위 밖으로 뻗었다가 되돌릴 때 EE 가 즉시 따라와야 한다.

    재앵커가 없으면 내부 누적 변위만 커져서, 손을 되돌려도 초과분을 다 되감기
    전까지 로봇이 안 움직인다. 실제 조작에서 가장 답답한 실패 모드.
    """
    clamp = _sphere_clamp(0.3)
    clutch = ClutchState(position_scale=1.0, max_step_m=1e9)
    eye = np.eye(3)
    clutch.update(np.zeros(3), eye, 1.0)

    # 한참 밖으로 뻗기 (앞으로 2 m → 클램프되어 0.3)
    _, far, _ = clutch.update(np.array([0.0, 0.0, -2.0]), eye, 1.0, clamp_fn=clamp)
    assert np.isclose(np.linalg.norm(far), 0.3)

    # 10 cm 만 되돌리면 EE 도 10 cm 되돌아와야 한다
    _, back, _ = clutch.update(np.array([0.0, 0.0, -1.9]), eye, 1.0, clamp_fn=clamp)
    assert np.isclose(back[0], far[0] - 0.1, atol=1e-6), (
        f"되돌릴 때 즉시 따라와야 한다: far={far[0]:.4f} back={back[0]:.4f}"
    )


def test_clamp_monotonic_return():
    """밖으로 뻗었다 완전히 되돌아오는 동안 EE 가 단조 감소해야 한다 (역전 없음)."""
    clamp = _sphere_clamp(0.3)
    clutch = ClutchState(position_scale=1.0, max_step_m=1e9)
    eye = np.eye(3)
    clutch.update(np.zeros(3), eye, 1.0)

    for z in (-0.3, -0.8, -1.5):  # 점점 밖으로
        clutch.update(np.array([0.0, 0.0, z]), eye, 1.0, clamp_fn=clamp)

    xs = []
    for z in (-1.5, -1.2, -0.9, -0.6, -0.3, 0.0):  # 되돌아오기
        _, d, _ = clutch.update(np.array([0.0, 0.0, z]), eye, 1.0, clamp_fn=clamp)
        xs.append(d[0])
    assert all(b <= a + 1e-9 for a, b in zip(xs, xs[1:])), f"단조 감소해야 한다: {np.round(xs,4)}"
    assert xs[-1] < 0.05, f"완전히 되돌아오면 원점 근처여야 한다: {xs[-1]:.4f}"


# ── 좌표 매핑 보정 (yaw / mirror) ──────────────────────────────────────
def test_default_mapping_unchanged():
    """기본값은 기존 동작과 같아야 한다 (회귀 방지)."""
    from rpo_teleop.transforms import frame_mapping
    assert np.allclose(frame_mapping(0.0, False), R_WEBXR_TO_ROBOT)


def test_yaw_is_proper_rotation():
    """yaw 보정은 회전이어야 한다 (det=+1). 반사가 섞이면 회전 방향이 뒤집힌다."""
    from rpo_teleop.transforms import frame_mapping
    for deg in (0, 45, 90, 180, 270, -90):
        M = frame_mapping(deg, False)
        assert np.isclose(np.linalg.det(M), 1.0), f"yaw={deg} det != 1"
        assert np.allclose(M @ M.T, np.eye(3), atol=1e-12)


def test_mirror_flips_only_left_right():
    """미러는 좌우만 뒤집고 앞뒤·위아래는 그대로여야 한다.

    사용자가 보고한 증상이 정확히 이것: 앞뒤는 맞는데 좌우만 반대.
    """
    from rpo_teleop.transforms import frame_mapping, webxr_pos_to_robot
    M = frame_mapping(0.0, True)
    fwd = webxr_pos_to_robot(np.array([0.0, 0.0, -1.0]), M)   # 손 앞으로
    right = webxr_pos_to_robot(np.array([1.0, 0.0, 0.0]), M)  # 손 오른쪽
    up = webxr_pos_to_robot(np.array([0.0, 1.0, 0.0]), M)     # 손 위로
    assert np.allclose(fwd, [1, 0, 0]), "앞뒤는 그대로여야 한다"
    assert np.allclose(up, [0, 0, 1]), "위아래는 그대로여야 한다"
    assert np.allclose(right, [0, +1, 0]), "좌우만 뒤집혀야 한다 (기본은 -y)"


def test_mirrored_rotation_is_still_valid():
    """반사가 섞인 매핑이라도 자세 변환 결과는 유효한 회전이어야 한다.

    M 자체는 det=-1 이지만 켤레 변환 M R M⁻¹ 은 det=+1 이 된다.
    이게 성립하지 않으면 IK 에 들어가는 목표 자세가 물리적으로 불가능한 행렬이 된다.
    """
    from rpo_teleop.transforms import frame_mapping, webxr_rot_to_robot
    M = frame_mapping(0.0, True)
    assert np.isclose(np.linalg.det(M), -1.0), "미러 매핑 자체는 반사"
    rng = np.random.default_rng(3)
    for _ in range(50):
        R = rotvec_to_rotation(rng.normal(0, 1, 3))
        Rr = webxr_rot_to_robot(R, M)
        assert np.isclose(np.linalg.det(Rr), 1.0, atol=1e-9)
        assert np.allclose(Rr @ Rr.T, np.eye(3), atol=1e-9)


def test_yaw_180_flips_both_axes():
    """yaw 180° 는 앞뒤와 좌우를 함께 뒤집는다 (좌우만 뒤집는 게 아니다)."""
    from rpo_teleop.transforms import frame_mapping, webxr_pos_to_robot
    M = frame_mapping(180.0, False)
    assert np.allclose(webxr_pos_to_robot(np.array([0.0, 0.0, -1.0]), M), [-1, 0, 0], atol=1e-12)
    assert np.allclose(webxr_pos_to_robot(np.array([1.0, 0.0, 0.0]), M), [0, +1, 0], atol=1e-12)


def test_clutch_respects_mirror():
    """클러치가 미러 설정을 실제로 반영하는가."""
    eye = np.eye(3)
    normal = ClutchState(position_scale=1.0, max_step_m=1e9)
    mirrored = ClutchState(position_scale=1.0, max_step_m=1e9, mirror=True)
    for c in (normal, mirrored):
        c.update(np.zeros(3), eye, 1.0)
    _, dn, _ = normal.update(np.array([1.0, 0.0, 0.0]), eye, 1.0)    # 손 오른쪽 1 m
    _, dm, _ = mirrored.update(np.array([1.0, 0.0, 0.0]), eye, 1.0)
    assert np.isclose(dn[1], -1.0) and np.isclose(dm[1], +1.0), "y 부호가 반대여야 한다"
    assert np.isclose(dn[0], dm[0]), "x(앞뒤)는 같아야 한다"


def test_clutch_mapping_updates_when_setting_changes():
    """실시간으로 mirror 를 토글하면 매핑이 즉시 반영되어야 한다."""
    c = ClutchState(position_scale=1.0, max_step_m=1e9)
    before = c.mapping.copy()
    c.mirror = True
    after = c.mapping
    assert not np.allclose(before, after), "설정을 바꿨는데 매핑이 그대로다 (캐시 버그)"


# ── 회전 적용 순서 (월드 기준 vs 바디 기준) ────────────────────────────
def test_clutch_returns_world_frame_relative_rotation():
    """클러치가 주는 상대 회전은 **월드 기준**이어야 한다.

    즉 R_now·R_ref⁻¹ 형태다. 이 규약이 바뀌면 시뮬레이터의 적용 순서
    (rel @ home_R) 도 함께 바꿔야 한다. 섞어 쓰면 회전축이 home 자세만큼
    틀어져 손목을 돌릴 때 로봇 손이 엉뚱한 축으로 돈다 (실측 170.4° 어긋남).
    """
    clutch = ClutchState()
    ref = rotvec_to_rotation(np.array([0.4, -0.2, 0.7]))   # 기울어진 기준 자세
    clutch.update(np.zeros(3), ref, 1.0)

    # 월드축 n 둘레로 정확히 theta 회전
    n = np.array([0.0, 1.0, 0.0])
    theta = np.radians(25)
    R_world = rotvec_to_rotation(n * theta)
    _, _, rotvec = clutch.update(np.zeros(3), R_world @ ref, 1.0)

    # 결과는 기준 자세와 무관하게 같은 월드축·같은 각이어야 한다
    got = rotvec_to_rotation(rotvec)
    want = webxr_rot_to_robot(R_world)      # 좌표계만 바뀐 같은 회전
    assert np.allclose(got, want, atol=1e-9), (
        "상대 회전이 월드 기준이 아니다 — 기준 자세에 따라 축이 달라진다"
    )


def test_relative_rotation_independent_of_reference_pose():
    """같은 손목 동작이면 잡은 순간의 자세와 무관하게 같은 회전이 나와야 한다."""
    n, theta = np.array([0.0, 0.0, 1.0]), np.radians(40)
    R_world = rotvec_to_rotation(n * theta)

    results = []
    for ref_rotvec in ([0, 0, 0], [0.5, 0.3, -0.2], [-1.1, 0.7, 0.4]):
        clutch = ClutchState()
        ref = rotvec_to_rotation(np.array(ref_rotvec, dtype=float))
        clutch.update(np.zeros(3), ref, 1.0)
        _, _, rv = clutch.update(np.zeros(3), R_world @ ref, 1.0)
        results.append(rv)

    for a, b in zip(results, results[1:]):
        assert np.allclose(a, b, atol=1e-9), f"기준 자세에 따라 결과가 달라짐: {a} vs {b}"


def test_world_frame_application_preserves_axis():
    """월드 기준 회전을 왼쪽에 곱하면 EE 회전축이 보존된다 (바디 기준이면 틀어진다)."""
    home_R = rotvec_to_rotation(np.array([np.pi, 0.05, -0.08]))   # 뒤집힌 home 자세
    R_rel = rotvec_to_rotation(np.array([0.0, 0.0, np.radians(30)]))

    def ee_axis(target_R):
        v = rotation_to_rotvec(target_R @ home_R.T)
        return v / (np.linalg.norm(v) + 1e-12)

    world_axis = ee_axis(R_rel @ home_R)     # 올바른 적용
    body_axis = ee_axis(home_R @ R_rel)      # 잘못된 적용

    # 회전벡터의 축 부호는 회전 방향을 뜻하므로 abs 를 쓰면 안 된다.
    # 부호가 뒤집히는 것(= 반대로 도는 것)이 바로 이 버그의 증상이다.
    assert np.allclose(world_axis, [0, 0, 1], atol=1e-6), \
        f"월드축이 그대로 보존되어야 한다: {world_axis}"

    ang = np.degrees(np.arccos(np.clip(float(world_axis @ body_axis), -1, 1)))
    assert ang > 90.0, (
        f"두 규약이 {ang:.1f}° 밖에 차이 안 남 — 이 home 자세로는 버그를 못 잡는다"
    )
