"""WebXR ↔ 로봇 좌표계 변환과 클러치(clutch) 로직.

좌표계
------
WebXR `local-floor` 는 **RUB**: x=오른쪽, y=위, z=뒤 (바닥이 원점).
RPO 팔 `base_link` 는 ROS **FLU**: x=앞, y=왼쪽, z=위.
  (URDF 에서 오른팔 arm_pitch origin 이 [0, -0.1217, 0.2052] 로 y 가 음수 → y 는 왼쪽)

    x_flu = -z_rub      손을 앞으로  → 로봇 앞으로
    y_flu = -x_rub      손을 오른쪽  → 로봇 오른쪽(-y)
    z_flu = +y_rub      손을 위로    → 로봇 위로

`teleop` 패키지의 TF_RUB2FLU 와 동일한 행렬이다.

클러치
------
오른손 grip 을 누르는 동안만 팔이 따라온다. 누르는 순간의 컨트롤러 pose 를 기준으로
잡고(latch), 그 이후의 **변위**만 로봇에 전달한다. 손을 뗐다가 편한 자세에서 다시
잡으면 기준이 새로 잡히므로, 사람 팔의 가동범위에 갇히지 않고 로봇을 계속 움직일 수 있다.
(LeRobot `EEReferenceAndDelta` 의 latched reference 와 같은 개념)
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# WebXR RUB → 로봇 FLU
R_WEBXR_TO_ROBOT = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


# 좌우 반사 (xz 평면 기준). 로봇을 마주 보고 조작할 때 "거울처럼" 느껴지는 경우용.
MIRROR_Y = np.diag([1.0, -1.0, 1.0])


def yaw_rotation(deg: float) -> np.ndarray:
    """수직축(z) 둘레 회전. 팔이 놓인 방향과 조작자 방향을 맞추는 데 쓴다."""
    a = np.radians(float(deg))
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame_mapping(base_yaw_deg: float = 0.0, mirror: bool = False) -> np.ndarray:
    """WebXR(RUB) → 로봇(FLU) 매핑 행렬을 만든다.

    Args:
        base_yaw_deg: 수직축 둘레 보정각. 팔을 어느 방향으로 놓았고 조작자가 어디
                      서는지에 따라 달라진다. URDF 만으로는 알 수 없어 실측해야 한다.
        mirror:       좌우 반사. 앞뒤는 맞는데 좌우만 뒤집혀 보일 때 쓴다.
                      회전이 아니라 반사(det=-1)라서 자세에는 그대로 곱하면 안 되고,
                      webxr_rot_to_robot 이 켤레(conjugation)로 처리한다.
    """
    R = yaw_rotation(base_yaw_deg) @ R_WEBXR_TO_ROBOT
    return MIRROR_Y @ R if mirror else R


def webxr_pos_to_robot(p_rub: np.ndarray, mapping: np.ndarray | None = None) -> np.ndarray:
    """WebXR 위치/변위 → 로봇 좌표계."""
    M = R_WEBXR_TO_ROBOT if mapping is None else mapping
    return M @ np.asarray(p_rub, dtype=float)


def webxr_rot_to_robot(rot_rub: np.ndarray, mapping: np.ndarray | None = None) -> np.ndarray:
    """WebXR 회전행렬 → 로봇 좌표계 (닮음 변환).

    mapping 에 반사가 섞여 있어도(det=-1) 켤레 변환 M R M⁻¹ 의 행렬식은
    det(M)·det(R)·det(M⁻¹) = (-1)(1)(-1) = +1 이라 결과는 항상 유효한 회전이다.
    즉 "거울에 비친 회전"이 제대로 나온다.
    """
    M = R_WEBXR_TO_ROBOT if mapping is None else mapping
    return M @ np.asarray(rot_rub, dtype=float) @ np.linalg.inv(M)


def rotation_to_rotvec(rot: np.ndarray) -> np.ndarray:
    """회전행렬 → 회전벡터(axis*angle). scipy 없이 구현."""
    cos_theta = np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)
    theta = float(np.arccos(cos_theta))
    if theta < 1e-8:
        return np.zeros(3)
    if theta > np.pi - 1e-6:
        # 180° 근처는 sin(θ)≈0 이라 일반식이 불안정하다. (R+I) 의 열에서 축을 뽑는다.
        axis_mat = (rot + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(axis_mat), 0.0, None))
        idx = int(np.argmax(axis))
        if axis[idx] > 1e-8:
            axis = axis_mat[:, idx] / axis[idx]
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        return axis * theta
    axis = np.array([
        rot[2, 1] - rot[1, 2],
        rot[0, 2] - rot[2, 0],
        rot[1, 0] - rot[0, 1],
    ]) / (2.0 * np.sin(theta))
    return axis * theta


def rotvec_to_rotation(rotvec: np.ndarray) -> np.ndarray:
    """회전벡터 → 회전행렬 (로드리게스)."""
    rotvec = np.asarray(rotvec, dtype=float)
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.eye(3)
    k = rotvec / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


@dataclass
class ClutchState:
    """grip 클러치. 누르는 순간을 기준으로 상대 변위/회전을 만든다."""

    position_scale: float = 0.76
    """사람 손 변위 → 로봇 EE 변위 배율.

    팔 최대 리치 0.495 m ÷ 사람 어깨~손 리치 0.65 m = 0.76.
    roboparty 의 scale_arms() 는 0.50/0.45 = 1.11 로 확대하지만 그건 전신 휴머노이드
    기준이라 단일 팔에는 46% 과대하다. docs/02_arm_kinematics.md §2 참고.
    """

    enable_threshold: float = 0.5
    """grip 아날로그값이 이 값을 넘으면 활성화."""

    max_step_m: float = 0.05
    """한 프레임에 허용할 최대 변위(m). 트래킹 튐 방지."""

    base_yaw_deg: float = 0.0
    """WebXR → 로봇 매핑의 수직축 보정각. 팔을 놓은 방향에 맞춰 실측으로 정한다."""

    mirror: bool = False
    """좌우 반사. 앞뒤는 맞는데 좌우만 뒤집혀 보일 때."""

    engaged: bool = field(default=False, init=False)
    _ref_pos: np.ndarray | None = field(default=None, init=False, repr=False)
    _ref_rot: np.ndarray | None = field(default=None, init=False, repr=False)
    _last_delta: np.ndarray = field(default_factory=lambda: np.zeros(3), init=False, repr=False)
    _last_rotvec: np.ndarray = field(default_factory=lambda: np.zeros(3), init=False, repr=False)
    # 재그립 시 이어붙일 기준 — grip 상승 엣지에서 직전 명령값으로 갱신된다
    _base_delta: np.ndarray = field(default_factory=lambda: np.zeros(3), init=False, repr=False)
    _base_rot: np.ndarray = field(default_factory=lambda: np.eye(3), init=False, repr=False)
    _mapping: np.ndarray | None = field(default=None, init=False, repr=False)

    @property
    def mapping(self) -> np.ndarray:
        """현재 좌표 매핑 행렬. base_yaw_deg / mirror 가 바뀌면 다시 만든다."""
        key = (self.base_yaw_deg, self.mirror)
        if self._mapping is None or getattr(self, "_mapping_key", None) != key:
            self._mapping = frame_mapping(self.base_yaw_deg, self.mirror)
            self._mapping_key = key
        return self._mapping

    def reset(self) -> None:
        self.engaged = False
        self._ref_pos = None
        self._ref_rot = None
        self._last_delta = np.zeros(3)
        self._last_rotvec = np.zeros(3)
        self._base_delta = np.zeros(3)
        self._base_rot = np.eye(3)


    def update(
        self,
        position_rub: np.ndarray,
        rotation_rub: np.ndarray,
        grip_value: float,
        clamp_fn=None,
    ) -> tuple[bool, np.ndarray, np.ndarray]:
        """한 프레임 갱신.

        Args:
            position_rub: WebXR 좌표계 컨트롤러 위치
            rotation_rub: WebXR 좌표계 컨트롤러 회전행렬
            grip_value:   grip 아날로그 0.0~1.0
            clamp_fn:     delta(로봇 좌표계) → 클램프된 delta. 워크스페이스 제한용.
                          클램프가 일어나면 **현재 손 위치를 기준으로 재앵커**한다.
                          그렇게 하지 않으면 사용자가 도달 범위 밖으로 손을 뻗는 동안
                          내부 누적 변위만 계속 커져서, 손을 되돌릴 때 그 초과분을
                          다 되감기 전까지 로봇이 안 움직이는 dead zone 이 생긴다.

        Returns:
            (engaged, delta_pos_robot, rel_rotvec_robot)
            · engaged      : grip 이 눌린 상태인가
            · delta_pos    : 기준 대비 로봇 좌표계 변위 (m, 이미 스케일 적용)
            · rel_rotvec   : 기준 대비 상대 회전 (회전벡터, 로봇 좌표계)
              LeRobot EEReferenceAndDelta 가 `desired_R = ref_R @ from_rotvec(w)` 로 쓰므로
              기준 프레임에서 본 상대 회전이어야 한다.
        """
        engaged = float(grip_value) > self.enable_threshold

        if not engaged:
            # 놓은 동안에는 마지막 명령을 유지한다. 다시 잡을 때 재기준을 잡으므로
            # 여기서 0으로 되돌리면 로봇이 튄다.
            self.engaged = False
            self._ref_pos = None
            self._ref_rot = None
            return False, self._last_delta.copy(), self._last_rotvec.copy()

        if not self.engaged or self._ref_pos is None:
            # 상승 엣지 — 현재 pose 를 기준으로 latch.
            # 이전 delta 는 유지해서 이어서 조작할 수 있게 한다(재그립).
            self._ref_pos = np.asarray(position_rub, dtype=float).copy()
            self._ref_rot = np.asarray(rotation_rub, dtype=float).copy()
            self._base_delta = self._last_delta.copy()
            self._base_rot = rotvec_to_rotation(self._last_rotvec)
            self.engaged = True

        position = np.asarray(position_rub, dtype=float)
        raw_delta_rub = position - self._ref_pos
        delta_robot = webxr_pos_to_robot(raw_delta_rub, self.mapping) * self.position_scale
        delta = self._base_delta + delta_robot

        step = delta - self._last_delta
        norm = float(np.linalg.norm(step))
        if norm > self.max_step_m:
            delta = self._last_delta + step * (self.max_step_m / norm)

        if clamp_fn is not None:
            clamped = np.asarray(clamp_fn(delta), dtype=float)
            if not np.allclose(clamped, delta, atol=1e-12):
                # 재앵커 — 지금 손 위치를 클램프된 변위에 대응시킨다.
                # 이후 손을 되돌리면 EE 가 즉시 따라온다.
                self._base_delta = clamped.copy()
                self._ref_pos = position.copy()
                delta = clamped

        rel_rub = np.asarray(rotation_rub, dtype=float) @ self._ref_rot.T
        rel_robot = webxr_rot_to_robot(rel_rub, self.mapping) @ self._base_rot

        self._last_delta = delta
        self._last_rotvec = rotation_to_rotvec(rel_robot)
        return True, delta.copy(), self._last_rotvec.copy()
