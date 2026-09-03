"""평행 그리퍼 — 트리거(0~1) → 그리퍼 관절각, 그리고 화면용 로봇.

왜 따로 있나
    신규 팔은 그리퍼가 팔에 붙어 있다. 그런데 **IK 에 쓰는 URDF 는 그리퍼
    관절을 fixed 로 굳힌 판**이다 (placo 가 <mimic> 을 무시해서, 안 굳히면
    종속 관절까지 구동 관절로 세고 IK 가 실물에 없는 자유도를 굴린다).

    그래서 IK 로봇으로는 조가 안 움직인다. 화면에서 그리퍼가 여닫히는 걸
    보려면 **그리퍼 관절이 살아있는 원본 URDF** 를 따로 띄워, 팔 관절은 IK
    결과를 복사하고 그리퍼 관절만 트리거로 몰아야 한다.

    ★ IK 로봇은 건드리지 않는다. 5축 그대로여야 프로토콜이 안 바뀐다.

파일 규약
    IK URDF 가  <이름>_ik.urdf  이면 원본은 같은 폴더의 <이름>.urdf 다.
        assets/arm_v2/arm_v2_ik.urdf  →  assets/arm_v2/arm_v2.urdf
    원본 옆에 gripper_map.py 가 있으면 그걸로 개구↔관절각을 변환한다.
    (실측 56점 피팅. 최대오차 로커각 0.07° / 개구 0.04 mm)

    규약에 안 맞거나 그리퍼가 없는 팔이면 attach() 가 None 을 돌려주고,
    호출하는 쪽은 지금까지처럼 IK 로봇으로 그리기만 하면 된다.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np

# 그리퍼 개구 범위 (mm). 최대는 gripper_map.OPENING_RANGE_MM 에서 읽는다.
# 2026-09-03 갱신판은 82.52 mm (서보 10~198°). 없으면 구판(10~120°) 값으로.
OPEN_MIN_MM = 0.5
OPEN_MAX_FALLBACK_MM = 57.9

# URDF 관절값을 만드는 상수 (기구 담당 문서 §4-4). 신판 gripper_map 은
# joint_rad() 로 직접 주므로 그쪽을 우선 쓴다.
ROCKER_L_ZERO_DEG = 45.10
ROCKER_R_ZERO_DEG = 52.47


class GripperLink:
    """화면용 로봇 + 트리거 → 그리퍼 관절 변환."""

    def __init__(self, robot, full_urdf: Path, gmap, arm_names: list[str]):
        self.robot = robot
        self.urdf_path = str(full_urdf)
        self._gmap = gmap
        self._arm = list(arm_names)
        self._names = set(robot.joint_names())
        rng = getattr(gmap, "OPENING_RANGE_MM", None)
        self.open_max_mm = float(rng[1]) if rng else OPEN_MAX_FALLBACK_MM

    def update(self, q_arm, grasp: float) -> None:
        """팔 관절은 IK 결과를 복사하고, 그리퍼는 trigger 로 연다.

        Args:
            q_arm: IK 가 낸 팔 관절각 (rad). self._arm 순서.
            grasp: 0=열림 … 1=닫힘. ★ 트리거를 당길수록 쥐는 쪽이다.
        """
        for name, val in zip(self._arm, q_arm, strict=True):
            self.robot.set_joint(name, float(val))

        # 트리거 0 → 활짝, 1 → 닫힘
        opening = self.opening_mm(grasp)
        try:
            servo = self._gmap.servo_for_opening(opening)
            if hasattr(self._gmap, "joint_rad"):          # 신판: URDF 값을 바로 준다
                gj, rj = self._gmap.joint_rad(servo)
            else:                                          # 구판: 로커각에서 계산
                rl, rr = self._gmap.rocker_deg(servo)
                gj = np.radians(ROCKER_L_ZERO_DEG - rl)
                rj = np.radians(rr - ROCKER_R_ZERO_DEG)
        except Exception:
            return                       # 변환이 실패해도 팔은 계속 그린다
        vals = {"gripper_joint": float(gj), "rocker_r_joint": float(rj)}
        # 조는 로커의 역회전이다 (mimic ×-1). 평행4절과 등가라 조가 안 돈다.
        vals["jaw_l_joint"] = -vals["gripper_joint"]
        vals["jaw_r_joint"] = -vals["rocker_r_joint"]
        for name, val in vals.items():
            if name in self._names:
                self.robot.set_joint(name, float(val))
        self.robot.update_kinematics()

    def opening_mm(self, grasp: float) -> float:
        m = self.open_max_mm
        return m + (OPEN_MIN_MM - m) * float(np.clip(grasp, 0.0, 1.0))


def attach(ik_urdf: str | Path, arm_names: list[str]) -> GripperLink | None:
    """IK URDF 옆에 그리퍼 원본이 있으면 화면용 로봇을 만들어 준다.

    없거나 그리퍼가 아닌 팔이면 None. 호출하는 쪽은 그냥 넘어가면 된다.
    """
    import placo

    ik_urdf = Path(ik_urdf)
    if not ik_urdf.name.endswith("_ik.urdf"):
        return None
    full = ik_urdf.with_name(ik_urdf.name[: -len("_ik.urdf")] + ".urdf")
    gmap_path = ik_urdf.parent / "gripper_map.py"
    if not full.exists() or not gmap_path.exists():
        return None

    try:
        robot = placo.RobotWrapper(str(full))
    except Exception:
        return None
    if "gripper_joint" not in set(robot.joint_names()):
        return None

    spec = importlib.util.spec_from_file_location("_gripper_map", gmap_path)
    gmap = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(gmap)
    except Exception:
        return None
    if not hasattr(gmap, "servo_for_opening") or not hasattr(gmap, "rocker_deg"):
        return None
    return GripperLink(robot, full, gmap, arm_names)
