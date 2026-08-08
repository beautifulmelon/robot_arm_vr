#!/usr/bin/env python3
"""방향 진단 — 자기 팔의 방향표를 직접 뽑는다.

    python3 check_orientation.py                       # 동봉된 두 팔 다
    python3 check_orientation.py urdf/robot_arm.urdf   # 자기 URDF

무엇을 알려주나
---------------
  1. WebXR(사람 손) → 로봇 좌표 매핑 8가지(yaw 4 × 미러 2) 전부
  2. URDF 관절 축 원문
  3. 관절을 + 로 돌리면 손끝이 어느 쪽으로 가는가  ← 실물과 대조할 표
  4. 손목 롤의 **EE 로컬 축** (URDF 축과 다르다. 팔마다 부호도 반대다)

의존성
------
  numpy      필수
  pinocchio  없으면 3·4 를 건너뛰고 1·2 만 낸다 (그것만으로도 절반은 잡힌다)
  메시(STL)  필요 없다. 기구학만 읽는다.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

# ── 이 프로젝트가 쓰는 매핑. transforms.py 와 같은 값이어야 한다 ──────────────
R_WEBXR_TO_ROBOT = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])
MIRROR_Y = np.diag([1.0, -1.0, 1.0])


def yaw_rotation(deg: float) -> np.ndarray:
    a = np.radians(float(deg))
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame_mapping(base_yaw_deg: float = 0.0, mirror: bool = False) -> np.ndarray:
    R = yaw_rotation(base_yaw_deg) @ R_WEBXR_TO_ROBOT
    return MIRROR_Y @ R if mirror else R


def _w(s: str) -> int:
    """터미널 표시폭. 한글은 2칸을 먹는다."""
    return sum(2 if ord(c) > 0x2500 else 1 for c in s)


def _pad(s: str, n: int) -> str:
    return s + " " * max(0, n - _w(s))


def _dir_name(v: np.ndarray) -> str:
    """로봇 좌표 단위벡터를 사람 말로."""
    names = {(1, 0, 0): "앞(+x)", (-1, 0, 0): "뒤(-x)",
             (0, 1, 0): "왼쪽(+y)", (0, -1, 0): "오른쪽(-y)",
             (0, 0, 1): "위(+z)", (0, 0, -1): "아래(-z)"}
    key = tuple(int(round(x)) for x in v)
    return names.get(key, str(np.round(v, 2)))


# ════════════════════════════════════════════════════════════════════════════
def section_mapping() -> None:
    print("=" * 78)
    print("  1. WebXR(사람 손) → 로봇 좌표  —  yaw/미러 8가지 조합")
    print("=" * 78)
    print(f"""
  WebXR local-floor 는 RUB : x=오른쪽  y=위    z=뒤(사람 쪽)
  로봇 base_link 는  FLU  : x=앞      y=왼쪽  z=위

  기본 매핑 R_WEBXR_TO_ROBOT (yaw 0, 미러 off), det = {np.linalg.det(R_WEBXR_TO_ROBOT):+.0f}
""")
    for label, v in (("손 오른쪽 +x", [1, 0, 0]), ("손 위    +y", [0, 1, 0]),
                     ("손 뒤    +z", [0, 0, 1])):
        print(f"      {label}  →  로봇 {_dir_name(R_WEBXR_TO_ROBOT @ np.array(v))}")

    print("\n  ── 조합표 : 손을 그 방향으로 밀면 로봇 EE 가 어디로 가는가 ──\n")
    print("      " + _pad("yaw", 6) + _pad("미러", 7)
          + _pad("손 오른쪽 →", 16) + _pad("손 앞(-z) →", 16) + "손 위 →")
    print("      " + "-" * 62)
    for mirror in (False, True):
        for yaw in (0, 90, 180, 270):
            M = frame_mapping(yaw, mirror)
            r = _dir_name(M @ np.array([1.0, 0, 0]))
            f = _dir_name(M @ np.array([0, 0, -1.0]))
            u = _dir_name(M @ np.array([0, 1.0, 0]))
            print("      " + _pad(str(yaw), 6) + _pad("ON" if mirror else "off", 7)
                  + _pad(r, 16) + _pad(f, 16) + u)

    print(f"""
  ★ 위 8줄 중 **하나는 반드시 맞는다** — 단 사람 손 좌표가 RUB 일 때만.
    8개를 다 돌려도 안 맞으면 yaw/미러로 고칠 수 있는 문제가 아니다.
    특히 '손 위 → 로봇 위(+z)' 는 8줄 전부 같다. 위아래가 반대로 나오면
    매핑 행렬 자체가 틀린 것이다 (§2 참고).

  ★ 미러는 det = {np.linalg.det(MIRROR_Y @ R_WEBXR_TO_ROBOT):+.0f} 이라 회전이 아니라 **반사**다.
    위치에는 그냥 곱해도 되지만 자세에는 그러면 안 된다.
    켤레 변환  M R M⁻¹  로 써야 det 가 +1 로 돌아온다.
    그리고 손처럼 좌우가 있는 부품이 붙어 있으면 조작자가 엄지 위치를
    반대로 인지한다 (거울상). 손을 달았으면 미러는 최후수단이다.
""")


# ════════════════════════════════════════════════════════════════════════════
def section_urdf_axes(urdf: Path) -> tuple[list[str], dict[str, str]]:
    print("=" * 78)
    print(f"  2. URDF 관절 축 원문  —  {urdf.name}")
    print("=" * 78)
    names: list[str] = []
    axes: dict[str, str] = {}
    root = ET.parse(urdf).getroot()
    print(f"\n      {'관절':10s} {'축(xyz)':12s} {'origin(xyz)':22s} {'하한~상한(deg)'}")
    print("      " + "-" * 68)
    for j in root.iter("joint"):
        if j.get("type") not in ("revolute", "continuous"):
            continue
        names.append(j.get("name"))
        ax = j.find("axis")
        og = j.find("origin")
        lim = j.find("limit")
        a = ax.get("xyz") if ax is not None else "1 0 0"
        o = og.get("xyz") if og is not None else "0 0 0"
        axes[j.get("name")] = a
        if lim is not None and lim.get("lower") is not None:
            lo = np.degrees(float(lim.get("lower")))
            hi = np.degrees(float(lim.get("upper")))
            rng = f"{lo:+7.1f} ~ {hi:+7.1f}"
        else:
            rng = "(제한 없음)"
        print(f"      {j.get('name'):10s} {a:12s} {o:22s} {rng}")
    print("""
  ★ 축의 **부호**를 보라. 이 프로젝트의 두 팔은 마지막 관절 축 부호가 서로
    반대다 (0 0 1  vs  0 0 -1). 그래서 손목 롤을 코드에 박으면 한쪽 팔에서
    반드시 반대로 돈다. §4 참고.
""")
    return names, axes


# ════════════════════════════════════════════════════════════════════════════
def section_fk(urdf: Path, names: list[str], axes: dict[str, str]) -> None:
    try:
        import pinocchio as pin
    except ImportError:
        print("=" * 78)
        print("  3·4 건너뜀 — pinocchio 가 없습니다 (pip install pin)")
        print("=" * 78 + "\n")
        return

    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    # EE 프레임 — 마지막 링크에 붙은 프레임을 자동으로 고른다
    fid = model.nframes - 1
    for i in range(model.nframes - 1, -1, -1):
        if model.frames[i].type == pin.FrameType.BODY:
            fid = i
            break
    ee = model.frames[fid].name

    print("=" * 78)
    print(f"  3. 관절을 +15° 돌리면 손끝이 어디로 가는가   (EE 프레임 = {ee})")
    print("=" * 78)
    print("""
  ★ 이 표를 **실물 옆에서** 대조하라. 화면이 아니라 실물이다.
    관절 하나만 + 로 조금 돌려보고 손끝이 표와 같은 쪽으로 가면 부호가 맞다.
    반대로 가면 그 관절의 모터 부호(sign)가 뒤집혀 있는 것이다.
""")

    def fk(q):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return data.oMf[fid].translation.copy()

    for label, q0 in (("전 관절 0 (팔이 곧게 선 자세)", np.zeros(model.nq)),
                      ("팔을 굽힌 자세 (각 관절 +25°)", np.full(model.nq, np.radians(25.0)))):
        p0 = fk(q0)
        print(f"      [{label}]")
        print("      " + _pad("관절", 11) + _pad("Δ앞뒤 x(mm)", 15)
              + _pad("Δ좌우 y(mm)", 15) + _pad("Δ상하 z(mm)", 15) + "  해석")
        print("      " + "-" * 72)
        for i, n in enumerate(names[:model.nq]):
            q = q0.copy()
            q[i] += np.radians(15.0)
            d = (fk(q) - p0) * 1000.0
            if np.linalg.norm(d) < 0.5:
                how = "손끝 안 움직임 (자세 회전만)"
            else:
                k = int(np.argmax(np.abs(d)))
                how = f"주로 {['앞뒤', '좌우', '상하'][k]} — " + \
                      (_dir_name(np.eye(3)[k] * np.sign(d[k])))
            print("      " + _pad(n, 11) + f"{d[0]:>14.1f} {d[1]:>14.1f} {d[2]:>14.1f}   {how}")
        print()

    print("""  ※ '손끝 안 움직임' 은 고장이 아니다. 그 자세에서 회전축이 손끝을 지나가면
     위치는 안 변하고 자세만 변한다. 곧게 선 자세의 베이스 관절이 대표적이다.
     그래서 위에 굽힌 자세를 같이 낸다 — 그쪽 표로 확인하라.
""")

    # ── 손목 롤 EE 로컬 축 ────────────────────────────────────────────────
    print("=" * 78)
    print("  4. 손목 롤의 EE 로컬 회전축  —  URDF 축과 다르다")
    print("=" * 78)
    q0 = np.zeros(model.nq)
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)
    R0 = data.oMf[fid].rotation.copy()
    q1 = q0.copy()
    q1[-1] += np.radians(5.0)
    pin.forwardKinematics(model, data, q1)
    pin.updateFramePlacements(model, data)
    R1 = data.oMf[fid].rotation.copy()

    R = R0.T @ R1
    ang = float(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)))
    if ang < np.radians(0.5):
        print("\n      마지막 관절이 EE 자세를 거의 안 바꾼다 — 이 팔은 손목 롤이 없다.\n")
        return
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(ang))
    axis = axis / np.linalg.norm(axis)
    print(f"""
      마지막 관절  {names[-1]}
      URDF 축      (링크 기준)      {axes.get(names[-1], '?')}
      EE 로컬 축   (실측)           {np.round(axis, 3).tolist()}

  ★ 이 둘이 다르다. EE 프레임이 관절 프레임에 대해 돌아가 있기 때문이다.
    롤 지령을 만들 때 써야 하는 건 **EE 로컬 축** 쪽이다.
    코드에 박지 말고 기동할 때 이렇게 한 번 재라.
""")


# ════════════════════════════════════════════════════════════════════════════
def main() -> int:
    args = sys.argv[1:]
    if args:
        urdfs = [Path(a) for a in args]
    else:
        urdfs = sorted((HERE / "urdf").glob("*.urdf"))
    if not urdfs:
        print("URDF 를 못 찾았습니다. 경로를 인자로 주세요.")
        return 1

    section_mapping()
    for u in urdfs:
        if not u.exists():
            print(f"!! 없는 파일: {u}")
            continue
        names, axes = section_urdf_axes(u)
        section_fk(u, names, axes)
    print("=" * 78)
    print("  끝. 증상별 원인은 28_ORIENTATION_GUIDE.txt §1 표를 보세요.")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
