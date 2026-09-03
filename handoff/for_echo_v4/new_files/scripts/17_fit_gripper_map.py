#!/usr/bin/env python3
"""gripper_sweep.json(퓨전 실측) → gripper_map.py 를 다시 만든다.

왜 있나 — 2026-09-03 전달본 gripper_map.py 의 7차 다항식은 최고차항이 -0.0 으로
찍혀(반올림) 서보 100° 를 넘으면 발산한다 (198° 에서 개구 -1092 mm). 그래서
다항식을 버리고 **실측 95점을 그대로 보간표**로 쓴다. 발산할 것이 없다.

로커각 정의는 기구 담당 규약을 그대로 따른다.
    gripper_joint  = radians(45.10 - 로커L)     rocker_r_joint = radians(로커R - 52.47)
스윕의 조 좌표(y,z)에 반경 40 mm 원을 맞춰 각도를 뽑고, 부호·오프셋은
구판(10~120°, 56점) gripper_map 과 최소자승으로 맞춘다. 결과가 기구 담당이 준
URDF 리밋(gripper_joint 47.50°, rocker_r -54.57°)과 0.2° 안에서 맞는 것으로 검산한다.
개구는 스윕에서 yR - yL 그대로다 (198° 에서 82.52 mm — 문서값과 일치).
"""
from __future__ import annotations
import argparse, importlib.util, json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
ROCKER_L_ZERO_DEG, ROCKER_R_ZERO_DEG = 45.10, 52.47

def load(p, n):
    s = importlib.util.spec_from_file_location(n, p); m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m

def circle(P):
    y, z = P[:, 1], P[:, 2]; A = np.c_[2*y, 2*z, np.ones_like(y)]; b = y**2 + z**2
    cy, cz, c = np.linalg.lstsq(A, b, rcond=None)[0]; return cy, cz, np.sqrt(c + cy**2 + cz**2)

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", type=Path, default=ROOT / "assets/arm_v2/gripper_sweep.json")
    ap.add_argument("--ref",   type=Path, default=ROOT / "assets/arm_v1/gripper_map.py",
                    help="부호·오프셋 기준으로 삼을 구판 (10~120° 신뢰)")
    ap.add_argument("--out",   type=Path, default=ROOT / "assets/arm_v2/gripper_map.py")
    a = ap.parse_args()
    ref = load(a.ref, "gref"); d = json.load(open(a.sweep))
    sv = np.array([p["servo"] for p in d], float)
    L = np.array([p["L"]["t"] for p in d], float); R = np.array([p["R"]["t"] for p in d], float)
    trust = sv <= 120
    rock = {}
    for side, P, fn in (("L", L, lambda s: ref.rocker_deg(s)[0]), ("R", R, lambda s: ref.rocker_deg(s)[1])):
        cy, cz, rad = circle(P)
        phi = np.degrees(np.unwrap(np.arctan2(P[:, 2] - cz, P[:, 1] - cy)))
        best = min(((sg, np.mean(fn(sv[trust]) - sg*phi[trust])) for sg in (1, -1)),
                   key=lambda t: np.abs(fn(sv[trust]) - (t[1] + t[0]*phi[trust])).max())
        rock[side] = best[1] + best[0]*phi
        print(f"  로커{side}  반경 {rad:.3f} mm  198°→{rock[side][-1]:+.2f}°  "
              f"(구판 대비 {np.abs(fn(sv[trust]) - rock[side][trust]).max():.2f}° @10~120)")
    opening = R[:, 1] - L[:, 1]
    print(f"  개구   10°→{opening[0]:.2f}  198°→{opening[-1]:.2f} mm")

    fmt = lambda arr: "[" + ", ".join(f"{v:.4f}" for v in arr) + "]"
    a.out.write_text(f'''"""그리퍼 서보각 -> URDF 관절값 변환.  (실측 95점 보간표)

퓨전에서 서보 조인트를 10~198° 로 훑어 실측한 95점(gripper_sweep.json)을
**그대로 보간표**로 쓴다. scripts/17_fit_gripper_map.py 가 만든다.

★ 다항식이 아니다. 2026-09-03 전달본의 7차 다항식은 최고차항이 반올림돼
  서보 100° 를 넘으면 발산했다 (198° 에서 개구 -1092 mm). 보간표는 발산할
  것이 없다. 원본은 gripper_map_delivered_2026-09-03.py 로 남겨 뒀다.

평행4절 — 조는 회전 없이 반경 40.000 mm 원호로 평행이동한다.
개구 0.3 ~ 82.52 mm.  품목별 필요 서보각:
    빨대 6mm→46° / 페트병목 25→74 / 뚜껑 30→82 / 나무토막 40→94
    페트병몸통 65→134 / 알루미늄캔 66→136 / 종이컵 80→174
좌우가 대칭이 아니다 (구동로드가 서보 혼에 181.32° 차이). 표를 좌우 따로 둔다.
"""
import numpy as np

SERVO_RANGE_DEG = ({sv[0]:.1f}, {sv[-1]:.1f})
ROCKER_RADIUS_MM = 40.00
OPENING_RANGE_MM = ({opening.min():.2f}, {opening.max():.2f})
ROCKER_L_ZERO_DEG, ROCKER_R_ZERO_DEG = {ROCKER_L_ZERO_DEG}, {ROCKER_R_ZERO_DEG}   # CAD 기준자세
# URDF 리밋 (기구 담당 2026-09-03 판). joint_rad 는 이 안으로 자른다.
GRIPPER_JOINT_LIM_DEG = (-15.30, 47.50)
ROCKER_R_JOINT_LIM_DEG = (-54.57, 7.93)

_SERVO   = np.array({fmt(sv)})
_ROCK_L  = np.array({fmt(rock["L"])})
_ROCK_R  = np.array({fmt(rock["R"])})
_OPENING = np.array({fmt(opening)})
_OPEN_MONO = np.maximum.accumulate(_OPENING)      # 역변환용 단조 포락선


def rocker_deg(servo_deg):
    """서보각(deg) -> (좌 로커각, 우 로커각) deg."""
    s = np.clip(np.asarray(servo_deg, float), *SERVO_RANGE_DEG)
    return np.interp(s, _SERVO, _ROCK_L), np.interp(s, _SERVO, _ROCK_R)


def joint_rad(servo_deg):
    """서보각(deg) -> (gripper_joint, rocker_r_joint) rad.  URDF 에 그대로 넣는 값."""
    l, r = rocker_deg(servo_deg)
    gj = np.clip(ROCKER_L_ZERO_DEG - l, *GRIPPER_JOINT_LIM_DEG)
    rj = np.clip(r - ROCKER_R_ZERO_DEG, *ROCKER_R_JOINT_LIM_DEG)
    return np.radians(gj), np.radians(rj)


def opening_mm(servo_deg):
    """서보각(deg) -> 조 개구(mm)."""
    s = np.clip(np.asarray(servo_deg, float), *SERVO_RANGE_DEG)
    return np.interp(s, _SERVO, _OPENING)


def servo_for_opening(mm):
    """개구(mm) -> 서보각(deg). 단조 포락선으로 역변환."""
    return np.interp(np.asarray(mm, float), _OPEN_MONO, _SERVO)
''')
    print(f"  → {a.out}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
