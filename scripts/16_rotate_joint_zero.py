#!/usr/bin/env python3
"""관절의 **영점(0 위치)** 을 축 둘레로 돌린다. 로봇 형상은 하나도 안 바뀐다.

왜 필요한가 — joint1 이 홈에서 −172° 라 리밋(±178)까지 6° 밖에 안 남았다.
한쪽으로 조금만 요(yaw)하면 벽에 닿는다. 영점을 180° 옮기면 같은 자세가
+8° 가 되어 양쪽에 170° / 186° 가 생긴다. 못 쓰는 4° 구간은 팔이 차체 뒤를
가리키는 쪽(옛 0° 근처)으로 밀려나는데, 거기는 어차피 본체와 부딪히는 자리다.

수식 — 자식 링크 pose = T_origin · Rot(axis, q) 이므로

    T_origin_new = T_origin_old · Rot(axis, offset)     →     q_new = q_old − offset

즉 origin 회전에 offset 을 곱해 넣고, 쓰던 관절값에서 offset 을 빼면 **완전히
같은 물리 자세**가 된다. 아래 --verify 가 그걸 실제로 대조한다.

★★ 실물도 같이 바꿔야 한다. 시뮬만 바꾸면 첫 지령에 팔이 반 바퀴 돈다.
"""
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    """URDF rpy(고정축 XYZ) → 회전행렬. R = Rz(y)·Ry(p)·Rx(r)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def R_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    p = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if abs(np.cos(p)) < 1e-9:                      # 짐벌락 — 이 URDF 들에는 안 생긴다
        return float(np.arctan2(-R[1, 2], R[1, 1])), float(p), 0.0
    return (float(np.arctan2(R[2, 1], R[2, 2])), float(p), float(np.arctan2(R[1, 0], R[0, 0])))


def axis_angle_to_R(axis: np.ndarray, ang: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", type=Path, required=True, nargs="+")
    ap.add_argument("--joint", required=True, help="영점을 돌릴 관절 이름")
    ap.add_argument("--deg", type=float, required=True,
                    help="영점 회전량. q_new = q_old − deg 가 된다")
    args = ap.parse_args()

    off = np.radians(args.deg)
    for path in args.urdf:
        tree = ET.parse(path)
        root = tree.getroot()
        j = next((x for x in root.findall("joint") if x.get("name") == args.joint), None)
        if j is None:
            print(f"  건너뜀 — {path.name} 에 {args.joint} 없음")
            continue

        ax_el = j.find("axis")
        axis = np.array([float(v) for v in (ax_el.get("xyz") if ax_el is not None
                                            else "0 0 1").split()])
        o = j.find("origin")
        if o is None:
            o = ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            j.insert(0, o)
        rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]

        R_new = rpy_to_R(*rpy) @ axis_angle_to_R(axis, off)
        r, p, y = R_to_rpy(R_new)
        o.set("rpy", f"{r:.12g} {p:.12g} {y:.12g}")

        path.write_text(ET.tostring(root, encoding="unicode"))
        print(f"  {path.name:28s} {args.joint} origin rpy "
              f"[{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}] → [{r:.4f} {p:.4f} {y:.4f}]"
              f"   축 {axis.astype(int).tolist()}")

    print(f"\n  ★ 쓰던 관절값에서 {args.deg:g}° 를 빼세요 (mod 360). "
          f"config 의 home_q 도 같이 고쳐야 합니다.")
    print("  ★★ 실물 영점도 같이 바꾸지 않으면 첫 지령에 팔이 그만큼 돕니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
