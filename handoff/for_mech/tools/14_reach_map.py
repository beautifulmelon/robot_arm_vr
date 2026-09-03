#!/usr/bin/env python3
"""신규 5축 팔이 본체에 붙었을 때 어디까지 닿는가 — 바닥 집기 / 쓰레기통 넣기.

기존 문서의 바닥 작업영역(x -120.5~750.7mm)은 **옛날 팔** 기준이다. 신규 팔은
리치가 다르므로 그대로 쓰면 안 된다. 이 스크립트로 다시 잰다.

이 팔은 닫힌 형태로 풀린다. joint2/3/4 축이 전부 Y 이고 링크 오프셋이 전부 +Z
이라 수직 평면 2R+1R 이고, joint1 이 그 평면을 돌린다. joint5 는 툴 축 자체의
롤이라 **위치에 전혀 영향이 없다**. 그래서 무작위 표본이 아니라 격자로 전수
조사할 수 있다.

    z_arm = L01 + a2·cos(θ2) + a3·cos(θ23) + a4·cos(θ234)
    r     =       a2·sin(θ2) + a3·sin(θ23) + a4·sin(θ234)
    툴 +Z (접근 방향) = ( sinθ234·cos q1, sinθ234·sin q1, cosθ234 )

★ 가정 : base_link 원점이 본체 arm_mount (314.375, 0, 184.46) mm 에 rpy 0 으로
  붙는다. 신규 팔의 실제 장착 인터페이스는 아직 확정 전이므로 기구 담당 확인 필요.
★ 자기 충돌·본체 충돌은 보지 않는다. 여기서 나오는 영역은 **상한**이다.
"""
from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MOUNT = np.array([0.314375, -0.000015, 0.18446])   # body_base → arm_mount


def link_lengths(urdf: Path) -> dict:
    """URDF 에서 링크 길이를 읽는다. 손으로 옮겨적지 않는다."""
    r = ET.parse(urdf).getroot()
    off = {}
    for j in r.findall("joint"):
        o = j.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
        off[j.get("name")] = xyz
    return {
        "L01": off["joint1"][2] + off["joint2"][2],   # base → joint2 (어깨)
        "a2": off["joint3"][2],
        "a3": off["joint4"][2],
        "a4": off["tool_joint"][2],                    # joint5 는 오프셋 0
    }


def limits(cfg: Path) -> tuple[np.ndarray, np.ndarray]:
    d = json.loads(cfg.read_text())
    return np.array(d["lower"]), np.array(d["upper"])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", type=Path, default=ROOT / "assets/arm_v2/arm_v2.urdf")
    ap.add_argument("--config", type=Path, default=ROOT / "config/arm_v2.json")
    ap.add_argument("--step", type=float, default=2.0, help="관절 격자 간격 (deg)")
    ap.add_argument("--down-tol", type=float, default=30.0,
                    help="'아래를 본다' 허용 각 (deg). 툴 +Z 와 -Z_world 사이 각")
    args = ap.parse_args()

    L = link_lengths(args.urdf)
    lo, hi = limits(args.config)
    s = np.radians(args.step)

    g2 = np.arange(lo[1], hi[1] + 1e-9, s)
    g3 = np.arange(lo[2], hi[2] + 1e-9, s)
    g4 = np.arange(lo[3], hi[3] + 1e-9, s)
    t2, t3, t4 = np.meshgrid(g2, g3, g4, indexing="ij")
    th2 = t2.ravel(); th23 = (t2 + t3).ravel(); th234 = (t2 + t3 + t4).ravel()

    z = L["L01"] + L["a2"]*np.cos(th2) + L["a3"]*np.cos(th23) + L["a4"]*np.cos(th234)
    r = L["a2"]*np.sin(th2) + L["a3"]*np.sin(th23) + L["a4"]*np.sin(th234)
    down = np.cos(th234)                       # 툴 +Z 의 world Z 성분
    z_body = z + MOUNT[2]

    print("=" * 74)
    print(f"  신규 팔 도달 범위 — {args.urdf.name}")
    print(f"  링크 : 어깨높이 {L['L01']*1000:.1f}  a2 {L['a2']*1000:.0f}  "
          f"a3 {L['a3']*1000:.0f}  툴 {L['a4']*1000:.1f} mm")
    print(f"  격자 : {args.step}° → 자세 {th2.size:,}개 (joint5 는 위치에 무관)")
    print(f"  장착 : 본체 ({MOUNT[0]*1000:.1f}, {MOUNT[1]*1000:.1f}, {MOUNT[2]*1000:.1f}) mm  rpy 0")
    print("=" * 74)

    rmax = float(np.max(np.abs(r)))
    print(f"\n[전체]  반경 최대 {rmax*1000:.1f} mm   "
          f"툴 높이 본체좌표 {z_body.min()*1000:+.1f} ~ {z_body.max()*1000:+.1f} mm")

    # ── 1. 바닥 집기 ────────────────────────────────────────────────
    cd = np.cos(np.radians(180.0 - args.down_tol))       # cos(150°) = -0.866
    for zt in (0.020, 0.035, 0.050, 0.080):
        m = (np.abs(z_body - zt) < 0.005) & (down < cd) & (r > 0)
        if not m.any():
            print(f"\n[바닥 z={zt*1000:.0f}mm]  ❌ 도달 불가")
            continue
        rr = r[m]
        print(f"\n[바닥 z={zt*1000:.0f}mm  툴 아래±{args.down_tol:.0f}°]  "
              f"반경 {rr.min()*1000:.1f} ~ {rr.max()*1000:.1f} mm  "
              f"→ 본체 x {(MOUNT[0]-rr.max())*1000:+.1f} ~ {(MOUNT[0]+rr.max())*1000:+.1f}")

    # ── 2. 쓰레기통 — 툴을 아래로 향한 채 얼마나 높이 갈 수 있나 ──────
    print("\n" + "-" * 74)
    print("  쓰레기통 개구부 후보 — 툴을 아래로 향한 채 도달 가능한 높이/반경")
    print("-" * 74)
    md = down < cd
    print(f"  툴 아래±{args.down_tol:.0f}° 로 도달 가능한 최대 높이 : "
          f"{z_body[md].max()*1000:+.1f} mm (본체 좌표)")
    for zb in (0.30, 0.40, 0.50, 0.60, 0.70, 0.80):
        m = md & (np.abs(z_body - zb) < 0.005) & (r > 0)
        if not m.any():
            print(f"    z {zb*1000:.0f} mm : ❌")
            continue
        rr = r[m]
        print(f"    z {zb*1000:.0f} mm : 반경 {rr.min()*1000:5.1f} ~ {rr.max()*1000:5.1f} mm"
              f"   본체 x {(MOUNT[0]-rr.max())*1000:+7.1f} ~ {(MOUNT[0]+rr.max())*1000:+7.1f}")

    print("\n  ※ 본체 윤곽 x -367 ~ +382 · y -243 ~ +243 · z 0 ~ 831 mm")
    print("  ※ 자기/본체 충돌 미고려 — 위 값은 상한이다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
