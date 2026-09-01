#!/usr/bin/env python
"""IK 전용 URDF 를 만든다 — 구동 관절만 남기고 나머지는 fixed 로 굳힌다.

    .venv/bin/python scripts/11_make_ik_urdf.py \
        --in assets/arm_v1/arm_v1.urdf --out assets/arm_v1/arm_v1_ik.urdf \
        --keep joint1 joint2 joint3 joint4 joint5

왜 필요한가
    placo 는 URDF 의 <mimic> 태그를 **무시한다.** 그래서 신규 팔의 arm_v1.urdf 를
    그대로 넣으면 종속 관절(jaw_l/jaw_r/rocker_r)까지 구동 관절 9개로 세고,
    IK 가 실물에 없는 자유도를 마음대로 쓴다.

    기존 팔에서 손(AmazingHand)을 URDF 에서 떼어내고 서보각으로 따로 몰았던 것과
    같은 처리다. 그리퍼는 IK 대상이 아니라 **스칼라 하나(개구)** 로 몰면 된다.

    ★ 형상은 그대로 남는다. 관절만 fixed 로 굳히므로 화면에는 그리퍼가 계속
      보이고, 링크 pose 계산도 정상이다.
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description="구동 관절만 남긴 IK 전용 URDF 생성")
    ap.add_argument("--in", dest="src", type=Path, required=True)
    ap.add_argument("--out", dest="dst", type=Path, required=True)
    ap.add_argument("--keep", nargs="+", required=True, help="구동으로 남길 관절 이름")
    ap.add_argument("--name", default=None, help="robot name 태그를 바꾼다")
    args = ap.parse_args()

    tree = ET.parse(args.src)
    root = tree.getroot()
    if args.name:
        root.set("name", args.name)

    keep = set(args.keep)
    frozen = []
    for j in root.findall("joint"):
        if j.get("type") == "fixed" or j.get("name") in keep:
            continue
        frozen.append(j.get("name"))
        j.set("type", "fixed")
        # fixed 는 axis/limit/mimic/dynamics 를 안 쓴다. 남겨두면 파서가 헷갈린다.
        for tag in ("axis", "limit", "mimic", "dynamics", "safety_controller"):
            for el in j.findall(tag):
                j.remove(el)

    args.dst.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.dst, encoding="utf-8", xml_declaration=True)
    print(f"  {args.src.name} → {args.dst.name}")
    print(f"  구동으로 남김 ({len(keep)}): {', '.join(args.keep)}")
    print(f"  fixed 로 굳힘 ({len(frozen)}): {', '.join(frozen) or '없음'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
