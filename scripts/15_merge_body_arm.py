#!/usr/bin/env python3
"""본체(차체) + 신규 팔을 하나의 URDF 로 합친다.

Isaac 은 본체+팔이 한 몸인 USD 를 필요로 한다. 그런데 기존
`body_with_arm_nomimic.urdf` 는 **구형 팔 + AmazingHand** 라 신규 5축 팔에는
쓸 수 없다. 기구 담당에게 정식판을 받기 전까지 쓸 잠정판을 만든다.

★ 가정 — 팔 base_link 원점이 본체 `arm_mount` 프레임에 rpy 0 으로 붙는다.
  기구 담당 확인이 필요하다. 확정되면 이 스크립트의 MOUNT_RPY/XYZ 만 고치면 된다.

메시는 body_with_arm 규약대로 `assets/body/meshes/<태그>/` 로 복사하고
URDF 안의 경로를 그에 맞게 고쳐 쓴다 (Isaac 임포트가 상대경로를 따라간다).
"""
from __future__ import annotations

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MOUNT_XYZ = "0 0 0"      # arm_mount 프레임이 곧 장착면이라 추가 오프셋 없음
MOUNT_RPY = "0 0 0"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--body", type=Path, default=ROOT / "assets/body/body.urdf")
    ap.add_argument("--arm",  type=Path, default=ROOT / "assets/arm_v2/arm_v2.urdf")
    ap.add_argument("--tag",  default="arm_v2", help="메시를 넣을 하위 폴더 이름")
    ap.add_argument("--out",  type=Path, default=ROOT / "assets/body/body_with_arm_v2.urdf")
    args = ap.parse_args()

    body = ET.parse(args.body).getroot()
    arm  = ET.parse(args.arm).getroot()

    # ── 메시 복사 (body_with_arm 규약: meshes/<태그>/) ──────────────
    src = args.arm.parent / "meshes"
    dst = args.body.parent / "meshes" / args.tag
    dst.mkdir(parents=True, exist_ok=True)
    for m in sorted(src.glob("*.stl")):
        shutil.copy2(m, dst / m.name)

    # ── 팔 링크/조인트를 본체 트리에 얹는다 ─────────────────────────
    body_links = {l.get("name") for l in body.findall("link")}
    for el in list(arm):
        if el.tag not in ("link", "joint"):
            continue
        if el.tag == "link" and el.get("name") in body_links:
            raise SystemExit(f"링크 이름 충돌: {el.get('name')}")
        for mesh in el.iter("mesh"):
            fn = mesh.get("filename") or ""
            if fn.startswith("meshes/"):
                mesh.set("filename", f"meshes/{args.tag}/{Path(fn).name}")
        body.append(el)

    j = ET.SubElement(body, "joint", {"name": f"{args.tag}_attach_joint", "type": "fixed"})
    ET.SubElement(j, "origin", {"xyz": MOUNT_XYZ, "rpy": MOUNT_RPY})
    ET.SubElement(j, "parent", {"link": "arm_mount"})
    ET.SubElement(j, "child",  {"link": "base_link"})

    body.set("name", args.out.stem)
    ET.indent(body, "  ")
    args.out.write_text('<?xml version="1.0"?>\n' + ET.tostring(body, encoding="unicode") + "\n")

    n_l = len(body.findall("link"))
    n_j = len(body.findall("joint"))
    n_act = sum(1 for x in body.findall("joint") if x.get("type") != "fixed")
    print(f"  {args.out}")
    print(f"    링크 {n_l} · 조인트 {n_j} (구동 {n_act})   메시 → meshes/{args.tag}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
