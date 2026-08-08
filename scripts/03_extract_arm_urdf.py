#!/usr/bin/env python
"""3단계: Atom01 전신 URDF에서 단일 팔(5-DOF)만 추출.

roboparty_xr_teleop 의 atom01.urdf 는 다리·허리를 포함한 전신 모델이라 그대로 쓰면
IK 솔버가 불필요한 관절까지 들고 있게 된다. (roboparty 본인들도 런타임에
buildReducedRobot 으로 13개 관절을 잠가서 쓴다.)

여기서는 아예 한쪽 팔만 남긴 독립 URDF 를 만들어 둔다. LeRobot 의 RobotKinematics
(placo) 는 URDF 파일 경로를 받으므로, 전용 URDF 가 있으면 그대로 물릴 수 있다.

추가로 하는 일:
  · torso_link → base_link 로 치환 (어깨 마운트 원점 유지)
  · EE 프레임 추가 — roboparty 가 코드에서 수동으로 붙이던 L_ee/R_ee 를 URDF 에 명시
  · Amazing Hand 마운트 프레임 추가 (커넥터 오프셋은 인자로 조정)
  · 메시 복사

사용법:
    .venv/bin/python scripts/03_extract_arm_urdf.py --side right
    .venv/bin/python scripts/03_extract_arm_urdf.py --side right --ee-offset 0.15
"""

from __future__ import annotations

import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SRC = ROOT / "assets" / "atom01_src" / "urdf" / "atom01.urdf"

# 어깨부터 손목까지 순서대로. roboparty 의 모터 인덱스 0..4 와 대응한다.
CHAIN = ["arm_pitch", "arm_roll", "arm_yaw", "elbow_pitch", "elbow_yaw"]


def indent(elem: ET.Element, level: int = 0) -> None:
    """ET.indent 는 3.9+ 지원이지만 속성 순서 유지를 위해 직접 정렬한다."""
    pad = "\n" + "  " * level
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + "  "
        for child in elem:
            indent(child, level + 1)
        if not (elem.tail or "").strip():
            elem.tail = pad
    elif level and not (elem.tail or "").strip():
        elem.tail = pad


def make_link(name: str, mass: float = 1e-4) -> ET.Element:
    """관성만 최소로 가진 빈 링크 (프레임 표식용)."""
    link = ET.Element("link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"{mass}"})
    ET.SubElement(
        inertial, "inertia",
        {"ixx": "1e-6", "ixy": "0", "ixz": "0", "iyy": "1e-6", "iyz": "0", "izz": "1e-6"},
    )
    return link


def make_fixed_joint(name: str, parent: str, child: str, xyz: str, rpy: str = "0 0 0") -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def main() -> int:
    ap = argparse.ArgumentParser(description="Atom01 전신 URDF → 단일 팔 URDF 추출")
    ap.add_argument("--side", choices=["right", "left"], default="right")
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC, help="원본 atom01.urdf")
    ap.add_argument("--out-dir", type=Path, default=None, help="출력 디렉토리 (기본 assets/rpo_arm)")
    ap.add_argument("--ee-offset", type=float, default=0.15,
                    help="elbow_yaw 관절에서 EE 프레임까지 +x 오프셋 (m). "
                         "roboparty robot_arm_ik.py 의 L_ee/R_ee 값이 0.15")
    ap.add_argument("--hand-offset", type=float, default=None,
                    help="Amazing Hand 마운트 프레임 오프셋 (m). 미지정 시 ee-offset 과 동일")
    args = ap.parse_args()

    side = args.side
    out_dir = args.out_dir or (ROOT / "assets" / "rpo_arm")
    hand_offset = args.hand_offset if args.hand_offset is not None else args.ee_offset

    if not args.src.exists():
        print(f"❌ 원본 URDF 없음: {args.src}", file=sys.stderr)
        print("   assets/atom01_src/ 에 roboparty_xr_teleop 의 Atom01_urdf 를 먼저 복사하세요.", file=sys.stderr)
        return 1

    tree = ET.parse(args.src)
    src_root = tree.getroot()

    joints = {j.get("name"): j for j in src_root.findall("joint")}
    links = {ln.get("name"): ln for ln in src_root.findall("link")}

    chain_joints = [f"{side}_{n}_joint" for n in CHAIN]
    chain_links = [f"{side}_{n}_link" for n in CHAIN]

    missing = [n for n in chain_joints + chain_links if n not in joints and n not in links]
    if missing:
        print(f"❌ 원본에 없는 요소: {missing}", file=sys.stderr)
        return 1

    # ── 새 URDF 구성 ──────────────────────────────────────────────────
    new_root = ET.Element("robot", {"name": f"rpo_arm_{side}"})
    new_root.append(ET.Comment(
        f" Atom01 전신 URDF 에서 {side} 팔 5-DOF 만 추출 (scripts/03_extract_arm_urdf.py 생성). "
        f"원본: Roboparty/roboparty_xr_teleop assets/Atom01_urdf "
    ))

    # base_link: 원본의 torso_link 자리. 팔이 붙는 마운트 원점이며 어깨 오프셋을
    # 그대로 보존하므로, 나중에 거치대에 올릴 때 좌표 기준으로 쓸 수 있다.
    new_root.append(make_link("base_link", mass=0.1))

    total_mass = 0.0
    for jname, lname in zip(chain_joints, chain_links, strict=True):
        joint = joints[jname]
        # 첫 관절의 부모(torso_link)를 base_link 로 치환
        parent = joint.find("parent")
        if parent.get("link") == "torso_link":
            parent.set("link", "base_link")
        new_root.append(joint)
        new_root.append(links[lname])
        m = links[lname].find("inertial/mass")
        if m is not None:
            total_mass += float(m.get("value"))

    wrist_link = chain_links[-1]  # *_elbow_yaw_link

    # EE 프레임 — roboparty 가 pin.Frame 으로 런타임에 붙이던 것을 URDF 에 명시.
    # elbow_yaw 축이 x 축이라 EE 를 +x 에 두면 이 관절은 EE '위치'를 바꾸지 않고
    # '자세'만 바꾼다. roboparty 가 command_q[4]=0 으로 이 축을 죽여도 위치 추종이
    # 되던 이유가 이것이다. 우리는 Amazing Hand 자세를 위해 이 축을 살려서 쓴다.
    new_root.append(make_link("ee_link"))
    new_root.append(make_fixed_joint("ee_joint", wrist_link, "ee_link", f"{args.ee_offset} 0 0"))

    # Amazing Hand 마운트 — 실제 커넥터 치수가 확정되면 --hand-offset 으로 조정
    new_root.append(make_link("hand_mount_link"))
    new_root.append(make_fixed_joint("hand_mount_joint", wrist_link, "hand_mount_link",
                                     f"{hand_offset} 0 0"))

    # ── 메시 복사 + 경로 재작성 ────────────────────────────────────────
    src_mesh_dir = args.src.parent.parent / "meshes"
    out_urdf_dir = out_dir / "urdf"
    out_mesh_dir = out_dir / "meshes"
    out_urdf_dir.mkdir(parents=True, exist_ok=True)
    out_mesh_dir.mkdir(parents=True, exist_ok=True)

    copied = []
    for mesh in new_root.iter("mesh"):
        fname = Path(mesh.get("filename")).name
        src_file = src_mesh_dir / fname
        if src_file.exists():
            shutil.copy2(src_file, out_mesh_dir / fname)
            copied.append(fname)
        mesh.set("filename", f"../meshes/{fname}")

    indent(new_root)
    out_urdf = out_urdf_dir / f"rpo_arm_{side}.urdf"
    ET.ElementTree(new_root).write(out_urdf, encoding="utf-8", xml_declaration=True)

    # ── 요약 ──────────────────────────────────────────────────────────
    print("=" * 70)
    print(f"  단일 팔 URDF 추출 완료 — {side}")
    print("=" * 70)
    print(f"  출력   : {out_urdf.relative_to(ROOT)}")
    print(f"  메시   : {len(copied)} 개 → {out_mesh_dir.relative_to(ROOT)}")
    print(f"  질량   : {total_mass:.3f} kg (팔 링크 합계)")
    print(f"  관절   : {len(chain_joints)} 개")
    for i, jn in enumerate(chain_joints):
        lim = joints[jn].find("limit")
        ax = joints[jn].find("axis").get("xyz")
        print(f"           [{i}] {jn:26s} axis={ax:9s} "
              f"[{float(lim.get('lower')):+.2f}, {float(lim.get('upper')):+.2f}] rad  "
              f"τ={lim.get('effort')} N·m")
    print(f"  프레임 : ee_link (elbow_yaw +{args.ee_offset} m x)")
    print(f"           hand_mount_link (elbow_yaw +{hand_offset} m x)")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
