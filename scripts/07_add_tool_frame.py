#!/usr/bin/env python
"""URDF 말단에 EE(툴) 프레임을 추가한다.

CAD 에서 뽑은 URDF 에는 보통 엔드이펙터 프레임이 없다. 마지막 링크의 원점은 대개
손목 관절 자리이지 손이 붙는 플랜지 면이 아니라서, 그대로 IK 목표로 쓰면 손 길이만큼
어긋난다. 여기서 말단 링크의 메시 형상을 실측해 플랜지 위치에 프레임을 얹어준다.

CAD 를 다시 뽑아 URDF 를 재생성해도 이 스크립트만 다시 돌리면 된다.

사용법:
    # 자동 — 말단 링크 메시가 가장 멀리 뻗은 축의 끝에 프레임을 놓는다
    .venv/bin/python scripts/07_add_tool_frame.py --urdf assets/robot_arm/robot_arm.urdf

    # 수동 — 부모 링크와 오프셋을 직접 지정
    .venv/bin/python scripts/07_add_tool_frame.py --urdf my.urdf \
        --parent link5 --offset 0 0 0.1885

    # 측정만 하고 쓰지 않기
    .venv/bin/python scripts/07_add_tool_frame.py --urdf my.urdf --dry-run
"""

from __future__ import annotations

import argparse
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def read_stl(path: Path) -> np.ndarray:
    """STL(바이너리/ASCII) → (N,3) 정점 배열."""
    data = path.read_bytes()
    # 바이너리 STL 은 헤더 80 + 개수 4 + 삼각형당 50 바이트. 크기로 판별하는 게 확실하다.
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if len(data) == 84 + n * 50 and n > 0:
            arr = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
            tri = arr[:, 12:48].copy().view("<f4").reshape(n, 9)
            return tri.reshape(-1, 3).astype(float)
    verts = []
    for line in data.decode("utf-8", "ignore").splitlines():
        tok = line.split()
        if len(tok) >= 4 and tok[0] == "vertex":
            verts.append([float(x) for x in tok[1:4]])
    return np.array(verts, dtype=float) if verts else np.zeros((0, 3))


def rpy_to_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def link_mesh_points(root: ET.Element, urdf_dir: Path, link_name: str) -> np.ndarray:
    """링크의 visual 메시 정점들을 링크 로컬 좌표로 모은다."""
    link = next((ln for ln in root.findall("link") if ln.get("name") == link_name), None)
    if link is None:
        return np.zeros((0, 3))

    chunks = []
    for vis in link.findall("visual"):
        mesh = vis.find("geometry/mesh")
        if mesh is None:
            continue
        f = (urdf_dir / mesh.get("filename")).resolve()
        if not f.exists():
            continue
        v = read_stl(f)
        v = v[np.isfinite(v).all(axis=1)]  # 깨진 삼각형이 bbox 를 오염시키지 않게
        if not len(v):
            continue
        if (scale := mesh.get("scale")):
            v = v * np.array([float(x) for x in scale.split()])
        origin = vis.find("origin")
        if origin is not None:
            rot = rpy_to_matrix([float(x) for x in (origin.get("rpy") or "0 0 0").split()])
            tr = np.array([float(x) for x in (origin.get("xyz") or "0 0 0").split()])
            # numpy 2.x + Apple Accelerate BLAS 조합에서 결과가 전부 유한한데도
            # matmul 중 FP 예외 플래그가 올라와 허위 경고가 뜬다 (입력/출력 모두
            # 유한함을 확인). 진짜 문제는 위에서 이미 걸렀으므로 여기서는 무시한다.
            with np.errstate(all="ignore"):
                v = v @ rot.T + tr
        chunks.append(v)
    return np.vstack(chunks) if chunks else np.zeros((0, 3))


def find_terminal_link(root: ET.Element) -> str:
    """구동 관절 체인의 마지막 링크. fixed 로 매달린 부속(카메라 브래킷 등)은 건너뛴다."""
    movable = [j for j in root.findall("joint") if j.get("type") not in ("fixed",)]
    if not movable:
        raise ValueError("구동 관절이 없습니다.")
    # 마지막 구동 관절의 자식 링크
    return movable[-1].find("child").get("link")


def auto_offset(points: np.ndarray) -> tuple[np.ndarray, str]:
    """메시가 원점에서 가장 멀리 뻗은 축의 끝을 툴 위치로 본다."""
    lo, hi = points.min(0), points.max(0)
    # 각 축에서 원점으로부터의 최대 거리 (양/음 방향 모두 고려)
    extents = np.where(np.abs(hi) >= np.abs(lo), hi, lo)
    axis = int(np.argmax(np.abs(extents)))
    offset = np.zeros(3)
    offset[axis] = extents[axis]
    return offset, "xyz"[axis]


def make_frame(root: ET.Element, name: str, parent: str, offset: np.ndarray, rpy: str) -> None:
    link = ET.SubElement(root, "link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "1e-6"})
    ET.SubElement(inertial, "inertia",
                  {"ixx": "1e-9", "ixy": "0", "ixz": "0",
                   "iyy": "1e-9", "iyz": "0", "izz": "1e-9"})

    joint = ET.SubElement(root, "joint", {"name": f"{name}_joint", "type": "fixed"})
    ET.SubElement(joint, "origin",
                  {"xyz": " ".join(f"{v:.6g}" for v in offset), "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": name})


def main() -> int:
    ap = argparse.ArgumentParser(description="URDF 에 EE(툴) 프레임 추가")
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--parent", type=str, default=None,
                    help="프레임을 붙일 링크 (기본: 마지막 구동 관절의 자식 링크)")
    ap.add_argument("--offset", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="부모 링크 원점 기준 오프셋 (m). 기본: 메시 실측으로 자동")
    ap.add_argument("--rpy", type=str, default="0 0 0", help="프레임 자세 (rad)")
    ap.add_argument("--name", type=str, default="ee_link")
    ap.add_argument("--hand-mount", action="store_true", default=True,
                    help="hand_mount_link 도 같이 추가 (기본 켜짐)")
    ap.add_argument("--hand-offset", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                    help="hand_mount_link 오프셋 (기본: ee 와 동일)")
    ap.add_argument("--out", type=Path, default=None, help="출력 경로 (기본: 원본 덮어쓰기 + .bak)")
    ap.add_argument("--dry-run", action="store_true", help="측정 결과만 보고 쓰지 않음")
    args = ap.parse_args()

    if not args.urdf.exists():
        print(f"❌ URDF 없음: {args.urdf}", file=sys.stderr)
        return 1

    tree = ET.parse(args.urdf)
    root = tree.getroot()
    urdf_dir = args.urdf.resolve().parent

    parent = args.parent or find_terminal_link(root)
    existing = {ln.get("name") for ln in root.findall("link")}
    if parent not in existing:
        print(f"❌ 부모 링크 '{parent}' 이 URDF 에 없습니다. 후보: {sorted(existing)}", file=sys.stderr)
        return 1

    print("=" * 70)
    print(f"  EE 프레임 추가 — {args.urdf.name}")
    print("=" * 70)
    print(f"  말단 링크 : {parent}")

    if args.offset is not None:
        offset = np.array(args.offset, dtype=float)
        print(f"  오프셋    : 수동 지정 [{offset[0]:+.4f} {offset[1]:+.4f} {offset[2]:+.4f}]")
    else:
        pts = link_mesh_points(root, urdf_dir, parent)
        if not len(pts):
            print(f"❌ '{parent}' 의 메시를 읽지 못해 자동 측정이 불가합니다. "
                  f"--offset 으로 직접 지정하세요.", file=sys.stderr)
            return 1
        lo, hi = pts.min(0), pts.max(0)
        print(f"  메시 범위 : x[{lo[0]:+.4f},{hi[0]:+.4f}] "
              f"y[{lo[1]:+.4f},{hi[1]:+.4f}] z[{lo[2]:+.4f},{hi[2]:+.4f}]  ({len(pts):,} 정점)")
        offset, axis = auto_offset(pts)
        print(f"  오프셋    : 자동 — {axis} 축으로 가장 멀리 뻗음 "
              f"[{offset[0]:+.4f} {offset[1]:+.4f} {offset[2]:+.4f}]")

    if args.dry_run:
        print("\n  (--dry-run 이므로 파일을 쓰지 않습니다)")
        print("=" * 70)
        return 0

    # 이미 있으면 지우고 다시 만든다 (재실행 가능하게)
    for name in (args.name, "hand_mount_link"):
        for el in [e for e in root.findall("link") if e.get("name") == name]:
            root.remove(el)
        for el in [e for e in root.findall("joint") if e.get("name") == f"{name}_joint"]:
            root.remove(el)

    make_frame(root, args.name, parent, offset, args.rpy)
    added = [args.name]
    if args.hand_mount:
        hand_off = np.array(args.hand_offset, dtype=float) if args.hand_offset is not None else offset
        make_frame(root, "hand_mount_link", parent, hand_off, args.rpy)
        added.append("hand_mount_link")

    ET.indent(root, "  ")
    out = args.out or args.urdf
    if out == args.urdf:
        backup = args.urdf.with_suffix(args.urdf.suffix + ".bak")
        if not backup.exists():
            shutil.copy2(args.urdf, backup)
            print(f"  백업      : {backup.name}")
    tree.write(out, encoding="utf-8", xml_declaration=True)

    print(f"  추가한 프레임: {', '.join(added)}  (부모 {parent}, fixed)")
    print(f"  💾 저장: {out}")
    print("=" * 70)
    print("  다음: .venv/bin/python scripts/06_setup_urdf.py --urdf "
          f"{out.relative_to(ROOT) if out.is_relative_to(ROOT) else out}")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
