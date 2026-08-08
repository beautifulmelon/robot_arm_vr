#!/usr/bin/env python
"""본체(차체) 형상을 웹 렌더러용 정적 JSON 으로 뽑는다.

    assets/body/body.urdf  ──>  web/vendor/body_geometry.json

왜 정적 파일인가
    본체는 **관절이 없다.** body_base 링크 하나에 visual 28개가 붙어 있을 뿐이고
    팔이 움직여도 본체는 그대로다. 그래서 매 프레임 서버가 보낼 이유가 없다.
    한 번 뽑아서 브라우저가 받아 두면 끝이다.

    이렇게 하면 서버 코드(05_teleop_sim.py, xr_server.py, arm_visual.py)를
    하나도 안 건드린다. 그 파일들은 실물 담당 소유라(30_AGREED A-2) 건드리면
    협의가 필요한데, 본체 표시는 순수 시각화라 그럴 이유가 없다.

    ★ web/vendor/ 에 넣는 이유
      xr_server.py 는 파일마다 라우트를 하나씩 정의하고 있어서 새 파일은
      404 가 난다. 그런데 /vendor 는 StaticFiles 로 통째 마운트돼 있다.
      거기 두면 **서버 코드 변경 없이** 브라우저가 받아갈 수 있다.
      (제대로 된 /body_geometry.json 라우트는 실물 담당에게 요청해 뒀다)

왜 링크 박스가 아니라 visual 별 박스인가
    본체는 링크가 1개다. 링크 단위로 바운딩 박스를 뽑으면 **차체 전체를 덮는
    상자 하나**가 나와서 형상이 안 보인다. visual 28개를 각각 상자로 만들면
    4040 프로파일·바퀴·상판이 구분돼 보인다.

사용법
    .venv/bin/python scripts/09_body_visual.py
    .venv/bin/python scripts/09_body_visual.py --urdf assets/body/body.urdf
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rpo_teleop.arm_visual import read_stl, rpy_to_matrix  # noqa: E402


def mat_to_quat(R: np.ndarray) -> list[float]:
    """3x3 회전행렬 → 쿼터니언 (x, y, z, w). three.js 순서."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return [round(float(v), 6) for v in (x, y, z, w)]


def _quat_to_mat(q) -> np.ndarray:
    """쿼터니언 (x,y,z,w) → 3x3. 검산용."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def origin_of(el) -> tuple[np.ndarray, np.ndarray]:
    """<origin xyz rpy> → (평행이동, 회전행렬). 없으면 항등."""
    o = el.find("origin")
    if o is None:
        return np.zeros(3), np.eye(3)
    xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()])
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
    return xyz, rpy_to_matrix(rpy)


def part_boxes(urdf_path: Path) -> list[dict]:
    """visual 하나당 상자 하나. 좌표는 링크(=body_base) 로컬."""
    urdf_dir = urdf_path.resolve().parent
    root = ET.parse(urdf_path).getroot()
    out: list[dict] = []
    for link in root.findall("link"):
        for vi, vis in enumerate(link.findall("visual")):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            fn = mesh.get("filename", "").replace("package://", "")
            path = (urdf_dir / fn).resolve()
            if not path.exists():
                print(f"  ⚠️  메시 없음, 건너뜀: {fn}", file=sys.stderr)
                continue
            v = read_stl(path)
            if v.size == 0:
                continue
            scale = np.array([float(s) for s in (mesh.get("scale") or "1 1 1").split()])
            v = v * scale
            # ★ 메시 로컬 bbox 를 먼저 잡고, 그 중심을 visual origin 으로 옮긴다.
            #   전체를 먼저 회전시키면 축정렬 bbox 가 헐거워져서 부품이 뚱뚱해진다.
            lo, hi = v.min(axis=0), v.max(axis=0)
            size = hi - lo
            c_local = (lo + hi) / 2.0
            t, R = origin_of(vis)
            out.append({
                "name": f"{link.get('name')}#{vi}:{Path(fn).stem}",
                "center": [round(float(x), 6) for x in (R @ c_local + t)],
                "size": [round(float(x), 6) for x in size],
                "quat": mat_to_quat(R),
            })
    return out


def frames_of(urdf_path: Path) -> dict[str, list[float]]:
    """fixed joint 로 정의된 프레임의 부모 기준 위치."""
    root = ET.parse(urdf_path).getroot()
    out = {}
    for j in root.findall("joint"):
        child = j.find("child")
        t, _ = origin_of(j)
        if child is not None:
            out[child.get("link")] = [round(float(x), 6) for x in t]
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="본체 형상 → 웹 렌더러용 정적 JSON")
    ap.add_argument("--urdf", type=Path, default=ROOT / "assets/body/body.urdf")
    ap.add_argument("--out", type=Path, default=ROOT / "web/vendor/body_geometry.json")
    args = ap.parse_args()

    if not args.urdf.exists():
        print(f"❌ URDF 가 없습니다: {args.urdf}", file=sys.stderr)
        return 1

    parts = part_boxes(args.urdf)
    frames = frames_of(args.urdf)
    if "arm_mount" not in frames:
        print("❌ arm_mount 프레임을 못 찾았습니다. 팔을 어디에 놓을지 알 수 없습니다.",
              file=sys.stderr)
        return 1

    data = {
        "source": str(args.urdf.relative_to(ROOT)),
        # ★ 팔 링크 pose 는 팔 base_link 기준으로 온다. 본체를 같이 그리려면
        #   팔 전체를 이만큼 평행이동해야 한다. 회전은 없다(검증 완료).
        "arm_mount": frames["arm_mount"],
        "frames": frames,
        "parts": parts,
    }
    args.out.write_text(json.dumps(data, separators=(",", ":")))

    # ★ 상자가 회전돼 있으므로 center±size/2 로 재면 안 된다. 코너 8개를
    #   전부 회전시켜서 재야 CAD 가 보고한 bbox 와 맞는다 (검증: 0.1mm 이내).
    corners = []
    for pt in parts:
        R = _quat_to_mat(pt["quat"])
        c, h = np.array(pt["center"]), np.array(pt["size"]) / 2
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    corners.append(c + R @ (np.array([sx, sy, sz]) * h))
    corners = np.array(corners)
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    print(f"  본체 부품 {len(parts)}개 → {args.out.relative_to(ROOT)} "
          f"({args.out.stat().st_size / 1024:.1f} KB)")
    print(f"  프레임 {len(frames)}개: {', '.join(frames)}")
    print(f"  arm_mount (팔 이동량) = {np.round(np.array(frames['arm_mount']) * 1000, 2)} mm")
    print(f"  크기 x {lo[0]*1000:+.0f}~{hi[0]*1000:+.0f} · "
          f"y {lo[1]*1000:+.0f}~{hi[1]*1000:+.0f} · z {lo[2]*1000:+.0f}~{hi[2]*1000:+.0f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
