#!/usr/bin/env python
"""등록된 팔들의 형상을 웹 렌더러용 정적 JSON 으로 뽑는다 (미리보기용).

    .venv/bin/python scripts/12_arm_preview.py
    → web/vendor/arms.json

무엇에 쓰나
    대시보드에서 **지금 서버가 돌리지 않는 팔도 골라 볼 수 있게** 한다.
    라이브 팔은 서버가 매 프레임 geometry/links 를 보내주므로 그대로 쓰고,
    나머지 팔은 여기서 뽑은 **홈 자세 정지 화면**을 보여준다.

    ★ 미리보기는 정지 화면이다. 조종되는 팔은 서버가 --config 로 정한 하나뿐이고,
      그건 실행할 때 정해진다. 브라우저에서 바꿀 수 있는 값이 아니다.
      (IK·안전계층이 그 팔로 만들어져 있다)

왜 정적 JSON 인가
    홈 자세는 안 변한다. 서버 코드(실물 담당 소유)를 안 건드리려고
    /vendor 아래 정적 파일로 둔다. body_geometry.json 과 같은 방식이다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rpo_teleop.arm_config import ArmConfig  # noqa: E402
from rpo_teleop.arm_visual import ArmVisual  # noqa: E402

# 대시보드 목록에 뜨는 순서. label 은 사람이 읽는 이름.
ARMS = [
    {"key": "arm_new",  "config": "config/arm_new.json",
     "label": "신규 5축 + 평행 그리퍼", "note": "Damiao DM-J4340P · STS3215 그리퍼"},
    {"key": "arm",      "config": "config/arm.json",
     "label": "기존 5축 + AmazingHand", "note": "Fusion robot_arm v36"},
    {"key": "arm_temp", "config": "config/arm_temp.json",
     "label": "임시 3축 (검증용)", "note": "위치랭크 2 — 조작 감각 제한적"},
]


def dump(cfg_path: Path) -> dict | None:
    import placo
    if not cfg_path.exists():
        return None
    cfg = ArmConfig.load(cfg_path)
    robot = placo.RobotWrapper(cfg.urdf_path)
    for n, v in zip(cfg.joint_names, cfg.home, strict=True):
        robot.set_joint(n, float(v))
    robot.update_kinematics()
    vis = ArmVisual(robot, cfg.urdf_path)
    ee = robot.get_T_world_frame(cfg.ee_frame)[:3, 3]
    return {
        "urdf": Path(cfg.urdf_path).name,
        "dof": cfg.dof,
        "ee_frame": cfg.ee_frame,
        "joint_names": list(cfg.joint_names),
        "home_deg": [round(float(np.degrees(v)), 2) for v in cfg.home],
        "reach_mm": [round(cfg.min_reach * 1000, 1), round(cfg.max_reach * 1000, 1)],
        "position_rank": cfg.position_rank,
        "mass_kg": round(float(cfg.total_mass), 3),
        # 렌더러가 그대로 먹는 형식 (서버가 보내는 것과 같은 모양)
        "geometry": vis.geometry,
        "links": vis.poses(),
        "ee_point": vis.point(ee),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="팔별 홈자세 형상 → 웹 미리보기 JSON")
    ap.add_argument("--out", type=Path, default=ROOT / "web/vendor/arms.json")
    args = ap.parse_args()

    out = []
    for a in ARMS:
        d = dump(ROOT / a["config"])
        if d is None:
            print(f"  ⚠️  건너뜀 (설정 없음): {a['config']}")
            continue
        out.append({**a, **d})
        print(f"  {a['key']:9s} {d['urdf']:22s} DOF {d['dof']}  "
              f"리치 {d['reach_mm'][0]:.0f}~{d['reach_mm'][1]:.0f}mm  "
              f"링크 {len(d['geometry'])}개")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"arms": out}, separators=(",", ":")))
    print(f"\n  → {args.out.relative_to(ROOT)}  ({args.out.stat().st_size/1024:.1f} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
