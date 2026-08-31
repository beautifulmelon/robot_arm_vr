#!/usr/bin/env python
"""지금 화면의 팔 자세를 읽어 홈 자세로 쓸 값을 뽑는다.

    .venv/bin/python scripts/13_pose_snapshot.py                 # 기본 4453
    .venv/bin/python scripts/13_pose_snapshot.py --port 4443

무엇에 쓰나
    VR 로 팔을 원하는 자세까지 움직인 뒤 이걸 돌리면, 그 순간의 관절각을
    **config 에 바로 붙여넣을 형태**로 찍어준다. 눈으로 각도를 읽어 옮겨
    적으면 틀리기 쉽다.

    ★ 조종을 멈춘 상태(Grip 을 뗀 상태)에서 돌릴 것. 움직이는 중에 찍으면
      찍힌 값과 화면이 다르다.
"""

from __future__ import annotations

import argparse
import json
import ssl
import sys
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def fetch(port: int) -> dict:
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE      # 자체서명 인증서
    with urllib.request.urlopen(f"https://127.0.0.1:{port}/state",
                                timeout=8, context=ctx) as r:
        return json.loads(r.read().decode())


def main() -> int:
    ap = argparse.ArgumentParser(description="현재 팔 자세 → 홈 자세 값")
    ap.add_argument("--port", type=int, default=4453, help="텔레옵 웹 포트")
    ap.add_argument("--write", type=Path, default=None,
                    help="이 config 파일의 home_q 를 바로 덮어쓴다")
    args = ap.parse_args()

    try:
        st = fetch(args.port)
    except Exception as e:
        print(f"❌ 서버에 못 붙었습니다 (포트 {args.port}): {e}", file=sys.stderr)
        print("   텔레옵이 떠 있는지, 포트가 맞는지 확인하세요 "
              "(--profile isaac 이면 4453, 기본이면 4443)", file=sys.stderr)
        return 1

    joints = st.get("joints") or []
    if not joints:
        print("❌ 상태에 관절 정보가 없습니다.", file=sys.stderr)
        return 1

    deg = [j["deg"] for j in joints]
    rad = [float(np.radians(d)) for d in deg]
    robot = (st.get("robot") or {}).get("name", "?")

    print(f"  로봇   {robot}")
    print(f"  관절   {', '.join(j['name'] for j in joints)}")
    print()
    print(f"  각도(deg)  [{'  '.join(f'{d:+7.2f}' for d in deg)}]")
    print()
    print("  config 의 home_q 에 넣을 값 (rad):")
    print("    " + json.dumps([round(v, 9) for v in rad]))

    if args.write:
        d = json.loads(args.write.read_text())
        old = d.get("home_q")
        d["home_q"] = [round(v, 9) for v in rad]
        args.write.write_text(json.dumps(d, indent=2, ensure_ascii=False))
        print()
        print(f"  ✅ {args.write} 의 home_q 를 덮어썼습니다")
        if old:
            print(f"     이전 {np.round(np.degrees(old), 2)}")
        print(f"     현재 {np.round(deg, 2)}")
        print("     ※ 대시보드 미리보기도 갱신하려면: "
              ".venv/bin/python scripts/12_arm_preview.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
