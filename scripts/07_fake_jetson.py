#!/usr/bin/env python
"""가짜 젯슨 실행 — 실물 젯슨 없이 모터 구간을 끝까지 테스트한다.

    .venv/bin/python scripts/07_fake_jetson.py                    # 기본
    .venv/bin/python scripts/07_fake_jetson.py --drop 0.3         # 30% 패킷 손실
    .venv/bin/python scripts/07_fake_jetson.py --blackout 10 4    # 10초마다 4초 끊김
    .venv/bin/python scripts/07_fake_jetson.py --motors 3         # 3축 다 붙은 경우

다른 터미널에서 텔레옵을 이렇게 띄우면 붙는다.

    ./run.sh --temp --motors jetson

관절 한계·속도는 config 에서 읽으므로 실물과 같은 값으로 돈다.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rpo_teleop.arm_config import ArmConfig  # noqa: E402
from rpo_teleop.jetson_sim import FakeJetson  # noqa: E402
from rpo_teleop import profiles  # noqa: E402

DEFAULT_CONFIG = ROOT / "config" / "arm_temp.json"


def main() -> int:
    ap = argparse.ArgumentParser(description="가짜 젯슨 (UDP 브리지 시뮬레이터)")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--motors", type=int, default=2,
                    help="실제로 모터가 붙은 관절 수 (현재 실물은 2)")
    ap.add_argument("--velocity-scale", type=float, default=0.1,
                    help="URDF 속도 한계 대비 허용 비율. 시운전은 0.1 부터")
    ap.add_argument("--drop", type=float, default=0.0,
                    help="수신 패킷 손실률 0~1 (핫스팟 흉내)")
    ap.add_argument("--blackout", type=float, nargs=2, metavar=("간격초", "지속초"),
                    default=None, help="주기적 통신 두절 (예: --blackout 10 4)")
    ap.add_argument("--rate", type=float, default=30.0, help="상태 송신 Hz")
    ap.add_argument("--motor-hz", type=float, default=None,
                    help="모터 지령 보간 Hz (안전계층 뒤). 실물 계획은 500")
    ap.add_argument("--profile", type=str, default="jetson",
                    choices=sorted(profiles.PROFILE_SLOTS),
                    help="포트 블록. 텔레옵 쪽과 같은 값을 써야 붙는다")
    ap.add_argument("--host", type=str, default="", help="바인드 주소 (기본 전체)")
    args = ap.parse_args()

    cfg = ArmConfig.load(args.config)
    n_motors = min(args.motors, cfg.dof)
    pf = profiles.ports(args.profile)

    jet = FakeJetson(
        lower=cfg.lower, upper=cfg.upper, max_velocity=cfg.velocity,
        n_joints=cfg.dof, n_motors=n_motors, dt=cfg.control_dt,
        rate_hz=args.rate, motor_hz=args.motor_hz,
        velocity_scale=args.velocity_scale,
        cmd_port=pf.cmd, state_port=pf.state, beacon_port=pf.beacon,
        robot=Path(cfg.urdf_path).name,
        host=args.host, drop_rate=args.drop,
        blackout=tuple(args.blackout) if args.blackout else None,
    )
    jet.start()

    vmax = float(np.max(cfg.velocity)) * args.velocity_scale
    print("=" * 74)
    print("  가짜 젯슨 — UDP 브리지 시뮬레이터")
    print("=" * 74)
    print(f"  프로파일    : {args.profile}")
    print(f"  로봇        : {Path(cfg.urdf_path).name}  프로토콜 {cfg.dof}축 / 모터 {n_motors}축")
    if n_motors < cfg.dof:
        print(f"                ★ q[{n_motors}:] 는 버립니다. 상태에서는 0 으로 채워 보냅니다.")
        print("                  joint3 은 위치 기여가 0 이라 손끝 도달 범위는 3축과 동일합니다.")
    print(f"  수신        : UDP {jet.cmd_port}   송신: UDP {jet.state_port}   비컨: UDP {jet.beacon_port}")
    print(f"  속도 제한   : {args.velocity_scale:.2f}× → {vmax:.3f} rad/s ({np.degrees(vmax):.1f}°/s)")
    print(f"  워치독      : {jet.limits.watchdog_freeze_s*1000:.0f}ms 동결 / "
          f"{jet.limits.watchdog_linklost_s*1000:.0f}ms RUN하차+재무장 / "
          f"{jet.limits.watchdog_s:.1f}s TRIP")
    print(f"  제어 주기   : 안전계층 {jet.control_hz:.0f} Hz"
          + (f" → 보간 {jet.motor_hz:.0f} Hz (안전계층 뒤)" if jet.motor_hz else ""))
    if args.drop:
        print(f"  ★ 패킷 손실 주입 : {args.drop*100:.0f}%")
    if args.blackout:
        print(f"  ★ 통신 두절 주입 : {args.blackout[0]:.0f}초마다 {args.blackout[1]:.0f}초")
    print("-" * 74)
    print(f"  텔레옵 쪽에서:  ./run.sh --temp --motors jetson"
          + ("" if args.profile == "jetson" else f" --profile {args.profile}"))
    print("  Ctrl+C 로 종료")
    print("=" * 74, flush=True)

    last = ""
    try:
        while True:
            time.sleep(0.5)
            s = jet.stats
            q = jet.motors.read_positions()
            line = (f"  [{jet.state:8s}] q={np.round(np.degrees(q), 1)}°  "
                    f"rx={s['rx']} tx={s['tx']} "
                    f"손실(주입)={s['dropped_sim']} 손실(seq역행)={s['dropped_seq']} "
                    f"트립={s['trips']} 세션={s['sessions']}")
            if jet.trip:
                line += f"  ⚠️ {jet.trip}"
            if line != last:
                print(line, flush=True)
                last = line
    except KeyboardInterrupt:
        print("\n  종료합니다.")
    finally:
        jet.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
