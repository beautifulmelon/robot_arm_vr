#!/usr/bin/env python
"""가짜 Mac — Quest 없이 지령 패킷만 흘려보낸다. 받는 쪽(젯슨/Isaac) 개발용.

07_fake_jetson.py 의 정확히 반대편이다. 저쪽은 "Mac 없이 받는 쪽을 테스트"하고,
이쪽은 "Quest·IK 없이 보내는 쪽을 흉내내서 받는 쪽을 테스트"한다.

    [이 스크립트] ──UDP 지령──> [Isaac 브릿지 / 젯슨]
                  <─UDP 상태──

★ JetsonBackend 를 그대로 쓴다. 흉내내지 않는다.
  프로토콜을 다시 구현하면 그 구현이 진짜 Mac 과 미묘하게 달라지고, 받는 쪽은
  "테스트는 통과했는데 실제로는 안 붙는" 상황을 만난다. 실제 텔레옵 루프
  (05_teleop_sim.py)가 부르는 것과 같은 객체·같은 순서로 부른다.

★ 포트는 --profile 이 정한다 (profiles.py). 받는 쪽과 반드시 같은 값이어야 한다.
      jetson  지령 5005 / 상태 5006     실물 젯슨 (기본)
      isaac   지령 5015 / 상태 5016     Isaac Sim 브릿지

사용법
    # Isaac 브릿지(집 리눅스)에 붙기 — 팔 5축.  ★ --profile isaac 을 빠뜨리지 말 것
    .venv/bin/python scripts/08_fake_mac.py --host 100.93.186.122 --profile isaac

    # 손 서보각 8개까지 같이 보내기
    .venv/bin/python scripts/08_fake_mac.py --host 100.93.186.122 --profile isaac --grasp

    # 관절 하나씩 순서대로 (원인 분리용 — 어느 관절이 안 도는지 본다)
    .venv/bin/python scripts/08_fake_mac.py --host 100.93.186.122 --profile isaac \
        --sweep one-by-one

받는 쪽이 정상이면 이렇게 보인다.
    RUN  seq  312 | link ok  42ms | cmd [ +0.0  +15.3 ...] | act [ +0.0  +15.1 ...] | err 0.2°
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rpo_teleop.arm_config import ArmConfig  # noqa: E402
from rpo_teleop import profiles  # noqa: E402
from rpo_teleop.hand_model import grasp_to_servo  # noqa: E402
from rpo_teleop.jetson_link import STATE_TRIP, JetsonBackend  # noqa: E402

DEFAULT_CONFIG = ROOT / "config" / "arm.json"


def main() -> int:
    ap = argparse.ArgumentParser(description="가짜 Mac (지령 송신기) — 받는 쪽 테스트용")
    ap.add_argument("--profile", type=str, default="jetson",
                    choices=sorted(profiles.PROFILE_SLOTS),
                    help="포트 블록. 받는 쪽과 같은 값을 써야 붙는다")
    ap.add_argument("--host", required=True, help="받는 쪽 주소 (예: 100.93.186.122)")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                    help="관절 한계·홈자세를 읽을 설정 파일")
    ap.add_argument("--rate", type=float, default=30.0, help="지령 Hz (실제 텔레옵과 같게)")
    ap.add_argument("--amp", type=float, default=20.0, help="흔드는 진폭 (도)")
    ap.add_argument("--period", type=float, default=6.0, help="한 주기 (초)")
    ap.add_argument("--sweep", choices=("all", "one-by-one", "hold"), default="all",
                    help="all=전 관절 동시, one-by-one=한 번에 하나씩, hold=홈자세 고정")
    ap.add_argument("--grasp", action="store_true",
                    help="손 servo 8개도 같이 보낸다 (받는 쪽이 손을 지원할 때)")
    ap.add_argument("--hand-period", type=float, default=4.0,
                    help="손을 폈다 쥐는 한 주기 (초). --grasp 일 때만")
    ap.add_argument("--hold-s", type=float, default=3.0,
                    help="RUN 으로 올리기 전 HOLD 로 링크를 확인하는 시간")
    ap.add_argument("--no-clear-trip", action="store_true",
                    help="시작할 때 남아 있는 트립을 풀지 않는다 (트립 동작 자체를 볼 때)")
    args = ap.parse_args()
    _pf = profiles.ports(args.profile)

    if not args.config.exists():
        print(f"❌ 설정 파일이 없습니다: {args.config}", file=sys.stderr)
        return 1
    cfg = ArmConfig.load(args.config)

    home = np.asarray(cfg.home, dtype=float)
    lo = np.asarray(cfg.lower, dtype=float) + 0.1
    hi = np.asarray(cfg.upper, dtype=float) - 0.1

    motors = JetsonBackend(n_joints=cfg.dof, host=args.host, discover=False,
                           cmd_port=_pf.cmd, state_port=_pf.state,
                           beacon_port=_pf.beacon)
    motors.connect()
    motors.hold()

    print("=" * 74)
    print("  가짜 Mac — 지령 송신기")
    print("=" * 74)
    print(f"  프로파일   : {args.profile}")
    print(f"  받는 쪽    : {args.host}:{_pf.cmd}   (상태는 {_pf.state} 로 되돌아옵니다)")
    print(f"  로봇       : {Path(cfg.urdf_path).name}  {cfg.dof}-DOF")
    print(f"  지령       : {args.rate:.0f} Hz   진폭 {args.amp:.0f}°   주기 {args.period:.0f}s   "
          f"방식 {args.sweep}")
    print(f"  손 grasp   : {'보냄' if args.grasp else '안 보냄'}")
    print(f"  먼저 {args.hold_s:.0f}초 HOLD 로 링크만 확인한 뒤 RUN 으로 올립니다.")
    print("  Ctrl+C 로 종료")
    print("-" * 74, flush=True)

    running = True

    def on_sigint(_s, _f):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_sigint)

    t0 = time.monotonic()
    period = 1.0 / args.rate
    armed = False
    last_print = 0.0
    seen_state = False
    cleared_stale_trip = False

    while running:
        t_loop = time.perf_counter()
        t = time.monotonic() - t0

        # ── 지령 생성 ────────────────────────────────────────────────
        phase = 2.0 * math.pi * t / args.period
        q = home.copy()
        if args.sweep == "all":
            q = home + math.sin(phase) * math.radians(args.amp)
        elif args.sweep == "one-by-one":
            # 관절 하나씩 순서대로. 어느 관절이 안 도는지 눈으로 분리된다.
            idx = int(t / args.period) % cfg.dof
            q[idx] = home[idx] + math.sin(phase) * math.radians(args.amp)
        q = np.clip(q, lo, hi)

        # ── 손 ────────────────────────────────────────────────────────
        # 트리거를 0↔1 로 왕복시키는 것과 같다. 서보 공간에서 보간해야 하므로
        # grasp 스칼라를 흔들고 grasp_to_servo() 로 8개를 만든다
        # (관절 공간 보간은 중간에서 최대 11.44° 어긋난다 — 실측).
        grasp = servo = None
        if args.grasp:
            grasp = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / args.hand_period))
            servo = grasp_to_servo(grasp)

        # ── 상태머신 — 실제 텔레옵과 같은 규칙 ────────────────────────
        #   ★ 트립/재무장 요구가 오면 무장을 푼다. 자동으로 다시 RUN 하지 않는다.
        #     (사람이 다시 잡아야 움직인다는 원칙을 테스트에서도 지킨다)
        st = motors.state
        if st is not None:
            seen_state = True
            if st.state == STATE_TRIP or st.await_rearm:
                if armed:
                    why = "트립" if st.state == STATE_TRIP else f"링크 {st.link}"
                    print(f"  ⏸ 무장 해제 ({why})", flush=True)
                armed = False

            # ★ 앞선 실행을 끊고 나면 받는 쪽 워치독이 트립을 겁니다. 그 상태가
            #   남아 있으면 이번 실행이 영원히 HOLD 에 머뭅니다 (직접 겪음).
            #   그래서 **초기 HOLD 창 안에서만** 한 번 풀어줍니다.
            #   ★ 그 창이 지난 뒤의 트립은 절대 자동으로 풀지 않습니다.
            #     그건 워치독이 진짜로 동작한 것이고, 자동 복구는 금지입니다.
            if (st.state == STATE_TRIP and t < args.hold_s
                    and not cleared_stale_trip and not args.no_clear_trip):
                cleared_stale_trip = True
                print(f"  ⏻ 이전 실행이 남긴 트립을 해제합니다 ({st.trip})", flush=True)
                motors.clear_trip()

        if not armed and t >= args.hold_s and args.sweep != "hold" and seen_state:
            if st is not None and st.state != STATE_TRIP and not st.await_rearm:
                armed = True
                print("  ▶ RUN 으로 올립니다", flush=True)

        motors.enable() if armed else motors.hold()
        # ★ servo 가 있으면 받는 쪽이 grasp 보다 우선한다 (jetson_link 주석).
        #   손은 반드시 서보각으로 — 관절각은 실물에 없는 자세가 나온다 (A-7).
        motors.write_positions(q, grasp=grasp, servo=servo, engaged=armed)

        # ── 표시 ─────────────────────────────────────────────────────
        now = time.time()
        if now - last_print >= 0.5:
            last_print = now
            s = motors.summary()
            if not seen_state:
                print(f"  … 상태 패킷 대기 중 (보낸 지령 {s['seq']}개, "
                      f"{args.host}:{_pf.cmd} 로 나가는 중)", flush=True)
            else:
                act = motors.read_positions()
                err = float(np.max(np.abs(np.degrees(act - q)))) if act.size == q.size else float("nan")
                link = f"{s['link_age_ms']:.0f}ms" if s["link_age_ms"] is not None else "—"
                # 손: 받는 쪽이 q 에 팔 5개 + 손 12개를 실어 보낸다.
                # read_positions() 는 앞 5개만 주므로 원본 상태에서 직접 꺼낸다.
                hand_txt = ""
                if grasp is not None:
                    stq = (motors.state.q if motors.state else []) or []
                    f1 = [v for v in stq[5:8] if v is not None]
                    hand_txt = (f" | grasp {grasp:.2f}"
                                + (f" f1[{' '.join(f'{np.degrees(v):+5.1f}' for v in f1)}]"
                                   if len(f1) == 3 else " f1(관절 회신 없음)"))
                print(f"  {s['state'] or '?':7s} seq {s['seq']:5d} | link {s['link']} {link} "
                      f"| cmd [{' '.join(f'{np.degrees(v):+6.1f}' for v in q)}] "
                      f"| act [{' '.join(f'{np.degrees(v):+6.1f}' for v in act)}] "
                      f"| 최대오차 {err:5.2f}°" + hand_txt
                      + (f" | trip: {s['trip']}" if s["trip"] else ""),
                      flush=True)

        sleep = period - (time.perf_counter() - t_loop)
        if sleep > 0:
            time.sleep(sleep)

    print("\n" + "-" * 74)
    # ★ 소자하지 않는다. 실물이라면 팔이 떨어진다. 시뮬이라도 같은 절차를 지킨다.
    motors.hold()
    motors.write_positions(motors.read_positions())
    motors.disconnect()
    print("  HOLD 로 두고 링크를 끊었습니다 (소자하지 않음)")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
