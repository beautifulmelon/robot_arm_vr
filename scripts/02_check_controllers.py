#!/usr/bin/env python
"""2단계: Quest 2 컨트롤러 전체 입력 매핑 확인.

커스텀 프론트엔드(web/index.html)를 서빙해서 양손 컨트롤러의 모든 버튼/축/pose와
손 트래킹을 받아본다. stock teleop 페이지가 오른손 pose + 버튼 3개만 주던 것을
전부 열어놓고, 어느 물리 버튼이 gamepad 몇 번 인덱스인지 실측으로 확정한다.

핵심 기능 — 버튼 인덱스 자동 발견:
    로그에 "buttons[4] 0.00 → 1.00" 처럼 변화한 인덱스를 즉시 찍어준다.
    Quest 컨트롤러를 손에 들고 버튼을 하나씩 누르면서 확인하면 된다.

사용법:
    .venv/bin/python scripts/02_check_controllers.py
    .venv/bin/python scripts/02_check_controllers.py --dump out.jsonl   # 원본 저장

종료: Ctrl+C → 매핑 요약
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import teleop as teleop_module  # noqa: E402
from teleop import Teleop  # noqa: E402

from rpo_teleop.certs import ensure_cert, get_local_ip  # noqa: E402

WEB_DIR = ROOT / "web"

# 눌러볼 물리 입력 체크리스트 (사용자에게 안내 + 요약에 사용)
CHECKLIST = [
    ("right", "trigger", "오른손 검지 트리거 (끝까지 당기기)"),
    ("right", "grip", "오른손 중지 그립 (끝까지 쥐기)"),
    ("right", "primary", "오른손 A 버튼"),
    ("right", "secondary", "오른손 B 버튼"),
    ("right", "stickPress", "오른손 썸스틱 누르기"),
    ("right", "stick", "오른손 썸스틱 상하좌우 끝까지"),
    ("left", "trigger", "왼손 검지 트리거"),
    ("left", "grip", "왼손 중지 그립"),
    ("left", "primary", "왼손 X 버튼"),
    ("left", "secondary", "왼손 Y 버튼"),
    ("left", "stickPress", "왼손 썸스틱 누르기"),
    ("left", "stick", "왼손 썸스틱 상하좌우 끝까지"),
]


class Observer:
    """콜백 스레드에서 상태를 모으고 리포터 스레드가 읽는다."""

    def __init__(self, dump_path: Path | None = None):
        self.lock = threading.Lock()
        self.count = 0
        self.first_ts: float | None = None
        self.last_ts: float | None = None
        self.recent: list[float] = []
        self.latest: dict = {}
        self.events: list[str] = []

        # side -> logical name -> 최대 관측값 (아날로그는 0~1, 스틱은 |값|)
        self.seen: dict[str, dict[str, float]] = defaultdict(lambda: defaultdict(float))
        # side -> raw button index -> 최대값  (물리 매핑 실측용)
        self.raw_btn_max: dict[str, dict[int, float]] = defaultdict(lambda: defaultdict(float))
        self.raw_btn_prev: dict[str, dict[int, float]] = defaultdict(lambda: defaultdict(float))
        self.raw_axis_range: dict[str, dict[int, tuple[float, float]]] = defaultdict(dict)

        self.profiles: dict[str, list[str]] = {}
        self.hand_frames: dict[str, int] = defaultdict(int)
        self.pos_range: dict[str, tuple[np.ndarray, np.ndarray]] = {}

        self._dump = dump_path.open("w", encoding="utf-8") if dump_path else None

    def close(self):
        if self._dump:
            self._dump.close()

    def __call__(self, pose: np.ndarray, message: dict) -> None:
        now = time.time()
        with self.lock:
            if self._dump:
                self._dump.write(json.dumps(message, separators=(",", ":")) + "\n")

            if self.first_ts is None:
                self.first_ts = now
                self.events.append("✅ 커스텀 프론트엔드 연결 — 전체 입력 수신 시작")
            self.count += 1
            self.last_ts = now
            self.recent.append(now)
            if len(self.recent) > 120:
                del self.recent[:-120]
            self.latest = message

            for side in ("left", "right"):
                ctrl = message.get(side)
                if not ctrl:
                    continue

                if side not in self.profiles and ctrl.get("profiles"):
                    self.profiles[side] = ctrl["profiles"]
                    self.events.append(f"🎮 {side} 컨트롤러 감지: {', '.join(ctrl['profiles'][:2])}")

                gp = ctrl.get("gamepad") or {}

                # 논리 이름별 최대 관측값
                for name in ("trigger", "grip"):
                    self.seen[side][name] = max(self.seen[side][name], float(gp.get(name, 0.0)))
                for name in ("primary", "secondary", "stickPress"):
                    if gp.get(name):
                        self.seen[side][name] = 1.0
                stick = gp.get("stick") or [0.0, 0.0]
                self.seen[side]["stick"] = max(self.seen[side]["stick"], max(abs(v) for v in stick))

                # 원본 버튼 인덱스 — 상승 엣지를 이벤트로 찍어 물리 매핑을 확정한다
                for idx, val in enumerate(gp.get("rawButtons") or []):
                    val = float(val)
                    prev = self.raw_btn_prev[side][idx]
                    if val > 0.5 >= prev:
                        self.events.append(f"🔘 {side} buttons[{idx}] → {val:.2f}")
                    self.raw_btn_prev[side][idx] = val
                    self.raw_btn_max[side][idx] = max(self.raw_btn_max[side][idx], val)

                for idx, val in enumerate(gp.get("rawAxes") or []):
                    val = float(val)
                    lo, hi = self.raw_axis_range[side].get(idx, (math.inf, -math.inf))
                    self.raw_axis_range[side][idx] = (min(lo, val), max(hi, val))

                # 위치 범위
                pose_src = ctrl.get("grip") or ctrl.get("targetRay")
                if pose_src:
                    p = pose_src["position"]
                    v = np.array([p["x"], p["y"], p["z"]])
                    lo, hi = self.pos_range.get(side, (v.copy(), v.copy()))
                    self.pos_range[side] = (np.minimum(lo, v), np.maximum(hi, v))

            for side in (message.get("hands") or {}):
                if self.hand_frames[side] == 0:
                    self.events.append(f"✋ {side} 손 트래킹 감지")
                self.hand_frames[side] += 1


def reporter(obs: Observer, stop: threading.Event, period: float = 0.5) -> None:
    waiting = False
    while not stop.is_set():
        time.sleep(period)
        with obs.lock:
            events, obs.events = obs.events[:], []
            count, recent, latest, last_ts = obs.count, obs.recent[:], dict(obs.latest), obs.last_ts

        for ev in events:
            print(f"  {ev}", flush=True)

        if count == 0:
            if not waiting:
                print("  … Quest 2 접속 대기 중", flush=True)
                waiting = True
            continue

        if last_ts and time.time() - last_ts > 2.0:
            print(f"  ⚠️  {time.time() - last_ts:.1f}초째 데이터 없음", flush=True)
            continue

        hz = 0.0
        if len(recent) >= 2 and (span := recent[-1] - recent[0]) > 0:
            hz = (len(recent) - 1) / span

        parts = [f"{hz:5.1f}Hz"]
        for side, tag in (("right", "R"), ("left", "L")):
            ctrl = latest.get(side)
            if not ctrl:
                parts.append(f"{tag}:—")
                continue
            gp = ctrl.get("gamepad") or {}
            src = ctrl.get("grip") or ctrl.get("targetRay")
            pos = src["position"] if src else {"x": 0, "y": 0, "z": 0}
            stick = gp.get("stick") or [0, 0]
            parts.append(
                f"{tag}[{pos['x']:+.2f} {pos['y']:+.2f} {pos['z']:+.2f}]"
                f" T{gp.get('trigger', 0):.2f} G{gp.get('grip', 0):.2f}"
                f" {'A' if gp.get('primary') else '·'}{'B' if gp.get('secondary') else '·'}"
                f" s[{stick[0]:+.2f}{stick[1]:+.2f}]"
            )
        hands = latest.get("hands") or {}
        if hands:
            parts.append(f"hands:{','.join(hands)}")
        parts.append(f"move{'●' if latest.get('move') else '○'}")
        parts.append(f"scale {latest.get('scale', 1.0):.2f}")
        print("  " + " | ".join(parts), flush=True)


def print_summary(obs: Observer) -> None:
    with obs.lock:
        count, first_ts, last_ts = obs.count, obs.first_ts, obs.last_ts
        seen = {s: dict(v) for s, v in obs.seen.items()}
        raw_btn = {s: dict(v) for s, v in obs.raw_btn_max.items()}
        raw_axis = {s: dict(v) for s, v in obs.raw_axis_range.items()}
        profiles, hand_frames = dict(obs.profiles), dict(obs.hand_frames)
        pos_range = dict(obs.pos_range)

    print("\n" + "=" * 78)
    print("  컨트롤러 매핑 요약")
    print("=" * 78)

    if count == 0:
        print("  ❌ 수신 데이터 없음 — 페이지 접속/[Start XR] 여부를 확인하세요.")
        return

    dur = (last_ts - first_ts) if (first_ts and last_ts) else 0.0
    print(f"  수신 {count} 프레임 / {dur:.1f} 초 → 평균 {count / dur if dur else 0:.1f} Hz\n")

    for side in ("right", "left"):
        prof = profiles.get(side)
        print(f"  [{side}] {'프로파일: ' + prof[0] if prof else '컨트롤러 미감지'}")
        if side not in seen:
            print("     (입력 없음)\n")
            continue
        for _s, name, desc in [c for c in CHECKLIST if c[0] == side]:
            val = seen[side].get(name, 0.0)
            mark = "✅" if val > 0.3 else "❌"
            print(f"     {mark} {name:12s} max={val:.2f}   {desc}")
        if side in pos_range:
            lo, hi = pos_range[side]
            span = hi - lo
            print(f"     위치 범위 Δ [{span[0]:.2f} {span[1]:.2f} {span[2]:.2f}] m")
        print()

    print("  [실측 gamepad 인덱스] — 눌렸던 것만 표시")
    for side in ("right", "left"):
        if side not in raw_btn:
            continue
        pressed = {i: v for i, v in sorted(raw_btn[side].items()) if v > 0.3}
        moved = {i: r for i, r in sorted(raw_axis.get(side, {}).items()) if (r[1] - r[0]) > 0.3}
        print(f"    {side}: buttons " + (", ".join(f"[{i}]={v:.2f}" for i, v in pressed.items()) or "없음"))
        print(f"    {side}: axes    " + (", ".join(f"[{i}]={r[0]:+.2f}~{r[1]:+.2f}" for i, r in moved.items()) or "없음"))

    if hand_frames:
        print(f"\n  ✋ 손 트래킹: {', '.join(f'{s} {n} 프레임' for s, n in hand_frames.items())}")
    else:
        print("\n  ✋ 손 트래킹: 미감지 (컨트롤러를 내려놓으면 자동 전환됩니다)")
    print("=" * 78)


def main() -> int:
    parser = argparse.ArgumentParser(description="Quest 2 컨트롤러 전체 입력 매핑 확인")
    parser.add_argument("--port", type=int, default=4443)
    parser.add_argument("--ip", type=str, default=None)
    parser.add_argument("--dump", type=str, default=None, help="원본 메시지를 JSONL 로 저장")
    parser.add_argument("--new-cert", action="store_true")
    args = parser.parse_args()

    (WEB_DIR / "assets").mkdir(parents=True, exist_ok=True)  # teleop 이 /assets 를 마운트한다

    ip = args.ip or get_local_ip()
    cert_file, _ = ensure_cert(ip, force=args.new_cert)
    teleop_module.THIS_DIR = str(cert_file.parent)  # run() 이 읽는 cert 경로 교체

    url = f"https://{ip}:{args.port}"
    print("=" * 78)
    print("  Quest 2 컨트롤러 전체 입력 매핑 확인")
    print("=" * 78)
    print(f"  서버 주소 : {url}")
    print(f"  프론트엔드: {WEB_DIR / 'index.html'}")
    print("-" * 78)
    print("  Quest 2에서: 위 주소 접속 → [고급]/[계속 진행] → [Start XR]")
    print("  그 다음 아래를 하나씩 눌러보세요 (양손 모두):")
    for _s, _n, desc in CHECKLIST:
        print(f"    · {desc}")
    print("  마지막으로 컨트롤러를 내려놓고 맨손을 들어 손 트래킹도 확인해 주세요.")
    print("-" * 78)
    print("  Ctrl+C 로 종료하면 매핑 요약이 출력됩니다.")
    print("=" * 78, flush=True)

    dump_path = Path(args.dump) if args.dump else None
    obs = Observer(dump_path)
    stop = threading.Event()

    teleop = Teleop(host="0.0.0.0", port=args.port, frontend_dir=str(WEB_DIR))
    teleop.subscribe(obs)

    thread = threading.Thread(target=reporter, args=(obs, stop), daemon=True)
    thread.start()

    def handle_sigint(_s, _f):
        stop.set()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=1.0)
        print_summary(obs)
        obs.close()
        if dump_path:
            print(f"  원본 메시지 저장됨: {dump_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
