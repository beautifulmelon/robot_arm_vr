#!/usr/bin/env python
"""1단계: Meta Quest 2 ↔ WebXR 연결 검증.

로봇 없이 이 Mac 하나로 돌린다. teleop 패키지(SpesRobotics)의 WebXR 서버를 띄우고
Quest 2 브라우저에서 접속했을 때 실제로 무엇이 들어오는지 전부 찍어본다.

확인하려는 것:
  1. Quest 2 브라우저가 HTTPS 페이지를 열고 immersive-ar 세션을 시작하는가
  2. 오른손 컨트롤러의 6-DOF pose가 들어오는가 (위치 단위 m, 자세 quaternion)
  3. 갱신 주기가 제어 루프에 쓸 만한가 (목표 30 Hz 이상)
  4. move / reservedButtonA / reservedButtonB / scale 이 Quest 컨트롤러 버튼에 물리는가
  5. 손을 움직였을 때 위치 범위(작업 공간)가 얼마나 나오는가

사용법:
    .venv/bin/python scripts/01_check_webxr.py
    .venv/bin/python scripts/01_check_webxr.py --ip 192.168.0.10   # IP 수동 지정
    .venv/bin/python scripts/01_check_webxr.py --raw              # 폰용 -45° 보정 끄기

종료: Ctrl+C → 검증 요약 리포트 출력
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import teleop as teleop_module  # noqa: E402
from teleop import Teleop  # noqa: E402

from rpo_teleop.certs import ensure_cert, get_local_ip  # noqa: E402

# Quest 2 컨트롤러 기준 버튼 매핑 (teleop/index.html이 읽는 gamepad 인덱스)
BUTTON_HELP = {
    "move": "buttons[1] — 오른손 그립(Grip) 또는 트리거",
    "reservedButtonA": "buttons[4] — 오른손 A 버튼",
    "reservedButtonB": "buttons[5] — 오른손 B 버튼",
    "scale": "axes[0] — 오른손 썸스틱 좌우 (0.2 ~ 1.0)",
}


@dataclass
class Stats:
    """콜백 스레드가 쓰고 리포터 스레드가 읽는 공유 상태."""

    lock: threading.Lock = field(default_factory=threading.Lock)

    count: int = 0
    first_ts: float | None = None
    last_ts: float | None = None
    recent_ts: list[float] = field(default_factory=list)

    raw_pos_min: np.ndarray = field(default_factory=lambda: np.full(3, np.inf))
    raw_pos_max: np.ndarray = field(default_factory=lambda: np.full(3, -np.inf))

    last_raw_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    last_raw_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    last_pose: np.ndarray = field(default_factory=lambda: np.eye(4))
    last_msg: dict = field(default_factory=dict)

    # 버튼별 상승 엣지 횟수
    edges: dict[str, int] = field(default_factory=lambda: {"move": 0, "reservedButtonA": 0, "reservedButtonB": 0})
    prev_button: dict[str, bool] = field(default_factory=lambda: {"move": False, "reservedButtonA": False, "reservedButtonB": False})
    scale_min: float = math.inf
    scale_max: float = -math.inf

    events: list[str] = field(default_factory=list)


def quat_to_euler_deg(quat_wxyz: np.ndarray) -> np.ndarray:
    """quaternion(w,x,y,z) → roll/pitch/yaw (도)."""
    w, x, y, z = quat_wxyz
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.degrees([roll, pitch, yaw])


def make_callback(stats: Stats):
    def callback(pose: np.ndarray, message: dict) -> None:
        now = time.time()
        pos = message.get("position") or {}
        rot = message.get("orientation") or {}
        raw_pos = np.array([pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)])
        raw_quat = np.array([rot.get("w", 1.0), rot.get("x", 0.0), rot.get("y", 0.0), rot.get("z", 0.0)])

        with stats.lock:
            if stats.first_ts is None:
                stats.first_ts = now
                stats.events.append("✅ 첫 pose 수신 — WebXR 세션이 살아있습니다")
            stats.count += 1
            stats.last_ts = now
            stats.recent_ts.append(now)
            if len(stats.recent_ts) > 120:
                del stats.recent_ts[:-120]

            stats.raw_pos_min = np.minimum(stats.raw_pos_min, raw_pos)
            stats.raw_pos_max = np.maximum(stats.raw_pos_max, raw_pos)
            stats.last_raw_pos = raw_pos
            stats.last_raw_quat = raw_quat
            stats.last_pose = pose.copy()
            stats.last_msg = message

            for name in stats.prev_button:
                pressed = bool(message.get(name, False))
                if pressed and not stats.prev_button[name]:
                    stats.edges[name] += 1
                    stats.events.append(f"🔘 {name} ON   ({BUTTON_HELP[name]})")
                elif not pressed and stats.prev_button[name]:
                    stats.events.append(f"⚪ {name} OFF")
                stats.prev_button[name] = pressed

            scale = float(message.get("scale", 1.0))
            stats.scale_min = min(stats.scale_min, scale)
            stats.scale_max = max(stats.scale_max, scale)

    return callback


def reporter(stats: Stats, stop: threading.Event, period: float = 0.5) -> None:
    """append-only 상태 출력. ANSI 리드로우 대신 줄 단위로 찍어 로그로 남겨도 읽힌다."""
    waiting_logged = False
    while not stop.is_set():
        time.sleep(period)
        with stats.lock:
            events = stats.events[:]
            stats.events.clear()
            count = stats.count
            recent = stats.recent_ts[:]
            raw_pos = stats.last_raw_pos.copy()
            raw_quat = stats.last_raw_quat.copy()
            pose = stats.last_pose.copy()
            msg = dict(stats.last_msg)
            last_ts = stats.last_ts

        for ev in events:
            print(f"  {ev}", flush=True)

        if count == 0:
            if not waiting_logged:
                print("  … Quest 2 접속 대기 중", flush=True)
                waiting_logged = True
            continue

        if last_ts is not None and time.time() - last_ts > 2.0:
            print(f"  ⚠️  {time.time() - last_ts:.1f}초째 데이터 없음 (세션이 끊겼거나 헤드셋 슬립)", flush=True)
            continue

        hz = 0.0
        if len(recent) >= 2:
            span = recent[-1] - recent[0]
            if span > 0:
                hz = (len(recent) - 1) / span

        euler = quat_to_euler_deg(raw_quat)
        flags = " ".join(
            f"{tag}{'●' if msg.get(key) else '○'}"
            for tag, key in (("move", "move"), ("A", "reservedButtonA"), ("B", "reservedButtonB"))
        )
        print(
            f"  {hz:5.1f}Hz | raw xyz [{raw_pos[0]:+.3f} {raw_pos[1]:+.3f} {raw_pos[2]:+.3f}] m"
            f" | rpy [{euler[0]:+6.1f} {euler[1]:+6.1f} {euler[2]:+6.1f}]°"
            f" | out xyz [{pose[0, 3]:+.3f} {pose[1, 3]:+.3f} {pose[2, 3]:+.3f}]"
            f" | scale {float(msg.get('scale', 1.0)):.2f} | {flags} | n={count}",
            flush=True,
        )


def print_summary(stats: Stats) -> None:
    with stats.lock:
        count = stats.count
        first_ts, last_ts = stats.first_ts, stats.last_ts
        pos_min, pos_max = stats.raw_pos_min.copy(), stats.raw_pos_max.copy()
        edges = dict(stats.edges)
        scale_min, scale_max = stats.scale_min, stats.scale_max

    print("\n" + "=" * 74)
    print("  검증 요약")
    print("=" * 74)

    if count == 0:
        print("  ❌ 수신된 pose 없음 — WebXR 세션이 성립하지 않았습니다.")
        print()
        print("  점검 순서:")
        print("   1. Quest 2와 이 Mac이 같은 Wi-Fi인가? (게스트망/AP 격리 주의)")
        print("   2. Quest 브라우저에서 인증서 경고 → '고급' → '계속 진행'을 눌렀는가?")
        print("   3. 페이지의 [Start] 버튼을 눌렀는가? (WebXR은 사용자 제스처가 있어야 시작됨)")
        print("   4. macOS 방화벽이 python을 막고 있지 않은가?")
        print("      시스템 설정 → 네트워크 → 방화벽 → 옵션에서 확인")
        return

    duration = (last_ts - first_ts) if (first_ts and last_ts) else 0.0
    avg_hz = count / duration if duration > 0 else 0.0
    span = pos_max - pos_min

    print(f"  수신 메시지     : {count} 개 / {duration:.1f} 초  →  평균 {avg_hz:.1f} Hz")
    if avg_hz >= 30:
        print("                    ✅ 30 Hz 이상 — 제어 루프에 충분")
    elif avg_hz >= 15:
        print("                    ⚠️  30 Hz 미만 — 동작은 하나 부드럽지 않을 수 있음")
    else:
        print("                    ❌ 너무 낮음 — 네트워크/헤드셋 상태 점검 필요")

    print(f"  위치 범위 (raw) : x {pos_min[0]:+.3f}~{pos_max[0]:+.3f}  "
          f"y {pos_min[1]:+.3f}~{pos_max[1]:+.3f}  z {pos_min[2]:+.3f}~{pos_max[2]:+.3f} (m)")
    print(f"  이동량          : Δ [{span[0]:.3f} {span[1]:.3f} {span[2]:.3f}] m")
    if max(span) < 0.05:
        print("                    ⚠️  거의 안 움직였습니다. 팔을 크게 휘저으며 다시 측정해 보세요")
    else:
        print("                    ✅ 6-DOF 트래킹 정상")

    print("  버튼 입력       :")
    any_button = False
    for name, n in edges.items():
        mark = "✅" if n else "—"
        print(f"    {mark} {name:18s} {n:3d} 회   {BUTTON_HELP[name]}")
        any_button = any_button or bool(n)
    if scale_max > -math.inf:
        mark = "✅" if (scale_max - scale_min) > 0.01 else "—"
        print(f"    {mark} {'scale':18s} {scale_min:.2f}~{scale_max:.2f}   {BUTTON_HELP['scale']}")
    if not any_button:
        print("    ⚠️  버튼이 하나도 안 눌렸습니다. Grip/A/B를 눌러가며 다시 측정해 보세요")

    print("=" * 74)


def main() -> int:
    parser = argparse.ArgumentParser(description="Quest 2 ↔ WebXR 연결 검증")
    parser.add_argument("--port", type=int, default=4443, help="HTTPS 포트 (기본 4443)")
    parser.add_argument("--ip", type=str, default=None, help="접속용 IP 수동 지정 (기본 자동 감지)")
    parser.add_argument("--raw", action="store_true",
                        help="teleop 기본값인 폰용 -45° pitch 보정을 끄고 컨트롤러 원본 자세를 본다")
    parser.add_argument("--new-cert", action="store_true", help="인증서 강제 재발급")
    args = parser.parse_args()

    ip = args.ip or get_local_ip()
    cert_file, key_file = ensure_cert(ip, force=args.new_cert)

    # teleop.Teleop.run() 은 모듈 전역 THIS_DIR 아래의 cert.pem/key.pem 을 읽는다.
    # 동봉된 인증서는 2025-07-28 만료라 우리가 만든 인증서 디렉토리로 갈아끼운다.
    # frontend_dir 은 __init__ 시점에 고정되므로 패키지 경로를 명시적으로 넘겨준다.
    package_dir = os.path.dirname(teleop_module.__file__)
    teleop_module.THIS_DIR = str(cert_file.parent)

    url = f"https://{ip}:{args.port}"
    print("=" * 74)
    print("  Quest 2 ↔ WebXR 연결 검증")
    print("=" * 74)
    print(f"  서버 주소   : {url}")
    print(f"  인증서      : {cert_file}  (SAN: IP:{ip})")
    print(f"  자세 보정   : {'없음 (원본)' if args.raw else '폰 기본값 pitch -45°'}")
    print("-" * 74)
    print("  Quest 2에서 할 일:")
    print(f"   1. 브라우저를 열고 주소창에  {url}  입력")
    print("   2. '연결이 비공개로 설정되어 있지 않습니다' → [고급] → [계속 진행]")
    print("   3. 페이지의 [Start] 버튼 누르기 → 권한 허용")
    print("   4. 오른손 컨트롤러를 들고 움직이면서 Grip / A / B / 썸스틱을 눌러보기")
    print("-" * 74)
    print("  Ctrl+C 로 종료하면 검증 요약이 출력됩니다.")
    print("=" * 74, flush=True)

    stats = Stats()
    stop = threading.Event()

    kwargs = {"host": "0.0.0.0", "port": args.port, "frontend_dir": package_dir}
    if args.raw:
        kwargs["natural_phone_orientation_euler"] = [0.0, 0.0, 0.0]

    teleop = Teleop(**kwargs)
    teleop.subscribe(make_callback(stats))

    reporter_thread = threading.Thread(target=reporter, args=(stats, stop), daemon=True)
    reporter_thread.start()

    def handle_sigint(_signum, _frame):
        stop.set()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        teleop.run()  # blocking
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        reporter_thread.join(timeout=1.0)
        print_summary(stats)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
