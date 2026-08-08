"""프로그램이 실제로 뜨는지 본다.

★ 왜 필요한가 — 테스트 115개가 통과하는데 **서버가 시작조차 안 되는** 일이
  실제로 있었다. 손목 롤 축을 재는 줄을 배너보다 뒤에 두는 바람에
  `UnboundLocalError` 로 죽었다. 다른 테스트는 전부 모듈을 import 해서 함수를
  직접 부르므로 main() 을 한 번도 안 돌린다. 그래서 못 잡았다.

  여기서는 진짜로 프로세스를 띄운다. 느리지만 이 한 종류를 확실히 막는다.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
PY = ROOT / ".venv" / "bin" / "python"
CONFIGS = [ROOT / "config" / "arm.json", ROOT / "config" / "arm_temp.json"]


def run_until(args, want: str, timeout: float = 40.0) -> str:
    """프로세스를 띄워 want 가 나올 때까지 기다린다. 나오면 죽이고 로그를 준다."""
    proc = subprocess.Popen(
        [str(PY), "-u", *args], cwd=str(ROOT),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        preexec_fn=os.setsid,
    )
    out, end = [], time.monotonic() + timeout
    try:
        while time.monotonic() < end:
            if proc.poll() is not None:                  # 죽었다
                out.append(proc.stdout.read())
                break
            line = proc.stdout.readline()
            if not line:
                continue
            out.append(line)
            if want in line:
                break
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
    return "".join(out)


@pytest.mark.parametrize("cfg", CONFIGS, ids=lambda p: p.stem)
def test_teleop_starts_without_error(cfg):
    """텔레옵이 배너까지 찍고 대기 상태로 들어가야 한다."""
    if not cfg.exists():
        pytest.skip(f"{cfg.name} 없음")
    if not PY.exists():
        pytest.skip(".venv 없음")

    # 실제 포트를 안 쓰도록 test 프로파일로 띄운다
    log = run_until(["scripts/05_teleop_sim.py", "--config", str(cfg),
                     "--profile", "test", "--no-hand"],
                    want="Quest 접속 대기 중")

    assert "Traceback" not in log, f"기동 중 예외:\n{log[-2500:]}"
    assert "Quest 접속 대기 중" in log, f"대기 상태까지 못 갔다:\n{log[-2500:]}"
    # 손목 롤 축을 실제로 재서 찍었는가 (배너보다 뒤에서 재면 여기서 걸린다)
    assert "손목 롤" in log, f"손목 롤 배너가 없다:\n{log[-2500:]}"


def test_fake_jetson_starts_without_error():
    if not PY.exists() or not CONFIGS[1].exists():
        pytest.skip("환경 없음")
    log = run_until(["scripts/07_fake_jetson.py", "--profile", "test",
                     "--config", str(CONFIGS[1])],
                    want="Ctrl+C 로 종료")
    assert "Traceback" not in log, f"기동 중 예외:\n{log[-2500:]}"
    assert "가짜 젯슨" in log


def test_teleop_and_motors_connect_end_to_end():
    """텔레옵 + 가짜 젯슨을 같이 띄워 실제로 붙는지 본다."""
    if not PY.exists() or not CONFIGS[1].exists():
        pytest.skip("환경 없음")

    jet = subprocess.Popen(
        [str(PY), "-u", "scripts/07_fake_jetson.py", "--profile", "test",
         "--config", str(CONFIGS[1])],
        cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    time.sleep(2.0)
    try:
        log = run_until(["scripts/05_teleop_sim.py", "--config", str(CONFIGS[1]),
                         "--profile", "test", "--no-hand", "--motors", "jetson",
                         "--jetson-host", "127.0.0.1"],
                        want="실물 관절각으로 동기")
        assert "Traceback" not in log, f"기동 중 예외:\n{log[-2500:]}"
        assert "실물 관절각으로 동기" in log, \
            f"모터에 안 붙었다 (연결 절차 3단계):\n{log[-2500:]}"
    finally:
        try:
            os.killpg(os.getpgid(jet.pid), signal.SIGKILL)
        except Exception:
            pass


def test_state_payload_has_receiver_and_actual_joints():
    """대시보드/HUD 가 쓰는 필드가 실제로 나가는지 본다.

    ★ q_act 가 없으면 화면이 **IK 목표만** 그린다. 모터가 부하에 걸리거나
      속도 제한에 막혀도 완벽하게 따라오는 것처럼 보이고, 사람이 모르고
      계속 밀어붙인다.
    ★ receiver 가 없으면 실물 갈래와 Isaac 갈래 중 어디에 붙었는지 모른다.
      헤드셋을 쓰면 터미널을 못 본다.
    """
    import json
    import ssl
    import urllib.request

    if not PY.exists() or not CONFIGS[1].exists():
        pytest.skip("환경 없음")

    jet = subprocess.Popen(
        [str(PY), "-u", "scripts/07_fake_jetson.py", "--profile", "test",
         "--config", str(CONFIGS[1])],
        cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    tel = subprocess.Popen(
        [str(PY), "-u", "scripts/05_teleop_sim.py", "--config", str(CONFIGS[1]),
         "--profile", "test", "--no-hand", "--motors", "jetson",
         "--jetson-host", "127.0.0.1"],
        cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    try:
        from rpo_teleop import profiles
        url = f"https://127.0.0.1:{profiles.ports('test').web}/state"
        data = None
        for _ in range(60):
            time.sleep(0.5)
            try:
                with urllib.request.urlopen(url, context=ctx, timeout=2) as r:
                    d = json.loads(r.read())
                if d.get("q_act") is not None:
                    data = d
                    break
            except Exception:
                continue
        assert data is not None, "q_act 가 실린 상태를 못 받았다"
        assert len(data["q_act"]) == len(data["q_cmd"]), "실제/지령 길이가 다르다"
        rcv = data.get("receiver")
        assert rcv and rcv["profile"] == "test", f"receiver 가 없다: {rcv}"
        assert rcv["label"], "받는 쪽 설명이 비어 있다"
    finally:
        for p_ in (tel, jet):
            try:
                os.killpg(os.getpgid(p_.pid), signal.SIGKILL)
            except Exception:
                pass


def test_wrong_robot_is_refused_not_crashed():
    """★ --temp 를 빠뜨리면 5-DOF 설정으로 3-DOF 실물에 지령이 나간다.

    예전에는 zip(strict=True) 가 터져 **서버가 반쯤 뜬 채 죽었다.** 메시지도
    "argument 2 is shorter than argument 1" 이라 원인을 알 수 없었다.
    지금은 사람이 읽을 수 있게 거부하고, 모터는 HOLD 로 둔다.

    죽는 것보다 나쁜 건 안 죽고 엉뚱한 관절을 움직이는 것이다.
    """
    if not PY.exists() or not all(c.exists() for c in CONFIGS):
        pytest.skip("환경 없음")

    # 받는 쪽은 3-DOF(임시 팔), Mac 은 5-DOF(전체 팔) 로 띄운다
    jet = subprocess.Popen(
        [str(PY), "-u", "scripts/07_fake_jetson.py", "--profile", "test",
         "--config", str(CONFIGS[1]), "--motors", "2"],
        cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    time.sleep(2.0)
    try:
        log = run_until(["scripts/05_teleop_sim.py", "--config", str(CONFIGS[0]),
                         "--profile", "test", "--no-hand", "--motors", "jetson",
                         "--jetson-host", "127.0.0.1"],
                        # ★ 안내문 **마지막 줄**까지 기다린다. 첫 줄에서 멈추면
                        #   뒤에 오는 조치 안내를 못 읽는다.
                        want="전체 팔(5-DOF)이면", timeout=25.0)
        assert "Traceback" not in log, f"거부가 아니라 크래시했다:\n{log[-2500:]}"
        assert "로봇이 안 맞습니다" in log, f"조용히 지나갔다:\n{log[-2500:]}"
        # ★ 팔 식별자로 잡혔는지(관절 수가 아니라) 확인 — 더 확실한 검사다
        assert "robot_arm_temp.urdf" in log and "robot_arm.urdf" in log, \
            f"팔 식별자로 안 잡고 관절 수로만 잡았다:\n{log[-2500:]}"
        assert "--temp" in log, "어떻게 고치는지 안내가 없다"
    finally:
        try:
            os.killpg(os.getpgid(jet.pid), signal.SIGKILL)
        except Exception:
            pass
