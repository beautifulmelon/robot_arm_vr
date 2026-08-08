"""프로파일 포트 분리 검증.

같은 저장소에서 실물 갈래(jetson)와 Isaac Sim 갈래(isaac)가 동시에 돈다.
포트가 겹치면 늦게 뜬 쪽이 죽거나, UDP 는 조용히 패킷을 나눠 먹는다.
"""

from __future__ import annotations

import socket
import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rpo_teleop import profiles  # noqa: E402


def scratch_ports() -> profiles.Ports:
    """검사 로직만 보는 테스트용 임시 포트.

    ★ 실제 프로파일('test')의 고정 포트를 쓰면, 이 파일이 저장소와 젯슨 전달
      묶음 양쪽에 있어서 두 pytest 를 동시에 돌릴 때 서로 부딪힌다.
      OS 에게 빈 포트를 받는다.
    """
    socks, got = [], []
    for _ in range(5):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("", 0))
        socks.append(s); got.append(s.getsockname()[1])
    for s in socks:
        s.close()
    return profiles.Ports(name="scratch", slot=-1, web=got[0], cmd=got[1],
                          state=got[2], beacon=got[3], meshcat=got[4])


def test_no_port_collision_between_profiles():
    """어떤 두 프로파일도 포트를 하나도 공유하면 안 된다."""
    seen: dict[int, str] = {}
    for name in profiles.PROFILE_SLOTS:
        p = profiles.ports(name)
        for role, port in p.as_dict().items():
            assert port not in seen, \
                f"{name}/{role} 포트 {port} 가 {seen[port]} 와 겹친다"
            seen[port] = f"{name}/{role}"


def test_owner_of_identifies_profile():
    """포트가 물려 있을 때 누구 것인지 알 수 있어야 한다."""
    assert profiles.owner_of(4443) == "jetson/web"
    assert profiles.owner_of(5016) == "isaac/state"
    assert profiles.owner_of(9999) is None


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError, match="모르는 프로파일"):
        profiles.ports("없는것")


def test_check_free_detects_busy_tcp():
    p = scratch_ports()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", p.web))
    srv.listen(1)
    try:
        problems = profiles.check_free(p, need_udp=False)
        assert any(str(p.web) in x for x in problems), f"막힌 포트를 못 잡았다: {problems}"
        msg = profiles.explain_conflict(p, problems)
        assert "lsof" in msg and "jetson" in msg, "조치 방법이 안내되지 않는다"
    finally:
        srv.close()


def test_check_free_detects_busy_udp():
    p = scratch_ports()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", p.state))
    try:
        problems = profiles.check_free(p, need_web=False)
        assert any(str(p.state) in x for x in problems), f"막힌 UDP 를 못 잡았다: {problems}"
    finally:
        s.close()


def test_check_free_passes_when_idle():
    assert profiles.check_free(scratch_ports()) == []


# ── 인증서 동시 생성 ───────────────────────────────────────────────────
def test_cert_generation_is_race_safe(tmp_path, monkeypatch):
    """두 서버가 동시에 뜨면 같은 인증서 파일에 부딪힌다.

    곧바로 쓰면 한쪽이 반쯤 쓰인 파일을 읽어 TLS 가 깨진다. 증상이 "가끔
    인증서 오류" 라 원인 찾기가 어렵다. 원자적 교체 + 락으로 막는다.
    """
    # certs 는 젯슨 전달 묶음에 안 들어간다 (그쪽은 서버를 안 띄운다).
    certs = pytest.importorskip("rpo_teleop.certs")

    monkeypatch.setattr(certs, "CERT_DIR", tmp_path)
    monkeypatch.setattr(certs, "CERT_FILE", tmp_path / "cert.pem")
    monkeypatch.setattr(certs, "KEY_FILE", tmp_path / "key.pem")

    results, errors = [], []

    def worker():
        try:
            results.append(certs.ensure_cert(force=True))
        except Exception as exc:      # noqa: BLE001
            errors.append(exc)

    threads = [threading.Thread(target=worker) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=90)

    assert not errors, f"동시 생성에서 예외: {errors}"
    assert len(results) == 4
    cert = tmp_path / "cert.pem"
    assert cert.exists() and cert.stat().st_size > 0
    # 반쯤 쓰인 파일이 아니라 온전한 PEM 이어야 한다
    text = cert.read_text()
    assert text.startswith("-----BEGIN CERTIFICATE-----")
    assert text.rstrip().endswith("-----END CERTIFICATE-----")
    assert not (tmp_path / ".cert.lock").exists(), "락이 남아 있다"


def test_docstring_table_matches_code():
    """독스트링 표와 실제 계산값이 어긋나면 안 된다.

    사람이 표를 보고 방화벽 규칙이나 브릿지 포트를 정한다. 표가 틀리면
    "설정은 문서대로 했는데 안 붙는" 상황이 되고, 코드를 안 읽으면 못 찾는다.
    (Isaac 쪽에서 test/meshcat 이 7090 으로 적혀 있는 걸 발견)
    """
    import re
    doc = profiles.__doc__ or ""
    rows = re.findall(r"^\s{4}(\w+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$",
                      doc, re.MULTILINE)
    assert rows, "독스트링에서 포트 표를 못 찾았다"
    for name, web, cmd, state, beacon, meshcat in rows:
        p = profiles.ports(name)
        assert (p.web, p.cmd, p.state, p.beacon, p.meshcat) == \
               (int(web), int(cmd), int(state), int(beacon), int(meshcat)), \
               f"'{name}' 행이 코드와 다르다: 문서 {(web,cmd,state,beacon,meshcat)} vs 코드 {p.as_dict()}"
