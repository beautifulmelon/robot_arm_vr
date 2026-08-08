"""자체 서명 TLS 인증서 생성/검증.

WebXR은 secure context(HTTPS)에서만 동작한다. teleop 패키지에 동봉된 cert.pem은
2025-07-28에 만료되어 그대로 쓸 수 없으므로, 접속에 사용할 IP를 SAN에 넣은
인증서를 직접 만들어 쓴다.

SAN(subjectAltName)에 IP를 넣어두면 Chromium 계열(Quest 브라우저)에서 경고 화면이
`ERR_CERT_AUTHORITY_INVALID` 하나로 줄어들어 "고급 → 계속"으로 넘어가기 쉬워진다.
"""

from __future__ import annotations

import datetime as _dt
import socket
import contextlib
import os
import shutil
import tempfile
import time
import subprocess
from pathlib import Path

CERT_DIR = Path(__file__).resolve().parents[2] / "certs"
CERT_FILE = CERT_DIR / "cert.pem"
KEY_FILE = CERT_DIR / "key.pem"

_VALID_DAYS = 825  # 브라우저가 수용하는 자체 서명 인증서 최대 기간


def get_local_ip() -> str:
    """외부로 나가는 인터페이스에 붙은 LAN IP를 얻는다 (패킷은 실제로 보내지 않음)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def list_local_ips() -> list[str]:
    """이 머신의 모든 사설 IPv4. 기본 경로 IP 를 맨 앞에 둔다.

    핫스팟/DHCP 환경에서는 IP 가 자주 바뀌는데, 인증서 SAN 에 하나만 넣어두면
    바뀔 때마다 재발급이 필요하고 헤드셋에서는 접속 주소까지 달라진다.
    잡히는 IP 를 전부 넣어두면 같은 망 안에서 주소가 바뀌어도 인증서는 계속 유효하다.
    """
    ips: list[str] = []
    primary = get_local_ip()
    if primary != "127.0.0.1":
        ips.append(primary)

    try:
        out = subprocess.run(["ifconfig"], capture_output=True, text=True, check=True).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ips

    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "inet":
            ip = parts[1]
            if ip.startswith("127.") or ip in ips:
                continue
            # 사설 대역만 (공인 IP 를 인증서에 넣을 이유가 없다)
            if ip.startswith(("10.", "192.168.")) or (
                ip.startswith("172.") and 16 <= int(ip.split(".")[1]) <= 31
            ):
                ips.append(ip)
    return ips


def _cert_days_left(cert_file: Path) -> float | None:
    """인증서 남은 유효일수. 파싱 실패 시 None."""
    try:
        out = subprocess.run(
            ["openssl", "x509", "-in", str(cert_file), "-noout", "-enddate"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None

    # notAfter=Jul 28 14:54:46 2025 GMT
    try:
        stamp = out.split("=", 1)[1].strip()
        expiry = _dt.datetime.strptime(stamp, "%b %d %H:%M:%S %Y %Z").replace(
            tzinfo=_dt.timezone.utc
        )
    except (IndexError, ValueError):
        return None

    return (expiry - _dt.datetime.now(_dt.timezone.utc)).total_seconds() / 86400.0


def _cert_covers_ip(cert_file: Path, ip: str | list[str]) -> bool:
    """인증서 SAN에 해당 IP(들)가 전부 들어있는지."""
    try:
        out = subprocess.run(
            ["openssl", "x509", "-in", str(cert_file), "-noout", "-text"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False
    wanted = [ip] if isinstance(ip, str) else ip
    return all(f"IP Address:{x}" in out for x in wanted)


@contextlib.contextmanager
def _cert_lock(timeout: float = 60.0):
    """인증서 생성 구간을 프로세스 사이에서 직렬화한다.

    O_EXCL 로 만든 파일이 락이다. 만들어져 있으면 다른 프로세스가 생성 중이니
    기다린다. 죽은 프로세스가 남긴 락은 오래되면 무시한다(스테일 정리).
    """
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    lock = CERT_DIR / ".cert.lock"
    start = time.monotonic()
    fd = None
    while True:
        try:
            fd = os.open(str(lock), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            break
        except FileExistsError:
            age = time.time() - lock.stat().st_mtime if lock.exists() else 1e9
            if age > 120:                      # 죽은 프로세스가 남긴 락
                lock.unlink(missing_ok=True)
                continue
            if time.monotonic() - start > timeout:
                raise TimeoutError(f"인증서 락 대기 시간 초과: {lock}")
            time.sleep(0.2)
    try:
        yield
    finally:
        if fd is not None:
            os.close(fd)
        lock.unlink(missing_ok=True)


def ensure_cert(ip: str | None = None, force: bool = False) -> tuple[Path, Path]:
    """유효한 cert/key 쌍을 보장하고 경로를 돌려준다.

    Args:
        ip: SAN에 반드시 넣을 IP. None이면 자동 감지.
             이 값과 무관하게 이 머신의 사설 IPv4 는 전부 SAN 에 들어간다.
        force: 기존 인증서가 멀쩡해도 새로 발급.

    Returns:
        (cert_file, key_file)
    """
    ips = list_local_ips()
    if ip and ip not in ips:
        ips.insert(0, ip)
    if not ips:
        ips = ["127.0.0.1"]
    CERT_DIR.mkdir(parents=True, exist_ok=True)

    if not force and CERT_FILE.exists() and KEY_FILE.exists():
        days = _cert_days_left(CERT_FILE)
        if days is not None and days > 7 and _cert_covers_ip(CERT_FILE, ips):
            return CERT_FILE, KEY_FILE

    alt = "\n".join(f"IP.{i + 1}  = {v}" for i, v in enumerate([*ips, "127.0.0.1"]))
    config = f"""
[req]
distinguished_name = dn
x509_extensions    = v3_req
prompt             = no

[dn]
C  = KR
O  = robot_arm_vr
CN = {ips[0]}

[v3_req]
basicConstraints = CA:FALSE
keyUsage         = digitalSignature, keyEncipherment
extendedKeyUsage = serverAuth
subjectAltName   = @alt_names

[alt_names]
{alt}
DNS.1 = localhost
"""
    # ★ 두 서버(실물 갈래 / Isaac Sim 갈래)가 동시에 뜨면 여기서 부딪힌다.
    #   같은 파일에 곧바로 쓰면 한쪽이 반쯤 쓰인 파일을 읽어서 TLS 가 깨지고,
    #   증상이 "가끔 인증서 오류" 로 나타나 원인을 찾기 어렵다.
    #   임시 파일에 만든 뒤 os.replace 로 갈아끼운다. rename 은 원자적이라
    #   읽는 쪽은 항상 옛 파일 아니면 새 파일을 본다. 반쯤은 없다.
    with _cert_lock():
        # 락을 잡고 다시 확인한다. 기다리는 동안 다른 프로세스가 이미
        # 만들어놨을 수 있고, 그러면 두 번 만들 이유가 없다.
        if not force and CERT_FILE.exists() and KEY_FILE.exists():
            days = _cert_days_left(CERT_FILE)
            if days is not None and days > 7 and _cert_covers_ip(CERT_FILE, ips):
                return CERT_FILE, KEY_FILE

        tmp = tempfile.mkdtemp(prefix="cert_", dir=str(CERT_DIR))
        try:
            conf_file = Path(tmp) / "openssl.cnf"
            conf_file.write_text(config.strip() + "\n")
            tmp_cert, tmp_key = Path(tmp) / "server.crt", Path(tmp) / "server.key"
            subprocess.run(
                [
                    "openssl", "req", "-x509", "-nodes",
                    "-newkey", "rsa:2048",
                    "-days", str(_VALID_DAYS),
                    "-keyout", str(tmp_key),
                    "-out", str(tmp_cert),
                    "-config", str(conf_file),
                ],
                check=True,
                capture_output=True,
            )
            os.replace(tmp_key, KEY_FILE)     # 원자적 교체
            os.replace(tmp_cert, CERT_FILE)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    return CERT_FILE, KEY_FILE
