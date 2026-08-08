"""실행 프로파일 — 포트 블록 분리.

같은 저장소에서 두 갈래가 동시에 돈다.

    jetson   Quest → Mac → UDP → 젯슨 → CAN → 실물 모터
    isaac    Quest → Mac → UDP → Isaac Sim 브릿지 → 시뮬 로봇

프로토콜·IK·좌표변환은 **같은 코드를 공유한다**. 다른 것은 받는 쪽뿐이다.
그래서 코드를 복제하지 않고 포트만 분리한다. 복제하면 한쪽에서 고친 버그가
다른 쪽에 안 옮겨가고, 결국 "테스트는 통과했는데 실제로는 안 붙는" 상황이 된다.

포트는 프로파일 번호 × 10 을 더해서 만든다.

    프로파일   웹(HTTPS)  UDP 지령  UDP 상태  UDP 비컨  Meshcat
    ----------------------------------------------------------------
    jetson       4443       5005      5006      5007      7000
    isaac        4453       5015      5016      5017      7010
    test         4523       5085      5086      5087      7080

※ 웹 포트가 다르면 Quest 에서 여는 주소도 달라진다. 헤드셋은 한 번에 한
  세션만 쓰므로, 두 서버를 동시에 띄워두고 **주소로 골라서** 들어가면 된다.
"""

from __future__ import annotations

import socket
from dataclasses import dataclass

# 프로파일 이름 → 슬롯 번호. 슬롯이 포트 오프셋(×10)을 정한다.
PROFILE_SLOTS: dict[str, int] = {
    "jetson": 0,   # 실물 로봇 (기본)
    "isaac": 1,    # Isaac Sim
    "test": 8,     # 사람이 손으로 돌려보는 용도
}

BASE_WEB = 4443
BASE_CMD = 5005
BASE_STATE = 5006
BASE_BEACON = 5007
BASE_MESHCAT = 7000


@dataclass(frozen=True)
class Ports:
    name: str
    slot: int
    web: int
    cmd: int
    state: int
    beacon: int
    meshcat: int

    def as_dict(self) -> dict[str, int]:
        return {"web": self.web, "cmd": self.cmd, "state": self.state,
                "beacon": self.beacon, "meshcat": self.meshcat}


def ports(profile: str = "jetson") -> Ports:
    """프로파일 이름 → 포트 블록."""
    if profile not in PROFILE_SLOTS:
        raise ValueError(
            f"모르는 프로파일 '{profile}'. 쓸 수 있는 것: {', '.join(PROFILE_SLOTS)}"
        )
    slot = PROFILE_SLOTS[profile]
    off = slot * 10
    return Ports(name=profile, slot=slot,
                 web=BASE_WEB + off, cmd=BASE_CMD + off, state=BASE_STATE + off,
                 beacon=BASE_BEACON + off, meshcat=BASE_MESHCAT + off)


def owner_of(port: int) -> str | None:
    """이 포트가 어느 프로파일 것인지. 모르면 None.

    포트가 물려 있을 때 "누가 쓰고 있는지"를 알려주기 위한 것이다.
    "Address already in use" 만 뜨면 상대가 누군지 몰라 한참 헤맨다.
    """
    for name in PROFILE_SLOTS:
        p = ports(name)
        for role, val in p.as_dict().items():
            if val == port:
                return f"{name}/{role}"
    return None


def _tcp_busy(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("", port))
            return False
        except OSError:
            return True


def _udp_busy(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        # ★ SO_REUSEADDR 을 켜지 않는다. UDP 는 이 옵션이 켜져 있으면 두 프로세스가
        #   같은 포트에 바인드되고, 패킷이 둘 중 아무데나 가서 조용히 절반씩
        #   사라진다. "가끔 명령이 씹힌다" 로 나타나는데 원인 찾기가 지독하다.
        try:
            s.bind(("", port))
            return False
        except OSError:
            return True


def check_free(p: Ports, need_web: bool = True, need_udp: bool = True) -> list[str]:
    """쓰려는 포트가 비어 있는지 미리 본다. 막힌 것들의 설명을 돌려준다."""
    problems: list[str] = []
    if need_web and _tcp_busy(p.web):
        problems.append(f"TCP {p.web} (웹/WebSocket)")
    if need_udp:
        for role, port in (("상태 수신", p.state), ("비컨 수신", p.beacon)):
            if _udp_busy(port):
                problems.append(f"UDP {port} ({role})")
    return problems


def explain_conflict(p: Ports, problems: list[str]) -> str:
    """사람이 읽고 바로 조치할 수 있는 메시지."""
    lines = [f"❌ 프로파일 '{p.name}' 의 포트가 이미 쓰이고 있습니다:"]
    for prob in problems:
        lines.append(f"     {prob}")
    lines.append("")
    lines.append("  같은 프로파일로 두 번 띄웠거나, 다른 갈래가 포트를 잘못 쓰고 있습니다.")
    lines.append("  지금 무엇이 물고 있는지 보려면:")
    lines.append(f"     lsof -nP -iTCP:{p.web} -sTCP:LISTEN")
    lines.append(f"     lsof -nP -iUDP:{p.state}")
    lines.append("")
    lines.append("  프로파일별 포트:")
    for name in PROFILE_SLOTS:
        q = ports(name)
        lines.append(f"     {name:8s} 웹 {q.web}  지령 {q.cmd}  상태 {q.state}  "
                     f"비컨 {q.beacon}  meshcat {q.meshcat}")
    return "\n".join(lines)


def table() -> str:
    """시작 화면에 찍을 요약표."""
    out = ["  프로파일   웹(HTTPS)  UDP지령  UDP상태  UDP비컨  Meshcat"]
    out.append("  " + "-" * 58)
    for name in PROFILE_SLOTS:
        p = ports(name)
        out.append(f"  {name:9s}  {p.web:^9d}  {p.cmd:^7d}  {p.state:^7d}  "
                   f"{p.beacon:^7d}  {p.meshcat:^7d}")
    return "\n".join(out)
