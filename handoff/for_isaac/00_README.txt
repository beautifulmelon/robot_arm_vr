================================================================================
  VR 텔레옵 → Isaac Sim 연동 패키지
  보낸이 : VR 텔레옵 담당 (Mac)
  받는이 : Isaac Sim 담당
  날짜   : 2026-08-07
================================================================================


 읽는 순서
--------------------------------------------------------------------------------
  1. 18_VR_TELEOP_REPLY.txt      ★ 본문. VR_TELEOP_REQUIREMENTS.txt 에 대한 회신.
                                   §1 을 먼저 읽으세요 — 구조가 예상과 다릅니다.
  2. protocol/jetson_link.py       프로토콜 원본. 그대로 import 하시면 됩니다.
  3. test/08_fake_mac.py           저 없이 브릿지를 검증하는 송신기.


 한 줄 요약
--------------------------------------------------------------------------------
  Mac 이 **손 포즈가 아니라 관절각 5개(rad)** 를 UDP 로 보냅니다.
  IK · 클러치 · 좌표변환 · 작업공간 배율은 전부 Mac 에 이미 있습니다.
  ★ Lula IK 나 robot_description YAML 을 만들지 마세요. 헛수고입니다.
    이유는 본문 §1.2 — 두 개의 IK 를 두면 시뮬과 실물이 갈라집니다.


 ★ 2026-08-07 오후 정정 — 포트가 바뀌었습니다 ★
--------------------------------------------------------------------------------
  이 패키지 초판에는 UDP **5005 / 5006** 으로 적혀 있었습니다.
  **5015 / 5016** 입니다.

      브릿지가 bind        UDP 5015   (지령 수신)
      브릿지가 되돌려 보냄  UDP 5016   (상태 송신)

  이 저장소에서 실물 젯슨 갈래와 Isaac 갈래가 같은 코드로 동시에 돕니다.
  Mac 쪽이 상태 포트를 bind 해서 받는데, 두 갈래가 같은 포트를 잡으면 UDP 는
  예외도 없이 패킷을 절반씩 나눠 먹습니다. 그래서 포트 블록을 나눴습니다.

      프로파일   웹     지령   상태   비컨
      -----------------------------------------
      jetson    4443   5005   5006   5007    실물 젯슨
      isaac     4453   5015   5016   5017    ← 그쪽
      test      4523   5085   5086   5087

  → protocol/profiles.py 를 쓰시면 숫자를 안 외워도 됩니다.
    자세한 배경은 본문 §2.1.


 파일
--------------------------------------------------------------------------------
  00_README.txt                이 파일
  18_VR_TELEOP_REPLY.txt       본문 (질문 24개 답변 + 프로토콜 규격 + 진행 순서)

  protocol/
    __init__.py                패키지로 쓰기 위한 빈 파일
    jetson_link.py             ★ Command / State / Beacon 정의.
                                 to_bytes() / from_bytes() 가 들어 있습니다.
                                 ★ 사양서를 보고 재구현하지 마세요 —
                                   미묘하게 어긋나면 "테스트는 통과했는데
                                   실제로는 안 붙는" 상황이 됩니다.
    motor_backend.py           jetson_link 가 import 합니다 (같이 필요).
                                 SafetyLayer 도 들어 있지만 ★ 쓰지 마세요.
                                 Isaac 의 드라이브가 그 역할을 이미 합니다.
                                 (본문 §2.5 참고)
    profiles.py                ★ 포트 블록. profiles.ports("isaac") 로 씁니다.
                                 의존성 없음 (socket + dataclasses 만).
                                 숫자를 코드에 직접 박지 마세요.

  test/
    08_fake_mac.py             Quest 없이 지령을 흘려보내는 송신기.
                                 진짜 Mac 과 같은 객체를 같은 순서로 부릅니다.
                                 ※ Mac 에서 돌리는 게 맞습니다 (numpy + config 필요).
                                   시간 맞춰 같이 돌려보시죠.


 최소 구현 (이것만 하면 1단계 끝)
--------------------------------------------------------------------------------
    import socket, sys
    sys.path.insert(0, "<이 폴더>")
    from protocol.jetson_link import Command, State, MODE_RUN, STATE_RUN, STATE_HOLD
    from protocol import profiles

    PF = profiles.ports("isaac")          # cmd 5015 / state 5016

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("", PF.cmd)); rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 매 시뮬 스텝:
    #   1) 버퍼를 끝까지 비우고 '가장 최신 패킷만' 사용 (이미 그렇게 하고 계십니다)
    #   2) seq 가 역행하면 버림
    #   3) session 이 바뀌면 seq 추적 리셋 + await_rearm=True 로 HOLD 처럼 행동
    #   4) mode == "RUN" 이면 cmd.q[0..4] 를 팔 드라이브 목표로 (라디안 그대로)
    #      mode == "HOLD" 면 목표를 현재 위치로 얼림
    #   5) 실제 관절각을 State 로 만들어 **지령이 온 송신지 주소**로 PF.state 송신
    #        tx.sendto(State(...).to_bytes(), (src_ip, PF.state))

    ★ 워치독은 시뮬 값으로: freeze 250ms / lost 1s / trip 5s  (본문 §2.5)
    ★ 소프트스타트·속도제한·관절한계를 직접 걸지 마세요 (본문 §2.5)


 검증 순서 제안
--------------------------------------------------------------------------------
  1) 브릿지만 띄우고 Mac 쪽에서 08_fake_mac.py 를 돌린다
     .venv/bin/python scripts/08_fake_mac.py --host 100.93.186.122 --profile isaac
     → 5016 상태 패킷이 Mac 에 도착하는지, 관절이 사인파로 흔들리는지

  2) --sweep one-by-one 으로 관절 하나씩
     → 어느 관절이 안 도는지, 순서 매핑이 맞는지 눈으로 분리

  3) Mac 쪽 08_fake_mac.py 를 Ctrl+C 로 죽여본다
     → 5초 뒤 TRIP 이 뜨는지. 다시 띄우면 session 이 바뀌므로
       await_rearm=True 로 RUN 을 한 번 거부해야 정상

  4) 그 다음에 진짜 Quest 연결   ★ --profile isaac 을 빠뜨리지 말 것
     ./run.sh --motors jetson --profile isaac --jetson-host 100.93.186.122

================================================================================
