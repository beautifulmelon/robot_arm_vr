================================================================================
  VR 텔레옵 → Isaac Sim 연동 패키지
  보낸이 : VR 텔레옵 담당 (Mac)
  받는이 : Isaac Sim 담당
  날짜   : 2026-08-07
================================================================================


 ★★ 2026-09-02 — 지금 할 일은 69번입니다. 여기부터 읽으세요
--------------------------------------------------------------------------------
  이 폴더는 8월부터 쌓인 것이라 파일이 많습니다. **지금 유효한 것만 추리면:**

    69_ISAAC_VERIFICATION_REQUEST.txt   ★ 이번 요청. 검증 5가지 (V-1~V-5)
    body/body_with_arm_v2_nomimic.urdf  ★ 이걸로 USD 를 뽑으세요 (신규 팔+그리퍼)
    config/arm_v2.json                  홈 자세 · 리밋 · 리치
    tools/gripper_map.py                grasp → 서보각 변환 (실측 56점 피팅)
    ARM_VERSIONS.md                     ★ joint1 영점이 180° 옮겨졌습니다
    67_ARM_NEW2_HANDOFF.txt             기구 담당 원본. §7 함정 필독
    62_CAMERA_SPEC.txt                  카메라 규격 (D435i 는 69°, 87° 아님)

  ★★ 바뀐 것 세 가지 — 이전 자료와 다릅니다
      1) 팔이 **신규 5축 + 평행 그리퍼** 로 바뀌었습니다. 구동관절 17 → 9
         (팔 5 + 그리퍼 4). 손(AmazingHand)은 이제 안 씁니다
      2) **joint1 영점이 180° 옮겨졌습니다.** 첨부 URDF 는 새 영점입니다.
         홈은 joint1 = 0° 이고 팔이 정확히 정면을 봅니다. ARM_VERSIONS.md 참고
      3) Command 에 **grasp(0~1)** 가 항상 실려 옵니다. servo(8개)는 이제 안 옵니다

  ※ 61 · 63 의 커버리지·박스 위치 숫자는 **옛날 팔** 기준이라 폐기입니다.
    63 의 "카메라 정면 그대로" 도 무효 — 각도를 정하는 것이 69번 V-1 입니다.
  ※ 70_MECH_REQUEST.txt 는 기구 담당에게 보낸 것입니다. 참고용으로 넣었습니다
    (카메라 각도·쓰레기통 높이가 거기서 정해집니다).


 읽는 순서 (아래는 8월 초판 그대로입니다 — 배경용)
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
