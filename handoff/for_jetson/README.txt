젯슨 브리지 참조 구현
================================================================================

읽는 순서
--------------------------------------------------------------------------------
  1. ../14_JETSON_IMPLEMENTATION_NOTES.txt   ★ 먼저. 규격 버그 3건이 여기 있습니다.
  2. src/rpo_teleop/jetson_sim.py            젯슨 쪽 참조 구현. 이걸 보고 만드세요.
  3. src/rpo_teleop/jetson_link.py           프로토콜 패킷 정의 (규격 최종본)
  4. src/rpo_teleop/motor_backend.py         SafetyLayer (3단계 워치독 반영본)

바로 돌려보기  (python 3.12 + numpy 만 있으면 됩니다)
--------------------------------------------------------------------------------
  python scripts/07_fake_jetson.py                    기본
  python scripts/07_fake_jetson.py --drop 0.3         패킷 30% 손실
  python scripts/07_fake_jetson.py --blackout 14 4    14초마다 4초 통신 두절
  python scripts/07_fake_jetson.py --motors 3         3번째 모터를 붙인 뒤

  python -m pytest tests/ -q                          테스트 33개

  ※ 이 폴더 그대로 젯슨에 복사해서 돌아가는 것을 확인했습니다 (33개 통과).

바꿀 곳은 한 군데입니다
--------------------------------------------------------------------------------
  jetson_sim.py 의 FakeJetson._tick() 안에서 MockBackend 를 CAN 으로 갈아끼우면
  끝입니다. 상태머신·워치독·세션·트립·비컨은 그대로 쓰시면 됩니다.

      actual = self.motors.read_positions()      # ← CAN 피드백 디코딩
      q_cmd  = self.safety.clamp(self._q_target, actual)
      self.motors.write_positions(q_cmd)         # ← MIT 위치 명령 인코딩

여러분의 '가짜 Mac' 으로 쓰기
--------------------------------------------------------------------------------
  jetson_link.py 의 JetsonBackend 가 Mac 쪽 구현입니다. 그대로 쓰시면
  저희 없이도 젯슨 브리지를 끝까지 검증할 수 있습니다.

      from rpo_teleop.jetson_link import JetsonBackend
      be = JetsonBackend(n_joints=3, host="127.0.0.1")
      be.connect(); be.hold()
      be.write_positions([0.1, 0.2, 0.0])

  tests/test_jetson_link.py 의 FakeJetson 자리에 진짜 브리지를 넣으면
  테스트 23개가 그대로 여러분 구현의 검증이 됩니다.

파일
--------------------------------------------------------------------------------
  src/rpo_teleop/
    jetson_link.py       프로토콜 + Mac 쪽 백엔드
    jetson_sim.py        ★ 젯슨 참조 구현
    motor_backend.py     SafetyLayer / SafetyLimits / MotorBackend / MockBackend
    arm_config.py        URDF → 관절/한계/작업공간 자동 도출
    hand_model.py        손 서보↔관절 (2차용)
    servo_map/           손 서보 매핑 (2차용)
  scripts/07_fake_jetson.py    실행 스크립트
  tests/                       테스트 33개
  config/arm_temp.json         임시 3-DOF 팔 설정 (지금 실물)
  config/arm.json              전체 5-DOF 팔 설정 (나중에 옮겨갈 것)
                               ★ 관절 한계를 손으로 옮겨적지 말고 이걸 읽으세요
                                 (urdf_path 는 저희 머신 경로라 무시하세요.
                                  lower/upper/velocity/effort 만 쓰시면 됩니다)
  assets/robot_arm_temp/       URDF + 메시
