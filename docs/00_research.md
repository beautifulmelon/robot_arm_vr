# 조사 노트 — VR 텔레오퍼레이션 + LeRobot 모방학습

작성일: 2026-08-01

---

## 1. `Roboparty/roboparty_xr_teleop` 분석

Unitree `xr_teleoperate` 포크. **RPO / Roboto Origin 휴머노이드 전신**용 양팔 텔레오퍼레이션.

### 스택
- Ubuntu 20.04/22.04 + Python 3.10 + **ROS2** (`rclpy`, `sensor_msgs`)
- IK: `pinocchio` + `casadi` (IPOPT), 시각화 `meshcat`
- XR 입력: `televuer/` (Vuer 기반 **WebXR**) → HTTPS `:8012` 웹페이지를 헤드셋 브라우저로 접속
- 기본 30 Hz 제어 루프

### 데이터 흐름
```
PICO/WebXR 컨트롤러 6-DOF pose
  → televuer (WebSocket)
  → RPOArmIK.solve_ik(left_wrist, right_wrist)   # casadi/IPOPT, max_iter=20
  → JointState 퍼블리시 → ROS2 토픽 `/joint_ref_states`
  → roboparty_deploy(로봇 측)가 구독해서 모터 구동
```

### 팔 구조 (URDF `assets/Atom01_urdf/urdf/atom01.urdf`)
한쪽 팔 = **5 DOF**, 그리퍼/손목 롤 없음:

| # | joint | axis | limit (rad) | effort |
|---|-------|------|-------------|--------|
| 0 | `*_arm_pitch_joint`   | 0 1 0  | ±2.0        | 18 |
| 1 | `*_arm_roll_joint`    | 1 0 0  | -0.25~2.25 (L) | 18 |
| 2 | `*_arm_yaw_joint`     | 0 0 -1 | ±2.6        | 18 |
| 3 | `*_elbow_pitch_joint` | 0 1 0  | -1.0~1.57   | 18 |
| 4 | `*_elbow_yaw_joint`   | 1 0 0  | ±1.57       | 18 |

EE 프레임은 `elbow_yaw_joint`에서 +x 방향 0.15 m 지점(`L_ee`/`R_ee`)에 수동 추가.

> ⚠️ **중요**: [`xr_control_rpo.py:78-81`](../teleop/xr_control_rpo.py) 에서 `command_q[4] = 0.0`, `command_q[9] = 0.0` —
> 즉 `elbow_yaw`(손목 롤)는 **항상 0으로 강제**. 실제 명령되는 건 팔당 **4 DOF**.
> 모터 이름은 `left_motor0..4`, `right_motor0..4`.

### 전신 의존성 (팔만 쓸 때 걸림돌)
- `roboparty_deploy`의 밸런스 policy(`inference_interrupt.yaml`)가 돌아가고 있어야 함
- 로봇 리모컨 `X`(모터 인에이블) → `A`(리셋) → `B`(추론 모드) → `LB`(상체 인터페이스 개방) 시퀀스 필요
- **팔만 단독 사용 시 이 전신 스택 전체가 불필요** → 모터 드라이버만 직접 쓰는 게 훨씬 깔끔

### 관련 Roboparty 저장소
| repo | 내용 |
|------|------|
| `roboparty_motors` | C++ 모터 드라이버 + **pybind 파이썬 바인딩**. 드라이버: `dm`(Damiao), `evo`, `lro`, `xyn`. 프로토콜: CAN / CAN-FD / EtherCAT |
| `rpo_hardware` | 기구 CAD, PCB, BOM (V1.0 `atom01_*` / V2.0 `roboto_origin_*`) |
| `roboparty_deploy` | ROS2 배포 프레임워크 (드라이버 + 추론) |
| `roboparty_hand` | 핸드 관련 (C++) |
| `roboto_origin` | 완전 오픈소스 DIY 휴머노이드 |

---

## 2. `huggingface/lerobot` 분석

### 결론: LeRobot에 **XR 전용 teleoperator는 아직 없음**
`src/lerobot/teleoperators/` 목록: `bi_openarm_leader`, `bi_openarm_mini`, `bi_rebot_102_leader`,
`bi_so_leader`, `gamepad`, `homunculus`, `keyboard`, `koch_leader`, `omx_leader`, `openarm_leader`,
`openarm_mini`, **`phone`**, `reachy2_teleoperator`, `rebot_102_leader`, `so_leader`, `unitree_g1`

### ✅ 핵심 발견 — `phone` teleoperator가 사실상 XR 경로다
`PhoneOS.ANDROID` 모드는 **SpesRobotics `teleop`** 패키지(WebXR)를 씀:
`src/lerobot/teleoperators/phone/teleop_phone.py:251` → `Teleop()` → `teleop.subscribe(callback)`

그리고 `teleop` 패키지의 `index.html`을 확인한 결과 **이미 VR 컨트롤러를 지원**한다:

```js
// teleop/index.html
navigator.xr.requestSession('immersive-ar', {
  optionalFeatures: ['hand-tracking', 'unbounded', 'dom-overlay', ...] })
...
if (inputSource.handedness === 'right' && inputSource.targetRayMode === 'tracked-pointer') {
    const pose = frame.getPose(inputSource.targetRaySpace, referenceSpace);
}
...
gamepad.buttons[1].pressed  → move (텔레옵 활성화, 트리거/그립)
gamepad.buttons[0].pressed  → toggle
gamepad.buttons[4] / [5]    → reservedButtonA / reservedButtonB  (Quest A/B 버튼)
gamepad.axes[0]             → scale (썸스틱)
setMessage(isVRDevice ? 'VR controllers detected' : 'Using device pose')
```

→ **Meta Quest 2 브라우저(Chromium, WebXR `immersive-ar` 패스스루 지원)로 그대로 접속 가능.**
→ 오른손 컨트롤러 1개만 읽음 = 팔 1개 조종에 정확히 부합.

### LeRobot EE-pose 텔레옵 파이프라인 (processor steps)
```
MapPhoneActionToRobotAction   # 폰/컨트롤러 pose → target_x/y/z 델타
  → EEReferenceAndDelta       # 활성화 시점 EE pose를 기준으로 절대 목표 pose 생성
  → EEBoundsAndSafety         # 워크스페이스 클램프 + max_ee_step_m 레이트 리밋
  → InverseKinematicsEEToJoints   # URDF 기반 IK → 관절각
  → GripperVelocityToJoint    # 그리퍼 속도 → 절대 위치 적분
  → ForwardKinematicsJointsToEE   # 실제 관절각 → 관측 EE pose (학습용 로깅)
```
- 참조 구현: `src/lerobot/robots/so_follower/robot_kinematic_processor.py` (26 KB), `src/lerobot/model/kinematics.py`
- 예제: `examples/phone_to_so100/{teleoperate,record,replay,evaluate}.py` + URDF 배치 필요
- 설치: `pip install lerobot[phone]`

### 모방학습 워크플로우 (CLI)
```bash
lerobot-teleoperate --robot.type=... --teleop.type=... --display_data=true
lerobot-record  --robot.type=... --teleop.type=... \
                --dataset.repo_id=$HF_USER/my-task --dataset.num_episodes=50 \
                --dataset.single_task="..." --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}"
lerobot-replay  --dataset.repo_id=$HF_USER/my-task --dataset.episode=0
lerobot-train   --dataset.repo_id=$HF_USER/my-task --policy.type=act \
                --output_dir=outputs/train/act_test --policy.device=cuda
lerobot-rollout --strategy.type=base --policy.path=$HF_USER/my_policy --robot.type=...
```
녹화 중 키: `→`/`n` 다음 에피소드, `←`/`r` 재녹화, `Esc`/`q` 종료+업로드
데이터 저장 위치: `~/.cache/huggingface/lerobot/{repo-id}`

### 커스텀 로봇 추가
`Robot` 서브클래스 구현 필요 — `connect/disconnect`, `get_observation()`, `send_action()`,
`observation_features` / `action_features` 딕셔너리. 참고: `docs/source/integrate_hardware.mdx`,
`src/lerobot/robots/so_follower/so_follower.py`

---

## 3. Amazing Hand (Pollen Robotics)

- **8 DOF**, 손가락 4개 × 2축(굴곡/신전 + 제한적 벌림), 병렬 링크
- 액추에이터: **Feetech SCS0009** 8개, 전부 손바닥 안에 내장 (케이블 없음)
- 무게 ~400 g, 완전 3D 프린팅(강성 프레임 + TPU 외피), 비용 < €200
- 저장소: `pollen-robotics/AmazingHand` (Python + Arduino 스택 오픈소스)
- 손목 인터페이스는 Reachy2 기준 → **RPO 팔 끝단 어댑터는 자체 설계 필요**

### 🎯 좋은 소식
Feetech 버스 서보 = **LeRobot이 이미 `lerobot.motors.feetech`로 지원하는 계열**.
SO-100/SO-101이 STS3215를 쓰고, SCS0009는 같은 SCS/STS 프로토콜 패밀리.
→ 팔(CAN) + 손(Feetech serial)을 하나의 LeRobot `Robot` 클래스 안에서 합쳐
   `action` 벡터를 `[arm_j0..j4, hand_j0..j7]` (13-dim)로 구성 가능.

---

## 4. 아키텍처 판단

### 경로 A — LeRobot 네이티브 (권장)
```
Quest 2 브라우저 (WebXR immersive-ar)
  → SpesRobotics teleop 패키지 (HTTPS :5000)
  → LeRobot Teleoperator (phone/ANDROID 또는 커스텀 XRTeleoperator)
  → LeRobot processor pipeline (EEReferenceAndDelta → Bounds → IK)
  → 커스텀 LeRobot Robot 클래스 (RPO 팔 5-DOF + Amazing Hand 8-DOF)
  → roboparty_motors pybind (CAN) + feetech (serial)
```
- ✅ ROS2 불필요, `lerobot-record`/`train`/`rollout` 전부 그대로 사용
- ✅ 데이터셋이 LeRobotDataset v3 포맷으로 바로 나옴
- ✅ Quest 2 컨트롤러 버튼이 이미 매핑되어 있음 (A/B → 손 개폐로 전용 가능)
- ⚠️ `Robot` 클래스 + IK 설정 직접 구현 필요

### 경로 B — roboparty ROS2 스택 유지 + LeRobot 브릿지
```
Quest 2 → televuer → RPOArmIK → ROS2 /joint_ref_states → roboparty_deploy
                                        ↕
                       LeRobot Robot 클래스가 ROS2 토픽 구독/발행
```
- ✅ 기존 IK/텔레옵 코드 재사용
- ❌ Ubuntu + ROS2 + 전신 밸런스 policy 필요 (팔만 쓸 거면 과잉)
- ❌ 레이턴시 한 단계 추가

**→ 경로 A 권장.** 단, roboparty의 URDF와 IK 설정값(팔 길이 스케일 0.50/0.45 등)은 그대로 참고.

---

## 5. 확인 필요 사항 (하드웨어)

1. 실제 보유 하드웨어: RPO 전신 / 팔 모듈만 / 아직 없음
2. 팔 모터 종류 + 통신 (Damiao CAN? USB2CAN 보드?)
3. 로봇 구동 PC (Ubuntu? Jetson? 현재 개발 머신은 macOS 26 arm64 / Python 3.9)
4. 현재 팔 끝단 상태 (그리퍼 유무)
5. 카메라 (모방학습에 필수 — 최소 1대, 보통 손목 + 정면 2대)
