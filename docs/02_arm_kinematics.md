# RPO 팔 기구학 — 추출 및 IK 검증

측정일: 2026-08-01 · [scripts/03_extract_arm_urdf.py](../scripts/03_extract_arm_urdf.py), [scripts/04_check_ik.py](../scripts/04_check_ik.py)

---

## 1. 단일 팔 URDF 추출

원본 `atom01.urdf`(전신 23관절)에서 오른팔 5-DOF만 뽑아 독립 URDF로 만들었다.

- 출력: [assets/rpo_arm/urdf/rpo_arm_right.urdf](../assets/rpo_arm/urdf/rpo_arm_right.urdf) + 메시 5개
- `torso_link` → `base_link` 치환 (어깨 마운트 오프셋 `[0, -0.1217, 0.2052]` 보존)
- `ee_link`, `hand_mount_link` 프레임 추가 — roboparty가 코드에서 런타임에 붙이던 `R_ee`를 URDF에 명시

> roboparty는 매 실행마다 `buildReducedRobot()`으로 13개 관절을 잠근다.
> 전용 URDF를 만들어두면 그 과정이 불필요하고, LeRobot `RobotKinematics`에 경로만 넘기면 된다.

### 관절 (모터 인덱스 0~4 = `right_motor0..4`)

| # | joint | axis | limit (rad) | τ (N·m) | ω (rad/s) |
|---|---|---|---|---|---|
| 0 | `right_arm_pitch_joint` | `0 1 0` | −2.00 ~ +2.00 | 18 | 3.77 |
| 1 | `right_arm_roll_joint` | `1 0 0` | −2.25 ~ +0.25 | 18 | 3.77 |
| 2 | `right_arm_yaw_joint` | `0 0 -1` | −2.60 ~ +2.60 | 18 | 3.77 |
| 3 | `right_elbow_pitch_joint` | `0 1 0` | −1.00 ~ +1.57 | 18 | 3.77 |
| 4 | `right_elbow_yaw_joint` | `1 0 0` | −1.57 ~ +1.57 | 18 | 3.77 |

팔 링크 질량 합계 **2.481 kg**. Amazing Hand(~0.4 kg)를 붙이면 약 2.9 kg.

---

## 2. 작업 공간 (관절 범위 내 균등 샘플 20,000개)

| 축 | 범위 (base_link 기준) | 폭 |
|---|---|---|
| x | −0.435 ~ +0.438 | 0.873 m |
| y | −0.616 ~ +0.076 | 0.693 m |
| z | −0.233 ~ +0.545 | 0.777 m |

**어깨(arm_pitch)로부터 거리: 0.086 ~ 0.495 m** (중앙값 0.382)

### 스케일 계수 = **0.76**

```
팔 최대 리치 0.495 m ÷ 사람 어깨~손 리치 0.65 m = 0.76
```

사람 손 변위에 **0.76을 곱해** 로봇 EE 변위로 쓴다.

> ⚠️ roboparty의 [`scale_arms()`](../assets/atom01_src/) 는 `robot/human = 0.50/0.45 = 1.11` 로 **확대**한다.
> 이는 전신 휴머노이드가 양팔을 크게 벌리는 상황 기준이라, 단일 팔에 그대로 쓰면 **46% 과대**해서
> 워크스페이스 밖으로 자주 튀어나간다. 우리는 0.76으로 축소한다.

Quest 실측 손 이동 bbox `[0.80, 1.00, 1.01] m` 대비 팔 도달 bbox `[0.87, 0.69, 0.78] m` —
x축은 여유가 있으나 y(높이)·z 축이 부족하므로 스케일 축소가 필수.

---

## 3. IK 성능 (placo, LeRobot `RobotKinematics` 와 동일 스택)

목표 = `FK(q_target)`, 초기값 = `q_target`에 σ=0.3 rad 노이즈. 300회 시행.

| iters | 위치오차 중앙값 | p95 | 최대 | 5mm 초과 | 자세오차 | 시간 | 환산 |
|---|---|---|---|---|---|---|---|
| 1 | 29.26 mm | 114.93 | 373.3 | 276/300 | 6.23° | 0.01 ms | 113,056 Hz |
| **5** | **0.00 mm** | **0.06** | 34.6 | **9/300** | **0.00°** | **0.03 ms** | **33,649 Hz** |
| 20 | 0.00 mm | 0.00 | 33.8 | 3/300 | 0.00° | 0.11 ms | 9,245 Hz |

### 결론: **iters=5 채택**
- 30 Hz 제어 루프 예산 33.3 ms 중 IK는 **0.03 ms (0.09%)** 만 소모
- 5 mm 초과 실패 9/300 (3%)은 워크스페이스 경계·특이점 근처 목표. 실제 텔레옵에서는
  `EEBoundsAndSafety`가 앞단에서 클램프하므로 문제되지 않음
- placo는 **속도 레벨 QP**라 매 프레임 1스텝만 풀어도 추종된다. iters=5는 안전 마진

> roboparty는 casadi + IPOPT(`max_iter=20`, `tol=1e-4`)를 쓴다. 정확도는 비슷하지만
> IPOPT 한 번 푸는 데 수 ms가 걸리고 실패 시 fallback 처리가 필요하다.
> placo가 **100배 빠르고** LeRobot 표준이므로 placo로 간다.

---

## 4. `elbow_yaw`(손목 롤) 축 — 실측 확인

`elbow_yaw`를 −90° ~ +90° 회전시켰을 때:

| 항목 | 변화량 |
|---|---|
| EE **위치** | **0.000 mm** |
| EE **자세** | **90.0°** |

`elbow_yaw` 축이 `1 0 0`(x축)이고 `ee_link`가 `[+0.15, 0, 0]`에 있어 **회전축 위에 EE가 놓여 있다.**
그래서 이 관절은 EE 위치를 전혀 바꾸지 않고 자세만 바꾼다.

→ **roboparty가 `command_q[4] = 0.0`으로 이 축을 죽여도 위치 추종이 되던 이유가 정확히 이것.**
→ 그러나 Amazing Hand를 붙이면 **손의 방향**이 이 축에 전적으로 달려 있으므로 반드시 살려야 한다.
   IK의 orientation task(가중치 0.01)가 자동으로 쓰거나, 오른손 썸스틱 X로 직접 지령한다.

---

## 5. 확정 파라미터

```python
URDF        = "assets/rpo_arm/urdf/rpo_arm_right.urdf"
EE_FRAME    = "ee_link"
JOINTS      = ["right_arm_pitch_joint", "right_arm_roll_joint", "right_arm_yaw_joint",
               "right_elbow_pitch_joint", "right_elbow_yaw_joint"]
IK_ITERS    = 5
POS_WEIGHT  = 1.0
ORI_WEIGHT  = 0.01      # LeRobot 기본값
HAND_SCALE  = 0.76      # 사람 손 변위 → 로봇 EE 변위
MAX_REACH   = 0.495     # m, 어깨 기준
```

---

## 6. 미확정 — 실측 필요

1. **Amazing Hand 마운트 오프셋** — 현재 `hand_mount_link`를 `ee_link`와 같은 `+0.15 m`에 임시 배치.
   커넥터 실제 치수가 나오면 `--hand-offset` 으로 재생성 필요.
   손 무게 0.4 kg이 손목 끝에 붙으므로 `elbow_yaw`(τ=18 N·m) 부하 재확인 권장.
2. **어깨 마운트 자세** — 거치대에 팔을 어떤 방향으로 고정할지에 따라 `base_link` 기준
   작업 공간이 통째로 회전한다. 실제 설치 후 확정.
