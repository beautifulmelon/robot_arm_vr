# 로봇 팔 버전 대장

버전을 붙이는 것은 **신규 계보**(바닥 파지 전용 5축 + 평행 그리퍼)뿐입니다.
구형·임시 팔은 다른 로봇이라 버전을 붙이지 않습니다.

## 규칙

1. **번호는 기구 담당이 부르는 번호를 그대로 씁니다.** 우리가 따로 매기면
   세 사람이 서로 다른 번호를 말하게 됩니다.
2. 새 URDF 를 받으면 `assets/arm_v<N>/` + `config/arm_v<N>.json` 을 만들고
   **이전 버전은 지우지 않습니다.** 대시보드에서 골라 비교할 수 있어야 합니다.
3. ★ **"운동학 변화" 칸을 반드시 채웁니다.** 이게 이 표의 핵심입니다.
   - **없음** → IK · 도달범위 · config · 홈 자세를 다시 안 재도 됩니다
   - **있음** → 전부 다시 재고 `12_arm_preview.py` · `14_reach_map.py` 를 돌립니다

## 대장

| 버전 | 받은 날 | 출처 (Fusion) | 운동학 변화 | 질량 | 문서 |
|---|---|---|---|---|---|
| v1 | 2026-08-29 | `new_robotarm_urdf` v8 / 그리퍼변형 v60 | 최초 | 2600.6 g | handoff/64, 66 |
| **v2** | 2026-09-01 | `new_robotarm_v2_urdf` v8 / 그리퍼변형 v60 | **없음** — 회전축 위치 전부 동일 | 2640.9 g | handoff/67, 68 |
| v2 (개구 갱신) | 2026-09-03 | 그리퍼 스윕 10~198° 95점 | 없음 — 그리퍼 리밋만 확장 | 2640.9 g | handoff/71, 72 |

### v2 에서 바뀐 것 (handoff/67 §8-1)
- j2/j3/j4 를 **양측 지지 요크**로. 외팔 지지 시 플랜지 볼트가 PLA 를 19 MPa 로
  눌러 크리프하는데, 요크로 받으면 0.47 MPa 로 떨어집니다.
- 모터를 자기 축 방향으로 12.25 mm 밀어 (모터+캡) 중심을 팔 중심선 y=0 에 맞춤
- 질량 link1 400.4→433.1 g · link4 23.5→32.0 g

회전축 위치 `j1 z=-1.70 / j2 56.00 / j3 236.00 / j4 416.00 / j5 (0,0)` 는 v1 과 동일.
따라서 **링크 길이·도달거리·IK·토크 계산이 그대로 유효**합니다.

## ★★ 2026-09-03 — 그리퍼 개구 82.52 mm · joint5 재체결 모델 · joint1 원복

기구 담당 회신(handoff/71)과 전달본(robotarm_delivery_2026-09-03)을 반영했습니다.

### 1. 개구 57 → **82.52 mm** — 하드웨어 변경 없음
57 mm 는 기계 한계가 아니라 **스윕 범위(서보 10~120°)** 였습니다. 198° 까지 다시
훑은 95점으로 **캔(66)·페트병 몸통(65)·종이컵(80) 전부 잡힙니다.**
`gripper_joint` 리밋 -15.30~47.50°, `rocker_r_joint` -54.57~7.93° 로 넓어졌습니다.

★ **전달본 `gripper_map.py` 의 7차 다항식은 발산합니다.** 최고차항이 `-0.0` 으로
찍혀(반올림) 서보 100° 를 넘으면 틀리고 198° 에서 개구 -1092 mm 가 나옵니다.
그래서 **실측 95점을 그대로 보간표**로 쓰는 판을 `scripts/17_fit_gripper_map.py` 로
만들어 교체했습니다. 원본은 `gripper_map_delivered_2026-09-03.py` 로 남겼습니다.
검산 — 기구 담당 서보각 표와 2° 안에서 일치, 198° 관절값이 URDF 리밋과 0.1° 안에서 일치.

★ **mimic 판(`arm_v2.urdf`)을 개구 60 mm 넘겨 쓰지 마세요** — 우측 조 오차 8.21 mm.
텔레옵은 `_ik.urdf`(그리퍼 fixed) + `gripper_link` 가 `joint_rad()` 로 좌우를 따로
넣으므로 사실상 nomimic 입니다. Isaac 에는 `_nomimic` 판을 줍니다.

### 2. ★★ joint1 영점 이동은 **되돌렸습니다.** 대신 joint5 를 180° 돌립니다
09-02 의 joint1 origin `[0 0 π]` 는 전제가 틀렸습니다. **joint1 은 팔 전체를 통째로
돌리므로 카메라와 툴의 상대 관계를 못 바꿉니다.** 카메라 위/아래를 정하는 것은 j5 입니다.
사용자 자세가 j1=-172° 로 간 진짜 이유는, 카메라(link5 +x 쪽)를 위로 올리려면
팔꿈치를 **반대 분지**로 접고 j1 을 반 바퀴 돌려야 했기 때문입니다.

기구 담당 제안(부품 0개): **그리퍼 조립체를 motor5 장착면에서 180° 돌려 체결**한다
(볼트원 4×90° / 6×60° 라 정확히 홀에 떨어짐). 그러면 j5=0 에서 카메라가 위로 옵니다.
URDF 로는 **joint5 origin rpy `[0 0 -π]`** 가 정확히 이 재체결입니다
(`16_rotate_joint_zero.py --joint joint5 --deg 180`).

```
joint1   origin rpy  [0 0 π] → [0 0 0]      되돌림
joint5   origin rpy  [0 0 0] → [0 0 -π]     그리퍼 180° 재체결 모델
홈       [0, -54.51, +84.45, +80.78, +4.90]°   ← j1 = 0, j5 ≈ 0
```

★ 홈은 사용자가 VR 로 잡은 자세와 **세계 좌표 형상이 완전히 같습니다** (정면을 보도록
8° 요만 다름). 옛 홈 대비 tool_frame·camera_mount·link5·jaw 위치차 0.02 mm / 0.005°.
j2 가 음수인 것은 **거울 분지**(어깨가 뒤로 젖고 팔꿈치가 앞으로)라서입니다.
툴 (161.2, 0, 290.8) mm · 아래 20.7° · 카메라 그리퍼 위 +32.7 mm · j1 여유 ±178°.

★★ **실물 영향 — 관절 캘리브레이션은 안 바뀝니다.** motor5 영점은 그대로고
그리퍼를 180° 돌려 다는 것뿐입니다. joint1 방식(관절값 규약 변경)과 다른 결정적 이점입니다.
조립 시 **그리퍼 조립체를 180° 돌려 체결**하기만 하면 됩니다. 기구 담당이 정식 URDF 를
낼 때 반영해 달라고 요청했습니다 (handoff/72).

### 다시 적용하는 법
기구 담당이 새 URDF 를 주면(정식판에 재체결이 반영되기 전까지):
```bash
python3 scripts/16_rotate_joint_zero.py --joint joint5 --deg 180 \
    --urdf assets/arm_vN/arm_vN.urdf assets/arm_vN/arm_vN_nomimic.urdf
python3 scripts/17_fit_gripper_map.py          # 다항식 대신 보간표 (스윕이 바뀌면)
python3 scripts/11_make_ik_urdf.py --in ... --out ..._ik.urdf --keep joint1 ... joint5
python3 scripts/15_merge_body_arm.py ; python3 scripts/12_arm_preview.py
```

## 파일 규약

```
assets/arm_v<N>/
    arm_v<N>.urdf            원본. 그리퍼가 mimic 으로 묶임
    arm_v<N>_nomimic.urdf    좌우 로커 독립. 정밀 파지용
    arm_v<N>_ik.urdf         ★ 텔레옵이 쓰는 IK 전용판 (11_make_ik_urdf.py 로 생성)
    meshes/ gripper_map.py gripper_sweep.json _props.json renders/
config/arm_v<N>.json
```

★ `<이름>_ik.urdf → <이름>.urdf` 규약을 `gripper_link.py` 가 씁니다.
  IK URDF 는 그리퍼 관절이 fixed 라 조가 안 움직이므로, 화면용으로 원본을 찾습니다.
  이 규약이 깨지면 그리퍼가 화면에서 안 움직입니다.

★ **팔 식별자는 파일명입니다.** `check_robot_match` 가 config 의 URDF 파일명과
  State 의 `robot` 을 대조합니다. 이름을 바꾸면 젯슨이 보내야 하는 값도 바뀝니다.
  v2 기준 기대값은 **`arm_v2_ik.urdf`** 입니다.

## 옛 이름 대응

2026-09-01 에 `arm_new`/`arm_new2` → `arm_v1`/`arm_v2` 로 바꿨습니다.
**handoff/64·65·66·67 과 board 의 과거 기록은 주고받은 원본이라 고치지 않았습니다.**
거기 적힌 옛 이름은 아래로 읽으세요.

| 옛 이름 | 지금 |
|---|---|
| `assets/arm_new/` · `arm_new*.urdf` | `assets/arm_v1/` · `arm_v1*.urdf` |
| `assets/arm_new2/` · `arm_new2*.urdf` | `assets/arm_v2/` · `arm_v2*.urdf` |
| `config/arm_new.json` | `config/arm_v1.json` |
| `config/arm_new2.json` | `config/arm_v2.json` |

※ URDF 안의 `<robot name="arm_new">` 은 기구 담당 생성기가 넣는 값이라 그대로 뒀습니다
  (고치면 재생성할 때 되돌아옵니다). 코드는 이 값을 쓰지 않습니다 — 파일명을 씁니다.
