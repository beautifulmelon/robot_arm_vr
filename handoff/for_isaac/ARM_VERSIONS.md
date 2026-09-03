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

### v2 에서 바뀐 것 (handoff/67 §8-1)
- j2/j3/j4 를 **양측 지지 요크**로. 외팔 지지 시 플랜지 볼트가 PLA 를 19 MPa 로
  눌러 크리프하는데, 요크로 받으면 0.47 MPa 로 떨어집니다.
- 모터를 자기 축 방향으로 12.25 mm 밀어 (모터+캡) 중심을 팔 중심선 y=0 에 맞춤
- 질량 link1 400.4→433.1 g · link4 23.5→32.0 g

회전축 위치 `j1 z=-1.70 / j2 56.00 / j3 236.00 / j4 416.00 / j5 (0,0)` 는 v1 과 동일.
따라서 **링크 길이·도달거리·IK·토크 계산이 그대로 유효**합니다.

## ★★ 2026-09-02 — joint1 영점을 180° 옮겼습니다 (규약 변경)

**형상은 하나도 안 바뀌었습니다. 관절값을 읽는 기준만 바뀝니다.**

```
URDF   joint1 origin rpy  [0 0 0] → [0 0 π]
관절값  q1_new = q1_old − 180°  (mod 360)
홈      joint1 −172.00° → +8.00°   ← 나머지 4축과 다른 관절은 그대로
```

### 왜

홈의 joint1 이 −172° 라 리밋(±178)까지 **6° 밖에 안 남았습니다.** 한쪽으로
조금만 요(yaw)하면 벽에 닿습니다. 더 나쁜 것은 **못 쓰는 4° 구간이 팔이
정면(툴 방위 0°)을 향하는 자리**에 있었다는 점입니다 — 작업 방향 한복판입니다.

| | joint1 범위 | 홈에서 좌 / 우 여유 | 못 쓰는 방향 |
|---|---|---|---|
| 전 | −178 ~ +178 (홈 −172) | **6° / 350°** | **툴 방위 0° — 정면** |
| 후 | −178 ~ +178 (홈 +8) | **186° / 170°** | 툴 방위 180° — 뒤쪽(차체) |

뒤쪽은 어차피 차체와 부딪히는 자리라 잃을 것이 없습니다.
새 규약에서는 **툴 방위 = joint1** 이라 읽기도 쉬워집니다.

### 검증

`arm_v1` · `arm_v2` 모두 홈에서 tool_frame · camera_mount · link5 · jaw_l/r 의
world pose 를 옛 모델과 대조했습니다 — **위치차 0.0000 mm / 자세차 0.0000°**.
도달 반경(0.2~472.0 mm)과 최대 높이(428.1 mm)도 그대로입니다.

### ★★ 실물도 같이 바꿔야 합니다

시뮬만 바꾸면 **첫 지령에 팔이 반 바퀴 돕니다.** joint1 서보의 영점을 180°
옮기거나, 드라이버에서 지령·피드백에 −180° 오프셋을 걸어야 합니다.
handoff/68 §3 에 적었습니다.

### 다시 적용하는 법

기구 담당이 새 URDF 를 주면 그 파일은 옛 영점입니다. 매번 이렇게 돌리세요.

```bash
python3 scripts/16_rotate_joint_zero.py --joint joint1 --deg 180 \
    --urdf assets/arm_vN/arm_vN.urdf assets/arm_vN/arm_vN_nomimic.urdf
python3 scripts/11_make_ik_urdf.py --in ... --out ..._ik.urdf --keep joint1 ... joint5
python3 scripts/15_merge_body_arm.py          # 본체 결합본
python3 scripts/12_arm_preview.py             # arms.json
```

★ 기구 담당에게 **CAD 쪽에서 아예 이 영점으로 바꿔 달라고** 요청하는 것이
  근본 해결입니다. 그러면 이 단계가 없어집니다.

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
