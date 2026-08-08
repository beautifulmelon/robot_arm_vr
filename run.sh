#!/usr/bin/env bash
# 텔레옵 서버 시작 — 이 파일 하나만 실행하면 된다.
#
#   ./run.sh                 기본 (미러 꺼짐 — 화면이 실물과 같은 좌우)
#   ./run.sh --mirror        좌우 반사 켜기 (권장하지 않음, 아래 주석 참고)
#   ./run.sh --temp          임시 테스트 팔 (config/arm_temp.json)
#   ./run.sh --profile isaac Isaac Sim 쪽 포트 블록으로 (실물과 동시 실행 가능)
#   ./run.sh --meshcat       Meshcat 3D 뷰도 띄우기
#   ./run.sh --help          전체 옵션
#
# 포트는 프로파일이 정한다. jetson=4443, isaac=4453 (src/rpo_teleop/profiles.py).
# 실물 갈래와 Isaac Sim 갈래를 동시에 띄울 수 있게 하려는 것이다.

set -euo pipefail
cd "$(dirname "$0")"

PY=.venv/bin/python
[ -x "$PY" ] || { echo "❌ .venv 가 없습니다. uv venv --python 3.11 .venv 로 먼저 만드세요"; exit 1; }
# --temp 는 임시 테스트 팔 설정으로 바꾼다. 기존 팔 설정은 건드리지 않는다.
CONFIG=config/arm.json
for a in "$@"; do [ "$a" = "--temp" ] && CONFIG=config/arm_temp.json; done

[ -f "$CONFIG" ] || {
  echo "❌ $CONFIG 이 없습니다. 먼저 URDF 를 등록하세요:"
  if [ "$CONFIG" = "config/arm_temp.json" ]; then
    echo "   $PY scripts/06_setup_urdf.py \\"
    echo "       --urdf assets/robot_arm_temp/robot_arm_temp.urdf --out config/arm_temp.json"
  else
    echo "   $PY scripts/06_setup_urdf.py --urdf assets/robot_arm/robot_arm.urdf"
  fi
  exit 1; }

# 프로파일 결정 (기본 jetson). 웹 포트는 여기서 나온다.
PROFILE=jetson
prev=""
for a in "$@"; do
  [ "$prev" = "--profile" ] && PROFILE="$a"
  prev="$a"
done
WEB_PORT=$("$PY" -c "import sys;sys.path.insert(0,'src');from rpo_teleop import profiles;print(profiles.ports('$PROFILE').web)") || {
  echo "❌ 모르는 프로파일: $PROFILE"; exit 1; }

# 이전 서버가 떠 있으면 정리.
# ★ 스크립트 이름으로 죽이면 **다른 프로파일까지 같이 죽는다.** 같은 저장소에서
#   실물 갈래와 Isaac Sim 갈래가 동시에 도는데, 한쪽을 재시작하면서 다른 쪽을
#   말없이 내리면 원인을 찾기 어렵다. 이 프로파일의 포트를 물고 있는 PID 만 죽인다.
PIDS=$(lsof -nP -iTCP:"$WEB_PORT" -sTCP:LISTEN -t 2>/dev/null || true)
if [ -n "$PIDS" ]; then
  echo "이전 서버 정리 중… (프로파일 $PROFILE, 포트 $WEB_PORT, pid $PIDS)"
  kill $PIDS 2>/dev/null || true
  sleep 2
fi

# ★ 미러는 기본으로 끈다.
#   미러는 반사(det=-1)라서 화면이 실물의 거울상이 된다. 이 손은 오른손
#   (amazing_hand_right)인데 미러를 켜면 화면에 왼손으로 보인다. 엄지 위치와
#   접근 방향이 실제와 반대로 인지되므로 데이터 수집에 쓰면 안 된다.
#   좌우 느낌이 안 맞으면 --base-yaw 0/90/180/270 (정상 회전) 으로 맞출 것.
#
# --no-mirror 는 하위호환으로 받아만 주고 무시한다. 원소를 빈 문자열로 치환하면
# argparse 에 빈 인자가 넘어가므로 실제로 걸러내야 한다.
ARGS=()
for a in "$@"; do
  [ "$a" = "--no-mirror" ] && continue      # 하위호환: 받아만 주고 무시
  [ "$a" = "--temp" ] && continue           # 위에서 CONFIG 로 이미 소비
  ARGS+=("$a")
done

exec "$PY" scripts/05_teleop_sim.py --config "$CONFIG" ${ARGS[@]+"${ARGS[@]}"}
