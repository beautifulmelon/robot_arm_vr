#!/usr/bin/env bash
# 공유 보드 빠른 확인 — 일 시작 전에 한 번 돌린다.
#   ./board/check.sh          양쪽 최신 부분
#   ./board/check.sh jetson   실물 담당 파일만
#   ./board/check.sh isaac    시뮬 담당 파일만
set -euo pipefail
cd "$(dirname "$0")"

show() {  # 파일, 제목
  [ -f "$1" ] || return 0
  echo
  echo "════════════════════════════════════════════════════════════════════"
  echo "  $2      (수정 $(date -r "$1" '+%m/%d %H:%M'))"
  echo "════════════════════════════════════════════════════════════════════"
  # '지금 상태' 한 줄 + 변경/메시지 절의 제목만 뽑는다. 전문은 파일을 열 것.
  awk '
    /^ 지금 상태/       { s=1; next }
    s==1 && /^-----/    { next }
    s==1 && NF          { print "  " $0; s=2; next }
    /^ ⚠️|^ 📮|^ ⏳/     { print ""; print $0; next }
    /^ [A-Z]-[0-9]|^ 20[0-9][0-9]-/ { print "   " $0 }
  ' "$1"
}

case "${1:-all}" in
  jetson) show 10_JETSON.txt "실물 젯슨 담당" ;;
  isaac)  show 20_ISAAC.txt  "Isaac Sim 담당" ;;
  *)      show 10_JETSON.txt "실물 젯슨 담당"
          show 20_ISAAC.txt  "Isaac Sim 담당"
          echo
          echo "  합의 사항: board/30_AGREED.txt  ·  규약: board/00_README.txt"
          echo ;;
esac
