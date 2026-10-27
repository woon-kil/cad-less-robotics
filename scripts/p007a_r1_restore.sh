#!/usr/bin/env bash
# P-007A-R1 롤백 — 격리로 바꾼 "구성"만 되돌린다. Mac에서 실행한다.
# 사용: bash scripts/p007a_r1_restore.sh [--dry-run]
#
# 🔴 이 스크립트는 프로세스를 다시 띄우지 않는다. 일부러 그렇게 만들었다.
#    autostart를 되돌린 뒤 다음 로그인·재부팅이 일어나면 start_app.sh가 joystick.py를
#    띄우고, joystick.py는 시작할 때 move_init_pose()로 팔을 [90,164,18,0,90,30]으로
#    움직인다. 전원 재투입도 벤더 initial posture 회전을 일으킬 수 있다.
#    복원 후 재로그인·재부팅·전원 재투입은 작업대 clearance를 다시 확인한 뒤에만 한다.
set -euo pipefail

REMOTE="${P007A_REMOTE:-jetson@192.168.0.94}"
AUTOSTART="/home/jetson/.config/autostart"
BAK_SUFFIX=".p007ar1.bak"
DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1

say() { printf '%s\n' "$1"; }
rsh() { ssh -o BatchMode=yes -o ConnectTimeout=8 "${REMOTE}" "$@"; }

say "== 사전 검증"
[ "$(rsh 'hostname')" = "yahboom" ] || { say "원격 hostname이 yahboom이 아니다"; exit 1; }

say "== 현재 autostart 상태"
rsh "ls -1 ${AUTOSTART}/ 2>/dev/null || echo '(디렉터리 없음)'"

say "== 1) autostart 원본 복원 (멱등)"
CMD="set -e
for bak in ${AUTOSTART}/*${BAK_SUFFIX}; do
  [ -e \"\$bak\" ] || continue
  orig=\"\${bak%${BAK_SUFFIX}}\"
  if [ -e \"\$orig\" ]; then
    echo \"건너뜀 (원본이 이미 있다): \$orig\"
  else
    mv -v \"\$bak\" \"\$orig\"
  fi
done"
if [ "${DRY}" = "1" ]; then
  say "--dry-run: 아래 명령을 원격에서 돌릴 예정"
  printf '%s\n' "${CMD}"
else
  rsh "${CMD}"
fi

say "== 2) 검증"
rsh "ls -1 ${AUTOSTART}/ 2>/dev/null; echo '--- sha256'; sha256sum ${AUTOSTART}/* 2>/dev/null || true"

say "== 수동으로 되돌릴 항목 (이 스크립트가 건드리지 않는다)"
say "  - Jupyter 서비스: 격리 때 중지했다면  sudo systemctl start jupyterlab"
say "  - 종료한 벤더 프로세스: 이 스크립트는 다시 띄우지 않는다."
say "    띄워야 하면 Jetson 데스크탑에서  bash /home/jetson/dofbot_pro/APP_DOFBOT_PRO/start_app.sh"
say "    🔴 그 순간 joystick.py가 move_init_pose()로 팔을 움직인다. 작업대를 먼저 비우고 전원 차단에 손이 닿는 상태에서만 실행한다."
