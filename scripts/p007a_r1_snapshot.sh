#!/usr/bin/env bash
# P-007A-R1 1단계 — 읽기 전용 조사·snapshot. Mac에서 실행한다.
# 사용: bash scripts/p007a_r1_snapshot.sh [before|after]
# 로봇을 움직이지 않는다. 원격 설정 변경 0, 프로세스 종료 0, i2c probe 0.
# 원복: 이 스크립트는 읽기만 하므로 되돌릴 것이 없다. 산출물만 지우면 된다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE="${P007A_REMOTE:-jetson@192.168.0.94}"
PHASE="${1:-before}"
OUT="${ROOT}/evidence/P-007A-R1"
LOGS="${OUT}/logs"
RAW="${LOGS}/raw_snapshot_${PHASE}.json"
READS="${LOGS}/pre_isolation_reads.csv"
[ "${PHASE}" = "after" ] && READS="${LOGS}/post_isolation_reads.csv"

step() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$1"; }

step "1/5 사전 검증"
[ "${PHASE}" = "before" ] || [ "${PHASE}" = "after" ] || { echo "phase는 before 또는 after"; exit 1; }
for f in p007a_r1_collect.py p007a_r1_read.py p007a_r1_split.py; do
  [ -f "${ROOT}/scripts/${f}" ] || { echo "없음: scripts/${f}"; exit 1; }
done
mkdir -p "${LOGS}"
HOSTNAME_REMOTE="$(ssh -o BatchMode=yes -o ConnectTimeout=8 "${REMOTE}" 'hostname' 2>&1 | tail -1)"
[ "${HOSTNAME_REMOTE}" = "yahboom" ] || { echo "원격 hostname이 yahboom이 아니다: ${HOSTNAME_REMOTE}"; exit 1; }
echo "원격 ${REMOTE} hostname=${HOSTNAME_REMOTE}"

step "2/5 기존 산출물 백업 (멱등 재실행)"
for f in "${RAW}" "${READS}"; do
  [ -f "$f" ] && cp -p "$f" "${f%.*}.$(date +%y%m%d-%H%M%S).bak" && echo "백업 $f"
done || true

step "3/5 원격 읽기 전용 수집"
ssh -o BatchMode=yes "${REMOTE}" 'python3 -' < "${ROOT}/scripts/p007a_r1_collect.py" > "${RAW}"
python3 -c "import json,sys;d=json.load(open('${RAW}'));assert d['writes_performed']==0;print('수집 JSON ok, 섹션',len(d))"

step "4/5 컨트롤러 읽기 (쓰기 0)"
ROUNDS=5
[ "${PHASE}" = "after" ] && ROUNDS=100
if ssh -o BatchMode=yes "${REMOTE}" "python3 - ${ROUNDS}" < "${ROOT}/scripts/p007a_r1_read.py" > "${READS}" 2> "${LOGS}/read_${PHASE}.stderr"; then
  echo "읽기 성공 $(( $(wc -l < "${READS}") - 1 ))건"
else
  echo "읽기 오류 있음. ${LOGS}/read_${PHASE}.stderr 와 ${READS} 확인"; cat "${LOGS}/read_${PHASE}.stderr"
fi

step "5/5 증거 분리·요약"
python3 "${ROOT}/scripts/p007a_r1_split.py" "${RAW}" "${OUT}" "${PHASE}"

echo
echo "원복 불필요 — 읽기 전용. 산출물: ${OUT}"
