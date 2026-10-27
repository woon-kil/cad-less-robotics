#!/usr/bin/env bash
# docs/ 를 git 추적에서 빼고 .gitignore 에 넣은 상태를 커밋한다.
# 사용: bash scripts/exclude_docs_from_git.sh [--dry-run]
#
# 파일은 디스크에 그대로 남는다. 추적만 해제한다.
# 되돌리기: .gitignore 에서 docs/ 줄을 지우고  git add docs && git commit
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1

step() { printf '\n== %s\n' "$1"; }

step "1/5 사전 검증"
git rev-parse --is-inside-work-tree > /dev/null
grep -qx 'docs/' .gitignore || { echo ".gitignore 에 docs/ 가 없다. 먼저 추가해야 한다."; exit 1; }
echo ".gitignore 에 docs/ 있음"
TRACKED="$(git ls-files docs | wc -l | tr -d ' ')"
echo "현재 추적 중인 docs 파일: ${TRACKED}개"

step "2/5 추적 해제 (멱등)"
if [ "${TRACKED}" = "0" ]; then
  echo "이미 추적되지 않음 — 건너뜀"
elif [ "${DRY}" = "1" ]; then
  echo "--dry-run: git rm -r --cached docs 를 실행할 예정"
else
  git rm -r -q --cached docs
  echo "추적 해제 완료"
fi

step "3/5 스테이징"
if [ "${DRY}" = "1" ]; then
  echo "--dry-run: .gitignore AGENTS.md 를 스테이징할 예정"
else
  git add .gitignore AGENTS.md
  git status --short | head -20
fi

step "4/5 커밋"
if [ "${DRY}" = "1" ]; then
  echo "--dry-run: 커밋하지 않음"
elif git diff --cached --quiet; then
  echo "스테이징된 변경이 없다 — 건너뜀"
else
  git commit -q -F - <<'MSG'
chore: docs/ 를 git 추적에서 빼고 .gitignore 에 넣는다

사용자 지시다. 플랜·완료보고서·이슈보고서는 로컬 디스크에만 둔다. 파일은
삭제하지 않고 추적만 해제한다. 게이트 증거는 evidence/ 가 담당한다.
AGENTS.md 의 커밋 대상 목록도 같이 맞췄다.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01AF1UasKjaerC56u5266qok
MSG
  git log --oneline -1
fi

step "5/5 검증"
echo "추적 중인 docs 파일: $(git ls-files docs | wc -l | tr -d ' ')개 (0이어야 한다)"
echo "디스크의 docs 파일: $(find docs -type f | wc -l | tr -d ' ')개 (그대로 남아야 한다)"
echo
echo "푸시가 필요하면:  git push origin main"
echo "되돌리기:  .gitignore 에서 docs/ 줄 삭제 후  git add docs && git commit"
