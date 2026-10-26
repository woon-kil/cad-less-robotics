#!/usr/bin/env bash
# 실측 앵커가 기록되기 전에는 카메라를 켜지 않는다. 로봇은 움직이지 않는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
if ! python3 scripts/p006c_r25_geometry.py --check-anchors; then
  printf 'OPERATOR_ANCHOR_REQUIRED\n'
  printf '촬영하지 않는다.\n'
  exit 2
fi

REMOTE="${P006C_REMOTE:-p620}"
REMOTE_ROOT="${P006C_REMOTE_ROOT:-/home/woonkil/cad-less-robotics}"
OUT="${ROOT}/evidence/P-006C-R2.5"
POSITIONS=(REG_A REG_B REG_C VAL_A VAL_B VAL_C)

ask() {
  local message="$1"
  local primary="$2"
  local secondary="$3"
  osascript << EOF
set resultDialog to display dialog "P-006C-R2.5" & return & "카메라, LED, 로봇, 테이블은 움직이지 마세요." & return & "${message}" buttons {"${secondary}", "${primary}"} default button "${primary}" giving up after 180
if gave up of resultDialog then
  return "timeout"
end if
return button returned of resultDialog
EOF
}

intro="$(ask "기록된 origin 좌표에 체크보드 원점만 맞춥니다. 좌표는 파일에 있는 실측값만 사용합니다." "시작" "취소" || true)"
if [[ "${intro}" != "시작" ]]; then
  printf 'BLOCKED_NO_OPERATOR\n'
  exit 2
fi

rsync -az \
  "${ROOT}/scripts/p006c_r25_capture.py" \
  "${ROOT}/scripts/p006c_r25_fit.py" \
  "${ROOT}/scripts/p006c_r25_geometry.py" \
  "${ROOT}/scripts/p006c_r25_launch.sh" \
  "${ROOT}/scripts/p006c_capture_streams.py" \
  "${ROOT}/scripts/p006c_validate_metric.py" \
  "${REMOTE}:${REMOTE_ROOT}/scripts/"
rsync -az "${ROOT}/evidence/P-006/camera-target/thresholds.yaml" "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/thresholds.yaml"
rsync -az "${OUT}/anchor_definition.json" "${OUT}/thresholds.json" "${OUT}/threshold_freeze.json" "${REMOTE}:${REMOTE_ROOT}/evidence/P-006C-R2.5/"

ssh -o BatchMode=yes "${REMOTE}" "mkdir -p '${REMOTE_ROOT}/evidence/P-006C-R2.5/captures' '${REMOTE_ROOT}/evidence/P-006C-R2.5/logs' && rm -f '${REMOTE_ROOT}/evidence/P-006C-R2.5/driver-stop.txt' '${REMOTE_ROOT}/evidence/P-006C-R2.5/driver-ready.txt'"
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh bash scripts/p006c_r25_launch.sh" > /tmp/p006c-r25-launch.log 2>&1 &
LAUNCH_SSH=$!
ready=0
for _ in $(seq 1 100); do
  if ssh -o BatchMode=yes "${REMOTE}" "test -f '${REMOTE_ROOT}/evidence/P-006C-R2.5/driver-ready.txt'"; then
    ready=1
    break
  fi
  if ! kill -0 "${LAUNCH_SSH}" 2>/dev/null; then
    break
  fi
  sleep 1
done
if [[ "${ready}" -ne 1 ]]; then
  echo "드라이버가 준비되지 않았다" >&2
  ssh -o BatchMode=yes "${REMOTE}" "touch '${REMOTE_ROOT}/evidence/P-006C-R2.5/driver-stop.txt'" || true
  wait "${LAUNCH_SSH}" || true
  exit 1
fi

mkdir -p "${OUT}/logs"
for position in "${POSITIONS[@]}"; do
  try_index=1
  while [[ "${try_index}" -le 3 ]]; do
    choice="$(ask "${position} 배치 ${try_index}. 파일에 기록된 origin에 보드 원점을 맞춘 뒤 촬영을 누르세요." "촬영" "이 위치 건너뜀" || true)"
    printf '%s %s try=%s choice=%s\n' "$(date -Iseconds)" "${position}" "${try_index}" "${choice:-error}" >> "${OUT}/logs/operator-log.txt"
    if [[ "${choice}" != "촬영" ]]; then
      break
    fi
    ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_r25_capture.py --position '${position}' --try-index '${try_index}' --count 4" || true
    try_index=$((try_index + 1))
    if [[ "${try_index}" -le 3 ]]; then
      more="$(ask "${position} 프레임을 남겼습니다. 같은 좌표로 한 번 더 찍을까요. 이전 프레임은 지우지 않습니다." "추가 촬영" "다음 위치" || true)"
      printf '%s %s more=%s\n' "$(date -Iseconds)" "${position}" "${more:-error}" >> "${OUT}/logs/operator-log.txt"
      if [[ "${more}" != "추가 촬영" ]]; then
        break
      fi
    fi
  done
done

ssh -o BatchMode=yes "${REMOTE}" "touch '${REMOTE_ROOT}/evidence/P-006C-R2.5/driver-stop.txt'"
wait "${LAUNCH_SSH}" || true
rsync -az "${REMOTE}:${REMOTE_ROOT}/evidence/P-006C-R2.5/" "${OUT}/"
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_r25_fit.py"
printf 'CAPTURE_DONE\n'
