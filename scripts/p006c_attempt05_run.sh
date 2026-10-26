#!/usr/bin/env bash
# Mac에서 실행한다. 이동이 부족하면 그 촬영을 남기고 보드만 다시 놓는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE="${P006C_REMOTE:-p620}"
REMOTE_ROOT="${P006C_REMOTE_ROOT:-/home/woonkil/cad-less-robotics}"
OUT="${ROOT}/evidence/P-006/camera-target/attempt-05"
POSES=(near far)
MAX_TRIES="$(python3 -c "import yaml; print(yaml.safe_load(open('${OUT}/capture-plan.yaml'))['max_tries'])")"

prompt_for() {
  case "$1" in
    near) printf '%s' "같은 높이의 연장된 면 위에서 화면 아래쪽으로 옮기세요. Attempt 3에서 판자 끝이던 자리보다 더 아래로 갈 공간을 쓰세요. 높이와 기울기는 그대로입니다. 2 cm를 옮겼다는 사실만으로는 합격이 아닙니다. 합격은 측정된 ΔZ가 -40 mm보다 더 가까운 값입니다. -50 mm 이하면 경계에서 떨어집니다." ;;
    far) printf '%s' "같은 높이의 면 위에서 화면 위쪽, 카메라 몸체 쪽으로 옮기세요. 처음 자리에서 15 cm 정도 갈 공간이 목표입니다. 그 거리만으로 합격시키지 않습니다. 합격은 측정된 ΔZ가 +40 mm보다 먼 값입니다. +50 mm 이상이면 경계에서 떨어집니다. 받침에 닿았는데도 모자라다면 받침에 닿음을 누르고 카메라를 옮기지 마세요." ;;
    *) return 1 ;;
  esac
}

ask() {
  local message="$1"
  local primary="$2"
  local secondary="$3"
  osascript << EOF
set resultDialog to display dialog "P-006C Attempt 5" & return & "카메라와 로봇은 움직이지 마세요." & return & "${message}" buttons {"${secondary}", "${primary}"} default button "${primary}" giving up after 180
if gave up of resultDialog then
  return "timeout"
end if
return button returned of resultDialog
EOF
}

ask3() {
  local message="$1"
  local primary="$2"
  local middle="$3"
  local secondary="$4"
  osascript << EOF
set resultDialog to display dialog "P-006C Attempt 5" & return & "카메라와 로봇은 움직이지 마세요." & return & "${message}" buttons {"${secondary}", "${middle}", "${primary}"} default button "${primary}" giving up after 180
if gave up of resultDialog then
  return "timeout"
end if
return button returned of resultDialog
EOF
}

log_remote() {
  printf '%s %s\n' "$(date -Iseconds)" "$1" | ssh -o BatchMode=yes "${REMOTE}" "cat >> '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/operator-log.txt'"
}

mm() {
  python3 -c 'import math,sys
raw = sys.argv[1] if len(sys.argv) > 1 else ""
try:
    value = float(raw)
except ValueError:
    print("없음")
else:
    print("없음" if math.isnan(value) else f"{value * 1000:.1f}")
' "${1:-}"
}

side_of() {
  python3 -c 'import sys
raw = sys.argv[1] if len(sys.argv) > 1 else ""
try:
    value = float(raw)
except ValueError:
    print("없음")
else:
    print("화면 왼쪽" if value < 0 else "화면 오른쪽")
' "${1:-}"
}

intro="$(ask "카메라와 로봇은 그대로 둡니다. 같은 높이로 늘린 면 위에서 near와 far만 찍습니다. 화면 위아래는 이동 방향이고, 합격은 측정된 ΔZ입니다. left와 right는 Attempt 3, tilt는 Attempt 2를 다시 계산합니다." "시작" "취소" || true)"
if [[ "${intro}" != "시작" ]]; then
  printf 'BLOCKED_NO_OPERATOR\n'
  exit 2
fi

rsync -az \
  "${ROOT}/scripts/p006c_attempt05_capture.py" \
  "${ROOT}/scripts/p006c_attempt05_feedback.py" \
  "${ROOT}/scripts/p006c_attempt05_launch.sh" \
  "${ROOT}/scripts/p006c_attempt05_validate.py" \
  "${ROOT}/scripts/p006c_capture_streams.py" \
  "${ROOT}/scripts/p006c_validate_metric.py" \
  "${REMOTE}:${REMOTE_ROOT}/scripts/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/thresholds.yaml" \
  "${ROOT}/evidence/P-006/camera-target/checkerboard.yaml" \
  "${ROOT}/evidence/P-006/camera-target/color-pose.csv" \
  "${ROOT}/evidence/P-006/camera-target/reproducibility.csv" \
  "${ROOT}/evidence/P-006/camera-target/summary.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/frames/0000.json" \
  "${ROOT}/evidence/P-006/camera-target/frames/0011.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/frames/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/attempt-02/capture-plan.yaml" \
  "${ROOT}/evidence/P-006/camera-target/attempt-02/diversity-validation.csv" \
  "${ROOT}/evidence/P-006/camera-target/attempt-02/poses.csv" \
  "${ROOT}/evidence/P-006/camera-target/attempt-02/summary.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/attempt-02/frames/tilt_positive-00.json" \
  "${ROOT}/evidence/P-006/camera-target/attempt-02/frames/tilt_negative-01.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/frames/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/attempt-03/capture-plan.yaml" \
  "${ROOT}/evidence/P-006/camera-target/attempt-03/diversity-validation.csv" \
  "${ROOT}/evidence/P-006/camera-target/attempt-03/poses.csv" \
  "${ROOT}/evidence/P-006/camera-target/attempt-03/summary.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-03/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/attempt-03/frames/left-t01-00.json" \
  "${ROOT}/evidence/P-006/camera-target/attempt-03/frames/right-t01-00.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-03/frames/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/attempt-04/capture-plan.yaml" \
  "${ROOT}/evidence/P-006/camera-target/attempt-04/diversity-validation.csv" \
  "${ROOT}/evidence/P-006/camera-target/attempt-04/poses.csv" \
  "${ROOT}/evidence/P-006/camera-target/attempt-04/summary.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-04/"
rsync -az \
  "${ROOT}/evidence/P-006/camera-target/attempt-04/frames/near-t01-00.json" \
  "${ROOT}/evidence/P-006/camera-target/attempt-04/frames/far-t01-00.json" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-04/frames/"
rsync -az "${OUT}/capture-plan.yaml" "${OUT}/baseline.txt" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/"

ssh -o BatchMode=yes "${REMOTE}" "mkdir -p '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/frames' && {
  echo \"observed_at: \$(date -Iseconds)\"
  lsusb -d 2bc5:0804 || true
  echo '---'
  lsusb -t || true
} > '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/hardware-inventory.txt'"

ssh -o BatchMode=yes "${REMOTE}" "rm -f '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/driver-stop.txt' '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/driver-ready.txt'"
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh bash scripts/p006c_attempt05_launch.sh" >/tmp/p006c-attempt03-launch.log 2>&1 &
LAUNCH_SSH=$!

ready=0
for _ in $(seq 1 100); do
  if ssh -o BatchMode=yes "${REMOTE}" "test -f '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/driver-ready.txt'"; then
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
  ssh -o BatchMode=yes "${REMOTE}" "touch '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/driver-stop.txt'" || true
  wait "${LAUNCH_SSH}" || true
  exit 1
fi

captured_any=0
for pose in "${POSES[@]}"; do
  try_index=1
  while [[ "${try_index}" -le "${MAX_TRIES}" ]]; do
    if [[ "${try_index}" -eq 1 ]]; then
      choice="$(ask "$(prompt_for "${pose}")" "촬영" "이 자세 건너뜀" || true)"
      log_remote "${pose} try=${try_index} choice=${choice:-error}"
      if [[ "${choice}" != "촬영" ]]; then
        if [[ "${captured_any}" -eq 0 && "${choice}" == "timeout" ]]; then
          try_index=100
        fi
        break
      fi
    fi
    ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_attempt05_capture.py --pose '${pose}' --try-index '${try_index}' --count 4" || true
    captured_any=1
    feedback="$(ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_attempt05_feedback.py --pose '${pose}' --try-index '${try_index}'" | awk '/^FEEDBACK /{line=$0} END{print line}')"
    log_remote "${feedback:-FEEDBACK missing}"
    rule="$(printf '%s\n' "${feedback}" | sed -n 's/.* rule=//p')"
    dx="$(printf '%s\n' "${feedback}" | sed -n 's/.* delta_x_m=\([^ ]*\).*/\1/p')"
    dy="$(printf '%s\n' "${feedback}" | sed -n 's/.* delta_y_m=\([^ ]*\).*/\1/p')"
    dz="$(printf '%s\n' "${feedback}" | sed -n 's/.* delta_z_m=\([^ ]*\).*/\1/p')"
    solved="$(printf '%s\n' "${feedback}" | sed -n 's/.* solved=\([^ ]*\).*/\1/p')"
    detail="${pose} ${try_index}차. 검출 ${solved:-0}장. ΔX $(mm "${dx}") mm ($(side_of "${dx}")). ΔY $(mm "${dy}") mm. ΔZ $(mm "${dz}") mm."
    if [[ "${rule}" == "PASS" ]]; then
      ask "${detail} 고정된 합격선을 넘었습니다. 다음 자세로 갑니다." "다음" "닫기" || true
      break
    fi
    if [[ "${pose}" == "far" ]]; then
      retry="$(ask3 "${detail} 측정된 ΔZ가 +40 mm를 넘지 않습니다. 이 촬영은 남깁니다. 받침에 닿았으면 카메라를 옮기지 말고 받침에 닿음을 누르세요." "추가 촬영" "받침에 닿음" "이 자세 종료" || true)"
      log_remote "${pose} retry_choice=${retry:-error}"
      if [[ "${retry}" == "받침에 닿음" ]]; then
        log_remote "FAR_PHYSICAL_WORKSPACE_LIMIT"
        ssh -o BatchMode=yes "${REMOTE}" "printf '%s\n' 'FAR_PHYSICAL_WORKSPACE_LIMIT' > '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/far-physical-limit.txt'"
        try_index=100
        break
      fi
      if [[ "${retry}" != "추가 촬영" || "${try_index}" -eq "${MAX_TRIES}" ]]; then
        break
      fi
    elif [[ "${try_index}" -eq "${MAX_TRIES}" ]]; then
      ask "${detail} 측정된 ΔZ가 -40 mm보다 가깝지 않습니다. 이 촬영은 남깁니다." "다음" "닫기" || true
      break
    else
      retry="$(ask "${detail} 측정된 ΔZ가 합격선에 못 미칩니다. 이 촬영은 남기고 같은 높이에서 보드만 다시 놓으세요." "추가 촬영" "이 자세 종료" || true)"
      log_remote "${pose} retry_choice=${retry:-error}"
      if [[ "${retry}" != "추가 촬영" ]]; then
        break
      fi
    fi
    try_index=$((try_index + 1))
  done
  if [[ "${try_index}" -eq 100 ]]; then
    break
  fi
done

ssh -o BatchMode=yes "${REMOTE}" "touch '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/driver-stop.txt'"
wait "${LAUNCH_SSH}" || true
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_attempt05_validate.py" || true
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_dev.sh bash -c 'colcon test --packages-select vision_path_generator robot_controller --event-handlers console_direct+'" \
  > "${OUT}/regression-test.log" 2>&1 || true
rsync -az "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-05/" "${OUT}/"
cd "${ROOT}"
python3 - << 'PY'
import json
from pathlib import Path
root = Path("evidence/P-006/camera-target/attempt-05")
log = (root / "regression-test.log").read_text(encoding="utf-8", errors="replace") if (root / "regression-test.log").is_file() else ""
passed = "100% tests passed, 0 tests failed" in log and "[  FAILED  ]" not in log
summary_path = root / "summary.json"
if not summary_path.is_file():
    raise SystemExit(0)
summary = json.loads(summary_path.read_text())
summary["checks"]["CT-24"] = passed
summary["regression_passed"] = passed
if summary["status"] == "PASS_PENDING_REGRESSION":
    summary["status"] = "PASS" if passed else "FAIL"
summary_path.write_text(json.dumps(summary, indent=2) + "\n")
lines = [f"CT-{index:02d} {'PASS' if summary['checks'][f'CT-{index:02d}'] else 'FAIL'}" for index in range(1, 25)]
lines.append(f"OVERALL {summary['status']}")
(root / "acceptance.txt").write_text("\n".join(lines) + "\n")
print(summary["status"])
PY
