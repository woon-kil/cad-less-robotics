#!/usr/bin/env bash
# Mac에서 실행한다. 자세마다 확인을 받은 뒤에만 촬영한다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE="${P006C_REMOTE:-p620}"
REMOTE_ROOT="${P006C_REMOTE_ROOT:-/home/woonkil/cad-less-robotics}"
OUT="${ROOT}/evidence/P-006/camera-target/attempt-02"
POSES=(left right near far tilt_positive tilt_negative)

prompt_for() {
  case "$1" in
    left) printf '%s' "체크보드를 카메라 화면의 왼쪽으로 옮겨 주세요. 코너가 보이게 둔 뒤 촬영을 누르세요." ;;
    right) printf '%s' "체크보드를 카메라 화면의 오른쪽으로 옮겨 주세요." ;;
    near) printf '%s' "체크보드를 카메라에 더 가깝게 옮겨 주세요. 보드를 자르지 마세요." ;;
    far) printf '%s' "체크보드를 카메라에서 더 멀리 옮겨 주세요." ;;
    tilt_positive) printf '%s' "체크보드를 한 방향으로 기울여 주세요. 이미지 회전이 아니라 보드 면의 기울기입니다." ;;
    tilt_negative) printf '%s' "방금과 반대 방향으로 체크보드를 기울여 주세요." ;;
    *) return 1 ;;
  esac
}

say_pose() {
  osascript -e "display notification \"$1\" with title \"P-006C Attempt 2\"" >/dev/null 2>&1 || true
}

confirm_pose() {
  local message="$1"
  osascript << EOF
set resultDialog to display dialog "P-006C Attempt 2\n로봇은 움직이지 마세요.\n${message}" buttons {"건너뜀", "촬영"} default button "촬영" giving up after 180
if gave up of resultDialog then
  return "timeout"
end if
return button returned of resultDialog
EOF
}

rsync -az \
  "${ROOT}/scripts/p006c_attempt02_capture.py" \
  "${ROOT}/scripts/p006c_attempt02_launch.sh" \
  "${ROOT}/scripts/p006c_attempt02_validate.py" \
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
rsync -az "${OUT}/capture-plan.yaml" "${OUT}/threshold-baseline.txt" \
  "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/"

ssh -o BatchMode=yes "${REMOTE}" "mkdir -p '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/frames' && {
  echo \"observed_at: \$(date -Iseconds)\"
  lsusb -d 2bc5:0804 || true
  echo '---'
  lsusb -t || true
} > '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/hardware-inventory.txt'"

ssh -o BatchMode=yes "${REMOTE}" "rm -f '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/driver-stop.txt' '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/driver-ready.txt'"
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh bash scripts/p006c_attempt02_launch.sh" >/tmp/p006c-attempt02-launch.log 2>&1 &
LAUNCH_SSH=$!

ready=0
for _ in $(seq 1 100); do
  if ssh -o BatchMode=yes "${REMOTE}" "test -f '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/driver-ready.txt'"; then
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
  ssh -o BatchMode=yes "${REMOTE}" "touch '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/driver-stop.txt'" || true
  wait "${LAUNCH_SSH}" || true
  exit 1
fi

confirmed=0
for pose in "${POSES[@]}"; do
  say_pose "${pose}"
  choice="$(confirm_pose "$(prompt_for "${pose}")" || true)"
  printf '%s %s %s\n' "$(date -Iseconds)" "${pose}" "${choice:-error}" | ssh -o BatchMode=yes "${REMOTE}" "cat >> '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/operator-log.txt'"
  if [[ "${choice}" != "촬영" ]]; then
    if [[ "${confirmed}" -eq 0 ]]; then
      break
    fi
    continue
  fi
  confirmed=1
  ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_attempt02_capture.py --pose '${pose}' --count 4" || true
done

ssh -o BatchMode=yes "${REMOTE}" "touch '${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/driver-stop.txt'"
wait "${LAUNCH_SSH}" || true

ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_camera.sh python3 scripts/p006c_attempt02_validate.py" || true
ssh -o BatchMode=yes "${REMOTE}" "cd '${REMOTE_ROOT}' && bash scripts/run_humble_dev.sh bash -c 'colcon test --packages-select vision_path_generator robot_controller --event-handlers console_direct+'" \
  > "${OUT}/regression-test.log" 2>&1 || true
rsync -az "${REMOTE}:${REMOTE_ROOT}/evidence/P-006/camera-target/attempt-02/" "${OUT}/"
cd "${ROOT}"
python3 - << 'PY'
import json
from pathlib import Path
root = Path("evidence/P-006/camera-target/attempt-02")
log = (root / "regression-test.log").read_text(encoding="utf-8", errors="replace") if (root / "regression-test.log").is_file() else ""
passed = "100% tests passed, 0 tests failed" in log and "[  FAILED  ]" not in log
summary_path = root / "summary.json"
if not summary_path.is_file():
    raise SystemExit(0)
summary = json.loads(summary_path.read_text())
summary["checks"]["CT-24"] = passed
if summary["status"] == "PASS_PENDING_REGRESSION":
    summary["status"] = "PASS" if passed else "FAIL"
summary_path.write_text(json.dumps(summary, indent=2) + "\n")
lines = []
for index in range(1, 25):
    key = f"CT-{index:02d}"
    lines.append(f"{key} {'PASS' if summary['checks'][key] else 'FAIL'}")
lines.append(f"OVERALL {summary['status']}")
(root / "acceptance.txt").write_text("\n".join(lines) + "\n")
print(summary["status"])
PY
