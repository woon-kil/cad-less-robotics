#!/usr/bin/env bash
# R2.5 동안 Gemini 드라이버만 유지한다. 로봇 명령은 없다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${ROOT}/evidence/P-006C-R2.5"
mkdir -p "${OUT}/captures" "${OUT}/logs"
rm -f "${OUT}/driver-ready.txt" "${OUT}/driver-stop.txt"
LAUNCH_LOG="${OUT}/logs/launch.log"

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ROS_DISTRO=humble 컨테이너에서 실행해야 합니다. 현재: ${ROS_DISTRO:-unset}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  source "${ROOT}/install/setup.bash"
fi
set -u

ros2 launch orbbec_camera gemini_330_series.launch.py > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  python3 - "${OUT}" << 'PY'
import pathlib, re, sys
root = pathlib.Path(sys.argv[1])
pattern = re.compile(r"(?i)(serial(?:\s*number)?\s*[:=]\s*)(\S+)")
for path in root.rglob("*"):
    if not path.is_file() or path.suffix.lower() in {".png", ".jpg", ".jpeg"}:
        continue
    text = path.read_text(encoding="utf-8", errors="replace")
    redacted = pattern.sub(r"\1SERIAL_REDACTED", text)
    if redacted != text:
        path.write_text(redacted, encoding="utf-8")
PY
}
trap cleanup EXIT

ready=0
for _ in $(seq 1 90); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "camera launch가 끝났다" >&2
    exit 1
  fi
  if ros2 topic list 2>/dev/null | grep -q "/camera/depth/image_raw"; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "${ready}" -ne 1 ]]; then
  echo "depth topic이 없다" >&2
  exit 1
fi
ros2 topic list -t > "${OUT}/logs/camera-topics.txt"
date -Iseconds > "${OUT}/driver-ready.txt"

while [[ ! -f "${OUT}/driver-stop.txt" ]]; do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "camera launch가 중간에 끝났다" >&2
    exit 1
  fi
  sleep 1
done
