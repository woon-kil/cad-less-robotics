#!/usr/bin/env bash
# W-001 한 장. launch 셸은 pkill 하지 않는다. 카메라는 옮기지 않는다.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
if [ -f /ws/cad-less-robotics/install/setup.bash ]; then
  source /ws/cad-less-robotics/install/setup.bash
fi
set -u
cd /ws/cad-less-robotics
OUT="evidence/W-001"
if [ -f "${OUT}/cloud.pcd" ]; then
  echo "이미 캡처가 있다: ${OUT}" >&2
  exit 3
fi
mkdir -p "${OUT}"
LOG="/tmp/w001_gemini.log"
: > "${LOG}"
stdbuf -oL -eL ros2 launch orbbec_camera gemini_330_series.launch.py \
  depth_registration:=true \
  align_mode:=SW \
  align_target_stream:=COLOR \
  enable_colored_point_cloud:=true \
  enable_publish_extrinsic:=true \
  enable_point_cloud:=true \
  enable_frame_sync:=true \
  > "${LOG}" 2>&1 &
echo $! > /tmp/w001_gemini.pid
cleanup() {
  if [ -f /tmp/w001_gemini.pid ]; then
    kill "$(cat /tmp/w001_gemini.pid)" >/dev/null 2>&1 || true
    wait "$(cat /tmp/w001_gemini.pid)" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
for _ in $(seq 1 90); do
  if grep -q "Serial number:" "${LOG}"; then
    break
  fi
  sleep 1
done
PYTHONPATH="/ws/cad-less-robotics/scripts:${PYTHONPATH-}" python3 -u scripts/w001_capture.py \
  "${LOG}" "/ws/cad-less-robotics/${OUT}"
cp "${LOG}" "${OUT}/driver.log" || true
chmod -R a+rwX evidence/W-001 || true
