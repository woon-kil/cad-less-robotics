#!/usr/bin/env bash
# W-002A 검출 화면. launch 셸은 pkill 하지 않는다.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
if [ -f /ws/cad-less-robotics/install/setup.bash ]; then
  source /ws/cad-less-robotics/install/setup.bash
fi
set -u
cd /ws/cad-less-robotics
mkdir -p evidence/W-002A/live
rm -f evidence/W-002A/STOP
LOG="/tmp/w002a_gemini.log"
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
echo $! > /tmp/w002a_gemini.pid
cleanup() {
  if [ -f /tmp/w002a_gemini.pid ]; then
    kill "$(cat /tmp/w002a_gemini.pid)" >/dev/null 2>&1 || true
    wait "$(cat /tmp/w002a_gemini.pid)" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
for _ in $(seq 1 90); do
  if grep -q "Serial number:" "${LOG}"; then
    break
  fi
  sleep 1
done
PYTHONPATH="/ws/cad-less-robotics/scripts:${PYTHONPATH-}" python3 -u scripts/w002a_view.py
cp "${LOG}" evidence/W-002A/driver.log || true
chmod -R a+rwX evidence/W-002A || true
