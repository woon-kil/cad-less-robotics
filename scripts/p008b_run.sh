#!/usr/bin/env bash
# 컨테이너 안에서 실행한다. ROS 환경은 run_humble_camera.sh 가 넣어 준다.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
if [ -f /ws/cad-less-robotics/install/setup.bash ]; then
  source /ws/cad-less-robotics/install/setup.bash
fi
set -u
cd /ws/cad-less-robotics
if [ ! -f evidence/P-008B/acquisition_contract.json ]; then
  echo "acquisition contract가 없다. 촬영하지 않는다." >&2
  exit 2
fi
mkdir -p evidence/P-008B/preflight
LOG=/tmp/p008b_driver.log
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
echo $! > /tmp/p008b_driver.pid
for _ in $(seq 1 90); do
  if grep -q "Serial number:" "${LOG}"; then
    break
  fi
  sleep 1
done
set +e
PYTHONPATH="/ws/cad-less-robotics/scripts:${PYTHONPATH-}" python3 -u scripts/p008b_acquire.py \
  /ws/cad-less-robotics/evidence/P-008B "${LOG}"
status=$?
set -e
kill "$(cat /tmp/p008b_driver.pid)" >/dev/null 2>&1 || true
pkill -f gemini_330_series.launch.py >/dev/null 2>&1 || true
pkill -f orbbec_camera_node >/dev/null 2>&1 || true
cp "${LOG}" evidence/P-008B/preflight/driver.log || true
chmod -R a+rX evidence/P-008B || true
exit "${status}"
