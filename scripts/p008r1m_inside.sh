#!/usr/bin/env bash
# 카메라 컨테이너 안에서 뷰 하나만 저장한다. launch 셸은 pkill 하지 않는다.
set -euo pipefail
VIEW="${1:?view_a|view_b|view_c}"
case "${VIEW}" in
  view_a|view_b|view_c) ;;
  *) echo "뷰 이름이 아니다: ${VIEW}" >&2; exit 2 ;;
esac
set +u
source /opt/ros/humble/setup.bash
if [ -f /ws/cad-less-robotics/install/setup.bash ]; then
  source /ws/cad-less-robotics/install/setup.bash
fi
set -u
cd /ws/cad-less-robotics
OUT="evidence/P-008/R1M/capture/${VIEW}"
if [ -f "${OUT}/cloud.pcd" ]; then
  echo "이미 캡처가 있다: ${OUT}" >&2
  exit 3
fi
mkdir -p "${OUT}"
LOG="/tmp/p008r1m_${VIEW}.log"
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
echo $! > "/tmp/p008r1m_${VIEW}.pid"
cleanup() {
  if [ -f "/tmp/p008r1m_${VIEW}.pid" ]; then
    kill "$(cat "/tmp/p008r1m_${VIEW}.pid")" >/dev/null 2>&1 || true
    wait "$(cat "/tmp/p008r1m_${VIEW}.pid")" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
for _ in $(seq 1 90); do
  if grep -q "Serial number:" "${LOG}"; then
    break
  fi
  sleep 1
done
PYTHONPATH="/ws/cad-less-robotics/scripts:${PYTHONPATH-}" python3 -u scripts/p008r1m_capture.py \
  "${LOG}" "/ws/cad-less-robotics/${OUT}"
cp "${LOG}" "${OUT}/driver.log" || true
chmod -R a+rwX evidence/P-008/R1M || true
