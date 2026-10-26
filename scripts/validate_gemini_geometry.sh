#!/usr/bin/env bash
# Gemini 스냅샷 한 장을 boundary 모드로 6D waypoint까지 측정한다.
# DOFBOT, MoveIt, camera→robot TF는 만들지 않는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${P004_PHASE:-}" != "camera" ]]; then
  export HOST_UID="$(id -u)"
  export HOST_GID="$(id -g)"
  mkdir -p "${ROOT}/evidence/P-004"
  set +e
  "${ROOT}/scripts/run_humble_camera.sh" bash -lc 'P004_PHASE=camera bash /ws/cad-less-robotics/scripts/validate_gemini_geometry.sh'
  CAMERA_EXIT=$?
  set -e
  "${ROOT}/scripts/run_humble_dev.sh" bash -lc 'set +u; source /opt/ros/humble/setup.bash; set -u
    cd /ws/cad-less-robotics
    colcon build --packages-select vision_path_generator robot_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
    colcon test --packages-select vision_path_generator robot_controller --event-handlers console_direct+
    colcon test-result --verbose --test-result-base /ws/cad-less-robotics/build/vision_path_generator
    colcon test-result --verbose --test-result-base /ws/cad-less-robotics/build/robot_controller
  ' | tee "${ROOT}/evidence/P-004/regression-test.log"
  if [[ -n "${HOST_UID:-}" ]]; then
    docker exec bytesize-robotics-humble chown -R "${HOST_UID}:${HOST_GID}" /ws/cad-less-robotics/evidence/P-004 || true
  fi
  exit "${CAMERA_EXIT}"
fi

set +u
source /opt/ros/humble/setup.bash
set -u
OUT=/ws/cad-less-robotics/evidence/P-004
ART="${OUT}/artifacts"
mkdir -p "${ART}"
cd /ws/cad-less-robotics

colcon build --packages-select vision_path_generator --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source /ws/cad-less-robotics/install/setup.bash
set -u

ros2 launch orbbec_camera gemini_330_series.launch.py > /tmp/p004-camera-launch.log 2>&1 &
LAUNCH_PID=$!
cleanup_launch() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup_launch EXIT
python3 /ws/cad-less-robotics/scripts/capture_gemini_snapshot.py \
  --pcd "${ART}/snapshot.pcd" \
  --meta "${ART}/snapshot-meta.txt"
cleanup_launch
trap - EXIT

sha256="$(sha256sum "${ART}/snapshot.pcd" | awk '{print $1}')"
printf 'sha256=%s\n' "${sha256}" >> "${ART}/snapshot-meta.txt"
cat > "${OUT}/environment.txt" << EOF
Host: p620
Container: bytesize-robotics-camera
Image: cad-less-robotics:humble-p003
ROS_DISTRO: humble
Snapshot topic: /camera/depth/points
Path mode: boundary
Waypoint frame: camera_depth_optical_frame
MoveIt planning: 0
Robot execution: 0
Camera to robot TF created by this gate: 0
EOF

set +e
ros2 run vision_path_generator geometry_report \
  "${ART}/snapshot.pcd" "${ART}/snapshot-meta.txt" "${OUT}"
GEOM_EXIT=$?
set -e
TOPIC_EXIT=2
if [[ -f "${OUT}/waypoints.csv" && -f "${OUT}/waypoints-run2.csv" && -f "${OUT}/waypoints-run3.csv" ]]; then
  {
    echo "sha256_waypoints $(sha256sum "${OUT}/waypoints.csv" | awk '{print $1}')"
    echo "sha256_run2 $(sha256sum "${OUT}/waypoints-run2.csv" | awk '{print $1}')"
    echo "sha256_run3 $(sha256sum "${OUT}/waypoints-run3.csv" | awk '{print $1}')"
  } >> "${OUT}/determinism.txt"
  frame_id="$(awk -F= '$1=="frame_id"{print $2}' "${ART}/snapshot-meta.txt")"
  stamp_sec="$(awk -F= '$1=="stamp_sec"{print $2}' "${ART}/snapshot-meta.txt")"
  stamp_nanosec="$(awk -F= '$1=="stamp_nanosec"{print $2}' "${ART}/snapshot-meta.txt")"
  ros2 run vision_path_generator path_extractor_node --ros-args \
    -p path_mode:=boundary \
    -p require_transform:=false \
    -p input_topic:=/vision/camera/points \
    -p output_topic:=/vision/waypoints > "${OUT}/path-node.log" 2>&1 &
  NODE_PID=$!
  set +e
  python3 /ws/cad-less-robotics/scripts/check_waypoint_topic.py \
    --pcd "${ART}/snapshot.pcd" \
    --csv "${OUT}/waypoints.csv" \
    --frame-id "${frame_id}" \
    --stamp-sec "${stamp_sec}" \
    --stamp-nanosec "${stamp_nanosec}" \
    --output "${OUT}/waypoints-topic.txt"
  TOPIC_EXIT=$?
  set -e
  kill "${NODE_PID}" 2>/dev/null || true
  wait "${NODE_PID}" 2>/dev/null || true
else
  printf 'comparison NOT EXECUTED\n' > "${OUT}/waypoints-topic.txt"
fi

python3 - "${sha256}" "${GEOM_EXIT}" "${TOPIC_EXIT}" << 'PY'
import pathlib, sys
sha, geom, topic = sys.argv[1:]
meta = dict(line.split("=", 1) for line in pathlib.Path("/ws/cad-less-robotics/evidence/P-004/artifacts/snapshot-meta.txt").read_text().splitlines() if "=" in line)
text = "\n".join([
    "camera Gemini 335L",
    "topic " + meta.get("topic", ""),
    "frame_id " + meta.get("frame_id", ""),
    "stamp_sec " + meta.get("stamp_sec", ""),
    "stamp_nanosec " + meta.get("stamp_nanosec", ""),
    "width " + meta.get("width", ""),
    "height " + meta.get("height", ""),
    "point_count " + meta.get("point_count", ""),
    "finite_count " + meta.get("finite_count", ""),
    "fields " + meta.get("fields", ""),
    "sha256 " + sha,
    "artifact /home/woonkil/cad-less-robotics/evidence/P-004/artifacts/snapshot.pcd",
    "git_commit no",
    "geometry_exit " + geom,
    "topic_exit " + topic,
    "",
])
pathlib.Path("/ws/cad-less-robotics/evidence/P-004/input-cloud.txt").write_text(text)
print(text)
PY

if [[ -n "${HOST_UID:-}" ]]; then
  chown -R "${HOST_UID}:${HOST_GID}" "${OUT}" || true
fi
echo "P-004 CAMERA PHASE EXIT geometry=${GEOM_EXIT} topic=${TOPIC_EXIT}"
