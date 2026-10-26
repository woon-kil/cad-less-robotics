#!/usr/bin/env bash
# Gemini 335L PointCloud2를 Humble 카메라 컨테이너에서 측정한다.
# DOFBOT과 camera→robot TF는 만들지 않는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  export HOST_UID="$(id -u)"
  export HOST_GID="$(id -g)"
  exec "${ROOT}/scripts/run_humble_camera.sh" bash /ws/cad-less-robotics/scripts/validate_gemini_pointcloud.sh
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f /ws/cad-less-robotics/install/setup.bash ]]; then
  source /ws/cad-less-robotics/install/setup.bash
fi
set -u

OUT=/ws/cad-less-robotics/evidence/P-003
mkdir -p "${OUT}"
COMMIT=c153462518ad674650bafa4464fda72c27ab797a
SRC=/ws/cad-less-robotics/third_party/OrbbecSDK_ROS2

if [[ ! -d "${SRC}/.git" ]]; then
  echo "OrbbecSDK_ROS2 소스가 없다: ${SRC}" >&2
  exit 2
fi
actual="$(git -c "safe.directory=${SRC}" -C "${SRC}" rev-parse HEAD)"
if [[ "${actual}" != "${COMMIT}" ]]; then
  echo "wrapper commit 불일치: ${actual}" >&2
  exit 2
fi

cd /ws/cad-less-robotics
colcon build \
  --base-paths src third_party/OrbbecSDK_ROS2 \
  --packages-select orbbec_camera_msgs orbbec_camera vision_path_generator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  2>&1 | tee "${OUT}/driver-build.log"
set +u
source /ws/cad-less-robotics/install/setup.bash
set -u

set +e
timeout 8 ros2 run orbbec_camera list_devices_node > "${OUT}/list-devices.raw.txt" 2>&1
LIST_EXIT=$?
set -e
python3 - "${OUT}/list-devices.raw.txt" "${OUT}/device.txt" << 'PY'
import re, sys
raw_path, dest = sys.argv[1:]
text = open(raw_path, encoding="utf-8", errors="replace").read()
text = re.sub(r"(?i)(serial(?:\s*number)?\s*[:=]\s*)(\S+)", r"\1SERIAL_REDACTED", text)
text = re.sub(r"(?i)(uid\s*[:=]\s*)(\S+)", r"\1SERIAL_REDACTED", text)
open(dest, "w", encoding="utf-8").write(text)
print(text)
PY
rm -f "${OUT}/list-devices.raw.txt"

launch_log="${OUT}/launch.log"
ros2 launch orbbec_camera gemini_330_series.launch.py > "${launch_log}" 2>&1 &
LAUNCH_PID=$!
redact_evidence() {
  python3 - "${OUT}" << 'PY'
import pathlib, re, sys
root = pathlib.Path(sys.argv[1])
pattern = re.compile(r"(?i)(serial(?:\s*number)?\s*[:=]\s*)(\S+)")
for path in root.glob("*"):
    if not path.is_file():
        continue
    text = path.read_text(encoding="utf-8", errors="replace")
    redacted = pattern.sub(r"\1SERIAL_REDACTED", text)
    if redacted != text:
        path.write_text(redacted, encoding="utf-8")
PY
}
cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  redact_evidence || true
  if [[ -n "${HOST_UID:-}" ]]; then
    chown -R "${HOST_UID}:${HOST_GID}" "${OUT}" || true
  fi
}
trap cleanup EXIT

topic=""
for _ in $(seq 1 90); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "camera launch가 끝났다" >&2
    exit 1
  fi
  topic="$(ros2 topic list -t | awk '/sensor_msgs\/msg\/PointCloud2/ {print $1; exit}')"
  if [[ -n "${topic}" ]]; then
    break
  fi
  sleep 1
done
if [[ -z "${topic}" ]]; then
  echo "PointCloud2 토픽이 없다" >&2
  exit 1
fi

ros2 topic list -t > "${OUT}/topics.txt"
info="$(ros2 topic info -v "${topic}")"
printf '%s\n' "${info}" > "${OUT}/pointcloud-info.txt"
reliability="reliable"
if printf '%s\n' "${info}" | grep -q "Reliability: BEST_EFFORT"; then
  reliability="best_effort"
fi

python3 /ws/cad-less-robotics/scripts/pointcloud_probe.py \
  --topic "${topic}" \
  --duration 35 \
  --reliability "${reliability}" \
  --output "${OUT}/pointcloud-results.json" | tee "${OUT}/pointcloud-rate.txt"

python3 - "${OUT}/pointcloud-results.json" "${topic}" << 'PY'
import json, sys
result = json.load(open(sys.argv[1]))
topic = sys.argv[2]
default = "/camera/depth/points"
if topic == default:
    verdict = "MATCH"
else:
    verdict = "CONFIGURABLE MISMATCH"
text = f"""Existing default: {default}
Actual topic: {topic}
Type: sensor_msgs/msg/PointCloud2
frame_id: {result.get("frame_id")}
fields: {[field["name"] for field in result.get("fields") or []]}
Result: {verdict}
Forwarding test: PENDING
"""
open("/ws/cad-less-robotics/evidence/P-003/vision-input-compatibility.txt", "w").write(text)
print(text)
PY

python3 - << 'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
rclpy.init()
node = Node("gemini_tf_probe")
frames = []
def on_msg(msg):
    for transform in msg.transforms:
        frames.append(f"{transform.header.frame_id} -> {transform.child_frame_id}")
static_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
live_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
node.create_subscription(TFMessage, "/tf_static", on_msg, static_qos)
node.create_subscription(TFMessage, "/tf", on_msg, live_qos)
end = node.get_clock().now().nanoseconds + 5 * 10**9
while rclpy.ok() and node.get_clock().now().nanoseconds < end:
    rclpy.spin_once(node, timeout_sec=0.2)
unique = []
for item in frames:
    if item not in unique:
        unique.append(item)
robot = [item for item in unique if "base_link" in item or item.endswith(" -> base")]
text = "Camera TF frames:\n" + ("\n".join(unique) if unique else "(none)") + "\n"
text += "Robot-base TF created: " + ("YES\n" if robot else "NO\n")
text += "Identity camera to robot TF created by this gate: NO\n"
open("/ws/cad-less-robotics/evidence/P-003/tf.txt", "w").write(text)
print(text)
node.destroy_node()
rclpy.shutdown()
PY

input_topic="$(python3 -c 'import json; print(json.load(open("/ws/cad-less-robotics/evidence/P-003/pointcloud-results.json"))["topic"])')"
ros2 run vision_path_generator camera_node --ros-args -p source:=relay -p input_topic:="${input_topic}" > "${OUT}/relay.log" 2>&1 &
RELAY_PID=$!
sleep 3
set +e
timeout 20 ros2 topic echo --once --qos-reliability best_effort /vision/camera/points > "${OUT}/relay-once.txt" 2>&1
RELAY_EXIT=$?
set -e
kill "${RELAY_PID}" 2>/dev/null || true
wait "${RELAY_PID}" 2>/dev/null || true
python3 - "${RELAY_EXIT}" "${input_topic}" << 'PY'
import pathlib, sys
exit_code = int(sys.argv[1])
topic = sys.argv[2]
path = pathlib.Path("/ws/cad-less-robotics/evidence/P-003/vision-input-compatibility.txt")
echo_text = pathlib.Path("/ws/cad-less-robotics/evidence/P-003/relay-once.txt").read_text(errors="replace")
forward = "PASS" if exit_code == 0 and "frame_id:" in echo_text else "FAIL"
text = path.read_text().replace(
    "Forwarding test: PENDING",
    f"Forwarding test: {forward}\nRelay input parameter: {topic}\nRelay output: /vision/camera/points",
)
path.write_text(text)
print(text)
if forward != "PASS":
    raise SystemExit(1)
PY

echo "P-003 POINTCLOUD MEASUREMENT DONE"
