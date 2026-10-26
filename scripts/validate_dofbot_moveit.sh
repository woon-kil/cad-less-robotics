#!/usr/bin/env bash
# P-002: DOFBOT Pro arm_group collision-aware dry planning.
# bytesize-robotics-humble 안에서 실행한다. trajectory는 실행하지 않는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMIT="906da9f9120c056dd0c8766fe1a72e72a086061c"
REPO="https://github.com/atticusrussell/ballbot"
RAW="https://raw.githubusercontent.com/atticusrussell/ballbot/${COMMIT}"
DESC_REL="third_party/dofbot-pro/New_Code/Orin/DofbotPro_source/dofbot_pro_ws/src/dofbot_pro_description"
MOVEIT_REL="third_party/dofbot-pro/New_Code/Orin/DofbotPro_source/dofbot_pro_ws/src/dofbot_pro_moveit"
CACHE="${ROOT}/third_party/dofbot-pro-cache"
DESC="${CACHE}/src/dofbot_pro_description"
MOVEIT="${CACHE}/dofbot_pro_moveit"
OUT="${ROOT}/evidence/P-002"
STAMP="${CACHE}/COMMIT"

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "이 스크립트는 ROS_DISTRO=humble 컨테이너에서 실행해야 합니다. 현재: ${ROS_DISTRO:-unset}" >&2
  exit 2
fi

download() {
  local url="$1"
  local dest="$2"
  mkdir -p "$(dirname "${dest}")"
  curl -fsSL --retry 3 --retry-delay 2 -o "${dest}" "${url}"
}

if [[ ! -f "${DESC}/launch/display.launch.py" ]]; then
  mkdir -p "${DESC}/launch"
  download "${RAW}/${DESC_REL}/launch/display.launch.py" "${DESC}/launch/display.launch.py"
fi

if [[ ! -f "${STAMP}" || "$(cat "${STAMP}")" != "${COMMIT}" ]]; then
  rm -rf "${CACHE}"
  mkdir -p "${DESC}/urdf" "${DESC}/meshes" "${DESC}/launch" "${MOVEIT}/config"
  download "${RAW}/${DESC_REL}/CMakeLists.txt" "${DESC}/CMakeLists.txt"
  download "${RAW}/${DESC_REL}/package.xml" "${DESC}/package.xml"
  download "${RAW}/${DESC_REL}/urdf/DOFBOT_Pro-V24.urdf" "${DESC}/urdf/DOFBOT_Pro-V24.urdf"
  download "${RAW}/${DESC_REL}/launch/display.launch.py" "${DESC}/launch/display.launch.py"
  download "${RAW}/${MOVEIT_REL}/package.xml" "${MOVEIT}/package.xml"
  download "${RAW}/${MOVEIT_REL}/config/DOFBOT_Pro-V24.srdf" "${MOVEIT}/config/DOFBOT_Pro-V24.srdf"
  download "${RAW}/${MOVEIT_REL}/config/kinematics.yaml" "${MOVEIT}/config/kinematics.yaml"
  download "${RAW}/${MOVEIT_REL}/config/joint_limits.yaml" "${MOVEIT}/config/joint_limits.yaml"
  download "${RAW}/${MOVEIT_REL}/config/initial_positions.yaml" "${MOVEIT}/config/initial_positions.yaml"
  download "${RAW}/${MOVEIT_REL}/config/moveit_controllers.yaml" "${MOVEIT}/config/moveit_controllers.yaml"
  download "${RAW}/${MOVEIT_REL}/config/DOFBOT_Pro-V24.urdf.xacro" "${MOVEIT}/config/DOFBOT_Pro-V24.urdf.xacro"
  python3 - "${DESC}/urdf/DOFBOT_Pro-V24.urdf" "${RAW}/${DESC_REL}/meshes" "${DESC}/meshes" << 'PY'
import sys
import urllib.request
import xml.etree.ElementTree as ET
urdf, base, dest = sys.argv[1:]
root = ET.parse(urdf).getroot()
names = sorted({m.attrib["filename"].rsplit("/", 1)[-1] for m in root.iter("mesh")})
if len(names) < 10:
    raise SystemExit(f"mesh reference count {len(names)}")
for name in names:
    url = f"{base}/{name}"
    path = f"{dest}/{name}"
    urllib.request.urlretrieve(url, path)
    print(name)
PY
  printf '%s\n' "${COMMIT}" > "${STAMP}"
fi

python3 - "${DESC}/meshes" << 'PY'
import hashlib
import pathlib
import sys
import urllib.request
mesh_dir = pathlib.Path(sys.argv[1])
mirror = (
    "https://raw.githubusercontent.com/chrisantuseze/dofbot-controller/"
    "142457e307670a4080267bdf6565cde19ff962cc/"
    "dofbot_pro_ws/src/dofbot_pro_v24/meshes"
)
resolved = 0
for path in sorted(mesh_dir.glob("*.STL")):
    data = path.read_bytes()
    if not data.startswith(b"version https://git-lfs.github.com/spec/v1"):
        continue
    text = data.decode()
    oid = next(line.split(" ", 1)[1].strip() for line in text.splitlines() if line.startswith("oid "))
    oid = oid.split(":", 1)[-1]
    urllib.request.urlretrieve(f"{mirror}/{path.name}", path)
    got = hashlib.sha256(path.read_bytes()).hexdigest()
    if got != oid:
        raise SystemExit(f"mesh hash mismatch {path.name} {got} != {oid}")
    resolved += 1
    print(f"mesh {path.name} sha256 {got}")
print(f"meshes_resolved_from_lfs_oid {resolved}")
PY
rm -rf "${ROOT}/install/dofbot_pro_description"

mkdir -p "${OUT}"
HW="${CACHE}/DOFBOT_Pro-V24.hardware.urdf"
if [[ ! -f "${HW}" ]]; then
  download "${RAW}/third_party/dofbot-pro/6.Hardware_Model_File/4.%20Model%20file/DOFBOT_Pro-V24.urdf" "${HW}" || true
fi

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${ROOT}"
colcon build \
  --base-paths src "${CACHE}/src" \
  --packages-select dofbot_pro_description dofbot_moveit_dry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source "${ROOT}/install/setup.bash"
set -u

PARAMS="${OUT}/dry-plan.params.yaml"
python3 - "${ROOT}/src/dofbot_moveit_dry/config/dry_plan.yaml" \
  "${MOVEIT}/config/kinematics.yaml" \
  "${MOVEIT}/config/joint_limits.yaml" \
  "${DESC}/urdf/DOFBOT_Pro-V24.urdf" \
  "${MOVEIT}/config/DOFBOT_Pro-V24.srdf" \
  "${OUT}" \
  "${PARAMS}" << 'PY'
import sys
import yaml
adapter, kin, limits, urdf, srdf, out, dest = sys.argv[1:]
params = yaml.safe_load(open(adapter))
params["urdf_path"] = urdf
params["srdf_path"] = srdf
params["output_dir"] = out
params["robot_description_kinematics"] = yaml.safe_load(open(kin))
params["robot_description_planning"] = yaml.safe_load(open(limits))
with open(dest, "w") as handle:
    yaml.safe_dump({"/**": {"ros__parameters": params}}, handle, sort_keys=False)
PY

set +e
ros2 run dofbot_moveit_dry dry_plan_node --ros-args --params-file "${PARAMS}" > "${OUT}/planning.log" 2>&1
PLAN_EXIT=$?
set -e
cat "${OUT}/planning.log"

python3 - "${DESC}" "${MOVEIT}" "${HW}" "${OUT}" "${PLAN_EXIT}" "${REPO}" "${COMMIT}" << 'PY'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
desc, moveit, hardware, out, plan_exit, repo, commit = sys.argv[1:]
out = Path(out)
urdf = ET.parse(Path(desc) / "urdf/DOFBOT_Pro-V24.urdf").getroot()
srdf = ET.parse(Path(moveit) / "config/DOFBOT_Pro-V24.srdf").getroot()
lines = [f"robot {urdf.attrib.get('name')}", ""]
lines.append("links")
for link in urdf.findall("link"):
    lines.append(f"  {link.attrib['name']}")
lines.append("")
lines.append("joints")
lines.append("name type parent child lower upper velocity")
for joint in urdf.findall("joint"):
    parent = joint.find("parent").attrib["link"]
    child = joint.find("child").attrib["link"]
    limit = joint.find("limit")
    lower = upper = velocity = "-"
    if limit is not None:
        lower = limit.attrib.get("lower", "-")
        upper = limit.attrib.get("upper", "-")
        velocity = limit.attrib.get("velocity", "-")
    lines.append(
        f"{joint.attrib['name']} {joint.attrib['type']} {parent} {child} {lower} {upper} {velocity}"
    )
(out / "robot-model.txt").write_text("\n".join(lines) + "\n")

sr_lines = [f"srdf {srdf.attrib.get('name')}"]
for group in srdf.findall("group"):
    members = []
    for child in list(group):
        members.append(f"{child.tag}:{child.attrib.get('name', '')}")
    sr_lines.append(f"group {group.attrib['name']} {' '.join(members)}")
for ee in srdf.findall("end_effector"):
    sr_lines.append("end_effector " + " ".join(f"{k}={v}" for k, v in ee.attrib.items()))
for state in srdf.findall("group_state"):
    vals = " ".join(f"{j.attrib['name']}={j.attrib['value']}" for j in state.findall("joint"))
    sr_lines.append(f"state {state.attrib['group']} {state.attrib['name']} {vals}")
sr_lines.append(f"disable_collisions {len(srdf.findall('disable_collisions'))}")
(out / "srdf.txt").write_text("\n".join(sr_lines) + "\n")

moveit_txt = []
moveit_txt.append("used_for_planning:")
moveit_txt.append("  URDF dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf")
moveit_txt.append("  SRDF dofbot_pro_moveit/config/DOFBOT_Pro-V24.srdf")
moveit_txt.append("  kinematics.yaml")
moveit_txt.append("  joint_limits.yaml")
moveit_txt.append("not_launched:")
moveit_txt.append("  demo.launch.py")
moveit_txt.append("  spawn_controllers.launch.py")
moveit_txt.append("  DOFBOT_Pro-V24.urdf.xacro FakeSystem ros2_control")
moveit_txt.append("  moveit_controllers.yaml FollowJointTrajectory")
moveit_txt.append("adapter: src/dofbot_moveit_dry MoveItCpp plan only")
moveit_txt.append("collision_adapter: FixStartStateCollision omitted")
moveit_txt.append("execute_trajectory: false")
(out / "moveit-config.txt").write_text("\n".join(moveit_txt) + "\n")

desc_pkg = (Path(desc) / "package.xml").read_text()
moveit_pkg = (Path(moveit) / "package.xml").read_text()
hw = Path(hardware)
same = "ABSENT"
if hw.exists():
    same = "SAME" if hw.read_bytes() == (Path(desc) / "urdf/DOFBOT_Pro-V24.urdf").read_bytes() else "DIFFERENT"
desc_rel = "third_party/dofbot-pro/New_Code/Orin/DofbotPro_source/dofbot_pro_ws/src/dofbot_pro_description"
moveit_rel = "third_party/dofbot-pro/New_Code/Orin/DofbotPro_source/dofbot_pro_ws/src/dofbot_pro_moveit"
prov = f"""Robot model source:
Repository: {repo}
Branch/tag: humble
Commit: {commit}
URDF/xacro: {desc_rel}/urdf/DOFBOT_Pro-V24.urdf
SRDF: {moveit_rel}/config/DOFBOT_Pro-V24.srdf
MoveIt config: {moveit_rel}
License: dofbot_pro_moveit package.xml BSD (maintainer yahboom@163.com). dofbot_pro_description package.xml license tag is TODO.
Official git: https://github.com/YahboomTechnology/DOFBOT-Pro commit 57cc4369af73d65188d33e49227650c789779a7e has course PDFs and no URDF. GitHub license SPDX is unset.
robot_sources: ~/robot_sources/mycobot_ros2 only. DOFBOT model was not there and that tree was not modified.
ballbot commit message: Drive downloads of the Yahboom Pro hardware reference and Orin ROS 2 stack.
Hardware folder URDF vs description URDF: {same}
Mesh bytes: ballbot stores Git LFS pointers and the LFS server returned 404. Each mesh was fetched from chrisantuseze/dofbot-controller commit 142457e307670a4080267bdf6565cde19ff962cc dofbot_pro_v24/meshes only after its sha256 matched the ballbot LFS oid. URDF and SRDF still come from the ballbot snapshot, not from that mirror.
Vendor files are downloaded into third_party/dofbot-pro-cache and are not committed.
description package.xml excerpt:
{desc_pkg}
moveit package.xml excerpt:
{moveit_pkg}
"""
(out / "model-provenance.txt").write_text(prov)

arm = [g for g in srdf.findall("group") if g.attrib.get("name") == "arm_group"]
joints = [c.attrib.get("name") for c in list(arm[0])] if arm else []
ee = srdf.find("end_effector")
parent = ee.attrib.get("parent_link") if ee is not None else ""
expected = ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint"]
group_match = joints == expected
link_match = parent == "Arm5_Link"
result = "MATCH" if group_match and link_match else "MISMATCH"
compat = f"""planning_group: arm_group
code_default: arm_group
srdf_group_present: {bool(arm)}
srdf_joints: {' '.join(joints)}
end_effector_link: Arm5_Link
srdf_end_effector_parent_link: {parent}
planning_chain_tip: Arm5_Link
process_tcp_claimed: false
Gripping_point_Link: fixed child of Arm5_Link in the URDF. Not used as a calibrated TCP.
grip_group: present and separate from arm_group. robot_controller gripper controller was not added.
execute_trajectory_code_default: false
Result: {result}
"""
(out / "controller-compatibility.txt").write_text(compat)
print(f"PLAN_EXIT {plan_exit}")
print(f"COMPAT {result}")
PY

exit "${PLAN_EXIT}"
