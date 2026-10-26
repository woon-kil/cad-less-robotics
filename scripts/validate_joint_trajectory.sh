#!/usr/bin/env bash
# P-005 attempt 5: projected joint waypoint를 관절 공간에서 직접 잇는다.
# Cartesian planner와 projection tolerance는 바꾸지 않는다. 로봇은 움직이지 않는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="${ROOT}/third_party/dofbot-pro-cache"
DESC="${CACHE}/src/dofbot_pro_description"
MOVEIT="${CACHE}/dofbot_pro_moveit"
OUT="${ROOT}/evidence/P-005/attempt-05"
PARAMS="${OUT}/joint-trajectory.params.yaml"
KIN="${MOVEIT}/config/kinematics.yaml"

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "이 스크립트는 ROS_DISTRO=humble 컨테이너에서 실행해야 합니다. 현재: ${ROS_DISTRO:-unset}" >&2
  exit 2
fi
if [[ ! -f "${DESC}/urdf/DOFBOT_Pro-V24.urdf" || ! -f "${MOVEIT}/config/DOFBOT_Pro-V24.srdf" || ! -f "${KIN}" ]]; then
  echo "P-002 모델 캐시가 없습니다." >&2
  exit 2
fi

mkdir -p "${OUT}"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${ROOT}"
colcon build \
  --base-paths src "${CACHE}/src" \
  --packages-select robot_controller dofbot_moveit_dry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source "${ROOT}/install/setup.bash"
set -u

set +e
colcon test \
  --base-paths src \
  --packages-select vision_path_generator robot_controller \
  --event-handlers console_direct+ \
  > "${OUT}/regression-test.log" 2>&1
TEST_EXIT=$?
set -e
cat "${OUT}/regression-test.log"
if [[ "${TEST_EXIT}" -ne 0 ]]; then
  echo "회귀 테스트가 실패했습니다." >&2
  exit "${TEST_EXIT}"
fi

python3 - "${ROOT}/src/dofbot_moveit_dry/config/dry_plan.yaml" \
  "${ROOT}/src/dofbot_moveit_dry/config/geometry_motion.yaml" \
  "${ROOT}/src/dofbot_moveit_dry/config/capability_search.yaml" \
  "${ROOT}/src/dofbot_moveit_dry/config/joint_trajectory.yaml" \
  "${KIN}" \
  "${MOVEIT}/config/joint_limits.yaml" \
  "${DESC}/urdf/DOFBOT_Pro-V24.urdf" \
  "${MOVEIT}/config/DOFBOT_Pro-V24.srdf" \
  "${ROOT}/evidence/P-004/attempt-04/waypoints.csv" \
  "${ROOT}/evidence/P-004/artifacts/snapshot.pcd" \
  "${ROOT}/evidence/P-005/attempt-03" \
  "${ROOT}/evidence/P-005/attempt-04/attempt-01-vs-attempt-04.csv" \
  "${OUT}" \
  "${PARAMS}" << 'PY'
import sys
import yaml
dry, extra, capability, joints, kin, limits, urdf, srdf, waypoints, snapshot, attempt03, attempt04, out, dest = sys.argv[1:]
params = yaml.safe_load(open(dry))
params.update(yaml.safe_load(open(extra)))
params.update(yaml.safe_load(open(capability)))
params.update(yaml.safe_load(open(joints)))
params["urdf_path"] = urdf
params["srdf_path"] = srdf
params["waypoints_csv"] = waypoints
params["snapshot_pcd"] = snapshot
params["attempt03_dir"] = attempt03
params["attempt04_compare"] = attempt04
params["output_dir"] = out
params["robot_description_kinematics"] = yaml.safe_load(open(kin))
params["robot_description_planning"] = yaml.safe_load(open(limits))
with open(dest, "w") as handle:
    yaml.safe_dump({"/**": {"ros__parameters": params}}, handle, sort_keys=False)
PY

set +e
ros2 run dofbot_moveit_dry joint_trajectory_dry --ros-args --params-file "${PARAMS}" \
  > "${OUT}/diagnostic.log" 2>&1
PLAN_EXIT=$?
set -e
cat "${OUT}/diagnostic.log"
exit "${PLAN_EXIT}"
