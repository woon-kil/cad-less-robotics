#!/bin/bash
# 컨테이너 PID 1에서 Humble을 켠다. build 전 install/setup.bash는 건너뛴다.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash" --

workspace_setup="${CAD_LESS_WORKSPACE_SETUP:-/ws/cad-less-robotics/install/setup.bash}"
if [ -f "${workspace_setup}" ]; then
  # shellcheck disable=SC1090
  source "${workspace_setup}"
fi

if [ "$#" -eq 0 ]; then
  exec bash
fi
exec "$@"
