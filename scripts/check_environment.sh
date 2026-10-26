#!/usr/bin/env bash
# 공식 검증환경 검사 전용. 패키지를 설치하거나 시스템 설정을 바꾸지 않는다.
set -u

pass_count=0
fail_count=0
missing_count=0
required_ok=1

report() {
  local status="$1"
  local name="$2"
  local detail="$3"
  printf '%-10s  %-16s  %s\n' "$status" "$name" "$detail"
  case "$status" in
    PASS) pass_count=$((pass_count + 1)) ;;
    FAIL) fail_count=$((fail_count + 1)) ;;
    "NOT FOUND") missing_count=$((missing_count + 1)) ;;
  esac
}

require_pass() {
  if [ "$1" != "PASS" ]; then
    required_ok=0
  fi
}

check_compiler() {
  local cmd="$1"
  local line
  if ! command -v "$cmd" >/dev/null 2>&1; then
    report "NOT FOUND" "$cmd" "$cmd"
    require_pass "NOT FOUND"
    return
  fi
  line="$("$cmd" --version 2>&1 | head -n 1)"
  if printf '%s\n' "$line" | grep -qi 'clang'; then
    report "FAIL" "$cmd" "$line"
    require_pass "FAIL"
  elif printf '%s\n' "$line" | grep -Eqi 'gcc|g\+\+'; then
    report "PASS" "$cmd" "$line"
    require_pass "PASS"
  else
    report "FAIL" "$cmd" "$line"
    require_pass "FAIL"
  fi
}

echo "check_environment.sh"
echo "host: $(hostname 2>/dev/null || echo unknown)"
echo "date: $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo

arch="$(uname -m 2>/dev/null || true)"
if [ -n "$arch" ]; then
  report "PASS" "architecture" "$arch"
else
  report "NOT FOUND" "architecture" "uname -m"
fi

os_status="NOT FOUND"
os_detail="os identity unavailable"
if [ -f /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  os_detail="${ID:-unknown} ${VERSION_ID:-unknown}"
  if [ "${ID:-}" = "ubuntu" ] && [ "${VERSION_ID:-}" = "22.04" ]; then
    os_status="PASS"
  else
    os_status="FAIL"
  fi
elif command -v sw_vers >/dev/null 2>&1; then
  os_detail="$(sw_vers -productName) $(sw_vers -productVersion)"
  os_status="FAIL"
elif command -v lsb_release >/dev/null 2>&1; then
  os_detail="$(lsb_release -ds 2>/dev/null || lsb_release -a 2>/dev/null | tr '\n' ' ')"
  os_status="FAIL"
fi
report "$os_status" "Ubuntu 22.04" "$os_detail"
require_pass "$os_status"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  report "NOT FOUND" "ROS setup" "/opt/ros/humble/setup.bash"
fi

ros_distro="${ROS_DISTRO:-}"
if [ "$ros_distro" = "humble" ]; then
  report "PASS" "ROS_DISTRO" "humble"
  require_pass "PASS"
elif [ -n "$ros_distro" ]; then
  report "FAIL" "ROS_DISTRO" "$ros_distro"
  require_pass "FAIL"
else
  report "FAIL" "ROS_DISTRO" "unset"
  require_pass "FAIL"
fi

if command -v ros2 >/dev/null 2>&1; then
  if ros2 --help >/dev/null 2>&1; then
    report "PASS" "ROS 2 CLI" "ros2 --help"
    require_pass "PASS"
  else
    report "FAIL" "ROS 2 CLI" "ros2 --help failed"
    require_pass "FAIL"
  fi
else
  report "NOT FOUND" "ROS 2 CLI" "ros2"
  require_pass "NOT FOUND"
fi

if command -v colcon >/dev/null 2>&1; then
  if colcon --help >/dev/null 2>&1; then
    colcon_detail="colcon --help ok"
    if colcon_version="$(colcon version 2>/dev/null)"; then
      colcon_detail="$colcon_version"
    fi
    report "PASS" "colcon" "$colcon_detail"
    require_pass "PASS"
  else
    report "FAIL" "colcon" "colcon --help failed"
    require_pass "FAIL"
  fi
else
  report "NOT FOUND" "colcon" "colcon"
  require_pass "NOT FOUND"
fi

if command -v cmake >/dev/null 2>&1; then
  cmake_line="$(cmake --version 2>&1 | head -n 1)"
  report "PASS" "CMake" "$cmake_line"
  require_pass "PASS"
else
  report "NOT FOUND" "CMake" "cmake"
  require_pass "NOT FOUND"
fi

check_compiler gcc
check_compiler g++

if command -v ros2 >/dev/null 2>&1; then
  moveit_list="$(ros2 pkg list 2>&1 || true)"
  moveit_count="$(printf '%s\n' "$moveit_list" | grep -c -E '^moveit($|_)' || true)"
  if [ "${moveit_count:-0}" -gt 0 ]; then
    report "PASS" "MoveIt 2" "${moveit_count} packages from ros2 pkg list"
    require_pass "PASS"
  else
    report "FAIL" "MoveIt 2" "ros2 pkg list has no moveit packages"
    require_pass "FAIL"
  fi
else
  report "NOT FOUND" "MoveIt 2" "ros2 pkg list unavailable"
  require_pass "NOT FOUND"
fi

if command -v dpkg >/dev/null 2>&1 && dpkg -s libeigen3-dev >/dev/null 2>&1; then
  eigen_version="$(dpkg -s libeigen3-dev 2>/dev/null | awk '/^Version:/{print $2; exit}')"
  report "PASS" "Eigen3" "libeigen3-dev ${eigen_version:-unknown}"
  require_pass "PASS"
elif command -v pkg-config >/dev/null 2>&1 && pkg-config --exists eigen3 2>/dev/null; then
  report "PASS" "Eigen3" "pkg-config eigen3 $(pkg-config --modversion eigen3)"
  require_pass "PASS"
else
  report "NOT FOUND" "Eigen3" "libeigen3-dev and eigen3.pc missing"
  require_pass "NOT FOUND"
fi

if command -v pkg-config >/dev/null 2>&1 && pcl_version="$(pkg-config --modversion pcl_common 2>/dev/null)"; then
  report "PASS" "PCL" "pcl_common ${pcl_version}"
  require_pass "PASS"
else
  report "NOT FOUND" "PCL" "pcl_common.pc"
  require_pass "NOT FOUND"
fi

if command -v rosdep >/dev/null 2>&1; then
  if rosdep_version="$(rosdep --version 2>&1)"; then
    report "PASS" "rosdep" "$rosdep_version"
    require_pass "PASS"
  else
    report "FAIL" "rosdep" "rosdep --version failed"
    require_pass "FAIL"
  fi
else
  report "NOT FOUND" "rosdep" "rosdep"
  require_pass "NOT FOUND"
fi

echo
echo "counts: PASS=${pass_count} FAIL=${fail_count} NOT_FOUND=${missing_count}"
if [ "$required_ok" -eq 1 ]; then
  echo "ENVIRONMENT PASS"
  exit 0
fi
echo "ENVIRONMENT FAIL"
exit 1
