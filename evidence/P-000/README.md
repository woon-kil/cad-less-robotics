# P-000 Development Environment Baseline

Commit:
c227f29acf3abaa849af04276732262fa471a3aa

Date:
2026-10-26 13:41 KST

## Environment

Ubuntu:
FAIL — macOS 26.6.2 (arm64). `/etc/os-release` 없음. `lsb_release` 없음.

Architecture:
arm64

ROS:
FAIL — `/opt/ros/humble` 없음. `ROS_DISTRO` unset. `ros2` NOT FOUND.

Compiler:
FAIL — `gcc`/`g++`는 Apple clang 21.0.0. GNU GCC/G++ 아님.

CMake:
NOT FOUND

colcon:
NOT FOUND

MoveIt:
NOT FOUND — `ros2 pkg list`를 실행할 수 없음.

Eigen:
NOT FOUND — `libeigen3-dev` 확인 불가(`dpkg` 없음), `eigen3.pc` 없음. Homebrew keg 없음.

PCL:
NOT FOUND — `pcl_common.pc` 없음.

rosdep:
NOT FOUND

## Acceptance Criteria

AC-01 Ubuntu 22.04
FAIL

AC-02 ROS_DISTRO=humble
FAIL

AC-03 ROS 2 CLI available
FAIL

AC-04 colcon available
FAIL

AC-05 CMake available
FAIL

AC-06 C/C++ compiler available
FAIL

AC-07 MoveIt 2 available
FAIL

AC-08 Eigen3 available
FAIL

AC-09 PCL available
FAIL

AC-10 rosdep available
FAIL

AC-11 dependency resolution succeeds
FAIL

AC-12 check_environment.sh succeeds
FAIL

AC-13 no hardware required
PASS

## Deviations

rosdep dependency resolution NOT EXECUTED

이 호스트는 Ubuntu 22.04 + ROS 2 Humble이 아니다. macOS 결과를 PASS로 바꾸지 않았다.

`gcc`/`g++` 명령은 있으나 Apple clang이다. GNU GCC/G++ PASS로 세지 않았다.

`/etc/os-release`의 첫 조회는 셸의 `cat` alias(bat) 오류였다. `/bin/cat`으로 다시 확인해 파일 없음을 기록했다.

OBSERVATION: 프로젝트 소스의 `colcon build`/`colcon test`는 실행하지 않았다. 빌드 검증은 P-001 범위다.

Hardware: DOFBOT NO. Camera NO. udev, 시리얼, 드라이버, trajectory execution은 하지 않았다.

## Result

FAIL
