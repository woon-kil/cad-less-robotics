# PRE-P000 Ubuntu GPU Server Inventory

Host alias: `p620` (192.168.0.148)
Survey time: 2026-10-26 13:47 KST
Method: SSH read-only commands. Raw log is `inventory-full.log`.

이 문서는 Acceptance Gate가 아니다. P-000 환경을 확정하지 않는다.

## Overall

POTENTIALLY SUITABLE

호스트는 Ubuntu 24.04.4 LTS, x86_64, RTX 3090이다. `/opt/ros`에는 `jazzy`만 있다. Humble은 없다. MoveIt 패키지 목록은 비어 있다. 컴파일러, CMake, colcon, rosdep, Eigen3, PCL, Docker, nvidia-container-toolkit은 있다.

## Answers

1. Ubuntu 24.04.4 LTS (noble). 22.04가 아니다.
2. Architecture는 x86_64이다.
3. `/opt/ros`에는 `jazzy`만 있다. `setup.bash`는 `/opt/ros/jazzy/setup.bash` 하나다.
4. Humble은 설치되어 있지 않다. `/opt/ros/humble`은 없다.
5. Jazzy가 설치되어 있다. 다른 ROS distribution 디렉터리는 보이지 않았다.
6. MoveIt 2는 확인된 ROS distribution에 없다. Jazzy를 source한 `ros2 pkg list | grep '^moveit'` 출력이 없고, `/opt/ros/jazzy/share` 아래 moveit 디렉터리도 없다.
7. colcon은 있다. `/usr/bin/colcon`, `colcon --help` 성공. 버전 숫자는 이 조사에서 출력하지 않았다.
8. rosdep은 있다. `/usr/bin/rosdep`, version 0.27.0.
9. GCC 13.3.0, G++ 13.3.0, CMake 3.28.3, GNU Make 4.3이 있다. clang과 ninja는 NOT FOUND.
10. Eigen3가 있다. `libeigen3-dev` 3.4.0-4build0.1, `/usr/include/eigen3` 존재.
11. PCL이 있다. `pcl_common` 1.14.0, `libpcl-dev` 1.14.0+dfsg-1.
12. Open3D는 NOT FOUND. `ModuleNotFoundError: No module named 'open3d'`. 이것만으로 이 조사를 실패로 보지 않는다.
13. GPU는 NVIDIA GeForce RTX 3090 24GB. Driver 595.99.02. `nvidia-smi`가 표시한 CUDA Version은 13.2. `nvcc`는 NOT FOUND. 조사 시점 GPU 프로세스는 없었다.
14. Docker 29.4.3이 설치되어 있다. `nvidia-container-toolkit` 1.19.0-1이 설치되어 있다.
15. `~/isaacsim`, `~/.nvidia-omniverse`가 있다. 홈 목록에 `twinbot_ws`, `robot_sources`, `maintenance-agentic-twin`도 있다. 후보 이름 `ros2_ws`, `catkin_ws`, `colcon_ws`, `moveit_ws`는 없다. 디렉터리는 수정하지 않았다.
16. 호스트 OS를 그대로 공식 Ubuntu 22.04 + Humble 검증 환경으로 쓰기는 어렵다. 기존 Jazzy, Isaac Sim, 실행 중 Docker 컨테이너를 유지한 채 격리된 Humble 환경을 검토할 여지는 있다.
17. Ubuntu 22.04 + ROS 2 Humble 환경은 이 호스트에 이미 존재하지 않는다. 로컬 Docker 이미지에 `ubuntu:22.04`와 `nvidia/cuda:12.4.1-base-ubuntu22.04`는 있으나 `ros:humble` 이미지는 없다.
18. 기존 환경을 건드리지 않는 후보는 별도 Humble 컨테이너 검토, 또는 별도 Ubuntu 22.04 호스트다. 이번 조사에서는 어느 쪽도 설치하지 않았다.

## Current shell

SSH 비대화형 셸의 `ROS_`, `AMENT_`, `COLCON_` 환경변수는 없었다. `ros2`는 Jazzy setup을 source한 subshell에서만 `/opt/ros/jazzy/bin/ros2`로 잡혔다. 그 subshell의 패키지 수는 287이다. `.bashrc`는 수정하지 않았다.

## Running containers

시작하거나 종료하지 않았다. 조사 시점:

- `hermes-bridge` Up 3 days
- `open-webui` Up 3 days (healthy)
- `ollama` Up 3 days

## Repository

PROJECT REPOSITORY: NOT PRESENT

`cad-less-robotics`를 clone하지 않았다.

## Hardware devices

USB는 루트 허브와 Realtek USB Audio Front/Rear만 보였다. `/dev/ttyUSB*`, `/dev/ttyACM*`는 없다. DOFBOT과 카메라는 연결하거나 제어하지 않았다.

## Changes made on the server

NONE
