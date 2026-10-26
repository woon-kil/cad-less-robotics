# P-000 Docker Environment — p620

SOURCE_BASE_COMMIT=cdd8d2b00df651f4aa9e35254c06a76301214cb9
EVIDENCE_GENERATED_AT=2026-10-26 14:09 KST
EVIDENCE_COMMIT=PENDING

Mac에서 만든 `evidence/P-000/` 실패 기록은 그대로 두었다.

## Host

Ubuntu 24.04.4 LTS, x86_64, kernel 6.8.0-147-generic.
`/opt/ros`에는 jazzy만 있다. `setup.bash` 시각은 2026-01-22 02:23:01 +0900으로 작업 전과 같다.
Docker 29.4.3. GPU는 RTX 3090, driver 595.99.02. 컨테이너에는 GPU를 넘기지 않았다.

작업 후에도 다음 컨테이너 ID와 `Up 3 days`가 유지됐다.

- hermes-bridge `fc695f0fbc92`
- open-webui `64c0e4bcfced`
- ollama `acc9433c035a`

## Container

Ubuntu 22.04.5 LTS, x86_64, `ROS_DISTRO=humble`.
이미지 `cad-less-robotics:humble-p000`.
컨테이너 `bytesize-robotics-humble`. privileged 아님. network는 bridge. `/dev`와 GPU device request는 없다.

Base image는 공식 `ros:humble-ros-base-jammy`다. 2026-10-26 p620 pull의 RepoDigest는 `sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963`이다. 이 태그 안에서 Ubuntu 22.04와 `ROS_DISTRO=humble`을 확인한 뒤 digest를 Dockerfile에 고정했다. `latest`는 쓰지 않았다.

## Acceptance Criteria

AC-01 Host Ubuntu/Jazzy/Isaac 미변경: PASS
AC-02 기존 실행 컨테이너 미중단: PASS
AC-03 Container Ubuntu 22.04: PASS
AC-04 Container x86_64: PASS
AC-05 ROS_DISTRO=humble: PASS
AC-06 ROS 2 CLI: PASS
AC-07 colcon: PASS
AC-08 CMake: PASS
AC-09 GCC/G++: PASS
AC-10 MoveIt 2: PASS (26 packages)
AC-11 Eigen3: PASS (3.4.0-2ubuntu2)
AC-12 PCL: PASS (pcl_common 1.12.1)
AC-13 rosdep: PASS (0.27.0, update exit 0)
AC-14 dependency resolution: PASS
AC-15 check_environment.sh exit 0: PASS
AC-16 ENVIRONMENT PASS: PASS
AC-17 DOFBOT 미연결: PASS
AC-18 Camera 미연결: PASS
AC-19 colcon build 미실행: PASS
AC-20 Evidence: PASS

## Deviations

첫 이미지에서는 `libpcl-dev`가 `pcl_common-1.12.pc`만 제공해서 `pkg-config pcl_common`이 실패했다. 그 검사 결과는 `ENVIRONMENT FAIL`, exit 1이다. `first-check-environment.log`에 남겼다. Dockerfile에 버전 접미사를 뗀 심볼릭 링크를 넣고 이미지를 다시 만든 뒤 통과했다.

그 과정에서 이 Gate가 만든 `bytesize-robotics-humble`만 중지 후 삭제하고 새 이미지로 다시 만들었다. hermes-bridge, open-webui, ollama는 대상이 아니었다.

`rosdep update`는 root 실행 경고를 냈고 exit code는 0이었다.

`build`, `install`, `log`는 빈 named volume이다. `colcon build`는 실행하지 않았다.

## Result

PASS
