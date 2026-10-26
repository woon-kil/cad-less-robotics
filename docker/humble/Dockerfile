# 공식 library/ros 태그 humble-ros-base-jammy.
# 2026-10-26 p620에서 pull한 RepoDigest:
# sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963
# 같은 이미지 안에서 Ubuntu VERSION_ID=22.04, ROS_DISTRO=humble을 확인했다.
# 공식 Dockerfile은 build-essential, colcon, rosdep, ros-humble-ros-base를 이미 넣는다.
# latest 태그는 쓰지 않는다.
FROM ros:humble-ros-base-jammy@sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963

ENV DEBIAN_FRONTEND=noninteractive

# base에 없는 MoveIt 2와 PCL을 명시한다.
# Eigen, CMake, GCC, colcon, rosdep은 base에 이미 있으나 요구사항으로 다시 적는다.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    lsb-release \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libpcl-dev \
    ros-humble-pcl-conversions \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

# jammy libpcl-dev는 pcl_common-1.12.pc처럼 버전 접미사를 붙인다.
# 상위 PCL과 환경 검사는 접미사 없는 모듈 이름을 조회한다.
RUN set -eu; \
    for f in $(dpkg -L libpcl-dev | grep '/pkgconfig/.*\.pc$'); do \
      dir="$(dirname "$f")"; \
      name="$(basename "$f" .pc)"; \
      short="$(printf '%s\n' "$name" | sed -E 's/-[0-9]+(\.[0-9]+)*$//')"; \
      if [ "$short" != "$name" ] && [ ! -e "${dir}/${short}.pc" ]; then \
        ln -s "$(basename "$f")" "${dir}/${short}.pc"; \
      fi; \
    done

# base 이미지가 rosdep을 이미 초기화한다. 없을 때만 init 하고, 기존 DB는 지우지 않는다.
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi

# docker exec로 연 대화형 셸도 Humble을 본다. 없는 workspace overlay는 source하지 않는다.
RUN printf '%s\n' 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

COPY entrypoint.sh /cad-less-entrypoint.sh
RUN chmod 755 /cad-less-entrypoint.sh

ENTRYPOINT ["/cad-less-entrypoint.sh"]
CMD ["bash"]
WORKDIR /ws/cad-less-robotics
