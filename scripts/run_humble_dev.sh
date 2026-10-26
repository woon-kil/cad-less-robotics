#!/usr/bin/env bash
# p620에서 이 저장소만 Humble 컨테이너에 마운트한다.
# 호스트 ROS, NVIDIA, 기존 컨테이너는 다루지 않는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${CAD_LESS_HUMBLE_IMAGE:-cad-less-robotics:humble-p000}"
NAME="${CAD_LESS_HUMBLE_NAME:-bytesize-robotics-humble}"
WORKSPACE="/ws/cad-less-robotics"

case "${NAME}" in
  hermes-bridge|open-webui|ollama)
    echo "보호 중인 컨테이너 이름은 사용할 수 없다: ${NAME}" >&2
    exit 2
    ;;
esac

if ! command -v docker >/dev/null 2>&1; then
  echo "docker 가 없다." >&2
  exit 1
fi

volume_args=(
  -v "${ROOT}:${WORKSPACE}"
  -v "${NAME}-build:${WORKSPACE}/build"
  -v "${NAME}-install:${WORKSPACE}/install"
  -v "${NAME}-log:${WORKSPACE}/log"
)

run_args=(
  --name "${NAME}"
  "${volume_args[@]}"
  -w "${WORKSPACE}"
)

ensure_container() {
  if docker container inspect "${NAME}" >/dev/null 2>&1; then
    local current_image
    current_image="$(docker inspect --format '{{.Config.Image}}' "${NAME}")"
    if [ "${current_image}" != "${IMAGE}" ]; then
      echo "컨테이너 ${NAME} 이(가) 다른 이미지(${current_image})로 이미 있다. 삭제하지 않는다." >&2
      exit 3
    fi
    if [ "$(docker inspect --format '{{.State.Running}}' "${NAME}")" != "true" ]; then
      docker start "${NAME}" >/dev/null
    fi
    return
  fi

  docker run -d "${run_args[@]}" "${IMAGE}" sleep infinity >/dev/null
}

ensure_container

tty_flags=()
if [ -t 0 ] && [ -t 1 ]; then
  tty_flags=(-it)
fi

if [ "$#" -eq 0 ]; then
  set -- bash
fi

exec docker exec "${tty_flags[@]}" -w "${WORKSPACE}" "${NAME}" bash -c '
source /opt/ros/humble/setup.bash
if [ -f /ws/cad-less-robotics/install/setup.bash ]; then
  source /ws/cad-less-robotics/install/setup.bash
fi
exec "$@"
' bash "$@"
