#!/usr/bin/env bash
# Gemini 335L(2bc5:0804) USB 노드만 넘기는 Humble 카메라 컨테이너.
# --privileged 와 -v /dev:/dev 는 쓰지 않는다. P-000 컨테이너는 지우지 않는다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${CAD_LESS_CAMERA_IMAGE:-cad-less-robotics:humble-p003}"
NAME="${CAD_LESS_CAMERA_NAME:-bytesize-robotics-camera}"
WORKSPACE="/ws/cad-less-robotics"

case "${NAME}" in
  hermes-bridge|open-webui|ollama|bytesize-robotics-humble)
    echo "이 스크립트가 다루면 안 되는 컨테이너 이름이다: ${NAME}" >&2
    exit 2
    ;;
esac

if ! command -v docker >/dev/null 2>&1 || ! command -v lsusb >/dev/null 2>&1; then
  echo "docker 또는 lsusb 가 없다." >&2
  exit 1
fi

line="$(lsusb -d 2bc5:0804 | head -n 1 || true)"
if [ -z "${line}" ]; then
  echo "호스트에 2bc5:0804 가 없다. 컨테이너를 만들지 않는다." >&2
  exit 4
fi
bus="$(printf '%s\n' "${line}" | awk '{print $2}')"
dev="$(printf '%s\n' "${line}" | awk '{print $4}' | tr -d ':')"
usb_node="$(printf '/dev/bus/usb/%s/%s' "${bus}" "${dev}")"
if [ ! -e "${usb_node}" ]; then
  echo "USB 노드가 없다: ${usb_node}" >&2
  exit 4
fi

volume_args=(
  -v "${ROOT}:${WORKSPACE}"
  -v "${NAME}-build:${WORKSPACE}/build"
  -v "${NAME}-install:${WORKSPACE}/install"
  -v "${NAME}-log:${WORKSPACE}/log"
)
run_args=(
  --name "${NAME}"
  --device "${usb_node}"
  "${volume_args[@]}"
  -w "${WORKSPACE}"
)

if docker container inspect "${NAME}" >/dev/null 2>&1; then
  current_image="$(docker inspect --format '{{.Config.Image}}' "${NAME}")"
  current_devices="$(docker inspect --format '{{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' "${NAME}")"
  if [ "${current_image}" != "${IMAGE}" ] || [[ "${current_devices}" != *"${usb_node}"* ]]; then
    echo "카메라 컨테이너를 같은 이미지와 현재 USB 노드로 다시 만든다: ${usb_node}" >&2
    docker rm -f "${NAME}" >/dev/null
  elif [ "$(docker inspect --format '{{.State.Running}}' "${NAME}")" != "true" ]; then
    docker start "${NAME}" >/dev/null
  fi
fi

if ! docker container inspect "${NAME}" >/dev/null 2>&1; then
  docker run -d "${run_args[@]}" "${IMAGE}" sleep infinity >/dev/null
fi

if [ "$#" -eq 0 ]; then
  set -- bash
fi

exec docker exec -e HOST_UID -e HOST_GID -w "${WORKSPACE}" "${NAME}" bash -c '
set +u
source /opt/ros/humble/setup.bash
if [ -f /ws/cad-less-robotics/install/setup.bash ]; then
  source /ws/cad-less-robotics/install/setup.bash
fi
set -u
exec "$@"
' bash "$@"
