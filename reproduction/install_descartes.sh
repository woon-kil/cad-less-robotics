#!/bin/bash
# Descartes Light와 boilerplate를 격리 prefix에 설치한다. /opt/ros는 바꾸지 않는다.
set -euo pipefail

WORK=${1:?work directory}
PREFIX="$WORK/descartes-prefix"
DESCARTES_COMMIT=49e2ee3305b5d7ba5018f28cf706e0dc71ecf6cf
DESCARTES_SHA=531ff2010aeac7130cc403f402b1c197bbdd00b7962211327ecce3bf2ff0d61d
BOILERPLATE_SHA=faee6cd5101b49c92d9499d8e32e851833e5a94acbe09c892e4a87f23a5aa3cd
mkdir -p "$WORK"
cd "$WORK"
curl -fsSL -o descartes.tar.gz \
  "https://github.com/swri-robotics/descartes_light/archive/${DESCARTES_COMMIT}.tar.gz"
curl -fsSL -o boilerplate.tar.gz \
  "https://github.com/ros-industrial/ros_industrial_cmake_boilerplate/archive/0.7.4.tar.gz"
echo "${DESCARTES_SHA}  descartes.tar.gz" | sha256sum -c -
echo "${BOILERPLATE_SHA}  boilerplate.tar.gz" | sha256sum -c -
tar -xzf descartes.tar.gz
tar -xzf boilerplate.tar.gz
cmake -S "$WORK/ros_industrial_cmake_boilerplate-0.7.4" -B "$WORK/build-boilerplate" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$WORK/build-boilerplate" --target install
cmake -S "$WORK/descartes_light-${DESCARTES_COMMIT}/descartes_light" -B "$WORK/build-descartes" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$PREFIX" \
  -DDESCARTES_BUILD_BGL=OFF \
  -DDESCARTES_ENABLE_TESTING=OFF \
  -DDESCARTES_PACKAGE=OFF
cmake --build "$WORK/build-descartes" --target install
test -f "$PREFIX/lib/libdescartes_light.so" || test -f "$PREFIX/lib/libdescartes_light.so.0.4.11"
echo "DESCARTES_PREFIX_OK $PREFIX"
