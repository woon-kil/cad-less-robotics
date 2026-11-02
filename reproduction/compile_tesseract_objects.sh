#!/bin/bash
# 고정 Tesseract commit의 공식 번역 단위만 컴파일한다. 전체 Tesseract CMake는 쓰지 않는다.
set -euo pipefail

WORK=${1:?work directory}
COMMIT=b303c2948e715290ba7128f5739abbb4fef40546
EXPECTED=df4a5e175b2e8b0308dd2a268be87e0cf76df836938921ca7681c6c4e5f85d66
mkdir -p "$WORK"
cd "$WORK"
curl -fsSL -o src.tar.gz "https://github.com/tesseract-robotics/tesseract/archive/${COMMIT}.tar.gz"
echo "${EXPECTED}  src.tar.gz" | sha256sum -c -
tar -xzf src.tar.gz
SRC="$WORK/tesseract-${COMMIT}"
test -f "$SRC/kinematics/ur/src/ur_inv_kin.cpp"

compile_one() {
  local output=$1
  shift
  g++ -std=c++17 -c "$@" -o "$output"
}

compile_one "$WORK/ur_inv_kin.o" \
  -I"$SRC/kinematics/ur/include" -I"$SRC/kinematics/core/include" \
  -I"$SRC/common/include" -I"$SRC/scene_graph/include" -I/usr/include/eigen3 \
  "$SRC/kinematics/ur/src/ur_inv_kin.cpp"
compile_one "$WORK/inverse_kinematics.o" \
  -I"$SRC/kinematics/core/include" -I"$SRC/common/include" \
  -I"$SRC/scene_graph/include" -I/usr/include/eigen3 \
  "$SRC/kinematics/core/src/inverse_kinematics.cpp"
compile_one "$WORK/types.o" \
  -I"$SRC/common/include" -I"$SRC/scene_graph/include" -I/usr/include/eigen3 \
  "$SRC/common/src/types.cpp"
compile_one "$WORK/utils.o" \
  -I"$SRC/common/include" -I"$SRC/scene_graph/include" -I/usr/include/eigen3 \
  "$SRC/common/src/utils.cpp"
compile_one "$WORK/logging.o" \
  -I"$SRC/common/include" -I"$SRC/scene_graph/include" -I/usr/include/eigen3 \
  "$SRC/common/src/logging.cpp"
echo "TESSERACT_OBJECTS_OK $SRC"
