#!/bin/bash
# P-007E-R2-R2 Stage B3 — passive single-frame RGB capture (MAX 1 FRAME)
# 🔴 ROS 0 · Arm_Lib 0 · I2C 0 · servo 0. 현재 설정 포맷을 그대로 쓴다(S_FMT 0)
set -u
OUT=/tmp/p007e_r2r2_b3_frame.mjpg
echo "### B3 — single-frame passive capture"
echo "### host=$(hostname) date=$(date -Is)"

echo; echo "## [G1] 캡처 직전 가드"
echo "-- 모션 가능 노드(arm_driver|dofbot_pro_driver|color_follow|grasp|apriltag|KCF|yolov11|voice_ctrl|mediapipe):"
N=$(ps -eo pid,cmd | grep -iE "arm_driver|dofbot_pro_driver|color_follow|grasp|apriltag|KCF|yolov11|voice_ctrl|mediapipe" | grep -v grep | wc -l)
echo "   count=$N"
if [ "$N" != "0" ]; then echo "🔴 ABORT — UNEXPECTED_NODE"; exit 2; fi
echo "-- /dev/video0 점유 프로세스:"
V=0
for q in $(ls /proc | grep -E '^[0-9]+$'); do
  if ls -l /proc/$q/fd 2>/dev/null | grep -qE "video0$"; then V=$((V+1)); fi; done
echo "   count=$V"
if [ "$V" != "0" ]; then echo "🔴 ABORT — DEVICE_BUSY"; exit 3; fi
echo "-- ROS 환경 비상속 확인: ROS_DISTRO=[${ROS_DISTRO:-}] ros2=[$(which ros2 2>/dev/null)]"
echo "-- 캡처 전 포맷:"; v4l2-ctl --device=/dev/video0 --get-fmt-video 2>&1 | sed 's/^/   /'
rm -f "$OUT"

echo; echo "## [C] 1프레임 캡처 — stream-count=1 · S_FMT 없음 · skip 없음"
set +e
timeout 25 v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=1 --stream-to="$OUT" 2>&1 | sed 's/^/   /'
RC=$?
set -e
echo "   v4l2-ctl rc=$RC"

echo; echo "## [R] 결과물"
ls -l "$OUT" 2>&1
echo "-- file:"; file "$OUT" 2>&1
echo "-- sha256:"; sha256sum "$OUT" 2>&1
echo "-- 첫 4바이트(JPEG SOI=ffd8ffe0/ffd8ffdb 계열):"; head -c 4 "$OUT" | od -An -tx1

echo; echo "## [G2] 캡처 직후 가드 재확인"
echo "-- 새 모션 가능 노드:"
ps -eo pid,cmd | grep -iE "arm_driver|dofbot_pro_driver|color_follow|grasp|apriltag|KCF|yolov11|voice_ctrl|mediapipe" | grep -v grep | wc -l
echo "-- /dev/i2c-7 fd 보유 프로세스 (B0 기준선: 2264 순환 · 3091 · 3092):"
for q in $(ls /proc | grep -E '^[0-9]+$'); do
  if ls -l /proc/$q/fd 2>/dev/null | grep -q "i2c-7"; then
    echo "   PID $q  $(tr '\0' ' ' < /proc/$q/cmdline)"; fi; done
echo "-- 캡처 후 포맷(변경 없어야 한다):"; v4l2-ctl --device=/dev/video0 --get-fmt-video 2>&1 | sed 's/^/   /'
echo "-- /dev/video0 잔존 점유:"
for q in $(ls /proc | grep -E '^[0-9]+$'); do
  if ls -l /proc/$q/fd 2>/dev/null | grep -qE "video0$"; then echo "   PID $q"; fi; done
echo "   (비어 있으면 0)"
echo; echo "### B3 END"
