#!/bin/bash
# P-007E-R2-R2 Stage B2 — UVC device capability enumeration (READ-ONLY)
# 🔴 스트리밍 0 · 프레임 캡처 0 · I2C 0 · ROS 0. ioctl 질의만 한다
echo "### B2 — UVC capability enumeration"
echo "### host=$(hostname) date=$(date -Is)"

echo; echo "## [0] 사전 상태 — 로봇 모션 가능 노드 / 비디오 점유"
echo "-- image 구독 + Arm_Lib 계열 프로세스:"
ps -eo pid,cmd | grep -iE "arm_driver|dofbot_pro_driver|color_follow|grasp|apriltag|KCF|yolov11|voice_ctrl|mediapipe" | grep -v grep
echo "   (위 목록이 비어 있으면 0개)"
echo "-- /dev/video0|1 fd 보유 프로세스:"
for q in $(ls /proc | grep -E '^[0-9]+$'); do
  if ls -l /proc/$q/fd 2>/dev/null | grep -qE "video[01]$"; then
    echo "   PID $q  $(tr '\0' ' ' < /proc/$q/cmdline)"; fi; done
echo "   (위 목록이 비어 있으면 0개)"
echo "-- 디스크:"; df -h /home/jetson | tail -1

echo; echo "## [1] v4l2 장치 목록"
v4l2-ctl --list-devices 2>&1

for d in /dev/video0 /dev/video1; do
  echo; echo "## [2] $d — capability"
  v4l2-ctl --device=$d --info 2>&1
  echo "-- 지원 포맷(요약):"
  v4l2-ctl --device=$d --list-formats 2>&1
  echo "-- 현재 설정된 포맷:"
  v4l2-ctl --device=$d --get-fmt-video 2>&1
done

echo; echo "## [3] /dev/video0 상세 포맷·프레임레이트"
v4l2-ctl --device=/dev/video0 --list-formats-ext 2>&1 | head -40

echo; echo "## [4] USB 토폴로지 — 어느 장치가 어느 노드인지"
for d in 0 1; do
  echo "-- /dev/video$d → $(readlink -f /sys/class/video4linux/video$d 2>/dev/null)"
  echo "   name: $(cat /sys/class/video4linux/video$d/name 2>/dev/null)"
  echo "   index: $(cat /sys/class/video4linux/video$d/index 2>/dev/null)"
done
lsusb -d 2bc5: 2>&1

echo; echo "### B2 END"
