#!/bin/bash
# P-007E-R2-R2 — DaBai passive observation path STATIC AUDIT
# 🔴 READ-ONLY. cat/grep/ls/ps/sha256sum 만 사용한다.
#    금지: ros2 CLI(데몬 기동) · python import of vendor modules · 카메라 개방 · I2C 접근
WS=/home/jetson/dofbot_pro_ws
ORB=$WS/src/OrbbecSDK_ROS2-main
echo "### P-007E-R2-R2 STATIC AUDIT"
echo "### host=$(hostname) date=$(date -Is) uname=$(uname -r)"

echo; echo "## [1] 감사 수단 자체의 안전성 — 이 스크립트가 쓰는 명령"
echo "cat sed grep find ls ps sha256sum tr wc id df  — 전부 read-only"
echo "ros2 / colcon / python3 -c 'import orbbec' / v4l2-ctl --stream : 사용하지 않음"

echo; echo "## [2] DaBai DCW2 launch 파일 — 전이 폐쇄(transitive closure)"
echo "-- 대상: $ORB/orbbec_camera/launch/dabai_dcw2.launch.py"
echo "-- IncludeLaunchDescription 건수:"; grep -c "IncludeLaunchDescription" $ORB/orbbec_camera/launch/dabai_dcw2.launch.py
echo "-- 기동되는 Node / ComposableNode 선언 전부:"
grep -nE "Node\(|ComposableNode\(|ComposableNodeContainer\(|package=|executable=|plugin=" $ORB/orbbec_camera/launch/dabai_dcw2.launch.py
echo "-- ExecuteProcess / cmd= / os.system / subprocess 건수:"
grep -cE "ExecuteProcess|os\.system|subprocess|cmd=" $ORB/orbbec_camera/launch/dabai_dcw2.launch.py

echo; echo "## [3] Orbbec 트리 전체 — 로봇/컨트롤러 키워드 건수 (Log·checkpoint 제외)"
for k in Arm_Lib smbus Arm_serial move_init_pose set_torque Button_Mode Action_Mode dofbot_pro ; do
  printf '%-16s = %s\n' "$k" "$(grep -rnIi "$k" $ORB --exclude-dir=Log --exclude-dir=.ipynb_checkpoints 2>/dev/null | wc -l)"
done
echo "-- i2c 히트는 전부 카메라 내부 버스 SDK 선언인지 확인:"
grep -rnIiE "i2c" $ORB --exclude-dir=Log --exclude-dir=.ipynb_checkpoints 2>/dev/null | sed 's#^'"$ORB"'/##' | head -15
echo "-- orbbec_camera 노드 소스가 그 I2C API 를 호출하는지:"
grep -rnI -E "write_i2c|writeI2C|read_i2c|readI2C" $ORB/orbbec_camera/src $ORB/orbbec_camera/include 2>/dev/null | wc -l

echo; echo "## [4] src ↔ install 동일성 + prefix resolve"
sha256sum $ORB/orbbec_camera/launch/dabai_dcw2.launch.py \
          $WS/install/orbbec_camera/share/orbbec_camera/launch/dabai_dcw2.launch.py
echo "-- /opt/ros 쪽 orbbec_camera 중복:"; ls -d /opt/ros/*/share/orbbec_camera 2>&1
echo "-- 실행물:"; ls -l $WS/install/orbbec_camera/lib/orbbec_camera/orbbec_camera_node /opt/ros/humble/lib/rclcpp_components/component_container

echo; echo "## [5] 간접 구동 경로 — image 구독 + Arm_Lib 호출 교차"
cd $WS/src
for p in dofbot_pro_color dofbot_pro_depth dofbot_pro_vision dofbot_pro_apriltag dofbot_pro_KCF dofbot_pro_mediapipe dofbot_pro_yolov11 dofbot_pro_driver dofbot_pro_moveit dofbot_pro_voice_ctrl dofbot_pro_info dofbot_pro_interface ; do
  arm=$(grep -rlI --include=*.py -E "Arm_Lib|Arm_serial_servo|move_init_pose" $p 2>/dev/null | wc -l)
  img=$(grep -rlI --include=*.py -E "create_subscription.*Image|Subscriber.*Image|/camera/|image_raw" $p 2>/dev/null | wc -l)
  both=$(grep -rlI --include=*.py -E "Arm_Lib|Arm_serial_servo|move_init_pose" $p 2>/dev/null | xargs -r grep -lI -E "create_subscription|Subscriber|/camera/|image_raw" 2>/dev/null | wc -l)
  printf '%-24s arm=%-3s image=%-3s BOTH=%s\n' "$p" "$arm" "$img" "$both"
done
echo "-- 카메라 토픽 문자열 집계:"
grep -rhoE "/camera/[a-z_0-9/]+" --include=*.py . 2>/dev/null | sort | uniq -c | sort -rn | head -8
echo "-- 🔴 arm_driver.py 기동 시 즉시 모션 (원문):"
grep -nE "Arm_serial_servo_write6|create_subscription|Arm_Device\(\)" dofbot_pro_driver/dofbot_pro_driver/arm_driver.py
echo "-- 🔴 color_follow.py 이미지→TargetAngle 연결 (원문):"
grep -nE "create_publisher|Subscriber\(" dofbot_pro_color/dofbot_pro_color/color_follow.py

echo; echo "## [6] 현재 실행 중인 프로세스 (ps 기반 — ros2 node list 아님)"
ps -eo pid,user,etime,cmd | grep -iE "ros2 |component_container|orbbec|dofbot|rclcpp" | grep -v grep
echo "-- (위 목록 끝)"
echo "-- /dev/i2c-7 fd 보유:"
for q in $(ls /proc | grep -E "^[0-9]+$"); do
  if ls -l /proc/$q/fd 2>/dev/null | grep -q "i2c-7"; then
    echo "   PID $q  $(tr '\0' ' ' < /proc/$q/cmdline)"; fi; done
echo "-- /dev/video0|1 fd 보유:"
for q in $(ls /proc | grep -E "^[0-9]+$"); do
  if ls -l /proc/$q/fd 2>/dev/null | grep -qE "video[01]$"; then
    echo "   PID $q  $(tr '\0' ' ' < /proc/$q/cmdline)"; fi; done
echo "-- (fd 스캔 끝)"

echo; echo "## [7] 셸 환경 · autostart"
echo "-- ~/.bashrc 비대화형 가드:"; sed -n "5,9p" ~/.bashrc
echo "-- ROS 소싱·도메인:"; grep -nE "^source|ROS_DOMAIN_ID|ROS_LOCALHOST" ~/.bashrc
echo "-- 비대화형 ssh 에서 본 환경: ROS_DISTRO=[$ROS_DISTRO] ROS_DOMAIN_ID=[$ROS_DOMAIN_ID] ros2=[$(which ros2 2>/dev/null)]"
echo "-- autostart 항목:"; ls ~/.config/autostart/
echo "-- bash-1.desktop Exec 대상 존재 여부:"
grep -h "^Exec=" ~/.config/autostart/bash-1.desktop
ls -l /home/jetson/dofbot_pro/APP_DOFBOT_PRO/setting.sh 2>&1

echo; echo "## [8] 기계적 부착 provenance (URDF 원문)"
U=$WS/src/dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf
sha256sum $U
echo "-- joint 이름·타입 전부:"; grep -nE "name=\"(Arm[0-9]_Joint|DaBai_DCW2_Joint|grip_joint|Gripping_point_Joint)\"" -A1 $U | grep -E "name=|type="
echo "-- DaBai_DCW2_Joint 원문:"; sed -n "319,331p" $U
echo "-- mesh:"; ls -l $WS/src/dofbot_pro_description/meshes/DaBai_DCW2_Link.STL

echo; echo "## [9] ROS 없는 대안 경로 — UVC"
id
ls -l /dev/video0 /dev/video1
which v4l2-ctl
python3 -c "import cv2;print('cv2',cv2.__version__)" 2>&1 | tail -1
df -h /home/jetson | tail -1
echo; echo "### AUDIT END"
