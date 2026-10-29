# CAD-less 3D Vision-Guided Robotics

CAD 없이 뎁스 카메라 포인트 클라우드에서 3D 작업 경로를 만들고, DOFBOT Pro가 MoveIt 2 Cartesian 궤적으로 따라가게 하는 ROS 2 Humble PoC다. 이 저장소 루트가 colcon 워크스페이스다.

하드웨어 드라이버, URDF, MoveIt 설정 패키지는 Yahboom DOFBOT Pro 워크스페이스를 그대로 쓴다. 여기서 만드는 것은 점군에서 웨이포인트를 뽑는 노드와, 그 포즈를 Cartesian 궤적으로 실행하는 노드다.

`/vision/waypoints`의 P-004 결과는 `camera_depth_optical_frame` 기준의 camera-frame 6D geometry waypoints다. camera→base 보정이 없으므로 로봇이 실행할 waypoint가 아니다.

## 데이터 흐름

1. Orbbec 드라이버가 `sensor_msgs/PointCloud2`를 발행한다. `camera_node`가 필드를 확인한 뒤 `/vision/camera/points`로 넘긴다.
2. `path_extractor_node`가 VoxelGrid, StatisticalOutlierRemoval, RANSAC 평면 제거, 법선, 2mm 웨이포인트 순서로 `geometry_msgs/PoseArray`(`/vision/waypoints`)를 만든다.
3. `motion_planner_node`가 `arm_group`에 대해 `computeCartesianPath`를 호출한다. 전체 경로가 만들어지고 안전 조건을 통과하며 `execute_trajectory:=true`일 때만 `execute`한다.

디버그 점군은 `/vision/cloud_filtered`, `/vision/cloud_object`다. 계획된 관절 궤적은 `/motion_planner_node/planned_trajectory`에만 나가며, 컨트롤러 명령 토픽으로 직접 쓰지 않는다.

점군 QoS는 `SensorDataQoS`(best effort)다. `ros2 topic echo`로 보려면 `--qos-reliability best_effort`가 필요하다.

## 빌드

Ubuntu 22.04, ROS 2 Humble에서 빌드한다. 이 개발 Mac에는 Humble이 없어 여기서 `colcon build`를 돌리지 않는다.

```bash
sudo apt install ros-humble-moveit ros-humble-pcl-conversions ros-humble-tf2-sensor-msgs \
  ros-humble-ament-cmake-gtest libpcl-dev
source /opt/ros/humble/setup.bash
cd /path/to/cad-less-robotics
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select vision_path_generator robot_controller
colcon test-result --verbose
```

기하 테스트만 검사한다. 실기 동작과 핸드아이 오차는 측정값이 생기기 전에는 통과로 적지 않는다.

## 실행

Yahboom MoveIt과 카메라 드라이버를 먼저 띄운 다음:

```bash
ros2 launch robot_controller system_bringup.launch.py
```

실기 명령은 기본으로 꺼져 있다. 켤 때는 아래처럼 명시한다.

```bash
ros2 launch robot_controller system_bringup.launch.py execute_trajectory:=true
```

하드웨어 없이 PCD만 보려면 MoveIt 노드를 빼면 된다. 포즈 프레임은 PCD의 `frame_id`가 된다.

```bash
ros2 launch robot_controller system_bringup.launch.py \
  camera_source:=pcd pcd_path:=/absolute/scan.pcd \
  require_transform:=false start_motion_planner:=false
```

`config/dofbot_controllers.yaml`은 Yahboom 문서의 `arm_group_controller`와 `Arm1_Joint`–`Arm5_Joint`를 옮긴 참조 파일이다. 벤더 드라이버가 이미 컨트롤러를 띄우면 이 파일을 중복 로드하지 않는다. 런치도 이 yaml을 controller_manager에 넣지 않는다.

## 처리 규칙

- 단위는 미터다. 센서가 밀리미터면 `input_scale:=0.001`.
- 평면 법선은 세계 +Z가 아니라, 평면을 뺀 뒤 제품 점이 있는 쪽으로 맞춘다. 이 부호는 작업면 제거에만 쓴다.
- 표면 법선 부호는 `camera_depth_optical_frame`의 카메라 원점 `(0,0,0)`을 향하게 고른다. 이웃 법선으로 부호를 전파하지 않는다.
- 법선 축 분산이 다음 주성분의 절반보다 크면 그 법선은 신뢰하지 않는다. 양쪽의 신뢰 가능한 법선이 90° 미만으로 같을 때만 둘의 평균으로 바꾼다. 그 경계는 평활하지 않는다.
- 툴 +Z는 그 바깥 법선의 반대(소재 쪽)이고 툴 +X는 경로 접선이다.
- `path_mode`는 `auto`(기본, 경계 우선 후 능선), `boundary`, `crest`다. 경계는 보이는 점군의 가장자리이며, 재봉선과 같다는 측정은 없다.
- 클라우드 스탬프를 PoseArray에 그대로 넘긴다. 스탬프가 없거나 3초보다 오래되면 실행하지 않는다. PCD 재생만 발행 시각을 새로 찍는다.
- Cartesian 분율이 전체 경로(1.0, 허용 오차 0.001)에 못 미치면 실행하지 않는다. `min_fraction`(기본 0.95)은 그 하한이다.
- 궤적 시간이 비었거나 관절 속도가 `fallback_joint_velocity`(기본 0.4 rad/s)를 넘으면 다시 맞춘다. 그래도 넘으면 실행하지 않는다.
- 원점에서 `max_reach_m`(기본 1m)를 넘는 포즈는 거절한다. 이 검사는 거리 한계를 넘는 경우만 거절하며, 모든 단위 오류를 잡지는 못한다.
- `jump_threshold` 기본값 1.0은 Humble 튜토리얼의 0.0이 실기에서 위험하다고 경고한 데 따른 출발값이다. 검증된 안전 한계는 아니다.

접근 2cm, 후퇴 2cm는 툴 +Z의 반대 방향이다. 작업대 위 표면에서 툴이 아래를 보면 접근 포즈는 그 위쪽에 생긴다.

## 확인이 필요한 항목

아래는 코드 기본값이다. 실기와 다르면 파라미터만 바꾸면 된다.

- 플래닝 그룹은 스펙 초안의 `arm`이 아니라 Yahboom MoveIt 2 문서의 `arm_group`이다. 조인트는 `Arm1_Joint`–`Arm5_Joint`만 넣었다. 마케팅의 6 DOF와 달리 그 문서에는 여섯 번째 팔 조인트 이름이 없다.
- 엔드이펙터 링크 기본값은 그리퍼 부모인 `Arm5_Link`다. MoveIt이 거절하면 그룹 기본 팁을 쓴다. 실제 TCP와 같은지는 확인되지 않았다.
- 그리퍼 조인트 이름은 문서에 없어 yaml에 없다.
- 카메라 기본 토픽은 Orbbec ROS 2 래퍼의 `/camera/depth/points`다. DOFBOT Pro 일체 카메라가 같은 이름인지는 확인되지 않았다.
- 핸드아이 변환은 측정값이 없다. `publish_hand_eye_tf:=true`는 측정한 xyz/rpy를 같이 줄 때만 쓴다. 0은 보정값이 아니다.
- P-005의 배치는 `TEST_ONLY_SYNTHETIC_PLACEMENT`다. calibration이 아니고 TF로 게시하지 않는다. 계획 링크는 `Arm5_Link`이며 process TCP가 아니다.
- `ros2` 브랜치 perception_pcl CMake에서 SAC 분할 타깃이 주석이라, RANSAC은 `pcl_ros` 노드가 아니라 이 패키지 안에서 직접 호출한다. Humble apt 바이너리를 이 세션에서 설치해 확인하지는 않았다.

## 참고

- 에이전트 운영 원칙은 `AGENTS.md`의 Evidence-First Engineering이다.
- [MoveIt 2 Humble Move Group C++ Interface](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [MoveGroupInterface Humble API](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html)
- [Yahboom DOFBOT Pro MoveIt2 Configuration](https://www.yahboom.net/public/upload/upload-html/1768547833/MoveIt2%20Configuration.html)
- [Orbbec ROS 2 point cloud](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/point_cloud.html)
