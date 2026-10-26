# CAD-less 3D Vision-Guided Robotics PoC 플랜

- 작성: 2026-10-26 12:55 (KST)
- 상태: 승인 대기
- 대상: 이 저장소 루트를 ROS 2 Humble colcon 워크스페이스로 사용

## 목표

Footwear/Fashion 포트폴리오용으로, CAD 없이 뎁스 카메라 포인트 클라우드에서 3D 작업 경로를 만들고 DOFBOT Pro가 MoveIt 2 Cartesian 궤적으로 따라가게 하는 PoC 골격을 만든다. 하드웨어 드라이버와 URDF는 Yahboom 패키지를 재사용하고, 이 저장소는 비전 경로 추출과 궤적 실행 노드를 소유한다.

## 현재 상태 (As-Is)

저장소 `cad-less-robotics`에는 ROS 패키지, URDF, 드라이버가 없다. 구현 착수 전 Glob `**/*` 결과는 0건이었다. 지금은 본 플랜과 `.gitignore`만 있다. DOFBOT Pro URDF·MoveIt config·Orbbec 드라이버는 이 저장소 밖에 있으며, Jetson 실기는 이 세션에서 연결되어 있지 않다.

## 목표 상태 (To-Be)

Ubuntu 22.04 / ROS 2 Humble에서 `colcon build`와 `colcon test`가 되는 두 패키지가 있다. 카메라는 `sensor_msgs/PointCloud2`를 흘리고, `path_extractor_node`는 2mm `PoseArray`를 낸다. `motion_planner_node`는 MoveIt Cartesian 경로를 만들며, 안전 게이트를 통과하고 `execute_trajectory:=true`일 때만 `execute`한다. README는 미측정 항목을 측정값처럼 적지 않는다.

## 접근 방법

- A. 스펙 파일 3개에 PCL·MoveIt 호출을 모두 인라인. 장점: 파일 수가 스펙과 같다. 단점: 기하 로직을 로봇 없이 검증할 수 없다.
- B. 기하 라이브러리(`polyline`, `waypoint_prep`, `path_extractor`)와 ROS 노드를 분리하고 gtest로 기하만 검증. 장점: Humble 없는 Mac에서도 기하 식은 컴파일해 볼 수 있고, 노드 실패와 수식 실패가 갈린다. 단점: 스펙 트리보다 파일이 늘어난다.
- C. `pcl_ros` 필터 노드를 런치로 직렬 연결. 장점: 노드 코드가 짧다. 단점: Humble `pcl_ros` CMake에서 `sac_segmentation` 라이브러리는 주석 처리되어 있어 RANSAC 노드가 설치 패키지에 없다. 확인: perception_pcl `pcl_ros/CMakeLists.txt`의 segmentation 타깃은 주석이다. https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/CMakeLists.txt

선택: B. C는 스펙의 RANSAC 단계를 충족하지 못한다. A는 테스트 없는 포트폴리오 코드가 된다.

## 검증 근거

| 파일 또는 출처 | 위치 | 확인한 내용 |
| --- | --- | --- |
| 워크스페이스 Glob `**/*` | 구현 착수 전 검색 결과 0건 | 기존 ROS 패키지 없음 |
| agent-tools `e8c1435d-...txt` (Humble MoveGroupInterface API) | 635–639행 | `MoveGroupInterface(node, group, tf_buffer, wait_for_servers = from_seconds(-1))`. -1은 무한 대기 |
| 같은 파일 | 442행 | `execute(const RobotTrajectory &)` 반환 `moveit::core::MoveItErrorCode` |
| 같은 파일 | 1271행 | `setEndEffectorLink` 반환형 `bool` |
| 같은 파일 | 870행 | `getCurrentState(double wait = 1)` |
| agent-tools `e9b72d2e-...txt` (Humble Cartesian 튜토리얼) | 364행 | `disabling the jump threshold while operating real hardware can cause large unpredictable motions` |
| 같은 파일 | 368–370행 | `const double jump_threshold = 0.0;` 다음 줄 `computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);` |
| 같은 파일 | 388행 | `The speed of Cartesian plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time the trajectory manually` |
| 같은 파일 | 393행 | `/* move_group.execute(trajectory); */` 실행은 주석 |
| Yahboom MoveIt2 Configuration | 검색 결과 본문, 2026-10-26 | `arm_group`, `Arm1_Joint`–`Arm5_Joint`, parent link `Arm5_Link`. Arm6 이름 없음. https://www.yahboom.net/public/upload/upload-html/1768547833/MoveIt2%20Configuration.html |
| Orbbec ROS 2 point cloud | 검색 결과 본문, 2026-10-26 | `/camera/depth/points`는 `enable_point_cloud:=true`일 때만. https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/point_cloud.html |
| perception_pcl `pcl_ros/CMakeLists.txt` | 검색 결과의 segmentation 블록 | `sac_segmentation.cpp` 타깃이 주석. pcl_ros 노드 그래프만으로는 RANSAC 불가 |

패턴 충돌: 기존 소스 0건이라 충돌 없음. SP/PHP/JS 도메인 게이트는 이 저장소에 해당 파일이 없어 적용하지 않는다.

## 역할

- 이 저장소: 점군 전처리, 웨이포인트, Cartesian 계획·실행 노드, 참조용 컨트롤러 yaml, 런치.
- Yahboom 패키지: URDF, ros2_control 하드웨어, MoveIt config, `joint_states`.
- Orbbec 드라이버: 원본 `PointCloud2`와 카메라 TF.
- 사용자: 핸드아이 측정값, 실기에서 `execute_trajectory:=true` 결정.

## 디렉터리

스펙의 `cad_less_robotics_ws`는 하위 폴더로 중첩하지 않는다. 저장소 루트가 colcon 워크스페이스다.

```
src/vision_path_generator/
  include/vision_path_generator/polyline.hpp
  include/vision_path_generator/path_extractor.hpp
  src/polyline.cpp
  src/path_extractor.cpp
  src/path_extractor_node.cpp
  src/camera_node.cpp
  test/test_polyline.cpp
  CMakeLists.txt
  package.xml
src/robot_controller/
  include/robot_controller/waypoint_prep.hpp
  src/waypoint_prep.cpp
  src/motion_planner_node.cpp
  launch/system_bringup.launch.py
  config/dofbot_controllers.yaml
  test/test_waypoint_prep.cpp
  CMakeLists.txt
  package.xml
.cursorrules
README.md
```

기하 함수는 노드 밖으로 빼서 MoveIt·PCL 없이 단위 테스트한다. 노드 파일은 ROS 입출력, TF, 에러 처리만 담당한다.

## vision_path_generator

`camera_node`는 Orbbec SDK를 직접 링크하지 않는다.

- `source=relay` (기본): `/camera/depth/points`를 필드 검증 후 `/vision/camera/points`로 전달한다. QoS는 `SensorDataQoS`다.
- `source=pcd`: 하드웨어 없이 PCD를 주기적으로 발행한다.

`path_extractor_node` 처리 순서:

1. 비유한 점 제거. `input_scale`로 미터로 맞춘 뒤, 프레임이 다르면 TF로 `target_frame`(기본 `base_link`)에 변환한다.
2. `pcl::VoxelGrid` (기본 leaf 2mm).
3. `pcl::StatisticalOutlierRemoval`.
4. `pcl::SACSegmentation` RANSAC 평면 제거. 평면 법선은 세계 +Z가 아니라, 평면을 뺀 뒤 남은 제품 점이 있는 쪽으로 맞춘다.
5. 평면에서 `min_height`(기본 3mm) 아래 점을 제거하고 가장 큰 유클리드 클러스터만 남긴다.
6. K-NN PCA 법선. 툴 +Z는 소재 쪽, 툴 +X는 경로 접선.
7. `path_mode=auto`: 경계 컨투어가 충분하면 그 경로, 부족하면 주축 방향 단면의 능선으로 폴백. 2mm 간격으로 리샘플해 `/vision/waypoints` (`geometry_msgs/PoseArray`)를 발행한다.

프레임이 다른데 TF가 없으면 포즈를 발행하지 않는다. 이전 클라우드 처리가 끝나기 전에 들어온 프레임은 버린다.

## robot_controller

- `MoveGroupInterface` 그룹 기본값 `arm_group`. 서버 대기는 10초에서 끊는다.
- 수신 `PoseArray`를 계획 프레임으로 TF 변환한다. 실패하면 실행하지 않는다.
- `getCurrentState`가 비면 실행하지 않는다. 시작 시 홈 포즈로 자동 이동하지 않는다.
- 접근 2cm, 후퇴 2cm를 툴 +Z의 반대 방향에 붙인다.
- `computeCartesianPath` (`eef_step` 2mm, `jump_threshold` 1.0, 충돌 회피). 분율이 `min_fraction`(0.95) 미만이면 `execute`하지 않는다. 튜토리얼의 0.0은 실기 경고가 있어 기본값으로 쓰지 않는다.
- 타임스탬프가 단조 증가하지 않으면 관절 속도 상한으로 다시 시간을 매긴 뒤에만 실행한다. 근거는 위 튜토리얼 388행이다.
- `execute_trajectory` 기본값은 false다. 실행 경로는 구현하고, 런치 인자로만 켠다.
- 디버그 궤적은 `~/planned_trajectory`에만 발행한다. 컨트롤러 명령 토픽에는 직접 쓰지 않는다.

`config/dofbot_controllers.yaml`은 Yahboom 문서의 `arm_group_controller`와 `Arm1_Joint`–`Arm5_Joint`를 옮긴 참조 설정이다. 벤더 드라이버가 이미 컨트롤러를 띄우면 중복 로드하지 않는다. 그리퍼 조인트 이름은 확인되지 않아 넣지 않는다.

## 안전 게이트

- 분율 미달, TF 실패, 빈 경로, 도달 거리 초과(기본 1m), 오래된 포즈(기본 3초), 현재 관절 상태 없음이면 로봇을 움직이지 않는다.
- 단위가 미터가 아닌 클라우드가 미터로 잘못 해석되면 도달 거리 검사에서 거절된다.
- 단위 행렬 핸드아이 TF는 기본으로 발행하지 않는다. 런치 인자 `publish_hand_eye_tf:=true`와 측정값을 줄 때만 정적 TF를 띄운다.

## 테스트

Humble에서 `colcon build`와 `colcon test`로 기하 단위 테스트만 검증한다. 이 Mac에는 ROS Humble이 없으므로 여기서 colcon 빌드는 하지 않는다.

- 2mm 리샘플이 끝점을 포함하는지
- 끊긴 컨투어에서 가장 긴 사슬만 남기는지
- 툴 축이 법선·접선과 오른손 좌표계를 이루는지
- 접근 포즈가 툴 +Z의 반대 방향인지
- 쿼터니언 반구 정렬

실기 구동과 핸드아이 정확도는 측정값이 생기기 전에는 통과로 적지 않는다.

## 승인 시 잠글 기본값

답이 없으면 아래 값으로 구현한다.

- 플래닝 그룹: `arm_group` (스펙 초안의 `arm` 대신 Yahboom 문서 값)
- 팔 조인트: `Arm1_Joint`–`Arm5_Joint`. `Arm6_Joint`는 만들지 않음
- 엔드이펙터 링크: `Arm5_Link`. 설정 실패 시 그룹 기본 팁
- 클라우드 입력: `/camera/depth/points`
- 포즈 프레임: `base_link`
- 경로 모드: `auto` (경계 우선, 실패 시 능선)
- 실기 실행: `execute_trajectory:=false`

## 하지 않는 것

- Orbbec SDK, DOFBOT URDF, MoveIt config 패키지 재작성
- 확인되지 않은 그리퍼 조인트 이름 생성
- 측정하지 않은 성공률·속도 수치를 README에 기재

## 구현 단계

1. `package.xml`, `CMakeLists.txt`, `.cursorrules`로 두 ament 패키지 골격을 만든다.
2. `polyline`과 `waypoint_prep`을 만들고 gtest를 붙인다. ROS 그래프 없이 리샘플·자세·접근 포즈를 검사한다.
3. `path_extractor`에 Voxel → SOR → RANSAC → 클러스터 → 법선 → 2mm 웨이포인트를 넣는다.
4. `camera_node`, `path_extractor_node`, `motion_planner_node`에 TF 실패·빈 입력·분율 미달·관절 상태 없음 처리를 넣는다.
5. `system_bringup.launch.py`, `dofbot_controllers.yaml`, README를 스펙 기본값과 미확인 항목이 보이게 작성한다.

## 리스크와 실패 가정

이 구현이 실패한다면 이유는 다음 세 가지다.

1. 핸드아이 TF 없이 카메라 좌표를 베이스 좌표로 실행한다. 예방: 프레임이 다르고 TF가 없으면 포즈를 버리며, 단위 행렬 정적 TF는 기본 발행하지 않는다.
2. 그룹 이름이 실기 URDF와 다르다. 스펙 초안은 `arm`, Yahboom 문서는 `arm_group`이다. 예방: 파라미터로 두고 기본값은 문서 값. 서버 대기는 10초에서 실패로 끝낸다.
3. 경계 컨투어가 재봉선이 아니라 실루엣이라 경로가 의도와 다르다. 예방: `path_mode`로 `boundary` / `crest` / `auto`를 고르게 하고, README에 기하 정의만 적는다. 실기 적합은 측정 전 PASS로 쓰지 않는다.

추가 예방: `jump_threshold` 기본 1.0, `min_fraction` 0.95, `execute_trajectory` 기본 false. 점군 필드에 x/y/z가 없거나 비유한 값이면 그 프레임은 버린다. 클라우드 문자열로 셸을 호출하지 않는다.

## 보안

인증 엔드포인트는 없다. 입력은 ROS 메시지 필드 검증(유한 좌표, xyz 필드, 쿼터니언 길이, 도달 거리)만 한다. 외부 명령을 점군이나 PCD 경로 문자열로 조합하지 않는다. PCD 경로는 `pcl::io::loadPCDFile` 인자로만 쓴다.

## 완료 기준

- 위 파일 목록이 저장소에 있다.
- 플래닝 그룹 기본값이 `arm_group`이고 yaml 조인트가 `Arm1_Joint`–`Arm5_Joint`뿐이다.
- 안전 게이트(TF, 분율, 관절 상태, execute 기본 false)가 노드 코드에 있다.
- gtest가 리샘플 끝점, 긴 사슬, 오른손 자세, 접근 방향, 쿼터니언 반구를 검사한다.
- README가 핸드아이·그리퍼 조인트·일체 카메라 토픽을 미확인으로 적는다.
- 이 세션에서는 Humble colcon을 돌리지 않는다. Mac에 Humble이 없기 때문이다.

## 영향 받는 파일

신규 생성만 한다. 기존 비즈니스 코드 수정은 없다.

- `docs/261026-1255-CAD-less-ROS2-PoC-플랜.md` (본 문서)
- `.gitignore`, `.cursorrules`, `README.md`
- `src/vision_path_generator/**` (package, cmake, include 2, src 4, test 1)
- `src/robot_controller/**` (package, cmake, include 1, src 2, launch 1, config 1, test 1)
