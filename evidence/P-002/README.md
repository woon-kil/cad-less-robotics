# P-002 DOFBOT Model + MoveIt Dry Planning

SOURCE_BASE_COMMIT=ce3ace9e923bfa365630feffc397db9dfb4468f3
TESTED_COMMIT=ee8fd01e1246a8aa96980d614477ea2fbc2b9320
EVIDENCE_GENERATED_AT=2026-10-26 14:32 KST
FINAL_COMMIT=PENDING

Container: bytesize-robotics-humble
Ubuntu: 22.04
ROS_DISTRO: humble
MoveIt: 2.5.10
Architecture: x86_64

재현:

```bash
docker exec bytesize-robotics-humble bash -lc 'bash /ws/cad-less-robotics/scripts/validate_dofbot_moveit.sh'
```

기대 출력:

```text
P-002 CASE-01 PASS
P-002 CASE-02 PASS
P-002 CASE-03 PASS
P-002 RESULT PASS
```

RViz는 사용하지 않았다. `PlanningComponent::execute()`는 호출하지 않았다. controller 이름은 비어 있고 MoveIt은 trajectory execution을 관리하지 않았다.

## Model

공식 `YahboomTechnology/DOFBOT-Pro` `57cc436`에는 URDF가 없다.
`~/robot_sources`에는 DOFBOT이 없고 mycobot만 있다. 그 트리는 수정하지 않았다.

계획에 쓴 파일은 ballbot `humble` `906da9f`에 벤더된 Yahboom 강좌 스냅샷이다.
URDF는 `dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf`.
SRDF는 `dofbot_pro_moveit/config/DOFBOT_Pro-V24.srdf`.
`dofbot_pro_moveit` maintainer는 `yahboom@163.com`, license 태그는 BSD다.
`dofbot_pro_description` license 태그는 TODO다. 그래서 모델 파일은 저장소에 커밋하지 않고 검증 스크립트가 받는다.

ballbot의 mesh는 Git LFS 포인터이고 LFS 서버는 404였다.
각 mesh는 sha256이 그 LFS oid와 같을 때만 `chrisantuseze/dofbot-controller` `142457e`의 `dofbot_pro_v24/meshes`에서 받았다.
URDF와 SRDF는 그 미러를 쓰지 않았다.

`arm_group`은 Arm1_Joint–Arm5_Joint다. `Arm6_Joint`는 없다.
planning end link는 `Arm5_Link`다. `Gripping_point_Link`는 calibrated TCP로 쓰지 않았다.

## Cases

SRDF named state만 썼다. grip_group은 `open` (`grip_joint=-0.0796`)으로 고정했다.

| Case | Start | Goal | Time (s) | Points | Collision | Limit violations |
|---|---|---|---|---|---|---|
| CASE-01 | init | up | 0.007424096 | 27 | clear | 0 |
| CASE-02 | up | down | 0.005955732 | 27 | clear | 0 |
| CASE-03 | down | init | 0.009531758 | 39 | clear | 0 |

init = 0, 1.2063, -1.5, -1.3104, 0
up = 0, 0, 0, 0, 0
down = 0, -1.57, 0, 0, 0

Pipeline: ompl / RRTConnect. Collision links 14. SRDF가 끄지 않은 self-collision pair 13. `FixStartStateCollision`은 어댑터에 없다.

## Warnings

벤더 `joint_limits.yaml`은 acceleration limit를 끄고 있다. MoveIt TOTG가 정의되지 않은 가속도에 기본값 1 rad/s^2를 썼다고 로그에 남겼다. joint_limits 파일은 바꾸지 않았다. 속도 검사는 URDF velocity 1 rad/s 기준이고 위반은 0이다.

KDL은 root link inertia를 지원하지 않는다고 경고했다. dummy link는 추가하지 않았다.

하드웨어 폴더 URDF와 description URDF 바이트는 다르다. arm joint 정의는 같고, Gripping_point origin 텍스트만 `0 0.09 0`과 `0 0.09 0.00` 차이다.

RRTConnect 샘플링 때문에 포인트 수는 다시 실행하면 달라질 수 있다. 목표 관절값은 SRDF named state라 같다.
