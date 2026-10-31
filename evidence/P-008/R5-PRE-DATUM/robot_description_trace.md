# robot description 추적

이 맥에는 URDF가 하나 있다. 다른 DOFBOT URDF를 고르지 않았다.

```text
ACTIVE_ROBOT_DESCRIPTION =
.cache/jetson_robot_model/DOFBOT_Pro-V24.urdf

ACTIVE_MOVEIT_CONFIG =
.cache/jetson_robot_model/DOFBOT_Pro-V24.srdf

MODEL_SOURCE =
https://github.com/atticusrussell/ballbot
branch humble
commit 906da9f9120c056dd0c8766fe1a72e72a086061c

MODEL_VERSION_OR_COMMIT =
906da9f9120c056dd0c8766fe1a72e72a086061c
```

파일 sha256은 `b626a845cd67998cd5ce47f2b3d302d9cb3e5b9da764d9a1e7e1404a277ccc28`이다. 바이트 수는 19234다. SRDF sha256은 `5c355b844eca2ba277e5df204b59b6d8886768b48e11701d17a23491bdffc1ee`이다.

## 계획에 쓰는 사슬

`scripts/validate_dofbot_moveit.sh`가 위 ballbot 커밋에서 description URDF와 MoveIt SRDF를 받는다. 받는 경로는 `third_party/dofbot-pro-cache`다. 이 맥에는 그 캐시가 없다.

계획 노드는 `src/dofbot_moveit_dry/src/dry_plan_node.cpp`에서 `urdf_path` 파일 전체를 `robot_description`으로 넣고, SRDF 파일을 `robot_description_semantic`으로 넣는다. xacro를 펼치지 않는다.

```text
dofbot_moveit_dry
  → robot_description = DOFBOT_Pro-V24.urdf 원문
  → robot_description_semantic = DOFBOT_Pro-V24.srdf
  → base_link
  → Arm1_Joint
```

`DOFBOT_Pro-V24.urdf.xacro`는 그 스크립트가 실행 경로가 아니라고 적는다. `demo.launch.py`도 띄우지 않는다. SRDF에는 `virtual_joint`가 없다.

Jetson에서 받아 둔 이 캐시의 URDF sha256은 P-006이 같은 이름으로 읽은 값과 같다. 하드웨어 폴더 URDF는 이 맥에 없고, 계획 모델로 쓰지 않았다.

공식 `YahboomTechnology/DOFBOT-Pro` `57cc436`에는 URDF가 없다는 기록은 P-002에 있다. 이번 조사에서 그 저장소를 다시 받지 않았다.
