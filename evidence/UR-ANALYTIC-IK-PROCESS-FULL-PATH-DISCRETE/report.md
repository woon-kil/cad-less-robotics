# 전체 경로 이산 검사 manifest

```text
GATE = UR-ANALYTIC-IK-PROCESS-FULL-PATH-DISCRETE
ROLE = SEPARATE_FULL_PATH_RECORD
CLAIM = 전체 기하 후보 경로 확보 및 고정 표본 이산 검사 통과
CROSS_IMPLEMENTATION_CHECK = false
CONTACT_FAILURE_BRANCH = NOT_EXECUTED
P-022 = UNCHANGED
HISTORICAL_MANIFEST = UNCHANGED
```

실행된 소스는 `/tmp/restore-check/src/ur_analytic_ik_process_connect_collision.cpp`이다. SHA-256 `a260f70938057a25643419ca6aefbb01e0ee515dfa3d2be214923f4eaa08cde1`은 보관한 `restore-src.cpp`와 같다. 빌드 로그는 그 파일을 컴파일해 `/tmp/restore-check/build/ur_analytic_ik_process_connect_collision`을 만들었다. 바이너리 SHA-256은 `2587e76f8694a79c1bed8497604f742f100564d735c233b2bd41c1e86e7fd818`이다. 시각은 소스 10:25:23, 바이너리 10:25:34, 결과 10:26:15다. 바이너리 안에 소스 해시를 넣어 둔 기록은 없다.

공유 함수 검토에서 채택을 막을 항목은 없었다.

- 좌표는 `base_link` 기준 `tool0`이다. `tool0`은 planning reference TCP다.
- PROCESS 목표는 저장 위치의 선형 보간과 저장 방향의 slerp다. TRANSITION에는 공정 정확성을 적용하지 않는다.
- 관절 한계는 URDF 위치 범위이고, `1e-12` rad를 넘는 바깥 값과 비유한값은 실패다.
- 충돌 결과는 호출마다 새로 만든다. 자체 충돌은 `checkSelfCollision`, 그다음 coupon은 `checkRobotCollision`이다. ACM은 시작 때 모델에서 복사하고 이후에 고치지 않는다.
- `runs.bin` 721개 방향의 노름은 1에서 최대 `2.220446049250313e-16`만큼 벗어난다.

실패 접촉을 저장하는 분기는 실행되지 않았다. 통과 판정의 결함으로 두지 않으며, 그 분기를 시험했다고 적지 않는다.

이 manifest는 별도 성과다. P-022 `CLOSED_FAIL`과 `evidence/UR-ANALYTIC-IK-PROCESS-COLLISION-ADOPTED/manifest.json`은 역사 기록으로 둔다.
