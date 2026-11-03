# TOTG 단일 실행

```text
TOTG_MESSAGE_CHECK = FAIL
CALLS = 1
INTERPOLATION = PER_JOINT_QUINTIC_HERMITE
CONTROLLER_INTERPOLATION = NOT_CLAIMED
QUINTIC_INTERIOR_LIMITS = FAIL
CORRESPONDENCE = FAIL
PROCESS_ACCURACY = FAIL
COLLISION = FAIL
POINT_COUNT = 689
GENERATE_S = 1.3515004429999999
CARTESIAN_PATH_IMPOSSIBILITY = NOT_CLAIMED
```

이 검사는 planning-reference UR5e의 저장 메시지를 5차 Hermite로 해석한 결과다. 실제 컨트롤러가 이 보간을 쓴다고 말하지 않는다. 대응 실패는 Cartesian 경로 자체가 불가능하다는 뜻이 아니다.

QUINTIC_INTERIOR_LIMITS와 공정 정확성·충돌은 별도 항목이다. 속도 최대 절대값 2.0280006557273045 rad/s, 가속도 최대 절대값 8.5440780728983761 rad/s^2.

대응: absent 0, ambiguous 1, reversal 0, boundary_skip 0, missing_chord 0.

공정 표본 67337, 실패 55, 위치 최대 0.0014662786040477603 m, 방향 최대 0.011423133684061663 rad.

충돌 표본 68801, 자체 0, coupon 0, error 0.

P-022, 기하 경로 manifest, 분사 데모는 이 실행에서 수정하지 않았다.

## 저장 결과 읽기

이 절은 저장된 `trajectory.json`과 `restore.json`을 다시 읽은 것이다. `computeTimeStamps`를 다시 호출하지 않았다. 위 블록의 수치와 `correspondence.json`은 실행 산출 그대로다.

`failures[0].value` = `0.00017300996959005485`는 차점과의 차이가 아니다. 코드는 `AMBIGUOUS_CHORD` 기록에 `assignment.residual`을 넣는다. 그 값은 최근접 현까지 6관절 유클리드 잔차 `r*`다. 모호함 조건은 `r2 - r* < 1e-9`다.

구간 311, 표본 72를 같은 5차 Hermite로 재구성하면 다음이다.

```text
r* = 0.00017300996959005485 rad
chord* = 433, u* = 1
r2 = 0.00017300996959005485 rad
chord2 = 434, u2 = 0
r2 - r* = 0
공유 꼭짓점 434의 관절별 최대 차이 = 0.00016766913274923745 rad
```

두 현의 최근접점은 같은 공유 꼭짓점이라 잔차가 같고, 간격 0은 `1e-9` 조건에 들어간다. 표본이 꼭짓점에서 `1e-9` rad 밖이라 `VERTEX`가 아니고 `AMBIGUOUS_CHORD`다. 판정은 고정 규칙과 일치한다. `1.73e-4`를 간격으로 읽은 것이 보고 불일치다.

`checks.json`의 `collision.status`는 `correspondence_pass`와의 논리곱이다. 충돌 검출은 자체 0, coupon 0, error 0이다.

`process_accuracy.status`도 같은 논리곱을 거친다. 그와 별도로 `fail_count` 55, 위치 최대 `0.0014662786040477603` m, 방향 최대 `0.011423133684061663` rad가 공정 정확성 실패다.

`quintic_interior_limits`의 가속도 최대 절대값 `8.5440780728983761` rad/s²는 한계 `5.0` rad/s² 초과다. 이 세 실패는 서로 다른 원인이다. 기하 경로 확보 결과를 바꾸지 않는다.
