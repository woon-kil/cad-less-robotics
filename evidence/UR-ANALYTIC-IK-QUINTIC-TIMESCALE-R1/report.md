# 구간별 5차 시간 스케일링

```text
QUINTIC_TIMESCALE_CHECK = PASS
TOTG_CALLS = 0
INTERPOLATION = PER_JOINT_QUINTIC_HERMITE
CONTROLLER_INTERPOLATION = NOT_CLAIMED
SPRAY_EFFICIENCY = NOT_CLAIMED
QUINTIC_INTERIOR_LIMITS = PASS
PATH = PASS
PROCESS_ACCURACY = PASS
COLLISION = PASS
POINT_COUNT = 944
TOTAL_DURATION_NS = 195015588556
GENERATE_S = 0.0024674379999999885
CARTESIAN_PATH_IMPOSSIBILITY = NOT_CLAIMED
```

이 검사는 점마다 정지하는 planning-reference UR5e 궤적을 5차 Hermite로 읽은 결과다. 실제 컨트롤러가 이 보간을 쓴다고 말하지 않는다. 연속 분사 효율과 실물 안전은 범위가 아니다.

시간 하한은 구간별 나노초 올림으로 저장했고, 운동 한계는 저장 메시지를 다시 읽은 Δt로 계산했다. 속도 최대 절대값 0.92949427628339187 rad/s, 가속도 최대 절대값 4.999999999826807 rad/s^2.

경로 이탈 0, 위치 차 최대 0 rad, 정지 경계 PASS.

공정 표본 92301, 실패 0, 위치 최대 0.00092728561297979336 m, 방향 최대 0.0076259116189791006 rad.

충돌 표본 94301, 자체 0, coupon 0, error 0.

P-022, 기하 경로, 분사 데모, TOTG 실패 기록은 이 실행에서 수정하지 않았다.
