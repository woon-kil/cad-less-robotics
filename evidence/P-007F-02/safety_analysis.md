# P-007F-02 — Safety Analysis for the Next Execution Gate

## Abort conditions (사전등록, 최소 목록)

다음 중 **하나라도** 발생하면 즉시:

```text
ABORT
NO RETRY
NO POSITION COMMAND
```

1. unexpected joint movement (사전등록한 witness 측정 외의 움직임)
2. gravity drop (Arm2 하류 구조의 눈에 보이는 처짐/낙하 — `torque_experiment_design.md`
   의 ~211g 모멘트 암 분석 대상)
3. cable tension (DaBai/전원 케이블에 비정상 장력)
4. camera mount instability (DaBai 마운트 흔들림/풀림 조짐)
5. robot base movement (base_link 자체가 흔들리거나 밀림)
6. uncontrolled oscillation (서보 진동/헌팅)
7. abnormal sound (평소와 다른 기계음 — 기어 그라인딩 등)
8. unexpected controller state (readback 이 예상 밖 값/None 연속)
9. witness failure (Candidate A 측정 기구 오류, DaBai POST window quality guard
   실패 등 — P-007E-R2-R5 Attempt #4 에서 실제로 발생한 바로 그 실패모드)
10. operator cannot maintain safe observation (운영자가 로봇을 계속 육안
    관찰할 수 없는 상태가 됨)
11. physical power cutoff inaccessible (비상 전원차단 접근 불가)

## ABORT 발생 시 처리

- 진행 중인 측정 즉시 중단.
- 추가 position command 송신 금지(이미 R3/R4/R5 에서 고정된 원칙 — 결과와
  무관하게 두 번째 명령 없음, 이번에도 동일하게 적용).
- `Arm_serial_set_torque(1)`(restoration)은 **ABORT 사유가 "torque OFF 상태에서
  발생"한 경우에만, 그리고 그것이 상황을 악화시키지 않는다고 운영자가 즉시
  판단할 수 있는 경우에만** 시도. 판단이 서지 않으면 physical power cutoff 를
  우선한다 — "소프트웨어로 되돌리려 하지 않는다"가 기본값이다.
- ABORT 사유·시각·당시 witness 측정치를 evidence 로 남긴다(다음 실행 gate
  설계 시 재사용).

## torque OFF 관련 특이 위험 (torque_experiment_design.md 요약 참조)

Arm2 는 하류 ~211g(Arm3+Arm4+DaBai+Arm5+그리퍼 조립체)를 지지하는 weight-bearing
pitch joint다. 현재 physical pose 가 UNKNOWN 이므로 모멘트 암의 크기를 사전에
알 수 없다 — **최악의 경우를 가정**해 abort 조건 2(gravity drop)·3(cable
tension)·4(camera mount instability)를 특히 민감하게 감시해야 한다.

## power cutoff vs torque command (요약)

- physical power cutoff: 즉시·확실한 무동력. 복구에 전체 재초기화 필요.
  "torque 상태 판별"에는 쓸 수 없다(이미 OFF 를 확정시킴).
- torque command: 소프트웨어 명령, 접수 여부 확인 API 없음(P-007F-01).
  Abort 상황에서 "torque(1)을 보내면 안전해질 것"이라고 가정하지 않는다 —
  위 "ABORT 발생 시 처리" 참조.

## 이번 gate(P-007F-02) 자체의 안전 상태

이 문서를 포함한 P-007F-02 전체 작업에서 하드웨어 상태 변경 0건, position
command 0건, torque command 0건, 카메라/USB/I2C 재설정 0건, 수동 이동 0건,
process kill/restart 0건. 위 abort 조건은 **다음 실행 gate(P-007F-03 또는
그 이후)를 위한 사전등록**이며 이 gate 자체에서 트리거된 적 없다.
