# P-007F-02 — Torque / Actuator State Discrimination Preregistration

🔴 **DESIGN / PREREGISTRATION ONLY.** 이 문서를 포함한 P-007F-02 전체 작업에서
로봇 상태는 전혀 바뀌지 않았다. servo write/torque write/PWM write/reset/
calibration write/offset write/action playback/servo ID 변경/position
command/수동 이동/R5 재실행/카메라-USB-I2C 재설정/관련 process kill-restart —
전부 **0회**.

이 문서는 `baseline.md` · `witness_candidates.md` · `camera_witness_analysis.md`
· `torque_experiment_design.md` · `result_matrix.md` · `safety_analysis.md` 를
요약·연결하는 master 문서다. 각 세부 근거는 해당 파일을 참조한다.

## 1. 출발점

`baseline.md` — P-007F-01 결과(READBACK_SEMANTICS=UNRESOLVED, TORQUE_STATE=
UNKNOWN, SERVO_COMMUNICATION=SUPPORTED, PHYSICAL JOINT POSITION=UNKNOWN) 와
Attempt #4 의 두 provenance(machine record=YES, operator correction=NO
MOTION)를 그대로, 서로 덮어쓰지 않고 보존. `PHYSICAL MOTION = UNRESOLVED`로
취급. `154°`는 controller-reported state 로만 표현.

## 2. 절대 금지(실행됨 없음, 사전등록으로 재확인)

```text
Arm_serial_servo_write* / Arm_serial_set_torque / PWM servo write / reset /
calibration write / offset write / action playback / servo ID 변경 /
position command / robot 수동 이동 / R5 재실행 /
카메라·USB·I2C 장치 재설정 / 관련 process kill·restart
= ZERO HARDWARE STATE CHANGE (이번 gate 전체)
```

## 3. 기존 evidence 보존

`evidence/P-007E-R2-R5/**`, `evidence/P-007F-01/**`, `.cache/p007e_r2r5_ledger/**`
— 전부 읽기만 했고 수정/삭제하지 않았다(`baseline.md` 가 그 읽은 내용을
인용). Attempt #4 의 machine record 와 operator correction 을 병렬 보존했다
(§1).

## 4. 실험 질문 (Q1~Q4)

`torque_experiment_design.md` 참조. 요약:

- Q1: torque 를 건드리지 않고 bounded force 로 현재 holding 상태를 probe —
  **가장 낮은 위험**, position command 0, torque command 0.
- Q2: torque ON/OFF 비교 — **실제 torque write 1회가 필요**, gravity-drop
  위험(§7) 분석 대상.
- Q3: MCU 접수 ≠ 실제 actuator 반응 — 물리 witness 로만 구별 가능(공식 read
  API 없음, P-007F-01).
- Q4: position command 없이 최소 물리적 증거 확보 가능 — Q1 설계로 충족.

## 5. Witness 설계

`witness_candidates.md` 참조. **권장**: Candidate A(bounded external force +
독립 inclinometer/protractor 측정, 카메라 비의존)를 primary, DaBai ego-motion
을 secondary/corroborating 으로 동시 기록. Gemini 는 미사용.

## 6. Camera 선택

`camera_witness_analysis.md` 참조.

- **DaBai DCW2**: SUITABLE AS SECONDARY/CORROBORATING SIGNAL ONLY — eye-in-hand
  구조상 독립 witness 자격 없음(명시적으로 그렇게 취급하지 않기로 결정).
- **Gemini 335L**: `NOT CURRENTLY USABLE` — 3건의 독립 기존 evidence(sensor
  inventory, 실측 프레임, 설치사 근거)가 모두 "팔이 화각 밖"을 확인. 이
  gate 에서 카메라 위치를 바꾸지 않았다.

## 7. Torque transition 실험 후보

`torque_experiment_design.md` §"Torque transition sequence 후보" 참조.
`torque_experiment_design.md` §"위험 분석"이 gravity/drop·cable tension·
camera mount instability·restoration 신뢰성·power cutoff vs torque command
차이를 분석. **결론: Q2(실제 torque OFF)는 이번 시점에서 REJECT** — 현재
physical pose UNKNOWN + restoration 성공을 확인할 API 없음 + spotting 절차
미설계. Q1(torque 변경 없는 probe)은 REJECT 대상 아님.

## 8. Position command 금지 재확인

이 설계 전체에서 `154→156`, `154→152` 등 어떤 position command 도 사용하지
않는다(torque_experiment_design.md §8). Q1/Q2 어느 단계에도
`Arm_serial_servo_write*` 가 등장하지 않는다.

## 9. Result matrix

`result_matrix.md` 참조. A/B/C/D/X 전부 사전 고정, 사후 threshold 변경 금지.

## 10. Safety abort conditions

`safety_analysis.md` 참조. 11개 조건 + ABORT 처리 절차(추가 position command
금지, torque(1) 복구는 조건부로만, 아니면 physical power cutoff 우선) 사전등록.

## 11. Evidence

```text
evidence/P-007F-02/
  baseline.md
  witness_candidates.md
  camera_witness_analysis.md
  torque_experiment_design.md
  result_matrix.md
  safety_analysis.md
  preregistration.md (이 문서)
```

runtime result 파일은 아직 생성하지 않았다(지시대로).

## 12. Static safety validation

🔴 이번 gate에서 **executable helper 파일을 만들지 않았다** — 지시(§12 "가능하면
executable hardware code 자체를 만들지 말고 analysis/preregistration 문서
중심으로 처리")에 따라 분석/설계 문서만 작성했다. 따라서 AST 기반 forbidden-API
정적검사는 **N/A**(대상 파일이 없음) — "0 calls" 요구사항은 "그런 코드를 아예
작성하지 않았다"로 자명하게 만족한다. 다음 실행 gate(P-007F-03)에서 실제
executable helper 가 필요해지면, 그 파일에 대해 P-007F-01 과 동일한 패턴
(allowlist AST 검사 + true-positive 증명 negative test, 참조-후-호출 패턴
포함)을 적용할 것을 사전등록한다.

## 13. 최종 recommendation

```text
TORQUE_DISCRIMINATION_REQUIRES_DESIGN_CHANGE
```

**근거**:

1. 현재 장비로는 "독립 visual witness"가 존재하지 않는다(DaBai=eye-in-hand라
   독립 자격 없음, Gemini=화각 밖). → camera 의존을 primary witness 로 쓸
   수 없다. **설계 변경**: Candidate A(비카메라, 외력+독립 inclinometer)를
   primary witness 로 채택.
2. Q2(실제 torque ON/OFF 비교)는 현재 physical pose 불명 + restoration
   신뢰성 확인 불가 + spotting 절차 미설계로 **그대로는 안전하지 않다**(§7).
   → **설계 변경**: spotting/지지 절차를 별도로 설계하기 전에는 Q2 를
   실행하지 않고, 위험이 훨씬 낮은 Q1(torque 변경 없는 probe)만 다음 실행
   gate 의 1차 범위로 좁힌다.
3. 위 두 가지(비카메라 primary witness로 교체, Q2를 spotting 절차 설계 전까지
   유보)를 반영하면 Q1 범위의 실행은 안전하게 설계 가능하다고 판단한다 —
   이것이 `TORQUE_DISCRIMINATION_UNSAFE_WITH_CURRENT_SETUP`(완전 불가) 이
   아니라 `REQUIRES_DESIGN_CHANGE`(설계를 바꾸면 가능)를 선택한 이유다.
   `READY_FOR_TORQUE_DISCRIMINATION_EXPERIMENT`를 선택하지 않은 이유는 위
   두 설계 변경이 아직 **P-007F-02 문서 수준에서만** 존재하고 구체적
   실행절차(힘의 상한 수치, blind 측정 절차, spotting 절차)로 완성되지
   않았기 때문이다 — 그 완성은 P-007F-03 의 몫이다.
