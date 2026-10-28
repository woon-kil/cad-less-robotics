# P-007F-02 — Torque Transition Experiment Design

## 실험 질문

### Q1 — 현재 actuator torque가 실제로 enabled 상태인가?

**설계**: torque 를 **건드리지 않은 채로**, Arm2 하류 구조에 사전 고정된 작고
일정한 외력(Candidate A, `witness_candidates.md`)을 가하고 변위를 측정한다.
변위가 거의 0이면 "현재 상태에서 저항이 있다"(torque 가 유효하게 걸려 있을
가능성), 유의미한 변위가 있으면 "현재 상태에서 저항이 약하다"는 신호다. 이
probe는 **`Arm_serial_set_torque` 를 호출하지 않는다** — 현재 상태를 그대로 둔
채 외부에서만 관찰한다. 가장 낮은 위험의 단계다.

### Q2 — torque command를 변경했을 때 physical actuator holding state가 객관적으로 달라지는가?

**설계**: Q1과 동일한 bounded force 프로토콜을 torque=1 상태와 torque=0 상태
양쪽에서 각각 수행해 변위를 비교한다. 이 질문에 답하려면 **실제로
`Arm_serial_set_torque(0)` 를 한 번 보내야 한다** — §7의 gravity-drop 위험
분석 대상이다. **이 gate는 이 호출을 실행하지 않는다** — 설계와 위험분석만
한다.

### Q3 — MCU/controller가 torque command를 받아들이는 것과 실제 servo actuator torque 변화 사이를 어떻게 구별할 것인가?

**설계**: `Arm_serial_set_torque()` 는 write-only이고(P-007F-01 Task A/E),
그 명령이 MCU 에 "받아들여졌는지"를 읽는 공식 API가 없다. 구별 수단은 **물리
witness 뿐**이다 — Q1/Q2 의 외부 displacement 측정이 "MCU가 명령을 접수했다"는
사실과 "실제 servo 가 그에 따라 거동을 바꿨다"는 사실을 분리해 확인하는 유일한
경로다. I2C 트랜잭션 성공(예외 없음) 은 MCU 접수의 약한 신호일 뿐 servo
동작의 증거가 아니다 — P-007F-01 의 readback semantics 미해결과 동일한 구조적
한계다.

### Q4 — position command 없이 actuator path의 최소 물리적 증거를 확보할 수 있는가?

**답**: 가능하다. Q1 설계(외력+변위 측정, torque 변경 없음)가 그 자체로
`position command 0` 조건을 만족하는 최소 증거 확보 경로다. §8과 일치.

---

## Torque transition sequence 후보

```text
Baseline(현재 상태 그대로, torque 미확인)
  ↓
Q1 probe: bounded external force, torque 변경 없음 → 변위 측정(독립 inclinometer)
  ↓
[여기서 STOP 가능 — Q1 만으로도 "현재 holding 이 강한지 약한지"에 대한 1차 신호 확보]
  ↓ (Q2 로 진행할 경우에만)
Torque state intervention: Arm_serial_set_torque(0) 1회
  ↓
Objective witness measurement: 동일 bounded force 프로토콜 재실행
  ↓
Restoration: Arm_serial_set_torque(1) 1회
  ↓
Optional post-restoration witness: 변위 측정 재확인(복구가 holding 을 되돌렸는지)
```

## 위험 분석(§7 필수 항목)

### torque OFF가 gravity/drop을 발생시킬 위험

**있음, 무시할 수준 아님.** URDF(`DOFBOT_Pro-V24.urdf`, 이미 R4/R5에서 SHA
검증됨) 기준:

- `Arm2_Joint` 축 = `(1,0,0)`, parent `Arm1_Link` 원점 `(0,0,0.1225)` — **pitch
  joint**다(팔을 들어올리는 방향). Arm1(yaw, base 회전)과 다르다.
- Arm2 **하류**(Arm3+Arm4+DaBai+Arm5+그리퍼 좌우 링크+Gripping_point, URDF
  `<mass>` 전부 합산) 총 질량 ≈ **0.211 kg**.
- 이 전체 질량이 Arm2 축을 중심으로 회전하는 모멘트 암이 된다 — 현재 pose(각
  링크의 실제 물리 각도)가 **UNKNOWN**(P-007F-01)이므로, 모멘트 암이 작은
  자세(거의 수직으로 접힌 상태)인지 큰 자세(수평으로 뻗은 상태)인지 **사전에
  알 수 없다**. 가장 나쁜 경우를 가정해야 한다.

### Arm2가 하중을 받는 구조인지

**예.** 위 분석대로 Arm2 는 자신의 하류 전체(약 211g)를 지지하는 1차
weight-bearing joint다. Arm1(요)이나 Arm5/그리퍼 롤 같은 수평축과는 risk
profile 이 다르다.

### camera/end-effector가 급격히 떨어질 가능성

DaBai(29.4g)와 그리퍼 조립체가 Arm2 하류에 포함되므로, torque OFF 상태에서
Arm2 가 중력 모멘트를 못 버티면 **DaBai와 그리퍼가 함께 낙하/드룹**한다 —
카메라 마운트·케이블에 급격한 하중이 걸릴 수 있다.

### torque ON 복구 실패 가능성

`Arm_serial_set_torque(1)` 호출 자체는 write-only 명령이라 I2C 트랜잭션은
"보낸 것으로 기록"되지만(P-007F-01 과 동일한 fail-closed 불확실성), 실제로
servo 가 그 즉시 holding 을 재개하는지 **읽어서 확인할 방법이 없다**(torque
읽기 API 없음, P-007F-01 Task E). 즉 **복구 성공을 소프트웨어로 보장할 수
없다** — 복구는 "명령을 보냈다"까지만 보장되고, 실제 holding 재개는 물리
witness(Q1 재실행)로만 확인 가능하다.

### power cutoff와 torque command의 차이

- **physical power cutoff**: MCU/구동계 전원 자체를 차단 — 모든 servo가
  즉시 무동력 상태가 된다(torque 유지 불가, 돌연 drop 위험이 torque-write 보다
  **더 급격**할 수 있다). 복구는 전원 재투입 + 전체 재초기화가 필요하다.
- **torque command**(`Arm_serial_set_torque`): 소프트웨어 명령, I2C 로
  MCU에 전달 — MCU/서보가 정상 응답한다는 전제하에 더 통제된 전환이지만,
  "명령이 실제로 반영됐는지" 자체가 Q3 의 미해결 질문이다. power cutoff 는
  **확실히 즉시 무동력**이라는 점에서 오히려 "불확실한 소프트 전환"보다 특정
  상황에서 더 예측 가능할 수 있으나, 이번 목적(torque 상태를 *판별*하는 것)
  에는 쓸 수 없다 — cutoff 는 이미 "OFF"를 확정시켜 버려 ON/OFF 비교가 안 된다.

### restoration command가 정말 안전한지

**단정할 수 없다.** 위에서 분석한 대로 restoration(`torque(1)`)의 실제 효과를
확인할 공식 API가 없다. 따라서 restoration 이 "안전하다"는 주장은 Q2 실험
설계 안에 **포함되어서는 안 되고**, 별도의 물리 witness(재측정)로 매번
확인해야 하는 **별도의 검증 대상**으로 취급한다.

## 결론 — torque OFF 실험(Q2) REJECT 여부

**이번 gate 시점에서는 REJECT.** 이유:

1. 현재 physical pose 가 UNKNOWN 이라 Arm2 하류 ~211g 에 대한 실제 모멘트 암을
   사전에 알 수 없다 — 최악의 경우(수평으로 뻗은 자세) 를 배제할 증거가 없다.
2. restoration 성공을 소프트웨어로 확인할 방법이 없다 — "복구했다고 믿고
   진행"하는 설계가 된다.
3. 완화책(예: 토크 해제 전 운영자가 물리적으로 받쳐주는 spotting 절차)이
   **아직 설계되어 있지 않다**.

**Q1(torque 변경 없는 bounded-force probe)은 REJECT 대상이 아니다** — torque를
전혀 건드리지 않으므로 이 gate의 "ZERO HARDWARE STATE CHANGE" 원칙과
`position command 0` 원칙(§8) 을 모두 만족하면서 최소 물리적 증거(Q4)를 준다.
P-007F-03 범위를 "Q1 probe 우선 실행, Q2(실제 torque write)는 spotting
절차가 별도로 설계·승인된 뒤에만" 으로 분리하는 것을 권장한다.

## §8 — position command 금지 재확인

이 설계 전체에서 `154→156`, `154→152` 등 어떠한 position command 도 사용하지
않는다. Q1/Q2 witness 측정은 **외력(사람 손)과 torque command** 만 쓰고,
`Arm_serial_servo_write*` 는 Q1/Q2 어느 단계에도 등장하지 않는다.
