# P-007F-01 — Diagnosis

## 배경

P-007E-R2-R5 에서 관측된 현상:

- 실제 hardware write: 1회 (`Arm_serial_servo_write(2, 154, 2000)`)
- ID2 controller readback: `152 → 154`
- operator corrected observation: Arm2 physical motion = **NO**
- physical Arm2 shaft angle: UNKNOWN

이 gate는 **새로운 motion command를 보내지 않고** 다음 질문을 얼마나 좁힐 수 있는지
조사했다: `Arm_serial_servo_write()` 이후 readback은 변했는데 실제 actuator가
움직이지 않은 원인.

## 수행한 것

- Task A: 실제 설치된 Arm_Lib 1.0.3(`/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py`,
  sha `edfcae72...`, 471줄 전체) source forensic — `arm_lib_source_forensics.md`
- Task B: H1(실측)/H2(캐시)/H3(판별불가) 가설 분석 — `readback_semantics.md`
- Task C: read-only runtime probe(`Arm_serial_servo_read` 5라운드×6, `Arm_serial_servo_read_any(2)`
  5라운드, `Arm_ping_servo` id1-6, `Arm_get_hardversion`) — `readonly_probe.json`
- Task D: `/dev/i2c-7` 소유권 조사(kill/restart 없음) — `device_ownership.txt`
- Task E: torque 상태 읽기 API 존재 여부 조사 — 아래

## 핵심 발견

1. **아키텍처**: Jetson은 bus servo와 직접 통신하지 않는다 — I2C addr `0x15`의 단일
   MCU에만 명령/질의를 보낸다. 그 MCU의 펌웨어 소스는 이 시스템에 없다. 이것이
   readback semantics 판별 불가의 근본 이유다.
2. **readback 안정성**: ID2 readback은 5라운드(20ms 간격) 동안 완전히 안정적으로
   `154`를 보고했다(분산 0, I2C 에러 0).
3. **교차검증**: `Arm_serial_servo_read(2)`(register `0x32`, 반전 적용)=154 와
   `Arm_serial_servo_read_any(2)`(register `0x37`, 반전 미적용)=26 은
   `180-154=26`으로 수학적으로 정확히 일치 — 두 독립 레지스터 경로가 모순 없는 단일
   raw 값을 공유함을 보인다. 그러나 이것이 "실측값"이라는 증거는 아니다(두 경로가
   같은 캐시 변수를 참조해도 똑같이 일치할 수 있다).
4. **통신 레이어**: `Arm_ping_servo(1..6)` 전부 `0xda`(정상) — bus servo 통신 링크
   자체는(적어도 ping 레벨에서는) 응답하고 있다. 완전 단선/무응답 상태는 아니다.
5. **torque 읽기 API 없음**: Arm_Lib 1.0.3 전체에 torque 상태를 읽는 공식 API가
   존재하지 않는다(`Arm_serial_set_torque`는 write-only). 과거 수동 관찰(저항감 등)을
   torque=ON의 확정 증거로 승격하지 않는다.
6. **device ownership**: `/dev/i2c-7`을 여는 프로세스는 `yahboom_oled.py`(기존에
   문서화된 능동 사용자) 와 `kinemarics_dofbot` launcher wrapper(소스 확인상 I2C
   코드 0, inherited fd) 뿐 — 새로운 능동 사용자는 발견되지 않았다.

## 판정

### Readback semantics

**`READBACK_SEMANTICS_UNRESOLVED`**

source(Jetson 쪽 Arm_Lib 1.0.3)만으로는 `Arm_serial_servo_read(2)`가 실제 bus
servo의 measured position을 반환하는지(H1), MCU의 command/cache 값을 반환하는지(H2)
구별할 수 없다. 교차검증·안정성·ping 결과 모두 양쪽 가설과 양립하는 corroborating
신호일 뿐 결정적 증거가 아니다. 추론으로 승격하지 않고 `UNKNOWN — firmware semantics
required`로 명시한다.

### Torque state

**`TORQUE_STATE = UNKNOWN`**

torque 상태를 읽는 공식 API가 Arm_Lib 1.0.3에 존재하지 않는다
(`TORQUE_STATE_READBACK = UNAVAILABLE`). torque가 ON인지 OFF인지 read-only 로는
확인할 방법이 없다. 과거 수동 관찰을 확정 증거로 쓰지 않는다.

### Servo communication

**`SERVO_COMMUNICATION = SUPPORTED`**

`Arm_ping_servo(id)` 가 id 1-6 전부에서 `0xda`(정상)를 반환했다 — MCU가 통신 레이어
수준에서 bus servo 들로부터 응답을 받고 있음을 보인다(완전 단선/무응답이 아님).
단, 이것은 "position 명령이 실제로 기계적 회전을 일으켰는지"를 보장하지 않는다 —
통신 가능 ≠ 명령 수행 확인.

## 다음 단계에 대한 제언(이 gate에서 결정하지 않음)

이번 조사로는 readback 불일치의 근본 원인(캐시 vs 실측, 토크 상태, 기계적 걸림,
전원 문제 등)을 확정할 수 없었다. source/read-only 조사로 좁힐 수 있는 범위는
여기까지다 — 추가로 좁히려면 MCU 펌웨어 소스(없음) 또는 **상태를 변경하는 최소
실험**(예: torque 재확인·재명령·물리 측정)이 필요한데, 이는 이 gate의 범위 밖이며
별도 승인된 P-007F-02 설계가 필요하다. 이 gate에서는 그런 실험을 설계·제안하지
않는다 — 지시대로 여기서 STOP한다.
