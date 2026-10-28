# P-007F-02 — Result Matrix (Preregistered for the Next Execution Gate)

🔴 사후(threshold/분류를 데이터를 본 뒤) 변경 금지. 이 matrix는 P-007F-03(또는
그 이후 실행 gate)의 결과를 분류하기 위해 **데이터를 보기 전에** 고정한다.

## 입력

- `controller/API intervention`: `Arm_serial_set_torque()` I2C 호출이 예외 없이
  완료됐는가(성공) — P-007F-01 과 동일한 fail-closed 관례를 따른다(예외가
  나도 보드 도달 여부를 모르면 "성공"으로 낙관 판정하지 않고 별도 invalid 사유로
  기록).
- `objective physical change`: Candidate A(primary) 측정치(외력-변위)가 사전
  고정된 임계값을 넘는가. 임계값 수치 자체는 P-007F-03 에서 서보 사양과
  Candidate A 캘리브레이션을 거쳐 확정한다(이 gate 는 "사전고정 임계값이
  있어야 한다"는 요구사항만 못박는다 — 숫자를 사후에 끼워 넣지 않는다).
- DaBai ego-motion 은 secondary 신호로 **기록**하되 이 matrix의 입력으로 쓰지
  않는다(Candidate A 가 primary).

## Matrix

### A

```text
controller/API intervention 성공
+ objective physical torque-state change 확인
→ ACTUATOR_TORQUE_RESPONSE_SUPPORTED
```

의미: torque command 가 실제로 physical holding state 를 바꾼다는 증거 확보.

### B

```text
controller/API intervention 성공
+ physical change 없음
→ ACTUATOR_TORQUE_RESPONSE_NOT_ESTABLISHED
```

의미: MCU는 명령을 접수(또는 접수한 것으로 보임)했지만 physical witness 상
holding state 차이가 측정되지 않음 — P-007F-01 의 H2(캐시/명령-수신 ≠ 실제
actuator 반응) 가설과 정합.

### C

```text
controller/API intervention 실패
+ physical change 발생
→ UNEXPECTED_PHYSICAL_STATE_CHANGE
```

의미: 명령이 실패(예외/오류)했는데 물리적으로 뭔가 바뀜 — 예상 밖, 별도 조사
필요(이 gate는 그 조사를 설계하지 않는다).

### D

```text
둘 다 변화 없음
→ NO_OBSERVABLE_TORQUE_RESPONSE
```

의미: 명령도 실패하고 물리적 변화도 없음 — 통신/실험 설정 자체를 재검토해야
한다는 신호.

### X

```text
measurement/witness/safety invalid
→ EXPERIMENT_INVALID
```

트리거 예: Candidate A 측정 실패(기구 오류·낙하 등 안전조건 발동·측정자 접근
불가), DaBai POST window quality guard 실패(P-007E-R2-R5 Attempt #4 와 동일
패턴), git/module SHA 불일치, operator 승인 누락.

## 주의

- A/B/C/D 어느 쪽이든 **physical Arm2 shaft angle 의 절대값**을 확정하지
  않는다 — 이 matrix는 "torque command 와 physical holding state 사이의
  인과관계"만 판별한다.
- X 는 B 와 다르다 — B 는 "측정했지만 변화가 없다"이고 X 는 "측정 자체가
  유효하지 않다"이다. 둘을 혼동해 B 로 낙관 분류하지 않는다.
