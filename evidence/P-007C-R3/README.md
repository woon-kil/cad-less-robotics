# P-007C-R3 — Passive Torque-State Mechanical Observation

**판정: `OBSERVATION PASS — TORQUE_ON_STRONGLY_SUPPORTED`** (분류 **B — HOLDING RESISTANCE**)

**ROBOT COMMAND COUNT = 0 · CONTROLLER STATE MUTATION = 0 · I2C 0 · 코드 실행 0 · 관절 이동 0**

## 관찰

운영자가 Arm2(ID2)에 **극소 힘**을 가했다. 관절이 움직이기 **전에** 명확한 holding resistance 를 느끼고 즉시 멈췄다.

| 보고 항목 | 값 |
| --- | --- |
| 1. 극소 힘 적용 | YES |
| 2. 실제 관절 이동 | **NO** |
| 3. holding resistance | **YES** |
| 4. servo buzz · 기어음 | NO |
| 5. 예상치 못한 motion | NO |
| 6. 최종 분류 | **B** |
| 7. robot command count | **0** |
| 8. controller state mutation | **0** |

공식 정의상 torque OFF 는 `the robot arm can manually turn the servo angle` 다. 수동으로 자유롭지 않다는 것은 **토크 ON 쪽 정황**이다.

P-007C 의 독립적 육안 관찰("단단하게 유지 중으로 보인다")과 방향이 같다. **하나는 시각, 하나는 촉각이다.**

## 🔴 표현 한계 — `STRONGLY_SUPPORTED`이지 `VERIFIED`가 아니다

- 정밀 torque 측정이 아니라 **운영자 촉각 판단**이다
- 🔴 **"명확한 저항"이 능동 holding torque 인지 감속기 자체의 마찰·self-locking 인지 이 시험만으로 완전히 갈리지 않는다.** 시리얼 버스 서보는 감속비가 커서 토크 OFF 에서도 상당한 정지 마찰을 가질 수 있다
- 더 날카로운 판별은 "밀었다 놓으면 서보가 **능동적으로 원위치로 돌아오는가**"지만, 그것은 관절을 실제로 이동시키는 행위라 이 게이트 범위 밖이다. **수행하지 않았다**
- ID2 한 관절만 보았다

## 🔴 가설 지형이 바뀌었다

| 가설 | 이전 | 현재 |
| --- | --- | --- |
| H-C1 communication | `ELIMINATED` | 변화 없음 |
| H-C3 required startup/init | `NOT_SUPPORTED` | 변화 없음 |
| H-C4 wrong single-servo API | `NOT_SUPPORTED` | 변화 없음 |
| H-C6.learning_mode | `ELIMINATED` (R2) | 변화 없음 |
| **H-C2 current torque state** | **`OPEN` ← 최우선** | **`WEAKENED`** — 기각은 아니다 |
| **H-C5 command propagation** | `OPEN` | **🔴 `OPEN` ← 최우선으로 승격** |
| H-C6.other_controller_state | `OPEN` | 유지 |

**왜 H-C5 가 올라오는가.** 토크가 켜져 있다면 공식 정의상 서보는 `can receive control commands` 상태다. 그런데 P-007B 의 명령은 효과가 없었다. 그러면 **명령이 서보가 수락하는 형태로 도달하지 못했다**는 쪽이 남는다.

이것이 P-007C 의 `most_specific_finding` 과 정확히 맞물린다:

> 성공한 조회(`0x01` · `0x38` · `0x30+id`)는 전부 `write_byte_data` 1바이트 + read 계열이다.
> 효과가 없었던 유일한 것이 **`write_i2c_block_data` 4바이트(`0x12`)** 다.

## APP `[Stop]` 이력 — 재질의했으나 미해소

이 게이트 착수 전에 공식 APP 문서를 새로 확인했다(`vendor_basis.json`):

> 【Stop】: After clicking this button, robotic arm **turns off the torque, all servos no longer receives control commands.**

P-007A-R1 이 찾아 둔 `YahboomArm.py:401` 의 `set_torque(0)` 핸들러가 **UI 상 `[Stop]` 버튼**이라는 뜻이다. 토크는 보드 상주 상태라 **APP 프로세스를 kill 해도 해소되지 않는다.**

P-007C 는 "토크 끈 이력" 을 물었고 답은 "없다 / 기억 없다" 였다. 운영자가 `[Stop]` 을 누른 것을 "토크를 껐다"로 기억할 이유가 없으므로 **과소표집** 가능성이 있었다. 그래서 **행동 어휘로 다시 물었다.**

답: **"기억 없다 / 모르겠다"**

→ `UNRESOLVED_BY_RECALL`. 기억 부재는 부정이 아니다. **기억에 의존하는 경로로는 더 얻을 것이 없다.**

## 스펙에 없는 안전 항목을 추가했다

**반대 손으로 팔 상부 무게를 받친 상태**에서 시험하도록 절차에 넣었다.

사유: 분류 A(토크 OFF)가 참이면 ID2 는 하중을 받는 어깨 관절이라 **손을 떼는 순간 팔이 중력으로 처질 수 있다.** 스펙은 "원위치 복귀 명령 금지"만 적고 낙하 위험을 다루지 않는다. 운영자에게 추가 사유를 **먼저 밝히고** 진행했다.

## 이월 위험 — 일부 약해졌다

P-007C 가 적은 위험: *"원인이 torque·enable 이면 P-007B 의 target(ID2 raw 1242)이 래치돼 있다가 토크를 켜는 순간 움직일 수 있다."*

토크가 **이미 켜져 있는** 쪽이 유력해졌으므로 그 특정 경로는 약해진다 — 토크가 켜진 상태에서 팔이 움직이지 않았다면 래치된 target 이 토크 복귀로 발현할 여지가 줄어든다.

🔴 **제거되지는 않는다.** (a) 토크 ON 은 `STRONGLY_SUPPORTED`이지 `VERIFIED`가 아니다 (b) 컨트롤러가 target 을 보관하는지 자체가 미확인이다 (c) `Arm_Button_Mode(0)` · `set_torque(1)` 은 **여전히 모션 가능 행위**로 취급한다.

## 다음

🔴 **이제 P-007D 를 열 근거가 생겼다.**

남은 최유력 가설이 H-C5(block-write 경로)이고, P-007D 의 **Q2** 가 정확히 그것을 겨냥한다 — *"P-007B 에서 효과가 없었던 `write_i2c_block_data` transport 자체에 별도 의심 근거가 있는가."* 플랜은 이미 작성돼 있다(`docs/261027-1105-P-007D-*`).

단 **P-007D 는 I2C 조회를 수반한다.** R2·R3 의 관찰 게이트와 위험 등급이 다르다. 플랜을 R3 결과에 맞춰 재검토하고 승인받은 뒤 연다.

## 가설 라벨 정정

검수 세션 지적: P-007C-R2 가 배제한 것은 **H-C6 전체가 아니라 `H-C6.learning_mode` 분기**다. `H-C6.other_controller_state` 는 `OPEN` 으로 남는다.

🔴 P-007C-R2 는 CLOSED 선행 게이트다. 프로젝트 규칙(선행 증거 불변)에 따라 **그 디렉터리를 수정하지 않았다.** 정정은 `vendor_basis.json` 과 `PROJECT-STATUS.md`(evidence 와 분리된 살아 있는 ledger)에만 적었다.

## 산출물

`acceptance.json`(R3-01~12 전부 PASS) · `vendor_basis.json` · `operator_observation.json`

## STOP

P-007D 를 실행하지 않았다. torque ON/OFF 명령을 실행하지 않았다. 다음 motion test 를 실행하지 않았다. 관절을 이동시키지 않았다.
