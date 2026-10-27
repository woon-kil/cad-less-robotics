# torque semantics — 공식 근거 (스펙 §13)

자료: `official/02-calibrating-arm.pdf` (Orin판 `2. Assembly course/2. Calibrating the Robotic Arm`, blob `3161df1f…`, 2페이지 전문 판독)
보조: `official/08-memory-action.pdf` (blob `9ee1ace0…`), `official/03-control-single-servo.pdf`, `05`, `06`, `04`

## 공식 원문

> **enable=0 turns off the torque, the robot arm can manually turn the servo angle, and does not receive the level signal; enable=1 turns on the torque, the servo receives the level signal to maintain the current angle, and the angle can be changed only after sending a command.**

## 스펙 §13 다섯 질문

| 질문 | 공식 답 | 근거 |
| --- | --- | --- |
| (a) 일반 이동 전 torque ON 을 항상 호출하는가 | **NO** | `03`·`05`·`06`·`04` 네 예제 전부 `Arm_serial_set_torque` 호출 0. `Arm_Device()` → `sleep(.1)` → `write` 가 전부 |
| (b) calibration·수동 교시 전용인가 | **대체로 YES** | 공식에서 `Arm_serial_set_torque` 가 등장하는 유일한 문서가 calibration 코스다. 학습 모드는 이 API 를 직접 쓰지 않고 보드가 자동으로 끈다 |
| (c) boot default torque 상태를 설명하는가 | **UNKNOWN** | 14종 어디에도 기술이 없다 |
| (d) torque 상태 read 방법이 있는가 | **UNKNOWN — 공식에도 없다** | API 소개가 `Return value: None` 이다. 설치본에도 읽기 경로가 없다(P-007C `0x1A` write-only). 공식·설치본 양쪽에서 동일하게 막혀 있다 |
| (e) torque ON 이 저장된 target 이동을 유발할 수 있다고 적는가 | **UNKNOWN** | 기술 없음. "the servo receives the level signal to **maintain the current angle**" 는 유지를 말하고 저장 target 복귀를 말하지 않는다. 이월 위험을 부정하는 근거로도 쓸 수 없다 |

## 무엇이 확인됐고 무엇이 안 됐는가

**확인된 것 — 메커니즘.** 제조사가 **토크 OFF 상태의 서보는 신호를 받지 않는다**고 명시한다. I2C 수준에서 명령이 ACK 돼도 서보가 무시하는 상태가 공식적으로 존재한다. P-007B 의 `NO_MOTION_RESPONSE`(예외 0 · `I2C error` 0 · 모션 0)를 설명할 수 있는 공식 메커니즘이다.

**확인되지 않은 것 — 상태.** 그 상태였다는 증거가 없다. 읽기 경로가 공식·설치본 양쪽에 없고 boot default 기술도 없다.

## 🔴 반대 정황 (판정에 반영해야 한다)

`evidence/P-007C/operator_observation.json`:

```
"arm_appearance": { "answer": "단단하게 유지 중으로 보인다",
                    "method": "육안. 만지지 않았다" }
"torque_off_history": { "answer": "없다 / 기억 없다" }
```

공식 정의상 torque OFF 는 `the robot arm can manually turn the servo angle` — 손으로 돌아가는 상태다. **운영자 관찰은 토크 ON 쪽을 가리킨다.** 특히 ID2 는 하중을 받는 관절이라, 토크가 꺼져 있었다면 자세가 어떻게 유지됐는지에 설명이 필요하다.

다만 그 관찰은 **육안이고 손으로 밀어 저항을 확인한 기록이 아니다**(`method: "육안. 만지지 않았다"`). 토크 OFF 상태의 팔이 외관상 어떻게 보이는지에 대한 근거가 우리에게 없으므로, 육안 관찰이 토크 ON 을 **확정하지도 배제하지도** 못한다. 그래서 H-C2 는 **기각되지 않고 후보로 남되, 반대 정황을 동반한 후보**다.

> 미검증 추론(기록만 하고 근거로 쓰지 않는다): 감속기 마찰로 토크 OFF 에서도 자세가 유지될 수 있다는 설명이 가능하지만, 이 기종에 대한 1차 근거가 없다. 공식 문구는 오히려 `can manually turn the servo angle` 로 **손으로 돌아간다**고만 적는다.

저항 확인은 하드웨어 상호작용이므로 이 게이트 범위 밖이다. 권고로만 남긴다.

## 이월 위험 (변함없음, 공식 근거로 강화)

`Arm_serial_set_torque(1)` 은 **모션 가능 행위**다. 공식이 (e)를 기술하지 않으므로 "켜도 안 움직인다"는 공식 보장이 없다. P-007B 가 보낸 target(ID2, raw 1242)이 컨트롤러에 래치돼 있을 가능성을 배제할 근거가 없다. 전체 모션 게이트 절차를 적용해야 한다.
