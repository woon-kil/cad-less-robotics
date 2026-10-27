# P-007C-R2 — Learning-Mode Visual State Observation

**판정: `OBSERVATION PASS — LEARNING MODE EXCLUDED`**

**I2C 0 · 코드 실행 0 · ssh 0 · torque 변경 0 · servo command 0 · 버튼 조작 0 · 로봇 모션 0.**
이 게이트는 **육안 관찰과 사진 한 장**으로만 수행됐다.

## 질문

> 현재 expansion board 의 RGB 가 제조사가 정의한 breathing-light 상태인가.

## 답 — **아니다**

확장보드에서 발광 중인 LED 는 셋이고 전부 실크 라벨이 붙은 표시등이다.

| 실크 라벨 | 상태 | 해석 |
| --- | --- | --- |
| `Servo` | 적색 **고정** | 서보 전원 레일 표시 |
| `5V` | 적색 **고정** | 5V 로직 전원 레일 표시 |
| `MCU` | 적색 **점멸** | MCU heartbeat |
| **RGB LED** | **소등** | 공식 문서가 learning mode 표시로 지목한 바로 그 소자 |

운영자 실물 사진(`board_led_observation.png`)에서, 프레임 좌상단의 **대형 백색 사각 패키지**(5050형 어드레서블 RGB LED 외형)가 소등 상태다. 표면의 붉은 기는 인접 적색 LED 의 **반사광**이고 자체 발광이 아니다.

**판독 근거(광학적):** 같은 프레임·같은 노출에서 점등 중인 세 LED 는 중심부가 백황색으로 **포화**되고 주변에 적색 **헤일로**가 퍼진다. 백색 사각 패키지에는 포화도 헤일로도 없다. 공식 programming-mode 사진에서 이 RGB LED 는 보드 전역을 청백색으로 물들일 만큼 밝다 — 점등이면 놓칠 수 없다.

→ 공식 정의상 learning mode 진입 시 RGB 가 breathing light 가 되어야 한다. 그렇지 않다. **learning mode 가 아니다.**

## 🔴 이것이 증명하지 않는 것

**torque ON 을 증명하지 않는다.**

공식 자료는 breathing RGB 를 **learning mode 의 외부 표시**로 정의할 뿐, 모든 torque-OFF 가 breathing RGB 를 만든다고 말하지 않는다. `Arm_serial_set_torque(0)` 으로 직접 꺼진 경우에는 RGB 표시가 없다.

이 비대칭은 **관찰 전에** 플랜에 고정해 두었다. 결과를 보고 해석을 맞추지 않았다.

## 벤더 근거 — `VENDOR_WEB_VERIFIED`

두 공식 채널에서 동일 문구를 확인했다(`vendor_verification.json`).

1. **공식 저장소 PDF** — pin `57cc4369…`, blob `9ee1ace0…`, `git hash-object` 검증
2. **공식 웹** `yahboom.net/study/DOFBOT-Pro/12626/` — 이 세션에서 직접 fetch

> "the RGB light on the robotic arm expansion board will show a breathing light effect" / "the robotic arm will automatically disable torque"

교차 세대 자료(Jetson Nano 판)도 같은 동작을 적어 오기 가능성을 낮춘다. 🔴 단 **다른 세대이므로 Pro 의 1차 근거로 쓰지 않았다.**

두 채널은 **같은 벤더의 서로 다른 배포본**이다. 일치는 전사·판독 오류를 배제하는 데까지이고, 벤더 문서 자체가 틀렸을 가능성은 배제하지 않는다.

## 가설 상태 변화

| 가설 | 이전 (P-007C-R1) | 현재 |
| --- | --- | --- |
| H-C6 learning mode | `SECOND_OFFICIAL_MECHANISM_IDENTIFIED` | **`ELIMINATED` (관측)** |
| H-C2 torque enable | `OFFICIAL_MECHANISM_CONFIRMED / STATE_UNREADABLE` | 변화 없음 — 이제 **유일하게 남은 공식 메커니즘** |
| H-C5 command propagation | `OPEN` | 변화 없음 |

## 덤으로 얻은 것

`SERVO` 라벨 LED 가 점등 상태다. **서보 전원 레일 부재 가설**을 약화시킨다 — 공식 common-problems Q3 가 전원 점검을 1차 조치로 제시하므로 확인 가치가 있었다.

🔴 `INFERENCE_NOT_EVIDENCE`. 공식 자료 14종에 이 LED 들의 정의가 **없다.** 라벨 문자열에서 읽은 통상적 해석이고, 레일 점등이 각 서보 단자 전압을 보장하지도 않는다.

## 다음

**P-007D 를 아직 열지 않는다.**

남은 단일 질문은 **"지금 torque 가 켜져 있는가"** 이고, 가장 싼 판별은 운영자가 **ID2 를 아주 약하게 밀어 저항을 느끼는 것**이다. 공식 정의상 torque OFF = `can manually turn the servo angle` 다.

🔴 별도 게이트로 연다. 하드웨어 상호작용이고, **torque ON 상태에서 force 를 주면 기어 손상 위험**이 있으므로 "아주 약하게, 저항이 느껴지면 즉시 멈춘다"를 절차에 명시해야 한다.

## 절차 기록 — 이 게이트가 6라운드 걸린 이유

1라운드 질문이 **"확장보드 RGB"** 라고만 해서 보드 위 **어느 소자**를 봐야 하는지 지시하지 못했다. 운영자가 "확장보드란?" 되물었고, 이후 LED 를 하나씩 좁히는 데 4라운드가 더 들었다.

**교훈: 관찰 게이트는 관측 대상을 부품 수준으로 지목해야 한다.** 공식 사진(RGB 위치에 네모 표시, 평상시/점등 대조)을 1라운드에 함께 보냈으면 2라운드에 닫혔다.

## 산출물

`acceptance.json`(R2-01~12 전부 PASS) · `vendor_verification.json` · `operator_observation.json`(round 1~6) · `board_led_observation.png`(운영자 실물 사진, 1차 증거)

## STOP

토크를 켜지 않았다. 버튼을 누르지 않았다. 코드를 돌리지 않았다. P-007D 를 열지 않았다.
