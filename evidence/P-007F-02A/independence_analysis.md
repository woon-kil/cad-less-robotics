# P-007F-02A — Independence Test

각 후보가 다음 네 가지와 독립인지 판정한다: Arm_Lib readback · MCU state ·
servo target register · DaBai ego-motion.

## A. Smartphone inclinometer

| 대상 | 독립 여부 | 근거 |
|---|---|---|
| Arm_Lib readback | 독립 | 전화기 자체 IMU(가속도계/자이로), I2C addr `0x15` MCU 와 무관 |
| MCU state | 독립 | 동일 |
| servo target register | 독립 | 동일 |
| DaBai ego-motion | 독립 | 별도 센서 원리(IMU vs 카메라 픽셀) |

```text
INDEPENDENT_PHYSICAL_WITNESS
```

(단, 보유/장착 여부는 `instrument_inventory.md`/`smartphone_loading_analysis.md`
에서 별도로 미확정으로 남김 — 독립성 판정과 가용성 판정은 별개다)

## B. Physical inclinometer / digital level

동일 논리로:

```text
INDEPENDENT_PHYSICAL_WITNESS
```

(보유 자체가 `NOT VERIFIED` — `instrument_inventory.md`)

## C. Printed protractor + fixed external observation

| 대상 | 독립 여부 | 근거 |
|---|---|---|
| Arm_Lib readback | 독립 | 순수 기계적/광학적 판독, I2C 와 무관 |
| MCU state | 독립 | 동일 |
| servo target register | 독립 | 동일 |
| DaBai ego-motion | 독립 | 관찰 카메라를 쓴다면 **DaBai 가 아닌** stationary 카메라만 허용(`protractor_design.md` §4) — 설계 자체가 DaBai 의존을 배제한다 |

```text
INDEPENDENT_PHYSICAL_WITNESS
```

## D. Fixed external camera(Gemini/Mac/기타)

Gemini/Mac 모두 I2C/MCU/servo register 와 무관한 별도 host·별도 채널
(`evidence/P-007E-R2-R1/sensor_inventory.json`: Gemini=p620, Mac=별도 host,
DaBai=Jetson). 사용 가능하다면:

```text
INDEPENDENT_PHYSICAL_WITNESS
```

이지만 `instrument_inventory.md` D 항목에서 이미 "신규로 사용 가능한 후보
없음"으로 판정했으므로 이 독립성 판정은 **현재 실행 불가능한 후보에 대한
이론적 결론**일 뿐이다.

## DaBai (참고 — primary 로 쓰지 않음, P-007F-02 에서 이미 결정)

```text
NOT_INDEPENDENT (해당 목적에서)
```

Arm_Lib/MCU/servo register 와는 센서 채널 수준에서 독립이지만, **측정
대상인 로봇 구조 자체에 물리적으로 실려 있어** "그 구조가 움직였는가"를
묻는 이번 목적에서는 독립 witness 가 될 수 없다 — ego-motion 측정은 측정
대상과 측정 장치가 분리되지 않는다는 것이 핵심 결함이다(P-007F-02
`camera_witness_analysis.md`와 동일 결론, 재확인).

## 요약

| 후보 | 독립성 판정 | 가용성(참고, 독립성과 별개) |
|---|---|---|
| A. Smartphone | INDEPENDENT | OPERATOR_CONFIRMATION_REQUIRED |
| B. Physical inclinometer | INDEPENDENT | NOT VERIFIED |
| C. Protractor | INDEPENDENT | OPERATOR_CONFIRMATION_REQUIRED(재료) |
| D. Fixed camera | INDEPENDENT(이론상) | 신규 사용가능 후보 없음 |
| DaBai | NOT INDEPENDENT | 가용(그러나 이 목적에 부적합) |
