# P-007F-01 Task B — Readback Semantics Determination

## 가설

- **H1**: `Arm_serial_servo_read(2)`가 실제 bus servo의 measured/current position을 반환한다.
- **H2**: controller board(MCU, I2C addr `0x15`)가 command/target/cache 값을 반환할 수 있다.
- **H3**: source만으로 H1/H2를 구별할 수 없다.

## 분석

### 1. 아키텍처상 구조적 한계

`Arm_serial_servo_read(id)`(및 `Arm_serial_servo_read_any`, `Arm_ping_servo`,
`Arm_get_hardversion`)는 전부 Jetson → I2C → **addr `0x15`인 단일 MCU**의 레지스터를
"트리거 write 한 바이트 → 짧은 sleep → 같은 레지스터 read" 패턴으로 질의한다
(`arm_lib_source_forensics.md` 참조). 이 MCU가 그 요청을 받고 **그 순간 실제로 bus
servo에 재질의(re-poll)해서 응답을 만드는지**, 아니면 **자신의 RAM에 보관된 마지막
write 값(또는 최근에 한 번 읽어둔 캐시)을 그대로 돌려주는지**는 전적으로 이 MCU의
펌웨어 로직에 달려 있다. 그 펌웨어의 소스는 Jetson 파일시스템(`/home/jetson/...`)
어디에도 존재하지 않는다 — 공장에서 별도 칩에 플래시된 바이너리로 추정된다.

**결론: Jetson 쪽 Arm_Lib 1.0.3 소스만으로는 H1/H2를 구조적으로 구별할 수 없다.**

### 2. 두 개의 독립 레지스터 경로 교차검증(corroborating, 결정적이지 않음)

이번 gate 의 read-only probe(Task C)에서 동일 물리 서보(ID2)를 **두 개의 서로 다른
레지스터 경로**로 질의했다:

| 경로 | register | 반환값(5라운드 전부 동일) | 반전 적용? |
|---|---|---|---|
| `Arm_serial_servo_read(2)` | `0x32` | **154** | 적용(`180-pos`, id∈{2,3,4} 전용) |
| `Arm_serial_servo_read_any(2)` | `0x37` | **26** | 미적용 |

`180 - 154 = 26` — **정확히 일치**한다. 즉 두 경로는 서로 다른 레지스터·서로 다른
후처리 공식(한쪽만 좌우반전)을 거치지만, **그 밑의 raw register 값은 수학적으로
동일한 하나의 값**에서 나온 것으로 재구성된다.

이것이 의미하는 것과 의미하지 않는 것을 분명히 구분한다:

- **의미하는 것**: 두 독립 코드경로가 서로 모순되지 않는다 — 디코딩 노이즈·레지스터
  오염·우연한 불일치가 아니라 일관된 단일 값을 보고하고 있다. readback 자체가
  무작위/깨진 값이 아니라는 신호다.
- **의미하지 않는 것**: 이 일치가 "그 raw 값이 실시간 측정값"이라는 증거는 **아니다**
  — `0x32`(전용 레지스터)와 `0x37`(범용 "any id" 레지스터) 둘 다 MCU 펌웨어 내부의
  **같은 캐시 변수**를 그대로 반환하도록 구현되어 있어도 똑같이 일치하는 결과가
  나온다. 즉 이 교차검증은 H1/H2 둘 다와 양립 가능하다.

### 3. `Arm_ping_servo` 결과의 해석 한계

ID 1-6 전부 `0xda`(정상)를 반환했다(Task C) — 이는 "MCU가 해당 ID의 bus servo와
통신 레이어에서 응답을 받고 있다"는 신호로, **servo가 완전히 단선/무응답 상태는
아님**을 뒷받침한다(아래 SERVO_COMMUNICATION 판정에 사용). 그러나 이 ping 이 position
레지스터와 같은 질의 경로를 쓰는지, 통신 health check 만 하는지는 source에 명시되어
있지 않아 — position readback 의 H1/H2 판별에는 **직접 기여하지 않는다**.

### 4. 안정성(stability)에 대한 해석 한계

5라운드(20ms 간격) 전부 `154`로 완전히 안정적이었다(분산 0). 이 안정성은:

- H1 하에서: 서보가 실제로 154° 근방에서 **정지해 있다**(기계적으로 그 위치에
  있다)는 뜻일 수 있다.
- H2 하에서: MCU가 **명령 레지스터를 캐시**하고 있을 뿐이라면, write 가 성공한 순간
  이후로는 당연히 항상 안정적으로 같은 캐시값을 반환한다 — 실제 shaft 가 움직였는지
  여부와 무관하게 안정적으로 보인다.

안정성 자체는 두 가설 모두와 양립하므로 판별력이 없다.

## 판정

**`READBACK_SEMANTICS_UNRESOLVED`**(H3) — 다음 근거로 확정한다:

1. Jetson 쪽 Arm_Lib 1.0.3 소스는 addr `0x15` MCU 와의 I2C 트랜잭션만 정의하고, 그
   MCU가 bus servo를 실시간 재질의하는지 캐시를 반환하는지는 MCU 펌웨어(소스 없음)의
   구현 세부사항이다 — **source 로 증명 불가능**.
2. 교차검증(read vs read_any)은 "raw 값이 일관되다"를 보이지만 H1/H2 어느 쪽도
   배제하지 못한다.
3. 안정성(5라운드 무변화)도 두 가설 모두와 양립한다.

추론만으로 H1 또는 H2 중 하나를 확정하지 않는다 — 지시된 그대로
`UNKNOWN — firmware semantics required` 로 판정한다.
