# P-007D-R2 — Servo Command Transport Observability Decision Gate

**판정: `OBSERVABILITY EXHAUSTED / R2-C / CLOSED`**

**hardware access 0 · 촬영 0 · wiring 0 · probe 0 · analyzer 연결 0 · I2C 0 · mutation 0 · motion 0 · sudoers 0 · 구매 0**

이 게이트는 **새 실험이 아니라 다음 실험 방법을 결정**하는 의사결정 게이트였다.
중간 기록은 `README_INTERIM.json` 에 남아 있다(게이트 진행 과정의 기록).

## 완료 보고 (스펙 13항목)

| | 항목 | 결과 |
| --- | --- | --- |
| 1 | kernel trace 기술적 가용성 | **있음**. `CONFIG_TRACEPOINTS=y` · `CONFIG_FTRACE=y` · `CONFIG_I2C_CHARDEV=y` · `CONFIG_I2C_SLAVE=y` (kernel `5.15.148-tegra`). 단 `/sys/kernel/debug` 가 `drwx------ root:root` 이고 `sudo -n` 불가 — 비밀번호 필요 |
| 2 | kernel trace evidence ceiling | **`maximum boundary = L3`** |
| 3 | Jetson-side 외부 capture 평가 | **`NOT_SELECTED`** — kernel trace 와 정보 중복 |
| 4 | servo bus 공식 식별 | STM8 coprocessor · `TXD`/`RXD` + `Servo` 점퍼 · S1~S6 bus-servo interface = **`VENDOR_WEB_VERIFIED`** |
| 5 | electrical interface | **UART serial command** |
| 6 | voltage | 서보 **공급**전압 body 6.0~7.4 V / gripper 4.8~6.0 V. 🔴 **UART logic voltage = `UNKNOWN`** |
| 7 | protocol / baud | UART / **115200** (`VENDOR_WEB_VERIFIED`). packet format = `UNKNOWN` |
| 8 | physical probe point | **`Servo`/`Upload` 점퍼 주변 STM8 `TXD`/`RXD`** — 공식 설명 + **공식 보드 사진 실크로 독립 확인** |
| 9 | passive capture safety | 🔴 **`UNKNOWN` — BLOCKER** |
| 10 | analyzer 최소 사양 | 아래 §R2-5. **voltage tolerance 를 적을 수 없다** |
| 11 | 최종 | **R2-C** |
| 12 | 다음 Gate 권고 | controlled servo diagnostic (아래 §다음) |
| 13 | counts | 전부 **0** |

## R2-2 — kernel trace ceiling 이 L3 인 이유

커널 i2c tracepoint 가 노출하는 것은 두 가지다.

- **(a) 어댑터에 전달된 바이트열** — P-007D 가 소스 산술로 `0x12` / `[0x04,0xDA,0x07,0xD0]` 를 **이미 정확히 재구성**했고 기록값과 일치했다. 중복이다
- **(b) 어댑터 수준 전송 결과(ACK/NACK)** — P-007C 가 "예외 없음" 에서 이미 추론한 것과 거의 같다

I2C ACK 는 슬레이브의 **하드웨어·ISR** 이 내는 것이지 애플리케이션 로직이 내는 것이 아니다.
→ **Jetson 커널에서 L4(펌웨어가 payload 를 해석하고 행동했는가)를 원리상 볼 수 없다.**

🔴 이것은 tracepoint 가 노출하는 범위에 대한 **추론**이다. 실행해 본 것이 아니다.
그래서 R2-A 를 **제거하지 않고 `DEFERRED`** 로 둔다 — 실제 transaction 에서 **NACK 가 찍히면 L3 가정 자체가 깨지므로** 반증 가치가 있다.

## 후보 비교

| 후보 | 증명 가능 | 판정 |
| --- | --- | --- |
| **A** Jetson kernel I2C trace | L3 확정. L4/L5 불가 | **`DEFERRED / LOW INFORMATION GAIN`** (not eliminated) |
| **B** Jetson↔보드 외부 I2C capture | A 와 같은 정보 | **`NOT_SELECTED`** |
| **C** 보드→servo bus capture | 🔴 **L4/L5 를 직접 분리하는 유일한 후보** | **`BLOCKED BY ELECTRICAL PROBE SAFETY`** |

## probe point 설계 변경 (검수 세션)

**S1~S6 를 probe point 로 삼지 않는다.** `Servo`/`Upload` 점퍼 주변 **STM8 `TXD`/`RXD`** 가 더 낫다.

서보 커넥터의 미확정 전기 구조를 건드리지 않고 **"STM8 이 P-007B 명령 후 servo 방향으로 데이터를 내보냈는가"** 라는 핵심 질문을 더 직접 본다.

🔴 **첫 capture 에는 다음이 필요 없다**: S1~S6 pinout · servo packet format · protocol decode.
P-007B 와 같은 명령 시점에 **STM8 servo-side TX 에서 waveform/activity 가 발생하는가** 만으로 경계가 갈린다.

- activity **있음** → 문제 범위가 `L5/L6` 로 이동
- activity **없음** → 문제 범위가 `L4 firmware processing/forwarding` 로 강하게 좁혀짐

## 확정된 아키텍처

```
Jetson
  │ I2C / addr 0x15
  ▼
YB-EVV01 / STM8 coprocessor
  ├── TXD ──┐
  └── RXD ──┤ [Upload | Servo] 점퍼
            ▼  Servo side
          S1~S6  (bus-servo interface, cascade 지원)
            ▼
       ID1~6 serial bus servo   UART 115200 / PH2.0-3PIN
```

`TXD`/`RXD` + `Upload`/`Servo` 는 **공식 설명과 공식 보드 사진 실크로 독립 확인**됐다 (`official_extract_manifest.json`).

## 🔴 미확정 — 추측하지 않는다

| 항목 | 상태 | 첫 capture 에 필요 |
| --- | --- | ---: |
| **UART logic voltage** | **`UNKNOWN`** | **✓ BLOCKER** |
| **probe ground reference** | **`UNKNOWN`** | **✓ BLOCKER** |
| S1~S6 핀 순서 | `UNKNOWN` | 아니오 |
| packet format | `UNKNOWN` | 아니오 |
| half/full duplex 토폴로지 | `UNKNOWN` | 아니오 |
| STM8 패키지 마킹 | `UNKNOWN` | 아니오 |

🔴 **서보 공급전압(6~7.4 V)과 UART logic voltage 는 전혀 다른 값이다.** 7.4 V 서보라고 logic 이 7.4 V 인 것이 아니다.

사진을 더 찍어도 해결되지 않는다. **지금 필요한 것은 실크가 아니라 전기적 근거다.**

## R2-5 — 계측 요구 사양 (구매하지 않음)

| 항목 | 산출 가능? | 값 |
| --- | --- | --- |
| 최소 샘플레이트 | ✅ | UART 115200 → bit period 8.68 µs. decode 에 ≥10× 권장 → **≥2 MS/s**. 저가 analyzer(24 MS/s)도 충분하다 — **제약이 아니다** |
| 채널 수 | ✅ | **1~2채널** (TX 필수, RX 선택) + GND |
| **입력 전압 허용범위** | 🔴 **불가** | **logic voltage 를 모르므로 적을 수 없다** |
| 프로브 액세서리 | 🔴 불가 | probe point 물리 형상·ground reference 미확정 |

→ **사양표의 핵심 칸을 채울 수 없다는 것 자체가 NO-GO 근거다.** 샘플레이트는 문제가 아니었고, 막는 것은 전기 안전이다.

## 왜 여기서 멈추는가

전원 OFF continuity 로 ground 는 특정할 수 있다. 그러나 **logic HIGH voltage 는 끝내 해결되지 않는다.** 그러려면 powered voltage measurement 가 필요하고, 그것을 위해 또 별도 측정 게이트와 probe safety 설계가 필요하다.

🔴 이 프로젝트의 목표는 **expansion-board firmware reverse engineering 이 아니다.** 실제 로봇을 bring-up 해서 CAD→geometry→planning→execution 파이프라인을 검증하는 것이다. 외부 계측으로 얻으려는 것은 결국 **L4 와 L5 사이 한 칸의 분리**다.

## 최종 경계

```
L1 Python API              VERIFIED
 ↓
L2 Arm_Lib encoding        VERIFIED
 ↓
L3 SMBus invocation        SUPPORTED
 ↓
L4 STM8
 ├ RGB    0x02 / 3B        VERIFIED — physical effect
 └ Servo  0x12 / 4B        UNKNOWN
       ↓
L5 STM8 → servo bus        UNKNOWN
       ↓
L6 servo                   OBSERVED_NO_MOTION
```

## 다음 — 이 세션은 설계하지 않는다

**또 다른 forensic gate 가 아니라 controlled servo diagnostic 으로 돌아간다.**

🔴 **P-007B 를 그대로 반복하면 안 된다.** "Arm2 를 또 +2° 보내본다" 가 아니라, **최소 1회 명령으로 servo subsystem 자체가 현재 command 를 실행할 수 있는지**를 판별하도록 설계해야 한다.

고려 요소: 기존 목표 위치 latch 가능성 · load 방향 · readback quantization · 명령 크기/시간.

현재 들고 들어가는 사실:

- torque 는 기계적으로 `TORQUE_ON_STRONGLY_SUPPORTED` (R3)
- learning mode 제외 (R2)
- global block-write failure 제외 (D-R1)
- 공식 API encoding 이상 없음 (D)
- RGB/STM8 command path 동작 (D-R1)
- 🔴 **그런데 ID2 servo command 만 physical effect 없음**

**설계는 검수 세션이 한다.** 이 세션은 R2-C 문서화까지만 하고 STOP 한다.

## provenance 경계 (유지)

제3자 GitHub 에 **다른 세대·다른 transport 의 `Arm_Lib`** 가 존재한다. 그것을 현재 YB-EVV01 I2C 구현의 근거로 **섞지 않는다.**

기준은 계속 **설치본 SHA `edfcae72…` + pinned 공식 자료 `57cc4369…`** 다.
(P-007C-R1 `transport_generations.json` 의 세대 분리 원칙과 동일 — 공식 저장소 안에도 타제품 serial 자료가 섞여 있었다)

## 산출물

`acceptance.json`(11항목 전부 PASS) · `README_INTERIM.json`(중간 기록) · `official_extract_manifest.json` · `official_extract/`(gitignore)

## STOP

kernel trace 실행하지 않았다. logic analyzer 연결하지 않았다. servo command 재전송하지 않았다. 구매하지 않았다. 추가 촬영하지 않았다. **다음 motion gate 를 설계하지 않았다.**
