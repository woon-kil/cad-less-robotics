# P-007D — Block-Write Command Propagation Static Diagnosis

**판정: `DIAGNOSIS PASS — COMMAND PATH VALID THROUGH HOST, FIRMWARE ACCEPTANCE UNRESOLVED`**
**분류: D2 — TRANSPORT BOUNDARY ISOLATED**

**ROBOT COMMAND 0 · I2C ACCESS 0 · CONTROLLER STATE MUTATION 0 · ROBOT MOTION 0 · RGB COMMAND 0**

기존 raw-state 중심 P-007D 는 폐기됐다. 이 게이트는 `H-C5 command propagation` 만 조사하는 **정적 분석 전용** 판본이다.

## 완료 보고 (스펙 10항목)

| | 항목 | 값 |
| --- | --- | --- |
| 1 | Arm_Lib SHA | `edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c` — 기준선 일치. 471행 · 28함수 |
| 2 | `write_i2c_block_data` call-site 수 | **13** (7개 함수) |
| 3 | API별 register / payload length | 아래 표 |
| 4 | P-007B source reconstruction 일치 | **완전 일치** — register `0x12` · payload `[0x04,0xDA,0x07,0xD0]` · raw `1242` |
| 5 | RGB path | `0x02` · 3바이트 · 같은 주소·같은 primitive · 모션 없음 · **육안 관측 가능**. 실행 0 |
| 6 | host-side 이상 발견 | **없음** — wrong register·payload·byte order·source divergence 전부 음성 |
| 7 | L1-L6 boundary | L1-L2 `VERIFIED` · L3 `SUPPORTED` · **L4-L5 `UNKNOWN`** · L6 `CONTRADICTED` |
| 8 | 최종 | **D2** |
| 9 | evidence immutability | 82파일 · 변경 0 · 누락 0 · 추가 0 |
| 10 | I2C / mutation / command / motion | **0 / 0 / 0 / 0** |

## SMBus call-site 전수 (§3)

| primitive | call-site | 함수 수 | 이 장비에서 검증됨 |
| --- | ---: | ---: | --- |
| `write_byte_data` | 22 | 18 | ✅ 조회 7회 + 읽기 선행 1,290+회 |
| `write_i2c_block_data` | **13** | **7** | 🔴 **0건** |
| `read_byte_data` | 5 | 4 | ✅ |
| `read_word_data` | 2 | 2 | ✅ 1,290+회 |

## block-write 7경로 비교 (§6)

| API | register | payload | 모션 | 관측 가능 효과 | 기실행 |
| --- | --- | ---: | --- | --- | --- |
| `Arm_RGB_set` | `0x02` | 3 | NO | **RGB 색 변화 (육안)** | NO |
| `Arm_serial_servo_write` | `0x10+id` | 4 | YES | 관절 이동 | **YES — 1회, 효과 없음** |
| `bus_servo_control` | `0x10+id` | 4 | YES | 관절 이동 | NO |
| `Arm_serial_servo_write_any` | `0x19` / `0x17` | 5 / 4 | YES | 관절 이동 | NO |
| `Arm_serial_servo_write6` | `0x1E` → `0x1D` | 2 → 12 | YES | 6축 동시 | NO |
| `Arm_serial_servo_write6_array` | `0x1E` → `0x1D` | 2 → 12 | YES | 6축 동시 | NO |
| `bus_servo_control_array6` | `0x1E` → `0x1D` | 2 → 12 | YES | 6축 동시 | NO |

register 가 payload 길이와 1:1 대응한다. 펌웨어가 register 별 고정 길이를 기대한다는 추정과 일관되나 **공식 근거는 없다.**

## 🔴 가장 구체적인 발견

> 설치 `Arm_Lib` 에서 `write_i2c_block_data` 를 쓰는 함수는 7개인데,
> **이 장비에서 실행된 적이 있는 것은 `Arm_serial_servo_write` 하나뿐이고 그 1회가 효과가 없었다.**
> 성공이 확인된 모든 연산은 `write_byte_data`(2바이트) 또는 `read` 계열이다.
> → **`write_i2c_block_data` 가 이 보드에서 의미 있게 수락된 사례가 0건이다.**

## §4 — P-007B 경로 소스 재구성 (실행 0)

`Arm_serial_servo_write(2, 152, 2000)` 을 소스 산술로 따라갔다.

```
L19  angle = 180 - 152                        = 28      ← ID 2·3·4 반전
L20  pos   = int(2200 * 28/180 + 900)         = 1242
L21  # pos = ((pos<<8)&0xff00)|((pos>>8)&0xff)          ← 주석. 비활성
L22  value_H = (1242>>8)&0xFF                 = 0x04
L23  value_L = 1242 & 0xFF                    = 0xDA
L24  time_H  = (2000>>8)&0xFF                 = 0x07
L25  time_L  = 2000 & 0xFF                    = 0xD0
L27  write_i2c_block_data(0x15, 0x10+2=0x12, [0x04,0xDA,0x07,0xD0])
```

P-007B 가 기록한 값과 **네 바이트·register·raw 모두 일치**한다. 설치 라이브러리가 의도한 그대로의 바이트가 나갔다.

## §7 — Transport boundary

```
L1 Python API                  VERIFIED     공식 API·공식 인자 형식 (P-007C-R1 R1-A)
L2 Arm_Lib encoding            VERIFIED     소스 재구성 완전 일치, byte order 정상
L3 Linux SMBus call            SUPPORTED    예외 0. 단 바이트열 자체는 미관측
────────────────────────────────────────── 정적 분석의 한계선
L4 firmware acceptance         🔴 UNKNOWN   block-write 수락 사례 0건. STM8 바이너리, 소스 없음
L5 board → servo-bus           🔴 UNKNOWN   L4 미결이면 결정 불가
L6 servo execution             CONTRADICTED 운영자 "전혀 움직이지 않았다"
```

🔴 **네 명제를 구분해서 유지했다.**
`write_i2c_block_data 가 예외 없이 반환했다` ≠ `firmware 가 수락했다` ≠ `servo 가 받았다` ≠ `servo 가 실행했다`

## D1 후보 검사 — 전부 음성

| 후보 | 결과 |
| --- | --- |
| wrong register | ✗ `0x10+id` 는 `bus_servo_control` 도 쓰는 벤더 표준 경로 |
| wrong payload construction | ✗ 소스 재구성이 기록값과 완전 일치 |
| **wrong byte order** | ✗ 주석 처리된 스왑을 검사 → **결함 아님**. 쓰기(비스왑)와 읽기(스왑)가 같은 big-endian 을 가리킨다 (`byte_order_analysis.md`) |
| installed source divergence | ✗ 공식 API semantics 와 명령 경로를 무효화할 차이 없음 |

부수 발견: 소스 헤더가 `# V1.0.1` 인데 설치 패키지는 **1.0.3** 이다. 헤더가 릴리스와 함께 갱신되지 않았다 — 명령 경로와 무관하지만 **버전 문자열을 provenance 근거로 쓸 때 주의**해야 한다.

## 가설 상태

| 가설 | 변화 |
| --- | --- |
| **H-C5 command propagation** | OPEN(최우선) → **호스트측 배제. L4 이후로 국소화.** "명령 전파 실패"에서 **"펌웨어가 block-write payload 를 수락하지 않는다"**로 구체화 |
| H-C2 current torque state | `WEAKENED` 유지 (R3) |
| H-C6.other_controller_state | `OPEN` 유지 |
| H-C6.learning_mode | `ELIMINATED` (R2) |
| H-C1 / H-C3 / H-C4 | `ELIMINATED` / `NOT_SUPPORTED` / `NOT_SUPPORTED` — H-C4 는 이 게이트가 소스 재구성으로 재확인 |

## 다음 — 제안만 한다

**RGB block-write control experiment.** 같은 주소·같은 primitive 를 쓰면서 **서보를 움직이지 않고** 효과가 육안으로 관측된다. L4 를 모션 없이 시험할 수 있는 유일한 경로다.

before 기준선은 **이미 있다** — P-007C-R2 가 RGB 소등 상태를 운영자 사진으로 확보했다.

🔴 **한계를 설계에 반영할 것**: RGB 는 3바이트, P-007B 는 4바이트다. 펌웨어 수락 실패가 **길이 또는 register 의존**이면 RGB 성공이 4바이트 서보 경로를 보증하지 못한다.

이 게이트에서 실행하지 않았다. controller state mutation 이므로 별도 승인이 필요하다.

## 🔴 하드웨어 접근에 대한 고지

**SSH 파일 읽기 1회를 했다.** `sha256sum` + `cat Arm_Lib.py` 뿐이다. `Arm_Lib` import 0 · I2C 0 · 보드 조회 0.

근거: 스펙 §2 가 *"파일 hash 만 다시 계산할 수 있다"* 고 적어 파일 접근을 상정한다. §3 의 AST 전수에는 소스 전문이 필요했고 로컬에는 함수 목록 + 2개 함수 본문만 있었다(`evidence/P-007B/logs/armlib_api_source.txt`).

분석은 전부 Mac 로컬에서 했다. 스크립트는 소스 디렉터리 **밖**(`scripts/`)에 두고, 경로를 인자로만 넘기고, `python3 -I` 로 실행했다.

## 산출물

`acceptance.json`(P007D-01~15 전부 PASS) · `block_write_inventory.json`(AST 42 call-site) · `block_write_comparison.json` · `single_servo_block_path.json` · `rgb_block_path.json` · `byte_order_analysis.md` · `propagation_boundary.json` · `official_provenance.json` · `logs/`

도구: `scripts/p007d_smbus_inventory.py`

## STOP

RGB 실험을 실행하지 않았다. P-006D 를 열지 않았다. torque command 를 실행하지 않았다. 서보 명령 0.
