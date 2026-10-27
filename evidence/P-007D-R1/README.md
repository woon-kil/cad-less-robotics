# P-007D-R1 — RGB Block-Write Control Experiment

**판정: `CONTROL PASS — BLOCK-WRITE PHYSICAL OUTPUT CONFIRMED` (결과 A)**

**RGB command 1 · servo command 0 · torque 0 · robot motion 0 · controller mutation 1**

이 프로젝트에서 **처음으로 컨트롤러 상태를 바꾼 게이트**다.

## 완료 보고 (스펙 12항목)

| | 항목 | 값 |
| --- | --- | --- |
| 1 | Arm_Lib SHA | 디스크 `edfcae72…` **+ 런타임 egg `3aba3583…`** 둘 다 일치 |
| 2 | preflight / exclusive control | PASS. APP·joystick·arm_driver·MoveIt 0. i2c-7 보유자는 승인 집합뿐 |
| 3 | RGB before | **OFF** (명령 직전 운영자 재확인, R2 사진과 연결) |
| 4 | 실행 API · RGB 값 | `Arm_RGB_set(0, 0, 255)` |
| 5 | command count | **1** |
| 6 | exception / I2C error | **없음 / 없음** (`library_stdout` 공백) |
| 7 | RGB after | **발광 — 파랑** |
| 8 | robot motion / servo buzz | **없음 / 없음** |
| 9 | 최종 | **A** |
| 10 | 기존 evidence 불변 | 94파일 · 변경 0 · 누락 0 · 추가 0 |
| 11 | controller mutation | **1** (RGB) |
| 12 | servo / torque / motion command | **0 / 0 / 0** |

실행 기록: `elapsed 0.000612s`, `commanded_at 2026-10-27T14:06:13+0800`

## 🔴 증명된 것

```
Python → Arm_Lib → SMBus write_i2c_block_data → expansion-board firmware → RGB physical output
```

**이 경로가 실제 장비에서 작동한다.**

→ **`GLOBAL_BLOCK_WRITE_FAILURE = ELIMINATED`**

P-007D 가 남긴 *"`write_i2c_block_data` 가 이 보드에서 수락된 사례 0건"* 이 **1건**이 됐다. block-write primitive 자체가 이 board/firmware 에서 무효인 것은 아니다.

**부수 획득**: 펌웨어 채널 순서가 **RGB** 임이 확정됐다 — `(0,0,255)` 가 파랑으로 나왔다. 공식 문서에 없던 사실이다.

## 🔴 증명되지 않은 것

| | 상태 |
| --- | --- |
| `SERVO_0x12_4B_ACCEPTANCE` | **UNRESOLVED** |
| `SERVO_BUS_PROPAGATION` | **UNRESOLVED** |

| | RGB | Servo (P-007B) |
| --- | --- | --- |
| register | `0x02` | `0x12` |
| payload | **3바이트** | **4바이트** |

P-007D 가 관찰한 **`register ↔ payload 길이` 1:1 대응** 구조 때문에, 수락 실패가 register 또는 길이 의존이면 이 결과가 서보 경로를 보증하지 못한다.

그리고 RGB 는 **보드 자체의 출력**이다. 서보 버스(L5)를 전혀 경유하지 않는다.

## 계층 경계 갱신

```
L1 Python API        VERIFIED
L2 Arm_Lib encoding  VERIFIED
L3 SMBus invocation  SUPPORTED
L4 firmware accept   PARTIALLY VERIFIED  ← 0x02/3B 수락 입증. 0x12/4B 는 UNKNOWN
L5 board→servo-bus   UNKNOWN             ← RGB 는 이 계층을 경유하지 않는다
L6 servo execution   OBSERVED_NO_MOTION
```

## 🔴 다음 게이트의 구조적 제약

설치 라이브러리의 block-write **7경로 중 비모션은 RGB(3바이트) 하나뿐**이다 (`P-007D/block_write_comparison.json`).

- `0x12`/4B · `0x19`/5B · `0x17`/4B · `0x1E`→`0x1D`/2→12B — **전부 모션**
- 부저(`0x06` 계열)는 `write_byte_data` 라 block-write 가 아니다

→ **`0x12`/4B 수락 여부를 모션 없이 가를 경로가 설치 라이브러리에 존재하지 않는다.** 이것이 다음 게이트 설계의 제약이다.

## ⚠️ 남겨진 상태 — 후속 관찰자 주의

**확장보드 RGB 가 파랑으로 켜진 채 남아 있다.**

플랜이 복구 OFF 명령을 금지했다 — 두 번째 block-write 가 되어 `command count = 1` 조건을 깬다.

🔴 이후 게이트에서 보드 LED 를 볼 때 **이 파랑은 우리가 켠 것**이다.
learning mode(breathing·색 변화)도 아니고 `Arm_Clear_Action` 완료(녹색)도 아니다.
**P-007C-R2 가 확보한 "RGB 소등" baseline 은 이 시점 이후 유효하지 않다.**

복구는 별도 승인에서 판단한다. 전원 재투입으로 해소될 것으로 보이나 재투입은 `joystick.py` 의 `move_init_pose()` 위험이 있어 금지 유지.

## 실행 전에 고친 것들

**preflight 에서 2건**

1. forbidden 프로세스 검사가 **자기 자신의 래퍼 셸**을 잡았다 — heredoc 본문이 args 에 들어가서다. 이 프로젝트에서 이미 한 번 난 `cmdline 부분일치` 버그와 같은 유형이고, 이번엔 실행 전에 잡았다. pid 조상 체인 제외로 수정
2. `/dev/i2c-7` 보유 프로세스 3개 발견 → 전부 P-007A-R1 이 남기기로 분류한 것이었다(OLED `0x3C`, ros2 래퍼). 같은 버스에 OLED 트래픽이 상존하지만 **P-007B 때부터의 상수**이지 새 변수가 아니다

**검토에서 4건**

3. 🔴 `Arm_RGB_set` 이 bare `except:` 로 I2C 예외를 삼키고 문자열만 출력한다 → `try/except` 는 **항상** `exception=None` 을 기록한다. stdout 을 캡처하지 않으면 NACK 로 실패한 실험이 **B(진단 신호)로 둔갑**한다. `redirect_stdout` 으로 잡고 `i2c_error_printed` 기록
4. command count 를 호출 **앞**으로 이동 — 성공이 아니라 **시도** 횟수다
5. 🔴 SHA 가드가 디스크 사본만 봤다 → 실제 import 되는 것은 **egg** 다(P-007C-R1 발견). egg 경로 assert + egg sha256 대조 추가. 없었으면 "SHA 일치" 가 **실행되지 않은 파일에 대한 주장**이 될 뻔했다
6. 색 판정이 채널 매핑에 의존하지 않도록 **1차 기준을 "5050 패키지가 발광하는가"** 로 바꾸고 **A′(발광하나 색이 다름)** 를 관찰 전에 분류에 추가. 운영자 질의도 발광 여부와 색을 분리

## 가설 상태

| 가설 | 변화 |
| --- | --- |
| **H-C5 command propagation** | **범위 축소.** "이 보드에서 block-write 전면 고장" 가설 제거. 남은 것은 **`0x12`/4B 서보 명령 고유의 수락 실패** 또는 **L5 이후** |
| H-C2 current torque state | `WEAKENED` 유지 |
| H-C6.other_controller_state | `OPEN` 유지 |

## 산출물

`acceptance.json`(15항목 전부 PASS) · `command_plan.json`(사전 고정 기준) · `source_integrity.json` · `preflight.json` · `command_log.json` · `operator_observation.json` · `result.json` · `logs/command_raw.txt` · `logs/evidence_manifest_{before,after}.json`

도구: `scripts/p007d_r1_rgb.py` (sha256 `7f2c6937…`)

## STOP

추가 RGB 명령을 보내지 않았다. 복구 OFF 명령도 보내지 않았다. servo command 0. P-006D 를 열지 않았다.
