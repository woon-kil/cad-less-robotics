# P-007F-01 Task A — Arm_Lib 1.0.3 Source Forensics

대상: Jetson 실제 runtime(`/usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg`,
sha256 `3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371`) 의 소스 원본
`/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py`(sha256
`edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c`), 471줄 전체를
2026-10-28 read-only ssh cat 으로 직접 읽어 분석했다(변경 0, 삭제 0).

🔴 **아키텍처 전제** — `Arm_Device.__init__`은 `smbus.SMBus(7)`로 `/dev/i2c-7`을 열고
`self.addr = 0x15`(단일 I2C 슬레이브 주소)에 모든 명령을 보낸다. 즉 Jetson은 **개별
bus servo 와 직접 I2C 로 통신하지 않는다** — Jetson↔I2C↔(addr 0x15, 별도 MCU/드라이버
보드)↔(그 MCU 내부에서 자체 프로토콜, 추정 UART)↔bus servo 1~6. 이 MCU의 펌웨어
소스는 Jetson 파일시스템에 없다(별도 칩에 공장에서 플래시됨) — 이것이 Task B의
H1/H2 판별이 **이 소스만으로는 구조적으로 불가능**한 이유다.

## 1. `Arm_serial_servo_write(id, angle, time)` — WRITE, 금지 대상

| 항목 | 값 |
|---|---|
| 위치 | Arm_Lib.py L15-45 |
| register | `0x10 + id` (id=2 → `0x12`) |
| 방향 | I2C **write** (`write_i2c_block_data`) — 4바이트 [value_H, value_L, time_H, time_L] |
| id=2,3,4 특수처리 | `angle = 180 - angle` (좌우 반전) 후 `pos = int((3100-900)*angle/180 + 900)` |
| id=5 | 별도 스케일 `(3700-380)*angle/270 + 380` |
| 예외처리 | bare `except:` — I2C 예외를 삼키고 `print('... I2C error')` 만 함(반환값 없음, 호출자는 stdout 캡처로만 실패를 안다) |
| 이번 gate 사용 | **0회** — 호출/참조 둘 다 없음(AST 로 보장) |

## 2. `Arm_serial_servo_read(id)` — READ(trigger-then-read), 허용·사용

| 항목 | 값 |
|---|---|
| 위치 | Arm_Lib.py L172-193 |
| register | `id + 0x30` (id=2 → `0x32`) — **동일 레지스터에 두 단계**: ① `write_byte_data(addr, id+0x30, 0x0)`(트리거) ② `time.sleep(0.003)` ③ `read_word_data(addr, id+0x30)` |
| 방향 | write(트리거 1바이트) + read(word) — 트리거 자체는 payload가 `0x0`(상수)이라 actuator 상태를 바꾸는 "명령"이 아니라 "다음 read를 준비하라"는 신호로 보인다(확정은 MCU 펌웨어 쪽, 아래 Task B) |
| 후처리 | byte-swap → raw(900~3100)→degree(0~180) 선형변환 → id∈{2,3,4}면 `180-pos` 재반전 → `pos==0`이거나 범위 밖이면 `None` |
| 실제 bus servo 에서 읽는 값인지 source만으로 판별 가능? | **불가능** — 이 함수는 addr `0x15`(MCU) 의 레지스터 `0x32`를 읽을 뿐, 그 레지스터값이 MCU가 그 순간 bus servo 에 폴링해서 얻은 **실측값**인지, MCU 가 마지막으로 **명령한 목표값**을 캐시해둔 것인지는 MCU 펌웨어 쪽 로직이다 — Jetson 쪽 소스에는 그 정보가 없다 |
| MCU cached value 가능성 배제 가능? | **배제 불가** |
| 이번 gate 사용 | 5라운드 × id 1-6, 20ms 간격(`readonly_probe.json`) |

## 3. `Arm_serial_set_torque(onoff)` — WRITE, 금지 대상

| 항목 | 값 |
|---|---|
| 위치 | Arm_Lib.py L~337-344 |
| register | `0x1A` — `onoff==1` → `write_byte_data(addr, 0x1A, 0x01)`, 아니면 `0x00` |
| 방향 | write-only |
| 대응하는 READ API | **파일 전체에 존재하지 않음**(아래 Task E) |
| 이번 gate 사용 | 0회 |

## 4. servo ping/status API — `Arm_ping_servo(id)` — READ(trigger-then-read, 재시도 포함), 허용·사용

| 항목 | 값 |
|---|---|
| 위치 | Arm_Lib.py L~230-248 |
| register | `0x38` — `write_byte_data(addr, 0x38, id)` 후 `read_byte_data(addr, 0x38)`, 값이 `0`이면 최대 5회 재시도 |
| 반환값 의미(주석) | "정상 0xda, 못읽으면 0x00, 그 외는 servo error" |
| actuator 상태 변경 가능성 | 레지스터 `0x38`은 position(`0x10-0x16`/`0x30-0x36`)·torque(`0x1A`)·offset(`0x1b/0x1c`)·reset(`0x05`) 어느 것과도 겹치지 않음 — source상 side effect 없음 |
| 이번 gate 사용 | id 1-6 각 1회 |

## 5. version/read-only API — `Arm_get_hardversion()` — READ(trigger-then-read), 허용·사용

| 항목 | 값 |
|---|---|
| 위치 | Arm_Lib.py L~251-259 |
| register | `0x01` — trigger write `0x01` 후 read |
| 반환 | `"0." + str(value)` 형식 버전 문자열 |
| 이번 gate 사용 | 1회 |

## 6. 그 밖의 servo 상태를 읽을 수 있는 API

| API | register | 실행 여부 | 사유 |
|---|---|---|---|
| `Arm_serial_servo_read_any(id)` | `0x37` (트리거+read, "임의 bus servo, id 1-250" 범용 경로) | **실행함** | 위치 변환 공식에 id=2/3/4 반전이 **없다** — `Arm_serial_servo_read`와 다른 코드경로. read-only(actuator 변경 없음) |
| `Arm_serial_servo_write_offset_state()` | `0x1b` (trigger write `0x01` + read) | **실행 안 함(의도적 skip)** | 이름은 "read"처럼 보이나 "一键设置总线舵机中位偏移" 캘리브레이션 서브시스템(`0x1b`/`0x1c`)에 묶인 trigger-write — side effect 가능성을 완전히 배제할 수 없어 지시에 따라 스킵, 사유만 기록 |
| `Arm_Read_Action_Num()` | `0x22` (trigger write `0x01` + read) | **실행 안 함(스코프 밖)** | 저장된 액션그룹 개수 — position/torque 서브시스템과는 무관해 보이나, 이번 ID2 모션 진단과 무관해 범위를 넓히지 않음(지시 §"수정 범위는 아주 작게") |

## 요약 표 — API 분류

| API | read/write | register | 이번 gate 사용 |
|---|---|---|---|
| Arm_serial_servo_write | write | 0x10+id | 금지·0회 |
| Arm_serial_servo_write6 / write6_array / write_any | write | 0x1d/0x1e/0x17/0x19 | 금지·0회 |
| Arm_serial_servo_write_offset_switch | write | 0x1c | 금지·0회 |
| Arm_serial_set_torque | write | 0x1A | 금지·0회 |
| Arm_serial_set_id | write | 0x18 | 금지·0회 |
| Arm_reset | write | 0x05 | 금지·0회 |
| Arm_PWM_servo_write | write | 0x50+id/0x57 | 금지·0회 |
| Arm_RGB_set / Product_Select / Button_Mode / Buzzer_On/Off | write | 0x02/0x04/0x03/0x06 | 금지·0회 |
| Arm_Clear_Action / Action_Study / Action_Mode | write | 0x23/0x24/0x20 | 금지·0회 |
| bus_servo_control / bus_servo_control_array6 | write | 0x10+id/0x1d/0x1e | 금지·0회 |
| **Arm_serial_servo_read** | read(trigger+read) | 0x30+id | 허용·사용(5라운드×6) |
| **Arm_serial_servo_read_any** | read(trigger+read) | 0x37 | 허용·사용(5라운드, id=2) |
| **Arm_ping_servo** | read(trigger+read, retry) | 0x38 | 허용·사용(id 1-6) |
| **Arm_get_hardversion** | read(trigger+read) | 0x01 | 허용·사용(1회) |
| Arm_serial_servo_write_offset_state | read(그러나 trigger write 동반) | 0x1b | 의도적 skip |
| Arm_Read_Action_Num | read(그러나 trigger write 동반) | 0x22 | 의도적 skip(스코프 밖) |
