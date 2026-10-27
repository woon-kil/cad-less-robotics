# 명령 API 정적 분석 — `Arm_serial_servo_write` / `Arm_serial_servo_read`

- 소스: `/home/jetson/dofbot_pro/Arm_Lib/Arm_Lib/Arm_Lib.py`
- sha256 `edfcae726658ce9340208de03d836aabfc5ebc1754d14ce8720a7ba36e13ed5c` (P-007A·R1과 동일, `source_integrity.json`)
- 장치 주소 `self.addr = 0x15` (10행), 기본 버스 `i2c_bus=7` (9행)
- 모든 줄 번호는 이 파일 기준이다. 추측 없음.

## `Arm_serial_servo_write(self, id, angle, time)` — 15행

```
15:    def Arm_serial_servo_write(self, id, angle, time):
16:        if id == 0:  # 此为所有舵机控制
18:        elif id == 2 or id == 3 or id == 4:  # 与实际相反角度
19:            angle = 180 - angle
27:                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
29:                print('Arm_serial_servo_write I2C error')
```

| 항목 | 값 | 근거 |
| --- | --- | --- |
| 🔴 `id == 0` | **전 서보 일괄 제어** — `Arm_serial_servo_write6(angle,…,angle, time)` 호출 | 16~17행 |
| ID2 angle 변환 | `angle = 180 - angle` (ID 2·3·4 공통, 주석 `与实际相反角度` = 실제와 반대 각도) | 18~19행 |
| pos 변환 | `pos = int((3100 - 900) * angle / 180 + 900)` → 900~3100 counts | 20행 |
| 1 library degree | `2200 / 180 = 12.22` counts | 위 식에서 유도 |
| register | `0x10 + id` → **ID2는 `0x12`** | 27행 |
| 전송 바이트 | `[value_H, value_L, time_H, time_L]` 4바이트 block write | 27행 |
| `time` semantics | 2바이트로 쪼개 그대로 전송 (`time_H`, `time_L`). 단위는 ms이고 컨트롤러가 그 시간에 걸쳐 보간한다. 범위 0~65535 | 24~25, 27행 + 벤더 호출 선례 |
| blocking | **non-blocking**. block write 후 `sleep` 없이 즉시 반환 | 27~28행 |
| 반환값 | **없음(None)** | 함수 전체에 `return` 없음 |
| 🔴 예외 처리 | `except: print('Arm_serial_servo_write I2C error')` — **예외를 삼킨다**. I2C 실패가 예외로 올라오지 않는다 | 28~29행 |
| angle 범위 검사 | **없다**. `write6_array`(101행대)만 `> 180` 검사를 한다. 단일 쓰기는 검사 없이 변환한다 | 15~52행 |
| 숨은 다중 관절 | `id != 0`이면 없다. `id == 0`만 6축 일괄 | 16~17행 |

**이 분석에서 나오는 실행 규칙**

1. 리터럴 `2`를 넘기고 호출 직전에 `assert id == 2`를 둔다. `id = 0`은 6축 일괄 경로다.
2. I2C 실패는 예외가 아니라 **stdout 문자열**로 나온다. 호출 stdout을 캡처해 `I2C error` 포함 여부로 판정한다.
3. 반환값이 없으므로 성공은 readback으로만 확인한다.

## `Arm_serial_servo_read(self, id)` — 189행

```
189:    def Arm_serial_servo_read(self, id):
194:            self.bus.write_byte_data(self.addr, id + 0x30, 0x0)
196:            pos = self.bus.read_word_data(self.addr, id + 0x30)
212:        if id == 2 or id == 3 or id == 4:
213:            pos = 180 - pos
```

| 항목 | 값 | 근거 |
| --- | --- | --- |
| id 검사 | `id < 1 or id > 6` → `None` | 190~192행 |
| register | `id + 0x30` → **ID2는 `0x32`** | 194, 196행 |
| 절차 | `write_byte_data(addr, 0x32, 0x0)` → `sleep(0.003)` → `read_word_data(addr, 0x32)` → 바이트 스왑 | 194~201행 |
| deg 변환 | `pos = int(180 * (raw - 900) / 2200)` | 207행대 |
| 범위 검사 | 변환 결과가 0~180 밖이면 `None` | 208행대 |
| ID2 역변환 | `pos = 180 - pos` (ID 2·3·4 공통) | 212~213행 |

## 🔴 write와 read의 ID2 변환은 서로 상쇄되지만 `int()` 절단이 +1 편향을 만든다

반전이 **양쪽에** 있으므로 라이브러리 각도 공간은 일관된다. 그러나 두 함수 모두 `int()`로 **버림**을 하기 때문에 명령값과 readback이 정확히 같지 않다.

실측 계산(위 식을 그대로 코드로 옮겨 계산):

```
  cmd 148 → raw 1291 → read 149
  cmd 149 → raw 1278 → read 150
  cmd 150 → raw 1266 → read 151
  cmd 151 → raw 1254 → read 152
  cmd 152 → raw 1242 → read 153
  cmd 153 → raw 1230 → read 153

현재 readback 150 을 만드는 raw 범위: 1267..1278 (12개)
readback 152 를 만드는 raw 범위:      1243..1254
1 library degree = raw 12.22 counts
```

**따라서 ID2에 152를 명령했을 때 예상 readback은 152가 아니라 `153`이다.** 서보가 명령 raw 1242에 정확히 안착하면 153으로 읽힌다. 1243~1254 구간에 머물면 152로 읽힌다.

이 사실을 모르고 "target 152인데 153이 나왔다"를 command semantics 불일치로 보면 스펙 §31에 따라 FAIL이 된다. 그래서 **예상 readback을 `152 또는 153`(명령 raw에 정확히 안착 시 153)으로 홀드 포인트에 고정**한다.

**실제 서보 이동량**: 현재 readback 150은 raw 1267~1278을 뜻한다. 명령 raw는 1242다. 차이는 25~36 counts = **2.0°~2.9°**(라이브러리 스케일). ID2는 반전 때문에 raw가 줄어드는 방향이 라이브러리 각도가 커지는 방향이다.

## duration 선택

| 항목 | 값 |
| --- | --- |
| 벤더 선례 | `YahboomArm.py:365` → `Arm.Arm_serial_servo_write(id, angle, 500)` — 임의 단일 관절 이동에 500ms |
| 이번 게이트 | **2000 ms** |
| 근거 | 벤더 선례보다 4배 느리다. 2° 이동에 2000ms면 평균 약 1°/s로, 첫 모션에서 급가속을 피하고 운영자가 방향을 눈으로 따라갈 시간을 준다. `time`은 2바이트라 2000은 범위 안이다(0~65535). |
| 자동 조정 | 하지 않는다. 한 번 고정하고 끝낸다 (스펙 §17) |

## 사용하지 않는 API (스펙 §8)

`Arm_serial_servo_write6`(145행) · `Arm_serial_servo_write6_array`(100행) · `Arm_serial_servo_write_any`(54행) · `Arm_serial_servo_write_offset_switch`(78행) · `Arm_serial_set_torque`(270행) · `Arm_reset`(308행) · `Arm_PWM_servo_write`(315행) · `move_init_pose`(벤더 `Robot_Controller`) · gripper(ID6) 명령.
