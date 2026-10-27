# 단일 서보 쓰기 경로 추적 — P-007B가 쓴 경로

`Arm_Lib.py` sha256 `edfcae72…` 기준. 모든 줄 번호는 그 파일이다.

## ID2 → 바이트까지

```
Arm_serial_servo_write(id=2, angle=152, time=2000)        15행
  │
  ├─ id == 0 ?                                            16행  → 아니다 (0이면 write6 일괄 경로)
  ├─ id in (2,3,4) ?                                       18행  → 예  (주석 与实际相反角度 = 실제와 반대 각도)
  │     angle = 180 - angle = 28                           19행
  ├─ pos = int((3100-900) * 28 / 180 + 900) = 1242         20행
  ├─ value_H = (1242 >> 8) & 0xFF = 0x04
  │  value_L = 1242 & 0xFF        = 0xDA
  │  time_H  = (2000 >> 8) & 0xFF = 0x07
  │  time_L  = 2000 & 0xFF        = 0xD0
  └─ bus.write_i2c_block_data(0x15, 0x10 + 2, [0x04, 0xDA, 0x07, 0xD0])   27행
                                   └─ register 0x12
     except: print('Arm_serial_servo_write I2C error')     28~29행  ← 예외를 삼킨다
```

반환값이 없다(함수에 `return`이 없다). 성공 여부는 readback으로만 확인할 수 있다.

## 이 경로가 벤더 표준인가 — 예다

`Arm_serial_servo_write(` 사용처를 `/home/jetson/dofbot_pro`와 `dofbot_pro_ws/src`에서 전수 조사했다. 벤더 데모가 이 함수를 직접 쓴다.

```
dofbot_garbage_yolov11/garbage_grap.py:35    self.arm.Arm_serial_servo_write(6, 180, 100)
dofbot_garbage_yolov11/garbage_grap.py:52    self.arm.Arm_serial_servo_write(1, joints_down[0], 1000)
dofbot_garbage_yolov11/single_garbage_identify.py:117  self.arm.Arm_serial_servo_write(2, 90, 1000)
dofbot_garbage_yolov11/speech_garbage_identify.py:320  self.arm.Arm_serial_servo_write(2, 90, 1000)
dofbot_garbage_yolov11/speech_garbage.py:306 self.arm.Arm_serial_servo_write(1, joints_down[0], 500)
```

`bus_servo_control`(372행)도 **같은 register `0x10+id`** 를 쓴다. 입력만 degree 대신 raw counts다.

→ 따라서 "single-servo 경로가 비표준이라 거절됐다"는 설명은 **벤더 사용 근거로 지지되지 않는다.** 스펙 §26의 DIAGNOSIS C 조건(다른 경로가 표준이고 single 경로에 제약)이 충족되지 않는다.

## 명령 전 초기화가 있는가 — 없다

위 사용처들의 명령 직전 코드를 확인했다. `Arm_serial_set_torque` 호출도, 보드 enable 호출도 선행하지 않는다. `Arm_Device()` 생성(9~12행)은 `smbus.SMBus(7)` 열기와 `self.addr = 0x15` 설정뿐이다. 패키지 `__init__.py`는 `from .Arm_Lib import Arm_Device` 한 줄로 import 부작용이 없다.

## 🔴 트랜잭션 유형 차이 — 이번 게이트의 가장 구체적인 발견

| 트랜잭션 | 사용 함수 | 이 세션에서 동작했는가 |
| --- | --- | --- |
| `write_byte_data` 1바이트 + `read_word_data` | `Arm_serial_servo_read` (0x30+id) | ✅ 1,290+회 성공 |
| `write_byte_data` 1바이트 + `read_byte_data` | `Arm_ping_servo` (0x38), `Arm_get_hardversion` (0x01) | ✅ 7회 성공 |
| `write_i2c_block_data` **4바이트** | `Arm_serial_servo_write` (0x10+id) | ❓ 1회 호출, 오류 없음, **효과 없음** |
| `write_i2c_block_data` 12바이트 | `write6` 계열 (0x1D) | 호출하지 않았다 |

즉 **이 세션에서 검증된 것은 바이트/워드 트랜잭션뿐이고, block write가 컨트롤러에 의미 있게 전달되는지는 한 번도 확인된 적이 없다.** 

한계: SMBus 쓰기는 커널 드라이버가 ACK를 받으면 예외를 내지 않는다. 따라서 "예외 없음"은 보드가 바이트를 ACK했다는 뜻까지이고, 펌웨어가 그 payload를 수락했다는 뜻은 아니다. 이 구분을 소스로 더 좁힐 수는 없다.
