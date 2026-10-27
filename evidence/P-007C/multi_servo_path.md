# 다중 서보 경로 비교 (정적 — 실행하지 않았다)

## `Arm_serial_servo_write6_array` (100행) / `Arm_serial_servo_write6` (145행)

```
범위 검사: s1~s4,s6 > 180 또는 s5 > 270 이면 메시지 출력 후 return    101~103행
s2 = 180 - s2 ;  s3 = 180 - s3 ;  s4 = 180 - s4      (단일 경로와 같은 반전)
각 축 pos = int(2200 * s / 180 + 900),  s5만 int(3320 * s5 / 270 + 380)
timeArr = [time_H, time_L]
bus.write_i2c_block_data(0x15, 0x1E, timeArr)          ← time 을 먼저 따로 보낸다
bus.write_i2c_block_data(0x15, 0x1D, data)             ← 12바이트 위치 payload
```

단일 경로와의 차이:

| 항목 | 단일 `0x10+id` | 6축 `0x1D` + `0x1E` |
| --- | --- | --- |
| register | 1개 | 2개 (time 먼저, 위치 나중) |
| payload | 4바이트 | 2바이트 + 12바이트 |
| trigger | 위치 쓰기 자체 | 위치 쓰기가 마지막 |
| 범위 검사 | **없다** | 있다 |
| timing | time 이 위치와 같은 block 안 | time 이 **선행 트랜잭션** |

이 구조 차이는 "6축 경로는 time을 별도 register에 먼저 적재한 뒤 위치 payload로 실행을 트리거한다"로 읽힌다. 그러나 **펌웨어가 실제로 그렇게 동작하는지는 소스로 확인할 수 없다 → UNKNOWN.**

## `Arm_serial_servo_write_any` (54행)

```
id != 0 : bus.write_i2c_block_data(0x15, 0x19, [id, posH, posL, timeH, timeL])
id == 0 : bus.write_i2c_block_data(0x15, 0x17, [posH, posL, timeH, timeL])
```

주석(53행)은 `id: 1-250(0是群发) angle: 0-180 表示900 3100`. 즉 임의 버스 서보 ID를 대상으로 하는 경로이고, **ID 2·3·4 반전이 없다.** 그래서 같은 degree를 넣어도 `Arm_serial_servo_write`와 다른 물리 방향이 된다.

벤더 사용처: `/home/jetson/dofbot_pro`·`dofbot_pro_ws/src`에서 `Arm_serial_servo_write_any` 호출 0건. 라이브러리에만 있다.

## `bus_servo_control` (372행) / `bus_servo_control_array6` (416행)

raw counts 입력판이다. `bus_servo_control`은 **단일 경로와 같은 `0x10+id`** 를 쓰고, array6은 `0x1E`+`0x1D`를 쓴다. id 2·3·4 반전은 `3100 - num + 900`으로 구현된다(392행, `__change_value` 409행). 범위 검사가 있다(900~3100, id5는 900~4200).

## 실행 금지 확인

이 게이트에서 위 함수를 **단 한 번도 호출하지 않았다**(`environment.json`의 `write6_calls`, `write_any_calls`, `bus_servo_control_calls` 모두 0).
