# `Arm_ping_servo` 정적 분석 — 이름이 ping이라고 안전하다고 가정하지 않았다

## 소스 (237~254행)

```python
# 读取舵机状态，正常返回0xda, 读不到数据返回0x00，其他值为舵机错误        236행
def Arm_ping_servo(self, id):
    data = int(id)
    if data > 0 and data <= 250:
        reg = 0x38
        self.bus.write_byte_data(self.addr, reg, data)      # ← 쓰기가 선행한다
        time.sleep(.003)
        value = self.bus.read_byte_data(self.addr, reg)
        times = 0
        while value == 0 and times < 5:                     # 0이면 최대 5회 재시도
            self.bus.write_byte_data(self.addr, reg, data)
            time.sleep(.003)
            value = self.bus.read_byte_data(self.addr, reg)
            times += 1
            if times >= 5:
                return None
        return value
    else:
        return None
```

| 항목 | 값 |
| --- | --- |
| register | `0x38` |
| write-before-read | **예.** 조회 대상 servo ID를 선택 바이트로 쓴다 |
| 재시도 | `value == 0`이면 최대 5회. 그래도 0이면 `None` |
| 🔴 예외 처리 | **try/except가 없다.** 다른 함수와 달리 I2C 예외가 그대로 올라온다 |
| controller 조회인가 servo 조회인가 | **servo 조회.** ID를 지정해 그 서보의 상태를 받는다 |
| 반환 의미 (벤더 주석 236행) | `0xda` 정상 · `0x00` 데이터 못 읽음 · 그 외 값은 서보 에러 |
| state 변경 | 소스 상 없다. 선택 바이트 쓰기 + 상태 바이트 읽기 |
| ID 범위 | 함수는 1~250을 허용. **이 게이트는 문서 범위 1~6만 조회했다** (스펙 §17 sweep 금지) |

## 승인 근거와 그 한계

- `0x38`은 `Arm_Lib.py` 전체에서 이 함수만 쓴다. 위치(`0x10+id`, `0x1D`/`0x1E`, `0x19`/`0x17`)·torque(`0x1A`)·offset(`0x1B`/`0x1C`)·읽기(`0x30+id`, `0x37`)와 겹치지 않는다.
- 🔴 **벤더 코드에 사용처가 0건이다.** 라이브러리에만 있다. 따라서 "벤더가 실제로 쓰는 안전한 조회"라는 근거는 없다.
- 🔴 파이썬 정적 분석으로는 펌웨어가 `0x38` 쓰기를 어떻게 처리하는지 알 수 없다. 쓸 수 있는 가장 강한 표현은 **"벤더가 이 register를 이 조회에만 쓰고 어떤 쓰기 경로도 공유하지 않는다"** 까지다. `proven safe`가 아니다.

## 실측 결과

```
Arm_ping_servo(1)  value=218 (0xda)  NORMAL_0xDA  0.004s
Arm_ping_servo(2)  value=218 (0xda)  NORMAL_0xDA  0.004s
Arm_ping_servo(3)  value=218 (0xda)  NORMAL_0xDA  0.004s
Arm_ping_servo(4)  value=218 (0xda)  NORMAL_0xDA  0.004s
Arm_ping_servo(5)  value=218 (0xda)  NORMAL_0xDA  0.004s
Arm_ping_servo(6)  value=218 (0xda)  NORMAL_0xDA  0.004s
```

예외 0건, 재시도 발생 없음(모두 1회에 0xda), 조회 전후 ID1~6 각 5라운드 읽기가 모두 `[86,150,0,18,93,178]`로 동일.

→ **컨트롤러가 6개 서보 전부와 통신되고 ID2도 정상이다.** 가설 H-C1(통신)은 제거된다.
