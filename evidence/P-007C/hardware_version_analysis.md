# `Arm_get_hardversion` 정적 분석

## 소스 (257~267행)

```python
# 读取硬件版本号                                                      256행
def Arm_get_hardversion(self):
    try:
        self.bus.write_byte_data(self.addr, 0x01, 0x01)   # ← 요청 바이트 쓰기
        time.sleep(.001)
        value = self.bus.read_byte_data(self.addr, 0x01)
    except:
        print('Arm_get_hardversion I2C error')
        return None
    version = str(0) + '.' + str(value)
    return version
```

| 항목 | 값 |
| --- | --- |
| register | `0x01` |
| bus 연산 | `write_byte_data(0x15, 0x01, 0x01)` → `sleep 1ms` → `read_byte_data(0x15, 0x01)` |
| 부작용 | 소스 상 없다. 요청 바이트를 쓰고 같은 register를 읽는다 |
| 예외 처리 | `try/except` 있음. 실패 시 메시지 출력 후 `None` |
| 반환 의미 | `'0.' + str(value)` 문자열. 즉 major는 항상 0으로 하드코딩되고 minor만 보드에서 온다 |
| level | **CONTROLLER_BOARD** (서보가 아니라 보드) |

## 승인 근거

`0x01`은 `Arm_Lib.py` 전체에서 이 함수만 쓴다. 인접 register는 RGB `0x02`, Button_Mode `0x03`, Product_Select `0x04`, reset `0x05`, buzzer `0x06`이고 겹치지 않는다. 같은 펌웨어 한계가 적용된다 — `proven safe`가 아니라 "register 비공유"까지다.

## 실측 결과

```
Arm_get_hardversion()  reg 0x01  value='0.20'  0.002s  예외 없음
```

→ 보드가 `0x01` 왕복에 응답한다. `0x30+id` 읽기 경로와 **독립적인 두 번째 경로**에서도 컨트롤러 통신이 정상이다.

벤더 코드 사용처는 0건이다(라이브러리에만 존재).
