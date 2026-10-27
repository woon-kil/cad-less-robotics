# controller firmware / hardware version (스펙 §16)

자료: `official/nt-01-7-gripping-anomaly-solutions.pdf` (`NewTutorials/1. Get Started Quickly/7.`, blob `3e7eb9f7…`, 11페이지 전수 판독)

1차 세션 인벤토리는 `NewTutorials/` 트리를 놓쳤다. 이 문서는 공식 자료 전체에서 **보드 정체와 펌웨어 채널을 규정하는 유일한 문서**다.

## 공식이 규정하는 하드웨어

| 항목 | 공식 값 |
| --- | --- |
| 보드 공식 명칭 | **EVV01 Robotic Arm Servo Control Board** (문서 내 폴더명 `YB-EVV01测试流程及固件（V1.0）`) |
| MCU | **STM8_32K** (ST FlashLoader Demonstrator 2.8.0 의 target 선택값) |
| 펌웨어 아티팩트 | **`arm_slave_Vxxxx.hex`** — 문서 스크린샷의 실제 파일은 `arm_slave_V2.1.hex`, 15.71 KB (16,084 bytes), 파일 날짜 2026-06-22 |
| 플래싱 경로 | MicroUSB → UART 115200/Even/Echo, 점퍼캡 2개를 Upload 측으로, K2(STOP) 15초 길게 눌러 programming mode (RGB 러닝라이트) |
| 보드 조작부 | K1 버튼, K2(STOP) 버튼, RESET 버튼, 점퍼캡 2개(Upload/Servo), 메인보드 RGB LED |

## 우리 측 측정값과의 대조

P-007C 실측: `Arm_get_hardversion()` → `"0.20"` (register `0x01`, `write_byte_data(0x01) → sleep 1ms → read_byte_data`).

P-007C `hardware_version_analysis.md` 가 이 함수의 소스 257~267행을 그대로 캡처해 뒀다:

```python
value = self.bus.read_byte_data(self.addr, 0x01)
...
version = str(0) + '.' + str(value)
```

`str(value)` 는 int 의 **10진 표현**이다. 즉 major `0` 은 라이브러리가 하드코딩한 것이고, 장치가 보낸 것은 1바이트뿐이다.

→ **보드가 보낸 원시값은 10진수 `20`.** 표시 문자열 `"0.20"` 의 `0.` 은 장치에서 온 것이 아니다. (16진 표기 가능성은 소스로 배제된다 — `str()` 는 hex 포맷을 하지 않는다.)

| 비교 | 결과 |
| --- | --- |
| 공식이 `0.20` 이라는 값을 어딘가에 기재하는가 | **NO** — 14종 어디에도 없다 |
| 공식이 버전 매핑표를 제공하는가 | **NO** |
| 원시값 `20` ↔ 공식 `V2.1` | **매핑 미확인** |

## 판정 — `VERSION_MAPPING_UNKNOWN` (단, 1차 세션보다 좁혀졌다)

확인된 것:

1. 우리 보드는 **EVV01 / STM8_32K** 계열이고, 펌웨어는 `arm_slave_Vx.y.hex` 로 관리된다.
2. 표시값 `"0.20"` 은 **라이브러리 포맷 산물**이다. 버전 문자열로 그대로 인용하면 안 된다. 원시값은 `20`.
3. 공식 최신 펌웨어로 문서에 등장하는 것은 `V2.1`(2026-06-22).

확인되지 않은 것:

- 원시 `20` 이 `V2.0` 을 뜻하는지. `major*10+minor` 인코딩이라면 `20 → V2.0` 이고 공식 최신 `V2.1` 보다 한 단계 낮다는 뜻이 되지만, **인코딩 규칙에 대한 공식·소스 근거가 없다.** 다른 해석(일련번호·하드웨어 리비전)도 배제되지 않는다. 함수 이름이 `hardversion`(하드웨어)이고 플래싱 대상은 펌웨어라는 점도 동일 축인지 불분명하게 만든다.
- 따라서 **"우리 펌웨어가 구버전이다"라고 주장하지 않는다.** 가설로만 기록한다.

## 관련 공식 troubleshooting

`official/01-2-common-problems.pdf` Q3:

> The servo behaves abnormally, but can be controlled separately?
> Answer: **Turn off the power of the device, check whether the power supply is normal, and then power it on again.**

공식이 서보 이상 거동에 제시하는 1차 조치가 **전원 재투입**이다. H-C2(torque OFF)·H-C6(learning mode)은 둘 다 **보드에 상주하는 상태**이고 전원 재투입으로 해소되는 종류다. 공식 조치가 두 가설의 성질과 일관된다 — 다만 이것은 정황이고, 공식 Q3 은 우리 증상("개별 제어는 되는데 거동 이상")과 **문면이 일치하지 않는다**(우리는 개별 제어가 안 됐다). 근거로 과대 해석하지 않는다.

`nt-01-7` 자체는 "calibrate 를 다 했는데도 **꽉 집지 못하는**" 증상용이다. 우리 증상(위치 명령 무반응)과 다르다. 펌웨어 업그레이드를 이 게이트의 권고로 올리지 않는다 — 그것은 비가역에 가까운 보드 조작이다.
