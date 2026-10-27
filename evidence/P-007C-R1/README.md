# P-007C-R1 — Yahboom 공식 소스·런타임 provenance 교차검증 (진행 중 — 사용 한도로 중단)

**ROBOT MOTION = 0 · CONTROLLER STATE MUTATION = 0 · 하드웨어 쓰기 0.** 이 게이트는 읽기와 문서 분석만 했다.

> ⚠️ 이 문서는 **완료 보고서가 아니다.** 세션 사용 한도에 걸려 증거 파일 일부(`acceptance.json`, 개별 분석 md)와 완료 보고서를 쓰지 못했다. 아래는 **이미 측정·확인된 사실**이며 재현에 필요한 핵심은 모두 들어 있다.

## 1. 공식 저장소 pin

| 항목 | 값 |
| --- | --- |
| repo | `YahboomTechnology/DOFBOT-Pro` (default `main`, 497.4 MB, license 없음 = 전권 보유) |
| commit | **`57cc4369af73d65188d33e49227650c789779a7e`** (2026-08-26T08:09:05Z) |
| tree | `8474218c675e73c0dd19a331205db10be6318367` |
| 트리 | 1,056 항목, `truncated: false` |

🔴 **확장자 분포: `.pdf` 765 · `.txt` 16 · `.db` 8 · `.yml` 5 · `.xlsx` 2 · `.jpg` 1 · `.md` 1.** `.py` **0개**, `Arm_Lib` **0건**.

→ **공식 저장소로는 byte-level 코드 provenance 비교가 불가능하다.** 저장소는 코스 PDF만 담는다. 공식 코스 README(`17. Basic control course/README.txt`)는 드라이버가 factory image에 설치돼 있고 경로가 `~/software/Arm_Lib`라고 적는다. 코드 배포 채널은 factory image와 `Annex_Download_Link.txt`의 Google Drive다.

저작권 때문에 **PDF를 이 저장소에 커밋하지 않았다.** 스크래치패드에만 두고 pin URL·blob SHA·짧은 인용만 남긴다.

## 2. 🔴 실행 체인 검증 — 네 게이트의 분석 기반이 유지된다

P-007B·C는 디스크의 `Arm_Lib.py`를 분석했다. 그런데 파이썬이 실제로 import하는 것은 그 파일이 아니었다.

```
Arm_Lib.__file__      : /usr/local/lib/python3.10/dist-packages/Arm_Lib-1.0.3-py3.10.egg/Arm_Lib/__init__.py
Arm_Device 정의 파일  : .../Arm_Lib-1.0.3-py3.10.egg/Arm_Lib/Arm_Lib.py
NotADirectoryError    : .egg 가 디렉터리가 아니라 zip 아카이브라는 증거
```

확인 결과 **문제 없었다.**

| 측정 | 결과 |
| --- | --- |
| 시스템 내 `Arm_Lib.py` 전수 (6개) | 전부 sha256 `edfcae72…`, 17739 bytes. `~/software/Arm_Lib`(공식 factory 경로)·`dofbot_pro/Arm_Lib`·`colcon_ws/src/Arm_Lib` + 각 `build/lib` |
| egg | zip, 9887 bytes, sha256 `3aba35832587428b83a231a94856fe89e36f93507c911fcf39c1b8ad27f13371` |
| egg 내 `Arm_Lib/Arm_Lib.py` | **`edfcae72…` — 디스크 사본과 바이트 동일** |
| egg 내 `.pyc` | magic `6f0d0d0a` 런타임 일치, 기록된 src size 17739 일치, **code object digest가 `.py` 컴파일 결과와 동일** |
| 런타임 시그니처 | `Arm_serial_servo_write(self, id, angle, time)` · `Arm_serial_servo_read(self, id)` |

→ **P-007B·C에서 실행된 바이트코드는 우리가 정적 분석한 소스 그 자체다.** provenance 공백이 닫혔다.

## 3. 🔴 공식 토크 정의 — root-cause candidate

`23.For_JetsonORIN_SUPER_JetPack6.2/2. Assembly course/2. Calibrating the Robotic Arm/Calibrating the Robotic Arm.pdf` (blob 일치 확인) 원문:

> **enable=0 turns off the torque, the robot arm can manually turn the servo angle, and does not receive the level signal; enable=1 turns on the torque, the servo receives the level signal to maintain the current angle, and the angle can be changed only after sending a command.**

→ 제조사가 **토크 OFF 상태에서는 서보가 신호를 받지 않는다**고 명시한다. 이것은 P-007B의 `NO_MOTION_RESPONSE`를 설명할 수 있는 **공식 메커니즘**이다. I2C 수준에서 명령이 ACK돼도 서보가 무시하는 상태가 공식적으로 존재한다.

단 **확정이 아니다.** 토크 상태를 읽는 경로가 없고(P-007C `NO_TORQUE_READ_PATH`), 공식 문서에도 boot default 토크 상태 설명이 없다. 운영자 관찰은 "팔이 단단히 유지 중으로 보인다"로 오히려 토크 ON 쪽이다.

같은 문서가 `Arm_serial_servo_write_offset_switch(id)`(id=0은 전체 초기화)와 `Arm_serial_servo_write_offset_state()`(0=미검출, 1=성공, 2=범위초과) semantics도 정의하며, 우리 정적 분석(`0x1C`/`0x1B`)과 일치한다.

## 4. 공식 single-servo 예제 vs P-007B

`.../17. Basic Control Course/3. Control a single servo/Control a single servo.pdf`, 코드 경로 `~/dofbot_pro/dofbot_ctrl/scripts/03.ctrl_servo.ipynb`:

```python
import time
from Arm_Lib import Arm_Device
Arm = Arm_Device()
time.sleep(.1)                              # ← 생성 직후 100ms
id = 6
Arm.Arm_serial_servo_write(id, 90, 500)
time.sleep(1)
```

공식 API 설명: `id` 1~6(아래가 1, 위로 증가) · `angle` ID5만 0~270, 나머지 0~180 · `time` 작을수록 빠르고 **0이면 최고속** · **Return value: None**.

| 항목 | 공식 예제 | P-007B | 판정 |
| --- | --- | --- | --- |
| constructor | `Arm_Device()` | 같음 | Same |
| **생성 후 지연** | **`time.sleep(.1)`** | 없음. 생성 → 해시·AST → read → write (수~수십 ms, 정밀 미기록) | **Different** |
| 초기화(torque·mode) | **없음** | 없음 | Same |
| API | `Arm_serial_servo_write` | 같음 | Same |
| 인자 형식 | `(id, angle, time_ms)` | `(2, 152, 2000)` | Same |
| 예제 대상 ID | 6 (그리퍼) | 2 (Arm2) | 다르지만 공식 범위 안 |
| 명령 후 지연 | `time.sleep(1)` | 2000ms + 1.5s | Same 성격 |

→ **P-007B의 명령 경로는 공식 경로와 실질적으로 동일하다.** API 오사용 가설은 공식 근거로도 약해진다. 유일한 실질 차이는 **생성 직후 100ms 지연**이고, 이것이 원인이라는 증거는 없다(우리 읽기는 같은 조건에서 1,290+회 성공했다).

## 5. 공식 read semantics

`.../17. Basic Control Course/4. Read the current position of the servo` — "Return value: The current angle of the corresponding ID servo. When ID=5, the angle range is 0~270, and the others are 0~180." **commanded / feedback 구분이 없다.**

→ **`CONTROLLER_REPORTED_STATE` 유지.** encoder feedback으로 승격하지 않는다.

흥미로운 교차확인: 같은 문서의 공식 실행 화면이 `90 90 90 90 89 178`을 출력한다. 네 번째까지 90인데 **ID5가 89**다. P-007B가 소스에서 유도한 **`int()` 절단 ±1 편향**과 같은 성격의 현상이 **공식 스크린샷에도 나타난다.**

## 6. 이 게이트가 아직 못 한 것

- `8. Robotic arm memory action` PDF(Button_Mode·학습 모드) — 다운로드·blob 검증은 끝났고 **읽지 못했다**
- `acceptance.json`(R1-01~29), 개별 분석 md 11종, `hypothesis_update.json`, 완료 보고서
- Google Drive annex 접근 시도 1회
- transport 세대 분리(§11) — Orin JetPack 6.2 자료가 이 I2C 보드를 기술하는지 확인

## 7. 잠정 판정 (증거로 뒷받침되나 게이트는 미완)

**`DIAGNOSIS PASS — OFFICIAL ROOT-CAUSE CANDIDATE IDENTIFIED`** 로 수렴하는 중이다.

- 공식 문서가 **토크 OFF = 신호 미수신**을 명시 → H-C2가 공식 근거를 얻은 **유일한 후보**
- H-C3(초기화 누락)는 공식 예제에도 초기화가 없어 **재확인 NOT_SUPPORTED**
- H-C4(경로 비표준)는 공식 예제가 같은 API·같은 인자 형식 → **재확인 NOT_SUPPORTED**
- 설치본 provenance: 공식 저장소에 코드가 없으므로 P2 기준 `NO_OFFICIAL_MATCH_FOUND`. 다만 공식 코스가 적은 factory 경로 `~/software/Arm_Lib`에 동일 바이트가 존재하는 것은 **보조 정황**이다(무수정 증명은 아니다)

## 8. 이월 위험 (변함없음)

토크를 켜는 순간 P-007B가 보낸 target(ID2, raw 1242)으로 움직일 수 있다. `Arm_serial_set_torque(1)`은 **모션 가능 행위**로 취급해야 한다.

## STOP

토크를 켜지 않았다. 서보 명령 0. P-007D를 열지 않았다. 공식 PDF는 저작권 때문에 커밋하지 않았다.
