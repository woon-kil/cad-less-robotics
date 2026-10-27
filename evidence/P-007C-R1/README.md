# P-007C-R1 — Yahboom 공식 소스·런타임 provenance 교차검증

**판정: `DIAGNOSIS PASS — NO OFFICIAL EXPLANATION FOUND`**

**ROBOT MOTION = 0 · CONTROLLER STATE MUTATION = 0 · 하드웨어 접근 0.** 이 게이트는 ssh·I2C·`Arm_Lib` import 를 한 번도 하지 않았다. 문서와 저장소만 읽었다.

> 이 문서는 1차 세션(사용 한도로 중단)의 중간 기록을 대체한다. 철회한 서술은 §9 에 적었다.

## 1. 공식 저장소 pin

| 항목 | 값 |
| --- | --- |
| repo | `YahboomTechnology/DOFBOT-Pro` (default `main`, 497.4 MB, license 없음) |
| commit | **`57cc4369af73d65188d33e49227650c789779a7e`** (2026-08-26T08:09:05Z) |
| tree | `8474218c675e73c0dd19a331205db10be6318367` — 1,056 항목, `truncated: false` |

확장자 분포: `.pdf` 765 · `.txt` 16 · `.db` 8 · `.yml` 5 · `.xlsx` 2 · `.jpg` 1 · `.md` 1. **`.py` 0개, `Arm_Lib` 0건.**

→ 공식 저장소는 **코스 PDF 배포소**다. byte-level 코드 provenance 비교가 이 채널로는 불가능하다. 공식 배포 채널은 factory image(`~/software/Arm_Lib`)와 Google Drive annex다.

공식 PDF 14종을 blob SHA 로 받아 `git hash-object` 로 14/14 일치를 확인하고 `official/` 에 두었다. 저작권 때문에 **커밋하지 않는다**(`.gitignore`). 재현 지시서는 `official_materials_manifest.json` 이다.

## 2. 실행 체인 검증 — 네 게이트의 분석 기반이 유지된다

P-007B·C 는 디스크의 `Arm_Lib.py` 를 분석했다. 그런데 파이썬이 실제 import 하는 것은 `.egg`(zip) 안의 사본이었다.

| 측정 | 결과 |
| --- | --- |
| 시스템 내 `Arm_Lib.py` 전수 (6개) | 전부 sha256 `edfcae72…`, 17739 bytes |
| egg | zip, 9887 bytes, sha256 `3aba3583…` |
| egg 내 `Arm_Lib/Arm_Lib.py` | **`edfcae72…` — 디스크 사본과 바이트 동일** |
| egg 내 `.pyc` | magic 런타임 일치, src size 17739 일치, code object digest 동일 |

→ **P-007B·C 에서 실행된 바이트코드는 우리가 정적 분석한 소스 그 자체다.** provenance 공백이 닫혔다.

## 3. 공식 자료가 기술하는 것과 기술하지 않는 것

공식 PDF 는 **Python API 층에서 끝난다.** 14종 전수 판독 결과 register·`smbus`·`write_i2c_block_data`·버스 번호·slave 주소 `0x15` 가 **0회** 등장한다. 유일한 I2C 전용 코스(`16. GPIO/6. I2C communication`)도 GPIO 핀에 OLED(`0x3C`)를 붙이는 범용 실습이다.

| 스펙 항목 | 결과 |
| --- | --- |
| §11 register 9개 | **전부 `NOT_DOCUMENTED_IN_OFFICIAL`**. API 층에서 비교 가능한 6개는 모순 0 |
| §12 block-write | **`NOT_DOCUMENTED_IN_OFFICIAL`**. 설치본으로 공식을 유도하는 순환논증 회피 |
| §16 firmware | 보드 = **EVV01 / STM8_32K**, 펌웨어 = **`arm_slave_Vx.y.hex`**(문서 내 실제 파일 `V2.1`). `Arm_get_hardversion()` 의 `"0.20"` 은 `str(0)+'.'+str(value)` 라는 **라이브러리 포맷 산물**이고 원시값은 10진 `20`. 매핑은 `VERSION_MAPPING_UNKNOWN` |
| §17 ping | **`NOT_DOCUMENTED_IN_OFFICIAL`**. `0xDA` 가 증명하는 것은 통신 경로까지이고 **명령 수용성이 아니다** |
| §18 read | commanded/feedback 구분 없음 → **`CONTROLLER_REPORTED_STATE` 유지** |
| §19 calibration | 공식 semantics 가 P-007C 정적 분석(`0x1C`/`0x1B`)과 일치. 실행 0 |

## 4. P-007B 재평가 — `R1-A OFFICIAL_MATCH`

15항목 비교: **Same 7 · Different 1 · 공식 미문서 7.**

| 항목 | 판정 |
| --- | --- |
| constructor · API · 인자 형식 · time 의미 · 명령 후 지연 · torque 미호출 · Button_Mode 미호출 | Same |
| **angle ranges (§20 E)** | Same — 공식 ID5 0~270/나머지 0~180 이 설치본 분기(`elif id == 5` → `/270`)·`write6_array` 범위검사와 정확히 일치 |
| **생성 직후 `time.sleep(.1)`** | **Different** — 공식 예제 5종 전부에 있고 P-007B 에만 없다 |
| bus · address · 반전 · 변환식 · register · payload · primitive | 공식 미문서 → 비교 불가 |

유일한 차이인 100ms 는 **WEAK_CANDIDATE** 다. 같은 조건에서 읽기가 1,290회 이상 성공했고 P-007B 자신도 write 직전 readback `150` 을 정상 수신했다. 버스가 살아 있었다는 뜻이다.

→ **software API misuse 가설(H-C3·H-C4)이 공식 근거로 더 약해졌다.**

## 5. 공식 기동 구성과 장비가 일치한다

`4. Preparation/1. Manage APP control services` 가 factory image 기동 구성을 기술한다. `evidence/P-007A-R1/autostart_before.json` 의 파일 내용과 **문자열 단위로 같다**:

```
공식 스크린샷 : Startup Applications → "App-Arm" / "app program"
공식 명령     : bash ~/dofbot_pro/APP_DOFBOT_PRO/start_app.sh
공식 3 프로그램: APP control · handle control · APP search device
장비 autostart : Name=App-Arm / Comment=app program
                 Exec=bash /home/jetson/dofbot_pro/APP_DOFBOT_PRO/start_app.sh
장비 실측 3개  : YahboomArm.py · joystick.py · yb-discover.py
```

→ 비표준 기동 구성 가설은 지지되지 않는다. 그리고 **P-007A-R1 의 격리는 공식이 제공하는 비활성화 방법(App-Arm 체크 해제)과 같은 것**을 파일 층에서 한 것이다.

차이 하나: 공식은 체크 해제 후 재부팅을 지시하는데 P-007A-R1 은 재부팅하지 않고 SIGKILL 로 종료했다. 재부팅 자체가 `move_init_pose()` 를 유발하기 때문이다. 그 선택이 보드 상태에 남긴 효과는 **미측정**이다.

## 6. 🔴 공식이 정의하는 "ACK 는 되는데 안 움직이는" 상태 두 가지

공식 자료가 제시한 것은 **차이가 아니라 상태 semantics** 다. 그래서 판정 근거가 되지 못한다. 그러나 가장 중요한 발견이다.

**(1) torque OFF** — `Calibrating the Robotic Arm`:

> enable=0 turns off the torque, the robot arm can manually turn the servo angle, and **does not receive the level signal**

**(2) learning mode** — `Robotic arm memory action` (1차 세션이 못 읽은 자료):

> After entering learning mode, the RGB light … will show a **breathing light** state, and **the robot arm will automatically turn off the torque**.

```
Button_Mode(1) ──공식──▶ 전 서보 torque OFF ──공식──▶ does not receive the level signal
   (0x03)                                                      ↓
                                                     I2C ACK 되지만 모션 0
```

두 경로의 종착점이 같다. H-C6 이 추상 `OPEN` 에서 **공식 정의를 가진 구체 경로**로 바뀌었다.

**그러나 그 상태였다는 증거는 없다.** 토크 읽기 경로가 공식에도 없고(§13 (d)), boot default 기술도 없다(§13 (c)). 운영자 육안 관찰("단단하게 유지 중으로 보인다", 만지지 않음)은 오히려 **토크 ON 쪽 반대 정황**이다. P-007C 가 운영자 증언을 증거로 승격하지 않은 기준을 여기에도 적용한다 — **메커니즘이 정의돼 있다는 것은 "그런 상태가 존재한다"까지이지 "그 상태였다"가 아니다.**

## 7. 🔴 쓰기 0 짜리 판별 수단이 생겼다 — 다음 행동

learning mode 는 **확장보드 RGB 가 breathing light 로 외부 표시**된다. 버스 쓰기 0, 컨트롤러 상태 변경 0, 운영자 육안만으로 판정 가능하다. 토크 상태와 달리 **읽기 경로 부재를 우회한다.**

**P-007D 를 열지 않는다.** 먼저 이 관찰을 한다.

- breathing light 가 **보이면** → learning mode 확정 → torque OFF 확정 → 원인 규명
- **안 보이면** → learning mode 만 배제. `set_torque(0)` 으로 직접 꺼진 경우에는 RGB 표시가 없으므로 torque OFF 자체는 남는다

그 다음 후보는 운영자가 ID2 를 손으로 밀어 저항을 확인하는 것이다(하드웨어 상호작용이므로 별도 게이트).

## 8. 이월 위험

`Arm_serial_set_torque(1)` 과 **`Arm_Button_Mode(0)`** 은 둘 다 토크를 켠다. 후자는 공식이 "automatically turn on the torque" 라고 적으므로 **모드 조작이 아니라 모션 가능 행위**다. 공식은 토크 ON 이 저장된 target 이동을 유발하는지 기술하지 않으므로, P-007B 가 보낸 target(ID2, raw 1242) 래치 가능성을 배제할 근거가 없다. 전체 모션 게이트 절차를 적용한다.

`Arm_Action_Mode(1|2)` 는 동작그룹 재생 = 명백한 모션. `Arm_Clear_Action()` 과 `write_offset_switch(0)` 은 공식이 복구 불가라고 적는다. 전부 호출하지 않았다.

## 9. 1차 세션 중간 기록에서 철회한 것

| 철회 | 이유 |
| --- | --- |
| §5 "공식 스크린샷의 `89` 가 `int()` 절단 편향의 교차확인" | 전문을 읽으니 그 화면은 읽기 루프 단독 출력이고 **같은 실행에 commanded 값이 없다**. 비교 기준이 없다. `NOT_EVIDENCE` 로 강등. 절단 편향 자체는 설치본 정적 분석 결과로서 유효 |
| §7 잠정 판정 "`OFFICIAL ROOT-CAUSE CANDIDATE` 로 수렴 중" | 스펙 §28 의 그 판정은 "설명할 수 있는 **공식 차이** 발견"이고 §21 R1-B 와 같은 어법이다. P-007B 를 R1-A(OFFICIAL_MATCH)로 분류한 이상 모순이다. torque·learning mode 는 차이가 아니라 상태 semantics 다 |
| §6 "Orin 하위 22개 코스" 인벤토리 | 세 번째 트리 `NewTutorials`(31섹션)를 놓쳤다. 보드 정체(EVV01/STM8)를 규정하는 유일한 문서가 거기 있었다 |

## 10. 못 한 것 / 한계

- **Google Drive annex** — 1회 시도. HTTP 200 이지만 JS 셸만 반환되고 파일 목록이 HTML 에 없다. `OFFICIAL_EXACT_MATCH` 로 승급할 수 있는 **유일한 남은 경로**이므로 브라우저 경유 재시도를 권고한다
- 공식 PDF 765개 중 14개만 읽었다. 선정 기준은 서보 제어·토크·학습모드·기동·I2C·펌웨어다. 전문 검색이 아니므로 **"공식에 없다"는 판정은 "판독한 14종에 없다"로 읽어야 한다**
- SIGKILL 종료가 보드에 남긴 상태는 미측정
- `.pyc` code object digest 일치는 **컴파일 동등성**이지 실행 시점 동일성의 증명은 아니다

## 11. 산출물

`acceptance.json`(R1-01~29 전부 PASS) · `environment.json` · `official_repo_pin.json` · `official_tree_inventory.json` · `orin_specific_inventory.json` · `official_materials_manifest.json` · `installed_library_provenance.json` · `transport_generations.json` · `official_servo_examples.json` · `p007b_official_diff.json` · `register_cross_validation.json` · `block_write_analysis.md` · `torque_official_analysis.md` · `button_mode_analysis.md` · `startup_cross_validation.md` · `firmware_version_analysis.md` · `ping_semantics.md` · `controller_read_semantics.md` · `calibration_semantics.md` · `hypothesis_update.json` · `logs/`

`official/` 은 gitignore 다. `official_materials_manifest.json` 의 blob SHA 로 재현한다.

## STOP

토크를 켜지 않았다. 서보 명령 0. Button_Mode 0. P-007D 를 열지 않았다. 하드웨어에 접속하지 않았다.
