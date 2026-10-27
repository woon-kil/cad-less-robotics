# torque API 분석 — 읽기 경로가 없다

## 발견한 torque 관련 API 전부

| 함수 | 줄 | register | payload | 반환 | 부작용 |
| --- | --- | --- | --- | --- | --- |
| `Arm_serial_set_torque(onoff)` | 270 | `0x1A` | `onoff==1` → `0x01`, 그 외 → `0x00` | 없음 | 🔴 torque on/off |

주석(269행): `扭矩开关 1：打开扭矩  0：关闭扭矩（可以掰动）` = "토크 스위치. 1=토크 켜기, 0=토크 끄기(손으로 밀 수 있음)".

## 핵심 질문 — 현재 torque 상태를 변경 없이 읽을 수 있는가

**없다.** `Arm_Lib.py` 471행 전체에서 `0x1A`는 270~275행의 쓰기 두 줄에만 나온다. `0x1A`를 읽는 코드가 없다.

→ 판정: **`NO_TORQUE_READ_PATH`**

## 같은 문제를 가진 두 번째 게이팅 상태

`Arm_Button_Mode(mode)` (301행, `0x03`)도 **write-only**다. 주석(300행): `设置K1按键模式， 0：默认模式 1：学习模式` = "K1 버튼 모드 설정. 0=기본, 1=학습 모드". 학습 모드 여부도 읽을 수 없다.

학습(teach) 모드는 통상 토크를 풀어 사람이 자세를 잡게 하고 보드가 위치를 기록하는 모드다. `Arm_Action_Study`(333행, `0x24`)가 그 기록 함수다. 즉 **위치 명령이 무시될 수 있는 두 유력 상태가 모두 read 불가다.**

## 벤더가 torque를 어디서 쓰는가

```
APP_DOFBOT_PRO/YahboomArm.py:401                    Arm.Arm_serial_set_torque(state)      ← APP 핸들러(사용자 조작)
dofbot_ctrl/scripts/Dofbot_Calibration.ipynb:43     Arm.Arm_serial_set_torque(0)
dofbot_ctrl/scripts/Dofbot_Calibration.ipynb:97     Arm.Arm_serial_set_torque(1)
dofbot_ctrl/scripts/PC_Control.ipynb:75/79/170      Arm.Arm_serial_set_torque(True/False)
dofbot_ctrl/scripts/12.sync_movement/B_arm_ctrl.py:21   Arm.Arm_serial_set_torque(0)
dofbot_ctrl/scripts/12.sync_movement/B_arm_ctrl.py:54   Arm.Arm_serial_set_torque(1)
```

🔴 **기동 경로(`start_app.sh` → `YahboomArm.pyc` / `joystick.py` / `yb-discover.py`)에 `set_torque` 호출이 없다.** 즉 벤더 스택을 띄워도 토크를 켜주지 않는다. 토크를 바꾸는 것은 전부 사용자가 명시적으로 누르거나 실행하는 경로다.

→ P-007A-R1의 격리가 "필수 torque enable 초기화"를 제거했다는 가설은 **소스 근거로 지지되지 않는다.**

## 운영자 증언 (하드웨어 무관 질문)

- torque를 끄는 작업(APP torque·학습 모드, calibration 노트북, `12.sync_movement` 데모, 손으로 자세 잡기) 이력: **"없다 / 기억 없다"**
- 팔 외관(육안, 만지지 않음): **"단단하게 유지 중으로 보인다"**

이것은 관찰이다. §26은 진단 A(TORQUE/ENABLE STATE SUSPECTED)에 **read evidence**를 요구하고, 위에서 본 대로 그 read 경로가 없다. 따라서 **진단 A를 선택하지 않는다.**

## 이 게이트에서 하지 않은 것

`Arm_serial_set_torque`를 호출하지 않았다(`environment.json` `torque_change: 0`). torque 상태를 바꾸는 것은 모션 가능 행위이고, 특히 P-007B가 보낸 목표가 컨트롤러에 래치돼 있다면 토크를 켜는 순간 ID2가 그 목표로 움직일 수 있다.
