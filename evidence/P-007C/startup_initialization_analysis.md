# 기동 스택 초기화 조사 — 필수 초기화를 찾지 못했다

## 조사 대상

P-007A-R1이 격리한 것들의 시작 시퀀스를 정적으로 읽었다.

```
~/.config/autostart/bash.desktop → start_app.sh
  ├─ source /opt/ros/humble/setup.bash
  ├─ source ~/dofbot_pro_ws/install/setup.bash
  ├─ gnome-terminal → python3 YahboomArm.pyc
  ├─ gnome-terminal → python3 yb-discover.py
  ├─ gnome-terminal → python3 joystick.py
  └─ pactl set-default-sink / set-default-source        ← 오디오 설정뿐
```

## `INITIALIZATION_REQUIRED_FOR_CONTROL` 로 분류된 것 — **없다**

기동 경로 전체에서 다음 호출이 **0건**이다.

- `Arm_serial_set_torque` — `set_torque` 사용처는 APP 핸들러(`YahboomArm.py:401`), calibration 노트북, `PC_Control.ipynb`, `12.sync_movement/B_arm_ctrl.py`뿐이고 모두 **사용자가 명시적으로 실행하는 경로**다.
- `Arm_Button_Mode` — `YahboomArm.py:424/426`의 APP 핸들러뿐이다.
- 보드 enable·초기화 성격의 다른 쓰기 — 발견하지 못했다.

`Arm_Device.__init__`(9~12행)은 `smbus.SMBus(7)` 열기와 `self.addr = 0x15` 설정만 한다. 패키지 `Arm_Lib/__init__.py`는 `from .Arm_Lib import Arm_Device` 한 줄로 **import 시점 부작용이 없다.**

벤더 데모들은 `Arm_Device()` 생성 직후 바로 `Arm_serial_servo_write`를 부른다(`single_servo_path.md` 참조). 선행 초기화가 없다.

## `VENDOR_CONVENIENCE_STARTUP` 로 분류된 것

| 항목 | 근거 |
| --- | --- |
| `joystick.py:320` → `Robot_Controller.move_init_pose()` → `write6_array([90,164,18,0,90,30], 1000)` | 조이스틱 입력 루프 진입 전 초기 자세로 **이동**시킨다. 제어 가능 상태를 만드는 초기화가 아니라 자세 이동이다 |
| `YahboomArm.py` main | 부저 3회 후 Flask 6500 시작. `write6` 없음 |
| `arm_driver.py` 생성자 | `write6([90,90,90,0,90,30,3000])` — 역시 자세 이동. 실행하지 않았고 떠 있지도 않다 |

🔴 **`move_init_pose()`가 존재한다는 사실만으로 그것이 필수 초기화라고 결론내리지 않는다**(스펙 §20). 그것은 모션이고, 모션 없이 제어 가능 상태를 만드는 코드는 찾지 못했다.

## i2c 클럭 레이트 스크립트 — 이 시스템에서 무효

`Arm_Lib/Arm_Lib/setup_i2c_clk_rate.py`(584 bytes)가 `/sys/class/i2c-adapter/i2c-7/bus_clk_rate`를 100000으로 맞추려 한다. 실측:

```
cat /sys/class/i2c-adapter/i2c-7/bus_clk_rate  → No such file or directory
i2c-0 / i2c-1 / i2c-2 / i2c-7 전부 동일하게 경로 없음  (커널 5.15.148-tegra)
setup_i2c_clk_rate 참조처: egg-info/SOURCES.txt 와 dist egg 안에만 있다. 기동 경로 참조 0건
```

→ 이 스크립트는 이 커널에서 동작할 수 없고 어떤 기동 경로에도 연결돼 있지 않다. **활성 초기화 단계가 아니다.** 버스 클럭 가설은 이 경로로는 확인할 수 없다.

## 결론

가설 H-C3(벤더 필수 초기화 누락)은 **소스 근거로 지지되지 않는다.** 격리가 제거한 것은 자세 이동(convenience)과 네트워크·조이스틱 진입점이고, 제어 가능 상태를 만드는 초기화는 발견되지 않았다.

다만 이것은 "Arm_Lib가 보이는 범위에 그런 초기화가 없다"는 뜻이다. 컨트롤러 펌웨어가 전원투입·보드 리셋 시점에 스스로 하는 초기화는 이 조사 범위 밖이다.
