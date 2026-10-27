# startup sequence 교차검증 (스펙 §15)

자료: `official/04-1-manage-app-services.pdf` (Orin판 `4. Preparation/1.`, blob `c6bf634d…`, 3페이지 전문),
`official/04-2-manage-jupyterlab.pdf` (blob `bd21b461…`), `official/04-3-manage-oled.pdf` (blob `d18e6753…`)

## 공식이 기술하는 factory image 기동 구성

> In the factory image, the APP control program will be **automatically started when the computer is turned on**. There are two programs in total, the first is the ROS node and the second is the APP program.
>
> \#Three terminals, **APP control program, handle control program, APP search device program**
>
> `bash ~/dofbot_pro/APP_DOFBOT_PRO/start_app.sh`
>
> **Permanently close the APP service** — Startup Applications Preferences 에서 **`App-Arm` (app program)** 체크 해제. After cancelling the check of APP-Arm, restart the robotic arm.

## P-007A-R1 실측과의 대조

| 항목 | 공식 | 우리 장비 (P-007A-R1) | 판정 |
| --- | --- | --- | --- |
| 자동 기동 수단 | GNOME Startup Applications 의 **`App-Arm`** 항목 (설명 `app program`) | `~/.config/autostart/bash.desktop` 의 내용이 `Name=App-Arm`, `Comment=app program`, `Exec=bash /home/jetson/dofbot_pro/APP_DOFBOT_PRO/start_app.sh` | **Same — 실측 일치.** 공식 스크린샷의 항목명·설명·대상 스크립트가 `evidence/P-007A-R1/autostart_before.json` 의 파일 내용과 문자열 단위로 같다 |
| 진입점 스크립트 | `~/dofbot_pro/APP_DOFBOT_PRO/start_app.sh` | 같음 | **Same** |
| 기동되는 프로그램 | APP control program · handle control program · APP search device program (3 터미널) | `YahboomArm.py`(Flask APP) · `joystick.py`(핸들) · `yb-discover.py`(UDP 8000 탐색 응답) | **Same** — 3:3 대응 |
| ROS 노드 | "the first is the ROS node" | `kinemarics_dofbot` | **Same** |
| Jupyter | `4. Preparation/2.` 가 별도 서비스로 관리 | `jupyterlab.service` (`inactive` + `enabled`) | **Same** |
| OLED | `4. Preparation/3.` 가 별도 서비스로 관리 | `yahboom_oled.py` (I2C `0x3C`) | **Same** |
| 공식 비활성화 절차 | `App-Arm` 체크 해제 후 재기동 | `bash.desktop` → `.p007ar1.bak` 로 rename (해시 동일) | **Same 효과, 다른 수단** |

→ **우리 장비의 기동 체인은 공식 factory image 구성 그대로다.** 비표준 기동 구성이 원인이라는 가설은 공식 근거로 지지되지 않는다.

→ P-007A-R1 의 격리는 **공식이 공식적으로 제공하는 비활성화 방법과 같은 것**을 파일 층에서 수행한 것이다. 공식은 GUI 체크 해제를 지시하고 우리는 그 체크박스가 쓰는 파일을 rename 했다. 공식 절차와 어긋난 조작이 아니다.

## 차이 하나 — 재기동

공식은 체크 해제 후 **"restart the robotic arm"**(재부팅)을 지시한다. P-007A-R1 은 **재부팅하지 않았다.** 재로그인·재부팅 자체가 `joystick.py` 의 `move_init_pose()` → `[90,164,18,0,90,30]` 을 유발하기 때문이다.

대신 실행 중 프로세스를 SIGKILL 로 종료했다. 결과는 같지만(쓰기 가능 프로세스 0) **공식이 기술한 상태 전이 경로를 밟지 않았다.** 보드 상태 관점에서 차이가 있을 수 있는 지점이며, 이것이 `NO_MOTION_RESPONSE` 와 관련된다는 증거는 없다.

## 공식이 기술하지 않는 것

- 기동 프로그램이 보드에 보내는 **초기화 시퀀스**. 공식은 어떤 프로그램이 뜨는지만 적고 그것들이 `set_torque`·`Button_Mode`·`reset` 중 무엇을 호출하는지 기술하지 않는다.
- P-007C 의 정적 분석은 기동 체인에서 `set_torque`·`Button_Mode`·보드 enable 호출이 **0건**이라고 기록했다. 공식 자료는 이를 확인도 반박도 하지 못한다.
- 🔴 따라서 "SIGKILL 로 종료된 프로세스가 보드를 어떤 상태로 남겼는가"는 **공식·정적 분석 어느 쪽으로도 닫히지 않았다.** `joystick.py`·`YahboomArm.py` 의 종료 경로(SIGKILL 이므로 cleanup 미실행)는 P-007A-R1 이 의도적으로 선택한 것이며, 그 선택이 보드 상태에 남긴 효과는 미측정이다.
