# Button_Mode / learning mode semantics — 공식 근거 (스펙 §14)

자료: `official/08-memory-action.pdf` (Orin판 `17. Basic Control Course/8. Robotic arm memory action`, blob `9ee1ace0…`, 3페이지 전문 판독)
이 문서는 P-007C-R1 1차 세션이 다운로드·blob 검증만 하고 **읽지 못한 채 중단된 자료**다. 재개 세션에서 가장 먼저 읽었다.

## 공식 원문

> `Arm_Button_Mode(enable)` — enable=0: means exiting learning mode, enable=1: means entering learning mode.
>
> After entering learning mode, the RGB light on the robot arm expansion board will show a **breathing light** state, and **the robot arm will automatically turn off the torque**. You can bend the angle of the robot arm at will. … After exiting learning mode, **the robot arm will automatically turn on the torque** and turn off the RGB light.

공식 노트북 `~/dofbot_pro/dofbot_ctrl/scripts/08.study_mode.ipynb` 주석도 같은 말을 반복한다:

> all the servos of the robot arm will enter the **torque-off** state

## 스펙 §14 네 질문

| 질문 | 공식 답 |
| --- | --- |
| normal servo movement 와 관련 있는가 | **YES — 간접적으로 결정적이다.** learning mode 자체는 교시 기능이지만, 진입이 **전 서보 토크를 끈다**. 토크 OFF 는 공식 정의상 신호 미수신이다(`torque_official_analysis.md`). 따라서 learning mode 는 위치 명령을 무력화한다 |
| learning mode 에만 관련 있는가 | NO (위와 같은 이유) |
| boot default 는 무엇인가 | **UNKNOWN** — 기술 없음 |
| 현재 mode read path 가 있는가 | **NO.** 공식 API 는 `Return value: None`. 설치본도 `0x03` write-only(P-007C `register_map.json`). `Arm_Read_Action_Num()`(`0x22`)은 읽기 지향이지만 **저장된 동작그룹 수**만 돌려주고 모드 자체를 읽지 않는다 |

## 🔴 이 게이트가 새로 얻은 것

**두 번째 공식 메커니즘.** 지금까지 `NO_MOTION_RESPONSE` 를 설명할 수 있는 공식 경로는 H-C2(torque) 하나였다. 이제 두 공식 문서가 연결된다:

```
Button_Mode(1)  ──공식──▶  전 서보 torque OFF  ──공식──▶  does not receive the level signal
   (0x03)                                                        ↓
                                                     I2C ACK 는 되지만 모션 0
```

H-C6(읽을 수 없는 게이팅 상태)이 **OPEN 추상 가설에서 공식 정의를 가진 구체 경로로** 바뀌었다. 종착점은 H-C2 와 같다.

## 🔴 쓰기 0 짜리 판별 수단이 생겼다

learning mode 는 **확장보드 RGB 가 breathing light 로 외부 표시**된다. 이것은:

- 버스 쓰기 0 · 컨트롤러 상태 변경 0
- 운영자 육안만으로 판정 가능
- torque 상태와 달리 **읽기 경로 부재를 우회한다**

P-007C 가 `H-C2`·`H-C6` 을 `CANNOT_DECIDE`/`OPEN` 으로 닫은 이유는 "읽을 수 없다" 였다. 적어도 learning mode 쪽은 **읽지 않고 볼 수 있다.** 후속 게이트의 1순위 관찰 항목으로 권고한다.

단 판별력의 범위를 분명히 한다:

- breathing light 가 **보이면** → learning mode 확정 → torque OFF 확정 → 원인 규명
- breathing light 가 **안 보이면** → learning mode 는 배제되지만 **torque OFF 자체는 배제되지 않는다.** `Arm_serial_set_torque(0)` 로 직접 꺼진 경우에는 RGB 표시가 없다

## 이월 위험 (신규)

`Arm_Button_Mode(0)` 은 **공식 정의상 토크를 자동으로 켠다.** 따라서 `Arm_serial_set_torque(1)` 과 동일하게 **모션 가능 행위**로 취급해야 한다. "모드만 끄는 것"이 아니다. 이 게이트는 호출하지 않았다.

`Arm_Action_Mode(1|2)`(`0x20`)는 저장된 동작그룹을 **재생**한다 — 명백한 모션이다. `Arm_Action_Study()`(`0x24`)·`Arm_Clear_Action()`(`0x23`)은 보드 상태를 바꾸며 clear 는 공식이 **복구 불가**라고 적는다. 전부 호출하지 않았다.
