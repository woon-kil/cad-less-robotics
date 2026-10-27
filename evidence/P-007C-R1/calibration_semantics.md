# calibration semantics (스펙 §19)

이 게이트는 calibration 을 **실행하지 않았다.** 문서 판독만 했다.

자료: `official/02-calibrating-arm.pdf` (blob `3161df1f…`), `official/01-3-calibration-offset-procedure.pdf` (blob `960205b2…`)

## 🔴 공식에 "offset" 이 둘 있다 — 혼동 금지

| | 서보 중점 offset | 카메라/월드 offset |
| --- | --- | --- |
| 문서 | `2. Assembly course/2. Calibrating the Robotic Arm` | `1. Get started quickly/3. Calibration offset procedure` |
| 대상 | 서보 중앙값 편차 | 그리퍼 착지점 xyz 보정 |
| 저장 위치 | **컨트롤러 보드 비휘발성 영역** (하위 MCU 가 저장) | `dofbot_pro_ws/src/dofbot_pro_driver/config/offset_value.yaml` |
| API | `Arm_serial_servo_write_offset_switch(id)` / `_state()` | ROS2 `dofbot_pro_apriltag calibrate_offset` |
| 되돌리기 | `id=0` 으로 전체 초기화 (공식) | yaml 편집 |
| 우리 게이트와의 관계 | **보드 상태를 바꾼다. 금지 대상** | 파일만 바꾼다. 다른 층 |

`01-3` 의 실제 내용은 AprilTag 를 카메라로 읽어 `x_offset/y_offset/z_offset` 을 yaml 에 쓰는 절차다(예시값 `0.0 / 0.01164 / -0.0092`). **서보 중점 교정과 무관하다.** 이름이 비슷해 섞이기 쉬우므로 분리해 기록한다.

## 서보 중점 calibration 공식 semantics

> `Arm_serial_servo_write_offset_switch(id)` — When id=0, **clear the calibration values of all servos and restore the default**; id=1~6 corresponds to the ID number of the six servos. After receiving this command, the underlying microcontroller will read the angle data of the servo corresponding to the ID. If it is within a reasonable range, it will be saved. If it exceeds the range or the servo ID cannot be found, it will not be saved.
>
> `Arm_serial_servo_write_offset_state()` — **state=0** means that the servo is not detected; **state=1** means that the median is set successfully; **state=2** means that the range of the median is exceeded.

P-007C 의 정적 분석(`0x1C` = offset 저장, `0x1B` = offset 상태)과 **의미가 일치한다.** register 번호는 공식 미기술이지만 semantics 층에서 모순이 없다.

### 스펙 §19 항목별

| 항목 | 공식 |
| --- | --- |
| neutral calibration | "keep the robot arm **upright** and hold the gripper tightly" 가 조정 목표 |
| offset save | `write_offset_switch(id)`. 범위 밖이거나 ID 미검출이면 **저장되지 않는다** |
| torque off/on | calibration 절차에서 `Arm_serial_set_torque` 를 쓴다. 손으로 각도를 맞춰야 하므로 OFF 가 전제다 |
| middle pose | 공식 사진이 수직 기립 자세를 보여준다. 각 관절의 수치는 **미기재** |
| 90° convention | **UNKNOWN** — 공식 문서가 "중점 = 90°" 라고 명시하지 않는다. `06-control-6-servos.pdf` 의 `write6(90,90,90,90,90,90,500)` 주석이 "Reset the servo to **center**" 라고 적는 것이 가장 가까운 간접 근거다 |

### 공식 경고 (원문)

> **Note: This program can only be used when the servo needs to be calibrated. It cannot be used at will, otherwise it will cause inaccurate servo control and affect the gripping effect.**

제조사 자신이 함부로 쓰지 말라고 적는다. 이 게이트의 calibration 금지와 일치한다. 특히 `write_offset_switch(0)` 은 **전 서보 교정값 소거**이고 공식이 복구 절차를 적지 않는다 — 사실상 비가역이다.
