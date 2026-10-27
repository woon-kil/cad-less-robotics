# P-007E-R2-R2 Stage B — Passive UVC Observation Qualification

> Stage A 는 `STATIC SAFETY AUDIT PASS / CAMERA-ONLY PATH IDENTIFIED / CLOSED @ 3579ab5` 로 닫혔다.
> 🔴 **Stage A 파일은 이 스테이지에서 수정하지 않았다** (`stage_b_immutability.json` 에서 `git diff --exit-code 3579ab5` exit 0 확인).
> Stage B 산출물은 전부 새 파일이다.

검수 결정으로 **경로 B(Direct UVC)** 가 선택됐다. ROS 그래프를 격리하는 것이 아니라
`camera topic → color_follow → TargetAngle → arm_driver → Arm_serial_servo_write` 경로를
**구조적으로 제거**한다.

## 판정

```
FOV_PASS
```

| 단계 | 결과 | 파일 |
|---|---|---|
| B0 상속 i2c fd 분류 | 3091/3092 = `INHERITED DESCRIPTOR ONLY` · 🔴 2264(OLED) = **`ACTIVE NON-SERVO I2C BUS USER`** | `b0_i2c_fd_classification.json` |
| B1 운영자 육안 provenance | mounted on distal structure = **YES** · independent articulated axis = **NO** | `b1_operator_provenance.json` |
| B2 UVC capability 열거 | RGB capture node = **`/dev/video0`** (video1 은 Metadata 전용) | `b2_uvc_enumeration.json` |
| B3 단일 프레임 | dequeued/saved **1장** · 44,208 B JPEG · 가드 전후 클린 · Jetson 잔존물 0 | `b3_b4_fov.json` |
| B4 FOV 판정 | 코너 3,481(강 917) · 32×32 타일 커버리지 **76%** · 네 사분면 모두 비지 않음 | `b3_b4_fov.json` · `logs/b4_feature_availability.png` |

🔴 **eye-in-hand 에서는 Arm3~gripper 가 화면에 보일 필요가 없다.** 필요한 것은
**로봇과 독립적으로 고정된 외부 장면**이고, 그것이 충분히 들어와 있다.

## 예산 (`stage_b_command_budget.json`)

```
UVC enumeration           = YES   (설정 변경 0 — S_FMT 0 · 제어값 write 0)
stream sessions           = 1
frames dequeued/saved     = 1     ← 승인 문구 "MAX 1 frame" 을 저장 1장으로 읽은 해석
frames emitted by sensor  = 🔴 NOT MEASURED (--stream-mmap 은 버퍼를 여러 개 큐에 올린다)
ros2 CLI · Arm_Lib · I2C · servo read/write · torque · RGB write · robot motion = 전부 0
camera/robot 물리 이동     = 0
Jetson 임시파일            = 1 생성 → 1 삭제 → 잔존 0
```

## 🔴 Stage B 가 드러낸 별건 — 배타적 제어 가정

`yahboom_oled.py:24` 의 `__BUS_LIST = [1, 0, 7, 8]` 을 영구 순환 탐색한다. **7번이 로봇 버스다.**
`Adafruit_SSD1306/SSD1306.py` 의 `begin()` 은 `reset()`(rst=None → 즉시 반환) → `_initialize()` →
`command()` → `write8` 까지 **반드시** 가므로, 지정된 버스의 0x3C 를 향한 트랜잭션이 시도된다.

- 선행 기록과의 관계: P-007A-R1 은 이미 2264 를 `NON_SERVO_I2C_CLIENT` 로 분류하고 `__BUS_LIST` 까지 적었다.
  🔴 새로 확인된 것은 **실제로 7번을 포함해 순환 점유 중이라는 런타임 관측**과 **그것이 quiet interval 정의에 갖는 함의**다.
- P-007E-R1 의 `exclusive_control_violations: []` 는 그 검사 정의
  (`FORB = ["YahboomArm","joystick","arm_driver","move_group","moveit","start_app"]` 이름 매칭) **안에서 정확하다.**
  fd 보유자와 버스 탐색은 **정의 밖**이었다. R1 파일은 수정하지 않았다.
- 상태: `EXCLUSIVE CONTROL ASSUMPTION — REOPENED FOR CLASSIFICATION`
- 🔴 프로세스 kill · fd close · autostart 수정 **하지 않았다**(검수 지시).
