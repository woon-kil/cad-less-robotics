# P-007E-R2-R2 — DaBai Passive Observation Path Qualification

- **Stage A** 정적 코드 감사 — `STATIC SAFETY AUDIT PASS / CLOSED @ 3579ab5`
- **Stage B** Passive UVC Observation Qualification — **`FOV_PASS`**

`P-007E-R2-R1 — CANDIDATE FOUND / PHYSICAL SETUP REQUIRED / PASS / CLOSED @ b576dbe` 종결 후,
검수 결정에 따라 우선순위가 바뀌었다.

```
PRIMARY    DaBai eye-in-hand        → visual ego-motion detector
SECONDARY  MacBook Pro camera       → external visual observer (fallback)
PRESERVED  Gemini 335L              → workplane / process sensor (DO NOT MOVE)
```

🔴 이 게이트는 **카메라를 켜지 않는다.** P-007A 가 벤더 startup 의 motion-capable 행위를 발견한 선례가 있으므로,
먼저 **로봇/컨트롤러에 전혀 접근하지 않고 DaBai 스트림만 띄우는 최소 실행 경로**가 존재하는지를
텍스트 근거로만 확인한다.

## 판정

```
R2-R2 STAGE A — CAMERA-ONLY PATH IDENTIFIED / STATIC EVIDENCE ONLY / START NOT AUTHORIZED
```

- 카메라 전용 경로는 **존재한다**: `dabai_dcw2.launch.py` 의 launch 폐쇄는 노드 1개(`orbbec_camera::OBCameraNodeDriver`)이고
  Orbbec 트리 전체에 로봇 API 참조가 0건이다.
- 🔴 그러나 **위험은 카메라 드라이버가 아니라 ROS 그래프에 있다.** 코드에는
  `카메라 이미지 → color_follow → TargetAngle → arm_driver → Arm_serial_servo_write` 사슬이 완전하게 존재하고,
  launch 기본 토픽 이름이 그 구독 이름과 정확히 일치한다. `arm_driver` 는 기동만으로 6축 모션을 명령한다.
- 따라서 기동은 **인자 수준의 격리(네임스페이스·도메인)와 사전조건 확인** 없이는 안전하지 않다.
- 기동 승인은 감사한 쪽이 내리지 않는다 → 검수 결정 사항.

## 감사 방법 — 무엇을 일부러 쓰지 않았는가

| 사용 | 금지 | 이유 |
|---|---|---|
| `cat` `sed` `grep` `find` `ls` `ps` `sha256sum` `tr` `wc` `id` `df` | `ros2 launch --show-args` / `--print` | launch `.py` 의 top-level 코드를 **실행**한다 |
| | `ros2 node list` / `topic list` | ros2 **데몬을 기동**한다 |
| | `python3 -c "import <vendor>"` | import 부작용 위험 |
| | `v4l2-ctl --stream-mmap` / `cv2.VideoCapture` | 카메라 **개방** |

raw 로그: `logs/static_audit_raw.txt` (168행, 읽기 전용 스크립트 1회 실행)

## 산출물

| 파일 | 내용 |
|---|---|
| `static_audit.json` | 진입점별 SAFE/UNSAFE 판정 — 전부 `file:line` 원문 인용 |
| `launch_closure.json` | launch 전이 폐쇄 · 기동 프로세스 · 기본 인자 |
| `install_integrity.json` | src↔install sha256 대조 · prefix resolve |
| `indirect_path_risk.json` | 🔴 이미지→모션 간접 사슬 · 패키지 위험표 · 현재 실행 프로세스 · i2c fd 누출 · 도메인 |
| `mounting_provenance.json` | 기계적 부착 근거 P1~P4 (P4 = 운영자 육안, 미확보) |
| `candidate_paths.json` | 경로 A(ROS) vs B(UVC) vs C(Mac fallback) — 비교만, 선택 안 함 |
| `startup_proposal.json` | 🔴 제안 전용. 미집행. 최소 기동 명령 + 사전조건 6 + 사전등록 abort 조건 A~E |
| `decision.json` | 판정 · 증거 명칭 고정 · 금지 표현 · 21px = PREDICTION 표지 · 검수 결정 요청 |
| `command_budget.json` | 전 항목 0 |
| `evidence_immutability.json` | sha 맵 대조 11게이트 161파일 changed 0 · missing 0 + 🔴 커밋 대조 `git diff --exit-code b576dbe HEAD` 12게이트 **exit 0** (PASS) |

## 이 게이트가 하지 않은 것

- 카메라 기동 · 캡처 · recorder · ros2 CLI 호출 (0회)
- 로봇/컨트롤러/I2C 접근 (0회) · servo readback 수집 (0회 — R2 금지 사항)
- 커스텀 launch 작성 · 벤더 launch 패치 (우회 즉석 구현 금지)
- 경로 A/B 선택 · ROI · marker · sham · Stage 2
- Gemini·DaBai 물리 포즈 변경 (만지지 않음)

## 🔴 불변성 서술 주의

baseline 으로 쓴 sha 맵(R2-R1, 161파일)에는 **R2-R1 자신의 6파일이 없다.** 그 6파일의 `changed 0` 은 비교 대상이 없어 자동으로 나온 값이다. R2-R1 산출물까지 실제로 검증한 근거는 커밋 대조(`git diff --exit-code b576dbe HEAD -- <12 게이트>` → exit 0)이다.

---

# Stage B — Passive UVC Observation Qualification

검수 결정으로 **경로 B(Direct UVC)** 가 선택됐다. ROS 그래프를 격리하는 것이 아니라
`camera topic → color_follow → TargetAngle → arm_driver → Arm_serial_servo_write` 경로를
**구조적으로 제거**한다.

## 판정

```
FOV_PASS
```

| 단계 | 결과 | 파일 |
|---|---|---|
| B0 상속 i2c fd 분류 | 3091/3092 = INHERITED DESCRIPTOR ONLY · 🔴 2264(OLED) = **ACTIVE NON-SERVO I2C BUS USER** | `b0_i2c_fd_classification.json` |
| B1 운영자 육안 provenance | mounted on distal structure = **YES** · independent articulated axis = **NO** | `mounting_provenance.json` (P4) |
| B2 UVC capability 열거 | RGB capture node = **`/dev/video0`** (video1 은 Metadata 전용) | `b2_uvc_enumeration.json` |
| B3 단일 프레임 캡처 | 1 frame · 44,208 B · JPEG · 가드 전후 클린 · Jetson 잔존물 0 | `b3_b4_fov.json` |
| B4 FOV 판정 | 코너 3,481(강 917) · 32×32 타일 커버리지 76% · 네 사분면 모두 비지 않음 | `b3_b4_fov.json` · `logs/b4_feature_availability.png` |

🔴 **eye-in-hand 에서는 Arm3~gripper 가 화면에 보일 필요가 없다.** 필요한 것은
**로봇과 독립적으로 고정된 외부 장면**이고, 그것이 충분히 들어와 있다.

## Stage B 에서 쓴 예산

```
camera captures          = 1   (MAX 1 frame 승인분. warm-up skip 0 · S_FMT 0)
UVC enumeration          = YES (설정 변경 0)
ros2 CLI · Arm_Lib · I2C · servo read/write · torque · RGB write · robot motion = 전부 0
camera/robot 물리 이동    = 0
Jetson 임시파일           = 1 생성 → 1 삭제 → 잔존 0
```

## 🔴 Stage B 가 드러낸 별건 — 배타적 제어 가정

`yahboom_oled.py` 가 `__BUS_LIST = [1, 0, 7, 8]` 을 영구 순환 탐색한다. **7 번이 로봇 버스다.**
Stage B 의 행위 때문이 아니라 **이전부터 그랬던 상태**다. 상태 명칭은
`EXCLUSIVE CONTROL ASSUMPTION — REOPENED FOR CLASSIFICATION`.
P-007E-R1 의 `exclusive_control_violations: []` 는 **수정하지 않았다.**
