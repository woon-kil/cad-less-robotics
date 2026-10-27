# P-007E-R2-R2 — DaBai Passive Observation Path Qualification / Stage A: 정적 코드 감사

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
| `evidence_immutability.json` | 선행 12게이트 167파일 — changed 0 · missing 0 (PASS) |

## 이 게이트가 하지 않은 것

- 카메라 기동 · 캡처 · recorder · ros2 CLI 호출 (0회)
- 로봇/컨트롤러/I2C 접근 (0회) · servo readback 수집 (0회 — R2 금지 사항)
- 커스텀 launch 작성 · 벤더 launch 패치 (우회 즉석 구현 금지)
- 경로 A/B 선택 · ROI · marker · sham · Stage 2
- Gemini·DaBai 물리 포즈 변경 (만지지 않음)
