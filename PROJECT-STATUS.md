# PROJECT-STATUS — cad-less-robotics

> 이 프로젝트 진행 상황의 **단일 진실의 원천(SSOT)**.
> 세션 시작 시 먼저 읽고, 커밋 후 해당 항목을 갱신한다.
>
> 🔴 `docs/` 는 `.gitignore` 되어 있다(커밋 `d3d65fb`). 플랜·완료보고서·이슈보고서는 로컬에만 있다.
> **추적되는 진행 기록은 이 파일과 `evidence/<gate>/{README.md,acceptance.json}` 뿐이다.**

- 최초 작성: 2026-10-27 14:06 (KST) — P-007C-R1 종료 직후, 사용자 요청으로 신설
- 목표: CAD 없이 뎁스 카메라 포인트 클라우드에서 3D 작업 경로를 만들고 **DOFBOT Pro** 가 **MoveIt 2 Cartesian 궤적**으로 따라가는 PoC (`docs/261026-1255-CAD-less-ROS2-PoC-플랜.md`)
- 실행 환경: p620 Docker / Ubuntu 22.04 / ROS 2 Humble (개발 Mac 은 `colcon` 불가 — P-000 에서 확정)
- 로봇 실기: Jetson Orin Nano `yahboom` / 192.168.0.94, I2C `/dev/i2c-7` addr `0x15`

## 현재 지표

| 항목 | 값 | 근거 |
| --- | --- | --- |
| **블로킹 지점** | **ID2 가 명령대로 물리 이동했는지 확인되지 않았다.** 🔴 "위치 명령에 응답하지 않는다"는 더 이상 맞지 않다 — P-007E-R1 Phase B 에서 **controller readback 이 150 → 152 로 응답했다**. 물리 모션은 `UNOBSERVED` | `evidence/P-007E-R1/phase_b_acceptance.json` |
| 최신 닫힌 게이트 | **`P-007E-R1 PHASE-B — … / CLOSED @ a28e41e`** (검수 종결) | `evidence/P-007E-R1/phase_b_acceptance.json` |
| 진행 중 게이트 | **P-007E-R2-R2 — DaBai Passive Observation Path Qualification** · Stage A `STATIC SAFETY AUDIT PASS / CLOSED @ 3579ab5` · Stage B **`FOV_PASS`** | `evidence/P-007E-R2-R2/stage_b_decision.json` |
| 선행 게이트 | P-007E-R2 Stage 1 `TARGET_NOT_DETECTED / STOP` — Gemini 고정 포즈로는 팔이 화각에 없다 | `evidence/P-007E-R2/stage1_fov.json` |
| 최신 조사 | **P-007E-R2-R2 Stage A `CAMERA-ONLY PATH IDENTIFIED / STATIC EVIDENCE ONLY / START NOT AUTHORIZED`** — 기동 승인 대기 | `evidence/P-007E-R2-R2/decision.json` |
| **게이트가 일으킨** 로봇 모션 | **P-007E-R1 = `UNKNOWN`** · 그 외 0 | 🔴 P-007E-R1 Phase B 는 모션 명령을 1회 집행했고 **물리 모션 여부가 관찰되지 않았다**(운영자 부재). P-007A·A-R1·B·C·C-R1 `environment.json` 은 전부 `robot_moved_by_this_gate: 0`. ⚠️ 프로젝트 전체가 0 이라는 뜻이 **아니다** — 격리 전에는 벤더 autostart 가 부팅마다 `move_init_pose()` 로 팔을 움직였다 |
| 서보 위치 명령 누적 | **2회** (P-007B · P-007E-R1 Phase B — 둘 다 ID2 152°, raw 1242) | P-007B `servo_position_writes: 1` · P-007E-R1 `phase_b_acceptance.json servo_writes: 1`. 나머지 게이트는 전부 `0` |
| `T_base_camera` | **없음** | P-006 BLOCKED. 실측 기준 부재 |
| 비전 파이프라인 | 기하는 동작, 게이트는 FAIL 유지 | P-004 / P-005 |
| 제어 권한 독점 | 확보됨 (가역 격리) | `evidence/P-007A-R1/` |
| 미해결 가설 | **`0x12`/4B 서보 명령 1회(n=1)에서 controller readback 150→152 가 관측됐다.** L4/L5 경계 해석은 검수 세션 몫이다. 남은 미해결은 **물리 모션 여부**다. block-write 전면 고장은 `ELIMINATED`. 아래 §evidence-strength 표 참조 | `evidence/P-007E-R1/phase_b_acceptance.json` |

## 🔴 Evidence-strength 고정 표현 (이 문구를 그대로 쓴다)

### Torque / controller state

```
Learning mode:                   ELIMINATED
Torque-off:                      WEAKENED
                                 P-007C-R3 = TORQUE_ON_STRONGLY_SUPPORTED
Other controller gating state:   NOT FULLY EXCLUDED
```

🔴 다음 표현은 **사용하지 않는다** — `torque-off completely eliminated` · `torque state verified` · `all controller gating eliminated`

### Timing — 측정 사실과 가설을 분리한다

```
OBSERVED:
  short-spacing read instability
  (ced8f52 — Arm_serial_servo_read, 0~8ms 간격에서 None 14~15/30, 10ms 이상 0/30,
   I2C error 출력은 전 구간 0회)

OBSERVED (P-007E-R1 Phase B):
  quiet interval 60.099 ms 로 보낸 동일 명령에서
  controller readback 이 150 → 152 로 바뀌었다 (5라운드 전부, None 0)
  P-007B 의 같은 명령에서는 readback 이 150 불변이었다

  🔴 그러나 P-007B 의 실제 read→write wall-clock interval 은 측정되지 않았다
     → 두 실행의 차이를 timing 단일 변수로 확정할 수 없다 (통제된 대조 아님)
  🔴 물리 모션은 어느 쪽에서도 timing 과 연결되지 않았다 — Phase B 의 모션은 UNOBSERVED

HYPOTHESIS (🔴 검수 세션이 고정한 문구 — 이대로 쓴다):
  Timing hypothesis remains OPEN; controlled retest produced a readback
  change, but the pre-registered physical-motion observation was not obtained.
```

🔴 `SUPPORTED` 도 `NOT SUPPORTED` 도 아니다. **OPEN** 이다.

🔴 다음 표현은 **사용하지 않는다** — `timing root cause confirmed` · `board requires 10 ms` · `servo write requires 10 ms` · `P-007B failed because of timing` · `TIMING HYPOTHESIS STRONGLY SUPPORTED` · `TIMING HYPOTHESIS NOT SUPPORTED BY CONTROLLED RETEST` (Phase B 는 A·D 어느 쪽도 성립하지 않았다)

**아직 어느 것도 증명되지 않았다.** 측정된 불안정은 **읽기** 경로이고, 너무 이른 **쓰기**가 같은 식으로 버려지는지는 직접 측정되지 않았다. P-007B 의 실제 read→write 간격도 기록돼 있지 않다(소스상 sleep 부재만 확인).

### Phase B readback 의 산술 (P-007E-R1)

```
cmd 152       → target raw 1242 → 정확 안착 시 readback 153
observed 152  ⇒ raw 1243..1254  (target 보다 1~12 count = 0.08°~0.98° 모자람)
pre     150   ⇒ raw 1267..1278
raw 이동폭      13..35 counts = 1.06°~2.86°
```

🔴 현재 evidence 가 지지하는 명칭은 **`controller-reported servo position`** 이다 — `physical joint position = 152°` 라고 부르지 않는다. 열린 모델은 `H1 actual servo position feedback` · `H2 controller-side estimated/interpolated position` · `H3 servo-reported internal position without independently observed shaft motion` 셋이다.

🔴 **독립 물리 관측 채널의 명칭(검수 고정, 2026-10-27)** — DaBai eye-in-hand 로 얻는 증거는 **`independent physical-motion evidence via eye-in-hand visual ego-motion`** 이라고만 부른다. 다음 표현은 **금지**한다: `independent external observer` · `independent robot pose measurement` · `absolute joint angle measurement`. 카메라가 로봇에 붙어 있으므로 robot-independent **mechanical reference** 는 아니다 — 독립인 것은 **sensing modality** 뿐이다.

readback 152 는 보낸 target raw 의 **정확한 echo 가 아니다**(echo 면 153) — **단순 command echo 가설만 약화**된다. target 근처 안착 위치와 모양이 같아 **물리 모션(A)을 배제할 수 없다**. 컨트롤러 내부 보간 보고와 이 데이터만으로는 갈리지 않는다. 근거 식은 `evidence/P-007B/command_api_analysis.md`.

## 게이트 현황

판정 문자열은 각 게이트의 `acceptance.json` / `README.md` 에 **기록된 그대로** 옮긴다. 임의로 요약하지 않는다.

### EP-1 환경·빌드 기반

- [x] **PRE-P000** — p620 읽기 전용 조사. `POTENTIALLY SUITABLE`. *게이트가 아니다*
- [x] **P-000** 개발환경 baseline — macOS 에서 `FAIL` → p620 Humble 컨테이너로 이전해 통과
- [x] **P-001** ROS 2 Humble `colcon build` + 단위 테스트 — **`PASS`**. `colcon test-result` 17 tests / 0 errors / 0 failures. 컨테이너 `bytesize-robotics-humble`
- [x] **P-002** DOFBOT URDF + MoveIt `arm_group` collision-aware dry planning — **`P-002 RESULT PASS`** (CASE-01~03 전부 PASS)
- [x] **P-003** Gemini 335L `PointCloud2` — `PASS`. 컨테이너 `bytesize-robotics-camera`

### EP-2 비전 → 경로

- [ ] **P-004** 경계 6D waypoint 추출 — 게이트 **`FAIL`** (법선 부호 불일치 19건, 분류 `NORMAL_ESTIMATION`)
  - [x] attempt-04 가 **`PASS CANDIDATE`** 까지 갔다. 게이트 README 의 `FAIL` 은 유지 중
  - 산출물은 `camera_depth_optical_frame` 기준이다. **로봇이 실행할 waypoint 가 아니다**
- [ ] **P-005** Geometry → Motion dry-run — 게이트 **`FAIL`** (분류 `IK` / `CARTESIAN_PATH`, Cartesian fraction < 0.95)
  - [x] attempt-05 가 joint-space 궤적으로 **`PASS CANDIDATE`**. 단 자체 기록이 "P-005 gate 은 계속 FAIL" 이라고 명시
  - process fidelity 합격 기준이 **아직 정의되지 않았다**

### EP-3 좌표계 정합 (extrinsic)

- [ ] **P-006** Gemini ↔ DOFBOT base extrinsic — **`BLOCKED`**. `T_base_camera` 없음
- [ ] **P-006A** 실물 사진 ↔ URDF 축 대응 — **`INCOMPLETE`** / `MODEL-DEFINED / PHYSICAL ALIGNMENT REQUIRED`
  - 기록 위치는 `evidence/P-006/base-reference/attempt-01~03/` 이다. `evidence/P-006A/` 에는 color 캡처 2개뿐
  - attempt-03 에서 **모델 측은 확정**했다 — Arm1 각 0 에서 `Arm2_Joint` 원점 XY=(0,0), 어깨 힌지 축이 모델 +X (XY-01~10 전부 PASS)
  - 남은 것은 **실물 정합**이다. 하드웨어 영점 표시가 없고, 현재 사진의 팔 방향을 zero 로 가정하지 않았다
- [x] **P-006B** 컨트롤러 관절 읽기 forensic (resume-01~04) — **`DIAGNOSIS PASS — NO SAFE READ PATH`**
  - 저장 중위값을 읽는 경로가 없어 H0 를 벤더 convention 으로 둠
- [x] **P-006C-R2** 작업 ROI ↔ 카메라 관측 분리 — **`CONDITIONAL_PASS`** (52 캡처 / 5 위치)
  - `camera_visible_roi` = `METRIC_BOUNDARY_UNRESOLVED`
- [ ] **P-006C-R2.5** workplane 등록 — **`IMPLEMENTED_NOT_VERIFIED`** / `OPERATOR_ANCHOR_REQUIRED`
  - 25항목 중 **PASS 11 · `NOT_VERIFIED` 14**. **운영자 실측 앵커가 들어와야 진행 가능**

### EP-4 로봇 제어 (현재 전선)

- [x] **P-007A** 읽기 경로 확인 — **`BLOCKED`**. 동시 서보 쓰기 프로세스 때문에 제어 권한이 독점적이지 않았다
- [x] **P-007A-R1** 배타적 제어 환경 — **`PASS`**. 쓰기 가능 프로세스를 **가역 격리**
  - ⚠️ `jupyterlab.service` 가 `inactive` + `enabled` 다. **재부팅하면 Jupyter 표면이 돌아온다**
  - ⚠️ 재로그인·재부팅·전원 재투입은 `joystick.py` 의 `move_init_pose()` → `[90,164,18,0,90,30]` 을 유발한다
  - 원복: `bash scripts/p007a_r1_restore.sh`
- [x] **P-007B** Arm2 최소 제어 모션 — **`NO_MOTION_RESPONSE`**
  - 명령 1회(ID2, 152°, 2000ms, raw 1242, reg `0x12`). 예외 0 · `I2C error` 0 · **운영자 관찰 "전혀 움직이지 않았다"**
- [x] **P-007C** 서보 명령 경로 진단 — **`DIAGNOSIS PASS — D`** (통신 정상, 모션 원인 미해결)
  - 보드 `0x01` 응답 `0.20` · 서보 6개 전부 ping `0xDA`
- [x] **P-007C-R1** 공식 provenance 교차검증 — **`DIAGNOSIS PASS — NO OFFICIAL EXPLANATION FOUND`**
  - P-007B 명령 경로 = **`R1-A OFFICIAL_MATCH`** (공식과 차이 없음)
  - 설치본 provenance = `NO_OFFICIAL_MATCH_FOUND` (공식 저장소에 `.py` 0개 → 확인 불가)
  - 보드 = **EVV01 / STM8_32K**, 펌웨어 채널 = `arm_slave_Vx.y.hex`. 버전 매핑 `VERSION_MAPPING_UNKNOWN`
- [x] **P-007C-R2** Learning-Mode 육안 관찰 — **`OBSERVATION PASS — LEARNING MODE EXCLUDED`**
  - 확장보드 발광 LED 는 `Servo`(고정)·`5V`(고정)·`MCU`(점멸) 셋뿐. **RGB LED 소등** (운영자 실물 사진)
  - 벤더 근거 `VENDOR_WEB_VERIFIED` — 공식 저장소 PDF + 공식 웹 직접 fetch 2채널
  - 🔴 **torque ON 을 증명하지 않는다.** `set_torque(0)` 직접 호출 경우엔 RGB 표시가 없다 (관찰 전 고정한 비대칭)
  - 덤: `SERVO` 레일 LED 점등 → 서보 전원 부재 가설 약화 (`INFERENCE_NOT_EVIDENCE`)
- [x] **P-007C-R3** 수동 토크 상태 관찰 — **`OBSERVATION PASS — TORQUE_ON_STRONGLY_SUPPORTED`** (분류 B)
  - Arm2 극소 힘 → 움직이기 전 명확한 holding resistance. 관절 이동 0 · buzz 0 · 이상 0
  - P-007C 육안 관찰("단단하게 유지")과 독립적으로 같은 방향 — 시각 + 촉각
  - 🔴 `VERIFIED` 아님. 능동 토크 vs 감속기 self-locking 이 이 시험만으로 갈리지 않는다
  - 🔴 **H-C5(block-write)가 최우선으로 승격** — 토크가 켜져 있다면 서보는 명령 수락 상태여야 한다
  - APP `[Stop]`(공식상 토크 토글) 이력을 행동 어휘로 재질의 → "기억 없다". `UNRESOLVED_BY_RECALL`
- [x] **P-007D** Block-Write Command Propagation **정적** 진단 — **`D2 — TRANSPORT BOUNDARY ISOLATED`**
  - 🔴 기존 raw-state 설계는 **폐기**. 검수 세션이 `H-C5` 전용 정적 게이트로 축소 재작성
  - P-007B 경로를 소스 산술로 재구성 → `0x12` · `[0x04,0xDA,0x07,0xD0]` · raw `1242` **완전 일치**
  - D1 후보(register·payload·byte order·source divergence) **전부 음성**
  - L1-L2 `VERIFIED` · L3 `SUPPORTED` · **L4-L5 `UNKNOWN`** · L6 `CONTRADICTED`
  - 🔴 **`write_i2c_block_data` 가 이 보드에서 수락된 사례 0건** — 7개 함수 중 1개만 실행됐고 그게 실패
  - I2C 0 · 모션 0 · mutation 0. SSH 파일 읽기 1회만(해시+cat, import 0)
- [x] **P-007D-R1** RGB Block-Write 제어 실험 — **`CONTROL PASS — BLOCK-WRITE PHYSICAL OUTPUT CONFIRMED`** (A)
  - 🔴 **이 프로젝트 최초의 컨트롤러 상태 변경.** `Arm_RGB_set(0,0,255)` 1회 → 5050 LED 소등→**파랑 점등**
  - 예외 0 · `I2C error` 출력 0 · 로봇 모션 0 · servo command 0
  - **`GLOBAL_BLOCK_WRITE_FAILURE = ELIMINATED`** — block-write 가 이 보드에서 물리 출력까지 도달한다
  - 🔴 `SERVO_0x12_4B_ACCEPTANCE` · `SERVO_BUS_PROPAGATION` 은 **UNRESOLVED 유지** (RGB `0x02`/3B vs 서보 `0x12`/4B)
  - 부수 획득: 펌웨어 채널 순서 **RGB** 확정 (공식 문서에 없던 사실)
- [x] **P-007D-R2** Observability Decision Gate — **`OBSERVABILITY EXHAUSTED / R2-C / CLOSED`**
  - 외부 계측을 접는다. probe point 는 S1~S6 가 아니라 STM8 `TXD`/`RXD` 로 특정됐으나 **전기 안전 미확정**
  - `logic voltage` · `probe ground` 가 blocker. 🔴 **공급전압(6~7.4V) ≠ UART logic voltage**
  - analyzer 샘플레이트(≥2 MS/s)는 제약이 아니었다 — **voltage tolerance 칸을 못 채우는 것**이 NO-GO 근거
  - 하드웨어 접근 0 · 촬영 0 · probe 0 · 구매 0
  - ⚠️ **RGB 가 파랑으로 켜진 채 남아 있다.** 검수 세션 판단: **그대로 둔다.** 기능 문제 없고 provenance 가 명확하다. 복구 전용 게이트를 따로 만들지 않고, LED 상태가 후속 관찰을 **실제로 방해하는 시점에** 복구 행위를 함께 설계한다
- [x] **P-007E** Controlled Servo Response Discrimination (1차) — 기록된 값은 `aborted: "BASELINE_CHANGED:[150, None, 150]"` 뿐이다 (**별도 verdict 문자열 없음** — `evidence/P-007E/phase1_aborted.json`)
  - 가드가 명령을 막았고 그 과정에서 **I2C 최소 간격**을 발견했다 (`evidence/P-007E/i2c_timing_discovery.json`)
  - offline FK/clearance 보존 — `+5°` 는 TCP 13.67 mm, `+2°` 는 5.47 mm (`fk_clearance.json`)
  - 서보 쓰기 0
- [x] **P-007E-R1** Timing-Controlled Servo Command Experiment
  - Phase A — **`PHASE-A PASS — HARNESS READY / NO MOTION COMMAND EXECUTED`**. 서보 쓰기 0
  - Phase B — **`EVIDENCE PASS / EXPERIMENTAL OBSERVATION INVALID / A-C NOT DISCRIMINATED / CLOSED @ a28e41e`** (검수 종결, `phase_b_acceptance.json`)
    - 명령 `Arm_serial_servo_write(2, 152, 2000)` **1회** · quiet interval **60.099 ms** · 예외 0 · `I2C error` 0
    - 🔴 **readback 150 → 152** (5라운드 전부, None 0) — P-007B 는 같은 명령에서 150 불변이었다
    - 🔴 **물리 모션 `UNOBSERVED`** — 운영자가 명령 시각에 자리에 없었다. 사후 사진 비교는 "5mm 판별 불가"로 철회
    - 🔴 `TIMING HYPOTHESIS STRONGLY SUPPORTED` · `NOT SUPPORTED` · `ROOT CAUSE VERIFIED` 어느 것도 쓰지 않는다
- [ ] **P-007E-R2** Independent Physical Observation System Qualification — Stage 1 **`TARGET_NOT_DETECTED / STOP`**
  - 승인 범위 (A): 설계 + **motion 0 실증**. command budget 전 항목 0 고정
  - 카메라 컨테이너가 stale USB 노드를 가리켜 장치 미노출 → `run_humble_camera.sh` 로 재생성(해시 대조 후)
  - 1 unit sanity: color 1280x720 `fx=610.51` · depth 848x480 `fx=410.41` · 프레임 1.94 MB · 인코딩 0.103 s
  - sham 부하 추산 600프레임 = **1166 MB / 61.8 s** → 전프레임 PNG 대신 ROI crop 저장이 맞다(미실행)
  - 🔴 **FOV 1프레임에 DOFBOT 구조물이 없다.** P-006C 고정 포즈는 **작업면**을 본다 — 팔을 보도록 놓인 적이 없다
  - 🔴 Jetson 접속 0 · I2C 접근 0 · robot motion 0. 드라이버 정지·임시물 삭제로 원복
  - 다음은 운영자 물리 결정 — 카메라 재배치(P-006C 고정 전제 파기) / 관측 장치 추가 / `OBSERVATION QUALIFICATION BLOCKED`
- [x] **P-007E-R2-R1** 관측장치 선정 조사 — **`CANDIDATE FOUND / PHYSICAL SETUP REQUIRED / PASS / CLOSED @ b576dbe`** (분류 B, 검수 종결)
  - 성격: DESIGN / INVENTORY / FEASIBILITY ONLY. 캡처 0 · 거치 0 · 로봇 모션 0 · robot controller access 0
  - 🔴 **URDF: `DaBai_DCW2_Joint` parent = `Arm4_Link`** → Jetson 의 Orbbec DaBai DCW2 는 **eye-in-hand**. ID2 와 함께 움직인다
  - p620 에 두 번째 카메라 없음 · Mac 은 내장 1대 · Jetson 에 DaBai RGB-D
  - PRIMARY = Mac 내장(R6 완전 독립, 배치·TCC·사양 미해결) / SECONDARY = DaBai(설치 0, ego-motion 민감도 우위, 🔴 §16 충돌 → 검수 결정)
  - 🔴 Gemini 재배치 NOT RECOMMENDED — workplane baseline 이 현재 포즈에 묶여 있다
  - 🔴 DaBai 사용 시 **벤더 launch 가 로봇 노드를 함께 띄울 위험** 정적 검사 필수 (P-007A `move_init_pose()` 계열)
  - 🔴 검수 결정으로 우선순위 역전 — **PRIMARY = DaBai eye-in-hand** / SECONDARY = Mac 외부 관찰자 / PRESERVED = Gemini(workplane)
- [x] **P-007E-R2-R2 Stage A** DaBai 수동관측 경로 **정적 감사** — **`CAMERA-ONLY PATH IDENTIFIED / STATIC EVIDENCE ONLY / START NOT AUTHORIZED`**
  - 성격: STATIC AUDIT ONLY. 🔴 카메라 기동 0 · 캡처 0 · recorder 0 · `ros2` CLI 0 · 로봇/컨트롤러/I2C 접근 0 · servo readback 0
  - `dabai_dcw2.launch.py` launch 폐쇄 = **노드 1개** (`rclcpp_components/component_container` + `orbbec_camera::OBCameraNodeDriver`). Include 0 · ExecuteProcess 0 · subprocess 0
  - Orbbec 트리 전체 `Arm_Lib`·`smbus`·`Arm_serial`·`move_init_pose`·`set_torque`·`Button_Mode`·`Action_Mode`·`dofbot_pro` = **각 0건**. `i2c` 12건은 전부 **카메라 내부 버스** SDK 선언이고 노드 소스 호출 0건
  - src ↔ install sha256 동일(`b785e5cf…`) · `/opt/ros` 쪽 `orbbec_camera` 중복 없음
  - 🔴 **위험은 드라이버가 아니라 ROS 그래프다** — `arm_driver.py:51` 은 `__init__` 에서 조건 없이 `Arm_serial_servo_write6(90,90,90,0,90,30, 3000)` 를 호출한다. **기동만으로 6축 모션**이다
  - 🔴 완전한 간접 사슬: 카메라 → `color_follow.py:36-37`(`/camera/{color,depth}/image_raw` 구독) → `:33` `TargetAngle` 발행 → `arm_driver.py:25` 구독 → `:72` servo write. launch 기본 `camera_name='camera'` 가 그 구독 이름과 **정확히 일치**한다
  - 🔴 비대화형 ssh 는 ROS 환경 **미상속**(`~/.bashrc` 5-9행 조기 return) · p620 컨테이너 `ROS_DOMAIN_ID` 미설정(=0) · Jetson 대화형 98 → **도메인 명시 없이는 격리가 깨진다**
  - 🔴 PID 3091(`sh`)·3092(`ros2` 래퍼)가 `/dev/i2c-7` fd 11 을 상속 보유 중. 실행 바이너리(3103)는 보유하지 않는다 — 배타적 제어 가정 재검토 사유
  - autostart `bash-1.desktop` 의 Exec 대상(`setting.sh`) **부재 → 현재 NO-OP**
  - 부착 provenance: URDF `type=fixed` + revolute 6개(Arm1~5·grip)가 서보 ID1~6 에 대응해 **카메라용 액추에이터가 없음** + mesh 자산. 🔴 운영자 **육안** 확인(P4)만 미확보 — 만지거나 흔들지 않는다
  - 비교만: 경로 A(ROS orbbec) vs 경로 B(ROS 없는 UVC 직접 읽기 — 표면적 최소, RGB only, intrinsics 표준 경로 없음). 🔴 **선택하지 않았다**
  - 🔴 `~21 px` 는 **PREDICTION** (fx≈600 가정, DaBai 실측 intrinsics 아님). 실측 qualification 전 검출 성능으로 승격 금지
  - 🔴 Jetson 디스크 여유 **9.3G (86% 사용)** — 녹화 예산 제약
  - 다음은 검수 결정 — ① 최소 기동 승인 ② 경로 A/B 선택 ③ 운영자 육안 provenance
- [x] **P-007E-R2-R2 Stage B** Passive UVC Observation Qualification — **`FOV_PASS`**
  - 검수 결정: **경로 B(Direct UVC)** 채택. ROS 그래프를 격리하는 대신 `camera topic → color_follow → TargetAngle → arm_driver → servo write` 경로를 **구조적으로 제거**한다
  - B0 상속 fd 분류 — 3091/3092 = `INHERITED DESCRIPTOR ONLY`. 🔴 **2264(`yahboom_oled.py`) = ACTIVE NON-SERVO I2C BUS USER** (아래 이월 위험)
  - B1 운영자 육안 — `mounted on distal structure = YES` · `independent articulated camera axis = NO` (접촉 0). `evidence/P-007E-R2-R2/b1_operator_provenance.json`
  - B2 열거 — RGB capture node = **`/dev/video0`**. `/dev/video1` 은 Metadata Capture 전용(ENUM_FMT 빈 목록·G_FMT 실패). 설정 변경 0
  - B3 캡처 — **dequeued/saved 1장** (예산 `MAX 1 frame` 을 저장 1장으로 읽은 해석). S_FMT 0 · 44,208 B JPEG · Jetson 잔존물 0 · 가드 전후 클린. 🔴 센서가 STREAMON~OFF 사이 흘린 프레임 수는 **미측정**
  - B4 FOV — 코너 3,481(강 917) · 32×32 타일 커버리지 **76%** · 네 사분면 모두 비지 않음 · 노출 mean 139(포화 12.7%)
  - 🔴 eye-in-hand 에서는 Arm3~gripper 가 보일 필요가 없다 — 필요한 것은 **로봇과 독립적으로 고정된 외부 장면**이고 충분히 들어와 있다
  - 🔴 Stage C 제약: 자동노출·자동WB·dynamic framerate ON · `power_line_frequency=50Hz`(국내 60Hz 불일치) · 특징 상당수가 변형/이동 가능 대상 위 · 경로 B 는 RGB only·intrinsics 표준 경로 없음
  - 예산: 저장 프레임 1장 외 전 항목 0 (ros2 CLI·Arm_Lib·I2C·servo read/write·torque·RGB write·robot motion·물리 이동 = 0)
  - 🔴 Stage A 는 `CLOSED @ 3579ab5` — Stage B 는 그 파일을 수정하지 않고 새 파일로만 기록했다 (`git diff --exit-code 3579ab5` exit 0)

## 🔴 다음 행동

~~1. RGB breathing 육안 확인~~ → **R2 완료. learning mode 배제**
~~2. ID2 수동 저항 확인~~ → **R3 완료. 토크 ON 쪽**
~~3. P-007D 로 H-C5 조사~~ → **완료. 호스트측 정상, L4 이후로 국소화**

~~4. RGB block-write 실험으로 L4 시험~~ → **R1 완료. block-write 는 이 보드에서 작동한다**

**남은 질문: `0x12`/4B 서보 명령만 수락되지 않는가, 아니면 L5(보드→서보 버스) 이후인가.**

🔴 **구조적 제약**: 설치 라이브러리의 block-write 7경로 중 **비모션은 RGB(3바이트) 하나뿐**이다. `0x12`/4B 수락 여부를 **모션 없이 가를 경로가 존재하지 않는다**.

| 경로 | register/길이 | 모션 |
| --- | --- | --- |
| `Arm_RGB_set` | `0x02` / 3B | **없음 — 이미 사용** |
| `Arm_serial_servo_write` | `0x12` / 4B | 있음 |
| `write_any` | `0x19` / 5B · `0x17` / 4B | 있음 |
| `write6` 계열 | `0x1E`→`0x1D` / 2→12B | 있음 |

**P-007D-R2 CLOSED — `OBSERVABILITY EXHAUSTED / R2-C`.**

외부 계측을 접는다. `R2-B`(servo-bus passive capture)는 아키텍처가 확인됐으나 **`TXD/RXD logic voltage` 와 `probe ground reference` 를 공식 근거로 확정하지 못했다.** 전원 OFF continuity 로 ground 는 잡아도 **logic HIGH voltage 는 끝내 안 풀리고**, 그러면 powered voltage measurement 게이트가 또 필요하다. 🔴 이 프로젝트 목표는 expansion-board firmware reverse engineering 이 아니다.

| 후보 | 판정 |
| --- | --- |
| R2-A kernel I2C trace | `DEFERRED` — **not eliminated**. ceiling 이 L3. 단 NACK 반증 가능성은 남는다 |
| R2-B servo-bus capture | `BLOCKED BY ELECTRICAL PROBE SAFETY` |
| **R2-C** | 🔴 **SELECTED** |

**최종 경계** (🔴 P-007E-R1 Phase B 로 갱신)
```
L1 VERIFIED → L2 VERIFIED → L3 SUPPORTED → L4 STM8 ┬ RGB    0x02/3B VERIFIED(물리효과)
                                                    └ Servo  0x12/4B — Phase B 1회:
                                                        readback 150→152 OBSERVED
                                                        (quiet interval 60.099 ms, n=1)
                                                        → L5 UNKNOWN → L6 MOTION UNOBSERVED
```

🔴 n=1 이다. 성질로 일반화하지 않는다. `L6 OBSERVED_NO_MOTION` 은 P-007B 한정이다. Phase B 의 L6 는 **관찰 자체가 없다**(`UNOBSERVED`) — 모션 없음이 관찰된 것이 아니다.

## 🔴 P-007E-R2 원안 (고정 외부 카메라 전제 — **SUPERSEDED**)

> 🔴 **아래 설계는 Stage 1 에서 전제가 깨졌다.** 고정 Gemini 는 작업면을 보고 팔은 화각 밖이다(`TARGET_NOT_DETECTED`).
> 검수 결정으로 관측 모델이 **고정 카메라의 pre/post 3D displacement → Arm4 eye-in-hand 의 visual ego-motion** 으로 바뀌었다.
> 현행 설계·제약은 `evidence/P-007E-R2-R2/`(판정·간접경로 위험·기동 제안)를 본다. 아래 본문은 **이력으로 보존**한다.

**P-007E-R1 Phase B 는 검수 종결됐다** — `EVIDENCE PASS / EXPERIMENTAL OBSERVATION INVALID / A-C NOT DISCRIMINATED / CLOSED @ a28e41e`.

검수 세션 결정 3건 (`phase_b_acceptance.json` → `🔴 reviewer_closure`):

1. **A/C 복구 불가** — 명령 당시의 pre/post synchronized physical observation 이 없다. 지금 depth 로 재는 것은 `post-state measurement` 일 뿐이다. 🔴 **영구 evidence gap 으로 보존한다.** 사후 복원을 시도하지 않는다
2. **baseline stale** — `STALE / DO NOT EXECUTE` 3종 표시 완료. 🔴 152→150 복귀 명령은 새로운 physical experiment 이므로 보내지 않는다
3. **readback 명칭** — `controller-reported servo position`. H1/H2/H3 열림. 다음은 또 다른 +2° 반복이 아니라 **readback ↔ physical pose correspondence** 검증이다

### P-007E-R2 — 목표는 하나 (🔴 Stage 1 에서 `TARGET_NOT_DETECTED` 로 멈춤 / 관측 모델은 R2-R2 로 대체됨)

> **다음 단 한 번의 motion command 가 발생할 때 controller readback 과 independent physical measurement 를 동시에 확보할 방법을 설계한다.**

세 채널을 동시에 잡는다.

```
controller channel      pre/post ID2 readback
independent physical    ⚠️ SUPERSEDED: 고정 카메라의 synchronized RGB-D pre/post displacement
                        → 현행: eye-in-hand visual ego-motion (DaBai, Arm4)
operator channel        realtime visual observation
```

- ⚠️ (SUPERSEDED — 고정 카메라 전제) 🔴 전체 P-006 calibration 은 불필요하다. `T_base_camera` 가 아니라 **"실제로 로봇 링크가 움직였는가"** 라는 독립 센서 증거만 있으면 된다. 고정 카메라에서 동일 physical feature 의 pre/post 3D displacement 로 충분하다
- 🔴 **명령 전에 카메라 기록이 실제로 시작됐다는 것을 machine-verifiable guard 로** 만든다. 실험 validity 가 운영자 육안 관찰 여부에 다시 의존하지 않도록
- 🔴 이 게이트에서 보내지 않는 것: **152→150 복귀 · +2° 재시험 · torque**. motion command 0

들고 들어가는 사실: torque `TORQUE_ON_STRONGLY_SUPPORTED` · learning mode 제외 · global block-write failure 제외 · 공식 API encoding 이상 없음 · RGB/STM8 path 동작 · **`0x12`/4B 서보 명령 1회에서 controller readback 이 target 근처 값으로 관측됐다(150→152, quiet interval 60.099 ms, n=1)** · **물리 모션은 어느 실행에서도 양성 관찰이 없다**.

## ⚠️ 이월 위험 (모든 세션에서 유효)

> 🔴 **E-R1 갱신 — baseline 이 바뀌었다.** 현재 controller readback 은 **`[86, 152, 0, 18, 93, 178]`** 이다(P-007E-R1 Phase B). **`150` 을 전제한 스크립트는 전부 stale** 이다 — `scripts/p007e_r1_timing_controlled.py` 의 `EXPECTED_BASELINE`/`EXPECTED_ID2`, `scripts/p007b_preflight.py`, `scripts/p007e_preflight.py` 포함. 그 전제를 쓰는 실행은 `PRE_COMMAND_BASELINE_ABORT` 로 끝난다. 🔴 **동일 조건 재실행은 추가 모션 명령(152→150) 없이는 불가능하고, 그것은 승인 예산 밖이다**
>
> 🔴 **`STALE / DO NOT EXECUTE`** — `ID2 == 150` 을 요구하는 실행 harness 3종: `scripts/p007e_r1_timing_controlled.py` · `scripts/p007e_phase1_command.py` · `scripts/p007b_write_once.py`(150 을 넘기는 호출). 목록·사유·sha256 은 `evidence/P-007E-R1/stale_harness_manifest.json`. 파일 본문은 **수정하지 않았다** — acceptance 에 기록된 `script_sha256` provenance 를 깨지 않기 위해서다
>
> 🔴 **152 → 150 복귀 명령 금지.** baseline 복구 목적이라도 그것은 데이터 정리가 아니라 **새로운 physical experiment** 다 (검수 지시 2026-10-27)
>
> 🔴 **물리 위치는 미확인이다.** readback 152 가 실제 관절 위치인지 컨트롤러 보고값인지 갈리지 않았다. 다음 게이트가 FK·clearance 를 계산할 때 `150` 을 실제 자세로 가정하면 안 된다


> 🔴 **R2-R2 Stage B 갱신 — 로봇 I2C 버스에 제3 사용자가 있다.** `yahboom_oled.py` 가 `__BUS_LIST = [1, 0, 7, 8]` 을 **영구 순환 탐색**한다. 7번이 로봇 버스다. open 은 성공하는데도 예외로 떨어져 회전하므로 **버스 7 에 0x3C(OLED) 를 향한 트랜잭션이 시도된다**. 서보 명령 능력은 없다(Arm_Lib 0 · 0x15 접근 0).
> 🔴 **따라서 `quiet interval` 은 "버스가 조용했다"를 뜻하지 않는다.** P-007E-R1 의 정의는 "**우리 프로세스가** 버스를 건드리지 않은 구간"이고 제3 프로세스를 배제하지 못한다. 다음 motion gate 의 quiet interval 설계는 이 사실 위에서 다시 세운다.
> 상태: `EXCLUSIVE CONTROL ASSUMPTION — REOPENED FOR CLASSIFICATION`. 🔴 **프로세스 kill·fd close·autostart 수정 금지**(검수 지시). P-007E-R1 의 `exclusive_control_violations: []` 도 수정하지 않았다.
> 🔴 이것이 P-007B 무응답이나 P-007E-R1 readback 변화를 **설명한다고 말하지 않는다** — 간섭원의 존재만 확인됐고 시간적 중첩은 측정되지 않았다.
> 근거: `evidence/P-007E-R2-R2/b0_i2c_fd_classification.json` · 이슈보고서 `docs/261027-2212-*`
>
> 🔴 **ROS 도메인 정정.** 실행 중 노드의 `/proc/<pid>/environ` 에 `ROS_DOMAIN_ID` 가 없다 → 현재 그래프는 **도메인 0** 이고 p620 컨테이너와 같다. `~/.bashrc:127` 의 98 은 그 프로세스에 적용되지 않았다.

> 🔴 **R2-R2 갱신 — 카메라를 띄울 때의 함정 2건.**
> ① `ros2 run dofbot_pro_driver arm_driver` 는 **기동만으로** `arm_driver.py:51` 의 `Arm_serial_servo_write6(90,90,90,0,90,30, 3000)` 를 집행한다. 이 노드(와 `dofbot_pro_driver`·`grasp`·`apriltag_*`·`voice_ctrl` 계열)는 **어떤 이유로도 띄우지 않는다**.
> ② 벤더 카메라 launch 를 **기본 `camera_name='camera'`** 로 띄우면 토픽이 `/camera/color/image_raw`·`/camera/depth/image_raw` 로 나가고, 그것은 `color_follow.py:36-37` 등 **앱 노드의 구독 이름과 정확히 일치**한다. 그 앱 노드가 떠 있으면 `TargetAngle` → `arm_driver` 로 모션이 이어진다. 카메라를 띄울 때는 `camera_name` 을 전용 이름으로 바꾸고 `ROS_DOMAIN_ID` 를 명시한다(p620 컨테이너는 도메인 **0**, Jetson 대화형은 **98**, 비대화형 ssh 는 **미설정**).
> 근거·원문 인용: `evidence/P-007E-R2-R2/indirect_path_risk.json`

> 🔴 **R1 갱신 — 보드 상태가 바뀌었다.** 확장보드 RGB 가 **파랑으로 켜진 채** 남아 있다(P-007D-R1). 이후 보드 LED 를 관찰할 때 baseline 을 **`P-007D-R1 induced BLUE`** 라고 명시한다 — learning mode(breathing)도 `Arm_Clear_Action` 완료(녹색)도 아니다. **P-007C-R2 의 "RGB 소등" baseline 은 더 이상 유효하지 않다.** 복구는 별도 게이트로 만들지 않는다(검수 세션 판단). LED 가 관찰을 실제로 방해하는 시점에 그 게이트 안에서 함께 설계한다.

> R3 갱신: 토크가 **이미 켜져 있는** 쪽이 유력해져 "토크를 켜는 순간 래치된 target 으로 움직인다" 경로는 **약해졌다.** 🔴 제거되지는 않았다 — 토크 ON 은 `STRONGLY_SUPPORTED`이지 `VERIFIED`가 아니고, 컨트롤러가 target 을 보관하는지 자체가 미확인이다.

- `Arm_serial_set_torque(1)` 과 **`Arm_Button_Mode(0)`** 은 둘 다 토크를 켠다. 후자는 공식이 "automatically turn on the torque" 라고 적는다 → **모션 가능 행위**
- 공식은 토크 ON 이 저장된 target 이동을 유발하는지 **기술하지 않는다**. P-007B 가 보낸 target(ID2, raw 1242) 래치 가능성을 배제할 근거가 없다
- `Arm_Action_Mode(1|2)` = 동작그룹 재생 = 명백한 모션
- `Arm_Clear_Action()` · `Arm_serial_servo_write_offset_switch(0)` = 공식이 **복구 불가**라고 적는다
- 정지 경로는 어느 것도 시험하지 않았다. **Emergency Stop 은 전원 차단뿐**(`evidence/P-007A-R1/stop_strategy.md`)

## 작업 구간 (커밋 시각 기준 — **세션 경계가 아니다**)

> 아래 구간은 `git log` 의 시각 빈틈으로 나눈 것이다. 실제 세션 경계와 일치하지 않는다.
> 확인된 세션 ID 는 `1a885caa`(P-000~P-007C-R1 1차)와 `cb876b8e`(P-007C-R1 재개, 현재) 둘뿐이다.

| 구간 | 커밋 시각 | 범위 | 결과 |
| --- | --- | --- | --- |
| 1 | 2026-10-26 13:37~15:33 | 저장소 개설 · P-000~P-004 | P-000~P-003 통과, P-004 측정 시작 |
| 2 | 2026-10-26 15:43~17:48 | P-004 법선 · P-005 IK/Cartesian | 양쪽 게이트 FAIL, attempt 로 PASS CANDIDATE 도달 |
| 3 | 2026-10-26 17:48~22:23 | P-006 · P-006A · P-006C (Attempt 1~6 → R1 → R2 → R2.5) | P-006 BLOCKED, R2 CONDITIONAL_PASS, R2.5 운영자 앵커 대기 |
| 4 | 2026-10-26 23:17~2026-10-27 00:00 | P-006B resume-01~04 · P-007A | NO SAFE READ PATH · P-007A BLOCKED |
| 5 | 2026-10-27 08:53~10:12 | P-007A-R1 · `docs/` 추적 해제 | PASS. 쓰기 프로세스 가역 격리 |
| 6 | 2026-10-27 10:12~11:14 | P-007B · P-007C · P-007C-R1 착수 | NO_MOTION_RESPONSE · DIAGNOSIS PASS D · R1 중간 중단(사용 한도) |
| 7 | 2026-10-27 13:41~14:06 | P-007C-R1 재개·종료 · PROJECT-STATUS 신설 (세션 `cb876b8e`) | NO OFFICIAL EXPLANATION FOUND. R1-01~29 전부 PASS |
| 8 | 2026-10-27 14:09~14:35 | P-007C-R2 육안 관찰 게이트 (세션 `cb876b8e`) | LEARNING MODE EXCLUDED. 6라운드 운영자 왕복 |
| 9 | 2026-10-27 14:35~14:50 | P-007C-R3 수동 토크 관찰 (세션 `cb876b8e`) | TORQUE_ON_STRONGLY_SUPPORTED. H-C5 최우선 승격 |
| 10 | 2026-10-27 14:50~14:57 | P-007D 정적 진단 (세션 `cb876b8e`) | D2. 호스트측 VERIFIED, L4 이후 UNKNOWN |
| 11 | 2026-10-27 14:57~ | P-007D-R1 RGB 제어 실험 (세션 `cb876b8e`) | **A. 최초 상태 변경.** block-write 물리 출력 확인 |
| 12 | 2026-10-27 15:59~18:39 (KST) | P-007D-R2 · P-007E · P-007E-R1 Phase A | R2-C CLOSED · E 가드 abort(`BASELINE_CHANGED`) · R1 harness READY (서보 쓰기 0) |
| 13 | 2026-10-27 18:39~19:02 (KST, 커밋 `be1491e`) | **P-007E-R1 Phase B** (세션 `66eb4937`) | **A/C NOT DISCRIMINATED.** 서보 명령 2번째 1회 집행(18:49:06 KST), readback 150→152, 물리 모션 UNOBSERVED |
| 14 | 2026-10-27 19:02~19:48 (KST, 커밋 `02de7b0`~`b576dbe`) | P-007E-R1 검수 종결 반영 · **P-007E-R2 Stage 1** · **P-007E-R2-R1** (세션 `66eb4937`) | Stage 1 `TARGET_NOT_DETECTED` · R2-R1 `CANDIDATE FOUND` — 로봇 접근 0
| 15 | 2026-10-27 20:47~21:54 (KST, 커밋 `ee0b2ca`) | **P-007E-R2-R2 Stage A** 정적 감사 (세션 `66eb4937`) | `CAMERA-ONLY PATH IDENTIFIED / START NOT AUTHORIZED` — 카메라 기동 0 · `ros2` CLI 0
| 16 | 2026-10-27 22:02~22:12 (KST, 커밋 `639c0ca`) | **P-007E-R2-R2 Stage B** B0~B4 (세션 `66eb4937`) | **`FOV_PASS`** — 캡처 1프레임 · 🔴 로봇 버스 제3 사용자 발견

> 🔴 **시계 주의**: Jetson 은 **+0800**, 이 표와 git 은 **KST(+0900)** 다. 증거 파일의 `commanded_at 17:49:06+0800` = KST 18:49:06.

## 운영 규칙 (이 프로젝트 고유)

- **게이트 판정 문자열은 재서술하지 않는다.** `acceptance.json` 의 값을 그대로 옮긴다
- **운영자 관찰은 증거로 승격하지 않는다.** 관찰로만 기록한다 (P-007C 가 세운 기준)
- **선행 게이트 증거는 불변이다.** 새 게이트는 자기 디렉터리에만 쓴다. `git diff --exit-code HEAD -- evidence/<선행>` 으로 착수·종료 시 확인
- **플랜 검토는 advisor 가 한다** (codex 자문 대체 — 사용자 지시 2026-10-27)
- **가설 라벨은 분기까지 적는다** — `H-C6.learning_mode`(ELIMINATED)와 `H-C6.other_controller_state`(OPEN)는 다르다. 상위 라벨만 적어 범위를 넓게 읽히게 하지 않는다 (검수 세션 지적 2026-10-27)
- 외부 수집 자료는 **gitignore 된 게이트 디렉터리 + 커밋되는 매니페스트**로 둔다. 스크래치패드 금지 (`docs/261027-1358-*-이슈보고서.md`)
