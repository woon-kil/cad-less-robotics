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
| 진행 중 게이트 | **P-007E-R2-R2** · Stage A/B/C0/C1/C0-R1/C0-R2/C1-R1/C1-R1-R1 CLOSED · **P-007E-R2-R3** `CONTROLLED PHYSICAL-MOTION EXPERIMENT PREREGISTERED` CLOSED · **P-007E-R2-R4 `MOTION EXECUTION SAFETY PREFLIGHT PASS / ARM2 152→154 CANDIDATE READY FOR SEPARATE EXECUTION APPROVAL / NO CAMERA OR ROBOT EXECUTION`** — Jetson 에서 URDF/SRDF/13개 STL mesh 를 read-only 로 가져와(SHA256 전송무결성 확인, 원본 미삭제) tree-FK 모듈을 새로 구현, 선례(p007b self-check/fk_clearance.json 150→155/P-007B +2° TCP 5.47mm/P-007A Arm3 margin) 4건을 전부 재현(오차 ≤0.005mm)해 검증한 뒤 152→154 실제 계산 수행. TCP 변위 5.47mm, DaBai 변위 5.44mm/회전 2.0°, 로봇 전체 최대 변위(exact, 정점기반) 5.93mm(DaBai_DCW2_Link)/sphere-bound cross-check 8.46mm, joint limit 전부 PASS(Arm3 margin≈0 공개), self-collision은 벤더 SRDF(전체 link pair Adjacent/Never/Default 로 disable)를 1차 근거로 FK spot-check(Arm1-Arm2 ~0.13mm 접촉/base-Arm2 ~16mm 간극, 2° 구간 안정)로 보완. raw servo 인코딩 재확인(152→raw1242 선례 재현, 154→raw1217, payload `[0x04,0xC1,0x07,0xD0]`), raw/readback 버킷 불확실성에서 Δ_max=3.03° 도출해 operator envelope=15mm 확정. R3(CLOSED, SHA 고정)는 1바이트도 수정하지 않고 import 재사용, R4 wrapper가 workspace-safety gate·write-ahead 재전송방지 ledger를 추가. negative test 25종 전부 PASS. 🔴 **실제 `152→154` 명령 실행은 이 gate로 승인되지 않음 — 검수자가 별도 실행 승인을 내려야 함** · **P-007E-R2-R5 코드/준비상태 구축 완료 — 실제 `--execute`(실제 capture/controller read/servo write ≤1회 허용)는 로봇을 직접 지켜보는 운영자가 Jetson 에서 §31 승인 문구를 입력해야만 진행됨, Claude 는 `--check-only`(opt-in 0)까지만 실행** · **P-007E-R2-R5 Attempt #3 결과: `ABORT_BEFORE_MOTION`(provenance self-contamination, 수정 완료 d02611e) · Attempt #4(운영자 직접 실행): 실제 write 1회 발생, readback 152→154, operator 관찰 physical motion=NO — 원인 미확정** · **P-007F-01 `READBACK_SEMANTICS_UNRESOLVED` / `TORQUE_STATE=UNKNOWN` / `SERVO_COMMUNICATION=SUPPORTED`** — read-only Arm_Lib 1.0.3 source forensic + runtime probe(새 motion 0건) · **P-007F-02 `TORQUE_DISCRIMINATION_REQUIRES_DESIGN_CHANGE`** — torque/actuator-state 판별 실험 사전등록(DESIGN ONLY, 하드웨어 변경 0), Candidate A(비카메라 외력+독립 inclinometer)를 primary witness로 채택, DaBai=secondary/corroborating만, Gemini=`NOT CURRENTLY USABLE`(화각 밖, 재배치 안 함), Q2(실제 torque OFF)는 spotting 절차 미설계로 보류 · **P-007F-02A `PHYSICAL_WITNESS_REQUIRES_OPERATOR_SETUP`** — 물리 계측 도구 qualification(저장소에 스마트폰/수평계/protractor 재료 보유 evidence 없음을 확인, 추정 안 함), PRIMARY=protractor(독립·하중≈0), SECONDARY=스마트폰(mechanical loading 안전성 수치 확인 불가로 조건부), 운영자 재료 확보 대기 | `evidence/P-007F-02A/recommendation.md` |
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
- [x] **P-007E-R2-R2 Stage C0** Stationary Ego-Motion Qualification **사전등록** — `PREREGISTRATION / C1 HARNESS READY`
  - 성격: DESIGN / OFFLINE ANALYSIS / PREREGISTRATION ONLY. 🔴 카메라 기동 0 · 새 캡처 0 · UVC 제어 변경 0 · 로봇 접근 0
  - ROI 동결 — 캐비닛 강체 3영역(프레임 5.44%, ORB 449개). 🔴 **robot-attached pixel 넉넉히 mask** (eye-in-hand 에서 그 화소는 ID2 회전 시 화면상 정지로 보이고 ID5·6 이 움직이면 거짓 모션)
  - 알고리즘 1종 고정 — ORB + BFMatcher(Hamming, crossCheck) + `estimateAffinePartial2D`(RANSAC 1.5 px). 모델 = 2D partial affine 4 DOF
  - 가드 — features≥150 · matches≥60 · inliers≥40 · ratio≥0.50 · residual≤1.50 px · |scale−1|≤0.05
  - 🔴 **K/M 의미 충돌 명시 해소** — 선행 설계(`docs/261027-1914-*` §D4) 우선: `K=5`는 **임계 배수**, `M=60`은 window 프레임 수, `N_sham=10`
  - 임계 공식 `thr = K × max(1.4826·MAD, σ_floor)` · σ_floor = 0.70 px · 0.060°(C0 합성 회수 오차 최댓값). FP 는 **leave-one-out** 으로 센다
  - 판정 어휘 `X > Q3 > Q4 > Q2 > Q1` — 🔴 성공 판정은 **Q1 하나뿐**
  - 🔴 sham 과 미래 motion trial 의 **알려진 차이** 기록 — R2 예산이 servo readback 을 금지하므로 sham 에 controller 측 read 가 없다. 카메라 측만 완전한 거울이다
  - 단위검증(🔴 `ALGORITHM UNIT VALIDATION`, 물리 감도 아님) — 합성 14종 평행이동 오차 ≤0.703 px · 회전 ≤0.060° · 밝기/대비 섭동 겉보기 변위 ≤0.015 px
  - C1 harness `scripts/p007e_r2r2_c1_sham.py` — 🔴 **기본값 NO CAPTURE**. `ALLOW_CAPTURE=1` + `--run-sham` 둘 다 필요. 사전등록 sha256 변조 시 중단(실측 확인). 음성 시험 3종 통과
  - 🔴 카메라 제어 변경 0 — 자동노출 ON·자동WB ON·50 Hz 그대로 C1 첫 baseline 을 측정한다
  - 예산 전 항목 0. Stage A(@3579ab5)·Stage B(@4d0c709)·선행 12게이트 `git diff --exit-code` 전부 exit 0
- [x] **P-007E-R2-R2 Stage C1** 최초 실캡처 — **`X — OBSERVATION INVALID`**
  - 독립 근거 2건: ① `HARNESS_OUTPUT_PATH_DEFECT` — `run_sham()` outdir 에 JSON 경로가 그대로 전달돼 `os.makedirs()` 가 그 경로를 디렉터리로 만들고, 최종 저장이 `IsADirectoryError` 로 죽어 **10 trial 측정값 전량 소실**
  - ② 🔴 `FORMAT_CHANGED_DURING_CAPTURE` — `cv2.VideoCapture(FOURCC 미지정)` 가 `MJPG→YUYV` 로 바꿨다. OpenCV `cap_v4l.cpp`(1차 소스, WebFetch 확인) `try_order[]` 가 YUYV 를 MJPEG 보다 먼저 시도해 생긴 일임을 **CONFIRMED** — 두 번째 근거만으로도 X. harness 버그 수정이 재실행을 정당화하지 않는다
  - 🔴 UVC 제어 예산: harness 명시적 write 0 이지만 cv2 암묵적 S_FMT ≥1 — `BUDGET DEVIATION` 으로 정직하게 기록
  - 안전 확인: video0/i2c-7 holders preflight 기준선과 동일 · 모션 가능 프로세스 0 · 이 단계가 보낸 로봇 명령 0 — 순수 소프트웨어 결함. 🔴 물리 모션 자체는 운영자 관측 채널이 없어 `NOT OBSERVED`(소프트웨어 가드 정상 ≠ 물리적으로 안전했다는 확인)
  - harness `scripts/p007e_r2r2_c1_sham.py` → `DEFECTIVE / DO NOT EXECUTE` (본문 미수정, provenance 보존). 불일치 M1~M5 기록, 수정은 **제안만**(미적용)
  - 🔴 재시도 금지(의뢰서 지시) — 고치고 바로 재실행하지 않았다. 회수한 anchor 프레임 20장은 포렌식 보존용, 수치 계산에 쓰지 않았다
  - 다음 결정은 검수자 몫: harness 수정 범위 승인 / 포맷을 MJPG 로 고정(C0-R1 필요)할지 YUYV 신규 기준선 수용할지 / C1-R1 재시도 승인
- [x] **P-007E-R2-R2 Stage C0-R1** Acquisition Harness Repair & Re-Preregistration — **`ACQUISITION REPAIR PASS / C1-R1 RE-PREREGISTERED / NO CAMERA OR ROBOT EXECUTION`**
  - 성격: STATIC REPAIR / OFFLINE VALIDATION / RE-PREREGISTRATION ONLY. 🔴 `/dev/video0` open **0회**
  - 신규 아키텍처 3+1계층 — `p007e_r2r2_acquisition.py`(NoNegotiationV4L2Reader, `v4l2-ctl --stream-mmap` 기반, S_FMT 호출 0을 1차 소스로 확인) · `p007e_r2r2_measurement.py`(M4 `roi_mean` 버그 수정) · `p007e_r2r2_analysis.py`(M5 — LOO/publication 임계·Q1~X 판정 결정적 순수함수) · `p007e_r2r2_c1r1_sham.py`(orchestration, 기본값 NO CAPTURE)
  - 🔴 구현 중 실제로 재발·발견한 버그 2건: ① `negative_tests` 의 `--out`→디렉터리 재사용(C1 과 같은 클래스 결함, `--work-dir` 분리+CLI 충돌가드로 수정) ② `RunManifest` 디렉터리 생성 누락(T8 통합 모의시험이 발견)
  - 🔴 **2라운드 자체 검증 8건 전부 반영** — 빈 opt-in stub→`execute_c1r1()` 실배선 / M5 "데이터 없이 못 짠다"는 틀린 판단→결정적 순수함수로 즉시 구현 / `roi_mean`·페이싱 미연결→`run_trial()` 실제 연결(0.25s 페이싱+2.0s trial 간격) / 사전등록 "골라담기"로 키 누락→`dict(c0)` 전체 베이스+python assert 로 누락 0 확인 / SHA 고정이 사전등록 1개뿐→acquisition·measurement·analysis 3개 모듈 SHA 고정+`--run-sham` 강제대조 / `v4l2-ctl` 단축플래그(`-v`/`-c`) 우회 가능→default-deny 로 교체 / crash 처리가 사전등록 문구와 불일치→SIGTERM/SIGHUP+`BaseException`+`atexit` 일치 / 검수자 1순위 결정사항 누락→`next_decision` 1번 승격
  - 음성시험 **T0~T12 전부 PASS** + 합성 추정기 회귀 일치(0.70305px/0.06007°, C0 와 동일) + 커밋 직전 로컬 SHA ↔ Jetson 실측 SHA 바이트 단위 일치 확인
  - 🔴 C1 의 20개 anchor 프레임은 `FORENSIC_ONLY` 로 재확인, 이번 설계에 전혀 쓰지 않았다. Stage A/B/C0/C1·선행 12게이트 `git diff --exit-code` 전부 exit 0
  - **검수 1순위 결정**: ① MJPG 복귀 선택됨 — C0-R2 에서 집행

- [x] **P-007E-R2-R2 Stage C0-R2** DaBai Pixel-Format Baseline Recovery — **`PIXEL FORMAT RECOVERED TO MJPG 640x480 / CONTROLS UNCHANGED / NO CAPTURE OR ROBOT EXECUTION`**
  - 목적: C1(invalid experiment) 이 변경한 YUYV state mutation을 Stage B 검증 baseline(MJPG 640×480)으로 **제한적으로 복구**. stationary sham 실행 아님
  - preflight 읽기전용: `/dev/video0` holders 없음 · `/dev/i2c-7` holders = pid 3091/3092(`ros2 run dofbot_pro_info kinemarics_dofbot`, 기존 관찰과 동일한 정보조회 노드, 모션 생성 노드 아님) · 모션 가능 프로세스 0
  - 실측: before = YUYV 640×480 (ALREADY_RECOVERED 아님) → 승인된 **S_FMT 정확히 1회** (`v4l2-ctl --set-fmt-video=width=640,height=480,pixelformat=MJPG`, rc=0) → after = MJPG 640×480
  - camera controls before/after `v4l2-ctl --list-ctrls` 읽기전용 비교 — brightness/contrast/saturation/hue/white_balance_automatic/gamma/gain/power_line_frequency/white_balance_temperature/sharpness/backlight_compensation/auto_exposure/exposure_time_absolute/exposure_dynamic_framerate/privacy **전부 동일**(diff 확인), 예상 밖 control 변화 없음
  - capture/stream 0 · OpenCV 0 · ROS/Arm_Lib/I2C/servo/robot motion 0 · Gemini/DaBai/DOFBOT 물리 이동 0 · 기존 evidence 수정 0
  - `evidence/P-007E-R2-R2/stage_c0r2_decision.json` + `c0r2_controls_before.txt`/`c0r2_controls_after.txt`
  - 이제 `stage_c1r1_preregistration.json` 의 `expected_device.pixelformat=MJPG` 가 실측 상태와 일치. **C1-R1 stationary sham 은 이 gate 범위 밖 — 검수 승인 대기**

- [x] **P-007E-R2-R2 Stage C1-R1** Stationary Sham Qualification (repaired harness, 첫 성공한 실캡처) — **`Q1 — OBSERVATION CHANNEL QUALIFIED`**
  - 실행 전 SHA 재검증: harness+acquisition+measurement+analysis+preregistration **5개 파일 모두** 로컬↔Jetson 배치 위치 실측이 의뢰서 명시값과 바이트 단위 일치. 배치 위치에서 T0~T12 재확인 전부 PASS 후 캡처 진입
  - `P007E_R2R2_C1R1_ALLOW_CAPTURE=1` + `--run-sham` 이중 opt-in. N_sham=10, M=60 완주(`manifest.status=COMPLETED`, `last_completed_trial=9`), 소요 약 15분
  - format invariant: 10 trial 전부 before/after match=true(MJPG 640×480 유지), 실험 종료 후 read-only 재확인도 MJPG 640×480. camera controls 종료 후 비교 결과 C0-R2 baseline과 **완전 동일**(diff 확인) — real-device S_FMT=0, camera control write=0
  - trial 결과: **10/10 VALID, dropout 0**. features 353~375 · matches 272~285 · inliers 247~260 · inlier_ratio 0.897~0.925 · residual_rms 0.44~0.48px(가드 1.50px 이내) · brightness_delta -0.07~-0.006(안정) · frame_interval_median≈0.729s(robust_sd≈0.0003s — 0.25s 목표 페이싱보다 v4l2-ctl subprocess 캡처 왕복시간이 실제 간격을 지배)
  - 집계는 **preregistered `analysis.py` classify_run 그대로** 사용(판정 로직 0 수정): robust_sigma_tx=0.0109px/ty=0.0160px/rot=0.00188° — 전부 sigma_floor(0.70px/0.060°) 이하라 **publication threshold 는 floor 로 결정**(thr_norm=4.95px, thr_rot=0.3°, K=5). LOO false_positive_count=0/10(rate 0.0), feature_dropout_rate=0.0, invalid_trial_rate=0.0, Q4 photometric `|ρ|=0.033`(0.7 미달)
  - 🔴 ~21px prediction은 이번에도 PREDICTION ONLY — threshold 계산에 미사용, Q1 결과가 "물리 모션 검증됨"을 의미하지 않음. **이 gate에 실제 robot motion 없음**
  - command budget 전부 목표치 충족: servo/Arm_Lib/ROS/raw I2C/robot controller access/모션가능명령 전부 0, `experiment_generated_motion_commands=0`, `physical_robot_motion_observation=NOT_OBSERVED`(독립 관측 채널 없음 — 소프트웨어 가드 정상과 물리적 비이동 확인을 혼동하지 않음), DaBai/Gemini 물리 이동 0
  - 예상 밖 사건(SIGNAL/크래시/SHA불일치/format위반/control변화/저장실패) 전부 미발생. 기존 evidence 수정 0
  - `evidence/P-007E-R2-R2/stage_c1r1_decision.json` + `c1r1_run1/`(manifest·10 trial JSON·20 anchor frame·aggregate_classify_run.json·trial_summary_table.json·post_experiment_readonly_check.txt)
  - 🔴 **advisor 재검토 후 공개 추가(`stage_c1r1_disclosures.json`, decision.json 은 amend 안 함)**: ① i2c-7 holder 집합이 C0-R2(3091/3092)와 C1-R1(2264/3091/3092)에서 달랐다 — pid 2264=`yahboom_oled.py`, `b0_i2c_fd_classification.json` 기지의 순환 i2c 버스 탐색 서비스(≈4s/바퀴)와 정확히 일치, `/dev/video0` holders 와는 무관 ② frame_interval 실측≈0.729s/frame(목표 0.25s 의 약 2.9배, window≈43.7s·trial≈92.5s·총 15.4분) — 사전등록 `session_per_frame_characteristic`이 이미 "미검증"이라 명시한 현상이며, 사전등록 전체에 `abort` 규칙 자체가 없어 self-judgment 범위 밖(PROTOCOL DEVIATION 으로만 분류, 판정에 미반영) ③ aggregate 는 harness 가 아니라 이 세션이 harness 밖에서 `analysis.py`(SHA 재확인 일치)를 그대로 import 해 계산 — harness 에 `summary/sham_summary.json` writer 가 없다는 결함 발견(수정 안 함) ④ manifest 의 `base_commit="64b32bc"`(harness 리터럴, 실제는 acc59bd)·`preregistration_commit="<set at commit time>"`(치환 안 됨)은 HARNESS_METADATA_DEFECT 로 기록(측정값에는 영향 없음) ⑤ Jetson 원본을 사전 sha256 diff 없이 삭제(비가역) — trial JSON 10개는 파싱 성공으로 부분적 무결성만 확인, JPG 20장·manifest 는 미검증
  - **검수자가 noise/LOO/dropout/photometric/threshold 와 위 공개 5건을 확인한 뒤에만 다음 physical-motion observation gate 가 열린다. 이번 세션은 여기서 STOP**

- [x] **P-007E-R2-R2 Stage C1-R1-R1** Evidence Pipeline Closure (정적 repair, C1-R1 재실행 아님) — **`EVIDENCE PIPELINE CLOSURE PASS / FUTURE EXPERIMENT PROVENANCE READY (조건부 — git-tracked checkout 실행 시만) / NO CAMERA OR ROBOT EXECUTION`**
  - C1-R1 종결 라벨(검수자 판정 `MEASUREMENT PASS / QUALIFICATION CONDITIONAL / CLOSED @ f92e155` · harness 사전등록 로직 출력 `Q1 — OBSERVATION CHANNEL QUALIFIED`)·실측값(10/10·FP 0/10·dropout 0·σtx 0.01089px/σty 0.01596px/σrot 0.00188°)·integrity defect(TRANSFER_INTEGRITY_NOT_VERIFIED_BEFORE_SOURCE_DELETION) 전부 소급 변경 없이 보존. ROI/ORB/RANSAC/K·M·N/sigma floor(0.70px·0.060°)/threshold formula/LOO/Q1~X 로직 1바이트도 변경 없음(analysis.py sha256 불변)
  - 🔴 **커밋 전 advisor 재검토에서 BLOCKING 3건 발견 → 전부 수정 + Jetson 재시험 후 커밋**: ① `artifact_sha256.json` 이 manifest 최종쓰기 전에 생성돼 자기참조 불일치(TRANSFER_FAILED 유발) → 쓰기 순서 재배치(summary→manifest 최종쓰기→artifact_sha256.json→self-verify), finalize 예외 시 CRASHED+INCOMPLETE 로 되돌림 ② `git -C`가 저장소 아닌 위치에서 상위 디렉터리로 올라가 엉뚱한 저장소를 조용히 기록할 위험 → `rev-parse --show-toplevel` 검증 + `GitProvenanceUnavailable` fail-closed(reader 생성 전 ABORT) ③ C1-R1 라벨을 "Q1 보존"으로만 적어 검수자의 실제 종결 라벨과 불일치 → 라벨 두 가지 모두 명시하도록 수정. T10/E4 의 `self_transfer_verification=TRANSFER_VERIFIED`, E16(provenance 실패→ABORT, manifest 미생성)·E17(dirty tree+require_clean→ABORT) 로 재확인
  - `evidence/` 디렉터리는 전부 A(추가)뿐, M(수정) 0건(`git diff --name-status f92e155 -- evidence/ scripts/` 실측). `scripts/p007e_r2r2_c1r1_sham.py` 만 M — harness 소스 자체의 의도된 repair(evidence json 아님)
  - 🔴 신규 `scripts/p007e_r2r2_evidence_pipeline.py` — Repair A(runtime git provenance: git_head/base_commit/preregistration_commit 전부 실행시점 조회로 교체, `"64b32bc"` 하드코딩 리터럴 제거) · Repair B/C(capture 완료 즉시 같은 프로세스에서 frozen analysis.py 호출해 summary/sham_summary.json 자동생성 — 별도 session 재계산 없음) · threshold provenance(observed/floor/effective/floor_dominated 축별 명시) · frame timing first-class metric · Repair D(artifact_sha256.json) · 전송무결성 verify_transfer + 기본값 NO DELETE(explicit opt-in 없으면 항상 거부) · Repair E(필수 artifact contract 완전성 가드)
  - §24 quasi-static 관측 semantics를 `stage_c1r1r1_architecture_note.json` + 코드 상수(`QUASI_STATIC_SEMANTICS_NOTE`, future summary에 자동 포함)로 명시 — "dynamic trajectory tracking 아님, pre/post quasi-static pose 비교만", controller readback과 DaBai observation은 독립 채널
  - 🔴 음성시험 T0~T12 회귀 전부 PASS(T10은 `*_module_sha256` 필드명 변경·mock git 주입으로 동작은 동치 유지) + selftest 추정기 회귀 일치(0.70305px/0.06007°, C0와 동일) + 신규 **E1~E17 전부 PASS**(런타임 provenance/stale literal 제거/summary 자동생성(실행값 M/N_sham 기록)/summary SHA/incomplete package/artifact SHA manifest/transfer 성공·불일치·누락·corrupt JSON/기본 NO DELETE/frame timing/threshold floor-dominated 양쪽 케이스/no-camera(poison-stub 실증)/provenance 실패 ABORT/dirty-tree ABORT) — 전부 Jetson에서 cv2만 쓰고 카메라·로봇 0
  - 🔴 **배포 아키텍처 제약 발견·공개**: ROOT가 git 저장소가 아닌 위치(Jetson scp 개별파일 배치)에서 실제로(`git_run_fn=None`) `execute_c1r1()`을 돌리면 `GitProvenanceUnavailable`로 fail-closed ABORT된다(advisor 재검토로 구조화됨, 이전에는 구조화되지 않은 traceback이었다) — 이번 gate의 모든 Jetson 시험은 mock `git_run_fn`으로 수행했다. 다음 physical-motion gate는 harness가 git-tracked checkout에서 실행되는 배포 방식을 설계해야 한다(이번 범위에서 해결 안 함)
  - 🔴 **자기참조 오탐이 이번 gate 구현 중 3회 추가 재발**(i2c·VideoCapture 선례 포함 프로젝트 전체 5회) — `"Video"+"Capture("` 상수를 새로 안 쓰고 리터럴을 직접 적어 오탐, 그 오탐을 설명하는 주석 자체가 재오탐, E2의 historical literal을 설명문에 그대로 적어 또 오탐. 전부 문자열 조립으로 수정, feedback 메모리로 일반화 기록
  - 실제 remote 파일 삭제는 이번 gate에서 시험하지 않음(mock 디렉터리 2개 비교만). command budget 전부 0(camera/ROS/Arm_Lib/servo/torque/RGB/raw I2C/robot access/motion-capable/물리이동)

- [x] **P-007E-R2-R3** Controlled Physical-Motion Observation Preregistration — **`CONTROLLED PHYSICAL-MOTION EXPERIMENT PREREGISTERED / NO CAMERA OR ROBOT EXECUTION`**
  - authoritative controller state 재확인: ID1..6 = `[86, 152, 0, 18, 93, 178]` — 기존 ID2=150 전제 harness(`scripts/p007e_phase1_command.py` 등)는 계속 STALE/DO NOT EXECUTE
  - **Block 3 명칭 통일**: "Block 3 — Eye-in-Hand Visual Motion Witness"(Pre/post quasi-static visual change against a rigid external scene). "Independent External/Robot Pose Observer/Measurement", "Absolute Robot Pose/Joint-Angle Measurement", "Dynamic Trajectory Tracker" 명칭 전부 금지 — DaBai는 Arm4_Link eye-in-hand sensor라 외부 독립 관찰자가 아니다. `evidence/P-007E-R2-R2/stage_c1r1r1_architecture_note.json`은 소급 수정하지 않음(불변 보존), Block 3는 이번 gate 신규 문서에서 처음 정의
  - **두 evidence channel 분리**: Channel A(controller-reported servo position, physical shaft motion 단독 증명 불가) / Channel B(DaBai visual witness, absolute joint angle 측정 불가) — 독립 채널, 같은 물리량 아님
  - **A/B/C/D/X 결과행렬 + claim mapping 동결**(pure function, 데이터 보기 전 고정): A=`PHYSICAL_MOTION_EVIDENCE_SUPPORTED`, B=`PHYSICAL_MOTION_NOT_ESTABLISHED`, C=`UNEXPECTED_VISUAL_MOTION`, D=`NO_OBSERVABLE_RESPONSE`, X=`EXPERIMENT_INVALID`. 결과 A라도 "정확한 물리 각도=controller 값/2.000° 물리 회전 검증/절대 로봇 pose 검증/trajectory·calibration 정확도 검증"은 아직 주장하지 않음
  - **명령 geometry 동결**: `Arm_serial_servo_write(id=2, angle=154, time=2000)` 1회만 허용(ID2 152→154, +2°, 2000ms). write6/write6_array/write_any/PWM/action group/MoveIt/trajectory command 전부 금지. 자동 복귀(154→152, 152→150) 금지 — `reject_automatic_return()`으로 탐지
  - **HOST-CONTROL QUIET INTERVAL**(I2C BUS QUIET/BUS SILENCE 명칭 금지 — yahboom_oled 가 공유 i2c 버스를 쓸 수 있어 "버스 전체가 조용하다"는 주장은 성립하지 않음) 최소 60ms로 동결(P-007E-R1 실측 60.099ms 채택)
  - **pre-motion hard gate** 14개 항목 전부 PASS 요구(git provenance/preregistration·harness·module SHA/DaBai identity/MJPG 640×480/camera control invariant/PRE visual window/ROI feature guards/controller baseline/unexpected process/host-control quiet interval/evidence writable/artifact path) — 하나라도 실패시 ABORT BEFORE MOTION. PRE window(M=60) 불량 시에도 **motion 보내기 전에** abort(명령 보낸 뒤 카메라 불량을 발견하는 구조를 피함)
  - **git-tracked execution 요구사항**: C1-R1-R1 에서 발견된 "scp 배치=git checkout 아님→GitProvenanceUnavailable" 문제를 해결하기 위해, 이 harness 는 `scripts/p007e_r2r2_evidence_pipeline.py`의 `check_dirty_tree()`/`build_runtime_provenance()`를 **재사용**(중복 구현 없음)한다. MODULE_SHA 강제 대상에 `p007e_r2r2_evidence_pipeline.py`를 추가(§26)
  - 🔴 **구조적 안전 설계**: `scripts/p007e_r2r3_motion_preregistration.py`는 Arm_Lib/smbus/rclpy/cv2/subprocess를 **직접 import·호출하지 않는다**(self_ast_check()로 보장). camera_check_fn/capture_pre_window_fn/capture_post_window_fn/controller_read_fn/controller_write_fn 이 전부 주입형 — opt-in 환경변수(`P007E_R2_R3_ALLOW_MOTION=1`)·`--execute-motion`·운영자 확인을 전부 참으로 설정해도 이 콜백들이 주입되지 않으면 `pre_motion_hard_gate`가 ABORT_BEFORE_MOTION 하거나 `HARDWARE_WIRING_NOT_IMPLEMENTED_IN_THIS_STAGE`로 abort — 이 파일 자체로는 실제 로봇/카메라에 닿을 방법이 구조적으로 없다. 실제 하드웨어 wiring(estimator 구성 포함)은 다음 EXECUTION gate에서 추가
  - **negative test 22종(R3-01~R3-22) + 확장시험 9종(R3X-01~09) 전부 PASS** — 전부 mock/synthetic, 실제 camera/robot 0
  - 🔴 **구현 중 자체발견 버그**: `check_preregistration_sha()`의 기본 파라미터 `expected=PREREG_SHA`가 모듈 로드 시점에 바인딩되는 Python late-binding 함정으로 R3-06(SHA mismatch) negative test 가 최초 실행에서 실패(exit 1) — `expected=None`으로 바꿔 호출 시점에 전역을 다시 읽도록 수정
  - 🔴 **커밋 전 advisor 1차 재검토에서 BLOCKING 5건 발견 → 전부 preregistration·harness 재작성 후 재시험**: ① A/B/C/D/X 입력(controller_changed/visual_changed/experiment_valid) 계산규칙이 동결 안 됨 → P-007E-R1 선례(readback_change_criterion, 3/5라운드·20ms spacing·분산 0 실측)로 controller_changed 규칙을, run_trial() 의 delta 공식으로 visual_changed 구성을, X-trigger 4종을 명시 동결 ② §11 tolerance=0 이 근거 없음 → 위 P-007E-R1 실측으로 근거 보강 ③ `pre_motion_hard_gate()`가 정의만 되고 실행경로에 미연결 → write 직전 단일 결정점에서 14개 항목 전부 평가하도록 재작성 ④ write 이후 settle/controller post-read/POST window/Δ계산/결과행렬/저장이 전부 없음 → `execute_motion_experiment()`를 끝까지 구현 ⑤ harness 자신의 SHA가 실행경로에서 강제되지 않음 → `expected_harness_sha256` 파라미터 주입 방식으로 해결(자기참조 순환 회피)
  - 🔴 **커밋 전 advisor 2차 재검토에서 BLOCKING 4건 추가 발견(그중 1건 치명적) → 전부 재수정 후 재시험**: ① **(치명적)** `capture_pre_window_fn`/`capture_post_window_fn` 이 각자 자기 첫 프레임을 anchor 로 쓰면 Δ≈0 으로 수렴해 **실제 물리 이동이 있어도 항상 B/D 로 오판**하는 결함(run_trial() 이 PRE/POST 공통 anchor_gray 하나만 쓰는 것과 다름) → `capture_pre_window_fn()`이 `(window_dict, anchor)`를 반환하고 `capture_post_window_fn(anchor)`가 그 **동일 객체**를 받는 계약으로 변경, R3X-11 로 anchor identity(`is`) 검증 ② write 이후 구간에 try/except 가 전혀 없어 예외 발생 시 "명령을 보냈다"는 기록 자체가 안 남음(C1 전량소실과 같은 모드, 물리명령까지 걸림) → write 호출 자체와 그 이후 구간을 각각 `try/except BaseException`으로 감싸 write 직후·최종 2회 저장, `WRITE_CALL_EXCEPTION`/`POST_WRITE_EXCEPTION`을 동결된 X 트리거 목록에 추가(총 6종), R3X-12(POST 콜백 raise 시 디스크 내용 직접 확인)·R3X-13(저장 자체 실패) 로 검증 ③ controller pre-read 가 PRE window/process check/writable check 보다 먼저 수행되어 `quiet_interval_ms`가 수십 초 묵은 값을 60ms 라고 거짓 기록할 위험 → pre-read 를 motion 직전(quiet sleep 바로 전)으로 재배치 ④ 코드(엄격: 라운드 전부 non-None)와 preregistration 문구(완화: 1개 이상 non-None)가 불일치 → 엄격 규칙으로 통일, R3X-10([152,None,152] abort) 으로 검증. **negative test 22종 + 확장시험 13종(R3X-01~13) 전부 PASS**(R3X-01~05 로 A/B/C/D/X 전부 생성 확인, write 정확히 1회·인자 `(2,154,2000)`)
  - `run_trial()`(C1-R1/C1-R1-R1 CLOSED, 불변) 은 §20 의 "PRE 불량 시 motion 전 abort" 요구를 구조적으로 만족시킬 수 없어(중간 결정점 없음) 재사용하지 않고 PRE/POST 캡처를 분리된 주입형 콜백으로 바꿨다 — sham.py는 1바이트도 수정하지 않았다
  - 🔴 **커밋 전 advisor 3차 재검토에서 BLOCKING 3건 추가 발견 → 전부 재수정 후 재시험(최종)**: ① write 직후 중간 저장이 `status='EXPERIMENT_COMPLETE'`로 남아 그 직후 프로세스가 죽으면(SIGKILL/전원차단/SSH 끊김) **끝나지 않은 실험이 COMPLETE로 잘못 기록**되는 문제(2차 BLOCKING 2가 막으려던 시나리오 그대로 재발) → 중간 저장은 `status='COMMAND_SENT_IN_PROGRESS'`로, 최종 저장에서만 `EXPERIMENT_COMPLETE`로 갱신. `check_r3_artifact_completeness()`에 `result['status']==EXPERIMENT_COMPLETE` 조건 추가, R3X-14(정상완주 시 COMPLETE 확인)·R3X-15(POST 콜백 안에서 디스크를 직접 읽어 중간상태가 IN_PROGRESS+motion_commands_sent=1이고 completeness가 INCOMPLETE인지 확인 — "프로세스가 그 자리에서 죽은 상황"을 흉내) 로 검증 ② R3-05~10/R3X-06~08/10 이 전부 `run_root=None`으로 실행돼 `evidence_package_writable`이 항상 같이 실패해 "각 항목 단독 실패" 주장이 시험으로 입증되지 않던 문제 → 전부 쓰기 가능한 tempdir + write 추적 콜백으로 바꾸고 `failed_checks`를 정확히 비교(예: R3-05==`['git_provenance_valid']` 단독) ③ 동결 문서(prereg) 안에 anchor 계약과 모순되는 문구(인자 없는 `capture_post_window_fn()`, 존재하지 않는 `reader_factory` 참조)가 남아 있어 다음 구현자가 그 줄만 읽으면 치명적이었던 anchor 결함이 재발할 위험 → 전부 현재 계약(anchor 전달)에 맞게 수정. **negative test 22종 + 확장시험 15종(R3X-01~15) 전부 PASS**로 최종 확정, 이후 advisor 재호출 없이 커밋
  - command budget 전부 0(camera starts/new captures/servo reads·writes/robot controller access/motion-capable/torque/RGB/raw I2C/ROS/ros2 CLI/DaBai·Gemini·DOFBOT 물리이동). **기존 evidence(C0-R2/C1-R1/C1-R1-R1) 수정 0건 — 전부 A(신규)**
  - `evidence/P-007E-R2-R3/`(`stage_r3_preregistration.json`·`stage_r3_architecture_note.json`·`stage_r3_decision.json`) + `scripts/p007e_r2r3_motion_preregistration.py`
  - 🔴 **성공해도 실제 Arm2 152→154 명령을 보내지 않음** — 검수자가 preregistration/command geometry/visual witness 조건/evidence pipeline/safety gate를 확인한 뒤 별도의 실행 승인을 내려야 다음 EXECUTION gate가 열린다. 이번 세션은 여기서 STOP

- [x] **P-007E-R2-R4** Motion Execution Safety Preflight — **`MOTION EXECUTION SAFETY PREFLIGHT PASS / ARM2 152→154 CANDIDATE READY FOR SEPARATE EXECUTION APPROVAL / NO CAMERA OR ROBOT EXECUTION`**
  - **입력 확보(read-only)**: Jetson 에서 URDF(`DOFBOT_Pro-V24.urdf`, sha `b626a845...`, 기존 `fk_clearance.json`과 동일 확인)·MoveIt SRDF(`DOFBOT_Pro-V24.srdf`)·13개 collision/visual STL mesh(총 51MB)를 scp 로 가져와 `.cache/jetson_robot_model/`(gitignored)에 저장, local↔remote sha256sum 14개 파일 전부 일치 확인(전송무결성, 원본 Jetson 삭제 안 함). `Arm_serial_servo_write/read`(id∈{2,3,4}) 소스도 ssh 로 read-only cat 하여 공식(180-angle 반전, `pos=int((3100-900)*angle/180+900)`, register=`0x10+id`) 직접 재확인(import 없음)
  - 🔴 **tree FK 신규 구현**(`scripts/p007e_r2r4_fk.py`) — 기존 `scripts/p007b_fk.py`는 직렬 체인(Arm1~5+Gripping_point)만 다루는데 실제 URDF는 Arm4→DaBai_DCW2_Link(fixed), Arm5→grip_joint/llink_joint1/rlink_joint3/llink_joint3/Gripping_point_Joint 로 가지가 갈라진다 — parent/child 트리 전체를 순회하는 FK로 재구현(H0 convention·회전행렬 수학은 동일, 재검증 후 재사용)
  - 🔴 **회귀 선행(advisor 권고, 152→154 계산 전에 먼저 통과시킴)**: ① `p007b_fk.py --self-check`(Arm1-3 @ 86/150/0/18/93) 오차 0.0 재현 ② `evidence/P-007E/fk_clearance.json`의 150→155 전체 chain_link_origins(Arm1-5+Gripping_point) + TCP 13.67mm + axis_dir`[0.997564,-0.069756,0]`/axis_origin`[0,0,122.5]mm` 전부 오차 ≤0.005mm(표시정밀도 반올림) 재현 ③ P-007B의 +2°(150→152) TCP 5.47mm → 재현값 5.47095mm ④ P-007A `Arm3_Joint` margin_to_lower=3.673205103416066e-06 rad → float 완전일치 재현. 4건 전부 PASS 확인 후에만 152→154 실제 계산 착수
  - **152→154 계산**: TCP 변위 5.471mm, DaBai 변위 5.438mm/회전 정확히 2.0°(강체부착이므로 Arm2 회전과 100% 동일). 불변식 직접 검증: base_link/Arm1_Link 불변(오차 0), 하위 전체 링크(그리퍼 포함) `R_post=Rot(axis_dir,2°)@R_pre`/`p_post=axis_origin+Rot@(p_pre-axis_origin)` 오차 2e-16~6e-17(기계정밀도)
  - **swept displacement 이중 계산**: (a) EXACT — STL 정점(mesh_R 를 실제 STL에서 재계산, 선례 12개 링크 값과 ≤0.005mm 일치) 을 PRE pose로 base frame 변환해 Arm2 축 수직거리 최댓값×2sin(1°) = **5.93mm(DaBai_DCW2_Link)**, 그리퍼 주 구동(grip_joint)은 자신의 limit [-1.6,0]rad 을 9점 샘플링(passive linkage는 0 rad 고정, 잔여 한계로 공개) (b) sphere-bound cross-check(선례와 동일 방법) = **8.46mm(Gripping_point_Link)** — 전 링크에서 sphere≥exact 실측 확인
  - **joint limit**: 5개 arm joint 전부 PASS, Arm3(lib 0°)의 margin_to_lower≈3.67e-6 rad(모델 하한선에 사실상 걸쳐 있음)을 "여유 있음"이라 쓰지 않고 수치 그대로 공개
  - **self-collision**: 1차 근거는 벤더 MoveIt SRDF(`DOFBOT_Pro-V24.srdf`, author yahboom, 2025-02-18 생성) — 133개 link pair **전부**가 Adjacent/Never/Default 로 disable되어 있음(Arm1_Link-Arm2_Link=Adjacent, Arm2_Link-base_link=Never, 전체 joint 범위 포괄). 2차로 FK 기반 spot-check(stride=40 다운샘플, scipy 미사용) — Arm1-Arm2 최소거리≈0.13mm(접촉, Adjacent와 정합)·base-Arm2≈16mm(간극, Never와 정합), PRE(152°)→MID(153°)→POST(154°) 전체에서 두 값 모두 안정(변화<0.1mm)
  - **command raw encoding**: raw(152)=1242(P-007E-R1 선례와 정확히 일치) 확인 후 raw(154)=1217 계산, register=`0x12`, payload=`[0x04,0xC1,0x07,0xD0]`. raw154 정확히 안착 시 예상 readback=155(152→153과 동일한 +1 int-truncation 패턴 — command semantics 불일치로 오판하지 않도록 사전 홀드)
  - **conservative operator envelope**: readback=152 raw bucket[1243..1254]과 target raw154=1217 의 최대 격차에서 Δ_max=3.027° 도출(실측 데이터 기반) → sphere-bound worst-link 기준 D_model_at_Δmax=12.81mm + 고정·비실측 model_uncertainty_margin 2mm(STL/CAD-실물 치수 불일치 대비, 명시적으로 "비실측" 공개) = **envelope ceil(14.81)=15mm**
  - 🔴 **R3(CLOSED, SHA `f8eda225...`) 는 1바이트도 수정하지 않고 import 재사용** — `scripts/p007e_r2r4_preflight.py`가 R3의 `execute_motion_experiment`/`validate_command_request`/`reject_automatic_return`/`motion_result_matrix`/`physical_motion_claim`을 그대로 호출, MODULE_SHA 로 변경 없음을 증명(R4-21). R4가 새로 추가한 것은 ① workspace-safety hard gate(§9, R3엔 없던 8항목 attestation, R3 호출 전에 평가) ② write-ahead 재전송방지 ledger(§15~17, R3의 `controller_write_fn` 주입점을 감싸 write 호출 **직전** ARMED 를 `.cache/p007e_r2r4_ledger/`(gitignored, dirty-tree guard 혼동 방지)에 원자적 기록 — R4-19로 write 중 crash 시에도 ARMED 가 남는 것을, R4-20으로 같은 experiment_id 재시작 시 재전송 차단을 직접 확인) ③ §13 서술적 순서 대비 실제 호출순서 기록·비교(dry validation) — 순서 차이 2건 발견했으나 둘 다 §14의 진짜 안전불변식(명령은 어떤 hard gate 보다도 먼저 오지 않음, R4-16/17/18로 write 호출수 0 직접확인)을 위반하지 않아 non-blocking 으로 공개
  - **negative test 25종(R4-01~R4-25) 전부 PASS** — FK 재현성, joint limit, self-collision, swept displacement, raw encoding, 명령 validator(ID/target/duration/count), 자동복귀 거부, PRE-validation/operator-ready/workspace-safe 이전 명령 거부, crash 후 ARMED 보존, 재시작 재전송 차단, A/B/C/D/X 불변, Gemini 미접근(조립식 문자열로 자기참조 오탐 1회 자체발견·수정, 프로젝트 6번째 재발) 전부 확인. command budget 전부 0
  - `evidence/P-007E-R2-R4/`(`fk_pre.json`·`fk_post.json`·`swept_motion.json`·`collision_check.json`·`command_encoding.json`·`joint_limits.json`·`execution_state_machine.json`·`preflight_tests.json`·`decision.json`) + `scripts/p007e_r2r4_fk.py` + `scripts/p007e_r2r4_preflight.py`. 기존 evidence(P-007E-R2-R2/R2-R3) 전부 불변
  - 🔴 **성공해도 실제 Arm2 152→154 명령을 보내지 않음** — 검수자가 R4 수치(특히 swept displacement·crash/재전송 방지)를 확인한 뒤 별도의 실행 승인을 내려야 다음 EXECUTION gate가 열린다. 이번 세션은 여기서 STOP

- [~] **P-007E-R2-R5** Single Controlled Physical-Motion Execution — **코드/준비상태 구축 완료, 실제 실행은 운영자 대기 중** (advisor 생략 지시 이후 진행, codex/advisor 재검토 없음)
  - 🔴 **최초로 실제 DaBai capture/controller read/Arm2 servo write(≤1회)를 허용하는 gate** — R3(CLOSED SHA `f8eda225...`)·R4(CLOSED SHA `cc5dd7bc...`)를 1바이트도 수정하지 않고 SHA-pin import 로 재사용. 신규 4개 파일: `scripts/p007e_r2r5_window.py`(run_trial() 단일-window 루프를 분리호출 가능하게 재구성, 알고리즘 재구현 없음, anchor 계약 보존) · `scripts/p007e_r2r5_execution.py`(하드웨어 0, self_ast_check 로 구조보장, durable ledger+재시작보호+§31 승인게이트) · `scripts/p007e_r2r5_hardware_wiring.py`(Jetson 전용 실제 Arm_Lib/v4l2-ctl wiring, p007e_phase1_command.py 선례와 동일 계층) · `scripts/p007e_r2r5_run.py`(실제 실행 CLI, `--check-only`/`--execute`)
  - 🔴 **§31 승인게이트 설계(self-discovered, advisor 없이 자체 검증)**: 승인 문구 확인을 R3.execute_motion_experiment()가 controller_write_fn 을 호출하는 그 시점(코드 흐름상 R3 내부 14항목 하드게이트가 전부 PASS 한 뒤에만 도달하는 지점 — 직접 읽어 확인한 구조적 불변식)에 둔다. real PRE capture(M=60)+controller baseline read 를 승인용으로 한 번, 실행용으로 또 한 번 하면 §30 camera-session 예산(1회)을 어기므로, 단일 real R3 호출 안에서 write 호출 시점에 승인을 받는 구조를 택했다. 거부 시 ledger 를 ARMED 하기 전에 `ApprovalDeclinedBeforeArming` 을 raise — 이 exp_id 의 ledger 에는 어떤 항목도 생기지 않고 실제 Arm_Lib 호출도 0회(`real_hardware_write_invoked=False`)
  - 🔴 **공개 한계**: 거부 경로에서 R3(수정하지 않음) 자신의 범용 `except BaseException` 이 `result_code="X", motion_commands_sent=1, invalid_reasons=["WRITE_CALL_EXCEPTION"]` 로 기록한다 — 이는 R3 자신의 기존 fail-closed 관례가 "애초에 승인하지 않았다"는 사례에도 적용된 결과일 뿐 실제 하드웨어 접촉의 권위 있는 기록이 아니다. 권위 있는 기록은 ledger(ARMED/SENT 항목 없음)와 `real_hardware_write_invoked=False` — R5-20 negative test 로 이 모순이 결과에 `🔴_known_r3_accounting_note` 로 공개되는 것을 확인
  - 🔴 **self-discovered 버그 1건(advisor 없이 자체발견·수정)**: `scripts/p007e_r2r5_run.py` 초안이 R5 execution harness 자신의 SHA를 "런타임에 그 파일을 다시 읽어 계산한 값"을 그대로 expected 로 되돌려주는 self-referential 검사였다 — 변조를 전혀 못 잡는 타당성 없는 검사(R3/R4가 서로의 SHA를 상수로 고정하는 이유와 동일) → `R5_EXEC_SHA` 고정 리터럴 상수로 교체, dead code(`_r5_self_sha`) 제거
  - 🔴 **환경 조사(비차단, 공개)**: Jetson `/dev/i2c-7` holder 중 `ros2 run dofbot_pro_info kinemarics_dofbot`(및 launcher wrapper)를 신규 발견 — 소스(`kinemarics_dofbotpro.cpp` 전체 123줄 직접 읽음) 확인 결과 KDL 순수 수학 FK/IK 서비스로 I2C/Arm_Lib 코드 0, 실제 spin 중인 컴파일된 바이너리(별도 PID)는 i2c-7 fd 를 들고 있지 않음(launcher 의 inherited fd 일 뿐, fd 생성 2일 전) → 능동적 I2C 사용자는 기존 문서화된 yahboom_oled.py 뿐임을 재확인, §10 전제 유지. `i2c7_holder_diagnostic()` 은 기록만 하고 게이트 판정에는 쓰지 않음(R3 의 `env_guards()` 그대로)
  - **입력 확보(read-only)**: Jetson 에 git-tracked checkout 신설(`git clone` → `/home/jetson/cad-less-robotics`, HEAD `70071d5` 로컬과 일치 확인, R5가 §6 git-tracked checkout 실행을 처음 요구) · `v4l2-ctl --info`로 DaBai 식별(`Serial: CH8R3620170` 실측 일치) · `--list-ctrls`로 camera_control_invariant 6항목 전부 Stage B baseline과 일치 확인 · Arm_Lib disk/egg SHA(`edfcae72.../3aba3583...`)가 `p007e_phase1_command.py` 선례와 동일함을 재확인
  - **self-test**: `p007e_r2r5_execution.py --self-test` R5-01~20 전부 PASS(opt-in/workspace/ledger-restart/승인거부-ledger미기록/승인수락-ARMED→SENT/write정확히1회/decline 공개노트 등) + `p007e_r2r5_hardware_wiring.py` HW-01~04 전부 PASS(v4l2 control parser 를 2026-10-28 실측 샘플로 검증) — 전부 하드웨어 0
  - `evidence/P-007E-R2-R5/`(`stage_r5_design_note.json`·`stage_r5_preflight_tests.json`) + `scripts/p007e_r2r5_{window,execution,hardware_wiring,run}.py`. 기존 evidence(P-007E-R2-R2/R2-R3/R2-R4) 전부 불변
  - 🔴 **`--execute`(실제 §31 흐름, 운영자 확인+workspace 체크리스트+승인 문구)는 Claude 가 직접 실행하지 않는다** — Claude 는 로봇을 육안으로 관찰할 수 없어 §12/§13 을 대신 확인할 수 없다. `--check-only`(opt-in 0, 서보 read 조차 하지 않음)까지만 Claude 가 Jetson 에서 직접 실행해 준비상태를 확인하고, 실제 승인은 로봇을 지켜보는 운영자가 직접 입력한다

- [x] **P-007E-R2-R5 Provenance Self-Contamination Repair** (commit `d02611e`)
  - 🔴 **Attempt #3 root cause**: R5 가 생성하는 `evidence/P-007E-R2-R5/run_*/`(untracked)가 다음 실행의 git dirty-tree 검사(R3/evidence_pipeline, CLOSED, 수정 안 함)에 그대로 잡혀 매 실행마다 `git_provenance_valid=false → ABORT_BEFORE_MOTION` 이 반복되는 구조였음 — harness 가 자기 자신의 evidence 로 스스로를 막는 결함(하드웨어 write/controller write/physical motion 전부 0회, safety 거동 자체는 PASS)
  - 🔴 **수정은 R5 자신의 wiring 계층에만 추가** — `scripts/p007e_r2r5_hardware_wiring.py` 에 `make_r5_provenance_filtered_git_run_fn()` 신설: 이미 있던 `run_fn` 주입점(원래 mock 음성시험용)에 real-git-call wrapper 를 꽂아 `status --porcelain` 응답에서만 `evidence/P-007E-R2-R5/run_` prefix 로 시작하는 untracked 줄을 제거. R3/R4/evidence_pipeline 1바이트도 수정 안 함, `git_provenance_valid=True` 하드코딩 안 함, `.gitignore` 로 숨기지 않음, 기존 evidence 두 디렉터리(`run_20261028T205930`·`run_20261028T210701`) 삭제/수정 안 함
  - **regression test(hardware-free) HW-05~13 전부 PASS**: A clean tree PASS / B R5 run_* 만 존재 PASS / C tracked modification FAIL / D staged modification FAIL / E 임의 untracked root 파일 FAIL / F 다른 gate(R2-R2) untracked artifact FAIL(기존 정책 유지) / G repo-root/HEAD mismatch 시 `GitProvenanceUnavailable` 그대로 발생 / H R5 자신의 module SHA·self_ast_check 영향 없음 / 혼재 시나리오(허용줄+실제 수정줄)도 실제 수정줄 때문에 여전히 FAIL. 기존 R5-01~20 전부 PASS 유지
  - Jetson 실기 재확인(run_* 2개 존재 상태): `--check-only` 의 `git_dirty_tree_check={"dirty": false}`, `overall_check_only_pass=true` 로 전환 확인

- [x] **P-007F-01** Controller Readback vs Physical Servo Motion / Read-Only Diagnosis — **`READBACK_SEMANTICS_UNRESOLVED` / `TORQUE_STATE=UNKNOWN` / `SERVO_COMMUNICATION=SUPPORTED`**
  - 🔴 **배경**: R5 Attempt #4(운영자 직접 실행)에서 실제 write 1회 발생, controller readback `152→154`, 그러나 operator 관찰 physical Arm2 motion = **NO**. 공식 Yahboom 문서(torque=1 시 각도변경·토크유지, torque=0 시 수동이동 가능) 재검토 후 "write→readback 변화→무동작"을 정상 최종상태로 받아들이지 않기로 하고 신규 motion 0건으로 원인 조사
  - **Task A**: 실제 설치 Arm_Lib 1.0.3(`/home/jetson/software/Arm_Lib/Arm_Lib/Arm_Lib.py`, sha `edfcae72...`, 471줄 전체) read-only cat 으로 전체 분석 — 🔴 **아키텍처 전제**: Jetson 은 bus servo 와 직접 통신하지 않고 I2C addr `0x15` 단일 MCU 에만 질의하며, 그 MCU 펌웨어 소스는 이 시스템에 없음(readback semantics 판별불가의 근본 원인). write API 전부(위치/토크/리셋/캘리브레이션/액션/PWM/RGB/버저/raw bus_servo_control) register 매핑 표로 정리, torque 읽기 API 자체가 소스에 **존재하지 않음** 확인
  - **Task B**: H1(실측)/H2(캐시)/H3(판별불가) — source 만으로 구별 불가, `READBACK_SEMANTICS_UNRESOLVED`(H3)로 확정. 🔴 **corroborating(결정적 아님) 발견**: `Arm_serial_servo_read(2)`(register `0x32`, 반전적용)=154 와 `Arm_serial_servo_read_any(2)`(register `0x37`, 반전미적용)=26 이 `180-154=26`으로 정확히 일치 — 두 독립 레지스터 경로가 모순없는 단일 raw 값을 공유함을 보이나, 그 raw 값이 실측인지 캐시인지는 여전히 판별 불가(같은 캐시 변수를 참조해도 똑같이 일치 가능)
  - **Task C**: read-only probe(새 motion 0건) — `Arm_serial_servo_read` 5라운드×ID1-6(20ms 간격, ID2=154 완전 안정 분산 0) · `Arm_serial_servo_read_any(2)` 5라운드(=26, 교차검증) · `Arm_ping_servo(1..6)` 전부 `0xda`(정상, 통신 레이어 응답 확인) · `Arm_get_hardversion()`="0.20". Arm_Lib disk/egg SHA 재확인(`edfcae72.../3aba3583...`)
  - **Task D**: `/dev/i2c-7` 소유권 재조사(kill/restart 0) — `yahboom_oled.py`(기존 문서화된 능동 사용자) + `kinemarics_dofbot` launcher wrapper(소스 확인상 I2C 코드 0, inherited fd) 뿐, 신규 능동 사용자 없음. `/proc/<pid>/io` byte-counter 샘플링은 권한 부족으로 실패 — 한계로 공개(결과에 영향 없음, source 분석이 결정적 근거)
  - **Task E**: `TORQUE_STATE_READBACK = UNAVAILABLE`(공식 read API 없음) → `TORQUE_STATE=UNKNOWN`. 과거 수동 관찰(저항감 등)을 torque=ON 확정 증거로 승격하지 않음
  - 🔴 **self-discovered 버그(AST guard 자체, advisor 없이 자체발견·수정)**: `self_ast_check()` 1차 초안이 `ast.Call(func=ast.Attribute)` 직접호출 패턴만 스캔해, 이 스크립트 자신이 쓰는 "메서드를 변수로 참조 후 호출"(`call_capturing(arm.Arm_serial_servo_read, sid)`) 패턴을 전혀 탐지하지 못함(1차 self-test 에서 허용 API 4종이 전부 `called_attrs`에 안 잡힘) → 모든 `ast.Attribute` 노드를 스캔하도록 수정 + PF01-10(합성 금지호출 2종에 대한 true-positive 탐지 증명) 추가
  - `Arm_serial_servo_write_offset_state()`/`Arm_Read_Action_Num()`은 source상 read 처럼 보이나 캘리브레이션/액션그룹 서브시스템에 묶인 trigger-write 를 동반해 **의도적으로 호출하지 않음**(사유 공개)
  - **negative test PF01-01~10 전부 PASS**(forbidden motion/torque write/reset/calibration write/action playback 전부 0회 + AST guard true-positive 증명). command budget: servo write 0, torque write 0, reset 0, calibration write 0, action playback 0, 신규 motion 0
  - `evidence/P-007F-01/`(`arm_lib_source_forensics.md`·`readback_semantics.md`·`readonly_probe.json`·`device_ownership.txt`·`diagnosis.md`) + `scripts/p007f_01_readonly_probe.py`. 기존 P-007E evidence 전부 불변
  - **다음 단계(원인 확정)는 이 gate 범위 밖** — MCU 펌웨어 소스가 없어 추가로 좁히려면 상태를 변경하는 최소 실험이 필요하나, 이 gate 는 그런 실험을 설계·제안하지 않고 여기서 STOP. 별도 승인된 P-007F-02 설계 대기
  - 🔴 **후속 발견(P-007F-02 작업 중, read-only)**: `evidence/P-007E-R2-R5/run_20261028T211746/r5_final_result.json`(Attempt #4, 실제 write 발생한 그 run)의 자동 기록은 `operator_observed_physical_motion="YES"`(실시간 질문-응답)이고 `result_code=X`는 `POST_WINDOW_QUALITY_GUARD_FAILED`(DaBai POST window 60프레임 중 28개만 valid, 80% 기준 미달) 때문이었다 — "무동작을 확인한 B/D 결과"가 아니라 "카메라 데이터 품질 미달로 실험 자체가 무효화된 X 결과"다. 작업의뢰서가 전제한 "operator corrected observation=NO MOTION"은 이 자동 기록에는 없고, 사후 운영자 재고로만 존재한다 — P-007F-02 `baseline.md`가 두 provenance를 덮어쓰지 않고 병렬 보존

- [x] **P-007F-02** Torque / Actuator State Discrimination Preregistration — **`TORQUE_DISCRIMINATION_REQUIRES_DESIGN_CHANGE`**
  - 🔴 **DESIGN/PREREGISTRATION ONLY** — 하드웨어 상태 변경 0건(servo/torque/PWM/reset/calibration/offset write·action playback·servo ID 변경·position command·수동 이동·R5 재실행·카메라/USB/I2C 재설정·process kill/restart 전부 0). 기존 `evidence/P-007E-R2-R5/**`·`evidence/P-007F-01/**`·`.cache/p007e_r2r5_ledger/**` 읽기만, 수정/삭제 0
  - **baseline**: P-007F-01 결과 그대로 사용(READBACK_SEMANTICS=UNRESOLVED/TORQUE_STATE=UNKNOWN/SERVO_COMMUNICATION=SUPPORTED/PHYSICAL JOINT POSITION=UNKNOWN) + Attempt #4의 두 provenance(machine record=YES, operator correction=NO MOTION)를 병렬 보존 → `PHYSICAL MOTION = UNRESOLVED`로 취급, `154°`는 controller-reported 로만 표현
  - **camera 선택**: DaBai DCW2는 eye-in-hand라 Arm2 motion에 대한 픽셀 감도는 높지만(R4 swept_motion.json에서 worst-case 링크로 이미 식별) **독립 witness 자격 없음**(Block 3 기존 정의와 동일 원칙) → SECONDARY/CORROBORATING ONLY로 강등. Gemini 335L은 3건의 독립 기존 evidence(sensor_inventory.json fov_fact·physical-arm1-reference.md 실측 프레임·decision.json not_recommended)로 "팔이 화각 밖"이 이미 확인되어 있어 `NOT CURRENTLY USABLE` — 이 gate에서 재배치 안 함
  - **witness 설계**: Candidate A(비카메라 — bounded external force + 독립 inclinometer/protractor 측정)를 primary로 채택, operator 정성 판단("단단하다/부드럽다")만으로 PASS 금지. Candidate B(visual)는 현재 독립 witness 불가로 기각, Candidate C(기타 비침습)는 Candidate A로 수렴(기존 P-007E-R2-R1 "MacBook 내장 카메라" 후보는 blocking unknown 미해결 상태 유지, 건드리지 않음)
  - 🔴 **torque OFF 위험분석(§7)**: URDF `<mass>` 전부 합산해 Arm2 하류(Arm3+Arm4+DaBai+Arm5+그리퍼 조립체) 총 질량 ≈0.211kg 산출, Arm2는 이를 지지하는 weight-bearing pitch joint(axis=(1,0,0))임을 확인. 현재 physical pose UNKNOWN이라 모멘트 암 크기를 사전에 알 수 없고, restoration(`torque(1)`) 성공을 확인할 공식 read API도 없음(P-007F-01 Task E) → **Q2(실제 torque ON/OFF 비교)는 이번 시점 REJECT**, torque를 건드리지 않는 Q1(bounded-force probe)만 다음 실행 gate의 1차 범위로 좁힘
  - **result matrix A/B/C/D/X**, **abort 조건 11개**(gravity drop·cable tension·camera mount instability·witness failure 등) 사전고정, 사후 변경 금지
  - **executable helper 파일 생성 0건**(지시에 따라 분석/설계 문서 중심) — AST 정적검사는 N/A로 명시, P-007F-03에서 실제 helper가 필요해지면 P-007F-01과 동일 패턴(allowlist+true-positive 증명) 적용을 사전등록
  - `evidence/P-007F-02/`(`baseline.md`·`witness_candidates.md`·`camera_witness_analysis.md`·`torque_experiment_design.md`·`result_matrix.md`·`safety_analysis.md`·`preregistration.md`). 기존 P-007E/P-007F-01 evidence 전부 불변
  - **최종 recommendation**: `TORQUE_DISCRIMINATION_REQUIRES_DESIGN_CHANGE`(READY도 UNSAFE도 아님 — witness를 비카메라 primary로 교체 + Q2를 spotting 절차 설계 전까지 유보하면 Q1 범위는 안전하게 설계 가능). **실제 torque/position command 미실행. P-007F-03 자동 시작 안 함.** 검수자가 P-007F-02만 검수 후 P-007F-03 개시 여부 결정

- [x] **P-007F-02A** Physical Witness Instrument Qualification — **`PHYSICAL_WITNESS_REQUIRES_OPERATOR_SETUP`**
  - 🔴 **DESIGN/QUALIFICATION ONLY** — 로봇 상태 변경 0, 외력 적용 0, 카메라 재배치 0
  - **inventory**: 전체 evidence 를 "스마트폰/수평계/protractor/digital level/caliper/tripod" 로 검색 — 신규 물리 계측 도구 보유에 대한 기존 evidence 전무. 추정하지 않고 스마트폰/protractor 재료=`OPERATOR_CONFIRMATION_REQUIRED`, 물리 수평계=`NOT VERIFIED`, 신규 고정 외부 카메라=Mac 카메라(R2-R1 blocking unknown 그대로 미해결)뿐 신규 후보 없음으로 정직하게 기록
  - 🔴 **스마트폰 mechanical loading**: `m_phone`·`r_phone` 둘 다 저장소에 없어 **수치 결과를 만들지 않음**(지시 그대로 — "일반적인 스마트폰 질량" 같은 공개 평균값도 쓰지 않음) → `tau_phone = m_phone*g*r_phone` 수식만 제시, "안전하다고 확인할 수 없음"으로 결론, primary 배제 근거로 사용(위험하다고 확인된 게 아니라 안전을 확인할 수 없다는 것이 근거임을 명확히 구분)
  - **protractor 설계**: robot base와 독립된 stationary reference + 경질 pointer(질량 최소화, 추가 하중≈0) + PRE/POST 동일 관찰위치 + parallax 관리 + 판독단위 사전고정 + blind 판독 권장. DaBai는 관찰 카메라로 쓸 수 없음(움직이는 구조라 "동일 관찰위치" 전제가 깨짐)을 설계로 명시
  - **measurement requirements**: resolution≤1°/repeatability≤1°/minimum detectable change≥2° — 서보 spec 역산이 아니라 "weight-bearing joint가 torque OFF면 수 도 단위로 처질 것"이라는 물리적 규모 추론(Arm2 하류 ~0.211kg + joint limit margin ~28°)에 근거함을 명시, 사후 변경 금지
  - **independence test**: A(스마트폰)/B(수평계)/C(protractor)/D(고정카메라) 전부 Arm_Lib readback·MCU state·servo target register·DaBai ego-motion 과 독립(`INDEPENDENT_PHYSICAL_WITNESS`). DaBai 자신은 측정대상 구조에 실려있어 `NOT_INDEPENDENT`(이 목적 한정, P-007F-02와 동일 결론 재확인)
  - **candidate ranking**: PRIMARY=C.protractor(독립+하중≈0+설계목표 분해능 충족), SECONDARY=A.스마트폰(mechanical loading 안전성 미확인으로 조건부, 단독 결론 근거로 쓰지 않음). B/D는 가용성 정보 부족/부재로 순위 제외
  - **§9 최대허용변위**: URDF joint limit(Arm2 upper margin≈27.996°, R4 joint_limits.json 재인용)은 하드 상한으로만 쓰고, 실제 bounded-force probe 상한은 `REQUIRES_OPERATOR_SETUP_MEASUREMENT`(서보 stiffness 추정치 없음 + 운영자의 실시간 중단판단 필요, 근거없는 숫자 임의선정 안 함)
  - `evidence/P-007F-02A/`(`instrument_inventory.md`·`candidate_comparison.md`·`smartphone_loading_analysis.md`·`protractor_design.md`·`measurement_requirements.md`·`independence_analysis.md`·`recommendation.md`). 기존 evidence 전부 불변
  - **최종 판정**: `PHYSICAL_WITNESS_REQUIRES_OPERATOR_SETUP`(방법론·순위는 확정, 재료 확보·§9 상한 설정은 운영자 확인 대기 — `QUALIFIED`로 격상 조건 2가지를 recommendation.md 에 명시). **하드웨어 명령 미실행. P-007F-03 자동 시작 안 함**

  - EPC-01~27 전부 충족 + EPC 외 추가 자체검증 1건(self_transfer_verification, advisor 요구). `evidence/P-007E-R2-R2/stage_c1r1r1_decision.json` + `stage_c1r1r1_architecture_note.json`(§24 원문 verbatim 추가) + `c1r1r1_repair_tests/repair_result.json`
  - 🔴 **advisor 2차 재검토 반영**: §24 원문("qualified only for ... rigid external scene / not qualified for dynamic trajectory tracking...")이 §11 문구로 잘못 들어가 있던 것을 architecture_note 에 `section_24_verbatim` 로 보완(코드 상수는 SHA 재시험 비용 때문에 그대로 둠, divergence 명시) · verdict 를 조건부로 수정(배포방식 결정 전까지 무조건 READY 아님) · 실제(non-mock) `build_runtime_provenance`/`check_dirty_tree` 1회 실행해 git_head=`7798e80`·dirty=False 확인 · 🔴 **이 gate의 evidence 회수 자체가 §14~16 을 어김**(`repair_result.json` scp 후 SHA diff 없이 Jetson rm -rf, 2회) — `TRANSFER_INTEGRITY_NOT_VERIFIED_BEFORE_SOURCE_DELETION` 로 공개(판정 영향 없음, 재현 가능한 pass/fail 로그라 재실행 불요)
  - **다음 physical-motion observation gate는 검수 승인 후 별도 설계. 이번 세션은 여기서 STOP**

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
> 🔴 **용어 고정(C0): 기존 50 ms 는 `HOST-CONTROL QUIET INTERVAL` 로 부른다.** `I2C BUS QUIET INTERVAL` · `bus was silent` 표현을 금지한다.
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

> 🔴 **C1 갱신 — DaBai `/dev/video0` 포맷이 바뀐 채 남아 있다.** Stage B 확인값은 `MJPG` 였으나, C1 의
> `cv2.VideoCapture()`(FOURCC 미지정)가 **`YUYV`** 로 바꿔놓았고 이번 단계에서 되돌리지 않았다
> (S_FMT 는 UVC control write 라 금지 범위). 이후 DaBai 를 다룰 때 baseline 을
> **`C1-induced YUYV`** 라고 명시한다 — Stage A/B/C0 가 가정한 `MJPG` 는 더 이상 현재 상태가 아니다.
> `--list-ctrls`·`G_FMT` 범위에서 다른 제어값(auto_exposure·auto WB·dynamic fps·50Hz·brightness 30·contrast 50)은
> Stage B 와 동일함을 확인했다 — **이 범위에서는** 포맷만 바뀌었다. 프레임 간격(S_PARM)은 Stage B 에 기준값이
> 없어 비교 불가 — 사후 1회 읽기 전용으로 확보한 `30.000 fps` 는 비교 근거가 아니라 **앞으로의 기준선**이다.
> 🔴 게다가 `cv2.VideoCapture` 는 여는 것만으로 **항상** S_FMT 를 호출한다(OpenCV `cap_v4l.cpp` 1차 소스 확인) —
> C0 의 `no_control_writes` 와 `capture_mechanism: cv2.VideoCapture` 자체가 내적으로 모순이다(M6). 복구(S_FMT 로
> MJPG 재지정)나 캡처 경로 변경은 검수 결정 사항이며 사전등록 개정(C0-R1 범위)이 필요하다.
> 근거: `evidence/P-007E-R2-R2/stage_c1_run.json`

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
| 17 | 2026-10-27 23:00~23:43 (KST, 커밋 `6144076`~`121dc43`) | **P-007E-R2-R2 Stage C0** 사전등록 (세션 `66eb4937`) | `PREREGISTRATION / C1 HARNESS READY` — 카메라 기동 0 · C1 미실행
| 18 | 23:47~23:53 KST (Jetson +0800 22:47:38~22:53 → +1h 환산) · 보고 커밋 2026-10-28 (세션 `66eb4937`) | **P-007E-R2-R2 Stage C1** 최초 실캡처 — 10 trial 완주했으나 저장 직전 크래시 | `X — OBSERVATION INVALID` — 이 단계가 보낸 로봇 명령 0, 데이터 전량 소실
| 19 | 2026-10-28 (세션 `66eb4937`, 커밋 `6f9b9c5`) | **P-007E-R2-R2 Stage C0-R1** acquisition harness repair & re-preregistration — 카메라 기동 0, 2라운드 자체검증 반영 | `ACQUISITION REPAIR PASS / C1-R1 RE-PREREGISTERED`
| 20 | 2026-10-28 (세션 `66eb4937`) | **P-007E-R2-R2 Stage C0-R2** DaBai pixel-format baseline recovery — 승인된 S_FMT 1회, capture 0 | `PIXEL FORMAT RECOVERED TO MJPG 640x480 / CONTROLS UNCHANGED`
| 21 | 2026-10-28 (세션 `66eb4937`) | **P-007E-R2-R2 Stage C1-R1** stationary sham 첫 성공 실캡처, N=10/M=60 | `Q1 — OBSERVATION CHANNEL QUALIFIED` — 10/10 VALID, false positive 0, dropout 0, robot motion 0
| 22 | 2026-10-28 (세션 `66eb4937`) | **P-007E-R2-R2 Stage C1-R1-R1** evidence pipeline 정적 repair(runtime provenance/자동summary/artifact SHA/전송무결성), 카메라·로봇 0 | `EVIDENCE PIPELINE CLOSURE PASS / FUTURE EXPERIMENT PROVENANCE READY`
| 23 | 2026-10-28 (세션 `66eb4937`) | **P-007E-R2-R3** Controlled Physical-Motion Observation Preregistration — Block 3 relabel, A/B/C/D/X 결과행렬 동결, Arm2 ID2 152→154(+2°) 명령 사전등록, 카메라·로봇 0 | `CONTROLLED PHYSICAL-MOTION EXPERIMENT PREREGISTERED / NO CAMERA OR ROBOT EXECUTION`
| 24 | 2026-10-28 (세션 `66eb4937`) | **P-007E-R2-R4** Motion Execution Safety Preflight — tree FK 신규 구현+선례 4건 회귀 PASS 후 152→154 계산, swept displacement/joint limit/self-collision(SRDF)/raw encoding/envelope, R3 CLOSED 재사용(수정 0), write-ahead ledger, 카메라·로봇 0 | `MOTION EXECUTION SAFETY PREFLIGHT PASS / ARM2 152→154 CANDIDATE READY FOR SEPARATE EXECUTION APPROVAL / NO CAMERA OR ROBOT EXECUTION`
| 25 | 2026-10-28 (세션 `66eb4937`) | **P-007E-R2-R5** Single Controlled Physical-Motion Execution — 최초로 실제 capture/controller read/servo write(≤1회) 허용 harness 구축(window/execution/hardware_wiring/run 4파일, R3/R4 CLOSED 재사용 수정 0), §31 승인게이트를 R3 write-call 지점에 배치, durable ledger+재시작보호, self-test 24종 전부 PASS, advisor 생략 지시로 자체검증만(self-referential SHA 버그 1건 자체발견·수정) | `코드/준비상태 구축 완료 — 실제 --execute 는 로봇을 지켜보는 운영자 입력 대기, Claude 는 --check-only 까지만`
| 26 | 2026-10-28 (세션 `66eb4937`) | **P-007E-R2-R5 Provenance 수정** — Attempt #3 `ABORT_BEFORE_MOTION`(R5 가 만든 run_* evidence가 다음 실행의 dirty-tree 검사를 스스로 막는 self-contamination) 수정. R5 wiring 계층에만 porcelain-filter 추가(R3/R4/evidence_pipeline 수정 0, 기존 evidence 2개 보존), regression HW-05~13 전부 PASS, Jetson 실기 재확인 | `d02611e — 수정 완료, run_* 2개 존재해도 overall_check_only_pass=true`
| 27 | 2026-10-28 (세션 `66eb4937`) | **P-007F-01** Controller Readback vs Physical Servo Motion 진단 — R5 Attempt #4(운영자 직접실행) 실제 write 1회·readback 152→154·physical motion=NO 에 대해 신규 motion 0건으로 Arm_Lib 1.0.3 source forensic(MCU 아키텍처 확인)+read-only probe(read vs read_any 교차검증 180-154=26 일치, ping 전부 0xda)+device ownership 재조사. AST guard 자체발견버그 1건 수정(참조-후-호출 패턴 탐지 누락) | `READBACK_SEMANTICS_UNRESOLVED / TORQUE_STATE=UNKNOWN / SERVO_COMMUNICATION=SUPPORTED`
| 28 | 2026-10-28 (세션 `66eb4937`) | **P-007F-02** Torque/Actuator State Discrimination Preregistration — DESIGN ONLY(하드웨어 변경 0). Candidate A(비카메라 외력+독립 inclinometer)를 primary witness 채택, DaBai=secondary/Gemini=`NOT CURRENTLY USABLE`(기존 evidence 재확인), URDF mass 합산으로 Arm2 하류 ~0.211kg weight-bearing 위험분석 후 Q2(torque OFF) REJECT·Q1(torque 불변 probe)만 승인범위로 좁힘. Attempt #4 r5_final_result.json 재조사로 "physical motion=NO"가 기계기록(YES)이 아니라 사후 재고임을 추가 발견 | `TORQUE_DISCRIMINATION_REQUIRES_DESIGN_CHANGE`
| 29 | 2026-10-28 (세션 `66eb4937`) | **P-007F-02A** Physical Witness Instrument Qualification — DESIGN ONLY(로봇 상태 변경 0). 저장소 전체 검색으로 스마트폰/수평계/protractor 보유 evidence 없음을 확인(추정 안 함), 스마트폰 mechanical loading은 m_phone/r_phone 미지라 수치 미생성·"안전확인불가"로 primary 배제, protractor(독립+하중≈0)를 PRIMARY·스마트폰을 조건부 SECONDARY로 선정, §9 최대변위는 운영자 실시간판단 필요로 REQUIRES_OPERATOR_SETUP_MEASUREMENT | `PHYSICAL_WITNESS_REQUIRES_OPERATOR_SETUP`

> 🔴 **시계 주의**: Jetson 은 **+0800**, 이 표와 git 은 **KST(+0900)** 다. 증거 파일의 `commanded_at 17:49:06+0800` = KST 18:49:06.

## 운영 규칙 (이 프로젝트 고유)

- **게이트 판정 문자열은 재서술하지 않는다.** `acceptance.json` 의 값을 그대로 옮긴다
- **운영자 관찰은 증거로 승격하지 않는다.** 관찰로만 기록한다 (P-007C 가 세운 기준)
- **선행 게이트 증거는 불변이다.** 새 게이트는 자기 디렉터리에만 쓴다. `git diff --exit-code HEAD -- evidence/<선행>` 으로 착수·종료 시 확인
- **플랜 검토는 advisor 가 한다** (codex 자문 대체 — 사용자 지시 2026-10-27)
- **가설 라벨은 분기까지 적는다** — `H-C6.learning_mode`(ELIMINATED)와 `H-C6.other_controller_state`(OPEN)는 다르다. 상위 라벨만 적어 범위를 넓게 읽히게 하지 않는다 (검수 세션 지적 2026-10-27)
- 외부 수집 자료는 **gitignore 된 게이트 디렉터리 + 커밋되는 매니페스트**로 둔다. 스크래치패드 금지 (`docs/261027-1358-*-이슈보고서.md`)
