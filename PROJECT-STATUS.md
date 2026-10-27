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
| 최신 닫힌 게이트 | **P-007E-R1 Phase B `A/C NOT DISCRIMINATED`** | `evidence/P-007E-R1/phase_b_acceptance.json` |
| **게이트가 일으킨** 로봇 모션 | **P-007E-R1 = `UNKNOWN`** · 그 외 0 | 🔴 P-007E-R1 Phase B 는 모션 명령을 1회 집행했고 **물리 모션 여부가 관찰되지 않았다**(운영자 부재). P-007A·A-R1·B·C·C-R1 `environment.json` 은 전부 `robot_moved_by_this_gate: 0`. ⚠️ 프로젝트 전체가 0 이라는 뜻이 **아니다** — 격리 전에는 벤더 autostart 가 부팅마다 `move_init_pose()` 로 팔을 움직였다 |
| 서보 위치 명령 누적 | **2회** (P-007B · P-007E-R1 Phase B — 둘 다 ID2 152°, raw 1242) | P-007B `servo_position_writes: 1` · P-007E-R1 `phase_b_acceptance.json servo_writes: 1`. 나머지 게이트는 전부 `0` |
| `T_base_camera` | **없음** | P-006 BLOCKED. 실측 기준 부재 |
| 비전 파이프라인 | 기하는 동작, 게이트는 FAIL 유지 | P-004 / P-005 |
| 제어 권한 독점 | 확보됨 (가역 격리) | `evidence/P-007A-R1/` |
| 미해결 가설 | **`0x12`/4B 서보 명령은 이제 controller readback 을 바꾼다.** 남은 미해결은 **물리 모션 여부**(L5 이후)다. block-write 전면 고장은 `ELIMINATED`. 아래 §evidence-strength 표 참조 | `evidence/P-007E-R1/phase_b_acceptance.json` |

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

HYPOTHESIS:
  transaction-spacing / board-processing timing
  may also affect servo write acceptance
```

🔴 다음 표현은 **사용하지 않는다** — `timing root cause confirmed` · `board requires 10 ms` · `servo write requires 10 ms` · `P-007B failed because of timing` · `TIMING HYPOTHESIS STRONGLY SUPPORTED` · `TIMING HYPOTHESIS NOT SUPPORTED BY CONTROLLED RETEST` (Phase B 는 A·D 어느 쪽도 성립하지 않았다)

**아직 어느 것도 증명되지 않았다.** 측정된 불안정은 **읽기** 경로이고, 너무 이른 **쓰기**가 같은 식으로 버려지는지는 직접 측정되지 않았다. P-007B 의 실제 read→write 간격도 기록돼 있지 않다(소스상 sleep 부재만 확인).

### Phase B readback 의 산술 (P-007E-R1)

```
cmd 152       → target raw 1242 → 정확 안착 시 readback 153
observed 152  ⇒ raw 1243..1254  (target 보다 1~12 count = 0.08°~0.98° 모자람)
pre     150   ⇒ raw 1267..1278
raw 이동폭      13..35 counts = 1.06°~2.86°
```

readback 152 는 보낸 target raw 의 **정확한 echo 가 아니다**(echo 면 153). target 근처 안착 위치와 모양이 같아 **물리 모션(A)을 배제할 수 없다**. 컨트롤러 내부 보간 보고와 이 데이터만으로는 갈리지 않는다. 근거 식은 `evidence/P-007B/command_api_analysis.md`.

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
- [x] **P-007E** Controlled Servo Response Discrimination (1차) — **`PHASE-1 ABORTED BY GUARD`**
  - 가드가 명령을 막았고 그 과정에서 **I2C 최소 간격**을 발견했다 (`evidence/P-007E/i2c_timing_discovery.json`)
  - offline FK/clearance 보존 — `+5°` 는 TCP 13.67 mm, `+2°` 는 5.47 mm (`fk_clearance.json`)
  - 서보 쓰기 0
- [x] **P-007E-R1** Timing-Controlled Servo Command Experiment
  - Phase A — **`PHASE-A PASS — HARNESS READY / NO MOTION COMMAND EXECUTED`**. 서보 쓰기 0
  - Phase B — **`A/C NOT DISCRIMINATED`** (`phase_b_acceptance.json`)
    - 명령 `Arm_serial_servo_write(2, 152, 2000)` **1회** · quiet interval **60.099 ms** · 예외 0 · `I2C error` 0
    - 🔴 **readback 150 → 152** (5라운드 전부, None 0) — P-007B 는 같은 명령에서 150 불변이었다
    - 🔴 **물리 모션 `UNOBSERVED`** — 운영자가 명령 시각에 자리에 없었다. 사후 사진 비교는 "5mm 판별 불가"로 철회
    - 🔴 `TIMING HYPOTHESIS STRONGLY SUPPORTED` · `NOT SUPPORTED` · `ROOT CAUSE VERIFIED` 어느 것도 쓰지 않는다

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
                                                    └ Servo  0x12/4B ACCEPTED_BY_CONTROLLER
                                                        (readback 150→152, quiet interval 60.099 ms)
                                                        → L5 UNKNOWN → L6 MOTION UNOBSERVED
```

🔴 `L6 OBSERVED_NO_MOTION` 은 P-007B 한정이다. Phase B 의 L6 는 **관찰 자체가 없다**(`UNOBSERVED`) — 모션 없음이 관찰된 것이 아니다.

## 🔴 다음 — 검수 세션 판단 대기 (STOP)

**P-007E-R1 Phase B 가 집행됐고 `A/C NOT DISCRIMINATED` 로 닫혔다.** 승인 지시대로 판정과 raw evidence 까지만 남기고 **STOP** 한다 — 다음 게이트를 열지 않고 원인에 맞춘 코드 수정도 하지 않는다.

검수 세션이 결정할 사항:

1. **A/C 판별을 어떻게 복구할지** — 물리 모션 관찰이 없다. 추가 모션 명령 없이 가능한 관찰 경로가 있는가
2. **baseline stale 처리** — 현재 readback `[86, 152, 0, 18, 93, 178]`. 150 복귀는 또 하나의 모션 명령이다
3. **readback 의미** — raw 1243..1254 는 target raw 1242 의 echo 가 아니다. 물리 위치 vs 컨트롤러 보고의 판별 설계

들고 들어가는 사실: torque `TORQUE_ON_STRONGLY_SUPPORTED` · learning mode 제외 · global block-write failure 제외 · 공식 API encoding 이상 없음 · RGB/STM8 path 동작 · **`0x12`/4B 서보 명령이 controller readback 을 target 근처 값으로 바꿨다(quiet interval 60.099 ms)** · **물리 모션은 어느 실행에서도 양성 관찰이 없다**.

## ⚠️ 이월 위험 (모든 세션에서 유효)

> 🔴 **E-R1 갱신 — baseline 이 바뀌었다.** 현재 controller readback 은 **`[86, 152, 0, 18, 93, 178]`** 이다(P-007E-R1 Phase B). **`150` 을 전제한 스크립트는 전부 stale** 이다 — `scripts/p007e_r1_timing_controlled.py` 의 `EXPECTED_BASELINE`/`EXPECTED_ID2`, `scripts/p007b_preflight.py`, `scripts/p007e_preflight.py` 포함. 그 전제를 쓰는 실행은 `PRE_COMMAND_BASELINE_ABORT` 로 끝난다. 🔴 **동일 조건 재실행은 추가 모션 명령(152→150) 없이는 불가능하고, 그것은 승인 예산 밖이다**
>
> 🔴 **물리 위치는 미확인이다.** readback 152 가 실제 관절 위치인지 컨트롤러 보고값인지 갈리지 않았다. 다음 게이트가 FK·clearance 를 계산할 때 `150` 을 실제 자세로 가정하면 안 된다


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
| 12 | 2026-10-27 15:59~18:39 | P-007D-R2 · P-007E · P-007E-R1 Phase A | R2-C CLOSED · E 가드 abort · R1 harness READY (서보 쓰기 0) |
| 13 | 2026-10-27 17:45~ | **P-007E-R1 Phase B** (세션 `01C11Mac`) | **A/C NOT DISCRIMINATED.** 서보 명령 2번째 1회 집행, readback 150→152, 물리 모션 UNOBSERVED |

## 운영 규칙 (이 프로젝트 고유)

- **게이트 판정 문자열은 재서술하지 않는다.** `acceptance.json` 의 값을 그대로 옮긴다
- **운영자 관찰은 증거로 승격하지 않는다.** 관찰로만 기록한다 (P-007C 가 세운 기준)
- **선행 게이트 증거는 불변이다.** 새 게이트는 자기 디렉터리에만 쓴다. `git diff --exit-code HEAD -- evidence/<선행>` 으로 착수·종료 시 확인
- **플랜 검토는 advisor 가 한다** (codex 자문 대체 — 사용자 지시 2026-10-27)
- **가설 라벨은 분기까지 적는다** — `H-C6.learning_mode`(ELIMINATED)와 `H-C6.other_controller_state`(OPEN)는 다르다. 상위 라벨만 적어 범위를 넓게 읽히게 하지 않는다 (검수 세션 지적 2026-10-27)
- 외부 수집 자료는 **gitignore 된 게이트 디렉터리 + 커밋되는 매니페스트**로 둔다. 스크래치패드 금지 (`docs/261027-1358-*-이슈보고서.md`)
