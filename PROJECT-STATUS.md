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
| **P-008-R3** | **`PASS_6D_WAYPOINT_GENERATION`**. R2 147점 XYZ 유지. k=20 PCA, 카메라 방향, tool Z=`-N`. 퇴화 0. 자세 변화 중앙값 7.2°, 최대 160.8°는 index 125. 기하 불연속 1. base 변환 없음. robot/servo 0 | `evidence/P-008/R3/result.json` |
| **P-008-R2** | **`PASS_PATH_EXTRACTION`**. capture_01 신발 후보에서 장축 측면 3D path 147점, 간격 중앙값 2.0 mm. ordering 위반 0. 표면 거리 중앙값 1.7 mm. crest/boundary 없음. capture_02–05 미사용. 6D waypoint 미개방. robot/servo 0 | `evidence/P-008/R2/result.json` |
| **P-008-R1GEO-V** | **`PASS_HOLDOUT_VALIDATION`**. capture_02–05에 동결 파라미터 그대로 적용. 4/4. segmentation track `CLOSED`. 경로 추출은 시작하지 않음. robot/servo 0 | `evidence/P-008/R1GEO-V/summary.json` |
| **P-008-R1GEO** | **`PASS_GEOMETRY_SEGMENTATION`**. capture_01 XYZ만. 매트 평면 분리 후 신발 후보 5182점. R1H는 동결. capture_02–05 미사용. robot/servo 0 | `evidence/P-008/R1GEO/result.json` |
| **P-008-R1H** | **`PASS_METHOD_SELECTION`**. capture_01만. 내부 색상 LOW, 높이 HIGH. 접촉 매트 쪽 0점이라 `NOT_IDENTIFIABLE`. `SEGMENTATION_STOP_RULE`. segmentation 없음. R1I 미개방. robot/servo 0 | `evidence/P-008/R1H/verdict.json` |
| **P-008B** | **`PASS_DATASET_V2`**. Dataset v2. SW D2C, XYZRGB, optical extrinsic 실측. depth↔cloud 5/5. 등록 정확도는 `NOT_CLAIMED`. hardware sync `NOT_VERIFIED`. segmentation 없음. R1H 미개방. robot/servo 0 | `evidence/P-008B/verdict.json` |
| **P-008-R1G** | **`FAIL_DEPTH_CLOUD_CORRESPONDENCE`**. capture_01 depth↔cloud 중앙값 1.08 mm이나 고정한 p95·양수 depth 비율 미달. color↔depth 행렬은 저장본에 없음. RGB 투영 없음. robot/servo 0 | `evidence/P-008/R1G/verdict.json` |
| **P-008A** | **`PASS`**. Baseline B serial `CP263630009D`, firmware `1.4.60`. 정지 운동화 RGB+depth+point cloud 5/5. 기하 알고리즘 없음. P-008 미재개. robot/servo 0 | `evidence/P-008A/verdict.json` |
| **P-008-R1F** | **`PASS_DIAGNOSIS`**. 10 mm 연결은 재현. 환경은 3.6 mm에서 붙고 원판 전체는 26.7 mm에야 한 component. 거리 분리 `NOT_FEASIBLE`. tolerance 불변. robot/servo 0 | `evidence/P-008/R1F/verdict.json` |
| **P-007 최종** | **`CLOSED / BLOCKED`**. command path `VERIFIED`. physical Arm2 motion `UNRESOLVED`, angle `UNKNOWN`. static tracking `VERIFIED`. observability `UNRESOLVED`. 추가 진단 `STOPPED`. P-006D는 열지 않음 | `evidence/P-007/P-007_FINAL_CLOSURE.md` |
| **블로킹 지점** | **ID2 가 명령대로 물리 이동했는지 확인되지 않았다.** 🔴 "위치 명령에 응답하지 않는다"는 더 이상 맞지 않다 — P-007E-R1 Phase B 에서 **controller readback 이 150 → 152 로 응답했다**. 물리 모션은 `UNOBSERVED` | `evidence/P-007E-R1/phase_b_acceptance.json` |
| 최신 닫힌 게이트 | **`P-007E-R1 PHASE-B — … / CLOSED @ a28e41e`** (검수 종결) | `evidence/P-007E-R1/phase_b_acceptance.json` |
| 진행 중 게이트 | **P-007E-R2-R2** · Stage A/B/C0/C1/C0-R1/C0-R2/C1-R1/C1-R1-R1 CLOSED · **P-007E-R2-R3** `CONTROLLED PHYSICAL-MOTION EXPERIMENT PREREGISTERED` CLOSED · **P-007E-R2-R4 `MOTION EXECUTION SAFETY PREFLIGHT PASS / ARM2 152→154 CANDIDATE READY FOR SEPARATE EXECUTION APPROVAL / NO CAMERA OR ROBOT EXECUTION`** — Jetson 에서 URDF/SRDF/13개 STL mesh 를 read-only 로 가져와(SHA256 전송무결성 확인, 원본 미삭제) tree-FK 모듈을 새로 구현, 선례(p007b self-check/fk_clearance.json 150→155/P-007B +2° TCP 5.47mm/P-007A Arm3 margin) 4건을 전부 재현(오차 ≤0.005mm)해 검증한 뒤 152→154 실제 계산 수행. TCP 변위 5.47mm, DaBai 변위 5.44mm/회전 2.0°, 로봇 전체 최대 변위(exact, 정점기반) 5.93mm(DaBai_DCW2_Link)/sphere-bound cross-check 8.46mm, joint limit 전부 PASS(Arm3 margin≈0 공개), self-collision은 벤더 SRDF(전체 link pair Adjacent/Never/Default 로 disable)를 1차 근거로 FK spot-check(Arm1-Arm2 ~0.13mm 접촉/base-Arm2 ~16mm 간극, 2° 구간 안정)로 보완. raw servo 인코딩 재확인(152→raw1242 선례 재현, 154→raw1217, payload `[0x04,0xC1,0x07,0xD0]`), raw/readback 버킷 불확실성에서 Δ_max=3.03° 도출해 operator envelope=15mm 확정. R3(CLOSED, SHA 고정)는 1바이트도 수정하지 않고 import 재사용, R4 wrapper가 workspace-safety gate·write-ahead 재전송방지 ledger를 추가. negative test 25종 전부 PASS. 🔴 **실제 `152→154` 명령 실행은 이 gate로 승인되지 않음 — 검수자가 별도 실행 승인을 내려야 함** · **P-007E-R2-R5 코드/준비상태 구축 완료 — 실제 `--execute`(실제 capture/controller read/servo write ≤1회 허용)는 로봇을 직접 지켜보는 운영자가 Jetson 에서 §31 승인 문구를 입력해야만 진행됨, Claude 는 `--check-only`(opt-in 0)까지만 실행** · **P-007E-R2-R5 Attempt #3 결과: `ABORT_BEFORE_MOTION`(provenance self-contamination, 수정 완료 d02611e) · Attempt #4(운영자 직접 실행): 실제 write 1회 발생, readback 152→154, operator 관찰 physical motion=NO — 원인 미확정** · **P-007F-01 `READBACK_SEMANTICS_UNRESOLVED` / `TORQUE_STATE=UNKNOWN` / `SERVO_COMMUNICATION=SUPPORTED`** — read-only Arm_Lib 1.0.3 source forensic + runtime probe(새 motion 0건) · **P-007F-02 `TORQUE_DISCRIMINATION_REQUIRES_DESIGN_CHANGE`** — torque/actuator-state 판별 실험 사전등록(DESIGN ONLY, 하드웨어 변경 0), Candidate A(비카메라 외력+독립 inclinometer)를 primary witness로 채택, DaBai=secondary/corroborating만, Gemini=`NOT CURRENTLY USABLE`(화각 밖, 재배치 안 함), Q2(실제 torque OFF)는 spotting 절차 미설계로 보류 · **P-007F-02A `PHYSICAL_WITNESS_REQUIRES_OPERATOR_SETUP`** — 물리 계측 도구 qualification(저장소에 스마트폰/수평계/protractor 재료 보유 evidence 없음을 확인, 추정 안 함), PRIMARY=protractor(독립·하중≈0), SECONDARY=스마트폰(mechanical loading 안전성 수치 확인 불가로 조건부), 운영자 재료 확보 대기 · **P-007F-02B-R1 `BASELINE_B_PLACEMENT_NEEDS_MINOR_ADJUSTMENT`** — 사용자가 재배치한 Gemini 335L 실제 RGB/Depth 캡처(12프레임, p620 기존 컨테이너/스크립트 재사용, 로봇 0 동작) 검수. robot/workspace coverage·depth 품질·independence·baseline stability 양호, Arm2 witness만 MARGINAL(시야각이 가파른 하향이라 pitch 평면 foreshortening+링크 self-overlap) → 하향 기울기 완화 방향 조정 권고, 수치는 임의 제시 안 함(**운영자가 이후 위치 자체를 재변경해 최종 채택되지 않음 — Baseline B Candidate #1, historical/not-selected 로 보존**) · **P-007F-02B-R2 `BASELINE_B_PLACEMENT_ACCEPTED`** — 재배치된 새 위치(Baseline B Candidate #2)를 R1과 독립적으로 재평가(동일 파이프라인 재사용, 로봇 0 동작). ARM2_WITNESS_GOOD(R1 MARGINAL→개선, Arm1/Arm2 분리+DaBai 식별 가능)·BALANCED·depth usable 3조건 동시 충족으로 **Gemini Global Workcell Baseline B 로 freeze 후보화**, 추가 최적화 위한 재이동 권고 안 함 · **P-007F-03 `P-007F-03 = PASS / CLOSED`** — Gemini Baseline B를 향후 1회 Arm2 명령의 독립 external motion witness로 쓰기 위한 protocol 사전등록(DESIGN ONLY, robot command 0). Baseline B 미변경 확인(global phase-correlation shift ~0.12px), 70프레임 static dataset, ORB+RANSAC을 PRIMARY visual algorithm으로 실측 선정(4종 비교), `MARKER_REQUIRED=false`, sham 10trial로 threshold(T_translation=1.78px/T_rotation=1.08°) 산출해 false-positive 0/10 달성, result matrix·claim boundary(controller readback≠physical angle 재확인)·restart/retry 보호 설계까지 전부 고정. 실제 execution은 미실행, 별도 승인 대기 · **P-007F-04 `P-007F-04 = PASS / CLOSED`** — Single-Motion Execution Safety Review & Preregistration(robot command 0). 현재 상태(154) 재확인·Baseline B freeze 재확인 후, ±1/±2/±3° 양방향 6개 candidate를 P-007E-R2-R4 FK 모듈 SHA-pin 재사용으로 평가(joint-limit/FK/displacement envelope/self-collision 전부 PASS), Gemini visual detectability 근사추정(9개 가정 조합)에서 |delta|=3°만 안정적으로 threshold 초과 → lower -3°(154→151, joint-limit margin 관리 근거) 선택, 향후 execution 사전조건(PRE abort·exact approval phrase·신규 ledger 설계)까지 전부 사전등록. AST hardware-free 7/7 PASS, negative test 6/6 PASS · **P-007F-05 `HARDWARE-FREE PREPARATION DONE / READY_FOR_EXPLICIT_APPROVAL = YES`** — 실제 Arm2 motion execution harness(2-host: Gemini=p620/ROS2, Arm2=Jetson/Arm_Lib) 신규 구현, AST hardware-free 검증(execution.py 하드웨어 import 0 + jetson_controller.py 단일 write callsite 인자/위치 증명) + negative test NT-01~15 전부 15/15 PASS(hardware_write_count=0) + 실측 read-only/camera-only dry-run(Gemini identity·Baseline B·controller PRE read 154 전부 확인, write 0건) 완료 후, **사용자가 정확한 승인 문구(`EXECUTE P-007F-05 ARM2 154 TO 151 ONCE`)를 직접 입력해 §36 실제 실행 개시** — Arm2 servo write 정확히 1회 집행(target=151), controller readback 154→152(commanded target과도 다름, raw/readback 불확실성), Gemini Channel B는 robot-relative translation 0.23px/rotation 0.40°로 threshold(1.78px/1.08°) 미달 → `motion_yes_no=NO`, 운영자 SECONDARY 관찰=YES(육안 목격) — 세 신호가 서로 다른 방향. result matrix 사전등록 규칙 기계적용 결과 `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION`. **`P-007F-05 = CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION — 최종 판정은 검수자(ChatGPT/사용자) 대기, write 1회 집행 후 재전송/복귀 없이 STOP`** · **P-007F-06 `F2 = POST_HOC_LOCAL_CHANGE_DETECTED_BUT_MOTION_NOT_ESTABLISHED`** — P-007F-05 저장된 PRE/POST RGB+Depth 재분석(완전 offline, robot command 0, 새 촬영 0). Primary ORB 재현(오차 0.05 이내) 후 5종 추가 방법(optical flow/registered abs-diff/edge/phase-correlation + within-window noise control: PRE-PRE·POST-POST 20쌍 분포와 PRE→POST 신호 비교)으로 재분석 — ARM2_ROI가 6개 metric 중 3개(flow/abs-diff/phase-correlation)에서 noise mean+3σ·max를 모두 초과, STATIC_REFERENCE_ROI는 0/6. 🔴 **자체발견·수정**: 처음엔 4개 ROI만 검사했다가 단순 registered-diff 수치에서 DABAI/ARM4/ARM3도 static보다 높게 나오는 것을 보고 9개 ROI 전체로 noise-control 확장 → ARM1_ROI·STATIC_REFERENCE_ROI를 제외한 거의 모든 ROI가 noise 초과, 즉 변화가 Arm2에 국한되지 않고 로봇 영역 전반에 diffuse함을 확인(최초 4-ROI 검사만 했다면 잘못된 F1 결론이 나올 뻔함, 더 넓게 검사해 정정). Arm2 trackability 재검토(keypoint 14개/P-007F-03과 일치하나 y축 분산 2.2px로 거의 한 줄에 몰려 rotation 추정 구조적 불안정) → `MARKER_RECOMMENDED_FOR_FUTURE`. RGB/Depth 간 23.8mm 실제 baseline 확인 → `DEPTH_SPATIAL_LOCALIZATION_LIMITED` 공개, point cloud는 억지로 수행 안 함. 운영자 YES 관찰은 `UNKNOWN`으로 유지(증명도 반증도 안 함). **P-007F-05 사전등록 결과는 소급 변경 없음**, figure 10개 생성, AST hardware-free PASS(robot_command_count=0) · **P-007F-07 Phase A `PASS / CLOSED — READY_FOR_MARKER_SETUP=YES, READY_AND_ARMED=NO, MOTION_AUTHORIZED=NO`** — marker 기반 continuous external motion witness 준비(완전 offline, robot/servo command 0). `cv2.aruco DICT_4X4_50` 선정, 실측(ARM2_ROI depth 677.6mm+color fx 610.5px) 기반 marker 크기 50mm(fallback 40mm) 계산, 부착 위치 PROPOSED(사진 기반, 실물 확인 필요). Secondary marker=RECOMMENDED. Trajectory classifier(M1~M6/MX taxonomy) 10/10 synthetic test PASS(Move&Hold/Twitch&Return/Oscillation/No-Motion/Downstream-Local + **§18 critical PRE=POST transient regression PASS**). 🔴 **자체발견·수정**: oscillation이 magnitude(hypot)만으로는 부호반전을 못 잡아 Case C 최초 실패 → 원형 sign-reversal 카운터 추가 + 분류 우선순위 재배치로 수정. One-write architecture(AST로 call-site=1·루프/except 밖 증명), 신규 ledger `.cache/p007f07_ledger/`(6단계 상태머신, COMMAND_SENT 이후 전 상태에서 재시도 차단 — P-007F-05에서 발견된 POST_COMPLETE 미차단 버그 처음부터 방지), 승인→명령 fast path(injected callable 5개만, 무거운 준비 없음) 설계 완료. Two-host clock 설계(orchestrator를 p620에서 실행해 continuous-capture와 SSH-dispatch가 같은 clock domain 공유). P-007F-03~06 evidence 전부 불변(git diff 확인) | `evidence/P-007F-04/README.md`, `evidence/P-007F-05/preparation/README.md`, `evidence/P-007F-05/run_1793274484/README.md`, `evidence/P-007F-06/README.md`, `evidence/P-007F-07/phase_a/README.md` |
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

- [x] **P-007F-02B-R1** Gemini 335L Global Workcell Placement Qualification — **`BASELINE_B_PLACEMENT_NEEDS_MINOR_ADJUSTMENT`**
  - 🔴 **사용자가 Gemini 335L 을 물리적으로 재배치**(DOFBOT 맞은편·더 높은 위치·중앙 offset·하향 사선 시점 후보) → 그 현재 위치(`Baseline B Candidate`)를 실제 RGB/Depth 캡처로 검수. DOFBOT position/torque/reset/calibration/action playback command 0, DaBai 재배치 0, R5 재실행 0 — **ZERO ROBOT MOTION**
  - **캡처**: p620 기존 `bytesize-robotics-camera` docker 컨테이너 + `scripts/run_humble_camera.sh`(USB 노드 변경 자동감지·재생성, 기존 reversible 패턴 그대로 재사용) + `ros2 launch orbbec_camera gemini_330_series.launch.py` + 기존 `scripts/p006c_capture_streams.py`(출력 경로만 변경, 알고리즘 수정 0)로 color+depth+camera_info+TF synced 12프레임 세트 캡처. 캡처 종료 후 드라이버 프로세스 정지(컨테이너는 기존 관례대로 idle 유지)
  - **camera identity**: serial `CP263630009D`, firmware 1.4.60, color 1280x720 MJPG@30fps(rgb8), depth 848x480 Y16@30fps(16UC1, mm 추정), T_depth_color baseline≈23.8mm. lsusb `2bc5:0804` 로 기존 evidence(P-006/P-007E-R2-R1)와 vid:pid 일치 재확인, 다른 Orbbec(DaBai, Jetson 소속) 과 혼동 없음 확인
  - **robot coverage**: base→end-effector 프레임 내 확보(잘림 없음), 다만 Arm1~Arm4 는 시야각상 self-overlap 으로 개별 구분 어려움(PARTIAL). DaBai assembly 는 다른 구조물과 겹쳐 식별 불가(UNKNOWN)
  - 🔴 **Arm2 witness suitability**: `ARM2_WITNESS_MARGINAL` — 배경 대비·texture 는 양호하나(무언가 움직였다는 대략적 감지는 가능할 것으로 추론), Arm2(pitch joint, axis=(1,0,0))의 회전평면이 현재의 가파른 하향 시야각에서 카메라 시선축에 가깝게 정렬돼 foreshortening 우려 + 상부 링크 클러스터의 강한 self-overlap. Arm2 를 실제로 움직여 확인하지 않았으므로 이는 기하학적 추론이지 실측이 아님을 명시
  - **workspace coverage**: robot bbox(프레임의 5.4%) << 작업영역+테이블(16.9%+25.8%) — robot close-up 에 치우친 구도 아님(PASS). checkerboard calibration target 이 작업영역 안에 확인됨
  - **depth 품질**: workspace/table ROI valid 96.6%(median 646mm), robot ROI valid 77.4%(median 811mm, 금속·좁은 틈새의 흔한 dropout 패턴과 일치). hole(15.2%)은 주로 먼 배경에 집중, robot/workspace 근접영역은 양호
  - **point cloud sanity**: depth_info K 로 직접 역투영(Open3D 미사용, numpy), table plane·robot geometry 존재 확인, 대규모 dropout/clipping 없음(정밀 reconstruction 아님, sanity 수준)
  - **occlusion**: self-occlusion=HIGH(Arm2 witness MARGINAL 과 동일 근거), robot→workspace=LOW, workspace→robot=ACCEPTABLE
  - **external witness independence**: `INDEPENDENT_EXTERNAL_VISION_WITNESS` — Gemini 는 고정 workcell 카메라로 로봇에 부착되지 않음, DaBai(eye-in-hand)와 명확히 구분
  - **static baseline stability**: `STATIC_BASELINE_STABLE` — 로봇 무변동 12프레임 중 인접 11쌍의 grayscale mean abs pixel diff 가 전체 프레임 ~2.22-2.24, robot ROI ~3.43-3.56 범위로 outlier 없이 일관
  - 🔴 **camera angle 는 수치화하지 않음**(§15 지시 그대로, extrinsic calibration 전) — `UNKNOWN / visually oblique`로만 기록. ROI bbox(`roi_measurements.json`)도 자동 segmentation 이 아니라 manual approximation 임을 명시
  - **최종 판정**: `BASELINE_B_PLACEMENT_NEEDS_MINOR_ADJUSTMENT` — robot/workspace coverage·depth 품질·independence·baseline stability 전부 양호/수용가능, 유일한 약점(Arm2 witness MARGINAL)은 완전 거부 사유가 아니라 시야각 조정으로 개선 가능한 종류로 판단. 조정 방향: 하향 기울기를 줄이는 쪽(REDUCE_DOWNWARD_TILT) — 정확한 각도·거리 수치는 증거 없이 제시하지 않음(top margin 85px/11.8% 도 같은 조정으로 함께 완화될 가능성, 별도 요구 아님)
  - 🔴 **이 PASS/CONDITIONAL 은 camera placement qualification 일 뿐** — calibration 완료·absolute pose 측정·Arm2 motion 검증 완료·P006D 완료·RGB-D geometry regression 완료를 의미하지 않음. 기존 extrinsic 을 새 위치에 재사용하지 않음
  - `evidence/P-007F-02B-R1/`(`camera_identity.json`·`depth_stats.json`·`roi_measurements.json`·`robot_visibility.md`·`arm2_witness_analysis.md`·`workcell_coverage.md`·`occlusion_analysis.md`·`pointcloud_sanity.md`·`placement_decision.json`·`README.md` + 원본 12프레임 세트 `frames/` + `rgb_overview.png`·`depth_overview.png`·`pointcloud_sanity.png`·`robot_closeup.png`·`roi_grid_overlay.png`). 기존 Gemini evidence(Baseline A: P-003/P-004/P-006 계열) 전부 불변
  - **Robot motion/torque/position command 미실행. P-007F-03 자동 시작 안 함**
  - 🔴 **후속**: 운영자가 R1 위치를 소폭 조정하는 대신 **카메라 위치 자체를 다시 변경** → R1 위치는 `Baseline B Candidate #1`로 보존(수정/삭제 없음, historical/not-selected). 새 위치는 아래 P-007F-02B-R2에서 R1 결과에 얽매이지 않고 독립 재평가

- [x] **P-007F-02B-R2** Gemini 335L Placement Re-evaluation — **`BASELINE_B_PLACEMENT_ACCEPTED`**
  - 🔴 R1(Candidate #1, `BASELINE_B_PLACEMENT_NEEDS_MINOR_ADJUSTMENT`)의 minor-adjustment 전제 없이, 재배치된 새 위치(`Baseline B Candidate #2`)를 **처음부터 독립적으로** 재평가. `Baseline A`(P-003/P-004)·Candidate #1 evidence 모두 수정/삭제 없이 보존. 로봇 동작 0(position/torque/reset/calibration/action playback/DaBai 재배치/R5 재실행 전부 0)
  - **캡처**: R1과 동일 파이프라인(p620 `bytesize-robotics-camera` 컨테이너·`scripts/run_humble_camera.sh`·`ros2 launch orbbec_camera gemini_330_series.launch.py`·기존 `scripts/p006c_capture_streams.py` 출력경로만 변경) 재사용해 synced 12프레임 신규 취득, 캡처 후 드라이버 정지. `rgb_annotated_overview.png`(robot/Arm2/end-effector/workspace bbox 표시) 신규 추가 — 자동 통계만으로 판정하지 않고 사람이 바로 확인 가능한 artifact 확보
  - **robot coverage**: base→end-effector 확보, 🔴 **Arm1/Arm2 가 R1과 달리 명확히 시각적으로 분리**(검은 원통 Arm1 위 흰 피벗 브래킷 Arm2), Arm3/Arm4 PARTIAL(잔존 self-overlap), **DaBai assembly 가 R1의 UNKNOWN 에서 VISIBLE 로 개선**(렌즈 있는 검은 박스형 모듈 식별 가능)
  - 🔴 **Arm2 witness — `ARM2_WITNESS_GOOD`**(R1 MARGINAL 대비 개선): link visibility·foreshortening(팔이 대각선으로 뚜렷이 뻗어 pitch 평면이 image plane 에 더 가깝게 추정)·trackability(나사머리·관절경계·DaBai 렌즈 등 특징점 풍부) 3항목 개선, self-overlap·DaBai 인접영역 배경대비 약함은 잔존 한계로 공개(완전 제거는 아님)
  - **workspace coverage**: 확보, R1보다 넓음(workspace 32.6% vs 16.9%, table 43.8% vs 25.8%)
  - **robot/workspace balance**(신설 항목): `BALANCED` — robot bbox 5.8%(R1 5.4%와 유사, 작아지지 않음) vs workspace/table 32.6%/43.8%
  - **depth 품질·point cloud sanity**: R1과 동등 수준(overall valid 83.3% vs 84.8%, robot ROI 75.4% vs 77.4%, table ROI 94.5% vs 96.6%) — 재배치로 악화되지 않음
  - **occlusion**: self-occlusion `HIGH→ACCEPTABLE`(개선), robot→workspace=LOW(동일), camera→table=ACCEPTABLE
  - **external witness independence**: `INDEPENDENT_EXTERNAL_VISION_WITNESS`(R1과 동일, 고정 카메라·DaBai 와 구분) / **static baseline**: `STATIC_BASELINE_STABLE`(11프레임쌍 전체 diff ~2.55, robot ROI ~3.57, outlier 없음 — R1과 동일 수준 안정성)
  - **frame margin**: top 90px(12.5%, R1 85px/11.8%와 유사 — 악화 없음)
  - **R1 vs R2 비교표**(`r1_vs_r2_comparison.md`, 실제 evidence만 사용, R2를 좋게 보이도록 임의 조정 안 함) — Arm2 witness·self-occlusion 두 항목만 뚜렷이 개선, 나머지는 동등 유지
  - **최종 판정**: `BASELINE_B_PLACEMENT_ACCEPTED` — `ARM2_WITNESS_GOOD`+`BALANCED`+depth usable 3조건 동시 충족(지시: 이 세 조건이 동시에 나오면 더 완벽한 위치를 찾겠다고 계속 옮기지 않는다) → 현재 위치를 **Gemini Global Workcell Baseline B** 로 freeze 후보화. 잔존 한계(Arm3/Arm4 일부 self-overlap·DaBai 인접 배경대비 약함·top margin 여전히 상대적 타이트) 는 숨기지 않고 공개. 🔴 이 PASS는 camera placement qualification 일 뿐 — calibration/absolute pose/Arm2 motion 검증/P006D/RGB-D geometry regression 완료를 의미하지 않음, 기존 extrinsic 새 위치 재사용 안 함
  - `evidence/P-007F-02B-R2/`(`camera_identity.json`·`depth_stats.json`·`roi_measurements.json`·`robot_visibility.md`·`arm2_witness_analysis.md`·`workcell_coverage.md`·`robot_workspace_balance.md`·`occlusion_analysis.md`·`pointcloud_sanity.md`·`static_baseline.md`·`r1_vs_r2_comparison.md`·`placement_decision.json`·`README.md` + 원본 12프레임 `frames/` + `rgb_overview.png`·`rgb_annotated_overview.png`·`depth_overview.png`·`pointcloud_sanity.png`·`robot_closeup.png`·`roi_grid_overlay.png`). R1·Baseline A evidence 전부 불변
  - **Robot motion/torque/position command 미실행. P-007F-03 자동 시작 안 함**

- [x] **P-007F-03** Gemini External Motion Witness Preregistration — **`P-007F-03 = PASS / CLOSED`**
  - 🔴 사전등록(preregistration)만 — **이번 gate에서 robot command 0회**, R5 재실행 안 함, Gemini/DaBai 위치 변경 안 함
  - **Baseline B freeze 확인**: p620 driver 재시작 후 1프레임 신규 취득(serial `CP263630009D` 재확인) → R2 캡처와 직접 비교, whole-frame/robot-ROI diff가 R2 within-session 안정성과 같은 자릿수(2.838/4.582 vs R2 2.554/3.572), global phase-correlation shift 서브픽셀(~0.12px) → `ABORT_BASELINE_B_CHANGED` 트리거 없음, 카메라 미이동 확인
  - **ROI**: ROBOT_ROI/ARM2_ROI는 R2 좌표 그대로 재사용, STATIC_REFERENCE_ROI(모니터+키보드, 고텍스처 rigid 배경)·DOWNSTREAM_ROI 신설, `roi_annotated.png`로 육안 확인
  - **Static dataset**: 로봇/카메라 전혀 미동 상태에서 70프레임(목표 M≥60 충족) synced RGB+depth 신규 취득(`frames/`)
  - **Visual algorithm 4종 실측 비교**(ORB+RANSAC / Farneback optical flow / edge+phaseCorrelate / template matching) — ORB+BFMatcher+RANSAC(`estimateAffinePartial2D`)을 PRIMARY로 선정(translation+rotation 두 metric과 inlier/match 품질진단을 유일하게 제공). template matching은 정수 픽셀 그리드 한계로 sub-pixel 신호 민감도 0 → 판정에서 제외(raw 데이터는 보존)
  - **Marker 요구**: `MARKER_REQUIRED = false` — ARM2_ROI 단일프레임 keypoint 14개(배경 429개 대비 적음)로 임의 threshold 15 미달이었으나, 실제 10/10 sham trial에서 valid+inlier_ratio 평균 0.983으로 안정 동작 확인. ARM2_ROI noise가 배경 대비 약 10배 큰 것은 숨기지 않고 threshold 산출에 그대로 반영(disclosed limitation)
  - **Sham trial**: 10개(WINDOW=3프레임), robot-relative(ARM2_ROI − STATIC_REFERENCE_ROI) translation 평균 0.592px/stdev 0.397px, rotation 평균 0.180°/stdev 0.299°
  - **Threshold**: `T_translation_px=1.7826`, `T_rotation_deg=1.0778`(sham mean+3·stdev vs engineering floor 중 큰 값 — 둘 다 통계 성분이 지배), static-reference guard `S_translation_px=0.2`/`S_rotation_deg=0.05`(카메라/조명 안정성 별도 게이트)
  - **False-positive**: 산출 threshold로 10개 sham 전부 올바르게 `NO_VISUAL_MOTION` 분류, **0/10**
  - **Depth**: CORROBORATING only(PRIMARY=RGB) — 3개 ROI 모두 median depth 변화 ±1mm 이내로 안정(valid_ratio 75.7~80.1%)
  - **Quality guard·execution protocol·result matrix(4-cell YES/NO)·claim boundary(controller readback=physical angle 미주장 등)·restart/retry 보호 설계(R5 ledger 불변, 신규 ledger 경로 필요) 전부 사전등록**. 다음 Arm2 target angle은 이번 gate에서 확정하지 않음(§18)
  - 🔴 **AST hardware-free 검증**: `scripts/p007f_03_hardware_free_check.py` — 두 gate 스크립트(capture/visual_analysis)에 서보 read/write 포함 로봇 API 참조 0건, true-positive 탐지기 동작 자체도 합성 금지호출 2종으로 증명(`hardware_free_check.txt`, 6/6 PASS)
  - `evidence/P-007F-03/`(`README.md`·`baseline_b_identity.json`·`current_rgb_overview.png`·`roi_definition.json`·`roi_annotated.png`·`static_dataset_manifest.json`·`frames/`(70프레임)·`sham_trials.json`·`trackability.json`·`algorithm_comparison_raw.json`·`robot_relative_sham_distribution.json`·`depth_sham_distribution.json`·`visual_algorithm.md`·`quality_guard.json`·`thresholds.json`·`static_reference_analysis.md`·`false_positive_analysis.md/.json`·`depth_role.md`·`execution_protocol.md`·`result_matrix.md`·`claim_boundary.md`·`preregistration.json`·`hardware_free_check.txt`). P-007F-02B-R2 evidence 전부 불변
  - **실제 Arm2 motion execution 미실행. 다음 gate(실제 1회 execution)는 별도 safety review·사용자 승인 후 개시. 이번 세션은 여기서 STOP**
- [x] **P-007F-04** Single-Motion Execution Safety Review & Preregistration — **`P-007F-04 = PASS / CLOSED`**
  - 🔴 실행 gate 아님 — **robot command 0회**, R5 재실행 안 함, Gemini/DaBai 위치 변경 안 함, P-007F-05 자동 시작 안 함
  - **현재 상태 재확인**(§4): `scripts/p007f_01_readonly_probe.py` 1바이트도 수정하지 않고 재사용(sha 일치) — self-test 10/10 재통과 후 실제 5-round read-only probe 재실행, ID1-6=`[86,154,0,18,93,178]` 완전 안정(variance 0), P-007F-01(2026-10-28)과 정확히 동일. `servo_writes=0`
  - **Baseline B 확인**(§5): p620 driver 재시작(firmware 1.4.60) → 신규 5프레임 synced RGB+depth 취득 → driver 즉시 정지. P-007F-03 baseline 대비 phase-correlation shift 0.256px(서브픽셀), response 1.0008 → `BASELINE_B_UNCHANGED`
  - **Candidate 6개(±1/±2/±3°, 양방향)**: 현재(154) 기준 `scripts/p007f_04_candidates.py`(신규)가 P-007E-R2-R4 FK 모듈(`p007e_r2r4_fk.py`, sha a4e9d2d7...)을 SHA-pin import로 재사용해 joint-limit margin·FK sweep(TCP/DaBai)·swept displacement(exact+sphere-bound cross-check)·self-collision spot-check(SRDF 1차+stride=40 FK 2차, R4의 152/153/154° 선례값과 float 오차 이내로 재현 확인한 뒤 신규 계산)를 산출. 전부 `JOINT_LIMIT_PASS`/`MODEL_LEVEL_COLLISION_CHECK_PASS`, D_conservative 4.963~10.888mm(전부 R4 operator envelope 선례 15mm 이내)
  - **Environment review**(§12): 신규 RGB 육안 확인(이불/모니터/케이블/펜 등 주변 물체) — 예측 envelope(≤10.89mm)가 관찰 clearance(수 cm 이상)보다 현저히 작음, 정밀 3D 모델 부재는 공개
  - **Gemini visual detectability**(§13): camera extrinsic 캘리브레이션 없음 → 엄밀 projection 불가, pinhole 근사(depth 3종×discount 3종=9조합)로 범위만 추정. |delta|=1°는 2/9, 2°는 5/9, **3°는 8/9** 조합에서 threshold(1.7826px) 초과 — 단일 확정 PASS 주장 아님, candidate 간 상대비교에만 사용
  - **선택**: lower, delta=-3°, controller_reported_pre=154 → command_target=151, duration=2000ms(선례 유지). 근거: (1) visual detectability margin이 가장 안정적인 유일한 크기(3°) (2) 현재 Arm2가 upper limit에 더 가까운 상태(margin 28° vs 152°)에서 lower 방향이 그 margin을 늘림(0.4887→0.5061rad), upper 방향은 더 줄임(→0.4014rad) — 두 후보가 변위/충돌/검출가능성 전부 대칭이라 이 비대칭이 유일한 차별화 근거
  - **향후 execution 사전조건 고정**(§17~§23): PRE visual abort 조건 7개(P-007F-03 quality_guard 재인용), controller PRE-state gate(154 불일치 시 `ABORT_BEFORE_MOTION/CONTROLLER_STATE_MISMATCH`, 자동 재계산 안 함), exact approval phrase `EXECUTE P-007F-05 ARM2 154 TO 151 ONCE`(yes/ok/그래/진행/execute 등 비정형 승인 불인정), 신규 ledger `.cache/p007f05_ledger/`(PREPARED→PRE_VALIDATED→ARMED→COMMAND_SENT→POST_COMPLETE, R5 ledger와 완전 분리·R5 ledger 미참조/미수정), No auto-retry/No auto-resend/No automatic return/One command maximum 전부 유지
  - **Result matrix·claim boundary**: P-007F-03 4-cell matrix 그대로 재사용, claim 상한 "physical motion accompanied an authorized command, independently observed by Gemini" — controller=physical/절대정확도/encoder정확도 주장 금지 재확인
  - 🔴 **AST hardware-free 검증**: `scripts/p007f_04_hardware_free_check.py`(신규) — `p007f_04_candidates.py`는 Arm_* API 참조 0건(읽기조차 없음), true-positive 2종 포함 7/7 PASS. Negative test 6/6 PASS(결정론성·candidate 단일성·joint-limit 위반 탐지·sphere≥exact·FK sha-pin 무결성)
  - `evidence/P-007F-04/`(`README.md`·`current_controller_state.json`·`baseline_b_check.json`·`baseline_check_frames/`(5프레임)·`candidate_set.json`·`candidate_raw.json`·`joint_limit_analysis.json`·`fk_sweep.json`·`displacement_envelope.json`·`collision_analysis.md`·`environment_review.md`·`visual_detectability.md/_raw.json`·`candidate_comparison.md`·`selected_candidate.json`·`execution_preconditions.md`·`approval_phrase.txt`·`ledger_design.md`·`claim_boundary.md`·`negative_tests.json`·`hardware_free_check.txt`·`preregistration.json`). P-007F-03 이하 기존 evidence 전부 불변
  - **selected_candidate는 PREREGISTERED_COMMAND_CANDIDATE일 뿐 실행 승인이 아니다. 실제 Arm2 motion command 미실행. P-007F-05는 별도 검수 후 명시적으로 열린 경우에만 개시. 이번 세션은 여기서 STOP**
- [x] **P-007F-05** Single Arm2 Motion Execution — **`CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION — 최종 판정은 검수자 대기`**
  - 🔴 §34 hardware-free preparation 완료 후, 사용자가 정확한 승인 문구(`EXECUTE P-007F-05 ARM2 154 TO 151 ONCE`)를 직접 입력 → §36 실제 실행 진행. **Arm2 servo write 정확히 1회 집행**(id=2, target=151, duration=2000ms)
  - **§36 실행 직전 발견·수정한 버그 2건**(둘 다 write 이전 안전하게 멈춤, hardware_write_count=0 유지): (1) `p007f04_consistency_check.json`의 `decision` 필드가 정확한 리터럴 "PROCEED"가 아니라 설명 문장이라 체커가 오탐 abort(안전) — `decision`/`decision_note` 필드 분리로 수정, 안전한 abort 기록은 `run_1793274190/`에 보존 (2) `build_real_callables()`가 Gemini driver를 실제로 기동시키지 않고 과거 로그만 읽어 identity_valid를 거짓 True로 반환 → 캡처가 빈 이미지로 실패(안전하게 중단) — 모든 Gemini 콜백에 `ensure_driver_fresh()`(매 호출 시 driver 재시작+settle, idempotent) 추가
  - **실제 실행 결과**: Controller PRE=[154×5](전부 일치) → write 1회 → Controller POST=[152×5](commanded target 151과도 다름, raw/readback 불확실성 — P-007F-01 `READBACK_SEMANTICS_UNRESOLVED` 재확인) → **Channel A = YES**(154→152 변경). PRE/POST quality guard PASS(5/5 valid frames, static-reference guard PASS) → Gemini robot-relative translation=0.2297px(<T=1.7826px)/rotation=0.3976°(<T=1.0778°) → **Channel B(Gemini) = NO**. 운영자 SECONDARY 관찰(AskUserQuestion으로 수집, immutable) = **YES**(육안으로 물리적 움직임을 봤다고 보고) — 세 신호가 일치하지 않음, 억지로 맞추지 않고 그대로 공개
  - **Result matrix 기계적용**(사전등록 규칙 변경 없이): Channel A=YES + Channel B=NO → `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION`. 최종 판정(검수자가 세 신호를 보고 결정)은 아직 확정하지 않음
  - **§30 No automatic return/retry 준수**: write 이후 추가 command 전송 없음, `151→154` 복귀 없음
  - **§2 consistency check**: P-007F-04 `joint_limit_analysis.json`의 서술문이 152° candidate POST-state margin(0.4887rad/28.0°)을 "현재(154°)" 값으로 잘못 인용한 documentation typo임을 `candidate_raw.json`(실제 계산)과 대조 확인 — 계산/선택 로직 오염 없음, `P007F04_MARGIN_REPORT_TYPO_CONFIRMED` / `PROCEED`
  - **harness 신규 구현**(2-host 설계, R3/R5 선례와 다름 — Gemini는 p620/ROS2, Arm2는 Jetson/Arm_Lib): `scripts/p007f_05_execution.py`(순수 오케스트레이션, 하드웨어 import 0, self_ast_check 보유)·`scripts/p007f_05_analysis.py`(Channel B, P-007F-03 ORB+RANSAC/threshold 그대로 SHA-pin 재사용)·`scripts/p007f_05_jetson_controller.py`(Jetson 전용, 유일한 `Arm_serial_servo_write` 호출 지점)·`scripts/p007f_05_hardware_wiring.py`(Mac↔p620/Jetson ssh 조율)·`scripts/p007f_05_run.py`(CLI)
  - 🔴 **AST hardware-free 검증**(`scripts/p007f_05_hardware_free_check.py`, 신규): execution.py 하드웨어 import 0 확인 + jetson_controller.py의 `Arm_serial_servo_write` 호출이 정확히 1곳·인자(2,151,2000) 리터럴·루프/except/finally 밖에 있음을 AST 노드로 증명, true-positive 8/8(합성 위반 샘플 탐지)
  - **negative tests NT-01~NT-15 전부 15/15 PASS**(승인문구 불일치·controller PRE≠154·Gemini identity 불일치·Baseline B 변경·PRE quality 실패·POST static-ref 실패·기존 SENT ledger·ARMED crash 재시작·카메라 캡처 실패·write 전 예외·중복 실행 요청·write 후 예외·cleanup 복귀명령 없음·wrong target/servo ID 주입 — 전부 `hardware_write_count=0`)
  - **실측(read-only/camera-only, write 0건)**: p620 Gemini driver 재시작(serial CP263630009D/firmware 1.4.60 재확인) → 5프레임 PRE dry-run capture → Baseline B 확인(P-007F-03 reference 대비 shift 0.226px, 임계 1.0px 이내, `BASELINE_B_UNCHANGED`) → window/session quality guard PASS(5/5 valid) → driver 정지. Jetson에서 controller PRE read 5라운드 재확인 — ID2=[154,154,154,154,154] 완전 안정, Arm_Lib 무결성(disk/egg SHA) PASS, `servo_writes=0`
  - 🔴 **이번 세션 자체 발견·수정 이슈**: (1) `nohup ...&`로 ssh 세션 내 장수 ROS2 프로세스 backgrounding 시 세션이 30s 타임아웃까지 블록 → `ssh -f`로 교체 (2) Python `subprocess.run(capture_output=True)`의 PIPE가 `ssh -f` 잔존 프로세스에 의해 EOF를 못 받아 블록 → DEVNULL로 교체 (3) 컨테이너 내부 `/tmp` ≠ host `/tmp`(기존 메모리 교훈 재발) → capture 출력 경로를 bind-mount workspace 내부로 변경
  - `evidence/P-007F-05/preparation/`(hardware-free 준비 산출물 전체, 변경 없음) + `evidence/P-007F-05/run_1793274190/`(1차 안전 abort 기록, write 0) + `evidence/P-007F-05/run_1793274484/`(실제 실행 run — `README.md`·`execution_manifest.json`·`executable_identity.json`·`operator_precheck.json`·`controller_pre.json`·`gemini_pre_manifest.json`·`pre_frames/`(5프레임)·`pre_quality.json`·`approval_record.json`·`approval_screen_shown.txt`·`workspace_attestation.json`·`ledger_snapshot_final.json`·`command_record.json`·`gemini_post_manifest.json`·`post_frames/`(5프레임)·`controller_post.json`·`operator_observation.json`·`post_quality.json`·`static_reference_result.json`·`visual_motion_result.json`·`depth_corroboration.json`·`result_classification.json`·`execution_result.json`·`baseline_check_frame/`)
  - **최종 판정(검수자 대기)**: `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION`이 사전등록 규칙의 기계적 산출물일 뿐, PHYSICAL_MOTION_EVIDENCE_SUPPORTED/NO_OBSERVABLE_RESPONSE/EXPERIMENT_INVALID 여부를 포함한 최종 해석은 사용자/ChatGPT가 Channel A·Channel B·운영자 관찰을 독립적으로 검수한 뒤 내린다. 이번 세션은 여기서 STOP — 추가 command 전송 없음
- [x] **P-007F-06** Offline Forensic Analysis of P-007F-05 Visual Evidence — **`F2 = POST_HOC_LOCAL_CHANGE_DETECTED_BUT_MOTION_NOT_ESTABLISHED`**
  - 🔴 완전 OFFLINE — **robot command 0회**, 새 촬영 0, `evidence/P-007F-05/run_1793274484/`에 저장된 PRE/POST RGB+Depth만 입력
  - **Source integrity**(§4): SHA256/size/timestamp manifest 최초 freeze(기존 manifest 없었음), 선언된 프레임 수(5/5 PRE, 5/5 POST)와 실제 파일 수 일치 확인
  - **ORB 재현**(§8): P-007F-05가 보고한 translation=0.2297px/rotation=0.3976°를 오차 0.05 이내로 정확히 재현
  - **Within-window noise control**(§13, 핵심): PRE 5프레임 10쌍 + POST 5프레임 10쌍(총 20개) 자연 변동 분포를 PRE→POST 신호와 6개 독립 metric(ORB translation/rotation, optical-flow median magnitude, raw abs-diff mean, edge centroid shift, phase-correlation magnitude)으로 비교. ARM2_ROI는 3/6(flow/abs-diff/phase-correlation)에서 noise mean+3σ·max 모두 초과, STATIC_REFERENCE_ROI는 0/6
  - 🔴 **자체발견·수정**: 1차는 4개 ROI(ARM2/STATIC/END_EFFECTOR/DOWNSTREAM)만 noise-control 검사했으나, 단순 registered-diff-mean 수치에서 DABAI_ROI/ARM4_ROI/ARM3_ROI가 STATIC_REFERENCE_ROI보다 높게 나오는 것을 보고 9개 ROI 전체로 확장 → ARM1_ROI·STATIC_REFERENCE_ROI를 제외한 거의 모든 ROI가 noise를 초과 — 변화가 Arm2 단독이 아니라 로봇 점유 영역 전반에 diffuse함을 확인. 이 확장이 없었다면 잘못된 F1(ROBOT_LOCALIZED 단정) 결론이 나올 뻔했음(자체 정정)
  - **Directional coherence**(§15): optical flow top-25%-magnitude 픽셀의 원형통계 resultant length — ARM2_ROI R=0.669 vs STATIC_REFERENCE_ROI R=0.203. (최초 구현은 "mag>1e-6 비율"이라는 약한 proxy였고 거의 항상 100%가 나와 의미가 없었음을 자체발견, 방향 일치를 실제로 측정하는 R로 교체)
  - **Arm2 trackability**(§21): keypoint 14개(P-007F-03과 정확히 일치)이나 y축 분산 2.2px(ROI 높이 70px의 8.6%)로 거의 한 줄에 몰려 rotation 추정이 구조적으로 불안정 → `MARKER_RECOMMENDED_FOR_FUTURE`(이번 gate에서 marker 미부착)
  - **Depth/point cloud**(§16~18): RGB↔Depth 간 23.8mm 실제 baseline 확인 → `DEPTH_SPATIAL_LOCALIZATION_LIMITED` 공개, point cloud forensic은 미검증 정렬을 근거로 억지로 수행하지 않고 `NOT_ATTEMPTED_DUE_TO_UNVERIFIED_REGISTRATION`
  - **Synthetic sensitivity test**(§20, `NOT REAL MOTION EVIDENCE` 명시): 0.25~5px/0.25~3° 합성 왜곡으로 ORB/flow/edge/phase-correlation 검출 특성 특성화
  - **운영자 YES 재검토**(§22): `UNKNOWN` 유지 — evidence가 증명도 반증도 하지 않음(actual motion/twitch/backlash/vibration/downstream-localized 전부 "diffuse 패턴과 정합적"이나 확정 불가)
  - **Controller 154→152 재검토**(§23): `physical shaft moved 2°` 해석 계속 금지, P-007F-01 `READBACK_SEMANTICS_UNRESOLVED` 유지, 이번 영상 forensic과 독립적으로 취급(서로 영향 없음)
  - 🔴 **P-007F-05 사전등록 결과 소급 변경 없음**(§25): `P-007F-05 PRIMARY = CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION`(불변) / `P-007F-06 POST-HOC = F2`로 완전히 분리 기록
  - **Figures**(§26): 10개 생성(01 PRE/POST overview ~ 10 synthetic sensitivity)
  - 🔴 **AST hardware-free 검증**: `scripts/p007f_06_hardware_free_check.py`(신규) — `p007f_06_forensics.py`가 Arm_Lib/smbus/rclpy/serial/VideoCapture 전혀 미참조, `robot_command_count=0` PASS
  - `evidence/P-007F-06/`(`README.md`·`source_manifest.json`·`source_integrity.json`·`roi_definition.json`·`registration_analysis.json`·`orb_reproduction.json`·`optical_flow_analysis.json`·`registered_difference_analysis.json`·`edge_analysis.json`·`phase_correlation_analysis.json`·`within_window_noise.json`·`spatial_localization.json`·`coherence_classification.json`·`depth_analysis.json`·`rgb_depth_alignment.json`·`pointcloud_analysis.json`·`synthetic_sensitivity.json`·`arm2_trackability_numeric.json`·`arm2_trackability_review.md`·`operator_observation_review.md`·`controller_readback_review.md`·`forensic_classification.json`·`claim_boundary.md`·`hardware_free_check.txt`·`figures/`(10개)). P-007F-05 이하 기존 evidence 전부 불변
- [x] **P-007F-07 Phase A** Marker + Continuous External Motion Witness Preparation — **`PASS / CLOSED — READY_FOR_MARKER_SETUP=YES / READY_AND_ARMED=NO / MOTION_AUTHORIZED=NO`**
  - 🔴 완전 OFFLINE — **robot command 0, servo write 0**. P-007F-03/04/05/06 evidence 전부 불변(git diff 확인)
  - **Marker 선정**(§4): `cv2.aruco DICT_4X4_50`(`scripts/p007f_07_marker_detector.py`, modern `ArucoDetector` API, `CORNER_REFINE_SUBPIX`). 합성 이미지로 27~45px 전구간 검출 검증(이상적 조건 기준, 공개)
  - **Marker 크기**(§5): 실측 ARM2_ROI depth 677.6mm(P-007F-06) + color fx=610.5px pinhole 계산 → 50mm=45px(preferred 충족), 40mm=36px(minimum만 충족) → `MARKER_SIZE=PROPOSED`, primary 50mm/fallback 40mm
  - **부착 위치**(§6/§7): 사진 기반 PROPOSED(흰색 디스크/호른 표면, 회전축에서 오프셋 — 축 정중앙 금지 이유까지 명시), ALTERNATE 후보(두 joint 사이 짧은 원통 링크)도 병기, `marker_mount_instruction.md` + `marker_candidate_overlay.png`. SECOND_MARKER=`RECOMMENDED`(downstream-local 구분용)
  - **Trajectory classifier**(§13~§18): `scripts/p007f_07_trajectory_classifier.py` — M1~M6/MX 전체 taxonomy, background-compensated(§14). **synthetic test 10/10 PASS**(Case A~E + §18 critical PRE=POST transient regression + marker-loss/background-instability abort 2종 + threshold formula 2종)
  - 🔴 **자체발견·수정**: `translation=hypot(dx,dy)`는 항상 ≥0이라 부호반전 진동(+3→-3px)에서 magnitude가 중간에 threshold 아래로 안 내려가 Case C 최초 실패 → `_count_sign_reversals()`(dominant axis 부호반전 카운트) 추가 + oscillation 판정을 hold/transient 분기보다 먼저 확인하도록 순서 재배치(2차 버그: end_above_fraction 중간값이 그냥 M6로 새던 것도 함께 수정)
  - **One-write architecture**(§23): `scripts/p007f_07_jetson_controller.py.do_write()`가 유일한 `Arm_serial_servo_write` 호출 지점(AST로 call-site=1·루프/except/finally 밖 증명), `scripts/p007f_07_execution.py`는 그 모듈조차 import하지 않음(AST로 증명, Phase A 코드 경로 어디서도 도달 불가)
  - **Ledger**(§24): 신규 `.cache/p007f07_ledger/`(미생성, 전부 tempdir로 테스트), `PREPARED→READY_AND_ARMED→APPROVAL_RECEIVED→COMMAND_SENT→POST_CAPTURE_COMPLETE→COMPLETE` 6단계, COMMAND_SENT 이후 전 상태에서 재시도 차단(P-007F-05에서 발견된 "POST_COMPLETE는 차단 안 됨" 버그를 이번엔 처음부터 방지)
  - **승인→명령 fast path**(§21/§22): `quick_invariant_checks()` — injected callable 5개(controller_state_unchanged/camera_stream_alive/marker_visible/background_reference_valid/ledger_not_already_sent)만 호출, 실패 시 즉시 ABORT_BEFORE_MOTION·재시도 없음. 구조상 build/test/camera-init/ssh-config/file-transfer가 이 경로에 없음
  - **Two-host clock 설계**(§11): orchestrator를 p620에서 실행해 continuous-capture host_monotonic_ns와 SSH-dispatch timestamp가 **같은 clock domain**을 공유하도록 설계(Jetson 쪽 clock은 참고용). COMMAND_INTERVAL(ssh 송신~수신)을 불확실성 구간으로 그대로 기록, ms 정밀도 주장 안 함
  - **negative tests**: `p007f_07_execution.py` NT-01~10 전부 PASS(ledger restart 차단 2종, quick-invariant 성공/실패/예외/누락 4종, 승인문구 exact-match, self_ast_check)
  - 🔴 **AST hardware-free 검증**: `scripts/p007f_07_hardware_free_check.py`(신규) — execution.py/marker_detector.py/trajectory_classifier.py/synthetic_tests.py/continuous_capture.py 전부 금지 import/call 0, jetson_controller.py 단일 write call-site 증명. 전체 PASS
  - `evidence/P-007F-07/phase_a/`(`README.md`·`design.md`·`marker_spec.md`·`marker_mount_instruction.md`·`marker_setup/marker_candidate_overlay.png`·`continuous_capture_design.md`·`timestamp_design.md`·`two_host_sync_design.md`·`marker_detector_design.md`·`trajectory_classifier.md`·`background_reference_design.md`·`synthetic_test_results.json`·`transient_return_regression.json`·`one_write_static_check.txt`·`hardware_free_check.txt`·`ledger_design.md`·`phase_a_acceptance.json`)
- [x] **P-007F-07 — Marker Setup Reference Capture** (사용자 지시, Phase A 직후) — Frozen Gemini 335L Baseline B 시점에서 실제 RGB frame 재확보(`scripts/p007f_07_marker_setup_capture.py`, 신규). `robot command=0/servo write=0`(Jetson/Arm_Lib 미import, AST 자기검증), Gemini identity 재확인(serial/firmware 일치), 5 frame 촬영 성공 → Laplacian variance 최대 프레임(`0004-color.png`) 채택. Baseline B 변경 없음 실측 확인(phase-correlation shift 0.49px < 1.0px). `marker_mount_instruction.md`의 PRIMARY/ALTERNATE/SECONDARY 후보를 기존 ROI 좌표(P-007F-06) 기준 근사 박스+범례로 신규 프레임 위에 재overlay(`CANDIDATE_ONLY`, 최종 확정 안 함). `evidence/P-007F-07/marker_setup_capture/`(`README.md`·`rgb_reference.png`·`marker_candidate_overlay.png`·`capture_metadata.json`·`raw_frames/`)
  - **다음은 사용자가 `rgb_reference.png`/`marker_candidate_overlay.png`를 검토해 marker 최종 위치를 확정 → marker 출력/부착 → P-007F-07A(MARKER STATIC QUALIFICATION). 이번 세션은 여기서 STOP, 실제 motion 자동 시작 안 함**
- [x] **P-007F-07 — Arm2 Marker Mount Close-up** (사용자 지시, 전체 로봇이 작게 보여 실물 부착면 판단이 어렵다는 문제 해결) — 이미 확보된 `marker_setup_capture` 프레임(재촬영 0)에서 Arm2 주변 확대(`scripts/p007f_07_marker_mount_closeup.py`, 신규, 하드웨어/카메라 API 미참조 AST 자기검증). 좌표 그리드 반복 판독(자동검출 아님)으로 Arm2 rotation axis(x≈712)/rigid link/next-joint seam 식별, 기존 PRIMARY_A/B(legacy, Phase A 저해상도 프레임 기준)를 새 프레임에 재배치해 보니 당시 박스가 디스크 영역을 부분적으로 벗어나 있었음을 확인. M1(disc top, rigid_with_arm2=YES·정의상)/M2(어깨 조인트 상단 브래킷, UNCERTAIN·DOFBOT 자체 joint 번호체계는 P-007F-01 READBACK_SEMANTICS_UNRESOLVED로 미해결)/M3(servo 라벨 스티커, UNCERTAIN) 3개 후보를 color→depth 국소 실측(각 후보별 median depth, 샘플 수 기록)으로 mm 환산. 🔴 M1은 established witness 영역이지만 최대 부착 가능 30mm가 Phase A 자체 가독성 최소선(~33mm)에 미달 가능성 발견, M2는 50mm까지 여유 있게 충족(52.6px, preferred 상회), M3는 20mm조차 세로로 미달해 제외 — Cursor 1순위 제안=M2(백업=M1), 최종 미확정. `evidence/P-007F-07/marker_mount_closeup/`(`01_arm2_original_context.png`·`02_arm2_closeup.png`·`03_marker_surface_candidates.png`·`04_candidate_size_comparison.png`·`marker_surface_review.md`·`candidate_report.json`)
  - **다음은 사용자/ChatGPT 검수자가 `03_marker_surface_candidates.png`를 보고 M1/M2/M3 중 최종 위치+크기를 확정 → marker 출력/부착 → P-007F-07A. MOTION_AUTHORIZED=NO, MARKER_LOCATION_FINAL=NO 그대로**
- [x] **P-007F-07 — 30mm ArUco Print Sheet** (사용자 지시, 출력용 파일 준비 누락분 보완) — Phase A에서 이미 사전등록된 `scripts/p007f_07_marker_detector.py`의 `DEFAULT_DICT=DICT_4X4_50`/`PRIMARY_MARKER_ID=0`을 재확인해 그대로 재사용(이번 작업에서 새로 정하지 않음, assert로 실행 시점 재검증). `scripts/p007f_07_marker_print_sheet.py`(신규)가 A4 PDF/PNG를 matplotlib 단일 mm-좌표axes로 생성(figsize=A4 inch 정확, dpi 무관한 벡터 배치) — marker 바깥 검정 사각형 30.0mm, quiet zone 가이드(+3mm), 30mm reference ruler bar, `PRINT AT 100%/ACTUAL SIZE/DO NOT FIT TO PAGE` 라벨 포함. 🔴 **2단계 실측 검증**: (1) PDF 바이트에서 `/MediaBox` 직접 정규식 파싱 → A4(595.28×841.89pt) 일치 확인 (2) 동일 레이아웃을 dpi=300 PNG로 렌더링 후 marker 영역 어두운 픽셀 bounding box 실측 → 29.972×29.972mm(허용오차 ±0.5mm 이내 PASS, 0.028mm 차이는 PNG 래스터 양자화 오차일 뿐 PDF 자체는 벡터). `evidence/P-007F-07/marker_print/`(`aruco_4x4_50_id0_30mm_A4.pdf`·`aruco_4x4_50_id0_30mm.png`·`print_verification.md`·`geometry_report.json`)
  - **다음은 사용자가 PDF를 100%/Actual Size로 출력 → 30mm reference bar를 자로 실측 확인 → M1 면에 30×30mm가 평평하게 들어가는지 확인. MOTION_AUTHORIZED=NO, MARKER_LOCATION_FINAL=NO 그대로**
- [x] **P-007F-07A — M2 Kinematic Membership Verification** (사용자 지시, "M2가 servo ID2 downstream인지 URDF로 확인") — `M2_DOWNSTREAM_OF_ID2 = YES`. 🔴 servo ID2↔URDF joint 매핑은 새로 추정하지 않고 **이미 검증된 `scripts/p007e_r2r4_fk.py`(P-007E-R2-R4, 실제 P-007F-05 §36 실행 안전판정 근거로 쓰인 모듈)의 `full_angles(): base["Arm2_Joint"]=id2_lib_deg` 소스에서 직접 재확인**(SHA-pin, 수정 없음). URDF parent/child 그래프를 독립적으로 BFS 재순회해 Arm2_Joint의 전체 downstream link 집합을 구하고, FK 모듈의 기존 `RIGID_MOVING_LINKS`가 그 부분집합임을 교차검증(자체발견·수정: 최초 "완전 집합 일치"로 비교해 그리퍼 손가락 link 차이로 FALSE NEGATIVE 발생 → "부분집합" 불변식으로 수정). M2(cream bracket)는 Arm2_Link 말단/Arm3_Link 시작부 중 하나(camera-robot extrinsic 미보유로 확정 불가) — 둘 다 downstream이라 YES 판정에 영향 없음. STL mesh bounding-box 길이 프로파일(Arm1_Link 43.5mm 짧은 disc / Arm2_Link·Arm3_Link 각 107.0mm 긴 쌍둥이 링크, 실측)이 close-up 사진 구조와 정성적으로 일치. 🔴 **corollary 발견**: M1(disc)은 Arm1_Link=Arm2_Joint의 parent이자 FK 모듈 `FIXED_LINKS`에 명시 포함 — **ID2 단독 회전으로는 움직이지 않음**(P-007F-05/06 사전등록 결과 소급 변경 없음, 참고 단서로만 기록). `evidence/P-007F-07/m2_kinematic_verification/`(`report.md`·`kinematic_chain.txt`·`m2_mapping.png`·`membership_report.json`)
  - **다음은 검수자 승인 후 M2를 marker 위치로 확정 → 적정 크기 재결정(이미 30mm 출력 완료) → static qualification. MOTION_AUTHORIZED=NO**
- [x] **P-007F-07B Phase B1 — M2 50mm Marker Print** (사용자 지시, 출력과 정적검증 분리) — robot/servo 0, 실제 motion 0. `DICT_4X4_50`/`PRIMARY_MARKER_ID=0`을 30mm 산출물과 동일하게 재확인(새로 정하지 않음). `scripts/p007f_07b_marker_print_50mm.py`(신규)가 `scripts/p007f_07_marker_print_sheet.py`의 `build_figure()`를 SHA-pin import로 그대로 호출하고 `BLACK_SQUARE_MM`/`MARKER_PX`/`OUT_DIR`만 override(원본 파일 바이트 불변 매 실행 실측 확인). 🔴 **자체발견·수정**: 재사용 중 reference bar가 `BLACK_SQUARE_MM` 폭으로 그려지는데 눈금/라벨은 `(0,10,20,30)`/`"30mm"`로 하드코딩된 잠재 버그 발견 → 눈금/라벨을 `BLACK_SQUARE_MM`에서 유도하도록 원본 모듈 근본 수정, 기존 30mm 산출물도 재생성(내용 동일, PDF 메타데이터만 갱신 확인). geometry 2단계 검증: PDF `/MediaBox` A4 일치 + dpi=300 PNG 실측 49.953×49.953mm(±0.5mm 이내 PASS, PDF 벡터가 authoritative). `evidence/P-007F-07/marker_print_50mm/`(`aruco_4x4_50_id0_50mm_A4.pdf`·`aruco_4x4_50_id0_50mm.png`·`print_verification.md`·`geometry_report.json`)
  - **Phase B1 STOP — 다음은 사용자가 50mm PDF를 100%/Actual Size로 출력 → reference bar 50mm 실측 확인 → M2에 평평하게 부착 가능한지 확인한 뒤에만 Phase B2(물리 부착) 이후 진행. MOTION_AUTHORIZED=NO, READY_FOR_MOTION_PREREGISTRATION=NO**
- [x] **P-007F-07B Phase B2 이후 — 정지 qualification** — 사용자 확인 `50 mm 실측 OK, M2 평면 부착 완료`. 마커·로봇 미이동. Gemini 119 frame / 11.981 s. Baseline B shift 0.470 px (`unchanged`). ID 0 이 두 군집으로 갈린다: 상단 41.76 px × 12 frame, 책상 카드 21.85 px × 47 frame. 검출률 0.496, 혼합 평균 변 25.90 px. **`MARKER_STATIC_QUALIFICATION = FAIL`**. `MOTION_AUTHORIZED = NO`. single-motion preregistration 은 열지 않음. `evidence/P-007F-07/marker_static_qualification/report.md`
- [x] **P-007F-07B-R1 사전확인 중단** — 책상 단일 카드는 없고, 키보드 위 ArUco 시트가 ID 0으로 1회 검출(중심 1085.8,371.5 / 15.41 px, M2 군집에서 395 px). Baseline B shift 0.471 px. 본촬영 미시작. `ABORT / DUPLICATE_ID0_STILL_PRESENT`. `MARKER_STATIC_QUALIFICATION_R1` 미발급. `MOTION_AUTHORIZED=NO`
- [x] **P-007F-07B-R1 attempt 2** — 물리 ID0는 M2 하나(책상 카드·키보드 시트 없음). 검출은 사전 1/24, 본촬영 17/258(6.6%), 변 길이 평균 41.65 px. Baseline B shift 0.463 px. 밝기·선명도는 검출/미검출이 같다. `MARKER_STATIC_QUALIFICATION_R1 = FAIL`. `MOTION_AUTHORIZED=NO`
- [x] **P-007F-07C — M2 marker mount geometry redesign** — 설계만. 17프레임 solvePnP. 영상 각 30.8°는 물리 장착각으로 쓰지 않음. 권장 쐐기 26.7° ESTIMATED, 판 56×56 mm, 경첩은 DaBai 쪽. `MOUNT_REDIRECTION_RECOMMENDED` / `DESIGN_READY`. 간섭 `POTENTIAL_INTERFERENCE`. robot/servo/torque 0. `MARKER_MOUNT_INSTALLATION_AUTHORIZED=NO`
- [x] **P-007F-08 Phase A — continuous external motion witness** — 마커 ID 검출 없이 LK+phase correlation. 정지 228 frame / 26.0 s. M2 특징점 13/13 생존, relative 최대 0.155 px. T_motion 은 바닥값 1.0 px로 고정(통계값 0.179). `READY_AND_ARMED=YES`. Phase B 미실행. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-007F-08A — motion observability qualification** — offline. `T_Gemini<-RobotBase` 없음. ±1/±2/±3° 픽셀 예측을 만들지 않음. `T_motion=1.0 px` 유지. `OBSERVABILITY_UNRESOLVED`. `READY_FOR_PHASE_B_REVIEW=NO`. 추가 calibration 없음. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-007 final closure** — 새 실험 없음. `CLOSED / BLOCKED`. physical motion `UNRESOLVED`. 추가 진단 `STOPPED`. P-006D 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`. `evidence/P-007/P-007_FINAL_CLOSURE.md`
- [x] **P-008 — shoe geometry pipeline** — 실제 신발 point cloud 없음. `BLOCKED_REAL_SHOE_INPUT_REQUIRED`. 촬영·파이프라인 없음. P-009 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008A — real shoe static dataset** — Gemini Baseline B에서 정지 운동화 RGB/depth/cloud 5장. identity 일치. FOV 충분. hash 서로 다름. 기하 추출 없음. P-008 미재개. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R1 — reference capture geometry** — capture_01만. SHA 일치. 기존 voxel/SOR/RANSAC/cluster 수치를 재사용. 신발 envelope는 매트 신발을 분리하지 못하고 왼쪽 cluster 8을 선택. `FAIL_SHOE_ISOLATION`. 파라미터 미동결. capture_02~05 미사용. R2/P-009 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R1D — isolation failure diagnosis** — capture_01만. 매트 중심 원판은 RAW·voxel·SOR에 있고 clustering 입력에는 0점. height 반공간이 제거. id 3은 0.20 m 밖. `PASS_DIAGNOSIS`. 파라미터 불변. 재시도/R2/P-009 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R1E — polygonal prism extraction** — capture_01만. viewpoint prism이 신발 원판을 전부 유지. cluster 1은 매트와 연결. `FAIL_CLUSTERING`. height·tolerance 불변. 파라미터 미동결. 재시도/R2/P-009 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R1F — Euclidean connectivity diagnosis** — capture_01 prism만. 10 mm component 재현. 환경 합류 3.6 mm, 원판 전체 연결 26.7 mm. `PASS_DIAGNOSIS` / `NOT_FEASIBLE`. tolerance 불변. 파라미터 미동결. 재시도/R2/P-009 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R1G — RGB↔depth correspondence** — capture_01만. depth↔cloud는 중앙값 1.08 mm이나 사전 p95 0.005 m과 양수 depth 비율 0.99를 넘지 못함. 공식 extrinsic 행렬은 저장되지 않음. RGB 투영·segmentation 없음. R1H/재시도/R2/P-009 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R3 — 6D surface pose** — R2 147개 점 위치는 유지하고 tool frame만 붙였다. 법선은 k=20 국소 PCA, 부호는 카메라 방향. 퇴화 frame 0. 앞코 쪽 index 125에서 자세 변화가 160.8°이고 기하 불연속은 1건. quaternion 부호는 정렬. base 변환과 로봇 명령은 없음. 파라미터 동결. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R2 — shoe surface path** — capture_01 신발 후보만. 장축을 덮는 한쪽 측면의 위쪽 점을 2 mm 간격으로 이음. ordering 위반 0. crest와 boundary는 만들지 않음. 파라미터 동결. capture_02–05 미사용. 6D waypoint 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R1GEO-V — frozen hold-out** — capture_02–05에 R1GEO 동결 파라미터를 그대로 적용. 4/4 PASS. 파라미터 변경 0. segmentation track `CLOSED`. 경로 추출은 시작하지 않음. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R1GEO — geometry segmentation** — capture_01만. RANSAC 지지 평면과 Euclidean clustering으로 신발 후보를 하나 골랐다. RGB는 특징으로 쓰지 않음. R1H 판정은 유지. 파라미터 동결. capture_02–05 미사용. 추가 segmentation 없음. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008-R1H — feature separability** — capture_01 진단 영역만. 색상 spatial-block은 LOW, 지지면 높이는 HIGH. 고정한 접촉 띠의 매트 쪽은 0점이라 접촉 분리는 `NOT_IDENTIFIABLE`. 방법은 `SEGMENTATION_STOP_RULE`. threshold는 고정하지 않음. capture_02–05 미사용. segmentation/R1I/재시도/R2/P-009 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`
- [x] **P-008B — Dataset v2** — 계약 고정 후 SW D2C와 optical extrinsic을 확인하고 정지 운동화 5장을 저장. depth↔cloud 5/5. color 대응은 내부 일치만 통과, 등록 정확도는 주장하지 않음. capture_01은 development, 02–05는 holdout. P-008A는 historical. segmentation 없음. R1H/재시도/R2/P-009 미개방. robot/servo 0. `MOTION_AUTHORIZED=NO`

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
| 30 | 2026-10-29 (세션 `66eb4937`) | **P-007F-02B-R1** Gemini 335L Global Workcell Placement Qualification — 사용자가 재배치한 Gemini 335L 실제 RGB/Depth 12프레임 캡처(p620 기존 컨테이너/스크립트 재사용, 로봇 동작 0)·분석. robot/workspace coverage·depth 품질·independence·baseline stability 양호, Arm2 witness만 MARGINAL(가파른 하향 시야각→pitch 평면 foreshortening+링크 self-overlap, 기하학적 추론). 하향 기울기 완화 조정 권고, 각도/거리 수치 임의 제시 안 함 | `BASELINE_B_PLACEMENT_NEEDS_MINOR_ADJUSTMENT`
| 31 | 2026-10-29 (세션 `66eb4937`) | **P-007F-02B-R2** Gemini 335L Placement Re-evaluation — 운영자가 R1 이후 위치 자체를 재변경, R1(Candidate #1)은 historical 보존하고 새 위치(Candidate #2)를 R1 결과에 얽매이지 않고 독립 재평가(동일 파이프라인 재사용, 로봇 동작 0). Arm2 witness MARGINAL→GOOD(Arm1/Arm2 분리+DaBai 식별 가능), self-occlusion HIGH→ACCEPTABLE, BALANCED 신설판정, depth/independence/stability는 R1과 동등 유지(악화 없음) | `BASELINE_B_PLACEMENT_ACCEPTED — Gemini Global Workcell Baseline B 로 freeze 후보화`
| 32 | 2026-10-29 (세션 `66eb4937`) | **P-007F-03** Gemini External Motion Witness Preregistration — DESIGN ONLY(robot command 0). Baseline B 미변경 재확인(phase-correlation 서브픽셀), 신규 70프레임 static dataset, 4종 visual algorithm(ORB+RANSAC/optical flow/edge+phaseCorrelate/template matching) 실측 비교해 ORB+RANSAC을 PRIMARY 선정, `MARKER_REQUIRED=false`(10/10 valid, 배경 대비 noise 10배를 disclosed limitation으로 threshold에 반영), sham 10trial 기반 threshold 산출 후 false-positive 0/10 달성, result matrix·claim boundary·restart/retry 보호 전부 사전등록. AST hardware-free 검증 6/6 PASS | `P-007F-03 = PASS / CLOSED — 실제 execution 은 별도 승인 대기`
| 33 | 2026-10-29 (세션 `66eb4937`) | **P-007F-04** Single-Motion Execution Safety Review & Preregistration — robot command 0. 현재 상태(154)·Baseline B freeze 재확인 후 ±1/±2/±3° 양방향 6개 candidate를 P-007E-R2-R4 FK 모듈 SHA-pin 재사용으로 평가(joint-limit/FK/displacement envelope/self-collision 전부 PASS, R4 선례값과 float 오차 이내 재현 확인). Gemini visual detectability 근사추정(camera extrinsic 無 → pinhole 근사 9개 가정 조합)에서 |delta|=3°만 8/9 조합 threshold 초과 → lower -3°(154→151) 선택(joint-limit margin 관리 근거: upper limit에 더 가까운 현재 상태에서 margin을 늘리는 방향). 향후 execution 사전조건(PRE abort 7종·controller PRE-state gate·exact approval phrase·신규 ledger `.cache/p007f05_ledger/` 설계, R5 ledger 완전 분리) 전부 사전등록. AST hardware-free 7/7, negative test 6/6 PASS | `P-007F-04 = PASS / CLOSED — P-007F-05 는 별도 검수·명시 승인 후에만 개시`
| 34 | 2026-10-29 (세션 `66eb4937`) | **P-007F-05 Hardware-free Preparation** — robot command 0. §2 consistency check로 P-007F-04 서술문 오기(152° candidate 값을 "현재 154°"로 잘못 인용)를 발견·`PROCEED` 판정(계산/선택 로직 자체는 정상). 신규 2-host 실행 harness 구현(Gemini=p620/ROS2, Arm2=Jetson/Arm_Lib — R3/R5의 Jetson 단일호스트 설계를 그대로 재사용 불가함을 소스 확인 후 판단해 새 아키텍처로 작성). AST hardware-free 검증(execution.py 하드웨어 import 0 + jetson_controller.py 단일 write callsite 인자/루프/except 위치까지 AST로 증명, true-positive 8/8) + negative test NT-01~15 전부 15/15 PASS. 실측 read-only/camera-only dry-run: Gemini identity(serial/firmware) 재확인, Baseline B 확인(P-007F-03 reference 대비 shift 0.226px), PRE quality guard PASS(5/5 valid frames), Jetson controller PRE read 5라운드(154 완전 안정, Arm_Lib 무결성 PASS), write 0건. 세션 중 ssh 백그라운딩/컨테이너 경로 이슈 2건 자체발견·수정(nohup→ssh -f, PIPE→DEVNULL, 컨테이너 내부 /tmp 경로 함정) | `HARDWARE-FREE PREPARATION DONE / READY_FOR_EXPLICIT_APPROVAL = YES — 실제 Arm2 motion 미실행, ChatGPT 검수+사용자 승인 문구 입력 후에만 §36 개시`
| 35 | 2026-10-29 (세션 `66eb4937`) | **P-007F-05 §36 Real Execution** — 사용자가 정확한 승인 문구(`EXECUTE P-007F-05 ARM2 154 TO 151 ONCE`)를 직접 입력해 실제 실행 개시. 실행 직전 자체발견 버그 2건을 write 이전에 안전하게 멈춰 수정(consistency decision 필드 exact-match 실패/Gemini driver 미기동으로 인한 identity 오탐) 후 재시도. Arm2 servo write 정확히 1회 집행(id=2, target=151, duration=2000ms). Controller PRE=[154×5]→POST=[152×5](commanded target과도 다름, raw/readback 불확실성 재확인) → Channel A=YES. Gemini PRE/POST quality guard PASS(5/5) → robot-relative translation=0.23px/rotation=0.40°, threshold(1.78px/1.08°) 미달 → Channel B=NO. 운영자 SECONDARY 관찰(immutable)=YES(육안 목격 보고) — 세 신호 불일치를 그대로 공개, 억지로 조정하지 않음. 사전등록 result matrix 기계적용 → `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION`. write 이후 추가 command/복귀 없음 | `CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION — 최종 판정은 검수자(ChatGPT/사용자) 대기`
| 36 | 2026-10-29 (세션 `66eb4937`) | **P-007F-06 Offline Forensic Analysis** — robot command 0, 새 촬영 0. P-007F-05 저장된 PRE/POST RGB+Depth 재분석. ORB 재현(오차 0.05 이내) 후 within-window noise control(PRE 10쌍+POST 10쌍 자연변동 vs PRE→POST 신호, 6 metric)로 ARM2_ROI 3/6 metric이 noise 초과·STATIC_REFERENCE_ROI 0/6 확인. 자체발견·수정: 최초 4-ROI 검사를 9-ROI 전체로 확장해(DABAI/ARM4/ARM3도 static보다 높게 나오는 것을 보고) 변화가 Arm2 국소가 아니라 로봇 영역 전반에 diffuse함을 재확인(잘못된 F1 결론을 자체 정정). directional coherence 약한 proxy(mag>1e-6 비율)를 원형통계 R로 교체(자체발견·수정). Arm2 trackability keypoint 14개(P-007F-03 일치)이나 y축 분산 2.2px로 rotation 추정 불안정 → MARKER_RECOMMENDED_FOR_FUTURE. RGB/Depth 23.8mm 실제 baseline 확인 → DEPTH_SPATIAL_LOCALIZATION_LIMITED, point cloud 억지 수행 안 함. 운영자 YES는 UNKNOWN 유지. P-007F-05 사전등록 결과 소급 변경 없음(분리 기록). figure 10개, AST hardware-free PASS | `F2 = POST_HOC_LOCAL_CHANGE_DETECTED_BUT_MOTION_NOT_ESTABLISHED — P-007F-05 historical result 불변, P-007F-07 NOT OPEN`
| 37 | 2026-10-29 (세션 `66eb4937`) | **P-007F-07 Phase A** Marker + Continuous External Motion Witness Preparation — robot command 0, servo write 0, 완전 offline. cv2.aruco DICT_4X4_50 선정, 실측 기반(depth 677.6mm/fx 610.5px) marker 크기 50mm(fallback 40mm) 계산, 부착 위치 PROPOSED(사진기반, 실물확인 필요) + secondary=RECOMMENDED. Trajectory classifier(M1~M6/MX) 10/10 synthetic PASS(Case A~E + critical PRE=POST transient regression). 자체발견·수정: oscillation 부호반전이 magnitude-only로는 안 잡혀 Case C 최초 실패 → sign-reversal 카운터 추가+분류 순서 재배치로 수정. One-write architecture AST 증명(execution.py가 write 모듈조차 import 안 함), 신규 ledger 6단계(COMMAND_SENT 이후 전체 차단, P-007F-05 버그 재발 방지), 승인→명령 fast path(callable 5개만) 설계. Two-host clock 설계(orchestrator를 p620에서 실행). P-007F-03~06 evidence 전부 불변 | `PHASE A = PASS / CLOSED — READY_FOR_MARKER_SETUP=YES, READY_AND_ARMED=NO, MOTION_AUTHORIZED=NO, 다음은 P-007F-07A(marker static qualification)`
| 38 | 2026-10-29 (세션 `66eb4937`) | **P-007F-07 Marker Setup Reference Capture** — 사용자 지시("Gemini 자신의 시점에서 marker 위치를 결정"). robot/servo/camera-movement 전부 0, Gemini만 RGB 5 frame 재촬영(`scripts/p007f_07_marker_setup_capture.py`, 신규, Jetson 미import로 robot_command_count=0 구조적 보장). Gemini identity 재확인(serial/firmware 일치), Baseline B 변경 없음 실측(phase-correlation shift 0.49px<1.0px), Laplacian variance 최대 프레임(`0004-color.png`) 채택. Phase A `marker_mount_instruction.md` 후보(PRIMARY_A/PRIMARY_B/SECONDARY_A)를 기존 P-007F-06 ROI 좌표 기준 근사 박스+범례로 새 프레임 위에 재overlay(`CANDIDATE_ONLY`, 최종 미확정) | `DONE — rgb_reference.png/marker_candidate_overlay.png 사용자 검수 대기, 다음은 사용자의 marker 최종 위치 확정+부착 → P-007F-07A`
| 39 | 2026-10-29 (세션 `66eb4937`) | **P-007F-07 Arm2 Marker Mount Close-up** — robot/servo/camera-movement/marker-attachment 전부 0, 재촬영 0. 이미 확보된 reference frame을 좌표 그리드 반복 판독으로 확대·분석(`scripts/p007f_07_marker_mount_closeup.py`, 신규). Arm2 rotation axis/rigid link/next-joint seam 식별, legacy PRIMARY_A/B 박스가 새 고해상도 프레임에서는 디스크를 부분적으로 벗어남을 확인. M1(disc top, rigid_with_arm2=YES 정의상, 가독성 최소선 미달 위험)/M2(어깨 브래킷, UNCERTAIN, 50mm까지 여유 충족)/M3(servo 라벨, UNCERTAIN, 부착 불가로 제외) color→depth 국소실측 기반 mm 환산. Cursor 제안: 1순위 M2/백업 M1/M3 제외 — 최종 미확정 | `CANDIDATE_ONLY — 03_marker_surface_candidates.png 검수 대기, MOTION_AUTHORIZED=NO, MARKER_LOCATION_FINAL=NO`
| 40 | 2026-10-29 (세션 `66eb4937`) | **P-007F-07 30mm ArUco Print Sheet** — robot/servo 0, 실제 motion 0. Phase A 사전등록 `DICT_4X4_50`/`PRIMARY_MARKER_ID=0`을 기존 코드(`scripts/p007f_07_marker_detector.py`)에서 재확인해 그대로 재사용(새로 정하지 않음, assert로 실행 시점 재검증 — 사용자가 제안한 ID=0과 우연히 일치). `scripts/p007f_07_marker_print_sheet.py`(신규)가 A4 PDF/PNG를 matplotlib mm-좌표 단일 axes(figsize=A4 inch 정확)로 생성, marker 30.0mm/quiet zone/30mm reference ruler/PRINT AT 100% 라벨 포함. 2단계 실측 검증: PDF `/MediaBox` 직접 파싱(A4 일치 확인)+dpi=300 PNG 래스터 실측(29.972×29.972mm, ±0.5mm 이내 PASS). `evidence/P-007F-07/marker_print/`(`aruco_4x4_50_id0_30mm_A4.pdf`·`aruco_4x4_50_id0_30mm.png`·`print_verification.md`·`geometry_report.json`) | `GEOMETRY_VERIFICATION_PASSED=true — 사용자 실측(자) 확인 대기, MOTION_AUTHORIZED=NO`
| 41 | 2026-10-29 (세션 `66eb4937`) | **P-007F-07A M2 Kinematic Membership Verification** — robot/servo/motion/marker-attachment 전부 0. servo ID2↔`Arm2_Joint` 매핑을 이미 검증된 `scripts/p007e_r2r4_fk.py`(P-007E-R2-R4, 실제 §36 안전판정 근거) 소스에서 재확인(SHA-pin, 새로 추정 안 함). URDF parent/child 그래프 독립 BFS로 Arm2_Joint downstream 집합을 재유도, FK 모듈의 `RIGID_MOVING_LINKS`가 부분집합임을 교차검증(자체발견·수정: 완전 집합 일치 비교의 FALSE NEGATIVE를 부분집합 불변식으로 수정). M2(Arm2_Link 말단/Arm3_Link 시작부 중 하나, extrinsic 미보유로 확정 불가하나 둘 다 downstream)는 STL mesh bbox 길이 프로파일(Arm1_Link 43.5mm/Arm2·Arm3_Link 각 107.0mm 실측)과 close-up 사진 구조 대조로 보강. corollary: M1(disc)=Arm1_Link=FIXED_LINKS 소속 → ID2 단독 회전으로 움직이지 않음(기존 사전등록 결과 소급 변경 없음) | `M2_DOWNSTREAM_OF_ID2 = YES — 검수자 승인 대기, MOTION_AUTHORIZED=NO`
| 42 | 2026-10-29 (세션 `66eb4937`) | **P-007F-07B Phase B1 — M2 50mm Marker Print** — robot/servo 0, 실제 motion 0. `DICT_4X4_50`/`PRIMARY_MARKER_ID=0`을 30mm 산출물과 동일 재확인(새로 정하지 않음). `scripts/p007f_07b_marker_print_50mm.py`(신규)가 30mm 버전의 `build_figure()`를 SHA-pin import로 그대로 호출, 전역만 override. 자체발견·수정: reference bar 길이=`BLACK_SQUARE_MM`인데 눈금/라벨은 `(0,10,20,30)`/`"30mm"`로 하드코딩된 버그 발견 → 원본 모듈 근본 수정(눈금/라벨을 `BLACK_SQUARE_MM`에서 유도), 기존 30mm 산출물도 재생성(내용 동일, PDF만 갱신). geometry 2단계 검증: PDF `/MediaBox` A4 일치 + PNG 실측 49.953×49.953mm(PDF 벡터가 authoritative) | `PRINT_READY=YES — 사용자 50mm 실측+M2 부착 확인 대기(Phase B1 STOP), READY_FOR_MOTION_PREREGISTRATION=NO, MOTION_AUTHORIZED=NO`

| 43 | 2026-10-29 | **P-007F-07B Phase B2 이후** — 사용자 50mm 실측·M2 평면 부착 확인 후 Gemini 정지 119 frame. Baseline B unchanged(shift 0.470px). ID 0 두 군집(상단 41.76px×12, 책상 21.85px×47). 검출률 0.496. robot/servo 0 | `MARKER_STATIC_QUALIFICATION = FAIL — 다음 Gate 미개방, MOTION_AUTHORIZED=NO` |

| 44 | 2026-10-30 | **P-007F-07B-R1** — 중복 ID0 제거 후 사전확인 24 frame. 책상 카드는 없고 키보드 위 시트가 ID0 1회. Baseline B unchanged. 본촬영 미시작, robot/servo/torque 0 | `ABORT / DUPLICATE_ID0_STILL_PRESENT — qualification 미발급, MOTION_AUTHORIZED=NO` |

| 45 | 2026-10-30 | **P-007F-07B-R1 attempt 2** — 중복 시트 제거 후 재실행. 물리 ID0=1, detector 17/258, side mean 41.65 px, Baseline B unchanged. robot/servo/torque 0 | `MARKER_STATIC_QUALIFICATION_R1 = FAIL — 다음 Gate 미개방, MOTION_AUTHORIZED=NO` |
| 46 | 2026-10-30 | **P-007F-07C** — 기존 attempt-02 검출 17프레임과 Gemini RGB intrinsics로 M2 면 자세를 추정. 영상 각 30.8°는 쐐기각으로 쓰지 않음. 절반 보정 26.7°(ESTIMATED), 모서리 이동 16 mm, 판 56×56 mm. 새 촬영·로봇 동작 0 | `DESIGN_READY / MOUNT_REDIRECTION_RECOMMENDED — 설치 미승인, MOTION_AUTHORIZED=NO` |
| 47 | 2026-10-30 | **P-007F-08 Phase A** — M2 50 mm 마커를 검출 대상이 아니라 LK 추적 패턴으로 사용. 경사판 미설치. 정지 sham 228 frame. 공식은 촬영 전 고정. robot/servo 0 | `READY_AND_ARMED=YES — PHASE_B_EXECUTED=NO, MOTION_AUTHORIZED=NO` |
| 48 | 2026-10-30 | **P-007F-08A** — 기존 evidence만으로 ID2 소회전의 영상 변위가 1.0 px를 넘는지 검토. extrinsic 부재. 예측 미계산. robot/servo 0 | `OBSERVABILITY_UNRESOLVED — READY_FOR_PHASE_B_REVIEW=NO, MOTION_AUTHORIZED=NO` |
| 49 | 2026-10-30 | **P-007 closure** — 하드웨어 motion qualification을 새 실험 없이 종료. 기존 evidence 불변. P-006D 미개방. robot/servo 0 | `P-007 = CLOSED / BLOCKED — PHYSICAL_ARM2_MOTION=UNRESOLVED, ADDITIONAL_DIAGNOSTICS=STOPPED` |
| 50 | 2026-10-30 | **P-008** — 신발 Gemini point cloud를 찾지 못해 geometry 파이프라인을 실행하지 않음. P-004 점군은 신발 입력이 아님. 컬러 신발 프레임은 point cloud가 아님. robot/servo 0 | `BLOCKED_REAL_SHOE_INPUT_REQUIRED — P-009=NOT OPENED` |
| 51 | 2026-10-30 | **P-008A** — Baseline B Gemini로 작업대 위 정지 운동화 RGB/depth/point cloud 5회 저장. 카메라와 신발은 움직이지 않음. 기하 파이프라인 없음. robot/servo 0 | `PASS — P-008=READY_TO_RESUME, NOT OPENED` |
| 52 | 2026-10-30 | **P-008-R1** — capture_01만으로 voxel→SOR→평면→cluster→PCA→concave hull을 실행. 사전 치수 창이 작업대 신발이 아닌 cluster 8을 선택. 창은 사후에 바꾸지 않음. capture_02~05 미사용. robot/servo 0 | `FAIL_SHOE_ISOLATION — PARAMETERS_FROZEN=NO, P-008-R2=NOT OPENED` |
| 53 | 2026-10-30 | **P-008-R1D** — R1 파라미터를 유지한 채 매트 중심 점의 단계별 잔존을 확인. 신발은 height gate 이전에만 있고 cluster에는 없다. id 3을 신발로 보지 않음. robot/servo 0 | `PASS_DIAGNOSIS — SHOE_REMOVED_BY_PREPROCESSING, P-008-R1_RETRY=NOT OPENED` |
| 54 | 2026-10-30 | **P-008-R1E** — majority orientation 대신 PCL polygonal prism을 capture_01에 적용. 신발 점은 전부 남고, 1 cm cluster는 매트와 이어짐. height와 tolerance는 유지. robot/servo 0 | `FAIL_CLUSTERING — PARAMETERS_FROZEN=NO, P-008-R1_RETRY=NOT OPENED` |
| 55 | 2026-10-30 | **P-008-R1F** — R1E prism 이후 10 mm 연결을 진단. 매트는 3.6 mm에서 붙고 진단 원판 전체는 26.7 mm에야 한 component가 된다. tolerance는 바꾸지 않음. robot/servo 0 | `PASS_DIAGNOSIS — DISTANCE_SEPARATION=NOT_FEASIBLE, P-008-R1_RETRY=NOT OPENED` |
| 56 | 2026-10-30 | **P-008-R1G** — capture_01에서 depth 점과 점군의 pinhole 대응을 측정. 중앙값은 1.08 mm이나 고정한 꼬리 기준은 실패. color↔depth 행렬은 캡처에 없어 RGB로 투영하지 않음. robot/servo 0 | `FAIL_DEPTH_CLOUD_CORRESPONDENCE — COLOR_DEPTH_EXTRINSIC=NOT_VERIFIED, P-008-R1H=NOT OPENED` |
| 57 | 2026-10-30 | **P-008B** — SW depth→color와 colored cloud, optical extrinsic을 이번 세션에서 읽고 정지 운동화 5장을 저장. depth↔cloud p95는 1e-7 m 아래. 물리 등록 정확도는 주장하지 않음. robot/servo 0 | `PASS_DATASET_V2 — HARDWARE_SYNC=NOT_VERIFIED, P-008-R1H=NOT OPENED` |
| 58 | 2026-10-30 | **P-008-R1H** — capture_01에서 갑피·매트 진단 영역의 색과 지지면 높이를 비교. 색은 분리되지 않고 높이는 내부에서 갈린다. 접촉 매트 쪽을 고정 기준으로 모으지 못해 방법을 고르지 않고 멈춘다. robot/servo 0 | `PASS_METHOD_SELECTION — NOT_ROBUSTLY_SEPARABLE, STOP_RULE=YES, P-008-R1I=NOT OPENED` |
| 59 | 2026-10-30 | **P-008-R1GEO** — capture_01 작업대에서 지지 평면을 제거하고 가장 큰 유효 군집을 신발 후보로 저장. 색 특징은 쓰지 않음. R1H는 그대로 둠. robot/servo 0 | `PASS_GEOMETRY_SEGMENTATION — PARAMETERS_FROZEN=YES, CAPTURE_02_05_USED=NO` |
| 60 | 2026-10-30 | **P-008-R1GEO-V** — 동결된 기하 분할을 capture_02–05에 그대로 적용. 네 장 모두 신발 후보가 나왔다. 파라미터는 바꾸지 않음. robot/servo 0 | `PASS_HOLDOUT_VALIDATION — HOLDOUT_SUCCESS=4/4, SEGMENTATION_TRACK=CLOSED` |
| 61 | 2026-10-30 | **P-008-R2** — capture_01 신발 후보의 장축 측면을 따라 연속 3D path를 만들었다. 중심선은 칼라 개구라 쓰지 않음. normal과 로봇 명령은 없음. robot/servo 0 | `PASS_PATH_EXTRACTION — PARAMETERS_FROZEN=YES, CAPTURE_02_05_USED=NO, 6D_WAYPOINT=NOT_OPENED` |
| 62 | 2026-10-30 | **P-008-R3** — R2의 147개 점에 카메라 방향 법선과 접선으로 tool frame을 붙였다. 점 위치는 바꾸지 않음. 앞코 쪽 큰 자세 변화는 기하 불연속으로 기록. robot/servo 0 | `PASS_6D_WAYPOINT_GENERATION — PARAMETERS_FROZEN=YES, ROBOT_EXECUTABLE=NO, CAMERA_TO_BASE=NOT_OPENED` |

> 🔴 **시계 주의**: Jetson 은 **+0800**, 이 표와 git 은 **KST(+0900)** 다. 증거 파일의 `commanded_at 17:49:06+0800` = KST 18:49:06.

## 운영 규칙 (이 프로젝트 고유)

- **게이트 판정 문자열은 재서술하지 않는다.** `acceptance.json` 의 값을 그대로 옮긴다
- **운영자 관찰은 증거로 승격하지 않는다.** 관찰로만 기록한다 (P-007C 가 세운 기준)
- **선행 게이트 증거는 불변이다.** 새 게이트는 자기 디렉터리에만 쓴다. `git diff --exit-code HEAD -- evidence/<선행>` 으로 착수·종료 시 확인
- **플랜 검토는 advisor 가 한다** (codex 자문 대체 — 사용자 지시 2026-10-27)
- **가설 라벨은 분기까지 적는다** — `H-C6.learning_mode`(ELIMINATED)와 `H-C6.other_controller_state`(OPEN)는 다르다. 상위 라벨만 적어 범위를 넓게 읽히게 하지 않는다 (검수 세션 지적 2026-10-27)
- 외부 수집 자료는 **gitignore 된 게이트 디렉터리 + 커밋되는 매니페스트**로 둔다. 스크래치패드 금지 (`docs/261027-1358-*-이슈보고서.md`)
