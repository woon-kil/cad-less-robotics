# AGENTS.md

## 언어

대화, 주석, 문서는 한글로 작성한다. 토픽·타입·API 이름은 영어 그대로 둔다.

## Git 자율화

정본: `.cursor/rules/git-autonomy.mdc`. 글로벌 `~/.cursor/rules/git-autonomy.mdc`보다 이 파일이 우선한다.

- 원격은 GitHub **public** (`private: false`). `git push origin main`은 작업 단위가 끝나면 확인 없이 한다.
- 트리거: 「자율 커밋」, 「자율 푸시」, 「자율 커밋 푸시」, 「커밋」, 「푸시」, 「진행」
- ROS 2 Humble이 있는 호스트에서 C++·패키지·런치를 바꾸면 `colcon test --packages-select vision_path_generator robot_controller` 통과 후 커밋한다. Humble이 없는 호스트에서는 colcon이 없다는 이유만으로 커밋을 막지 않는다.
- `.env`, 시크릿, `--force`, 미측정 하드웨어 PASS, 가짜 성능 숫자는 커밋하지 않는다.
- `README.md`, `AGENTS.md`, `.cursor/rules/`, `.cursorrules`, `evidence/`는 커밋 대상이다.
- `docs/`는 **커밋 대상이 아니다** (2026-10-27 사용자 지시). `.gitignore`에 들어 있다. 플랜·완료보고서·이슈보고서는 로컬 디스크에만 둔다. 게이트 증거는 `evidence/`가 담당한다.

## Gate 절차

Gate sequence, 현재 Gate 상태, `NEXT_ALLOWED_GATE`의 정본은 `evidence/PROJECT-ROADMAP.md`다. 이 파일은 그 상태를 복사해 소유하지 않는다.

Gate 작업을 시작하기 전에 다음 순서로 읽는다.

1. `AGENTS.md`
2. `evidence/PROJECT-ROADMAP.md`
3. 해당 Gate의 contract
4. dependency evidence

새 Gate는 `NEXT_ALLOWED_GATE`가 요청한 Gate와 같을 때만 시작한다. 다르면 `BLOCKED_BY_ROADMAP`으로 멈추고, 그 Gate를 열지 않는다.

구현 결과가 PASS여도 `CLOSED`나 `reviewer_closed = true`로 바꾸지 않는다. repository evidence에 closure가 있을 때만 닫힌 것으로 읽는다.

`status.json`의 `NOT_OPENED`, `FAIL`, `BLOCKED`는 그 파일을 쓰던 당시의 snapshot이다. 현재 상태에 맞춰 소급 수정하지 않는다.

자산 역할은 나뉜다.

```text
PUBLIC_FOOTWEAR_CAD_PRIMARY = NONE
REFERENCE_GEOMETRY = assets/cad/complex_surface_coupon.step
ROLE = CONTROLLED_TEST_COUPON
GROUND_TRUTH = NONE
```

공개 화형 CAD가 없다는 뜻과, 프로젝트가 만든 Coupon이 reference geometry라는 뜻을 한 상태로 섞지 않는다. Coupon은 측정 표면의 ground truth가 아니다.

## Evidence vs Presentation (2026-11-02)

```text
Evidence   = immutable Gate/artifact (예: P-013 / P-022 / P-025). reopen 없이 설명력을 올리려면 Presentation을 쓴다.
Presentation = evidence를 소비하는 downstream layer (예: presentation/concept_processpath/). RobotTrajectory 성공을 연출하지 않는다.
```

포트폴리오 시각 검수는 `DATA_VALIDATION`과 `COMMUNICATION_VALIDATION`을 분리한다. frozen 데이터가 맞아도 화면 전달(크기·대비·text-safe-area)이 FAIL일 수 있다.

결함 대응: 원본에서 Observation → Impact → Acceptance를 고정한 뒤, 실패한 Criteria만 최소 repair scope로 연다. Hook/커밋 skip은 discard가 아니다.

## Evidence-First Engineering

기술적 문제의 해결 방법을 정할 때 즉흥적인 실험법을 추가하지 않는다. 밖에서 검증된 방법을 먼저 찾고, 이 프로젝트 환경에 맞게 고친 부분만 따로 적는다.

그럴듯하다는 이유만으로 새 진단법이나 실험법을 바로 구현하지 않는다. 순서는 다음과 같다.

```text
Research established methods first, record the supporting references,
compare alternatives, select a method, and only then implement the
project-specific adaptation.
```

외부 검증이나 로컬 측정이 없는 아이디어는 `EXPERIMENTAL_IDEA`다. `PROJECT_ADAPTATION`이나 `EXPERIMENTAL_IDEA`를 검증된 표준 방법처럼 쓰지 않는다.

### 1. Problem Definition

검증할 질문을 한 문장으로 고정한 뒤에 방법을 고른다. 방법을 정해 두고 문제를 끼워 맞추지 않는다.

예: `Does the commanded servo motion produce independently observable physical robot motion?`

### 2. External Method Research

새 측정법, 진단법, calibration, tracking, workaround를 설계하기 전에 이미 검증된 방법을 조사한다.

우선순위:

1. `LITERATURE_VERIFIED` — peer-reviewed paper, 표준, 학술기관 자료
2. `OFFICIAL_IMPLEMENTATION_VERIFIED` — 공식 프로젝트 또는 공식 GitHub
3. `VENDOR_VERIFIED` — 제조사 공식 documentation, SDK, reference implementation
4. `EXTERNAL_CORROBORATED` — 출처가 분명한 robotics 구현 사례

공개 웹에서 확인되는 문헌·공식 문서·공식 GitHub 조사의 1차 담당은 reviewer다. Cursor에게 같은 공개 자료를 다시 조사시키지 않는다. Cursor는 그 근거가 저장소에 없을 때, 로컬 구현에 필요한 범위만 확인한다.

### 3. Evidence Classification

근거의 종류를 이 이름 중 하나로 적는다.

- `LITERATURE_VERIFIED`
- `OFFICIAL_IMPLEMENTATION_VERIFIED`
- `VENDOR_VERIFIED`
- `EXTERNAL_CORROBORATED`
- `LOCAL_EVIDENCE`
- `PROJECT_ADAPTATION`
- `EXPERIMENTAL_IDEA`
- `UNKNOWN`

### 4. Method Selection Before Implementation

후보가 여러 개면 바로 구현하지 않고 아래를 비교한 뒤 하나를 고른다.

- 무엇을 실제로 측정하는가
- 고정한 질문에 직접 답하는가
- 필요한 hardware
- 현재 장비로 구현 가능한가
- measurement independence
- expected accuracy / resolution
- failure modes
- implementation complexity
- safety implications

### 5. Project Mapping

고른 외부 방법을 현재 환경에 옮긴다.

- DOFBOT Pro
- Jetson Orin Nano
- Yahboom controller / `Arm_Lib`
- Orbbec Gemini 335L
- fixed Gemini Baseline B
- ROS 2 / MoveIt
- P620 development workstation

그대로 쓸 수 없는 부분만 `PROJECT_ADAPTATION`으로 적는다.

### 6. No Unjustified Reinvention

검증된 방법이 있고 이 환경에 적용할 수 있으면 새 측정법을 만들지 않는다. 새 방법이 필요하면 다음을 남긴다.

- 기존 방법을 쓸 수 없는 이유
- 바뀌는 부분
- 새로 생기는 uncertainty
- 그 방법을 어떻게 검증할 것인지

### 7. Preregister Before Physical Experiment

로봇 motion이나 hardware state를 바꾸는 실험은 실행 전에 다음을 고정한다.

- hypothesis
- measurement channels
- acceptance criteria
- invalidation criteria
- safety limits
- command count
- retry policy
- evidence to preserve

결과를 본 뒤에 threshold나 PASS 조건을 바꾸지 않는다.

### 8. Independent Physical Evidence

controller command / readback과 실제 물리 상태를 구분한다.

```text
controller-reported position != independently verified physical joint angle
```

물리적 움직임이 있었다는 주장이 필요하면, 가능한 한 controller와 독립된 measurement channel을 쓴다.

### 9. Evidence Chain

가능한 경우 다음 사슬을 유지한다.

```text
Requirement → Method → Implementation → Test → Measured Result → Evidence Artifact → Verdict
```

구현만 있고 시험이 없으면 `IMPLEMENTED / NOT VERIFIED`다.

### 10. Stop Rule

검증 비용이 프로젝트 목표보다 커지면 실험을 더 쌓지 않는다. 다음을 다시 본다.

- 이 문제가 portfolio의 핵심 목표를 막는가
- 검증된 더 단순한 방법이 있는가
- 한계를 `UNRESOLVED`로 적고 다음 단계로 갈 수 있는가

필요하면 `UNRESOLVED` 또는 `BLOCKED`로 끝낸다.

## 역할

### ChatGPT / reviewer

- 공개 문헌, 공식 문서, 공식·공개 GitHub 조사
- 방법 비교, 구조, 실험 설계
- acceptance criteria, evidence 검토, PASS/FAIL 판단

### Cursor / local implementation agent

- 로컬 저장소와 소스 분석
- 구현, 테스트
- Jetson / runtime 점검
- hardware-facing 실행
- evidence artifact 작성

검토자가 이미 가져온 공개 근거를 Cursor가 웹에서 다시 찾지 않는다. 로컬에 없는 구현 세부만 확인한다.

Cursor는 `codex exec`와 Codex 재검토를 호출하지 않는다. 플랜 파일 변경 트리거가 Codex 검토를 요구해도 건너뛴다. 작업 결과의 PASS/FAIL은 ChatGPT가 판단한다.

## Learned User Preferences

- 한 Gate가 끝나면 다음 Gate를 자동으로 열지 않는다. 「진행」, 「커밋」, 「푸시」는 현재 산출물의 커밋·푸시이며, 그 말만으로 다음 단계를 시작하지 않는다.
- 열린 Gate가 motion을 명시하지 않으면 joint, servo, torque, trajectory, MoveIt `execute`, home, init을 보내지 않는다. 읽기 전용 관찰은 동작 허가가 아니다.
- 장치가 없을 때 `/joint_states`를 만들거나 URDF `offset="0"`을 검증된 물리 영점으로 쓰지 않는다. URDF에서 축이 읽혀도 `base_link`의 물리 원점·z=0·+X가 실물과 연결되기 전에는 physical datum으로 쓰지 않으며, 식별되지 않으면 `UNRESOLVED`로 둔다. 대화에만 있고 저장소에 없는 입력은 `INPUT_EVIDENCE_MISSING`으로 두고 홈 디렉터리를 더 찾지 않는다.
- eye-to-hand에서 `solvePnP`의 `^C T_T`로 `^B T_T`나 `^B T_C`를 만들지 않는다. `T_base_target`이 비어 있으면 임의 좌표, 작업대 평면=`base_link` z=0, 육안 yaw=0, 미검증 TCP probing으로 채우지 않고 checkerboard 촬영을 시작하지 않는다. 실패한 ArUco qualification을 calibration target으로 되살리지 않는다.
- 이미 기록된 evidence와 verdict를 나중에 고치지 않는다. 닫힌 Gate의 수치는 후속 정밀 측정으로 덮어쓰지 않고 별도 evidence로 남긴다. PASS 기준을 낮춰 게이트를 통과시키지 않는다. 결과를 본 뒤 W-002B registration threshold를 완화하지 않는다. 설치 목표 좌표를 실측 좌표로 복사하지 않는다. 도구 표시 분해능이나 사진만으로 mm 정확도를 주장하지 않는다. 실제 센서 입력이 없으면 synthetic 데이터로 PASS를 만들지 않고 새 capture를 자동으로 시작하지 않는다. 제작 방식·재료·측정 기구가 정해지지 않은 치수 허용오차는 임의 mm로 만들지 않고 `TBD`로 두며, 근거가 없으면 `BLOCKED_INSUFFICIENT_MANUFACTURING_CONTEXT`로 남긴다. 프린터 정확도와 측정 불확도를 한 값으로 합치지 않는다. Gemini 335L을 physical coupon의 치수 qualification 측정기로 쓰지 않는다. 제작 결과를 본 뒤 허용오차를 넓혀 PASS시키지 않는다. calibration 표본의 관측 최댓값을 acceptance threshold로 쓰지 않는다. timeout 민감도에서 실패가 사라진 값을 kinematics timeout repair로 동결하지 않는다.
- 파라미터 선정, 시각 확인, 통계는 그 Gate가 지정한 reference capture만 사용한다. holdout capture는 열람·튜닝·비교에 쓰지 않는다.
- 별도 RGB 이미지나 필드 이름만으로 point cloud color와 depth/cloud 대응을 가정하지 않는다. extrinsic·alignment provenance가 검증되기 전에는 RGB ROI를 cloud에 적용하지 않고, diagnostic ROI를 production segmentation으로 쓰지 않는다. 다른 세션의 calibration 값을 복사하지 않는다. 등록 오차를 mm로 주장하지 않는다. 멀티캡처는 identity·stream·depth↔cloud·color correspondence preflight가 통과하기 전에 시작하지 않으며, 빠진 provenance를 사후에 만들지 않는다.
- Codex 검토는 하지 않는다. Cursor가 만든 결과물을 ChatGPT가 판단한다 (2026-10-30).
- P-007/DOFBOT 실물 구동과 hand-eye calibration은 핵심 경로가 아니다. 핵심 증거는 Geometry → 6D ProcessPath → collision-aware RobotTrajectory다. 실물 관절 부호를 다시 볼 때는 육안 CW/CCW로 판정하지 않는다. CAD↔Scan 확장(P-014~P-020)은 robot integration의 기술 전제가 아니다. ProcessWaypoint, IK goal, trajectory point는 서로 다르고, ProcessPath, RobotPlanningInput, RobotTrajectory도 서로 다르다. Cartesian 합격은 연속 PROCESS run마다 fraction이 0.95 이상(`MIN_PROCESS_RUN_FRACTION`)일 때이며 평균으로 가리지 않는다. TRANSITION은 그 성공률에 넣지 않는다. `avoid_collisions=false`이면 collision-free를 주장하지 않고, jump detection이 꺼져 있으면 joint-jump 안전을 주장하지 않는다. 시간 제한 KDL IK 실패를 기하학적 도달 불가능성으로 단정하지 않고, arm-length heuristic으로 reach나 IK를 주장하지 않는다. 실패한 실행을 미실행으로 되돌리지 않는다. P-025는 검증된 파이프라인의 성공 구간과 P-022 실패 경계를 보여주는 시각화 층이라 datum, `eef_step`, threshold, upstream evidence를 화면 때문에 바꾸지 않는다. `RobotTrajectory` 생성, collision-free, trajectory-valid, physical execution, calibration accuracy, production readiness를 주장하지 않는다. 원본에 없는 `trajectory_written=false`를 만들지 않고, `absent`를 `false`로 바꾸거나 gate-level `NO`로 개별 run을 false로 추론하지 않는다. V-01–V-10 데이터 검증을 화면 AC나 `P025_PASS`로 대체하지 않는다. P-022의 `START_IK_FAILED`와 partial Cartesian fraction을 같은 start-pose timeout으로 설명하지 않는다. 첫 midpoint precision 실패를 pathological solution으로 단정하지 않고, precision·joint wrapping·solver를 바꿔 그 Gate를 PASS시키지 않는다. IKFast 생성 스크립트 존재, 컨테이너 동작, 템플릿이 seed 거리로 해를 정렬한다는 사실을 UR5e IKFast qualification이나 branch divergence 해결로 쓰지 않는다. 동결 UR5e와 공식 MoveIt/OpenRave 경로의 `NOT_FEASIBLE_FOR_THIS_ENVIRONMENT`를 IKFast 기술 자체의 거부로 확대하지 않고, 생성기 검증에 탈락한 중간 solver를 benchmark에 쓰지 않는다. pick_ik의 load·seed consistency나 midpoint 오차 감소를 branch continuity·mechanism improvement로 승격하지 않으며, 결과를 본 뒤 `minimal_displacement_weight`나 timeout을 올려 P-022를 통과시키지 않는다. `computeCartesianPath`가 이미 이전 state를 다음 IK seed로 쓰므로 previous-q seeding 추가를 별도 대안으로 세지 않는다. `NO_JUSTIFIABLE_ALTERNATIVE`는 frozen contract 안에서 정당화된 대안이 없다는 뜻이며, 이 ProcessPath를 RobotTrajectory로 만드는 방법이 없다는 뜻이 아니다. 그 판정 뒤에 solver 탐색을 자동으로 계속하지 않는다. IK 성공만으로 qualification을 닫지 않고, 같은 pose의 복수 해를 `q'=q`로 강제하지 않는다. P-025의 AC-09 최초 판정 FAIL과 AC-10 최초 BLOCKED는 남긴다. 이후 repair·unblock PASS는 최초 산출이 정상이었다는 뜻이 아니며, 개별 AC PASS만으로 `P025_PASS`를 확정하지 않는다. 포트폴리오 영상은 P-013의 6D ProcessPath와 P-022 경계(17 FULL, run 5·7·12 `START_IK_FAILED`, run 20 PARTIAL 20/33·fraction 0.606061, `CLOSED_FAIL` / `RobotTrajectory NOT ACHIEVED`)를 재생하는 presentation asset이다. UR5e가 741 waypoint 전체를 성공 수행하는 장면으로 만들지 않고, P-025를 그 영상 때문에 다시 열지 않는다. P-022 후속 실험은 원본 evidence를 덮어쓰지 않는 별도 산출이다. fraction만으로 실패 waypoint를 확정하지 않고, IK 실패를 fraction 0으로 쓰지 않으며, 단일 실행으로 안정적 개선을 주장하지 않는다. 시작 IK 개선, PROCESS run별 fraction ≥ 0.95, RobotTrajectory 산출, 시뮬레이션 실행 가능성, 실물 실행은 서로 대체하지 않는다. RobotTrajectory 파일만으로 실행 가능성을 판정하지 않고, 공구 축 회전이 자유롭다고 가정하지 않는다. datum·orientation·ProcessPath·solver·경로 분할 변경은 조건 변경 실험으로 분리하며, 그 결과로 P-022 `CLOSED_FAIL`을 고치지 않는다. 검사를 끄고 얻은 결과를 개선으로 세지 않는다.
- 신발·support 분리와 표면 추출에서 color, HSV, CIELAB은 diagnostic이다. primary 추출 기준으로 쓰지 않고, ML·SAM·foundation model segmentation으로 PASS를 만들지 않는다.
- R1 perception 방법 조사는 R1N에서 종료한다. segmentation, threshold, multiview, cutting-plane의 추가 변형 Gate를 열지 않는다. 멀티뷰 registration과 fusion은 현재 핵심 경로 밖에 둔다.
- P-010 PASS는 공개 CAD 자산이 CAD branch의 유효한 입력이라는 뜻이며, 측정 표면의 ground truth가 아니다. `GROUND_TRUTH`는 부여하지 않는다. 스캔 측정값을 CAD에 맞추어 왜곡하지 않는다. Registration은 correspondence를 만들 뿐 truth가 아니다. 탈락한 CAD를 PASS시키려고 repair하지 않고, FreeCAD가 LGPL이라는 사실만으로 워크벤치나 생성 자산의 라이선스를 추정하지 않는다. CAD reference branch는 기존 CAD-less 파이프라인 위에 두는 병렬 분기다. 저장소 정체성을 CAD-only로 바꾸거나 저장소 이름을 바꾸지 않는다. 동결된 P-009 ProcessPath 의미(`SURFACE_ANCHOR`, `COMMAND_POSITION`, 20 mm standoff, `tool_z=-normal`)와 P-011 STEP, P-012 geometry engine을 PASS를 만들려고 수정하지 않는다. `tool_standoff_mm`은 계약 의미이고 생성 필드 `process_offset_mm`과 이름만으로 같은 필드가 아니다. 둘 다 20이라는 이유만으로 호환을 단정하지 않는다. `1e-9 mm` 잔차는 수치 계약 일관성이며 로봇 위치 정확도가 아니다. BREP face 경계를 공정 경계로 쓰지 않고, face 개수를 맞추려고 topology를 만들지 않는다.

## Learned Workspace Facts

- P620 호스트는 Ubuntu 24.04 x86_64, RTX 3090, ROS 2 Jazzy, Isaac Sim이다. 호스트 Isaac, NVIDIA, 기존 Docker 서비스는 바꾸지 않는다. Humble과 기존 MoveIt 2 검증은 `bytesize-robotics-humble` 컨테이너(Ubuntu 22.04)에서 한다. 호스트 Jazzy에 Orbbec 패키지를 설치하지 않는다. Robot integration의 planning reference robot은 Universal Robots UR5e이며, 공식 Jazzy 바이너리 `ur_description`, `ur_moveit_config`, `ur_robot_driver`를 P620 호스트에서 쓴다. 이 런타임을 Humble 컨테이너나 Isaac URDF로 바꾸지 않는다. planning group은 `ur_manipulator`, chain은 `base_link` → `tool0`이다. `tool0`은 planning reference TCP이며 실물 공정 공구 TCP가 아니다. `default_kinematics.yaml`은 실측 calibration이 아니다. `CONFIGURED_SIMULATION_DATUM`은 배치이며 측정 transform, calibration, ground truth가 아니다. UR5e는 프로젝트의 유일한 로봇이 아니고 P-021 vendor independence를 바꾸지 않는다. 이 planning reference 모델의 관절은 6개의 bounded revolute이고 continuous joint는 없다. shortest-angle wrapping으로 MoveIt interpolation 거리를 바꾸지 않는다. MoveIt `JumpThreshold`의 absolute 값은 호출자가 주며, 0이면 그 검사를 하지 않고, 공식 구현이 threshold를 산출하지 않는다.
- macOS는 편집 환경이다. Humble `colcon`은 P620 컨테이너에서 한다. Gemini 335L과 DOFBOT/Jetson의 연결 위치는 작업 시작 때 확인하고, 이전에 붙였던 호스트를 현재 연결로 가정하지 않는다.
- 실물 checkerboard는 7×10 칸, 내부 코너 6×9, 칸 크기 20.0 mm다. OpenCV pattern size는 6×9 inner corners다.
- `/vision/waypoints` 기하 결과는 `camera_depth_optical_frame` 기준이다. P-008 Dataset v2 점군과 R3/R4 task waypoint의 source frame은 `camera_color_optical_frame`이며, 두 frame을 하나의 변환으로 섞지 않는다. `T_base_camera`와 checkerboard `^B T_T`가 검증되기 전에는 로봇 실행 포즈가 아니다. W-002E는 `FAIL_REPROJECTION / CLOSED`이고, 그 medoid transform은 실패 Gate의 진단용이며 검증된 `T_camera_workboard`가 아니다. baseline marker model은 명목 60 mm이며, 진단용 58 mm나 마커별 캘리퍼 치수를 그 model로 채택하지 않는다. ADR-010의 복구 우선순위는 PnP 추가 fitting이 아니라 독립 physical reference다. RGB PnP 평면을 depth ground truth로 쓰지 않고, 벤더 전형 사양이나 작은 평면 noise로 absolute depth accuracy를 주장하지 않는다. ID2 비평면은 그 reprojection 실패의 단독 원인으로 확정하지 않는다.
- 대표 공정은 CAD-less 신발 표면 접착제/처리제 분사용 6D ProcessPath다. ADR-008은 공정 의도(ProcessSpecification)와 측정 표면 6D ProcessPath 생성을 분리한다. CAD-less는 CAD·기준 mesh·티칭 궤적 없이 측정 기하로 경로를 만든다는 뜻이며, 공정 경계의 자동 발견을 뜻하지 않는다. ProcessSpecification에 최종 XYZ·법선·접선·쿼터니언·관절값·수동 추적 경로를 넣지 않는다. ProcessPath는 측정 표면 프레임(현재 개발 데이터는 `camera_color_optical_frame`)에 남고 로봇 base·tool·MoveIt 프레임으로 바꾸지 않으며 로봇에 독립이다. RGB는 핵심 ProcessPath 인터페이스의 필수 입력이 아니다. 전체 perimeter 복원, 자동 segmentation·공정경계·멀티뷰 fusion, 접촉식 접착·연마는 현재 핵심 주장이 아니다. 접촉 공정은 force·contact·tool calibration 없이 주장하지 않는다. outsole·기준 geometry는 핵심 요구가 아니라 선택적 미래 adapter다.
- P-008B Dataset v2(Gemini 335L, software depth-to-color, XYZRGB)는 검은 매트 시기의 historical evidence로 불변이다. `capture_01`이 그 세트의 development/reference이고 `capture_02`–`capture_05`는 holdout이다. 매트 제거 후 운동화는 support에 직접 놓이며, 이후 development acquisition은 R1I로 v2 capture를 덮어쓰지 않는다. P-008A Dataset v1도 historical diagnostic이며 수정·사후 보정하지 않는다. hardware sync는 `NOT_VERIFIED`, registration accuracy는 `NOT_CLAIMED`다.
- 닫힌 판정은 유지한다. P-008-R1H와 P-008-R1J는 `NOT_ROBUSTLY_SEPARABLE`, P-008-R1I는 `PASS_CONTACT_OBSERVABILITY`, P-008-R1K는 `FAIL_GEOMETRY_FEASIBILITY`, P-008-R1L은 `FAIL_PROCESS_SURFACE_FEASIBILITY`, P-008-R1M은 `FAIL_ADDITIONAL_VIEW_VALUE`, P-008-R1N은 `FAIL_CUTTING_PLANE_FEATURE_FEASIBILITY`다. R1 perception 조사는 닫혀 있다. evidence를 수정하거나 기준을 낮춰 재오픈하지 않는다. P-022는 `CLOSED_FAIL`/`FAIL_CARTESIAN_API` immutable baseline이다. 재개방하거나 repair하지 않고, closure 과정에서 진단 evidence를 다시 쓰지 않는다. Alternative planning feasibility review는 `CLOSED` / `NO_JUSTIFIABLE_ALTERNATIVE`다. 동결 UR5e와 공식 MoveIt/OpenRave 생성 경로의 IKFast는 `NOT_FEASIBLE_FOR_THIS_ENVIRONMENT`다. pick_ik default diagnostic은 `NO_MECHANISM_IMPROVEMENT`다. RobotTrajectory 연구 branch는 STOPPED다. KDL scalar callback은 `RECONSIDER`이며 구현 허가가 아니다. P-025는 `PASS`/`CLOSED`이며, 그 PASS는 AC-09 최초 FAIL과 AC-10 최초 BLOCKED를 지운 무결점 판정이 아니다. `NEXT_ALLOWED_GATE`는 이 문장에 두지 않고 `evidence/PROJECT-ROADMAP.md`만 본다.
- 작업판 물리 크기는 801 mm × 400 mm다. 카메라 스탠드 점유(X 0~100 mm), 로봇 팔 점유(X 약 500~801 mm), 후보 배치 영역(X 약 100~500 mm, Y 약 0~400 mm)은 시야 관측용 참고이며 ProcessSurfaceScope로 동결된 값이 아니다. PnP object point는 W-002B 목표 layout이 아니라 실측 설치 마커 중심이다. W-002D-R1 historical 중심은 ID0 (148, 71), ID1 (500, 69), ID2 (500, 336), ID3 (149, 332) mm이며 그 기록을 고치지 않는다. W-002E-R7-R1 이후 현재 registration geometry는 ID0 [148, 71, 0], ID1 [500, 69, 0], ID2 [450, 336, 0], ID3 [149, 332, 0] mm다. ID2만 같은 물리 시트를 옮겨 재설치했고, historical (500, 336)은 현재 object point가 아니다. 1 mm 줄자 눈금은 측정 분해능이며 정확도 주장이 아니다.
- 작업대 위 지지 물체 추출은 majority-noninlier signed-distance halfspace가 아니라 viewpoint로 법선을 맞춘 PCL `ExtractPolygonalPrismData`다. 검증된 prism과 height 방향은 되돌리지 않는다.
- 카메라 컨테이너의 ChArUco 런타임은 OpenCV 4.5.4다. 보드는 `DICT_5X5_100`, squares 7×5, 공칭 square 30 mm·marker 15 mm이며 `CharucoBoard_create`가 만드는 legacy pattern이다. `CharucoDetector`와 `setLegacyPattern()`은 이 런타임에 쓰지 않는다. 정본 source PNG SHA-256은 `dd37b453f71ccb86be32d506fad85bb1673c4240b3004889da6aa056f35d29d6`이고, 이전 해시 `a67dc728…`는 historical이며 덮어쓰지 않는다. 작업판 ArUco는 `DICT_4X4_50`이며 ChArUco 검출이나 registration에 쓰지 않는다. Gemini RGB의 인쇄 ChArUco 디코딩은 실패했다. 같은 인쇄물의 휴대폰 사진은 source layout의 ID 0–16을 수락해 인쇄 payload는 보존되었고, 재인쇄만을 원인으로 두지 않는다. 그 결과는 Gemini 복구나 registration PASS가 아니다. `VALIDATED_REGISTRATION_TRANSFORM`은 `NONE`이다.
- P-010의 `freecad-shoe-last-wb` 판정은 `FAIL_CAD_ASSET_QUALIFICATION`이다. 그 자산을 수리하거나 다시 자격 심사하지 않는다. P-010A/ADR-011은 CAD geometry 검증과 footwear 검증을 분리하며, 닫힌 등록 회복 결정인 ADR-010을 덮어쓰지 않는다. 이 문장은 Gate가 열려 있는지를 적지 않는다. 개폐와 다음 Gate는 `evidence/PROJECT-ROADMAP.md`만 본다.
- CAD-derived ProcessPath는 측정 표면 ProcessPath와 같은 robot-independent 6D ProcessPath로 수렴한다. `command_position`은 surface anchor에 `+N` 방향 standoff를 더한 점이고 tool Z는 `-N`이다. `traversal_tangent`는 serpentine pass마다 반전될 수 있고, `tool_tangent`는 CAD `+X`를 접평면에 투영한 tool roll 기준이라 pass 방향과 함께 뒤집지 않는다. 투영이 사라지면 다른 축으로 바꾸지 않고 `TOOL_TANGENT_DEGENERATE`로 실패한다. Coupon STEP `assets/cad/complex_surface_coupon.step`의 SHA-256은 `2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa`이며, hash가 다르면 `BLOCKED_INPUT_INTEGRITY`로 멈춘다.
