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
- 이미 기록된 evidence와 verdict를 나중에 고치지 않는다. PASS 기준을 낮춰 게이트를 통과시키지 않는다. 결과를 본 뒤 W-002B registration threshold를 완화하지 않는다. 실제 센서 입력이 없으면 synthetic 데이터로 PASS를 만들지 않고 새 capture를 자동으로 시작하지 않는다.
- 파라미터 선정, 시각 확인, 통계는 그 Gate가 지정한 reference capture만 사용한다. holdout capture는 열람·튜닝·비교에 쓰지 않는다.
- 별도 RGB 이미지나 필드 이름만으로 point cloud color와 depth/cloud 대응을 가정하지 않는다. extrinsic·alignment provenance가 검증되기 전에는 RGB ROI를 cloud에 적용하지 않고, diagnostic ROI를 production segmentation으로 쓰지 않는다. 다른 세션의 calibration 값을 복사하지 않는다. 등록 오차를 mm로 주장하지 않는다. 멀티캡처는 identity·stream·depth↔cloud·color correspondence preflight가 통과하기 전에 시작하지 않으며, 빠진 provenance를 사후에 만들지 않는다.
- Codex 검토는 하지 않는다. Cursor가 만든 결과물을 ChatGPT가 판단한다 (2026-10-30).
- P-007/DOFBOT 실물 구동과 hand-eye calibration은 핵심 경로가 아니다. 핵심 증거는 Geometry → 6D ProcessPath → collision-aware RobotTrajectory다. 실물 관절 부호를 다시 볼 때는 육안 CW/CCW로 판정하지 않는다.
- 신발·support 분리와 표면 추출에서 color, HSV, CIELAB은 diagnostic이다. primary 추출 기준으로 쓰지 않고, ML·SAM·foundation model segmentation으로 PASS를 만들지 않는다.
- R1 perception 방법 조사는 R1N에서 종료한다. segmentation, threshold, multiview, cutting-plane의 추가 변형 Gate를 열지 않는다. 멀티뷰 registration과 fusion은 현재 핵심 경로 밖에 둔다.

## Learned Workspace Facts

- P620 호스트는 Ubuntu 24.04 x86_64, RTX 3090, ROS 2 Jazzy, Isaac Sim이다. 호스트 ROS, Isaac, NVIDIA, 기존 Docker 서비스는 바꾸지 않는다. Humble과 MoveIt 2 검증은 `bytesize-robotics-humble` 컨테이너(Ubuntu 22.04)에서 한다. 호스트 Jazzy에 Orbbec 패키지를 설치하지 않는다.
- macOS는 편집 환경이다. Humble `colcon`은 P620 컨테이너에서 한다. Gemini 335L과 DOFBOT/Jetson의 연결 위치는 작업 시작 때 확인하고, 이전에 붙였던 호스트를 현재 연결로 가정하지 않는다.
- 실물 checkerboard는 7×10 칸, 내부 코너 6×9, 칸 크기 20.0 mm다. OpenCV pattern size는 6×9 inner corners다.
- `/vision/waypoints` 기하 결과는 `camera_depth_optical_frame` 기준이다. P-008 Dataset v2 점군과 R3/R4 task waypoint의 source frame은 `camera_color_optical_frame`이며, 두 frame을 하나의 변환으로 섞지 않는다. `T_base_camera`와 checkerboard `^B T_T`가 검증되기 전에는 로봇 실행 포즈가 아니다. W-002E registration은 실패했고, 그 medoid transform은 진단용이며 검증된 `T_camera_workboard`가 아니다.
- 대표 공정은 CAD-less 신발 표면 접착제/처리제 분사용 6D ProcessPath다. ADR-008은 공정 의도(ProcessSpecification)와 측정 표면 6D ProcessPath 생성을 분리한다. CAD-less는 CAD·기준 mesh·티칭 궤적 없이 측정 기하로 경로를 만든다는 뜻이며, 공정 경계의 자동 발견을 뜻하지 않는다. ProcessSpecification에 최종 XYZ·법선·접선·쿼터니언·관절값·수동 추적 경로를 넣지 않는다. ProcessPath는 측정 표면 프레임(현재 개발 데이터는 `camera_color_optical_frame`)에 남고 로봇 base·tool·MoveIt 프레임으로 바꾸지 않으며 로봇에 독립이다. RGB는 핵심 ProcessPath 인터페이스의 필수 입력이 아니다. 전체 perimeter 복원, 자동 segmentation·공정경계·멀티뷰 fusion, 접촉식 접착·연마는 현재 핵심 주장이 아니다. 접촉 공정은 force·contact·tool calibration 없이 주장하지 않는다. outsole·기준 geometry는 핵심 요구가 아니라 선택적 미래 adapter다.
- P-008B Dataset v2(Gemini 335L, software depth-to-color, XYZRGB)는 검은 매트 시기의 historical evidence로 불변이다. `capture_01`이 그 세트의 development/reference이고 `capture_02`–`capture_05`는 holdout이다. 매트 제거 후 운동화는 support에 직접 놓이며, 이후 development acquisition은 R1I로 v2 capture를 덮어쓰지 않는다. P-008A Dataset v1도 historical diagnostic이며 수정·사후 보정하지 않는다. hardware sync는 `NOT_VERIFIED`, registration accuracy는 `NOT_CLAIMED`다.
- 닫힌 판정은 유지한다. P-008-R1H와 P-008-R1J는 `NOT_ROBUSTLY_SEPARABLE`, P-008-R1I는 `PASS_CONTACT_OBSERVABILITY`, P-008-R1K는 `FAIL_GEOMETRY_FEASIBILITY`, P-008-R1L은 `FAIL_PROCESS_SURFACE_FEASIBILITY`, P-008-R1M은 `FAIL_ADDITIONAL_VIEW_VALUE`, P-008-R1N은 `FAIL_CUTTING_PLANE_FEATURE_FEASIBILITY`다. R1 perception 조사는 닫혀 있다. evidence를 수정하거나 기준을 낮춰 재오픈하지 않는다.
- 작업판 물리 크기는 801 mm × 400 mm다. 카메라 스탠드 점유(X 0~100 mm), 로봇 팔 점유(X 약 500~801 mm), 후보 배치 영역(X 약 100~500 mm, Y 약 0~400 mm)은 시야 관측용 참고이며 ProcessSurfaceScope로 동결된 값이 아니다. 이후 PnP object point는 W-002B 목표 layout이 아니라 W-002D-R1에서 실측한 설치 마커 중심을 쓴다. 1 mm 줄자 눈금은 측정 분해능이며 정확도 주장이 아니다.
- 작업대 위 지지 물체 추출은 majority-noninlier signed-distance halfspace가 아니라 viewpoint로 법선을 맞춘 PCL `ExtractPolygonalPrismData`다. 검증된 prism과 height 방향은 되돌리지 않는다.
