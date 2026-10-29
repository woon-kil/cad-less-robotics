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
