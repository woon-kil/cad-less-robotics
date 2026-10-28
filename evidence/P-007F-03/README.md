# P-007F-03 — Gemini External Motion Witness Preregistration

🔴 이번 gate는 **사전등록(preregistration)만**이다. 로봇 motion command는
0회 전송됐다(`preregistration.json.robot_command_count_this_gate`). R5도
재실행하지 않았고, Gemini/DaBai 위치도 변경하지 않았다.

## 목적

Gemini Baseline B(`evidence/P-007F-02B-R2/`, `BASELINE_B_PLACEMENT_ACCEPTED`)
를 이용해, 향후 단 한 번의 Arm2 명령 전후 실제 물리적 움직임이 있었는지를
controller readback과 독립적으로 판정할 재현 가능한 protocol을
사전등록했다.

## 수행한 것 (전부 읽기 전용/오프라인)

1. **Baseline B freeze 확인** — p620에서 Gemini 335L driver를 재시작해
   1프레임 읽고(`current_rgb_overview.png`) R2 캡처와 직접 비교. 전체
   프레임/robot ROI diff가 R2의 within-session 안정성과 같은 자릿수,
   global phase-correlation shift가 서브픽셀(~0.12px) — 카메라 미이동
   확인(`baseline_b_identity.json`, `static_reference_analysis.md`).
2. **ROI 정의** — ROBOT_ROI/ARM2_ROI는 R2 수치 그대로 재사용, DOWNSTREAM_ROI/
   STATIC_REFERENCE_ROI는 신설하고 `roi_annotated.png`로 육안 확인
   (`roi_definition.json`).
3. **Static dataset 확보** — 로봇/카메라 전혀 미이동 상태에서 70프레임
   synced RGB+depth 신규 캡처(목표 M>=60 충족, `static_dataset_manifest.json`,
   `frames/`).
4. **Visual algorithm 4종 비교** — ORB+RANSAC / Farneback optical flow /
   edge+phaseCorrelate / template matching을 10개 sham trial에 대해 실측
   비교, ORB+RANSAC을 PRIMARY로 선정(`visual_algorithm.md`,
   `algorithm_comparison_raw.json`).
5. **Marker 필요성 평가** — 부착 없이 trackability 실측, 10/10 trial에서
   안정적으로 동작함을 확인 → `MARKER_REQUIRED=false`
   (`visual_algorithm.md` 근거 섹션).
6. **Sham PRE/POST 10 trial 구성·측정** — robot-relative(ARM2_ROI −
   STATIC_REFERENCE_ROI) translation/rotation 계산
   (`sham_trials.json`, `robot_relative_sham_distribution.json`).
7. **Quality guard 고정** — window-level + static-reference guard
   (`quality_guard.json`).
8. **Threshold 산출** — sham 통계(mean+3*stdev) + engineering floor
   (`thresholds.json`).
9. **False-positive 검증** — 산출한 threshold로 10개 sham trial 전부
   `NO_VISUAL_MOTION`으로 올바르게 분류됨, 0/10
   (`false_positive_analysis.md`/`.json`).
10. **Depth 역할 고정** — CORROBORATING only(`depth_role.md`).
11. **향후 execution 순서·restart/retry 보호·result matrix·claim boundary
    사전등록**(`execution_protocol.md`, `result_matrix.md`,
    `claim_boundary.md`).

## 파일

```text
README.md
baseline_b_identity.json
current_rgb_overview.png
roi_definition.json
roi_annotated.png
static_dataset_manifest.json
frames/                         70프레임 원본(color/depth/json) 전부 보존
sham_trials.json
trackability.json
algorithm_comparison_raw.json
robot_relative_sham_distribution.json
depth_sham_distribution.json
visual_algorithm.md
quality_guard.json
thresholds.json
static_reference_analysis.md
false_positive_analysis.json / .md
depth_role.md
execution_protocol.md
result_matrix.md
claim_boundary.md
preregistration.json
```

## 최종 판정

```text
P-007F-03 = PASS / CLOSED
```

§23 PASS 기준(Baseline B unchanged, independent static reference 확보,
Arm2 trackability 충분, static dataset M>=60, sham trials>=10, visual
algorithm 고정, quality guard 고정, motion threshold 고정, false-positive
기록, result matrix 고정, execution sequence 고정, claim boundary 고정,
robot command 0) 전부 충족 — `preregistration.json` 참조.

🔴 이 PASS는 **protocol 사전등록**일 뿐이다. 실제 Arm2 motion execution,
command magnitude/target 확정, 실행 harness 구현(restart/retry ledger
포함)은 포함하지 않는다 — 전부 별도 safety review/gate 대상으로 남긴다
(§18, `execution_protocol.md`).
