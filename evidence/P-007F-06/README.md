# P-007F-06 — Offline Forensic Analysis of P-007F-05 Visual Evidence

🔴 완전 OFFLINE. `robot_command_count = 0` (hardware_free_check.txt로 증명). 새 촬영 없음 —
`evidence/P-007F-05/run_1793274484/`에 이미 저장된 PRE/POST RGB+Depth PNG만 입력으로 썼다.

## 질문

P-007F-05에 이미 저장된 PRE/POST RGB+Depth 안에, preregistered primary detector(ORB+RANSAC,
ARM2_ROI/STATIC_REFERENCE_ROI)가 놓친 robot-localized physical visual change가 존재하는가?

## 답

```text
F2 — POST_HOC_LOCAL_CHANGE_DETECTED_BUT_MOTION_NOT_ESTABLISHED
```

P-007F-05의 사전등록 결과(`CONTROLLER_CHANGE_WITHOUT_EXTERNAL_MOTION`)는 변경하지 않는다(§25).

## 핵심 발견

1. **ORB 재현**: P-007F-05가 보고한 translation=0.2297px/rotation=0.3976°를 오차 0.05 이내로
   정확히 재현(`orb_reproduction.json`).
2. **Within-window noise control이 핵심**(§13) — 단순 PRE→POST 비교가 아니라, 같은 PRE window
   내 5프레임 10쌍 + POST window 내 5프레임 10쌍(총 20개 "noise" 샘플)의 자연 변동 분포와
   PRE→POST 신호를 비교했다. ARM2_ROI는 6개 독립 metric 중 **3개**(optical-flow median
   magnitude, raw abs-diff mean, phase-correlation magnitude)에서 noise의 mean+3σ와 max를 모두
   초과했다. STATIC_REFERENCE_ROI(배경)는 **0/6** metric에서 noise를 초과했다 — 배경은
   안정적이라는 뜻.
3. **🔴 자체발견·수정 — 처음엔 4개 ROI만 noise-control 검사했다가, 단순 registered-diff-mean
   수치에서 DABAI_ROI/ARM4_ROI/ARM3_ROI도 STATIC_REFERENCE_ROI보다 높게 나오는 것을 보고
   9개 ROI 전체로 확장**(`within_window_noise.json`, `spatial_localization.json`). 확장한 결과:
   `ARM1_ROI`와 `STATIC_REFERENCE_ROI`를 제외한 **거의 모든 ROI**(ROBOT/ARM2/ARM3/ARM4/DABAI/
   END_EFFECTOR/DOWNSTREAM)가 noise를 초과했다 — 변화가 Arm2에 국한되지 않고 로봇이 차지하는
   영상 영역 전반에 diffuse하게 퍼져 있다. 이 확장이 없었다면(최초 4-ROI만 보고 판정했다면)
   잘못된 F1(ROBOT_LOCALIZED-specific) 결론이 나올 뻔했다 — 더 넓게 검사해 정정함.
4. **Directional coherence**(optical flow top-25% magnitude 픽셀의 원형통계 resultant length
   R): ARM2_ROI R=0.669(상당히 일치), STATIC_REFERENCE_ROI R=0.203(거의 무작위) — ARM2 쪽에
   방향적으로 더 일관된 flow가 있다는 근거는 있지만, 위 3번의 "diffuse함" 때문에 F1이 아닌
   F2로 판정했다.
5. **Arm2 trackability**(§21): keypoint 14개(P-007F-03과 정확히 일치)이지만 y축 분산이 2.2px
   (ROI 높이 70px의 8.6%)로 거의 한 줄에 몰려 있어 rotation 추정이 구조적으로 불안정 —
   `MARKER_RECOMMENDED_FOR_FUTURE`.
6. **Depth**: RGB↔Depth 간 23.8mm의 실제 baseline이 있어(§17) 단순 scaled-ROI 매핑은
   `DEPTH_SPATIAL_LOCALIZATION_LIMITED`로 공개, point cloud forensic은 억지로 수행하지 않고
   `POINTCLOUD_FORENSIC_NOT_ATTEMPTED_DUE_TO_UNVERIFIED_REGISTRATION`로 명시.
7. **Global registration**(§6): STATIC_REFERENCE_ROI 기준 global shift=(0.093, -0.047)px(거의
   0) — Gemini 자체의 subpixel jitter는 무시할 수준.
8. **Synthetic sensitivity test**(§20, `NOT REAL MOTION EVIDENCE`): 0.25~5px/0.25~3° 합성
   왜곡으로 ORB/flow/edge/phase-correlation의 실제 검출 특성을 특성화.

## 파일

```text
README.md
source_manifest.json             §4 — SHA256/size/timestamp manifest (최초 freeze)
source_integrity.json            §4 — 무결성 판정
roi_definition.json              §7 — 9개 ROI 정의 + 근거(사전 고정, 사후 선택 아님)
registration_analysis.json       §6 — global registration
orb_reproduction.json            §8 — primary detector 재현
optical_flow_analysis.json       §9 — dense optical flow 전체 ROI
registered_difference_analysis.json  §10 — registered abs diff
edge_analysis.json               §11 — edge/silhouette
phase_correlation_analysis.json  §12 — local phase correlation
within_window_noise.json         §13 — PRE-PRE/POST-POST noise vs PRE-POST signal (9개 ROI, 6 metric)
spatial_localization.json        §14 — ROI별 noise 초과 패턴
coherence_classification.json    §15 — directional coherence R
depth_analysis.json              §16 — depth PRE/POST
rgb_depth_alignment.json         §17 — RGB/depth 정렬 한계 공개
pointcloud_analysis.json         §18 — point cloud (LIMITED)
synthetic_sensitivity.json       §20 — 합성 민감도 테스트(NOT REAL MOTION EVIDENCE)
arm2_trackability_numeric.json   §21 — keypoint 분포 수치
arm2_trackability_review.md      §21 — 서술 리뷰
operator_observation_review.md   §22 — 운영자 YES 검토(UNKNOWN 유지)
controller_readback_review.md    §23 — controller 값 독립 취급 재확인
forensic_classification.json     §24 — F1/F2/F3/F4 최종 판정
claim_boundary.md                §20/§25 — claim 상한 + P-007F-05 관계
hardware_free_check.txt          §28 — AST 검증(0 Arm_Lib/서보/카메라 API)
figures/                         §26 — 10개 그림(01~10)
```

## Hardware-free

```text
robot_command_count = 0
```

`scripts/p007f_06_forensics.py`는 Arm_Lib/smbus/rclpy/serial/VideoCapture 등을 전혀 import하지
않는다(`scripts/p007f_06_hardware_free_check.py`, AST 검증, PASS).

## P-007F-07

```text
NOT OPEN
```
