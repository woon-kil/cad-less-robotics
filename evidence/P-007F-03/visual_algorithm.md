# P-007F-03 — Visual Algorithm Selection

4개 후보(§7 우선순위)를 10개 sham trial(`sham_trials.json`)의 ARM2_ROI/
STATIC_REFERENCE_ROI 크롭에 대해 실측 비교했다. raw 결과:
`algorithm_comparison_raw.json`.

## 후보별 실측 결과 (robot-relative 적용 전, raw)

| 알고리즘 | ARM2_ROI translation_px (mean±std / max) | ARM2_ROI rotation_deg | Metric B 제공 | match/inlier 진단 |
|---|---|---|---|---|
| A. ORB+BFMatcher+RANSAC(estimateAffinePartial2D) | 0.658 ± 0.398 / 1.288 | 0.173 ± 0.300 (max 0.702) | **제공** | match_count 8~13, inlier_ratio 0.909~1.0 |
| B. Farneback optical flow(dense) | 0.055 ± 0.007 / 0.066 | 미제공 | 없음 | 없음(dense flow는 match 개념 없음) |
| C. edge(Canny)+phaseCorrelate | 0.055 ± 0.032 / 0.129 | 미제공 | 없음 | phase_response만 제공 |
| D. template matching(TM_CCOEFF_NORMED) | **0.000 / 0.000 (전 trial)** | 미제공 | 없음 | match_score만 제공 |

## 선정 — PRIMARY = A. ORB + BFMatcher(ratio test 0.75) + RANSAC affine

이유:

1. **§11이 요구하는 두 계열(translation + rotation) 모두를 유일하게 제공**한다.
   B/C/D는 translation만 추정하고 rotation을 추정하지 않는다(알고리즘
   한계로 문서에 명시했지 임의로 생략한 게 아니다).
2. **§11이 요구하는 품질 진단(inlier count, inlier ratio, match count)을
   네이티브로 제공**한다. B/C/D는 이런 진단이 구조적으로 없다(D의
   match_score는 global peak 하나뿐, 매칭 개수·inlier 개념 없음).
3. D(template matching)는 **정수 픽셀 그리드 탐색이라 sub-pixel 신호에
   대해 항상 0.000px을 반환**했다 — static dataset(실제 변화 없음)에서는
   우연히 "정답"처럼 보이지만, 이는 민감도가 전혀 없다는 뜻이다. 실제로
   작은 물리적 motion이 있어도 검출 못 할 위험이 매우 큼 — PRIMARY 후보에서
   제외.
4. B(optical flow)는 ARM2_ROI에서 noise가 오히려 더 작게 나왔다(0.055 vs
   STATIC_REFERENCE_ROI 0.125 — 반전). 이는 Farneback이 저텍스처 영역에서
   aperture problem으로 flow 자체를 약하게 추정하기 때문으로 보이며,
   "noise가 작다"가 "신뢰할 수 있다"를 의미하지 않는다. 또한 rotation을
   추정 못 해 §11 요구를 충족하지 못한다.

## SECONDARY / corroborating

optical flow(B), edge+phaseCorrelate(C)는 ORB(A) 결과와 교차검증용
corroborating 신호로 `algorithm_comparison_raw.json`에 보존한다. template
matching(D)은 위 이유로 사용하지 않는다(이번 gate 판정에서 제외, raw 데이터는
투명성을 위해 그대로 남긴다).

## Marker 필요 여부 — 근거

`trackability.json`: ARM2_ROI ORB keypoint count = **14**(단일 프레임, 임의
threshold 15 기준으로는 "미달"). 그러나 실제 sham trial에서는:

- **10/10 trial 모두 valid**(RANSAC affine 추정 성공)
- match_count 8~13(RANSAC 최소 요구치 4를 항상 크게 상회)
- inlier_ratio 0.909~1.0(평균 0.983)

즉 "키포인트 개수가 적다" = "알고리즘이 작동 안 한다"가 아니었다. 다만
ARM2_ROI의 noise 크기(translation mean 0.658px)가 STATIC_REFERENCE_ROI
(0.0655px)보다 **약 10배 크다** — 이는 marker 없이도 작동은 하지만 정밀도가
낮다는 뜻이고, 이 정밀도 저하를 숨기지 않고 threshold 산출에 그대로
반영했다(`thresholds.json` — robot-relative sham distribution 자체가 이
noise를 포함).

```text
MARKER_REQUIRED = false
```

판정 근거: 알고리즘이 10/10 sham trial에서 일관되게 valid+고inlier_ratio로
동작했고, 이 noise 수준은 threshold 설계에 그대로 반영되어 false-positive
0/10을 달성했다(`false_positive_analysis.json`). 다만 ARM2_ROI의 낮은
정밀도는 **disclosed limitation**으로 남긴다 — marker(AprilTag/ArUco)를
부착하면 정밀도를 개선할 수 있으나 이번 gate 범위에서 로봇에 임의로
부착하지 않는다(§8 명시).
