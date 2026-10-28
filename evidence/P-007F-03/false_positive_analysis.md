# P-007F-03 — False-Positive Analysis

모든 sham trial은 robot motion이 0이므로 기대 결과는 `NO_VISUAL_MOTION`.
`thresholds.json`에서 산출한 threshold(T_translation_px=1.7826,
T_rotation_deg=1.0778)를 robot-relative sham distribution
(`robot_relative_sham_distribution.json`)의 10개 trial에 그대로 적용했다.

```text
threshold_used  : T_translation_px=1.7826, T_rotation_deg=1.0778
nsham           : 10
false_positive_count : 0
false_positive_rate  : 0.0
```

| trial | robot_relative_translation_px | robot_relative_rotation_deg | flagged |
|---|---|---|---|
| 0 | 0.498 | 0.336 | NO |
| 1 | 0.789 | 0.227 | NO |
| 2 | 1.178 | 0.655 | NO |
| 3 | 0.474 | -0.089 | NO |
| 4 | 0.734 | -0.246 | NO |
| 5 | -0.021 | 0.003 | NO |
| 6 | 1.066 | 0.704 | NO |
| 7 | 0.938 | -0.069 | NO |
| 8 | 0.206 | 0.043 | NO |
| 9 | 0.063 | 0.236 | NO |

목표(`0/Nsham`, §15)를 그대로 달성했다 — threshold 또는 algorithm을
재설계할 필요가 없었다. 전체 raw 판정은 `false_positive_analysis.json`
참조.

## 주의 — 소표본 한계

N=10은 소표본이다. threshold가 mean+3*stdev(사실상 이 10개 자체의 분포)에서
유도됐으므로, false-positive 0/10은 "이 threshold가 이 10개 표본을
설명한다"는 순환적 사실이기도 하다. 독립적인 신뢰도는 (a) engineering
floor가 통계적 성분보다 작아 threshold가 순수 통계치로 결정됐다는 점,
(b) STATIC_REFERENCE_ROI raw noise가 ARM2_ROI robot-relative noise보다
훨씬 작다는 교차검증(`static_reference_analysis.md`)으로만 부분적으로
뒷받침된다. 향후 추가 static session으로 Nsham을 늘려 threshold를
재검증하는 것이 바람직하나, 이번 gate의 PASS 기준(§23: sham trials >= 10)은
충족한다.
