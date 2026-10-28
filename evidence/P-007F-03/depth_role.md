# P-007F-03 — Depth Role

```text
PRIMARY = RGB (ORB+RANSAC, visual_algorithm.md)
DEPTH   = CORROBORATING
```

§16 지시대로 depth를 primary motion detector로 쓰지 않는다. 대신 ROI
median depth 변화를 secondary evidence로 측정했다(`depth_sham_distribution.json`,
10 sham trial, ROBOT_ROI/ARM2_ROI/STATIC_REFERENCE_ROI 각각).

## 실측 — depth 단위는 mm로 추정(공식 depth_scale 필드 미확인 — R1/R2와 동일 가정 유지)

| ROI | n_valid_trials | mean_delta_mm | stdev | max_abs_delta_mm | mean_valid_ratio |
|---|---|---|---|---|---|
| ROBOT_ROI | 10/10 | 0.1 | 0.7 | 1.0 | 0.757 |
| ARM2_ROI | 10/10 | -0.1 | 0.3 | 1.0 | 0.776 |
| STATIC_REFERENCE_ROI | 10/10 | -0.2 | 0.748 | 1.0 | 0.801 |

세 ROI 모두 median depth 변화가 ±1mm 이내로 극히 안정적이다 — static
dataset(실제 motion 0)이므로 기대한 결과와 일치한다. 이는 depth가
corroborating 신호로서 신뢰할 만한 안정성을 가짐을 보여준다(valid_ratio
75.7~80.1% 로 완전 커버리지는 아니지만 급락·붕괴 없음).

## Depth를 primary로 쓰지 않는 이유

1. ARM2_ROI는 120x70px(약 80x47 depth px) 수준의 작은 영역이라 median
   depth 1개 스칼라만으로는 rotation 성분을 전혀 포착할 수 없다(RGB의
   Metric B에 해당하는 신호가 depth에는 구조적으로 없음).
2. valid_ratio가 75~80%로 완전하지 않아(depth dropout), 단독 판정
   근거로 쓰기엔 결측 영향이 더 크다.
3. RGB(ORB)가 이미 §11이 요구하는 두 계열 metric + 품질진단을 제공하므로
   depth를 억지로 PASS 조건에 넣을 필요가 없다(§16 명시 지시 그대로 따름).

## 향후 execution에서 depth의 역할

PRE/POST ROI median depth 변화를 RGB 판정과 나란히 기록하되, result
matrix(§19, `result_matrix.md`)의 Channel B 판정에는 사용하지 않는다.
depth가 극단적으로 불안정(예: valid_ratio 급락, median이 큰 폭으로
튐)하면 그 자체를 `VISUAL_WINDOW_INVALID`의 보조 신호로만 참고한다.
