# P-007F-02B-R2 — Static Baseline Stability

방법: 로봇 무변동 상태에서 연속 12프레임 중 인접 11쌍의 grayscale mean
absolute pixel diff(numpy) — 전체 프레임 + robot ROI(595:805,90:345) 각각.

```text
whole_frame_diffs = [2.553, 2.555, 2.552, 2.561, 2.554, 2.544, 2.557, 2.553, 2.559, 2.550, 2.558]
robot_roi_diffs    = [3.576, 3.581, 3.501, 3.545, 3.600, 3.550, 3.625, 3.625, 3.559, 3.561, 3.576]
```

전체 프레임 평균 2.554(범위 2.544~2.561), robot ROI 평균 3.572(범위
3.501~3.625) — 둘 다 **좁은 범위에서 일관**되며 outlier 없음. R1(전체 평균
2.233, robot ROI 평균 3.497)과 같은 자릿수·같은 안정성 수준이다.

## 판정

```text
STATIC_BASELINE_STABLE
```
