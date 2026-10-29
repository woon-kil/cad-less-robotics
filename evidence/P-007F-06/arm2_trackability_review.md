# P-007F-06 — §21 Arm2 Trackability Review

## 실측 수치 (arm2_trackability_numeric.json)

| | PRE | POST | P-007F-03 reference |
|---|---|---|---|
| ORB keypoint count | 14 | 11 | ~14 |
| x_std (px) | 13.8 | 14.9 | — |
| y_std (px) | **2.2** | **2.3** | — |
| x_range (px, of 120 width) | 41 | 41 | — |
| y_range (px, of 70 height) | **6** | **6** | — |

## 판단

- **14개 feature가 실제 Arm2 위에 존재하는가?** — PRE에서 P-007F-03 보고치(14개)와 정확히 일치. 재현됨.
- **feature spatial distribution은 충분한가?** — **아니오.** `y_range=6px`(ROI 높이 70px의 8.6%)로
  키포인트가 거의 수평 한 줄(아마 Arm2 브래킷의 고대비 모서리)에만 몰려 있다. `x_range=41px`로
  가로로는 퍼져 있지만 세로 방향 분산이 극히 작다. `estimateAffinePartial2D`/RANSAC은 이런
  거의 공선(collinear)에 가까운 점 분포에서 **회전 추정이 특히 불안정**해진다 — rotation은
  점들이 중심에서 멀리, 여러 방향으로 퍼져 있어야 잘 제약된다.
- **background feature contamination은?** — ROI 경계(120x70px)가 Arm2 브래킷보다 넓어, 인접한
  Arm1 몸체/배경 일부가 섞였을 가능성이 있다(P-007F-03에서 이미 "Arm1 원통형 몸체 바로 위
  흰색 브래킷/피벗 구조"로 식별). 이번 gate에서 별도 마스킹은 하지 않음 — 한계로 공개.
- **occlusion 영향은?** — PRE 14개 → POST 11개로 keypoint 수가 줄었다. 같은 정적 장면에서도
  frame-to-frame으로 11~14개 사이로 변동하는 것은 `within_window_noise.json`의 PRE-PRE/
  POST-POST 쌍들에서도 관찰되는 정상 범위의 변동으로 보이며, 이번 POST 감소만으로 occlusion을
  단정하지 않는다.
- **texture 부족 문제는?** — y축 분산 2.2px는 **texture가 부족하다기보다 "선형적인 edge
  texture만 있다"**는 신호에 가깝다 — keypoint 개수 자체는 P-007F-03 threshold(15개)에 근접해
  나쁘지 않지만, 기하학적 분포가 나쁘다.

## 결론

```text
MARKER_RECOMMENDED_FOR_FUTURE
```

이유: keypoint 개수는 유지되지만 공간 분포(특히 y축)가 rotation 추정을 불안정하게 만들 수 있는
구조이고, 이는 `within_window_noise.json`에서 ORB의 PRE-PRE/POST-POST noise(mean±3σ ≈ 2.3px
translation / 1.4° rotation)가 다른 4개 metric(optical flow/abs-diff/edge/phase-correlation)의
noise 대비 상대적으로 크다는 관찰과 일치한다. 이번 gate에서 marker는 부착하지 않았다(§21 금지).
