# P-007F-04 — §14/§15 Candidate Comparison

현재 controller-reported ID2 = 154 (model_assumption_only). 탐색 범위 ±1/±2/±3°,
양방향.

| Candidate | target | Model Δ (TCP trans, mm) | Arm2 joint margin (lower/upper, rad) | TCP Δ (rot, deg) | Max robot displacement (D_conservative, mm) | Collision | Visual detectability (9개 가정 중 초과) | Decision |
|---|---:|---:|---|---:|---:|---|---|---|
| lower −1° | 153 | 2.736 | 2.6704 / 0.4712 | 1.000 | 4.963 | MODEL_LEVEL_PASS | 2/9 | 제외(detectability 불충분) |
| lower −2° | 152 | 5.471 | 2.6529 / 0.4887 | 2.000 | 7.926 | MODEL_LEVEL_PASS | 5/9 | 제외(경계선) |
| **lower −3°** | **151** | **8.206** | **2.6355 / 0.5061** | **3.000** | **10.888** | **MODEL_LEVEL_PASS** | **8/9** | **✅ 선택** |
| upper +1° | 155 | 2.736 | 2.7053 / 0.4363 | 1.000 | 4.963 | MODEL_LEVEL_PASS | 2/9 | 제외(detectability 불충분) |
| upper +2° | 156 | 5.471 | 2.7227 / 0.4189 | 2.000 | 7.926 | MODEL_LEVEL_PASS | 5/9 | 제외(경계선) |
| upper +3° | 157 | 8.206 | 2.7402 / 0.4014 | 3.000 | 10.888 | MODEL_LEVEL_PASS | 8/9 | 제외(§14 참조 — 아래) |

(원본 수치: `candidate_set.json`, `joint_limit_analysis.json`, `fk_sweep.json`,
`displacement_envelope.json`, `collision_analysis.md`, `visual_detectability.md`)

## §14 — Safety 와 detectability 의 균형

원칙(작업의뢰서 §14 그대로): "가장 큰 motion"이 아니라 "안전 envelope를
최소화하면서 Gemini threshold 를 충분히 넘길 가능성이 가장 높은 motion". 또한
"threshold 바로 위가 아니라 noise margin 을 가진 후보를 선호".

- |delta|=1°: 안전 envelope 는 가장 작지만(4.963mm), detectability 가 9개 가정
  중 2개만 넘는다 — threshold 바로 근처/미달이 대부분이라 noise margin 이
  없다. **탈락**
- |delta|=2°: 경계선(5/9) — 보수적 가정의 과반에서 미달. **탈락**
- |delta|=3°: envelope 는 가장 크지만(10.888mm, 그래도 R4 의 operator envelope
  선례 15mm 이내), 9개 가정 중 8개에서 threshold 를 넘고 margin 비율이
  1.04~4.15배로 noise margin 을 확보한다. **"가장 큰 motion 이라서"가 아니라
  "안전 PASS 를 유지하면서 Gemini 가 안정적으로 검출할 가능성이 가장 높은
  유일한 후보라서"** 선택한다 — 1°/2° 는 "작아서 더 안전"해 보이지만, 사실상
  Gemini witness 가 그 motion 을 검출하지 못할 위험이 더 크다는 점에서 이
  gate 의 궁극 목적(Baseline B 를 독립 witness 로 쓰는 것)에 부합하지 않는다

## 방향 선택 — lower(−3°) vs upper(+3°)

두 후보는 변위 크기(TCP/DaBai translation·rotation)·self-collision clearance·
visual detectability 추정이 **모두 대칭**이다(§9~§13 표 참조, 동일 |delta|).
유일한 차이는 Arm2 joint-limit margin 이다:

```text
현재(154°): margin_to_lower=2.6529rad(152.0°) margin_to_upper=0.4887rad(28.0°)
```

현재 상태는 upper limit 에 lower limit 보다 훨씬 가깝다(28° vs 152° 여유).

- **+3°(157°)**: margin_to_upper 가 0.4014rad(23.0°)로 더 줄어든다 — 이미
  가까운 쪽 한계로 더 다가간다
- **−3°(151°)**: margin_to_upper 가 0.5061rad(29.0°)로 오히려 늘어난다 —
  더 먼 쪽(lower)으로 이동하면서 가까운 쪽(upper) 여유를 넓힌다

두 후보 모두 `JOINT_LIMIT_PASS`이지만, **더 가까운 한계로부터 멀어지는 방향을
선호**하는 것이 표준적인 안전 공학 관례(한쪽으로 치우친 여유를 소모하지
않는다)다. 이 비대칭이 유일한 차별화 근거이므로 **lower(−3°, target=151)**를
선택한다.

## 선택

```text
SELECTED_CANDIDATE = lower, delta=-3deg, target=151
```
