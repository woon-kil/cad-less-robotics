# P-007F-04 — §13 Gemini Visual Detectability

## 정확한 projection 의 불가능성

camera extrinsic(카메라-로봇 베이스 간 정밀 변환)이 캘리브레이션되어 있지 않다
(`P-007F-02B-R2/placement_decision.json.not_to_be_confused_with` 가 이미 명시:
"calibration·absolute pose 측정... 완료를 의미하지 않는다"). 따라서 FK가 예측한
3D 변위(§9/§10)를 image-plane px 변위로 **엄밀하게 투영**할 수 없다.

## 선택한 접근 — 보수적 근사 추정(§13 이 명시적으로 허용하는 두 옵션 중 하나)

`VISUAL_DETECTABILITY_NOT_QUANTIFIABLE`로 남기는 대신, 알려진 camera intrinsic
(focal length, 이 gate 신규 캡처에서 측정) + ROI depth(이 gate 신규 depth 캡처에서
측정) + pinhole 근사를 사용한 **근사 추정**을 산출한다 — 단일 숫자가 아니라
가정 조합별 범위(range)로 제시하고, 모든 가정을 명시적으로 공개한다.

```text
px_estimate = in_plane_discount × (f_px / depth_mm) × displacement_mm
```

- `f_px = 610.5074` — 이 gate 신규 캡처(`baseline_check_frames/0002.json`)
  color camera_info `k[0]`(실측, 가정 아님)
- `depth_mm` — 이 gate 신규 depth 캡처에서 실측한 3개 가정: `arm2_roi 중위값
  678mm`(Arm2 에 가장 가까운 추정), `robot_roi 중위값 746mm`, `전체 장면 p95
  1177mm`(가장 먼 쪽 보수적 가정 — depth 가 클수록 px/mm 민감도가 낮아진다)
- `displacement_mm` — §9 TCP(Gripping_point_Link) translation(그리퍼가 가장
  원위/가장 trackable, `arm2_witness_analysis.md` §8.5 참조)
- `in_plane_discount` — **임의로 고정한 비엄밀 근사 계수**다. Arm2 의 pitch
  회전평면이 image plane 에 얼마나 가까운지 정밀 측정하지 않았으므로(R2
  `arm2_witness_analysis.md` §8.2 의 정성적 판단만 있음), 1.0(완전히 image
  plane 내 — 낙관적)/0.5(절반만 image plane 성분 — 거친 보수적
  가정)/0.25(매우 보수적) 세 값을 모두 제시한다

🔴 이 추정은 **엄밀한 기하 투영이 아니다** — 단일 "충분히 보일 것이다" 주장을
뒷받침하는 근거로 쓰지 않는다. 대신 아래처럼 **9가지 가정 조합 중 몇 개가
threshold 를 넘는지**로 candidate 간 상대적 margin 을 비교하는 용도로만 쓴다.

## Threshold

```text
T_translation = 1.7826 px   (P-007F-03 sham-based threshold, 재사용 — R5 의
                              DaBai threshold 재사용 금지 원칙을 지킨 P-007F-03
                              산출값)
```

## 결과 — delta 별 9가지 가정 조합 중 threshold 초과 개수

| |delta| | TCP translation (mm) | 초과 조합 수 (9개 중) | 범위 (px) |
|---:|---:|---:|---|
| 1° | 2.736 | 2/9 | 0.355 ~ 2.464 |
| 2° | 5.471 | 5/9 | 0.709 ~ 4.926 |
| 3° | 8.206 | 8/9 | 1.064 ~ 7.389 |

(전체 27행 raw 데이터: `evidence/P-007F-04/visual_detectability_raw.json`)

- **1°**: 가장 낙관적인 가정(discount=1.0, depth=678mm)에서만 겨우 초과
  (1.38배) — 9개 조합 중 7개에서 threshold 미달. **신뢰도 낮음**
- **2°**: 중간 가정들에서 넘지만, 보수적 가정(discount≤0.25, 또는 depth=1177mm
  +discount≤0.5)에서는 미달 — **경계선(marginal)**
- **3°**: 가장 보수적인 단일 조합(depth=1177mm, discount=0.25, 추정 1.064px)
  을 제외한 **8/9 조합에서 threshold 초과**, margin 비율 1.04~4.15배 — 세
  delta 중 유일하게 거의 모든 가정에서 안정적으로 threshold 를 넘긴다

## 판정

```text
VISUAL_DETECTABILITY_APPROXIMATE_ESTIMATE — NOT RIGOROUSLY QUANTIFIABLE
(no camera extrinsic calibration; pinhole order-of-magnitude approximation only)
```

threshold 를 넘는다는 **근거 없이** "충분히 보일 것"이라고 PASS 시키지 않는다
— 대신 이 근사 추정을 §14/§15 candidate 비교에 투입해, **|delta|=3° 만이
대부분의 합리적 가정 조합에서 안정적으로 Gemini witness threshold 를 넘긴다**
는 비교우위 판단의 근거로 사용한다(단일 수치 PASS 주장이 아니라 상대적 선호
근거).
