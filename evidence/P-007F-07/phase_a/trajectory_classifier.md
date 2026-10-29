# P-007F-07 Phase A — §13/§15 Trajectory Extraction & Classification Design

`scripts/p007f_07_trajectory_classifier.py`. 전체 taxonomy:

```text
M1  NO_EXTERNAL_MOTION_DETECTED
M2  RIGID_MOTION_AND_HOLD_SUPPORTED
M3  TRANSIENT_MOTION_AND_RETURN_SUPPORTED
M4  OSCILLATION_OR_VIBRATION_SUPPORTED
M5  DOWNSTREAM_LOCAL_MOTION_SUPPORTED
M6  EXTERNAL_MOTION_DETECTED_BUT_TYPE_UNRESOLVED
MX  EXPERIMENT_INVALID
```

## 분류 알고리즘 (요약)

1. background-compensated `translation(t) = hypot(comp_dx, comp_dy)` 계산(PRE 안정 구간
   평균을 baseline으로).
2. `above(t) = translation(t) > T_translation`.
3. 어디서도 `above`가 없으면 → primary=M1, 단 secondary marker가 있고 그쪽만 `above`면
   → M5(downstream-localized).
4. **방향 반전(sign reversal) 우선 확인**(아래 "자체발견" 참조) — dx/dy 중 진폭이 큰 축의
   부호가 threshold 이상 변위 구간에서 반전되면 즉시 M4(oscillation).
5. 그 외: 마지막 구간(`end_window`)에서 `above` 비율이 `hold_fraction`(기본 0.8) 이상이면
   M2, `return_fraction`(기본 0.2) 이하면 M3, 그 사이는 M6.
6. `valid_frame_ratio < min_valid_frame_ratio` 또는 `background_instability=True`이면
   가장 먼저 MX로 멈춘다(다른 모든 분류보다 우선).

## 🔴 자체발견·수정 — magnitude만으로는 부호반전 oscillation을 놓친다

`translation = hypot(dx,dy)`는 항상 0 이상이라, "+3px -> -3px" 같은 부호반전 진동은
magnitude가 중간에 threshold 아래로 내려가지 않을 수 있다(Case C 최초 합성 테스트
실패로 발견). `_count_sign_reversals()`를 추가해 dominant axis(dx 또는 dy 중 진폭이
큰 쪽)의 부호 반전을 별도로 세고, oscillation 판정을 hold/transient 분기보다 **먼저**
확인하도록 분기 순서를 바꿨다(처음엔 end_above_fraction 분기 안에서만 oscillation을
봐서 "중간값"이 그냥 M6로 새는 2차 버그도 함께 발견·수정).

## §16 threshold formula (수치는 미확정)

```text
T = max(fixed_floor, sham_mean + 5*sigma)
```

`threshold_formula(sham_mean, sham_stdev, fixed_floor)` 로 구현. 실제 `sham_mean`/`sigma`는
§27 marker static qualification 이후에만 채워진다 — Phase A의 synthetic test는 전부
placeholder `T=1.5`(실제 preregistered 값 아님, 명시적으로 disclose됨)로 로직만 검증.

## §13/§17/§18 synthetic test 결과

`synthetic_test_results.json`(10/10 PASS) — Case A/B/C/D/E 전부 통과, **§18 critical
regression(PRE==POST, 중간에 motion) 통과**(`transient_return_regression.json`) —
"CASE_B처럼 중간 excursion 후 정확히 원위치로 복귀하면 M3(TRANSIENT_AND_RETURN)로 잡아야
하고, 단순 PRE/POST 정적 비교라면 NO_CHANGE로 놓쳤을 것"이라는 §18의 핵심 요구를
구조적으로 만족한다.
