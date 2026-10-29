# P-007F-07 Phase A — §14 Background Reference Design

`scripts/p007f_07_trajectory_classifier.py.extract_trajectory(primary_series,
background_series, ...)`가 이미 이 요구를 구현한다:

- `background_series`는 P-007F-03의 `STATIC_REFERENCE_ROI`(또는 marker 부착 후 그
  영역에서 검출되는 보조 ArUco marker — 둘 중 marker 검출기를 그대로 재사용할 수 있으면
  두 번째 방식을 선호, 영역 전체 phase-correlation보다 marker corner 기반이 더 정밀)를
  매 frame 추적한 결과다.
- 매 frame마다 `background_dx`/`background_dy`를 primary marker의 raw dx/dy에서 빼서
  `comp_dx`/`comp_dy`(camera motion 보정 완료) 를 만든다 — §14 "최종 primary quantity는
  가능하면 robot marker motion - background camera motion" 요구를 그대로 구현.
- `classify_motion(..., background_instability=...)` 플래그로, background 자체가
  noise를 넘는 움직임을 보이면(카메라 흔들림 신호) `MX EXPERIMENT_INVALID`로 처리한다
  (§26 negative test `BACKGROUND_INSTABILITY_ABORT`로 검증, PASS).

## Phase A에서 검증한 것

`BACKGROUND_INSTABILITY_ABORT` negative test(synthetic, `scripts/p007f_07_execution.py`가
아니라 `p007f_07_synthetic_tests.py`에서 커버 — 실제로는 trajectory_classifier 자체
로직이라 `p007f_07_execution.py`의 negative test 목록과 별개로 분리되어 있음, 아래
"파일 간 테스트 분배" 참고) — background가 불안정하면 classification이 즉시 MX로
멈춘다는 것을 확인.

## 실제 marker 기반 background tracking (Phase B 설계, Phase A 미실행)

marker 부착 이후에는 `STATIC_REFERENCE_ROI` 영역에 **background 전용 보조 marker**를
하나 더 붙이는 안도 검토 가능하나, 이번 Phase A에서는 기존 P-007F-03
phase-correlation(§6 global registration과 동일 방법, `cv2.phaseCorrelate`)을 1차
background tracker로 유지한다 — marker 3개(primary+secondary+background)는 과설계로
판단, 필요성이 확인되면 이후 gate에서 추가.
