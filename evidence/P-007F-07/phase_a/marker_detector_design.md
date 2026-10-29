# P-007F-07 Phase A — §4/§12 Marker Detector Design

`scripts/p007f_07_marker_detector.py`(신규). 요약은 `marker_spec.md` 참조. 핵심 함수:

```text
create_detector()            cv2.aruco.ArucoDetector(DICT_4X4_50, CORNER_REFINE_SUBPIX)
generate_marker_image()      인쇄용 비트맵 생성(실제 출력은 §27에서 사람이 수행)
detect_markers()             단일 frame -> 검출된 marker 리스트
                              (corner_0..3, center_x/y, orientation_deg, marker_area_px,
                               quality.side_length_consistency_cv)
detect_markers_by_id()       expected id별 결과 dict(미검출 시 marker_detected=false)
```

6D pose는 구현하지 않았다(§4 "camera calibration/intrinsics가 충분하지 않으면 2D
image-plane witness만 사용" — 이번 로봇 작업 전체에서 камера calibration 공식 보정
절차를 거친 적이 없으므로, 억지로 6D pose를 만들지 않는다는 원칙을 그대로 따름).

## 실측 검증 (합성 이미지, 하드웨어 0)

| 조건 | 결과 |
|---|---|
| 200px marker, 정면 | 검출 성공, center 오차 <0.5px |
| 27px marker(=30mm 후보 예상 footprint) | 검출 성공 |
| 33px marker(=40mm 후보) | 검출 성공 |
| 40px marker | 검출 성공 |
| 45px marker(=50mm 권장, 677.6mm 거리) | 검출 성공 |
| 45px marker, 15° 회전 | 검출 성공, orientation 추정 -15.63°(기대 -15°, 합성 테스트 특유의 코너 양자화 오차) |

⚠️ 이 수치는 **noise/blur/조명 없는 이상적 합성 이미지** 기준이다 — 실제 카메라 영상은
motion blur·압축·조명 변화가 섞이므로 §5의 보수적 목표(>=40px preferred)를 그대로 유지한다.
