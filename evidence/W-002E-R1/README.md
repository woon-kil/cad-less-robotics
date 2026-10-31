# W-002E-R1 — Registration Residual Root-Cause Diagnosis

상태: `PASS_DIAGNOSIS / CLOSED`

원인은 `INCONCLUSIVE`다. W-002E의 `FAIL_REPROJECTION`은 바꾸지 않았다. medoid 변환은 `FAILED_GATE_DIAGNOSTIC_TRANSFORM`이고, 승인된 등록 변환은 없다.

분석은 W-002E 대표 프레임 이미지만 다시 검출한 것이다. 재계산한 재투영 평균은 저장된 1.843768 px와 같다. 마커, 카메라, 로봇, 작업판은 움직이지 않았다.

## 재투영

잔차 정의는 `projected - observed`다. 네 마커의 관측 변 길이는 60 mm 투영보다 짧다. 비율은 ID0 0.975, ID1 0.960, ID2 0.946, ID3 0.966이다. 패턴 플래그는 `COMMON_SCALE` 하나다. 마커마다 짧아진 양은 같지 않다. 중심 잔차의 방향도 서로 같지 않다. 주점 거리와 오차의 상관은 -0.232라 주변으로 커지는 패턴은 아니다.

마커를 하나씩 빼고 푼 맞춤 평균은 1.63–1.91 px이고, 뺀 마커는 3.62–5.90 px다. 네 장 모두 같은 형태라 특정 ID만 제거하지 않았다. 이 계산은 새 등록 후보가 아니다.

목표 좌표로 바꾸면 평균이 1.844 px에서 2.194 px로 커진다. 차이는 0.350 px라 민감도 라벨은 `MATERIAL`이고, 목표 좌표는 채택하지 않는다.

정사각형 변을 58–62 mm로만 바꿔 보면 평균은 58 mm에서 1.385 px, 60 mm에서 1.844 px, 62 mm에서 2.901 px다. 작은 쪽이 잔차를 줄이지만 58 mm도 평균 1.0 px와 p95 2.0 px를 넘는다. 크기를 바꾸지 않았고 `PRINT_SIZE_MEASURED = NO`다.

## 평면

진단용 변환으로 옮긴 맨 나무 점 28829개의 평균 z는 -2.340 mm다. `|z|` 평균 2.343 mm는 W-002E와 같다. 기울기 RMS는 0.452 mm라 고정한 규칙으로 주성분은 `OFFSET`이다. `dz/dx`는 -8.45 mm/m, `dz/dy`는 -5.93 mm/m다. 이 부호를 종이 두께나 depth 편향으로 확정하지 않는다.

RGB 재투영과 depth 평면은 공통 원인으로 묶지 않는다.

## 파일

- `result.json`
- `center_sensitivity.json`
- `marker_size_sensitivity.json`
- `leave_one_marker_out.json`
- `reprojection/per_corner.json`
- `reprojection/per_marker.json`
- `reprojection/residual_vectors.png`
- `plane_validation/structure.json`
- `plane_validation/workboard_xyz.npy`
- `plane_validation/z_trends.png`
