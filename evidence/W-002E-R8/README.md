# W-002E-R8 — 복구 후 baseline 재등록

상태: `FAIL_REGISTRATION_RECOVERY / CLOSED`

W-002E와 같이 코너 refinement는 NONE, solver는 SQPnP, 검은 사각형 모델은 60 mm다. ID2 중심만 실측 `(450, 336)` mm다. SUBPIX와 마커별 크기는 쓰지 않았다. K와 D는 W-002E CameraInfo와 같다. 나무 ROI `u=480..700`, `v=430..560`은 나무결이고 마커가 들어 있지 않다.

## 결과

재투영 평균 1.582 px, p95 2.226 px, 최대 2.334 px다. 평균이 1.0 px를 넘어서 첫 실패다. p95도 2.0 px를 넘는다. 최대는 3.0 px 안이다.

W-002E보다 평균은 0.262 px, p95는 0.526 px, 최대는 0.422 px 줄었다.

작업판 `|z|` 평균은 2.449 mm로 2 mm를 넘는다. p95는 3.964 mm로 4 mm 안이다. 법선 각은 0.095°다. signed z 평균은 -2.446 mm다. W-002E의 -2.340 mm보다 절대값이 0.106 mm 커졌다.

10프레임 반복은 이동 0.503 mm, 회전 0.120°다. 회전 행렬과 역행렬은 맞다.

실행 전에 고정한 분류로 재투영은 줄고 평면 간격의 절대값은 늘어서 `FIDUCIAL_RECOVERY_PACKAGE_EFFECT = MIXED`다. ID2 비평면이 유일한 원인이라는 뜻은 아니다. 설치 상태와 중심 X가 함께 바뀌었다.

`VALIDATED_REGISTRATION_TRANSFORM = NONE`

## 파일

- `result.json`
- `camera_info.json`
- `reprojection.json`
- `per_marker_residual.json`
- `plane_metrics.json`
- `repeatability.json`
- `transform.json`
- `comparison_w002e.json`
- `raw_color.png`
- `detected_ids.png`
- `reprojection_overlay.png`
- `bare_wood_roi.png`
