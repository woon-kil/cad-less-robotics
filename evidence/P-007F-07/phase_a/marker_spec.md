# P-007F-07 Phase A — §4/§5 Marker Specification

## Marker family

```text
OpenCV cv2.aruco, DICT_4X4_50
```

근거: 이 환경(OpenCV 5.0.0, `.venv-p007f03`)에서 `cv2.aruco`는 추가 dependency 없이 바로
사용 가능(실측 확인). 4x4 비트 사전은 marker 1개당 필요한 모듈 수가 6x6/7x7 계열보다 적어
작은 pixel footprint(30~50px)에서 상대적으로 더 안정적으로 디코딩된다. 50개 ID 중
`PRIMARY_MARKER_ID=0`(Arm2), `SECONDARY_MARKER_ID=1`(downstream, §7) 사용.

`scripts/p007f_07_marker_detector.py`로 구현, modern `cv2.aruco.ArucoDetector` API 사용
(`CORNER_REFINE_SUBPIX` — 작은 marker에서 subpixel corner 정확도가 특히 중요).

합성 이미지로 실측 검증(hardware 없음): 27px까지는 깨끗한 합성 이미지에서 안정적으로
검출/center/orientation 복원됨(§8 아래 표). 단 이는 noise/blur/조명이 없는 이상적
조건이므로, 실측 카메라 영상에서는 더 큰 여유가 필요하다 — 그래서 §0 목표치(>=40px
preferred, >=30px minimum)를 그대로 유지한다.

## 실측 기반 marker 물리 크기 계산

P-007F-06 `depth_analysis.json`의 **실측값**(합성/가정 아님):

```text
ARM2_ROI median depth (PRE) = 677.6 mm
color camera fx             = 610.5073852539062 px  (P-007F-05 0000.json color_info.k[0])
```

Pinhole 근사: `apparent_px ≈ fx * marker_size_mm / depth_mm`

| 후보 크기 | 예상 pixel footprint (677.6mm 기준) | §5 목표 대비 |
|---|---|---|
| 30mm | 27.0px | minimum(30px) **미달** |
| 40mm | 36.0px | minimum 충족, preferred(40px) 근접 미달 |
| 50mm | **45.0px** | preferred(40~80px) **충족** |

## 권장

```text
MARKER_SIZE = PROPOSED
primary candidate   = 50mm
fallback candidate  = 40mm (물리 장착 공간이 50mm보다 좁을 경우)
```

- ARM2_ROI의 실측 color-pixel 폭은 120px → 677.6mm 환산 시 물리 폭 ≈ 133mm. 50mm marker는
  이 폭의 약 38%를 차지 — 이미지 상으로는 여유가 있어 보이나, 실제 브래킷의 평평한 부착
  가능 면적은 사진만으로 확정할 수 없다(§6).
- 실제 부착 전까지 숫자는 **제안**일 뿐이며, §27 marker static qualification 단계에서
  사람이 실측 부착 후 재확인한다.
