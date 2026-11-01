# ADR-010 — Workboard Registration Recovery Strategy

상태: `ACCEPTED / CLOSED`  
날짜: 2026-10-31  
결정: `SELECT_B_INDEPENDENT_PHYSICAL_REFERENCE`

DESIGN / DECISION ONLY다. registration을 실행하지 않았고, 새 transform을 만들지 않았다. 마커, 카메라, 로봇, 작업판은 움직이지 않았다. W-002B threshold는 바꾸지 않았다. W-002E는 `FAIL_REPROJECTION / CLOSED`로 남긴다. P-009-R1, P-010, W-002F는 열지 않았다.

## 질문

RGB PnP plane과 RGB-D observed plane 중 어느 쪽이 physical workboard geometry에 더 가까운지, 독립적으로 판정할 수 있는가.

현재 두 관측은 `OFFSET_DOMINANT`로 어긋난다. 둘 중 어느 것이 ground truth인지 정할 독립 reference는 없다. 파라미터를 바꿔 기존 threshold를 통과시키는 문제는 아니다.

## 현재 사실

| Gate | 상태 |
| --- | --- |
| W-002E | `FAIL_REPROJECTION / CLOSED` |
| W-002E-R1 | `PASS_DIAGNOSIS / CLOSED` |
| W-002E-R2 | `PASS_PHYSICAL_GEOMETRY_METROLOGY / CLOSED` |
| W-002E-R3 | `PASS_PRECISION_PHYSICAL_METROLOGY / CLOSED` |
| W-002E-R4 | `PASS_RGB_GEOMETRY_AUDIT / CLOSED` |
| W-002E-R5 | `PASS_RGBD_PLANE_GEOMETRY_AUDIT / CLOSED` |

`VALIDATED_REGISTRATION_TRANSFORM = NONE`  
`ROOT_CAUSE = INCONCLUSIVE`

현재 evidence에서 주요 원인으로 지지되지 않는 항목이다. 절대적으로 불가능하다는 뜻은 아니다.

- `GLOBAL_58MM_PRINT_SCALE_HYPOTHESIS = NOT_SUPPORTED`
- `IMAGE_CALIBRATION_DOMAIN_MISMATCH = NOT_SUPPORTED`
- `RESOLUTION_INTRINSICS_MISMATCH = NOT_SUPPORTED`
- `CORNER_ORDERING_ERROR = NOT_SUPPORTED`
- `COORDINATE_CHAIN_ERROR = NOT_SUPPORTED`
- `UNIT_ERROR = NOT_SUPPORTED`
- `ROI_CONTAMINATION = NOT_SUPPORTED`
- `DEPTH_TEMPORAL_INSTABILITY = NOT_SUPPORTED`

남아 있는 RGB 관측은 `CORNER_LOCALIZATION_EFFECT = MATERIAL`이다. NONE 재투영 평균은 1.844 px, SUBPIX 평균은 1.038 px다. SUBPIX도 동결된 평균 1.0 px를 넘지 못한다. `SUBPIX_REGISTRATION_PASS = NO`.

남아 있는 depth 관측은 signed offset -2.340 mm, tilt RMS 0.452 mm, local plane RMS 0.889 mm, temporal z std 0.014 mm, 법선 차 0.592°다. `RGB_PNP_VS_DEPTH_PLANE = OFFSET_DOMINANT`.

## 비교

| | A 기존 ArUco/PnP 추가 최적화 | B 독립 physical reference | C requirement 재정의 |
| --- | --- | --- | --- |
| 독립 증거 | LOW | HIGH | NONE |
| 사후 fitting 위험 | HIGH | LOW | LOW |
| 추가 장비·설치 | LOW | MEDIUM | — |
| 원인 분리 | LOW | HIGH | NONE |
| 이번 회복의 주 경로 | 아니오 | 예 | 아니오 |

A는 SUBPIX 채택, 마커별 치수, 중심 보정, PnP 변형, 파라미터 최적화를 포함한다. 추가 장비는 적지만, 이미 본 결과를 사후에 맞출 위험이 크고 -2.34 mm 불일치를 독립적으로 풀지 못한다. primary recovery로 고르지 않는다.

B는 registration 알고리즘을 더 맞추기 전에 독립 physical geometry reference를 둔다. 비교 대상은 RGB PnP, depth plane, physical reference 세 개다. 정밀 평판, rigid ChArUco, 측정 가능한 평면, known-distance planar target이 예시다. 특정 제품 구매는 강제하지 않는다.

C는 P-009의 CAD-less measured-surface 6D ProcessPath 관점에서 필요한 정확도를 나중에 다시 적을 수 있는 구조적 선택이다. 기존 threshold를 낮추거나 W-002E를 PASS로 바꾸지 않는다. 필요하면 별도 ADR에서 정의한다. 현재 registration recovery는 아니다.

## ChArUco와 depth GT

ChArUco를 쓰더라도 허용 역할은 고품질 image corner reference, board geometry reference, RGB pose cross-check다. ChArUco pose를 ground truth로 두지 않고, ArUco보다 무조건 정확하다고 하지 않는다. 인쇄 치수, 설치 평탄도, 카메라 calibration의 영향을 받는다.

depth의 systematic error를 판단하려면 known physical reference가 필요하다. RGB PnP plane을 depth의 ground truth로 쓰지 않는다. Gemini 335L의 2 m 사양은 특정 조건의 typical performance이며 이번 장면의 GT가 아니다. High Accuracy depth preset은 켜지 않는다. 원인 진단 전에 측정 시스템을 바꾸지 않기 때문이다.

검토자가 제시한 자료는 [Depth Quality Metrics](https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation/Depth%20Quality%20Metrics), [Gemini 330 Datasheet V1.4](https://orbbec.com/wp-content/uploads/2024/04/Gemini-330-series-Datasheet-V1.4_20250124.pdf), [predefined presets](https://orbbec.github.io/OrbbecSDK_ROS1/en/source/camera_devices/5_advanced_guide/configuration/predefined_presets.html)다. 이 ADR은 그 페이지를 다시 열지 않았다.

## 이후 순서

이 ADR은 다음 Gate를 실행하지 않는다. 후보만 적는다.

`W-002F — Independent Physical Reference Design & Metrology Contract`

geometry, dimensions, coordinate convention, measurement method, mounting/flatness, RGB observability, depth observability, acceptance criteria를 측정 전에 동결하는 Gate다. 아직 열지 않았다.

순서는 ADR-010, 그다음 W-002F 계약 동결, 그다음 물리 reference 구축·측정, 그다음 RGB/depth/reference 3자 비교, 그 다음에만 registration recovery다.
