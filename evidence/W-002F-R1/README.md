# W-002F-R1 — 독립 기준 실행 계약

상태: `PASS_EXECUTION_CONTRACT / CLOSED`

이 합격은 등록 합격이 아니다. 촬영, PnP, 등록, 보드 생성, 출력, 설치는 하지 않았다. 다음 실행 Gate는 열지 않았다.

## OpenCV

실행 컨테이너 `bytesize-robotics-camera`의 OpenCV는 4.5.4다. `CharucoDetector`, `matchImagePoints`, `setLegacyPattern`은 없다.

`CHARUCO_LEGACY_PATTERN = YES`. 이 값은 플래그를 켠 기록이 아니다. 4.5.4 `CharucoBoard_create`가 만드는 유일한 pattern이고, OpenCV 4.6 이후 문서가 그 pattern을 legacy로 부른다. 다른 버전의 기본 보드와 같다고 두지 않는다. 런타임 버전이 바뀌면 실행을 멈춘다.

공식 문서의 pose correspondence는 ChArUco chessboard corner다. 이 런타임에서는 다음으로 고정한다.

1. `detectMarkers`. corner refinement는 `CORNER_REFINE_NONE`.
2. `interpolateCornersCharuco`.
3. object point는 `chessboardCorners[charucoIds]`.
4. `solvePnP`, flag는 기본값 `SOLVEPNP_ITERATIVE`.

ArUco marker corner만으로 pose를 내지 않는다. `estimatePoseCharucoBoard`는 채택 경로가 아니다. SUBPIX는 채택하지 않는다. 이 API 대응은 `PROJECT_ADAPTATION`이다.

## 보드

`DICT_5X5_100`, 7×5, 명목 칸 30 mm, 마커 15 mm, 내부 코너 24개. 명목 pattern은 210×150 mm이고 backing 외곽과 같지 않다. 명목 치수는 physical ground truth가 아니다.

생성은 아직 하지 않았다. 생성할 때는 이 런타임에서 10 px/mm, pattern 2100×1500 px, margin 10 mm, borderBits 1, 배율 100%로 만든다. 뷰어의 맞춤 배율은 쓰지 않는다. 생성 파일의 hash와 pattern 항목을 남긴다.

## 계측

부착 후 pattern 가로, 세로, 그리고 코너 0–5(명목 150 mm)와 코너 0–18(명목 90 mm)을 잰다. 표시 분해능은 정확도가 아니다. 재지 않은 값을 명목과 같다고 적지 않는다.

실측과 명목의 절대 차이가 하나라도 2.0 mm를 넘거나 측정이 없으면 `BLOCKED_PHYSICAL_TARGET_METROLOGY`다. 이 2.0 mm는 TEST C에 이미 동결한 물리 예산이다. datasheet 환산이 아니다. 차이가 나도 object geometry를 scale-fit하지 않는다. 합격이면 object geometry는 명목 30 mm와 15 mm로 남는다.

총 높이는 작업판 접촉면에서 인쇄면까지 완성품을 직접 잰다. 네 곳의 값, 평균, 범위를 남긴다. 층별 두께 합은 우선 방법이 아니다. 범위가 2.0 mm를 넘으면 `BLOCKED_REFERENCE_PLANARITY`다. 촬영 전 보이는 휨과 틈은 없어야 한다. 높이를 만들기 위해 target을 누르고 찍지 않는다.

높이 불확도가 없거나 2.0 mm보다 작지 않으면 TEST C의 합격과 불합격을 강제하지 않고 `BLOCKED_INSUFFICIENT_PHYSICAL_REFERENCE_PRECISION`으로 끝낸다.

## 관측과 판정

RGB는 raw distorted color와 당시 K, D다. 프레임은 `camera_color_optical_frame`이다. W-002E와 R8의 K/D와 다르면 자동으로 진행하지 않는다.

검출 코너는 20개 이상이고, 6×4 격자에서 가로와 세로 양끝에 걸쳐 있어야 한다. 재투영 기준은 평균 0.5 px, p95 1.0 px, 최대 1.5 px다. W-002B 기준은 바꾸지 않는다.

TEST A는 RGB 평면과 같은 인쇄면의 depth 점이다. 부호는 board +Z이며, 결과를 본 뒤 뒤집지 않는다. 기준은 절대 평균 2.0 mm, 절대 p95 4.0 mm, 법선 1.0 deg다.

TEST B는 depth reference와 bare wood의 signed separation이다. 두 법선은 카메라 원점을 향한다. 양수는 reference가 wood보다 카메라 쪽에 있다는 뜻이다.

TEST C는 `B - 실측 총 높이`다. 절대값 2.0 mm 이내일 때의 이름은 `DEPTH_HEIGHT_CONSISTENT_WITH_PHYSICAL_REFERENCE`다. 실패한 ArUco 변환은 ground truth가 아니다.

TEST D는 A/B/C가 유효할 때만 하는 원인 분류다.

판정 이름은 `RGB_DEPTH_REFERENCE_CONSISTENT`, `RGB_REFERENCE_GEOMETRY_OR_RGB_DEPTH_ALIGNMENT_SUSPECTED`, `DEPTH_ABSOLUTE_GEOMETRY_SUSPECTED`, `MULTI_SOURCE_DISAGREEMENT`, `INDEPENDENT_REFERENCE_INCONCLUSIVE`만 쓴다. 공장 calibration, depth, RGB intrinsics가 틀렸다고 단정하지 않는다.

Gemini 335L의 2 m spatial precision을 이 거리의 absolute accuracy 기준으로 바꾸지 않는다. 0.1 mm 정밀도를 주장하지 않는다.

## 아직 없는 것

backing 재질, 실측 치수, 총 높이, 평탄 확인, 촬영 ROI 픽셀, 프레임 수는 아직 없다. ROI 규칙은 동결되어 있고, 픽셀 범위는 target을 놓은 뒤에 지표 계산 전에 고정한다. 결과를 보고 ROI를 옮기지 않는다.
