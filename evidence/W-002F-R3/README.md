# W-002F-R3 — 독립 기준 RGB-D 관측

상태: `BLOCKED_CHARUCO_DETECTION / CLOSED`

이 상태는 등록 합격이 아니다. solvePnP, TEST A~D, 카메라 이동, 로봇 이동은 하지 않았다. 검출 파라미터는 완화하지 않았다.

## 스트림

Gemini 335L, 시리얼 `CP263630009D`, 펌웨어 1.4.60. 값은 실행 중인 컨테이너 로그에서 읽었다. 드라이버 패키지는 `orbbec_camera` 2.10.6이다.

색은 `/camera/color/image_raw`, depth는 `/camera/depth/image_raw`다. 열 장 모두 1280×720이고 프레임은 `camera_color_optical_frame`이다. 왜곡 모델은 `rational_polynomial`이다. K와 D는 이번 구독의 CameraInfo다.

OpenCV는 4.5.4다. `DICT_5X5_100`, `CharucoBoard_create(7, 5, 0.030, 0.015)`, `detectMarkers`, `interpolateCornersCharuco`만 사용했다. corner refinement는 `CORNER_REFINE_NONE`이다.

## 관측

`frames/color/000.png`에서 인쇄한 ChArUco 시트는 화면 안에 있다. 이미지 밖으로 잘려 있지는 않다.

열 장 모두 수락된 마커는 0개, 보간된 코너는 0개다. 첫 프레임을 기본 파라미터로 다시 보면 거부 후보가 14개다. 그 후보는 수락된 ID가 아니다. 크기 문턱을 낮추거나 카메라를 움직이지 않았다.

인쇄면 depth 비율은 계산하지 않았다. 수락된 마커가 없어 인쇄면 마스크가 비었다. bare wood ROI도 정하지 않았다.

시트 위를 가린 물체는 보이지 않는다. 시트 옆에 작은 마커 카드가 따로 있다.

`observability.json`의 `marker_corners_touch_image_border`가 true인 것은 마커가 0개일 때 빈 입력을 경계 접촉으로 처리한 기록이다. 시트가 화면 끝에 걸렸다는 측정이 아니다.

## 하지 않은 것

실패한 W-002E와 R8 변환은 쓰지 않았다. 작업 지시의 실측 높이 3.155 mm와 칸 길이는 맞추거나 고치지 않았다. 다음 Gate는 열지 않았다.
