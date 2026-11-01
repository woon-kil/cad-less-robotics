# W-002F-R1 근거

조회일: 2026-10-31

이 파일은 검토자가 제시한 공식 문서와, 같은 날 p620 `bytesize-robotics-camera`에서 확인한 OpenCV API를 구분한다. 공식 문서를 이 Gate에서 다시 조사하지 않았다.

## OpenCV ChArUco

- 문서: OpenCV 4.13.0 ChArUco detection tutorial
- URL: https://docs.opencv.org/4.13.0/df/d4a/tutorial_charuco_detection.html
- 검토자 요지: 고정밀 pose에는 ArUco marker corner보다 ChArUco chessboard corner를 쓴다. 공식 경로는 `CharucoBoard`, `CharucoDetector`, `matchImagePoints()`, `solvePnP()`다. OpenCV 4.6 이후 board 좌표계와 pattern 호환이 바뀌므로 legacy pattern을 명시해야 한다.
- 분류: `OFFICIAL_IMPLEMENTATION_VERIFIED` (검토자가 제시한 문서)

- 문서: `cv::aruco::CharucoDetector` class reference
- URL: https://docs.opencv.org/doc/doxygen/html/d9/df5/classcv_1_1aruco_1_1CharucoDetector.html
- 검토자 요지: 4.6 이후 pattern 비호환이 이 계열 문서에 적혀 있다. 버전을 확인하기 전에 보드를 출력하지 않는다.
- 분류: `OFFICIAL_IMPLEMENTATION_VERIFIED` (검토자가 제시한 문서)

## 로컬 런타임

- 위치: p620, 컨테이너 `bytesize-robotics-camera`, `cv2.__version__`
- 값: OpenCV 4.5.4
- 없는 기호: `CharucoDetector`, `CharucoBoard.matchImagePoints`, `setLegacyPattern`
- 있는 기호: `CharucoBoard_create`, `interpolateCornersCharuco`, `chessboardCorners` 24개
- 분류: `LOCAL_EVIDENCE`
- 적용: 보드는 이 런타임의 유일한 pattern으로 만든다. OpenCV 4.6 이후 이름으로는 `CHARUCO_LEGACY_PATTERN = YES`다. 플래그 API는 없다. `CharucoDetector`가 없으므로 실행 경로는 `interpolateCornersCharuco`의 chessboard corner와 `solvePnP`다. ArUco marker corner로 pose를 대체하지 않는다. 이 매핑은 `PROJECT_ADAPTATION`이다.

## Orbbec

- 문서: Orbbec Gemini 330 Series Depth Quality Metrics
- URL: https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation/Depth%20Quality%20Metrics
- 검토자 요지: accuracy(절대 bias)와 spatial precision(평면 내부 noise)은 별개다. known-distance planar reference와 ground truth 거리를 비교한다.
- 분류: `VENDOR_VERIFIED` (검토자가 제시한 문서)
- 적용: TEST C는 실측 총 높이와 depth separation을 비교한다. 약 -2.4 mm signed offset과 약 0.9 mm local plane RMS를 한 숫자로 합치지 않는다.

- 문서: Orbbec Gemini 335L product specification
- URL: https://www.orbbec.com/products/stereo-vision-camera/gemini-335l/
- 검토자 요지: spatial precision은 2 m에서 ≤0.8% 조건이다. 이 값을 0.6~0.8 m 거리의 absolute accuracy mm 합격 기준으로 환산하지 않는다.
- 분류: `VENDOR_VERIFIED` (검토자가 제시한 문서)
- 적용: `SPATIAL_PRECISION != ABSOLUTE_DEPTH_ACCURACY`. TEST A 2.0 mm / 4.0 mm / 1.0 deg와 TEST C 2.0 mm는 이 datasheet에서 환산한 값이 아니다. 검토자가 이 Gate에 동결한 independent-reference 기준이다.

## 이 Gate에서 만들지 않은 것

출력 파일 hash, backing 재질, 실측 치수, 네 점 높이, 촬영 K/D는 없다. 보드를 생성하지 않았다.
