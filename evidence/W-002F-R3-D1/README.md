# W-002F-R3-D1 — ChArUco 검출 원인 분리

상태: `CLOSED`

등록과 solvePnP는 하지 않았다. 검출 파라미터와 dictionary는 바꾸지 않았다. 카메라와 보드는 움직이지 않았다. 다시 출력하지 않았다.

## Test 1

입력은 디스크에 있는 `charuco_7x5_30mm_15mm_opencv454.png`다. SHA-256은 `dd37b453f71ccb86be32d506fad85bb1673c4240b3004889da6aa056f35d29d6`이다. 지시문에 적힌 `a67dc728…`은 pHYs를 넣기 전 파일이다. 픽셀은 그때 그대로 두었다.

OpenCV 4.5.4, `DICT_5X5_100`, R3와 같은 기본 파라미터다.

수락 마커 17개, ID 0~16. 거부 후보 62개. ChArUco 코너 24개. 생성 PNG는 검출된다.

## Test 3

같은 detector로 `evidence/W-002F-R3/frames/color/000.png`를 보면 수락 마커는 0개, 거부 후보는 14개다.

생성물은 검출되고 촬영 이미지는 검출되지 않는다. 분기는 `PRINT_OR_IMAGING_PATH`다. 인쇄 배율 실패, 카메라 품질 실패, dictionary 불일치, 생성 오류는 확정하지 않았다.

생성에 쓴 dictionary와 검출 dictionary의 `bytesList`는 같다. 다른 dictionary를 채택하지 않았다.

## Test 4

비트 판독기는 디지털 마커 ID 0에서 Hamming 거리 0으로 맞았다. 이 판독기는 OpenCV 내부 `identify`가 아니다.

촬영 이미지의 사각형 후보는 보드의 마커 위에 있다. 후보를 못 찾은 것이 아니다. 가까운 ID까지의 거리는 대부분 7이고, `maxCorrectionBits`는 3이다. 수락된 ID는 없다. 얇은 후보 하나는 마커 사각형이 아니다.

원인은 아직 `UNRESOLVED`다. 갈래만 정해졌다. 생성 PNG는 정상이고, 인쇄본을 찍은 RGB에서 비트 해독이 되지 않는다.
