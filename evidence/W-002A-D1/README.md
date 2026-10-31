# W-002A-D1

작업판 `(150,0) mm`의 `DICT_4X4_50` ID 0이 RGB에는 보이지만 검출되지 않는 원인을 현재 프레임에서만 확인했다. 마커, 카메라, 로봇, 작업판은 움직이지 않았다. 검출 파라미터는 기본값 그대로다.

## 입력

검출기가 받은 프레임은 `/camera/color/image_raw`의 `rgb8`이다. `raw_color.png`는 그 배열을 무손실 PNG로 저장한 것이다. 미리보기 JPEG는 쓰지 않았다.

인쇄 원본은 `Downloads/aruco_workboard_4x4_50_ids_0-3_60mm_A4.pdf` 1면을 200 dpi로 래스터한 `source_print_page.png`다. 같은 OpenCV 4.5.4 기본 검출기가 이 면에서 ID 0, 1, 2, 3을 읽었다. ID 0이 포함된다.

## 결과

실시간과 오프라인 검사는 같은 프레임에 같은 `Dictionary_get(DICT_4X4_50)`와 `DetectorParameters_create()`를 한 번 호출한 것이다. 둘 다 ID를 반환하지 않았다. 사전 불일치나 라이브 파이프라인만의 차이가 아니다.

rejected candidate는 2개다. 둘 다 영상 위쪽(y 90–188)이고, 아래쪽 테스트 마커와 겹치지 않는다. 테스트 마커는 candidate로도 잡히지 않았다.

마커 열에서 마지막 행 y=719에 어두운 픽셀 58개와 흰 픽셀 34개가 같이 있다. 검은 패턴과 흰 종이가 프레임 끝에 걸려 아래쪽 quiet zone이 영상 안에 닫히지 않는다. 위쪽 흰 여백은 보인다. 완전한 바깥 사각형의 픽셀 크기는 재지 않았다.

마커 창의 Laplacian 분산은 907이고, 같은 프레임의 나무면은 48이다. 초점이 나무면보다 무뎌서 실패했다고 볼 측정은 아니다. 검은 영역 중앙 밝기 75, 흰 영역 185다.

`marker_diagnostic_crop.png`는 이 확인용이다. ProcessSurfaceScope나 P-009에 넣지 않는다.

## 원인

인쇄 PDF는 ID 0으로 읽힌다. 카메라 원본만 실패하고, 그 원본에서 마커 아래 흰 테두리가 프레임 밖으로 나가 있다. 그래서 primary는 `QUIET_ZONE_PROBLEM`이다. 두 번째 원인은 두지 않았다.
