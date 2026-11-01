# W-002F-R5 — compact ChArUco 제작 준비

R4 Candidate C PNG를 다시 그리지 않고 그대로 복사했다. SHA-256은 `c7c2d840aba38483831dde95f41b352bc6d15d4ca022017e9a4936807ddc1459`다. OpenCV 4.5.4 `DICT_5X5_100` 검출은 마커 10개, ChArUco 코너 12개다.

PNG에는 DPI 정보가 없다. 그 파일을 100%로 인쇄하면 72 dpi로 커진다. 인쇄 파일은 같은 픽셀을 A4 가로 페이지 가운데에 220×180 mm로 놓은 PDF다. 페이지에 맞춰 확대하지 않았다.

미리보기에서 보드 전체가 보이고 주변에 흰 종이가 남아 있어야 한다. 마커 몇 개만 페이지를 채우면 인쇄하지 않는다.

세 크기는 서로 다르다.

- 검은 ChArUco pattern: 200×160 mm
- PNG/PDF 그림 영역, 흰 여백 10 mm 포함: 220×180 mm
- rigid backing: 아직 미정. 220×180 mm가 backing 치수는 아니다.

출력 직후 backing에 붙이기 전에 pattern X 200 mm, pattern Y 160 mm, 코너 0–3 120 mm, 코너 0–8 80 mm를 잰다. 하나라도 nominal 대비 2.0 mm를 넘으면 backing을 만들지 않고 `BLOCKED_PHYSICAL_TARGET_METROLOGY`로 끝낸다.

아직 출력, backing, 실측, 높이, 작업판 배치는 없다. 그래서 이 Gate는 통과가 아니다. Gemini 촬영도 없다.

OpenCV board 좌표에서 고른 직교 장거리는 다음과 같다.

- 코너 0–3, board X, nominal 120.000 mm
- 코너 0–8, board Y, nominal 80.000 mm

허용은 nominal 대비 2.0 mm다. 이 한계는 여기서 바꾸지 않았다.

```text
DIGITAL_DETECTION = PASS
PHYSICAL_PATTERN_GEOMETRY = NOT_YET_MEASURED
PRINT_EXECUTED = NO
GEMINI_CAPTURE_EXECUTED = NO
GATE_STATE = NOT_CLOSED
NEXT_GATE_OPENED = NO
```
