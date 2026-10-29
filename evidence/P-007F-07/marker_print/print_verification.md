# P-007F-07 — 30mm ArUco Print Sheet — Print Verification

## Dictionary / ID (재확인, 새로 정하지 않음)

`scripts/p007f_07_marker_detector.py`(Phase A에서 이미 작성·사전등록됨)를
직접 읽어 기존 상수를 그대로 가져왔다 — 이번 작업에서 새로 선택하지 않았다.

```text
DEFAULT_DICT        = cv2.aruco.DICT_4X4_50
PRIMARY_MARKER_ID    = 0
```

`scripts/p007f_07_marker_print_sheet.py`의 `assert det.DEFAULT_DICT == det.cv2.aruco.DICT_4X4_50`가
이 값을 실행 시점에 재검증한다. 사용자가 제안한 `ID=0`은 이 기존 사전등록
값과 **우연히 일치**한다 — 이번 작업에서 임의로 확정한 것이 아니라 기존
코드에서 재확인한 값이다.

## 물리 크기 보장 설계

`scripts/p007f_07_marker_print_sheet.py`가 matplotlib Figure를 **A4
inch 크기로 정확히 생성**(`figsize=(210/25.4, 297/25.4)`)하고, `[0,1]×[0,1]`
전체를 덮는 단일 axes 하나에 `xlim=[0,210], ylim=[0,297], aspect='equal'`을
고정해 모든 요소(marker 이미지·quiet zone·ruler·텍스트)를 **동일한 mm
좌표계**에 직접 그린다. matplotlib의 PDF 백엔드는 이 axes 배치를 analytic
(벡터) 좌표로 기록하므로 **dpi에 의존하지 않는다**.

marker bitmap 자체는 `generate_marker_image()`(Phase A, 수정 없이 SHA-pin
재사용)로 360×360px(모듈당 60px, 4×4 code + 1-module border = 6×6 모듈)
생성 후 `imshow(..., interpolation="nearest")`로 삽입 — 모듈 경계가
흐려지지 않는다.

## 검증 방법 (2단계, 실측)

### 1. PDF MediaBox 직접 파싱 (페이지 물리 크기)

저장된 PDF 바이트에서 정규식으로 `/MediaBox [...]`를 직접 추출해 A4 기대값
(210mm/297mm → 595.28pt/841.89pt, 1pt=1/72in)과 비교했다.

```text
pdf_mediabox_pt          = [[0.0, 0.0, 595.2755905512, 841.8897637795]]
pdf_mediabox_expected_pt = [0.0, 0.0, 595.28, 841.89]
pdf_mediabox_matches_a4  = true
```

### 2. PNG 실측 교차검증 (marker 바깥 검정 사각형 크기)

같은 axes 레이아웃을 dpi=300(px_per_mm=300/25.4=11.811, 정확히 알려진 값)으로
PNG 래스터화한 뒤, marker가 위치할 것으로 계산된 mm 영역 안에서 실제 어두운
픽셀(값<128)의 bounding box를 측정해 mm로 역산했다.

```text
png_verify_dpi               = 300
png_measured_marker_width_mm  = 29.972
png_measured_marker_height_mm = 29.972
(허용오차 ±0.5mm 이내 PASS — 0.028mm 차이는 300dpi 래스터 양자화 오차,
 PDF 자체는 벡터 좌표라 이론상 정확히 30.000mm)
geometry_verification_passed  = true
```

전체 수치 원본: `geometry_report.json`.

## 결과물

```text
aruco_4x4_50_id0_30mm_A4.pdf   -- 인쇄용 (A4, 100%/Actual Size로 출력)
aruco_4x4_50_id0_30mm.png      -- 검증용 래스터(300dpi) + 화면 미리보기
geometry_report.json           -- 위 2단계 검증 원본 수치
```

## 사용자 확인 절차 (이번 작업 범위 밖, 출력 후 수행)

1. PDF를 프린터 대화상자에서 **Scale=100% / Actual Size / Fit to Page=OFF /
   Shrink-Oversize=OFF**로 출력
2. 출력물의 **30mm REFERENCE BAR**를 자로 측정해 정확히 30mm인지 확인
3. marker 바깥 검정 사각형 한 변도 자로 측정해 30mm 확인
4. 맞으면 quiet zone 여백(점선 바깥 포함)까지 잘라 M1 후보면에 평평하게
   들어가는지만 확인 — 이번 작업은 여기까지 수행하지 않는다

## 제한 (실측 확인됨)

```text
ROBOT COMMANDS   = 0
SERVO WRITES     = 0
MOTION_AUTHORIZED = NO
```
