# P-007F-07B Phase B1 — 50mm ArUco Print Sheet — Print Verification

## Dictionary / ID (새로 정하지 않음, 재확인만)

`scripts/p007f_07_marker_detector.py`(Phase A에서 이미 작성·사전등록)의
기존 상수를 그대로 가져왔다 — P-007F-07 30mm 출력본과 **동일한 값**이다.

```text
DEFAULT_DICT        = cv2.aruco.DICT_4X4_50
PRIMARY_MARKER_ID    = 0
```

`scripts/p007f_07b_marker_print_50mm.py`가 실행 시점에 다음을 전부 재검증한다:
`assert det.DEFAULT_DICT == det.cv2.aruco.DICT_4X4_50`, `assert marker_id == 0`
(= 30mm 버전과 ID가 달라지지 않았음을 명시적으로 확인).

## 재사용 방법

30mm 버전(`scripts/p007f_07_marker_print_sheet.py`)의 `build_figure()` 함수를
**SHA-pin import 후 그대로 호출**하고, 모듈 전역 `BLACK_SQUARE_MM`/`MARKER_PX`/
`OUT_DIR`만 이 gate에서 override했다. 원본 파일은 호출 중 수정되지 않는다
(`print30_module_sha256_before == print30_sha256_after`로 매 실행마다 실측
확인, `geometry_report.json` 참조).

## 🔴 자체발견·수정 — reference bar 버그

이번 50mm 재사용 과정에서 30mm 버전의 **잠재 버그**를 발견했다: 기준 bar 자체는
`BLACK_SQUARE_MM` 폭으로 그려지지만, 눈금(tick)과 라벨 텍스트가 `(0,10,20,30)` /
`"30 mm REFERENCE BAR"`로 **하드코딩**되어 있었다. `BLACK_SQUARE_MM=50`으로
재사용하면 bar가 마지막 눈금(30) 너머까지 라벨 없이 뻗어나가는 모순된 결과가
나왔을 것이다. `scripts/p007f_07_marker_print_sheet.py`를 수정해 눈금/라벨
모두 `BLACK_SQUARE_MM`에서 유도하도록 고쳤다(근본 수정, 땜질 아님) — 이
수정으로 **기존 30mm 산출물도 재생성**해 일관성을 맞췄다(geometry 수치는
동일, 30mm 버전은 원래도 우연히 올바른 값이었으므로 PDF/PNG 바이트만 갱신,
`aruco_4x4_50_id0_30mm_A4.pdf`/`.png`). footer의 `print_verification.md`
경로 문자열도 같은 이유로 `OUT_DIR` 전역에서 동적으로 유도하도록 함께 고쳤다.

## 물리 크기 보장 설계 (30mm 버전과 동일 방법)

matplotlib Figure를 A4 inch 크기로 정확히 생성(`figsize=(210/25.4, 297/25.4)`),
`[0,1]×[0,1]` 전체를 덮는 단일 axes에 `xlim=[0,210], ylim=[0,297],
aspect='equal'`을 고정해 모든 요소를 동일한 mm 좌표계에 직접 그린다. PDF
백엔드는 이 axes 배치를 analytic(벡터) 좌표로 기록하므로 dpi에 의존하지 않는다.

marker bitmap은 `generate_marker_image()`(Phase A, 수정 없이 SHA-pin
재사용)로 600×600px(모듈당 100px, 4×4 code + 1-module border = 6×6 모듈,
30mm 버전과 동일 density 12px/mm 유지) 생성 후 `imshow(...,
interpolation="nearest")`로 삽입.

## 검증 결과 (2단계, 실측)

### 1. PDF MediaBox 직접 파싱 (페이지 물리 크기, authoritative)

```text
pdf_mediabox_pt          = [[0.0, 0.0, 595.2755905512, 841.8897637795]]
pdf_mediabox_expected_pt = [0.0, 0.0, 595.28, 841.89]
pdf_mediabox_matches_a4  = true
```

이 PDF 벡터 좌표(axes-fraction 배치, dpi 무관)가 **authoritative 값**이다 —
marker black square는 이론상 정확히 50.000×50.000mm다.

### 2. PNG 실측 교차검증 (보조, authoritative 아님)

dpi=300(px_per_mm=300/25.4=11.811) PNG 래스터에서 marker 영역의 어두운
픽셀(값<128) bounding box를 실측:

```text
png_verify_dpi               = 300
png_measured_marker_width_mm  = 49.953
png_measured_marker_height_mm = 49.953
(허용오차 ±0.5mm 이내 PASS — 0.047mm 차이는 300dpi 래스터 양자화 오차일 뿐)
geometry_verification_passed  = true
```

전체 수치 원본: `geometry_report.json`.

## 결과물

```text
aruco_4x4_50_id0_50mm_A4.pdf   -- 인쇄용 (A4, 100%/Actual Size로 출력)
aruco_4x4_50_id0_50mm.png      -- 검증용 래스터(300dpi) + 화면 미리보기
geometry_report.json           -- 위 2단계 검증 원본 수치 + 원본 모듈 불변 확인
```

## 사용자 확인 절차 (이번 작업 범위 밖, Phase B1 STOP 이후 수행)

1. PDF를 프린터 대화상자에서 **Scale=100% / Actual Size / Fit to Page=OFF /
   Shrink-Oversize=OFF**로 출력
2. 출력물의 **50mm REFERENCE BAR**를 자로 측정해 정확히 50mm인지 확인
3. marker 바깥 검정 사각형 한 변도 자로 측정해 50mm 확인
4. 맞으면 M2에 평평하게 부착 가능한지 확인 — **이번 작업은 여기까지 수행하지
   않는다(Phase B1 STOP). Phase B2 이후는 별도 지시 필요**

## 제한 (실측 확인됨)

```text
ROBOT COMMANDS   = 0
SERVO WRITES     = 0
MOTION_AUTHORIZED = NO
```
