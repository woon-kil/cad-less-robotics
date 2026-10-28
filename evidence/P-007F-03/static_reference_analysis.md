# P-007F-03 — Static Reference Analysis

## STATIC_REFERENCE_ROI 선정

`roi_definition.json` ROI = x:900,y:150,w:300,h:300 — 모니터+키보드 배경
영역. 로봇과 물리적으로 분리되어 있고(로봇 작업반경 밖), 텍스처가
풍부해(ORB 매칭에 유리) rigid world feature로 적합함을 `roi_annotated.png`
로 직접 확인했다.

## raw(비상대) sham 분포 — ORB, 10 trial

```text
translation_px : mean=0.0655  stdev=0.0292  max=0.1098
rotation_deg   : mean=-0.0074 stdev=0.0110  max_abs=0.0250
match_count    : [380, 370, 367, 365, 371, 364, 353, 369, 372, 356]
inlier_ratio   : 0.986~0.997 (평균 0.991)
```

ARM2_ROI(match 8~13) 대비 match_count가 **30배 이상** 많고(모니터/키보드의
풍부한 텍스처), noise(translation std 0.029px, rotation std 0.011deg)도
ARM2_ROI보다 한 자릿수 작다 — STATIC_REFERENCE_ROI는 ARM2_ROI보다 훨씬
안정적인 기준점임을 확인했다. 이는 §6 핵심 원칙(global camera/scene
displacement를 먼저 추정 후 robot ROI 상대변화를 본다)이 실제로 유효한
전제임을 뒷받침한다 — 배경이 ARM2보다 훨씬 더 "고정된" 기준으로 쓸 만큼
안정적이다.

## 카메라 자체 이동(Baseline B freeze) 교차검증

R2(`evidence/P-007F-02B-R2/rgb_overview.png`, 과거 캡처)와 이번 세션
frame 0(`current_rgb_overview.png`)을 직접 비교:

```text
whole_frame_mean_abs_diff = 2.838
robot_roi_mean_abs_diff   = 4.582
global phase-correlation shift = (-0.115px, -0.035px), response=1.030
```

global shift가 서브픽셀 수준(~0.12px)으로, 두 세션 사이(수 시간~수일 경과,
드라이버 재시작 포함) 카메라가 물리적으로 이동하지 않았음을 뒷받침한다.
R2 자체의 within-session 안정성(whole_frame diff 평균 2.554, robot ROI
3.572 — `static_baseline.md`)과 이번 cross-session 수치(2.838 / 4.582)가
같은 자릿수라는 점도 일치한다. → `ABORT_BASELINE_B_CHANGED` 트리거 없음,
Baseline B는 변경되지 않았다고 판단한다(§3).

## Static-reference guard threshold

`thresholds.json.static_reference_guard_threshold`:
`S_translation_px=0.2`, `S_rotation_deg=0.05` — 위 raw sham 분포(mean+3*stdev,
engineering floor 포함)에서 유도. 향후 실행 시 STATIC_REFERENCE_ROI의
raw 변화가 이 값을 넘으면 ARM2_ROI 판정 전에 `VISUAL_WINDOW_INVALID` 로
처리한다(§13, `quality_guard.json`).
