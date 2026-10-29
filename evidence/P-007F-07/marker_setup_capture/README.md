# P-007F-07 — Marker Setup Reference Capture

목적: 현재 Frozen Gemini 335L Baseline B 시점에서 실제 DOFBOT Arm2 marker
부착 위치를 결정할 reference RGB image 확보. 사용자 지시(2026-10-29): "Gemini
335L 자체가 앞으로 실제 external witness가 될 카메라이므로, 현재 Frozen
Baseline B에서 Gemini가 직접 촬영한 RGB frame으로 marker 위치를 결정하는
것이 가장 정확하다."

## 제한 (전부 충족, 실측 확인됨)

```text
ROBOT COMMANDS    = 0   (scripts/p007f_07_marker_setup_capture.py 는
                         Jetson/Arm_Lib/서보 관련 모듈을 import하지 않음 --
                         self_check_no_jetson_import() 로 AST 기반 자기검증)
SERVO WRITES      = 0
TORQUE COMMANDS   = 0
ROBOT MOVEMENT    = 0
GEMINI MOVEMENT   = 0   (baseline_b_check 로 실측 확인: shift 0.49px < 1.0px)
```

## 실행 결과

- 카메라: Gemini 335L, serial `CP263630009D`, firmware `1.4.60` (driver 로그
  재확인, P-007F-03/04/05 와 동일 식별값)
- 해상도: 1280x720 (원본 유지)
- 촬영: 5 frame 요청, 5 frame 전부 성공 (`raw_frames/marker_setup_capture_*/`)
- 선정 기준: Laplacian variance(선명도) 최대 프레임 -- `0004-color.png`
  (544.64, 5개 중 최고) 를 `rgb_reference.png` 로 채택
- Baseline B: **UNCHANGED** -- whole-frame mean-abs-diff 13.52,
  global phase-correlation shift 0.4909px (max_allowed=1.0px) -- 세부는
  `capture_metadata.json.baseline_b_status`

## Overlay (`marker_candidate_overlay.png`)

Phase A `marker_mount_instruction.md` §PRIMARY/ALTERNATE/SECONDARY 후보를
기존 P-007F-06 ROI 좌표계(`scripts/p007f_06_forensics.py ROIS`, 동일
1280x720 Gemini 프레임) 기준 근사 박스로 표시했다.

```text
1 = PRIMARY_A   : white disc (ARM2_ROI 영역), 회전축에서 오프셋
2 = PRIMARY_B   : 흰 디스크<->상단 브래킷 사이 대각선 링크 구간
3 = SECONDARY_A : wrist housing (END_EFFECTOR_ROI 경계)
```

🔴 **박스는 근사치다(`marker_location_status = CANDIDATE_ONLY`)** -- 기존
ROI 정의를 그대로 가져온 것이지 이번 프레임에서 새로 측정한 것이 아니다.
최종 marker 위치는 사용자가 이 이미지(또는 실물)를 보고 확정한다.

## 파일

```text
rgb_reference.png              -- 선정된 reference RGB frame (원본 해상도)
marker_candidate_overlay.png   -- 위 3개 후보 박스 + 범례
capture_metadata.json          -- 카메라 identity/해상도/Baseline B/프레임별
                                   선명도 점수/박스 좌표 전체
raw_frames/marker_setup_capture_<ts>/
                                -- 5 frame 원본(color+depth+json) 전체,
                                   capture-summary.json 포함
```
