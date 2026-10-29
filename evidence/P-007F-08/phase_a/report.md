# P-007F-08 Phase A — Continuous External Motion Witness

판정 질문: 고정 Gemini에서 M2 구조의 물리적 움직임이 보이는가.
이번 Phase A는 그 질문에 답하지 않는다. 명령을 보내지 않고, 추적기가 정지 상태에서 안정한지만 본다.

```text
READY_AND_ARMED = YES
PHASE_B_EXECUTED = NO
ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT = 0
MOTION_AUTHORIZED = NO
```

실패한 확인: 없음

## 1. Gemini serial / firmware

identity_valid = True
serial_found = True
firmware_found = True
기대값은 기존 기록과 같은 CP263630009D / 1.4.60 이고, 이번 기동 로그에서 다시 찾았다.

## 2. Baseline B invariant

unchanged = True
shift = 0.4501 px (한도 1.0)
whole-frame mean abs diff = 10.1850

## 3. M2 ROI

x=738 y=92 width=100 height=85
역사적 000128 프레임의 같은 영역과 phase correlation: shift 0.163 px, response 0.986
visible = True

## 4. STATIC_REFERENCE ROI

x=1040 y=300 width=160 height=120
책상 위 키보드다. 로봇에 달린 부품이 아니다.
P-007F-03의 300×300 박스는 케이블을 포함해서 이번 기준으로 쓰지 않았다.

## 5. Tracking algorithm

Primary: sparse pyramidal Lucas-Kanade. 첫 프레임 Shi-Tomasi, 이후 calcOpticalFlowPyrLK, 재검출 없음.
forward-backward 최대 오차 1 px.
Secondary: cv2.phaseCorrelate. 판정에 승격하지 않는다.
template matching 은 쓰지 않았다.
마커 ID 검출은 acceptance 에 없다.

## 6. Initial feature count

M2 13
STATIC 18
최소 8

## 7. Feature survival/continuity

M2 survival check = True
STATIC survival check = True
기준: 프레임의 95% 이상에서 처음 점의 50% 이상이 살아 있을 것.

## 8. Static capture duration / frame count / FPS

duration 25.956 s
frames 228
fps 8.786
최소 20.0 s, 150 frames

## 9. M2 raw static motion statistics

n=228 mean=0.014 median=0.012 std=0.008 min=0.000 max=0.039 p95=0.029

## 10. Static-reference motion statistics

n=228 mean=0.075 median=0.076 std=0.031 min=0.000 max=0.159 p95=0.127

## 11. Relative static motion statistics

n=228 mean=0.067 median=0.065 std=0.029 min=0.000 max=0.155 p95=0.116

## 12. Phase-correlation static statistics

M2 magnitude: n=228 mean=0.040 median=0.039 std=0.021 min=0.000 max=0.105 p95=0.077
STATIC magnitude: n=228 mean=0.040 median=0.038 std=0.022 min=0.000 max=0.106 p95=0.077
relative magnitude: n=228 mean=0.053 median=0.047 std=0.027 min=0.000 max=0.130 p95=0.098
M2 response: n=228 mean=1.000 median=1.000 std=0.009 min=0.972 max=1.021 p95=1.014
STATIC response: n=228 mean=0.987 median=0.987 std=0.010 min=0.958 max=1.013 p95=1.006
valid = True

## 13. Frozen motion-threshold formula

frames [1, n) 를 반으로 나눈다. 앞쪽은 추정, 뒤쪽은 홀드아웃. frame 0 은 기준 프레임이라 변위가 0 이므로 분포에 넣지 않는다. sigma = 1.4826 * MAD. T_stat = median + 6 * sigma. T_motion = max(1.0 px, T_stat of relative LK displacement). T_static = max(0.2 px, T_stat of static LK displacement). 홀드아웃에서 relative displacement > T_motion 인 프레임이 1장이라도 있으면 false positive 로 보고 READY_AND_ARMED 를 주지 않는다. 분류의 연속 구간은 3프레임 이상만 사건으로 센다. 이 공식은 sham 을 보기 전에 고정한다.

추정 프레임 수 113. frame 0 은 빼었다. 홀드아웃을 보고 공식을 바꾸지 않았다.

## 14. Frozen numeric threshold

T_motion = {"valid": true, "n": 113, "median": 0.05158623617298235, "mad": 0.014345774258097674, "sigma": 0.02126904491505561, "T_stat": 0.17920050566331602, "floor_px": 1.0, "T": 1.0}
T_static = {"valid": true, "n": 113, "median": 0.056253400342208194, "mad": 0.016162234005075292, "sigma": 0.023962128135924625, "T_stat": 0.20002616915775592, "floor_px": 0.2, "T": 0.20002616915775592}

## 15. Static false-positive result

홀드아웃에서 relative displacement > T_motion 인 프레임: 0
프레임 번호: []
분류: NO_MOTION
static guard runs: 0

## 16. Continuous trajectory recording test

static_sham.csv 행 수 = 228 (프레임 수와 같음)
열에 timestamp, frame index, M2/STATIC/relative dx dy displacement, feature count, quality 가 있다.

## 17. READY_AND_ARMED

YES

## 18. Robot command count

0

## 19. Servo write count

0

## 20. Evidence path / Commit SHA / git status

evidence/P-007F-08/phase_a/
color sha256-of-sha256 = f09632534351c20bf8a8d87c6b37e37bcecb4579ac8e9c766897db6fa2594c87
raw color 는 .cache/p007f08_phase_a/sham 에만 있다.
커밋 해시는 푸시 뒤 완료 보고에 적는다.

```text
PHASE_B_EXECUTED = NO
ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT = 0
MOTION_AUTHORIZED = NO
```
