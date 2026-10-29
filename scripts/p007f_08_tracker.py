#!/usr/bin/env python3
"""P-007F-08 Phase A — sparse pyramidal Lucas-Kanade tracker.

마커 ID 검출과 자세 추정, 로봇 명령은 이 모듈에 없다.
첫 프레임에서 Shi-Tomasi 점을 잡고 calcOpticalFlowPyrLK 로만 이어 간다.
점은 다시 뽑지 않는다. 끊기면 그 점은 되살리지 않는다.

파라미터 출처 (구현 전에 고정, sham 이후 변경 금지):
- OpenCV 4.10 LK tutorial:
  https://docs.opencv.org/4.10.0/d4/dee/tutorial_optical_flow.html
  maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7
  winSize=(15,15), maxLevel=2, criteria=(EPS|COUNT, 10, 0.03)
- OpenCV sample lk_track.py 의 forward-backward 확인 (최대 오차 1 px):
  https://github.com/opencv/opencv/blob/master/samples/python/lk_track.py
- phaseCorrelate 식은 scripts/p007f_06_forensics.py _phasecorr 와 같다.
- 바닥값 1.0 px / 0.2 px 는 evidence/P-007F-03/thresholds.json 의
  engineering floor 와 static guard floor 다. 이번 sham 으로 맞춘 값이 아니다.
"""
import math

import cv2
import numpy as np

# (x, y, width, height). 역사적 attempt-02 프레임에서 sham 촬영 전에 고정.
# M2: 카드와 그 브래킷. 왼쪽 DaBai 몸통은 빼기 위해 x 를 카드 왼쪽에 붙인다.
M2_ROI = (738, 92, 100, 85)
# 키보드. 로봇에 고정된 구조물이 아니다. P-007F-03 의 300x300 박스는
# 책상 위 케이블을 포함하므로 이번 정적 기준으로 쓰지 않는다.
STATIC_REFERENCE_ROI = (1040, 300, 160, 120)

FEATURE_PARAMS = {
    "maxCorners": 100,
    "qualityLevel": 0.3,
    "minDistance": 7,
    "blockSize": 7,
}
LK_WIN_SIZE = (15, 15)
LK_MAX_LEVEL = 2
LK_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
FORWARD_BACKWARD_MAX_PX = 1.0

# T = max(floor, median + K_SIGMA * 1.4826 * MAD). 추정 구간과 홀드아웃을 나눈다.
K_SIGMA = 6.0
MAD_TO_SIGMA = 1.4826
MOTION_FLOOR_PX = 1.0
STATIC_GUARD_FLOOR_PX = 0.2
MIN_RUN_FRAMES = 3
MIN_INITIAL_FEATURES = 8
SURVIVAL_FRACTION = 0.5
SURVIVAL_FRAME_FRACTION = 0.95
MAX_MISSING_FRACTION = 0.05
HOLD_TAIL_FRACTION = 0.2
MIN_DURATION_S = 20.0
MIN_FRAME_COUNT = 150
ROI_SHIFT_MAX_PX = 15.0
ROI_PHASE_RESPONSE_MIN = 0.2
BASELINE_B_MAX_SHIFT_PX = 1.0

FORMULA_TEXT = (
    "frames [1, n) 를 반으로 나눈다. 앞쪽은 추정, 뒤쪽은 홀드아웃. "
    "frame 0 은 기준 프레임이라 변위가 0 이므로 분포에 넣지 않는다. "
    "sigma = 1.4826 * MAD. T_stat = median + 6 * sigma. "
    "T_motion = max(1.0 px, T_stat of relative LK displacement). "
    "T_static = max(0.2 px, T_stat of static LK displacement). "
    "홀드아웃에서 relative displacement > T_motion 인 프레임이 1장이라도 있으면 "
    "false positive 로 보고 READY_AND_ARMED 를 주지 않는다. "
    "분류의 연속 구간은 3프레임 이상만 사건으로 센다. "
    "이 공식은 sham 을 보기 전에 고정한다."
)


def crop(gray, roi):
    x, y, width, height = roi
    return gray[y:y + height, x:x + width]


def phase_correlate_shift(reference, current):
    """scripts/p007f_06_forensics.py _phasecorr 와 같은 호출이다."""
    shift, response = cv2.phaseCorrelate(reference.astype(np.float32), current.astype(np.float32))
    dx = float(shift[0])
    dy = float(shift[1])
    return {
        "dx": dx,
        "dy": dy,
        "magnitude": float(math.hypot(dx, dy)),
        "response": float(response),
    }


def init_features(gray, roi):
    mask = np.zeros(gray.shape, np.uint8)
    x, y, width, height = roi
    mask[y:y + height, x:x + width] = 255
    points = cv2.goodFeaturesToTrack(gray, mask=mask, **FEATURE_PARAMS)
    if points is None:
        return np.zeros((0, 2), dtype=np.float32)
    return points.reshape(-1, 2).astype(np.float32)


def _lk(prev_gray, gray, points):
    found, status, _err = cv2.calcOpticalFlowPyrLK(
        prev_gray, gray, points.reshape(-1, 1, 2), None,
        winSize=LK_WIN_SIZE, maxLevel=LK_MAX_LEVEL, criteria=LK_CRITERIA)
    back, status_back, _err_back = cv2.calcOpticalFlowPyrLK(
        gray, prev_gray, found, None,
        winSize=LK_WIN_SIZE, maxLevel=LK_MAX_LEVEL, criteria=LK_CRITERIA)
    fb = np.max(np.abs(points - back.reshape(-1, 2)), axis=1)
    good = (status.reshape(-1) == 1) & (status_back.reshape(-1) == 1) & (fb < FORWARD_BACKWARD_MAX_PX)
    return found.reshape(-1, 2), good, fb


def track_roi(grays, roi):
    """프레임 0 의 점을 다시 검출하지 않고 이어 간다. 변위는 그 점의 처음 위치 기준이다."""
    origin = init_features(grays[0], roi)
    current = origin.copy()
    initial_count = int(len(origin))
    prev = grays[0]
    rows = []
    for index, gray in enumerate(grays):
        if index == 0:
            rows.append({
                "dx": 0.0, "dy": 0.0, "displacement": 0.0,
                "valid_count": initial_count, "fb_median_px": 0.0,
                "quality": 1.0 if initial_count else 0.0,
            })
            continue
        if len(current) == 0:
            rows.append({
                "dx": None, "dy": None, "displacement": None,
                "valid_count": 0, "fb_median_px": None, "quality": 0.0,
            })
            prev = gray
            continue
        found, good, fb = _lk(prev, gray, current)
        current = found[good]
        origin = origin[good]
        if len(current) == 0:
            rows.append({
                "dx": None, "dy": None, "displacement": None,
                "valid_count": 0, "fb_median_px": None, "quality": 0.0,
            })
        else:
            delta = current - origin
            dx = float(np.median(delta[:, 0]))
            dy = float(np.median(delta[:, 1]))
            rows.append({
                "dx": dx, "dy": dy, "displacement": float(math.hypot(dx, dy)),
                "valid_count": int(len(current)),
                "fb_median_px": float(np.median(fb[good])),
                "quality": float(len(current) / initial_count) if initial_count else 0.0,
            })
        prev = gray
    return {"initial_count": initial_count, "frames": rows, "final_points": current, "origin_of_final": origin}


def phase_series(grays, roi):
    reference = crop(grays[0], roi)
    return [phase_correlate_shift(reference, crop(gray, roi)) for gray in grays]


def combine(m2, static_track, m2_phase, static_phase, timestamps):
    rows = []
    for index, stamp in enumerate(timestamps):
        m2_row = m2["frames"][index]
        st_row = static_track["frames"][index]
        rel_dx = rel_dy = rel_disp = None
        if m2_row["dx"] is not None and st_row["dx"] is not None:
            rel_dx = m2_row["dx"] - st_row["dx"]
            rel_dy = m2_row["dy"] - st_row["dy"]
            rel_disp = float(math.hypot(rel_dx, rel_dy))
        phase_dx = m2_phase[index]["dx"] - static_phase[index]["dx"]
        phase_dy = m2_phase[index]["dy"] - static_phase[index]["dy"]
        rows.append({
            "timestamp": stamp,
            "frame_index": index,
            "m2_dx": m2_row["dx"], "m2_dy": m2_row["dy"], "m2_displacement": m2_row["displacement"],
            "static_dx": st_row["dx"], "static_dy": st_row["dy"], "static_displacement": st_row["displacement"],
            "relative_dx": rel_dx, "relative_dy": rel_dy, "relative_displacement": rel_disp,
            "m2_valid_feature_count": m2_row["valid_count"],
            "static_valid_feature_count": st_row["valid_count"],
            "m2_tracking_quality": m2_row["quality"],
            "static_tracking_quality": st_row["quality"],
            "m2_fb_median_px": m2_row["fb_median_px"],
            "static_fb_median_px": st_row["fb_median_px"],
            "m2_phase_dx": m2_phase[index]["dx"],
            "m2_phase_dy": m2_phase[index]["dy"],
            "m2_phase_magnitude": m2_phase[index]["magnitude"],
            "m2_phase_response": m2_phase[index]["response"],
            "static_phase_dx": static_phase[index]["dx"],
            "static_phase_dy": static_phase[index]["dy"],
            "static_phase_magnitude": static_phase[index]["magnitude"],
            "static_phase_response": static_phase[index]["response"],
            "relative_phase_dx": phase_dx,
            "relative_phase_dy": phase_dy,
            "relative_phase_displacement": float(math.hypot(phase_dx, phase_dy)),
        })
    return rows


def split_indices(n_frames):
    """frame 0 을 빼고 [1, n) 을 추정/홀드아웃으로 나눈다."""
    rest = list(range(1, n_frames))
    mid = len(rest) // 2
    return rest[:mid], rest[mid:]


def _finite(values):
    return [float(value) for value in values if value is not None and math.isfinite(value)]


def robust_threshold(values, floor_px):
    clean = _finite(values)
    if len(clean) < 5:
        return {"valid": False, "reason": "TOO_FEW_ESTIMATION_FRAMES", "T": None}
    arr = np.asarray(clean, dtype=float)
    median = float(np.median(arr))
    mad = float(np.median(np.abs(arr - median)))
    sigma = MAD_TO_SIGMA * mad
    t_stat = median + K_SIGMA * sigma
    return {
        "valid": True,
        "n": int(arr.size),
        "median": median,
        "mad": mad,
        "sigma": sigma,
        "T_stat": t_stat,
        "floor_px": floor_px,
        "T": float(max(floor_px, t_stat)),
    }


def _runs(mask, min_run):
    found = []
    index = 0
    while index < len(mask):
        if not mask[index]:
            index += 1
            continue
        end = index
        while end < len(mask) and mask[end]:
            end += 1
        if end - index >= min_run:
            found.append((index, end - 1))
        index = end
    return found


def classify_relative(rows, t_motion):
    values = [row["relative_displacement"] for row in rows]
    signed = [row["relative_dx"] for row in rows]
    missing = sum(value is None for value in values)
    if not values or missing / len(values) > MAX_MISSING_FRACTION:
        return "INCONCLUSIVE"
    mask = [value is not None and value > t_motion for value in values]
    runs = _runs(mask, MIN_RUN_FRAMES)
    if not runs:
        return "NO_MOTION"
    flips = 0
    previous = None
    for value, active in zip(signed, mask):
        # 임계값 아래의 부호 흔들림은 진동으로 세지 않는다.
        if not active or value is None or abs(value) < 1e-9:
            continue
        sign = 1 if value > 0 else -1
        if previous is not None and sign != previous:
            flips += 1
        previous = sign
    if len(runs) >= 2 or flips >= 2:
        return "OSCILLATION_OR_VIBRATION"
    tail_n = max(1, int(len(values) * HOLD_TAIL_FRACTION))
    tail = _finite(values[-tail_n:])
    if not tail:
        return "INCONCLUSIVE"
    end_median = float(np.median(tail))
    if len(runs) == 1 and end_median > t_motion:
        return "MOVE_AND_HOLD"
    if len(runs) == 1 and end_median <= t_motion:
        return "TWITCH_AND_RETURN"
    return "ROBOT_LOCAL_DISTURBANCE"


def series_stats(values):
    clean = _finite(values)
    if not clean:
        return {"n": 0}
    arr = np.asarray(clean, dtype=float)
    return {
        "n": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "median": float(np.median(arr)),
        "min": float(arr.min()),
        "max": float(arr.max()),
        "p95": float(np.percentile(arr, 95)),
    }


def survival_ok(rows, count_key, initial_count):
    if initial_count < MIN_INITIAL_FEATURES:
        return False
    need = SURVIVAL_FRACTION * initial_count
    alive_frames = sum(row[count_key] >= need for row in rows)
    return alive_frames / len(rows) >= SURVIVAL_FRAME_FRACTION


def preregistration_document():
    return {
        "gate": "P-007F-08",
        "phase": "A",
        "primary": "sparse pyramidal Lucas-Kanade",
        "secondary": "phase correlation, not used for the motion call",
        "template_matching": "not used",
        "marker_id_detection_used": False,
        "feature_params": FEATURE_PARAMS,
        "lk_win_size": list(LK_WIN_SIZE),
        "lk_max_level": LK_MAX_LEVEL,
        "lk_criteria": [int(LK_CRITERIA[0]), int(LK_CRITERIA[1]), float(LK_CRITERIA[2])],
        "forward_backward_max_px": FORWARD_BACKWARD_MAX_PX,
        "sources": {
            "lk_tutorial": "https://docs.opencv.org/4.10.0/d4/dee/tutorial_optical_flow.html",
            "lk_sample_fb": "https://github.com/opencv/opencv/blob/master/samples/python/lk_track.py",
            "phase_correlate": "scripts/p007f_06_forensics.py _phasecorr",
            "floors": "evidence/P-007F-03/thresholds.json engineering floor 1.0 px and static guard 0.2 px",
        },
        "m2_roi": {"x": M2_ROI[0], "y": M2_ROI[1], "width": M2_ROI[2], "height": M2_ROI[3]},
        "static_reference_roi": {
            "x": STATIC_REFERENCE_ROI[0], "y": STATIC_REFERENCE_ROI[1],
            "width": STATIC_REFERENCE_ROI[2], "height": STATIC_REFERENCE_ROI[3],
            "what": "keyboard on the workcell, not a robot-mounted part",
        },
        "formula": FORMULA_TEXT,
        "K_SIGMA": K_SIGMA,
        "mad_to_sigma": MAD_TO_SIGMA,
        "motion_floor_px": MOTION_FLOOR_PX,
        "static_guard_floor_px": STATIC_GUARD_FLOOR_PX,
        "min_initial_features": MIN_INITIAL_FEATURES,
        "survival_fraction_of_initial": SURVIVAL_FRACTION,
        "survival_frame_fraction": SURVIVAL_FRAME_FRACTION,
        "min_duration_s": MIN_DURATION_S,
        "min_frame_count": MIN_FRAME_COUNT,
        "roi_confirmation": {
            "shift_max_px": ROI_SHIFT_MAX_PX,
            "phase_response_min": ROI_PHASE_RESPONSE_MIN,
            "against": "attempt-02 representative frame 000128, same ROI, phase correlation",
        },
        "baseline_b_max_shift_px": BASELINE_B_MAX_SHIFT_PX,
        "false_positive_rule": "holdout frames with relative LK displacement > T_motion must be 0",
        "static_guard_rule": "a holdout run of >= 3 frames above T_static invalidates the window",
        "phase_b_executed_by_this_module": False,
    }
