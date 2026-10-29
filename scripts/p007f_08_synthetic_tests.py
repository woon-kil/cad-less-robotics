#!/usr/bin/env python3
"""P-007F-08 Phase A — 카메라와 로봇 없이 추적·분류·금지 경로를 확인한다."""
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import p007f_08_phase_b
import p007f_08_tracker as tr


def texture(size=180, seed=0):
    rng = np.random.default_rng(seed)
    noise = rng.integers(0, 255, (size, size), dtype=np.uint8)
    return cv2.GaussianBlur(noise, (0, 0), 0.6)


def shift(image, dx, dy):
    matrix = np.array([[1, 0, dx], [0, 1, dy]], dtype=np.float32)
    return cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]), borderMode=cv2.BORDER_REFLECT)


def blank_row(index, relative=0.0, static=0.0, m2=None):
    if m2 is None and relative is not None:
        m2 = relative + static
    return {
        "frame_index": index,
        "relative_displacement": relative,
        "relative_dx": relative,
        "static_displacement": static,
        "m2_displacement": m2,
    }


def run():
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    base = texture()
    roi = (20, 20, 140, 140)
    moved = shift(base, 4.0, -2.0)
    tracked = tr.track_roi([base, moved], roi)
    last = tracked["frames"][-1]
    rec("lk_recovers_shift",
        last["dx"] is not None and abs(last["dx"] - 4.0) < 0.6 and abs(last["dy"] + 2.0) < 0.6,
        last)
    rec("lk_keeps_features", tracked["initial_count"] >= 8 and last["valid_count"] >= 8, tracked["initial_count"])

    phased = tr.phase_series([base, moved], roi)
    rec("phasecorr_recovers_shift",
        abs(phased[1]["dx"] - 4.0) < 0.6 and abs(phased[1]["dy"] + 2.0) < 0.6,
        phased[1])

    # 카메라 전체가 같이 움직이면 relative 는 0 근처여야 한다.
    both = [base, moved]
    m2 = tr.track_roi(both, roi)
    static = tr.track_roi(both, roi)
    rows = tr.combine(m2, static, tr.phase_series(both, roi), tr.phase_series(both, roi), [0.0, 0.1])
    rec("global_shift_cancels",
        rows[1]["relative_displacement"] is not None and rows[1]["relative_displacement"] < 0.6,
        rows[1]["relative_displacement"])

    quiet = [blank_row(i, 0.1) for i in range(20)]
    rec("class_no_motion", tr.classify_relative(quiet, 1.0) == "NO_MOTION")
    held = [blank_row(i, 0.0 if i < 5 else 8.0) for i in range(20)]
    rec("class_hold", tr.classify_relative(held, 1.0) == "MOVE_AND_HOLD", tr.classify_relative(held, 1.0))
    twitch = [blank_row(i, 8.0 if 5 <= i < 10 else 0.0) for i in range(20)]
    rec("class_twitch", tr.classify_relative(twitch, 1.0) == "TWITCH_AND_RETURN", tr.classify_relative(twitch, 1.0))
    bumps = []
    for i in range(30):
        on = (4 <= i < 8) or (12 <= i < 16) or (20 <= i < 24)
        bumps.append(blank_row(i, 8.0 if on else 0.0))
    rec("class_oscillation", tr.classify_relative(bumps, 1.0) == "OSCILLATION_OR_VIBRATION",
        tr.classify_relative(bumps, 1.0))
    missing = [blank_row(i, None) for i in range(20)]
    for row in missing:
        row["relative_displacement"] = None
        row["relative_dx"] = None
    rec("class_inconclusive", tr.classify_relative(missing, 1.0) == "INCONCLUSIVE")

    values = [0.2] * 10 + [5.0]
    estimated = tr.robust_threshold(values[:10], tr.MOTION_FLOOR_PX)
    rec("threshold_uses_floor", estimated["valid"] and estimated["T"] >= tr.MOTION_FLOOR_PX, estimated)
    rec("threshold_formula_constant", tr.K_SIGMA == 6.0 and tr.MOTION_FLOOR_PX == 1.0)

    est, hold = tr.split_indices(11)
    rec("split_drops_frame0", est[0] == 1 and 0 not in est and 0 not in hold, (est, hold))

    try:
        p007f_08_phase_b.execute_phase_b()
        rec("phase_b_refuses", False)
    except p007f_08_phase_b.PhaseBNotAuthorized:
        rec("phase_b_refuses", True)

    banned = ("Arm_serial", "controller_write_once", "ssh_jetson", "aruco", "solvePnP")
    for name in ("p007f_08_tracker.py", "p007f_08_phase_b.py"):
        text = open(os.path.join(ROOT, "scripts", name), encoding="utf-8").read()
        rec("no_banned_" + name, not any(token in text for token in banned))

    failed = [name for name, item in results.items() if not item["pass"]]
    for name, item in results.items():
        print(("PASS " if item["pass"] else "FAIL ") + name + " " + str(item["detail"]))
    if failed:
        raise SystemExit("FAILED " + ",".join(failed))
    print("ALL_PASS %d" % len(results))


if __name__ == "__main__":
    run()
