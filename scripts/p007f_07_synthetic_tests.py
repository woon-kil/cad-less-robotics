#!/usr/bin/env python3
"""P-007F-07 Phase A — §13/§17/§18 synthetic unit tests for the trajectory
classifier (Cases A-E) + the critical PRE=POST transient regression (§18).
Entirely synthetic numeric data -- no camera/robot calls.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import p007f_07_trajectory_classifier as tc

T = 1.5  # placeholder threshold for synthetic-logic verification only (NOT a real preregistered value)


def make_series(center_offsets, start_xy=(100.0, 100.0)):
    """center_offsets: list of (dx,dy) tuples -> synthetic marker_detector-style records."""
    out = []
    for i, (ox, oy) in enumerate(center_offsets):
        out.append({"frame_index": i, "timestamp": float(i) * 0.1, "marker_detected": True,
                   "marker_id": 0, "center_x": start_xy[0] + ox, "center_y": start_xy[1] + oy})
    return out


def flat_series(n, start_xy=(500.0, 500.0)):
    return make_series([(0.0, 0.0)] * n, start_xy)


def run():
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    # Case A: 0 -> 5px -> 5px (hold)
    primary_a = make_series([(v, 0) for v in [0, 0, 0, 5, 5, 5, 5, 5]])
    bg_a = flat_series(8)
    traj_a = tc.extract_trajectory(primary_a, bg_a)
    cls_a = tc.classify_motion(traj_a, T)
    rec("CASE_A_MOVE_AND_HOLD", cls_a["code"] == "M2", cls_a)

    # Case B: 0 -> 5px -> 0 (transient and return)
    primary_b = make_series([(v, 0) for v in [0, 0, 0, 5, 5, 5, 0, 0, 0]])
    bg_b = flat_series(9)
    traj_b = tc.extract_trajectory(primary_b, bg_b)
    cls_b = tc.classify_motion(traj_b, T)
    rec("CASE_B_TWITCH_AND_RETURN", cls_b["code"] == "M3", cls_b)

    # Case C: 0 -> +3 -> -3 -> +2 -> 0 (oscillation). 3 stable PRE frames so the
    # baseline (mean of first 3) is not itself contaminated by the first excursion.
    primary_c = make_series([(v, 0) for v in [0, 0, 0, 3, 3, -3, -3, 2, 2, 0, 0]])
    bg_c = flat_series(11)
    traj_c = tc.extract_trajectory(primary_c, bg_c)
    cls_c = tc.classify_motion(traj_c, T)
    rec("CASE_C_OSCILLATION", cls_c["code"] == "M4", cls_c)

    # Case D: 0 throughout (no motion)
    primary_d = flat_series(8)
    bg_d = flat_series(8)
    traj_d = tc.extract_trajectory(primary_d, bg_d)
    cls_d = tc.classify_motion(traj_d, T)
    rec("CASE_D_NO_MOTION", cls_d["code"] == "M1", cls_d)

    # Case E: primary static, secondary moves and holds -> downstream local motion
    primary_e = flat_series(8)
    bg_e = flat_series(8)
    secondary_e = make_series([(v, 0) for v in [0, 0, 0, 5, 5, 5, 5, 5]], start_xy=(300.0, 300.0))
    traj_e_primary = tc.extract_trajectory(primary_e, bg_e)
    traj_e_secondary = tc.extract_trajectory(secondary_e, bg_e)
    cls_e = tc.classify_motion(traj_e_primary, T, secondary_trajectory=traj_e_secondary)
    rec("CASE_E_DOWNSTREAM_LOCAL", cls_e["code"] == "M5", cls_e)

    # -----------------------------------------------------------------
    # §18 CRITICAL REGRESSION -- PRE == POST but motion occurred between.
    # If this fails, Phase A FAILS (per work order §18).
    # -----------------------------------------------------------------
    primary_regress = make_series([(v, 0) for v in [0, 0, 0, 6, 6, 0, 0, 0]])  # PRE(0,0,0) == POST(0,0,0), moved in between
    bg_regress = flat_series(8)
    traj_regress = tc.extract_trajectory(primary_regress, bg_regress)
    cls_regress = tc.classify_motion(traj_regress, T)
    pre_frames = primary_regress[0]["center_x"], primary_regress[1]["center_x"], primary_regress[2]["center_x"]
    post_frames = primary_regress[5]["center_x"], primary_regress[6]["center_x"], primary_regress[7]["center_x"]
    pre_equals_post = pre_frames == post_frames
    regression_pass = pre_equals_post and cls_regress["code"] == "M3"
    rec("CRITICAL_REGRESSION_PRE_EQ_POST_TRANSIENT_DETECTED", regression_pass,
       {"pre_equals_post_static_comparison_would_say": "NO_CHANGE" if pre_equals_post else "CHANGED",
        "continuous_classifier_says": cls_regress})

    # -----------------------------------------------------------------
    # background instability / insufficient valid frames / marker-loss aborts
    # -----------------------------------------------------------------
    primary_loss = [{"frame_index": i, "timestamp": i * 0.1, "marker_detected": (i < 2),
                     "marker_id": 0, "center_x": 100.0, "center_y": 100.0} for i in range(8)]
    bg_loss = flat_series(8)
    traj_loss = tc.extract_trajectory(primary_loss, bg_loss)
    cls_loss = tc.classify_motion(traj_loss, T)
    rec("MARKER_LOSS_ABORT", cls_loss["code"] == "MX", cls_loss)

    bg_unstable = make_series([(v, 0) for v in [0, 0, 10, 10, 10, 10, 10, 10]])  # background itself "moves" a lot
    primary_stable_bg_test = flat_series(8)
    traj_bg_unstable = tc.extract_trajectory(primary_stable_bg_test, bg_unstable)
    cls_bg_unstable = tc.classify_motion(traj_bg_unstable, T, background_instability=True)
    rec("BACKGROUND_INSTABILITY_ABORT", cls_bg_unstable["code"] == "MX", cls_bg_unstable)

    # threshold formula sanity
    t_formula = tc.threshold_formula(sham_mean=0.3, sham_stdev=0.1, fixed_floor=1.0)
    rec("THRESHOLD_FORMULA_USES_FLOOR_WHEN_SHAM_LOW", t_formula == 1.0, t_formula)
    t_formula2 = tc.threshold_formula(sham_mean=1.0, sham_stdev=0.5, fixed_floor=1.0)
    rec("THRESHOLD_FORMULA_USES_SHAM_WHEN_HIGHER", abs(t_formula2 - 3.5) < 1e-9, t_formula2)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "threshold_used_placeholder": T,
           "disclosure": "T=1.5 is a PLACEHOLDER for verifying classifier LOGIC only -- not a preregistered detection threshold. Real threshold set after marker static qualification (§16/§27).",
           "results": results}


if __name__ == "__main__":
    out = run()
    print(json.dumps(out, ensure_ascii=False, indent=1))
    sys.exit(0 if out["all_pass"] else 1)
