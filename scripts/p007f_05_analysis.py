#!/usr/bin/env python3
"""P-007F-05 — §25/§26 Channel B visual motion analysis + quality guard.

P-007F-03 에서 고정한 알고리즘(ORB+BFMatcher+estimateAffinePartial2D/RANSAC,
scripts/p007f_03_visual_analysis.py.algo_orb, SHA-pin import)·ROI·threshold를
그대로 재사용한다. 이 파일은 그 알고리즘을 실행창(window)의 PRE/POST mean-frame
쌍에 적용해 quality guard(§19/evidence/P-007F-03/quality_guard.json) ->
static-reference guard -> robot-relative motion classification 순서로 판정한다.
새 threshold를 만들지 않는다 — 전부 evidence/P-007F-03/thresholds.json·
quality_guard.json 에서 그대로 가져온 고정값이다.
"""
import hashlib
import importlib
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

VISUAL_ANALYSIS_MODULE_RELPATH = "scripts/p007f_03_visual_analysis.py"
VISUAL_ANALYSIS_MODULE_SHA = "5ded8c75349cd91018cb6c024812e6fa07c80a2a77c525f21136ff6b11931ed2"

# evidence/P-007F-03/roi_definition.json (color 1280x720) — 그대로 재사용
ROBOT_ROI = (595, 90, 210, 255)
ARM2_ROI = (650, 220, 120, 70)
STATIC_REFERENCE_ROI = (900, 150, 300, 300)

# evidence/P-007F-03/thresholds.json — 그대로 재사용, 이 gate에서 변경하지 않음
T_TRANSLATION_PX = 1.7826
T_ROTATION_DEG = 1.0778
S_TRANSLATION_PX = 0.2
S_ROTATION_DEG = 0.05

# evidence/P-007F-03/quality_guard.json.window_level_guards — 그대로 재사용
MIN_VALID_FRAMES_OF_3 = 2
MIN_MATCH_COUNT = 4
MIN_INLIER_COUNT = 4
MIN_INLIER_RATIO = 0.5
MAX_REJECTED_FRAME_RATIO = 0.2


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def check_module_sha(repo_root=ROOT):
    full = os.path.join(repo_root, VISUAL_ANALYSIS_MODULE_RELPATH)
    try:
        actual = sha256(full)
    except OSError as e:
        return {"match": False, "expected": VISUAL_ANALYSIS_MODULE_SHA, "actual": "READ_ERROR:%r" % e}
    return {"match": actual == VISUAL_ANALYSIS_MODULE_SHA, "expected": VISUAL_ANALYSIS_MODULE_SHA,
            "actual": actual}


def _load_visual_analysis():
    return importlib.import_module("p007f_03_visual_analysis")


def crop(frame, roi):
    x, y, w, h = roi
    return frame[y:y + h, x:x + w]


def window_level_guard(valid_frames, requested_frames):
    """evidence/P-007F-03/quality_guard.json.window_level_guards 그대로.
    여기서는 '한 window' 자체의 valid-frame 비율만 본다(2/3 최소) — rejected-frame
    ratio(세션 전체, 0.2)는 호출자가 session_level_guard() 로 별도 평가한다."""
    enough = valid_frames >= MIN_VALID_FRAMES_OF_3
    return {"pass": enough, "valid_frames": valid_frames, "minimum_required": MIN_VALID_FRAMES_OF_3,
            "reason": None if enough else "INSUFFICIENT_VALID_FRAMES"}


def session_level_guard(valid_frames, requested_frames):
    if requested_frames <= 0:
        return {"pass": False, "reason": "NO_FRAMES_REQUESTED"}
    rejected_ratio = 1.0 - (valid_frames / requested_frames)
    ok = rejected_ratio <= MAX_REJECTED_FRAME_RATIO
    return {"pass": ok, "rejected_frame_ratio": rejected_ratio,
            "maximum_allowed": MAX_REJECTED_FRAME_RATIO,
            "reason": None if ok else "EXCESSIVE_REJECTED_FRAME_RATIO"}


def evaluate_window_pair(pre_mean_gray, post_mean_gray,
                         pre_valid_frames, pre_requested_frames,
                         post_valid_frames, post_requested_frames):
    """§19/§23/§24/§25/§26 — 전체 파이프라인을 한 번에 평가한다.

    순서(quality_guard.json.evaluation_order 그대로):
      1) window-level guard (PRE/POST 각각)
      2) static_reference_guard (raw STATIC_REFERENCE_ROI)
      3) (1,2 모두 통과시에만) robot_motion_threshold 적용

    반환 dict 의 "quality_pass" 가 False 면 호출자는 EXPERIMENT_INVALID로
    처리하고(§19/§23) "motion_yes_no" 를 사용하지 않는다 — result matrix 미적용.
    """
    va = _load_visual_analysis()

    pre_window = window_level_guard(pre_valid_frames, pre_requested_frames)
    post_window = window_level_guard(post_valid_frames, post_requested_frames)
    pre_session = session_level_guard(pre_valid_frames, pre_requested_frames)
    post_session = session_level_guard(post_valid_frames, post_requested_frames)

    window_guard_pass = (pre_window["pass"] and post_window["pass"]
                        and pre_session["pass"] and post_session["pass"])

    arm2_pre = crop(pre_mean_gray, ARM2_ROI)
    arm2_post = crop(post_mean_gray, ARM2_ROI)
    static_pre = crop(pre_mean_gray, STATIC_REFERENCE_ROI)
    static_post = crop(post_mean_gray, STATIC_REFERENCE_ROI)

    arm2_result = va.algo_orb(arm2_pre, arm2_post)
    static_result = va.algo_orb(static_pre, static_post)

    arm2_feature_guard_pass = (
        arm2_result.get("valid")
        and arm2_result.get("match_count", 0) >= MIN_MATCH_COUNT
        and arm2_result.get("inlier_count", 0) >= MIN_INLIER_COUNT
        and arm2_result.get("inlier_ratio", 0.0) >= MIN_INLIER_RATIO)

    if not (window_guard_pass and arm2_feature_guard_pass):
        return {
            "quality_pass": False, "reason": "WINDOW_OR_FEATURE_GUARD_FAILED",
            "pre_window": pre_window, "post_window": post_window,
            "pre_session": pre_session, "post_session": post_session,
            "arm2_result": arm2_result, "static_result": static_result,
            "static_reference_guard": None, "robot_relative": None, "motion_yes_no": None,
        }

    static_translation_px = static_result.get("translation_px") if static_result.get("valid") else None
    static_rotation_deg = static_result.get("rotation_deg") if static_result.get("valid") else None
    static_ref_pass = (
        static_result.get("valid")
        and static_translation_px is not None and static_translation_px <= S_TRANSLATION_PX
        and static_rotation_deg is not None and abs(static_rotation_deg) <= S_ROTATION_DEG)
    static_reference_guard = {
        "pass": bool(static_ref_pass),
        "static_translation_px": static_translation_px, "static_rotation_deg": static_rotation_deg,
        "S_translation_px": S_TRANSLATION_PX, "S_rotation_deg": S_ROTATION_DEG,
        "reason": None if static_ref_pass else "STATIC_REFERENCE_GUARD_FAILED",
    }

    if not static_ref_pass:
        return {
            "quality_pass": False, "reason": "STATIC_REFERENCE_GUARD_FAILED",
            "pre_window": pre_window, "post_window": post_window,
            "pre_session": pre_session, "post_session": post_session,
            "arm2_result": arm2_result, "static_result": static_result,
            "static_reference_guard": static_reference_guard, "robot_relative": None,
            "motion_yes_no": None,
        }

    robot_relative_translation_px = arm2_result["translation_px"] - static_translation_px
    robot_relative_rotation_deg = arm2_result["rotation_deg"] - static_rotation_deg
    motion_detected = (robot_relative_translation_px > T_TRANSLATION_PX
                       or abs(robot_relative_rotation_deg) > T_ROTATION_DEG)

    return {
        "quality_pass": True, "reason": None,
        "pre_window": pre_window, "post_window": post_window,
        "pre_session": pre_session, "post_session": post_session,
        "arm2_result": arm2_result, "static_result": static_result,
        "static_reference_guard": static_reference_guard,
        "robot_relative": {
            "translation_px": robot_relative_translation_px,
            "rotation_deg": robot_relative_rotation_deg,
            "T_translation_px": T_TRANSLATION_PX, "T_rotation_deg": T_ROTATION_DEG,
        },
        "motion_yes_no": "YES" if motion_detected else "NO",
    }


# ---------------------------------------------------------------------------
# selftest — hardware-free. 합성(synthetic) 이미지 쌍으로 세 경로(guard fail /
# static-ref fail / 정상판정) 를 모두 돌려본다. 실 카메라 프레임 없이 검증 가능.
# ---------------------------------------------------------------------------
def _synthetic_textured_frame(shape=(720, 1280), seed=0):
    import numpy as np
    rng = np.random.RandomState(seed)
    base = rng.randint(0, 255, size=shape, dtype=np.uint8)
    return base


def selftest():
    import numpy as np
    results = {}

    def rec(name, cond, detail=None):
        results[name] = {"pass": bool(cond), "detail": detail}

    rec("AN-01_module_sha_pin_matches", check_module_sha()["match"], check_module_sha())

    frame = _synthetic_textured_frame()
    # identical frames -> no motion anywhere, both ROIs' algo_orb should see ~0 translation
    r_identical = evaluate_window_pair(frame, frame, 3, 3, 3, 3)
    rec("AN-02_identical_frames_no_motion_detected",
        r_identical["quality_pass"] and r_identical["motion_yes_no"] == "NO", r_identical)

    # insufficient valid frames -> EXPERIMENT_INVALID via window guard, before any ROI analysis
    r_bad_window = evaluate_window_pair(frame, frame, 1, 3, 3, 3)
    rec("AN-03_insufficient_valid_frames_invalid",
        not r_bad_window["quality_pass"] and r_bad_window["reason"] == "WINDOW_OR_FEATURE_GUARD_FAILED",
        r_bad_window)

    # excessive rejected-frame ratio at session level -> invalid even if this window's count is ok
    r_bad_session = evaluate_window_pair(frame, frame, 3, 10, 3, 3)
    rec("AN-04_excessive_rejected_ratio_invalid",
        not r_bad_session["quality_pass"], r_bad_session)

    all_pass = all(v["pass"] for v in results.values())
    return {"all_pass": all_pass, "results": results}


if __name__ == "__main__":
    import json
    out = selftest()
    print(json.dumps(out, ensure_ascii=False, indent=1, default=repr))
    sys.exit(0 if out["all_pass"] else 1)
