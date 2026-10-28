#!/usr/bin/env python3
"""P-007F-03 — offline visual-algorithm comparison + sham trial analysis.

읽기 전용 오프라인 분석. evidence/P-007F-03/static_frames/ 의 70프레임
static dataset(로봇/카메라 무변동)만 사용한다. 하드웨어 호출 없음.

4개 후보 알고리즘(ORB+RANSAC, Farneback optical flow, edge+phaseCorrelate,
template matching)을 10개 sham PRE/POST trial에 대해 비교하고, ARM2_ROI와
STATIC_REFERENCE_ROI의 차이로 robot-relative visual change를 계산한다.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
FRAMES_DIR = ROOT / "evidence" / "P-007F-03" / "frames"
OUT_DIR = ROOT / "evidence" / "P-007F-03"

ROIS = {
    "robot_roi": (595, 90, 210, 255),
    "arm2_roi": (650, 220, 120, 70),
    "downstream_roi": (600, 90, 200, 170),
    "static_reference_roi": (900, 150, 300, 300),
}

NUM_FRAMES = 70
WINDOW = 3
NSHAM = 10


def load_gray(index):
    path = FRAMES_DIR / f"{index:04d}-color.png"
    img = cv2.imread(str(path))
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def crop(img, roi):
    x, y, w, h = roi
    return img[y:y + h, x:x + w]


def mean_frame(indices):
    frames = [load_gray(i).astype(np.float64) for i in indices]
    return np.mean(frames, axis=0).astype(np.uint8)


# ---- Algorithm A: ORB + BFMatcher + RANSAC affine ----
_orb = cv2.ORB_create(nfeatures=500)
_bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)


def algo_orb(pre_crop, post_crop):
    kp1, des1 = _orb.detectAndCompute(pre_crop, None)
    kp2, des2 = _orb.detectAndCompute(post_crop, None)
    if des1 is None or des2 is None or len(kp1) < 4 or len(kp2) < 4:
        return {"valid": False, "reason": "insufficient_keypoints", "kp1": len(kp1) if kp1 else 0, "kp2": len(kp2) if kp2 else 0}
    matches = _bf.knnMatch(des1, des2, k=2)
    good = []
    for pair in matches:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < 0.75 * n.distance:
            good.append(m)
    if len(good) < 4:
        return {"valid": False, "reason": "insufficient_matches", "match_count": len(good)}
    src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    affine, inlier_mask = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=3.0)
    if affine is None:
        return {"valid": False, "reason": "affine_estimation_failed", "match_count": len(good)}
    inliers = int(inlier_mask.sum()) if inlier_mask is not None else 0
    tx, ty = float(affine[0, 2]), float(affine[1, 2])
    rotation_deg = math.degrees(math.atan2(affine[1, 0], affine[0, 0]))
    return {
        "valid": True,
        "match_count": len(good),
        "inlier_count": inliers,
        "inlier_ratio": inliers / len(good),
        "translation_px": math.hypot(tx, ty),
        "rotation_deg": rotation_deg,
        "kp1": len(kp1),
        "kp2": len(kp2),
    }


# ---- Algorithm B: Farneback dense optical flow ----
def algo_optical_flow(pre_crop, post_crop):
    if pre_crop.shape[0] < 8 or pre_crop.shape[1] < 8:
        return {"valid": False, "reason": "roi_too_small"}
    flow = cv2.calcOpticalFlowFarneback(pre_crop, post_crop, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    mag = np.sqrt(flow[..., 0] ** 2 + flow[..., 1] ** 2)
    return {
        "valid": True,
        "translation_px": float(np.mean(mag)),
        "translation_px_median": float(np.median(mag)),
        "rotation_deg": None,
        "note": "optical flow 는 rotation 을 직접 추정하지 않음(§11 명시)",
    }


# ---- Algorithm C: edge + phase correlation ----
def algo_edge_phasecorr(pre_crop, post_crop):
    pre_edge = cv2.Canny(pre_crop, 50, 150).astype(np.float32)
    post_edge = cv2.Canny(post_crop, 50, 150).astype(np.float32)
    if pre_edge.sum() == 0 or post_edge.sum() == 0:
        return {"valid": False, "reason": "no_edges"}
    shift, response = cv2.phaseCorrelate(pre_edge, post_edge)
    return {
        "valid": True,
        "translation_px": math.hypot(shift[0], shift[1]),
        "rotation_deg": None,
        "phase_response": float(response),
        "note": "edge+phaseCorrelate 는 translation 만 추정, rotation 미추정",
    }


# ---- Algorithm D: template matching (ROI registration) ----
def algo_template_match(pre_crop, post_frame_full, roi):
    x, y, w, h = roi
    margin = 20
    x0 = max(0, x - margin)
    y0 = max(0, y - margin)
    x1 = min(post_frame_full.shape[1], x + w + margin)
    y1 = min(post_frame_full.shape[0], y + h + margin)
    search = post_frame_full[y0:y1, x0:x1]
    if search.shape[0] < h or search.shape[1] < w:
        return {"valid": False, "reason": "search_region_too_small"}
    result = cv2.matchTemplate(search, pre_crop, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    found_x = x0 + max_loc[0]
    found_y = y0 + max_loc[1]
    shift_x = found_x - x
    shift_y = found_y - y
    return {
        "valid": True,
        "translation_px": math.hypot(shift_x, shift_y),
        "rotation_deg": None,
        "match_score": float(max_val),
        "note": "template matching 은 translation 만 추정, rotation 미추정",
    }


def build_sham_trials():
    trials = []
    for i in range(NSHAM):
        base = i * (2 * WINDOW + 1)
        pre = list(range(base, base + WINDOW))
        post = list(range(base + WINDOW + 1, base + 2 * WINDOW + 1))
        trials.append({"trial": i, "pre_frames": pre, "post_frames": post})
    return trials


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # 1. trackability test (marker requirement) — ORB keypoints inside ARM2_ROI, single frame
    frame0 = load_gray(0)
    arm2_crop0 = crop(frame0, ROIS["arm2_roi"])
    robot_crop0 = crop(frame0, ROIS["robot_roi"])
    kp_arm2, des_arm2 = _orb.detectAndCompute(arm2_crop0, None)
    kp_robot, des_robot = _orb.detectAndCompute(robot_crop0, None)
    trackability = {
        "arm2_roi_keypoint_count": len(kp_arm2) if kp_arm2 else 0,
        "robot_roi_keypoint_count": len(kp_robot) if kp_robot else 0,
        "threshold_for_adequate_trackability": 15,
        "marker_required": (len(kp_arm2) if kp_arm2 else 0) < 15,
    }

    # 2. sham trials
    trials = build_sham_trials()
    full_frames_pre_mean = {}
    full_frames_post_mean = {}
    for t in trials:
        full_frames_pre_mean[t["trial"]] = mean_frame(t["pre_frames"])
        full_frames_post_mean[t["trial"]] = mean_frame(t["post_frames"])

    results = {"orb": [], "optical_flow": [], "edge_phasecorr": [], "template_match": []}
    for t in trials:
        idx = t["trial"]
        pre_full = full_frames_pre_mean[idx]
        post_full = full_frames_post_mean[idx]

        for roi_name in ("arm2_roi", "static_reference_roi"):
            roi = ROIS[roi_name]
            pre_c = crop(pre_full, roi)
            post_c = crop(post_full, roi)

            r_orb = algo_orb(pre_c, post_c)
            r_orb.update({"trial": idx, "roi": roi_name})
            results["orb"].append(r_orb)

            r_flow = algo_optical_flow(pre_c, post_c)
            r_flow.update({"trial": idx, "roi": roi_name})
            results["optical_flow"].append(r_flow)

            r_edge = algo_edge_phasecorr(pre_c, post_c)
            r_edge.update({"trial": idx, "roi": roi_name})
            results["edge_phasecorr"].append(r_edge)

            r_tmpl = algo_template_match(pre_c, post_full, roi)
            r_tmpl.update({"trial": idx, "roi": roi_name})
            results["template_match"].append(r_tmpl)

    (OUT_DIR / "trackability.json").write_text(json.dumps(trackability, indent=2) + "\n")
    (OUT_DIR / "sham_trials.json").write_text(json.dumps({"window": WINDOW, "nsham": NSHAM, "trials": trials}, indent=2) + "\n")
    (OUT_DIR / "algorithm_comparison_raw.json").write_text(json.dumps(results, indent=2) + "\n")
    print("trackability:", json.dumps(trackability))
    print("done")


if __name__ == "__main__":
    main()
