#!/usr/bin/env python3
"""P-007F-07B-R1 attempt 2. 정지 촬영을 기존 합격선으로만 판정한다.

production detector(create_detector) 결과만 qualification에 쓴다.
parameter variant는 진단 숫자일 뿐 PASS에 쓰지 않는다.
로봇/카메라 API는 호출하지 않는다. 이미 받아 둔 PNG만 읽는다.
이전 abort evidence 디렉터리에는 쓰지 않는다.
"""
import csv
import hashlib
import importlib.util
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAPTURE_DIR = os.environ.get(
    "P007F07B_R1_CAPTURE_DIR",
    os.path.join(ROOT, ".cache", "p007f07b_r1", "attempt2_run"))
PRECHECK_DIR = os.environ.get(
    "P007F07B_R1_PRECHECK_DIR",
    os.path.join(ROOT, ".cache", "p007f07b_r1", "attempt2_precheck"))
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_static_qualification_r1", "attempt-02")
FIG_DIR = os.path.join(OUT_DIR, "figures")
RAW_DET = os.path.join(OUT_DIR, "raw", "representative_detected")
RAW_FAIL = os.path.join(OUT_DIR, "raw", "representative_failed")

DETECTOR_PATH = os.path.join(ROOT, "scripts", "p007f_07_marker_detector.py")
TRAJ_PATH = os.path.join(ROOT, "scripts", "p007f_07_trajectory_classifier.py")
FORENSICS_PATH = os.path.join(ROOT, "scripts", "p007f_06_forensics.py")
WIRING_PATH = os.path.join(ROOT, "scripts", "p007f_05_hardware_wiring.py")

DETECTION_RATE_ACCEPTANCE = 0.95
MEAN_SIDE_PX_ACCEPTANCE = 35.0
VALID_FOUR_CORNER_ACCEPTANCE = 0.95
CLIP_MARGIN_PX = 3
SAMPLE_COUNT = 10


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def mean_std_ptp(values):
    import numpy as np
    arr = np.asarray(values, dtype=float)
    return {
        "n": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "median": float(np.median(arr)),
        "min": float(arr.min()),
        "max": float(arr.max()),
        "peak_to_peak": float(arr.max() - arr.min()),
    }


def evenly(indices, count):
    if not indices:
        return []
    if len(indices) <= count:
        return list(indices)
    return [indices[int(round(i * (len(indices) - 1) / (count - 1)))] for i in range(count)]


def corner_metrics(hit, np):
    corners = np.array([hit["corner_%d" % k] for k in range(4)], dtype=float)
    sides = [float(np.linalg.norm(corners[i] - corners[(i + 1) % 4])) for i in range(4)]
    angles = []
    for i in range(4):
        v1 = corners[(i - 1) % 4] - corners[i]
        v2 = corners[(i + 1) % 4] - corners[i]
        denom = float(np.linalg.norm(v1) * np.linalg.norm(v2)) + 1e-9
        angles.append(float(math.degrees(math.acos(float(np.clip(np.dot(v1, v2) / denom, -1, 1))))))
    aspect = max(sides) / max(min(sides), 1e-9)
    opposite = max(sides[0] / sides[2], sides[2] / sides[0], sides[1] / sides[3], sides[3] / sides[1])
    return sides, angles, aspect, opposite


def roi_stats(gray, cv2, np):
    flat = gray.astype(np.float64)
    p10, p90 = np.percentile(flat, [10, 90])
    return {
        "laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "mean_intensity": float(flat.mean()),
        "std_intensity": float(flat.std()),
        "p10_intensity": float(p10),
        "p90_intensity": float(p90),
        "contrast_p90_p10": float(p90 - p10),
        "near_black_ratio": float((flat < 25).mean()),
        "near_white_ratio": float((flat > 240).mean()),
    }


def loss_runs(flags):
    runs = []
    start = None
    for i, flag in enumerate(flags):
        if not flag and start is None:
            start = i
        elif flag and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(flags) - 1))
    return runs


def loss_pattern(runs, n_frames):
    if not runs:
        return "no_loss"
    lengths = [b - a + 1 for a, b in runs]
    longest = max(lengths)
    if longest >= 0.5 * n_frames:
        return "long_continuous_loss"
    long_runs = sum(1 for length in lengths if length >= 5)
    if long_runs >= 3:
        return "burst"
    if len(runs) >= 4:
        gaps = [runs[i + 1][0] - runs[i][0] for i in range(len(runs) - 1)]
        mean_gap = sum(gaps) / len(gaps)
        if mean_gap > 0:
            var = sum((g - mean_gap) ** 2 for g in gaps) / len(gaps)
            if (var ** 0.5) / mean_gap < 0.25:
                return "periodic"
    return "random"


def variant_detector(cv2, kind):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    if kind == "no_corner_refine":
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    elif kind == "adaptive_wide":
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
    elif kind == "adaptive_narrow":
        params.adaptiveThreshWinSizeMin = 13
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
    return cv2.aruco.ArucoDetector(dictionary, params)


def id0_hits(hits):
    return [h for h in hits if h.get("marker_detected") and h.get("marker_id") == 0]


def scan_dir(det, detector, directory):
    meta = json.load(open(os.path.join(directory, "frames_meta.json"), encoding="utf-8"))
    rows = []
    for fm in meta:
        image = det.cv2.imread(os.path.join(directory, fm["rgb_filename"]))
        hits = det.detect_markers(image, detector, timestamp=fm["elapsed_s_since_start"],
                                   frame_index=fm["frame_index"])
        rows.append((fm, image, hits))
    return rows


def main():
    det = _load("marker_detector_pin", DETECTOR_PATH)
    traj = _load("trajectory_pin", TRAJ_PATH)
    forensics = _load("forensics_pin", FORENSICS_PATH)
    wiring = _load("wiring_pin", WIRING_PATH)
    cv2, np = det.cv2, det.np
    production = det.create_detector()

    meta = json.load(open(os.path.join(CAPTURE_DIR, "frames_meta.json"), encoding="utf-8"))
    n_frames = len(meta)
    duration_s = meta[-1]["elapsed_s_since_start"]
    command_events = json.load(open(os.path.join(CAPTURE_DIR, "command_events.json"), encoding="utf-8"))

    static_roi = forensics.ROIS["STATIC_REFERENCE_ROI"]
    frame0 = cv2.imread(os.path.join(CAPTURE_DIR, meta[0]["rgb_filename"]))
    img_h, img_w = frame0.shape[:2]
    frame0_static = forensics.crop(cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY), static_roi)
    baseline = wiring.baseline_b_check(os.path.join(CAPTURE_DIR, meta[0]["rgb_filename"]))

    detection_rows = []
    primary_series = []
    background_series = []
    duplicate_frames = []
    other_id_frames = []

    for fm in meta:
        image = cv2.imread(os.path.join(CAPTURE_DIR, fm["rgb_filename"]))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ts = fm["elapsed_s_since_start"]
        hits = det.detect_markers(image, production, timestamp=ts, frame_index=fm["frame_index"])
        detected = [h for h in hits if h.get("marker_detected")]
        zeros = id0_hits(hits)
        others = sorted({h["marker_id"] for h in detected if h["marker_id"] != 0})
        if others:
            other_id_frames.append({"frame_index": fm["frame_index"], "ids": others})
        if len(zeros) >= 2:
            duplicate_frames.append(fm["frame_index"])
        hit = zeros[0] if len(zeros) == 1 else None
        clipped = False
        sides = [None, None, None, None]
        aspect = None
        angle_dev = None
        opposite = None
        if hit is not None:
            corners = [hit["corner_%d" % k] for k in range(4)]
            for cx, cy in corners:
                if cx < CLIP_MARGIN_PX or cy < CLIP_MARGIN_PX or cx > img_w - CLIP_MARGIN_PX or cy > img_h - CLIP_MARGIN_PX:
                    clipped = True
            sides, angles, aspect, opposite = corner_metrics(hit, np)
            angle_dev = max(abs(a - 90.0) for a in angles)
            primary_series.append(hit)
        else:
            primary_series.append({
                "frame_index": fm["frame_index"], "timestamp": ts,
                "marker_detected": False, "marker_id": 0,
                "id0_instance_count": len(zeros),
            })
        pc = forensics._phasecorr(frame0_static, forensics.crop(gray, static_roi))
        background_series.append({"marker_detected": True, "center_x": pc["dx"], "center_y": pc["dy"], "timestamp": ts})
        detection_rows.append({
            "frame_index": fm["frame_index"],
            "timestamp_s": ts,
            "id0_detected": hit is not None,
            "id0_instance_count": len(zeros),
            "other_ids": " ".join(str(i) for i in others),
            "center_x": None if hit is None else hit["center_x"],
            "center_y": None if hit is None else hit["center_y"],
            "orientation_deg": None if hit is None else hit["orientation_deg"],
            "side_0_px": sides[0], "side_1_px": sides[1], "side_2_px": sides[2], "side_3_px": sides[3],
            "side_length_mean_px": None if hit is None else hit["quality"]["side_length_mean_px"],
            "aspect_ratio": aspect,
            "max_corner_angle_deviation_deg": angle_dev,
            "opposite_side_ratio_max": opposite,
            "clipped_near_border": clipped,
            "corner_0_x": None if hit is None else hit["corner_0"][0],
            "corner_0_y": None if hit is None else hit["corner_0"][1],
            "corner_1_x": None if hit is None else hit["corner_1"][0],
            "corner_1_y": None if hit is None else hit["corner_1"][1],
            "corner_2_x": None if hit is None else hit["corner_2"][0],
            "corner_2_y": None if hit is None else hit["corner_2"][1],
            "corner_3_x": None if hit is None else hit["corner_3"][0],
            "corner_3_y": None if hit is None else hit["corner_3"][1],
            "background_dx_px": pc["dx"],
            "background_dy_px": pc["dy"],
            "background_phasecorr_response": pc["response"],
        })

    flags = [r["id0_detected"] for r in detection_rows]
    detected_rows = [r for r in detection_rows if r["id0_detected"]]
    failed_indices = [r["frame_index"] for r in detection_rows if not r["id0_detected"]]
    detected_indices = [r["frame_index"] for r in detected_rows]
    detection_rate = sum(flags) / n_frames
    sides_mean = [r["side_length_mean_px"] for r in detected_rows]
    side_stats = mean_std_ptp(sides_mean) if sides_mean else None
    valid_corner = sum(1 for r in detection_rows if r["id0_detected"] and not r["clipped_near_border"]) / n_frames
    runs = loss_runs(flags)
    longest_loss = max((b - a + 1 for a, b in runs), default=0)
    pattern = loss_pattern(runs, n_frames)
    clip_count = sum(1 for r in detected_rows if r["clipped_near_border"])
    systematic_clipping = bool(detected_rows) and clip_count >= 0.5 * len(detected_rows)
    persistent_occlusion = longest_loss >= max(5, int(0.1 * n_frames))

    jitter = None
    if len(detected_rows) >= 2:
        jitter = {
            "center_x_px": mean_std_ptp([r["center_x"] for r in detected_rows]),
            "center_y_px": mean_std_ptp([r["center_y"] for r in detected_rows]),
            "orientation_deg": mean_std_ptp([r["orientation_deg"] for r in detected_rows]),
            "side_length_px": mean_std_ptp(sides_mean),
        }

    trajectory = traj.extract_trajectory(primary_series, background_series, baseline_frames=3)
    raw_disp = None
    if detected_rows:
        bx, by = detected_rows[0]["center_x"], detected_rows[0]["center_y"]
        raw_disp = mean_std_ptp([
            math.hypot(r["center_x"] - bx, r["center_y"] - by) for r in detected_rows])
    comp_disp = None
    if trajectory["valid"]:
        comp_values = [t for t in trajectory["translation"] if t is not None]
        if comp_values:
            comp_disp = mean_std_ptp(comp_values)

    # 진단용 ROI는 검출 프레임의 중앙값 위치. 실패 프레임에도 같은 박스를 쓴다.
    os.makedirs(FIG_DIR, exist_ok=True)
    os.makedirs(RAW_DET, exist_ok=True)
    os.makedirs(RAW_FAIL, exist_ok=True)
    os.makedirs(FIG_DIR, exist_ok=True)
    os.makedirs(RAW_DET, exist_ok=True)
    os.makedirs(RAW_FAIL, exist_ok=True)
    roi_box = None
    diag_rows = []
    if detected_rows:
        cx = float(np.median([r["center_x"] for r in detected_rows]))
        cy = float(np.median([r["center_y"] for r in detected_rows]))
        side = float(np.median(sides_mean))
        half = int(max(48, round(side * 2.2)))
        roi_box = {
            "center_x": cx, "center_y": cy, "half_px": half,
            "x0": max(0, int(round(cx)) - half), "y0": max(0, int(round(cy)) - half),
            "x1": min(img_w, int(round(cx)) + half), "y1": min(img_h, int(round(cy)) + half),
        }
        by_index = {r["frame_index"]: r for r in detection_rows}
        sample_detected = evenly(detected_indices, SAMPLE_COUNT)
        sample_failed = evenly(failed_indices, SAMPLE_COUNT)
        longest = max(runs, key=lambda run: run[1] - run[0]) if runs else None
        loss_sample = []
        if longest is not None:
            loss_sample = list(range(longest[0], min(longest[1] + 1, longest[0] + SAMPLE_COUNT)))
        wanted = {}
        for idx in sample_detected:
            wanted.setdefault(idx, set()).add("detected_sample")
        for idx in sample_failed:
            wanted.setdefault(idx, set()).add("failed_sample")
        for idx in loss_sample:
            wanted.setdefault(idx, set()).add("longest_loss_run")
        for fm in meta:
            image = cv2.imread(os.path.join(CAPTURE_DIR, fm["rgb_filename"]))
            crop = image[roi_box["y0"]:roi_box["y1"], roi_box["x0"]:roi_box["x1"]]
            stats = roi_stats(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2, np)
            row = by_index[fm["frame_index"]]
            stats.update({
                "frame_index": fm["frame_index"],
                "timestamp_s": fm["elapsed_s_since_start"],
                "id0_detected": row["id0_detected"],
                "in_saved_sample": fm["frame_index"] in wanted,
            })
            diag_rows.append(stats)
            if fm["frame_index"] not in wanted:
                continue
            roles = wanted[fm["frame_index"]]
            if "detected_sample" in roles:
                cv2.imwrite(os.path.join(RAW_DET, fm["rgb_filename"]), image)
                cv2.imwrite(os.path.join(RAW_DET, "crop-" + fm["rgb_filename"]), crop)
            if "failed_sample" in roles or "longest_loss_run" in roles:
                cv2.imwrite(os.path.join(RAW_FAIL, fm["rgb_filename"]), image)
                cv2.imwrite(os.path.join(RAW_FAIL, "crop-" + fm["rgb_filename"]), crop)

    def group_stats(key, detected_flag):
        vals = [r[key] for r in diag_rows if r["id0_detected"] is detected_flag]
        return mean_std_ptp(vals) if vals else None

    sharpness = {
        "detected": group_stats("laplacian_variance", True),
        "failed": group_stats("laplacian_variance", False),
    }
    intensity = {
        "detected": {k: group_stats(k, True) for k in (
            "mean_intensity", "std_intensity", "p10_intensity", "p90_intensity",
            "contrast_p90_p10", "near_black_ratio", "near_white_ratio")},
        "failed": {k: group_stats(k, False) for k in (
            "mean_intensity", "std_intensity", "p10_intensity", "p90_intensity",
            "contrast_p90_p10", "near_black_ratio", "near_white_ratio")},
    }
    geometry = None
    if detected_rows:
        geometry = {
            "aspect_ratio": mean_std_ptp([r["aspect_ratio"] for r in detected_rows]),
            "max_corner_angle_deviation_deg": mean_std_ptp([r["max_corner_angle_deviation_deg"] for r in detected_rows]),
            "opposite_side_ratio_max": mean_std_ptp([r["opposite_side_ratio_max"] for r in detected_rows]),
        }

    variant_summary = {}
    for kind in ("no_corner_refine", "adaptive_wide", "adaptive_narrow"):
        try:
            variant = variant_detector(cv2, kind)
        except Exception as exc:
            variant_summary[kind] = {"error": str(exc)}
            continue
        hit_frames = 0
        multi = 0
        for fm in meta:
            image = cv2.imread(os.path.join(CAPTURE_DIR, fm["rgb_filename"]))
            hits = det.detect_markers(image, variant, frame_index=fm["frame_index"])
            zeros = id0_hits(hits)
            if zeros:
                hit_frames += 1
            if len(zeros) >= 2:
                multi += 1
        variant_summary[kind] = {
            "id0_frames": hit_frames,
            "detection_rate": hit_frames / n_frames,
            "frames_with_two_or_more_id0": multi,
            "used_for_qualification": False,
        }

    causes = []
    if detection_rate < DETECTION_RATE_ACCEPTANCE:
        causes.append("INSUFFICIENT_DETECTION_RATE")
    if side_stats is not None and side_stats["mean"] < MEAN_SIDE_PX_ACCEPTANCE:
        causes.append("INSUFFICIENT_IMAGE_FOOTPRINT")
    if geometry is not None and (
            geometry["aspect_ratio"]["median"] > 1.35
            or geometry["max_corner_angle_deviation_deg"]["median"] > 25):
        causes.append("VIEWING_GEOMETRY_LIMITATION")
    if sharpness["detected"] and sharpness["failed"]:
        if sharpness["failed"]["median"] < 0.7 * sharpness["detected"]["median"]:
            causes.append("IMAGE_SHARPNESS_LIMITATION")
    det_contrast = intensity["detected"]["contrast_p90_p10"]
    fail_contrast = intensity["failed"]["contrast_p90_p10"]
    if det_contrast and fail_contrast and fail_contrast["median"] < 0.7 * det_contrast["median"]:
        causes.append("CONTRAST_OR_ILLUMINATION_LIMITATION")
    if systematic_clipping or persistent_occlusion:
        causes.append("OCCLUSION_OR_SURFACE_LIMITATION")
    variant_rates = [v.get("detection_rate", 0) for v in variant_summary.values() if "detection_rate" in v]
    if variant_rates and max(variant_rates) - detection_rate >= 0.15:
        causes.append("DETECTOR_CONFIGURATION_LIMITATION")
    if "INSUFFICIENT_DETECTION_RATE" in causes and "INSUFFICIENT_IMAGE_FOOTPRINT" not in causes:
        specific = [c for c in causes if c != "INSUFFICIENT_DETECTION_RATE"]
        if not specific:
            causes.append("CAUSE_UNRESOLVED")

    acceptance_fail = []
    if not baseline["unchanged"]:
        acceptance_fail.append("BASELINE_B_CHANGED")
    if n_frames < 200 or duration_s < 20:
        acceptance_fail.append("CAPTURE_TOO_SHORT")
    if duplicate_frames:
        acceptance_fail.append("DUPLICATE_ID0_DECODED")
    if detection_rate < DETECTION_RATE_ACCEPTANCE:
        acceptance_fail.append("DETECTION_RATE_BELOW_95_PCT")
    if side_stats is None or side_stats["mean"] < MEAN_SIDE_PX_ACCEPTANCE:
        acceptance_fail.append("MEAN_SIDE_BELOW_35_PX")
    if valid_corner < VALID_FOUR_CORNER_ACCEPTANCE:
        acceptance_fail.append("VALID_FOUR_CORNER_BELOW_95_PCT")
    if systematic_clipping:
        acceptance_fail.append("SYSTEMATIC_CLIPPING")
    if persistent_occlusion:
        acceptance_fail.append("PERSISTENT_OCCLUSION")
    if jitter is None:
        acceptance_fail.append("STATIC_JITTER_NOT_CHARACTERIZED")
    if not trajectory["valid"]:
        acceptance_fail.append("BACKGROUND_COMPENSATION_NOT_OPERATIONAL")

    if "BASELINE_B_CHANGED" in acceptance_fail or "CAPTURE_TOO_SHORT" in acceptance_fail:
        verdict = "INCONCLUSIVE"
    elif not acceptance_fail:
        verdict = "PASS"
    else:
        verdict = "FAIL"

    os.makedirs(FIG_DIR, exist_ok=True)
    os.makedirs(RAW_DET, exist_ok=True)
    os.makedirs(RAW_FAIL, exist_ok=True)

    fieldnames = list(detection_rows[0].keys())
    with open(os.path.join(OUT_DIR, "marker_detection.csv"), "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(detection_rows)

    sample_diag = [r for r in diag_rows if r.get("in_saved_sample")]
    if sample_diag:
        with open(os.path.join(OUT_DIR, "failed_frame_diagnostics.csv"), "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(sample_diag[0].keys()))
            writer.writeheader()
            writer.writerows(sample_diag)

    with open(os.path.join(OUT_DIR, "static_jitter.csv"), "w", newline="", encoding="utf-8") as handle:
        jitter_fields = ["frame_index", "timestamp_s", "center_x", "center_y", "orientation_deg",
                         "side_length_mean_px", "background_dx_px", "background_dy_px"]
        writer = csv.DictWriter(handle, fieldnames=jitter_fields)
        writer.writeheader()
        for row in detected_rows:
            writer.writerow({key: row[key] for key in jitter_fields})

    color_files = [fm["rgb_filename"] for fm in meta]
    hash_of_hashes = hashlib.sha256()
    for name in color_files:
        hash_of_hashes.update(bytes.fromhex(sha256(os.path.join(CAPTURE_DIR, name))))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if detected_rows:
        overlay_idx = detected_indices[len(detected_indices) // 2]
        overlay = cv2.imread(os.path.join(CAPTURE_DIR, meta[overlay_idx]["rgb_filename"])).copy()
        row = detection_rows[overlay_idx]
        corners = np.array([
            [row["corner_0_x"], row["corner_0_y"]], [row["corner_1_x"], row["corner_1_y"]],
            [row["corner_2_x"], row["corner_2_y"]], [row["corner_3_x"], row["corner_3_y"]],
        ], dtype=np.int32)
        cv2.polylines(overlay, [corners], True, (0, 255, 0), 2)
        cv2.putText(overlay, "ID 0", tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(FIG_DIR, "marker_overlay_detected.png"), overlay)
    if failed_indices and roi_box is not None:
        fail_idx = failed_indices[len(failed_indices) // 2]
        fail_img = cv2.imread(os.path.join(CAPTURE_DIR, meta[fail_idx]["rgb_filename"])).copy()
        cv2.rectangle(fail_img, (roi_box["x0"], roi_box["y0"]), (roi_box["x1"], roi_box["y1"]), (0, 0, 255), 2)
        cv2.putText(fail_img, "ID0 not detected", (roi_box["x0"], max(20, roi_box["y0"] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(FIG_DIR, "marker_overlay_failed.png"), fail_img)

    fig, ax = plt.subplots(figsize=(9, 3.2))
    ax.step([r["timestamp_s"] for r in detection_rows], [1 if r["id0_detected"] else 0 for r in detection_rows], where="post")
    ax.set_ylim(-0.1, 1.2)
    ax.set_xlabel("elapsed time (s)")
    ax.set_ylabel("ID0 detected")
    ax.set_title("Production detector timeline")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "detection_timeline.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    if detected_rows:
        ax.plot([r["center_x"] for r in detected_rows], [r["center_y"] for r in detected_rows], ".", markersize=4)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("center_x (px)")
    ax.set_ylabel("center_y (px)")
    ax.set_title("Detected-frame centers only")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "marker_trajectory.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 3.2))
    if detected_rows:
        ax.plot([r["timestamp_s"] for r in detected_rows], [r["side_length_mean_px"] for r in detected_rows], ".")
    ax.axhline(MEAN_SIDE_PX_ACCEPTANCE, color="gray", linewidth=1)
    ax.set_xlabel("elapsed time (s)")
    ax.set_ylabel("mean side (px)")
    ax.set_title("Detected frames only, no interpolation")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "marker_side_length.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.5))
    if sharpness["detected"] and sharpness["failed"]:
        ax.bar(["detected", "failed"], [sharpness["detected"]["median"], sharpness["failed"]["median"]])
    ax.set_ylabel("median Laplacian variance")
    ax.set_title("Fixed M2 ROI sharpness")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "sharpness_comparison.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.5))
    if det_contrast and fail_contrast:
        ax.bar(["detected", "failed"], [det_contrast["median"], fail_contrast["median"]])
    ax.set_ylabel("median p90-p10")
    ax.set_title("Fixed M2 ROI contrast")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "intensity_contrast_comparison.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 3.6))
    if detected_rows:
        ax.plot([r["timestamp_s"] for r in detected_rows],
                [math.hypot(r["center_x"] - detected_rows[0]["center_x"], r["center_y"] - detected_rows[0]["center_y"])
                 for r in detected_rows], ".", label="raw displacement", color="tab:red")
    if trajectory["valid"]:
        ts = [fm["elapsed_s_since_start"] for fm, value in zip(meta, trajectory["translation"]) if value is not None]
        ax.plot(ts, [value for value in trajectory["translation"] if value is not None], ".",
                label="background-compensated", color="tab:blue")
    ax.legend()
    ax.set_xlabel("elapsed time (s)")
    ax.set_ylabel("displacement (px)")
    ax.set_title("Detected frames only")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "background_compensation.png"), dpi=130)
    plt.close(fig)

    precheck = None
    precheck_meta_path = os.path.join(PRECHECK_DIR, "frames_meta.json")
    if os.path.isfile(precheck_meta_path):
        pre_meta = json.load(open(precheck_meta_path, encoding="utf-8"))
        pre_hits = 0
        pre_multi = 0
        pre_centers = []
        for fm in pre_meta:
            image = cv2.imread(os.path.join(PRECHECK_DIR, fm["rgb_filename"]))
            zeros = id0_hits(det.detect_markers(image, production, frame_index=fm["frame_index"]))
            if len(zeros) >= 2:
                pre_multi += 1
            if len(zeros) == 1:
                pre_hits += 1
                pre_centers.append({
                    "frame_index": fm["frame_index"],
                    "center_x": zeros[0]["center_x"],
                    "center_y": zeros[0]["center_y"],
                    "side_length_mean_px": zeros[0]["quality"]["side_length_mean_px"],
                })
        precheck = {
            "frame_count": len(pre_meta),
            "duration_s": pre_meta[-1]["elapsed_s_since_start"],
            "detector_id0_frames": pre_hits,
            "detector_frames_with_two_or_more_id0": pre_multi,
            "detector_id0_centers": pre_centers,
        }

    qualification = {
        "gate": "P-007F-07B-R1",
        "attempt": 2,
        "MARKER_STATIC_QUALIFICATION_R1": verdict,
        "acceptance_fail": acceptance_fail,
        "root_cause_classification": causes,
        "cause_rules_note": "diagnostic detector rates are not acceptance. size below 35px is the only footprint cause.",
        "n_frames": n_frames,
        "duration_s": duration_s,
        "fps": n_frames / duration_s,
        "resolution": {"width": img_w, "height": img_h},
        "detected_frames": len(detected_rows),
        "detection_rate": detection_rate,
        "detection_rate_acceptance": DETECTION_RATE_ACCEPTANCE,
        "side_px": side_stats,
        "side_mean_acceptance_px": MEAN_SIDE_PX_ACCEPTANCE,
        "valid_four_corner_rate": valid_corner,
        "valid_four_corner_acceptance": VALID_FOUR_CORNER_ACCEPTANCE,
        "valid_four_corner_definition": "ID0 detected and no corner within 3 px of the frame border, divided by all frames",
        "longest_detection_loss_run": longest_loss,
        "loss_run_count": len(runs),
        "loss_pattern": pattern,
        "duplicate_id0_frames": duplicate_frames,
        "other_marker_ids": other_id_frames[:20],
        "clip_count": clip_count,
        "systematic_clipping": systematic_clipping,
        "persistent_occlusion": persistent_occlusion,
        "static_jitter_detected_frames_only": jitter,
        "raw_displacement_detected_frames_only": raw_disp,
        "background_compensation_operational": trajectory["valid"],
        "background_compensation_reason": None if trajectory["valid"] else trajectory.get("reason"),
        "background_compensated_displacement_px": comp_disp,
        "geometry_detected_frames_only": geometry,
        "sharpness_fixed_roi": {"detected_median": None if not sharpness["detected"] else sharpness["detected"]["median"],
                                "failed_median": None if not sharpness["failed"] else sharpness["failed"]["median"],
                                "detected": sharpness["detected"], "failed": sharpness["failed"]},
        "intensity_fixed_roi": intensity,
        "roi_box": roi_box,
        "diagnostic_detector_variants": variant_summary,
        "production_detector": "scripts/p007f_07_marker_detector.py create_detector",
        "baseline_b_check": baseline,
        "precheck_detector": precheck,
        "command_event_count": len(command_events.get("events", [])),
        "robot_command_count": 0,
        "servo_write_count": 0,
        "torque_command_count": 0,
        "READY_FOR_MOTION_PREREGISTRATION": "NO",
        "MOTION_AUTHORIZED": "NO",
        "local_capture_dir": CAPTURE_DIR,
        "color_frame_count": len(color_files),
        "color_sha256_of_sha256": hash_of_hashes.hexdigest(),
        "depth_frames_location": "/home/woonkil/cad-less-robotics/.cache/p007f07b_r1/attempt2_run on p620; not copied into git",
    }
    with open(os.path.join(OUT_DIR, "qualification.json"), "w", encoding="utf-8") as handle:
        json.dump(qualification, handle, ensure_ascii=False, indent=1)

    metadata = {
        "camera": "Gemini 335L",
        "serial": wiring.GEMINI_EXPECTED_SERIAL,
        "firmware": wiring.GEMINI_EXPECTED_FIRMWARE,
        "launch": "ros2 launch orbbec_camera gemini_330_series.launch.py",
        "settings_changed": False,
        "frame_count": n_frames,
        "duration_s": duration_s,
        "resolution": {"width": img_w, "height": img_h},
        "baseline_b_check": baseline,
        "robot_command_count": 0,
        "servo_write_count": 0,
        "torque_command_count": 0,
        "local_capture_dir": CAPTURE_DIR,
        "color_sha256_of_sha256": hash_of_hashes.hexdigest(),
    }
    with open(os.path.join(OUT_DIR, "capture_metadata.json"), "w", encoding="utf-8") as handle:
        json.dump(metadata, handle, ensure_ascii=False, indent=1)
    print(json.dumps({
        "verdict": verdict,
        "detected": len(detected_rows),
        "n": n_frames,
        "rate": detection_rate,
        "side_mean": None if side_stats is None else side_stats["mean"],
        "longest_loss": longest_loss,
        "pattern": pattern,
        "causes": causes,
        "acceptance_fail": acceptance_fail,
        "variants": {k: v.get("detection_rate") for k, v in variant_summary.items()},
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
