#!/usr/bin/env python3
"""P-007F-07B Phase B4~B9 -- M2 50mm marker static qualification analysis.

입력: scripts/p007f_07b_continuous_capture_runner.py 가 p620 에서 실제로
생성한 연속촬영(.cache/p007f07b_capture_full/run1/, 117 frame, ~12s, 로봇/서보
command 0)을 읽기만 한다. 이 스크립트 자체는 카메라/로봇 API를 전혀 참조하지
않는다(AST 자기검증 대상).

재사용(SHA-pin, 전부 수정 없음):
  - scripts/p007f_07_marker_detector.py  -- create_detector/detect_markers_by_id
  - scripts/p007f_07_trajectory_classifier.py -- extract_trajectory (background 보정)
  - scripts/p007f_06_forensics.py -- crop/_phasecorr/ROIS["STATIC_REFERENCE_ROI"]
  - scripts/p007f_05_hardware_wiring.py -- baseline_b_check
"""
import csv
import importlib.util
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAPTURE_DIR = os.environ.get(
    "P007F07B_CAPTURE_DIR",
    os.path.join(ROOT, ".cache", "p007f07b_capture_full", "run1"))
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_static_qualification")
FIG_DIR = os.path.join(OUT_DIR, "figures")
RAW_DIR = os.path.join(OUT_DIR, "raw")

MARKER_DETECTOR_PATH = os.path.join(ROOT, "scripts", "p007f_07_marker_detector.py")
TRAJ_CLASSIFIER_PATH = os.path.join(ROOT, "scripts", "p007f_07_trajectory_classifier.py")
FORENSICS_PATH = os.path.join(ROOT, "scripts", "p007f_06_forensics.py")
WIRING_PATH = os.path.join(ROOT, "scripts", "p007f_05_hardware_wiring.py")

EXPECTED_MARKER_ID = 0
DETECTION_RATE_ACCEPTANCE = 0.95
MEAN_SIDE_PX_ACCEPTANCE = 35.0
VALID_FOUR_CORNER_ACCEPTANCE = 0.95
CLIP_MARGIN_PX = 3  # corner 가 frame 경계에서 이 거리 이내면 clipping 의심으로 표시
REPRESENTATIVE_FRAME_STRIDE = 10


def self_check_no_hardware_import():
    import ast
    with open(os.path.abspath(__file__), "r", encoding="utf-8") as f:
        tree = ast.parse(f.read())
    forbidden = {"rclpy", "Arm_Lib", "p007f_05_jetson_controller", "p007f_07_jetson_controller", "subprocess"}
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.add(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module.split(".")[0])
    return forbidden.intersection(imported)


def sha256(path):
    import hashlib
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mean_std_ptp(values):
    import numpy as np
    arr = np.asarray(values, dtype=float)
    return {"mean": float(arr.mean()), "std": float(arr.std()),
           "peak_to_peak": float(arr.max() - arr.min()),
           "median": float(np.median(arr)),
           "mad": float(np.median(np.abs(arr - np.median(arr))))}


def main():
    forbidden = self_check_no_hardware_import()
    if forbidden:
        print(json.dumps({"status": "ABORT", "reason": "FORBIDDEN_IMPORT", "found": list(forbidden)}))
        sys.exit(1)

    shas = {name: sha256(p) for name, p in (
        ("marker_detector", MARKER_DETECTOR_PATH), ("trajectory_classifier", TRAJ_CLASSIFIER_PATH),
        ("forensics", FORENSICS_PATH), ("wiring", WIRING_PATH))}
    det = _load("p007f_07_marker_detector_pin", MARKER_DETECTOR_PATH)
    traj = _load("p007f_07_trajectory_classifier_pin", TRAJ_CLASSIFIER_PATH)
    forensics = _load("p007f_06_forensics_pin", FORENSICS_PATH)
    wiring = _load("p007f_05_hardware_wiring_pin", WIRING_PATH)

    cv2, np = det.cv2, det.np

    frames_meta = json.load(open(os.path.join(CAPTURE_DIR, "frames_meta.json"), encoding="utf-8"))
    n_frames = len(frames_meta)
    assert n_frames >= 1

    detector = det.create_detector()
    static_roi = forensics.ROIS["STATIC_REFERENCE_ROI"]

    color_cache = {}

    def load_color(idx):
        if idx not in color_cache:
            color_cache[idx] = cv2.imread(os.path.join(CAPTURE_DIR, frames_meta[idx]["rgb_filename"]))
        return color_cache[idx]

    frame0_gray_full = cv2.cvtColor(load_color(0), cv2.COLOR_BGR2GRAY)
    frame0_static_crop = forensics.crop(frame0_gray_full, static_roi)
    img_h, img_w = frame0_gray_full.shape[:2]

    primary_series = []
    background_series = []
    detection_rows = []

    for i, fm in enumerate(frames_meta):
        color = load_color(i)
        ts = fm["elapsed_s_since_start"]
        # 🔴 detect_markers_by_id() 의 not-found fallback dict는 raw detect_markers()
        # 가 돌려주는 num_rejected_candidates 를 보존하지 않는다(그 함수 자체 설계)
        # -- Phase B8 corner_rejection_count 를 실제로 쓰려면 raw detect_markers()
        # 를 직접 호출해 ID0 hit을 내가 골라야 한다(detect_markers_by_id와 동일
        # lookup 로직을 그대로 재현, 단지 폐기되는 필드만 보존).
        all_hits = det.detect_markers(color, detector=detector, timestamp=ts, frame_index=i)
        by_id = {h["marker_id"]: h for h in all_hits if h.get("marker_detected")}
        hit = by_id.get(EXPECTED_MARKER_ID)
        if hit is None:
            if all_hits[0].get("marker_detected") is False:
                # 프레임 전체에서 아무 marker 도 검출 안 됨 -- 이 sentinel record가
                # 진짜 num_rejected_candidates 를 갖고 있다
                hit = dict(all_hits[0])
                hit["marker_id"] = EXPECTED_MARKER_ID
            else:
                # 다른 ID는 검출됐지만 ID0는 아님(이 실험에선 발생해선 안 되는
                # 경우지만, 조립식으로 거짓 0을 채우지 않고 None으로 명시)
                hit = {"frame_index": i, "timestamp": ts, "marker_detected": False,
                      "marker_id": EXPECTED_MARKER_ID, "num_rejected_candidates": None,
                      "other_ids_detected_this_frame": sorted(by_id.keys())}
        primary_series.append(hit)

        clipped = False
        side_mean_px = None
        if hit["marker_detected"]:
            corners = [hit["corner_0"], hit["corner_1"], hit["corner_2"], hit["corner_3"]]
            for cx, cy in corners:
                if cx < CLIP_MARGIN_PX or cx > img_w - CLIP_MARGIN_PX or cy < CLIP_MARGIN_PX or cy > img_h - CLIP_MARGIN_PX:
                    clipped = True
            side_mean_px = hit["quality"]["side_length_mean_px"]

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        pc = forensics._phasecorr(frame0_static_crop, forensics.crop(gray, static_roi))
        background_series.append({"marker_detected": True, "center_x": pc["dx"], "center_y": pc["dy"],
                                  "timestamp": ts})

        detection_rows.append({
            "frame_index": i, "timestamp_s": ts, "marker_detected": hit["marker_detected"],
            "marker_id": hit.get("marker_id"), "center_x": hit.get("center_x"), "center_y": hit.get("center_y"),
            "orientation_deg": hit.get("orientation_deg"), "side_length_mean_px": side_mean_px,
            "clipped_near_border": clipped,
            "background_dx_px": pc["dx"], "background_dy_px": pc["dy"],
            "background_phasecorr_response": pc["response"],
        })

    # -- Phase B4: detection rate / footprint / four-corner integrity --
    detected_flags = [r["marker_detected"] for r in detection_rows]
    detection_rate = sum(detected_flags) / n_frames
    side_lengths = [r["side_length_mean_px"] for r in detection_rows if r["marker_detected"]]
    valid_corner_flags = [r["marker_detected"] and not r["clipped_near_border"] for r in detection_rows]
    valid_corner_rate = sum(valid_corner_flags) / n_frames

    footprint_stats = None
    if side_lengths:
        footprint_stats = {"mean_px": float(np.mean(side_lengths)), "min_px": float(np.min(side_lengths)),
                           "max_px": float(np.max(side_lengths))}

    # -- Phase B5: static jitter (raw, pre background compensation) --
    detected_rows = [r for r in detection_rows if r["marker_detected"]]
    jitter_stats = None
    if len(detected_rows) >= 2:
        cxs = [r["center_x"] for r in detected_rows]
        cys = [r["center_y"] for r in detected_rows]
        thetas = [r["orientation_deg"] for r in detected_rows]
        jitter_stats = {"center_x_px": mean_std_ptp(cxs), "center_y_px": mean_std_ptp(cys),
                        "orientation_deg": mean_std_ptp(thetas)}

    # -- Phase B6: background compensation (raw vs compensated) --
    # 🔴 extract_trajectory()의 공식 baseline(기본 처음 3 frame)이 전부 marker
    # 미검출이면(이 실측 데이터에서 실제로 발생) {"valid": False} 를 돌려준다 --
    # 이를 silent 하게 우회하지 않는다. raw displacement 는 이 공식 함수와 무관하게
    # "첫 검출 frame"을 기준으로 별도 계산해(Phase B6이 요구하는 raw 값 자체는
    # baseline 성립 여부와 무관), compensated 는 공식 함수가 valid 일 때만 보고한다.
    trajectory = traj.extract_trajectory(primary_series, background_series, baseline_frames=3)
    first_detected_idx = next((i for i, r in enumerate(primary_series) if r["marker_detected"]), None)
    raw_disp_stats = None
    raw_disp_baseline_note = None
    if first_detected_idx is not None:
        rb_x = primary_series[first_detected_idx]["center_x"]
        rb_y = primary_series[first_detected_idx]["center_y"]
        if first_detected_idx == 0:
            raw_disp_baseline_note = "first detected frame used as baseline (frame_index=0)"
        else:
            raw_disp_baseline_note = (
                "first detected frame used as baseline (frame_index=%d); frames before it were undetected"
                % first_detected_idx)
        raw_disp = [math.hypot(r["center_x"] - rb_x, r["center_y"] - rb_y)
                   for r in primary_series if r["marker_detected"]]
        if raw_disp:
            raw_disp_stats = mean_std_ptp(raw_disp)
    comp_disp_stats = None
    if trajectory["valid"]:
        comp_disp = [t for t in trajectory["translation"] if t is not None]
        if comp_disp:
            comp_disp_stats = mean_std_ptp(comp_disp)

    # -- Phase B8: occlusion / loss --
    loss_count = n_frames - sum(detected_flags)
    max_consecutive_loss = 0
    cur = 0
    for f in detected_flags:
        cur = 0 if f else cur + 1
        max_consecutive_loss = max(max_consecutive_loss, cur)
    clip_count = sum(1 for r in detection_rows if r["marker_detected"] and r["clipped_near_border"])
    corner_rejection_count = sum(r["num_rejected_candidates"] for r in primary_series
                                 if not r["marker_detected"] and r.get("num_rejected_candidates") is not None)
    systematic_clipping = clip_count > 0 and clip_count >= 0.5 * sum(detected_flags) if any(detected_flags) else False
    persistent_occlusion = max_consecutive_loss >= max(5, int(0.1 * n_frames))

    # -- Baseline B invariant (Phase B3) -- frame0 vs frozen P-007F-03 reference
    baseline_check = wiring.baseline_b_check(os.path.join(CAPTURE_DIR, frames_meta[0]["rgb_filename"]))

    # -- verdict --
    reasons = []
    if not baseline_check["unchanged"]:
        reasons.append("BASELINE_B_CHANGED")
    if detection_rate < DETECTION_RATE_ACCEPTANCE:
        reasons.append("DETECTION_RATE_BELOW_%.0f_PCT" % (DETECTION_RATE_ACCEPTANCE * 100))
    if footprint_stats is None:
        reasons.append("NO_DETECTED_FRAMES_FOR_FOOTPRINT")
    elif footprint_stats["mean_px"] < MEAN_SIDE_PX_ACCEPTANCE:
        reasons.append("MARKER_TOO_SMALL_IN_IMAGE")
    if valid_corner_rate < VALID_FOUR_CORNER_ACCEPTANCE:
        reasons.append("VALID_FOUR_CORNER_RATE_BELOW_%.0f_PCT" % (VALID_FOUR_CORNER_ACCEPTANCE * 100))
    if systematic_clipping:
        reasons.append("SYSTEMATIC_CLIPPING")
    if persistent_occlusion:
        reasons.append("PERSISTENT_OCCLUSION")
    if jitter_stats is None:
        reasons.append("STATIC_JITTER_NOT_MEASURABLE")
    if not trajectory["valid"]:
        reasons.append("BACKGROUND_COMPENSATION_NOT_OPERATIONAL: %s" % trajectory.get("reason"))

    if not reasons:
        verdict = "PASS"
    elif jitter_stats is None or footprint_stats is None or not trajectory["valid"]:
        verdict = "INCONCLUSIVE"
    else:
        verdict = "FAIL"

    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(FIG_DIR, exist_ok=True)
    os.makedirs(RAW_DIR, exist_ok=True)

    # -- CSVs --
    with open(os.path.join(OUT_DIR, "marker_detection.csv"), "w", newline="", encoding="utf-8") as f:
        wtr = csv.DictWriter(f, fieldnames=list(detection_rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(detection_rows)

    jitter_rows = []
    for i, (r, t_val, dx_val, dy_val) in enumerate(zip(detection_rows, trajectory.get("translation", [None] * n_frames),
                                                        trajectory.get("dx", [None] * n_frames),
                                                        trajectory.get("dy", [None] * n_frames))):
        jitter_rows.append({"frame_index": r["frame_index"], "timestamp_s": r["timestamp_s"],
                           "marker_detected": r["marker_detected"], "center_x_px": r["center_x"],
                           "center_y_px": r["center_y"], "orientation_deg": r["orientation_deg"],
                           "background_dx_px": r["background_dx_px"], "background_dy_px": r["background_dy_px"],
                           "compensated_dx_px": dx_val, "compensated_dy_px": dy_val,
                           "compensated_translation_px": t_val})
    with open(os.path.join(OUT_DIR, "static_jitter.csv"), "w", newline="", encoding="utf-8") as f:
        wtr = csv.DictWriter(f, fieldnames=list(jitter_rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(jitter_rows)

    # -- representative raw frames --
    rep_indices = list(range(0, n_frames, REPRESENTATIVE_FRAME_STRIDE))
    if (n_frames - 1) not in rep_indices:
        rep_indices.append(n_frames - 1)
    for idx in rep_indices:
        cv2.imwrite(os.path.join(RAW_DIR, frames_meta[idx]["rgb_filename"]), load_color(idx))

    # -- figures --
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # marker_overlay.png -- representative frame with detected corners/ID drawn
    overlay_idx = next((i for i, r in enumerate(detection_rows) if r["marker_detected"]), 0)
    overlay_img = load_color(overlay_idx).copy()
    if primary_series[overlay_idx]["marker_detected"]:
        c = np.array([primary_series[overlay_idx]["corner_%d" % k] for k in range(4)], dtype=np.int32)
        cv2.polylines(overlay_img, [c], True, (0, 255, 0), 2)
        cv2.putText(overlay_img, "ID %d" % EXPECTED_MARKER_ID, tuple(c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                   (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imwrite(os.path.join(FIG_DIR, "marker_overlay.png"), overlay_img)

    # marker_trajectory.png -- cx,cy path over time
    fig, ax = plt.subplots(figsize=(6, 6))
    if detected_rows:
        ax.plot([r["center_x"] for r in detected_rows], [r["center_y"] for r in detected_rows], "o-",
               markersize=3, linewidth=0.8, color="tab:blue")
        ax.scatter([detected_rows[0]["center_x"]], [detected_rows[0]["center_y"]], color="green", s=60, label="start", zorder=5)
        ax.scatter([detected_rows[-1]["center_x"]], [detected_rows[-1]["center_y"]], color="red", s=60, label="end", zorder=5)
    ax.set_xlabel("center_x (px)"); ax.set_ylabel("center_y (px)")
    ax.set_title("Marker center trajectory (static, robot stationary)\nn_detected=%d/%d" % (len(detected_rows), n_frames))
    ax.legend(); ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout(); fig.savefig(os.path.join(FIG_DIR, "marker_trajectory.png"), dpi=130); plt.close(fig)

    # marker_jitter.png -- time series of center_x/center_y/orientation
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    ts_list = [r["timestamp_s"] for r in detected_rows]
    axes[0].plot(ts_list, [r["center_x"] for r in detected_rows], ".-", color="tab:blue")
    axes[0].set_ylabel("center_x (px)")
    axes[1].plot(ts_list, [r["center_y"] for r in detected_rows], ".-", color="tab:orange")
    axes[1].set_ylabel("center_y (px)")
    axes[2].plot(ts_list, [r["orientation_deg"] for r in detected_rows], ".-", color="tab:green")
    axes[2].set_ylabel("orientation (deg)"); axes[2].set_xlabel("elapsed time (s)")
    fig.suptitle("Static jitter time series (robot + Gemini both stationary)")
    fig.tight_layout(); fig.savefig(os.path.join(FIG_DIR, "marker_jitter.png"), dpi=130); plt.close(fig)

    # background_compensation.png -- raw vs compensated displacement
    fig, ax = plt.subplots(figsize=(9, 4.5))
    if first_detected_idx is not None:
        rb_x = primary_series[first_detected_idx]["center_x"]
        rb_y = primary_series[first_detected_idx]["center_y"]
        raw_ts = [r["timestamp_s"] for r in detected_rows]
        raw_series = [math.hypot(r["center_x"] - rb_x, r["center_y"] - rb_y) for r in detected_rows]
        ax.plot(raw_ts, raw_series, ".-", label="raw displacement (px, baseline=first detected frame)",
               color="tab:red", alpha=0.7)
    if trajectory["valid"]:
        comp_series_ts = [fm["elapsed_s_since_start"] for fm, t in zip(frames_meta, trajectory["translation"]) if t is not None]
        comp_series = [t for t in trajectory["translation"] if t is not None]
        ax.plot(comp_series_ts, comp_series, ".-", label="background-compensated displacement (px, official baseline_frames=3)",
               color="tab:blue")
    else:
        ax.text(0.5, 0.5, "background-compensated series UNAVAILABLE\n(extract_trajectory baseline_frames=3 all undetected: %s)" % trajectory.get("reason"),
               transform=ax.transAxes, ha="center", va="center", fontsize=9, color="firebrick")
    ax.set_xlabel("elapsed time (s)"); ax.set_ylabel("displacement from baseline (px)")
    ax.set_title("Raw vs background-compensated marker displacement (static)")
    ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(FIG_DIR, "background_compensation.png"), dpi=130); plt.close(fig)

    # -- capture_metadata.json --
    capture_metadata = {
        "camera": "Gemini 335L", "serial": wiring.GEMINI_EXPECTED_SERIAL, "firmware": wiring.GEMINI_EXPECTED_FIRMWARE,
        "capture_dir_source": CAPTURE_DIR,
        "frame_count": n_frames, "duration_s": frames_meta[-1]["elapsed_s_since_start"],
        "target_fps_hint": 10, "actual_mean_fps": n_frames / frames_meta[-1]["elapsed_s_since_start"],
        "resolution": {"width": img_w, "height": img_h},
        "baseline_b_check": baseline_check,
        "robot_command_count": 0, "servo_write_count": 0,
        "raw_frames_committed_to_git": "representative subset only (stride=%d, %d of %d frames). full sequence stays local at %s and is not committed" % (
            REPRESENTATIVE_FRAME_STRIDE, len(rep_indices), n_frames, CAPTURE_DIR),
    }
    with open(os.path.join(OUT_DIR, "capture_metadata.json"), "w", encoding="utf-8") as f:
        json.dump(capture_metadata, f, ensure_ascii=False, indent=1, default=repr)

    qualification = {
        "gate": "P-007F-07B Phase B4-B9",
        "marker_id": EXPECTED_MARKER_ID,
        "n_frames": n_frames,
        "duration_s": frames_meta[-1]["elapsed_s_since_start"],
        "detection_rate": detection_rate,
        "detection_rate_acceptance": DETECTION_RATE_ACCEPTANCE,
        "footprint_px": footprint_stats,
        "footprint_mean_acceptance_px": MEAN_SIDE_PX_ACCEPTANCE,
        "valid_four_corner_rate": valid_corner_rate,
        "valid_four_corner_acceptance": VALID_FOUR_CORNER_ACCEPTANCE,
        "static_jitter": jitter_stats,
        "raw_displacement_stats_px": raw_disp_stats,
        "raw_displacement_baseline_note": raw_disp_baseline_note,
        "background_compensated_displacement_stats_px": comp_disp_stats,
        "marker_loss_count": loss_count,
        "max_consecutive_loss": max_consecutive_loss,
        "clip_count": clip_count,
        "corner_rejection_count": corner_rejection_count,
        "systematic_clipping": systematic_clipping,
        "persistent_occlusion": persistent_occlusion,
        "background_compensation_operational": trajectory["valid"],
        "background_compensation_invalid_reason": None if trajectory["valid"] else trajectory.get("reason"),
        "baseline_b_unchanged": baseline_check["unchanged"],
        "fail_inconclusive_reasons": reasons,
        "MARKER_STATIC_QUALIFICATION": verdict,
        "module_shas": shas,
        "robot_command_count": 0,
        "servo_write_count": 0,
    }
    with open(os.path.join(OUT_DIR, "qualification.json"), "w", encoding="utf-8") as f:
        json.dump(qualification, f, ensure_ascii=False, indent=1, default=repr)

    print(json.dumps(qualification, ensure_ascii=False, indent=1, default=repr))


if __name__ == "__main__":
    main()
