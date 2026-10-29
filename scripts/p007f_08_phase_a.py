#!/usr/bin/env python3
"""P-007F-08 Phase A — 정지 sham 을 찍고, 고정된 공식으로만 임계값을 계산한다.

--prepare 는 촬영 전에 preregistration.json 만 쓴다.
--capture 는 Gemini 만 연다. 로봇 명령 함수는 import 하지 않는다.
--analyze 는 이미 받아 둔 프레임으로 표와 그림을 만든다.
"""
import argparse
import csv
import hashlib
import json
import os
import subprocess
import time

import cv2
import numpy as np

import p007f_08_tracker as tr

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-08", "phase_a")
FIG_DIR = os.path.join(OUT_DIR, "figures")
HISTORICAL = os.path.join(
    ROOT, "evidence", "P-007F-07", "marker_static_qualification_r1", "attempt-02",
    "raw", "representative_detected", "000128-color.png")
CACHE_REL = ".cache/p007f08_phase_a/sham"
FONT_PATH = "/System/Library/Fonts/Supplemental/AppleGothic.ttf"


def _write_json(name, payload):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, name), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=1)
        handle.write("\n")


def prepare():
    _write_json("preregistration.json", tr.preregistration_document())
    print("preregistration_written")


def _gray(path):
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("unreadable " + path)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), image


def _draw_box(image, roi, color, text):
    x, y, width, height = roi
    cv2.rectangle(image, (x, y), (x + width, y + height), color, 2)
    cv2.putText(image, text, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def _save_roi(color, roi, name, label):
    canvas = color.copy()
    _draw_box(canvas, roi, (0, 180, 255), label)
    x, y, width, height = roi
    inset = cv2.resize(color[y:y + height, x:x + width], (width * 3, height * 3), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(os.path.join(FIG_DIR, name), canvas)
    cv2.imwrite(os.path.join(FIG_DIR, name.replace(".png", "_crop.png")), inset)


def _stats_line(stats):
    if not stats or stats.get("n", 0) == 0:
        return "없음"
    return "n=%d mean=%.3f median=%.3f std=%.3f min=%.3f max=%.3f p95=%.3f" % (
        stats["n"], stats["mean"], stats["median"], stats["std"], stats["min"], stats["max"], stats["p95"])


def analyze(cache_dir, identity):
    os.makedirs(FIG_DIR, exist_ok=True)
    frozen = json.load(open(os.path.join(OUT_DIR, "preregistration.json"), encoding="utf-8"))
    current = tr.preregistration_document()
    if frozen["formula"] != current["formula"] or frozen["m2_roi"] != current["m2_roi"]:
        raise RuntimeError("preregistration 이 코드와 다르다. 촬영 후에 공식을 바꾸지 않는다.")

    meta = json.load(open(os.path.join(cache_dir, "frames_meta.json"), encoding="utf-8"))
    events = json.load(open(os.path.join(cache_dir, "command_events.json"), encoding="utf-8"))
    grays = []
    colors = []
    stamps = []
    hashes = []
    for row in meta:
        path = os.path.join(cache_dir, row["rgb_filename"])
        gray, color = _gray(path)
        grays.append(gray)
        colors.append(color)
        stamps.append(float(row["elapsed_s_since_start"]))
        digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
        hashes.append({"file": row["rgb_filename"], "sha256": digest})
    manifest_hash = hashlib.sha256("".join(item["sha256"] for item in hashes).encode()).hexdigest()

    duration_s = stamps[-1] if stamps else 0.0
    fps = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
    m2 = tr.track_roi(grays, tr.M2_ROI)
    static = tr.track_roi(grays, tr.STATIC_REFERENCE_ROI)
    m2_phase = tr.phase_series(grays, tr.M2_ROI)
    static_phase = tr.phase_series(grays, tr.STATIC_REFERENCE_ROI)
    rows = tr.combine(m2, static, m2_phase, static_phase, stamps)

    est, hold = tr.split_indices(len(rows))
    t_motion = tr.robust_threshold([rows[i]["relative_displacement"] for i in est], tr.MOTION_FLOOR_PX)
    t_static = tr.robust_threshold([rows[i]["static_displacement"] for i in est], tr.STATIC_GUARD_FLOOR_PX)
    _write_json("thresholds.json", {
        "frozen_before_holdout_evaluation": True,
        "formula": tr.FORMULA_TEXT,
        "estimation_frames": est,
        "holdout_frames": [hold[0], hold[-1]] if hold else [],
        "T_motion": t_motion,
        "T_static": t_static,
        "changed_after_seeing_series": False,
    })

    def over(indices, key, limit):
        return [i for i in indices if rows[i][key] is not None and rows[i][key] > limit]

    fp_frames = over(hold, "relative_displacement", t_motion["T"]) if t_motion["valid"] else list(hold)
    static_mask = [False] * len(rows)
    if t_static["valid"]:
        for index in hold:
            value = rows[index]["static_displacement"]
            static_mask[index] = value is not None and value > t_static["T"]
    static_runs = tr._runs(static_mask, tr.MIN_RUN_FRAMES)

    historical_gray, _ = _gray(HISTORICAL)
    roi_check = tr.phase_correlate_shift(tr.crop(historical_gray, tr.M2_ROI), tr.crop(grays[0], tr.M2_ROI))
    import p007f_05_hardware_wiring as wiring
    baseline = wiring.baseline_b_check(os.path.join(cache_dir, meta[0]["rgb_filename"]))

    phase_ok = all(np.isfinite(row["m2_phase_dx"]) and np.isfinite(row["static_phase_dx"])
                   and row["m2_phase_response"] > 0 and row["static_phase_response"] > 0 for row in rows)
    label = tr.classify_relative(rows, t_motion["T"]) if t_motion["valid"] else "INCONCLUSIVE"
    checks = {
        "baseline_b_unchanged": bool(baseline["unchanged"]),
        "m2_roi_visible": roi_check["magnitude"] <= tr.ROI_SHIFT_MAX_PX and roi_check["response"] >= tr.ROI_PHASE_RESPONSE_MIN,
        "static_reference_valid": static["initial_count"] >= tr.MIN_INITIAL_FEATURES,
        "m2_initial_features": m2["initial_count"] >= tr.MIN_INITIAL_FEATURES,
        "m2_survival": tr.survival_ok(rows, "m2_valid_feature_count", m2["initial_count"]),
        "static_survival": tr.survival_ok(rows, "static_valid_feature_count", static["initial_count"]),
        "phase_correlation_valid": phase_ok,
        "duration": duration_s >= tr.MIN_DURATION_S and len(rows) >= tr.MIN_FRAME_COUNT,
        "threshold_valid": bool(t_motion["valid"] and t_static["valid"]),
        "false_positive_count_zero": len(fp_frames) == 0,
        "static_guard_clear": len(static_runs) == 0,
        "classified_no_motion": label == "NO_MOTION",
        "command_events_empty": events.get("events") == [],
        "identity_valid": bool(identity.get("identity_valid")),
    }
    ready = all(checks.values())

    fields = list(rows[0].keys())
    with open(os.path.join(OUT_DIR, "static_sham.csv"), "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    with open(os.path.join(OUT_DIR, "tracking_quality.csv"), "w", newline="", encoding="utf-8") as handle:
        quality_fields = ["frame_index", "timestamp", "m2_valid_feature_count", "m2_tracking_quality",
                          "m2_fb_median_px", "static_valid_feature_count", "static_tracking_quality",
                          "static_fb_median_px"]
        writer = csv.DictWriter(handle, fieldnames=quality_fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row[key] for key in quality_fields})

    _save_roi(colors[0], tr.M2_ROI, "m2_roi.png", "M2")
    _save_roi(colors[0], tr.STATIC_REFERENCE_ROI, "static_reference_roi.png", "STATIC")
    overlay = colors[0].copy()
    _draw_box(overlay, tr.M2_ROI, (40, 220, 40), "M2")
    _draw_box(overlay, tr.STATIC_REFERENCE_ROI, (255, 180, 0), "STATIC")
    initial = tr.init_features(grays[0], tr.M2_ROI)
    for point in initial:
        cv2.circle(overlay, (int(point[0]), int(point[1])), 2, (40, 220, 40), -1)
    for point in m2["final_points"]:
        cv2.circle(overlay, (int(point[0]), int(point[1])), 2, (0, 140, 255), -1)
    cv2.imwrite(os.path.join(FIG_DIR, "tracking_overlay.png"), overlay)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    if os.path.exists(FONT_PATH):
        font_manager.fontManager.addfont(FONT_PATH)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT_PATH).get_name()
    times = [row["timestamp"] for row in rows]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, [row["m2_displacement"] or 0 for row in rows], label="M2 raw")
    ax.plot(times, [row["static_displacement"] or 0 for row in rows], label="STATIC")
    ax.plot(times, [row["relative_displacement"] or 0 for row in rows], label="relative")
    if t_motion["valid"]:
        ax.axhline(t_motion["T"], color="black", linestyle="--", label="T_motion")
    ax.set_xlabel("elapsed s")
    ax.set_ylabel("displacement px")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "sham_trajectory.png"), dpi=130)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4))
    rel = [row["relative_displacement"] for row in rows if row["relative_displacement"] is not None]
    ax.hist(rel, bins=40, color="darkorange")
    if t_motion["valid"]:
        ax.axvline(t_motion["T"], color="black", linestyle="--")
    ax.set_xlabel("relative displacement px")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "sham_relative_motion.png"), dpi=130)
    plt.close(fig)

    summary = {
        "m2_raw": tr.series_stats([row["m2_displacement"] for row in rows]),
        "static": tr.series_stats([row["static_displacement"] for row in rows]),
        "relative": tr.series_stats([row["relative_displacement"] for row in rows]),
        "m2_phase": tr.series_stats([row["m2_phase_magnitude"] for row in rows]),
        "static_phase": tr.series_stats([row["static_phase_magnitude"] for row in rows]),
        "relative_phase": tr.series_stats([row["relative_phase_displacement"] for row in rows]),
        "m2_phase_response": tr.series_stats([row["m2_phase_response"] for row in rows]),
        "static_phase_response": tr.series_stats([row["static_phase_response"] for row in rows]),
    }
    capture_meta = {
        "duration_s": duration_s,
        "frame_count": len(rows),
        "fps": fps,
        "color_sha256_of_sha256": manifest_hash,
        "command_events": events,
        "identity": identity,
        "baseline_b": baseline,
        "m2_roi_vs_historical": roi_check,
        "m2_initial_features": m2["initial_count"],
        "static_initial_features": static["initial_count"],
        "classification": label,
        "false_positive_frames": fp_frames,
        "static_guard_runs": static_runs,
        "checks": checks,
        "ready_and_armed": ready,
        "robot_command_count": 0,
        "servo_write_count": 0,
        "phase_b_executed": False,
        "statistics": summary,
    }
    _write_json("capture_metadata.json", capture_meta)
    with open(os.path.join(OUT_DIR, "frame_hashes.json"), "w", encoding="utf-8") as handle:
        json.dump({"sha256_of_sha256": manifest_hash, "frames": hashes}, handle)
        handle.write("\n")

    ready_text = "YES" if ready else "NO"
    failed = [name for name, ok in checks.items() if not ok]
    report = """# P-007F-08 Phase A — Continuous External Motion Witness

판정 질문: 고정 Gemini에서 M2 구조의 물리적 움직임이 보이는가.
이번 Phase A는 그 질문에 답하지 않는다. 명령을 보내지 않고, 추적기가 정지 상태에서 안정한지만 본다.

```text
READY_AND_ARMED = %s
PHASE_B_EXECUTED = NO
ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT = 0
MOTION_AUTHORIZED = NO
```

실패한 확인: %s

## 1. Gemini serial / firmware

identity_valid = %s
serial_found = %s
firmware_found = %s
기대값은 기존 기록과 같은 CP263630009D / 1.4.60 이고, 이번 기동 로그에서 다시 찾았다.

## 2. Baseline B invariant

unchanged = %s
shift = %.4f px (한도 1.0)
whole-frame mean abs diff = %.4f

## 3. M2 ROI

x=%d y=%d width=%d height=%d
역사적 000128 프레임의 같은 영역과 phase correlation: shift %.3f px, response %.3f
visible = %s

## 4. STATIC_REFERENCE ROI

x=%d y=%d width=%d height=%d
책상 위 키보드다. 로봇에 달린 부품이 아니다.
P-007F-03의 300×300 박스는 케이블을 포함해서 이번 기준으로 쓰지 않았다.

## 5. Tracking algorithm

Primary: sparse pyramidal Lucas-Kanade. 첫 프레임 Shi-Tomasi, 이후 calcOpticalFlowPyrLK, 재검출 없음.
forward-backward 최대 오차 1 px.
Secondary: cv2.phaseCorrelate. 판정에 승격하지 않는다.
template matching 은 쓰지 않았다.
마커 ID 검출은 acceptance 에 없다.

## 6. Initial feature count

M2 %d
STATIC %d
최소 %d

## 7. Feature survival/continuity

M2 survival check = %s
STATIC survival check = %s
기준: 프레임의 %.0f%% 이상에서 처음 점의 %.0f%% 이상이 살아 있을 것.

## 8. Static capture duration / frame count / FPS

duration %.3f s
frames %d
fps %.3f
최소 %.1f s, %d frames

## 9. M2 raw static motion statistics

%s

## 10. Static-reference motion statistics

%s

## 11. Relative static motion statistics

%s

## 12. Phase-correlation static statistics

M2 magnitude: %s
STATIC magnitude: %s
relative magnitude: %s
M2 response: %s
STATIC response: %s
valid = %s

## 13. Frozen motion-threshold formula

%s

추정 프레임 수 %s. frame 0 은 빼었다. 홀드아웃을 보고 공식을 바꾸지 않았다.

## 14. Frozen numeric threshold

T_motion = %s
T_static = %s

## 15. Static false-positive result

홀드아웃에서 relative displacement > T_motion 인 프레임: %d
프레임 번호: %s
분류: %s
static guard runs: %d

## 16. Continuous trajectory recording test

static_sham.csv 행 수 = %d (프레임 수와 같음)
열에 timestamp, frame index, M2/STATIC/relative dx dy displacement, feature count, quality 가 있다.

## 17. READY_AND_ARMED

%s

## 18. Robot command count

0

## 19. Servo write count

0

## 20. Evidence path / Commit SHA / git status

evidence/P-007F-08/phase_a/
color sha256-of-sha256 = %s
raw color 는 .cache/p007f08_phase_a/sham 에만 있다.
커밋 해시는 푸시 뒤 완료 보고에 적는다.

```text
PHASE_B_EXECUTED = NO
ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT = 0
MOTION_AUTHORIZED = NO
```
""" % (
        ready_text,
        ", ".join(failed) if failed else "없음",
        identity.get("identity_valid"), identity.get("serial_found"), identity.get("firmware_found"),
        baseline["unchanged"], baseline["shift_magnitude_px"], baseline["whole_frame_mean_abs_diff"],
        tr.M2_ROI[0], tr.M2_ROI[1], tr.M2_ROI[2], tr.M2_ROI[3],
        roi_check["magnitude"], roi_check["response"], checks["m2_roi_visible"],
        tr.STATIC_REFERENCE_ROI[0], tr.STATIC_REFERENCE_ROI[1], tr.STATIC_REFERENCE_ROI[2], tr.STATIC_REFERENCE_ROI[3],
        m2["initial_count"], static["initial_count"], tr.MIN_INITIAL_FEATURES,
        checks["m2_survival"], checks["static_survival"],
        tr.SURVIVAL_FRAME_FRACTION * 100, tr.SURVIVAL_FRACTION * 100,
        duration_s, len(rows), fps, tr.MIN_DURATION_S, tr.MIN_FRAME_COUNT,
        _stats_line(summary["m2_raw"]),
        _stats_line(summary["static"]),
        _stats_line(summary["relative"]),
        _stats_line(summary["m2_phase"]),
        _stats_line(summary["static_phase"]),
        _stats_line(summary["relative_phase"]),
        _stats_line(summary["m2_phase_response"]),
        _stats_line(summary["static_phase_response"]),
        phase_ok,
        tr.FORMULA_TEXT,
        t_motion.get("n"),
        json.dumps(t_motion, ensure_ascii=False),
        json.dumps(t_static, ensure_ascii=False),
        len(fp_frames), fp_frames[:30], label, len(static_runs),
        len(rows),
        ready_text,
        manifest_hash,
    )
    with open(os.path.join(OUT_DIR, "report.md"), "w", encoding="utf-8") as handle:
        handle.write(report)
    print(json.dumps({"ready_and_armed": ready, "failed": failed, "label": label,
                      "T_motion": t_motion.get("T"), "m2_features": m2["initial_count"],
                      "static_features": static["initial_count"], "frames": len(rows),
                      "duration_s": duration_s}, ensure_ascii=False, indent=1))


def capture(local_cache):
    import p007f_05_hardware_wiring as wiring
    wiring.start_gemini_driver()
    try:
        time.sleep(12)
        identity = wiring.gemini_identity_check()
        os.makedirs(local_cache, exist_ok=True)
        with open(os.path.join(local_cache, "identity.json"), "w", encoding="utf-8") as handle:
            json.dump({"identity": identity}, handle, indent=1)
        if not identity.get("identity_valid"):
            raise RuntimeError("gemini identity mismatch")
        remote_dir = wiring.P620_REPO + "/" + CACHE_REL
        wiring.ssh_p620(
            "docker exec -w /ws/cad-less-robotics bytesize-robotics-camera mkdir -p " + CACHE_REL)
        rc, out, err = wiring.ssh_p620(
            "cd %s && bash scripts/run_humble_camera.sh python3 scripts/p007f_07b_continuous_capture_runner.py %s 26"
            % (wiring.P620_REPO, CACHE_REL),
            timeout=180)
        wiring.ssh_p620(
            "docker exec -u 0 -w /ws/cad-less-robotics bytesize-robotics-camera chmod -R a+rX " + CACHE_REL)
        subprocess.run(
            ["rsync", "-a", "--include", "*-color.png", "--include", "frames_meta.json",
             "--include", "command_events.json", "--exclude", "*",
             "p620:" + remote_dir + "/", local_cache + "/"],
            check=True)
        with open(os.path.join(local_cache, "identity.json"), "w", encoding="utf-8") as handle:
            json.dump({"identity": identity, "capture_rc": rc, "capture_stdout": out, "capture_stderr": err},
                      handle, indent=1)
        if rc != 0:
            raise RuntimeError("capture rc=%s stderr=%s" % (rc, err))
    finally:
        wiring.stop_gemini_driver()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--analyze", default="")
    args = parser.parse_args()
    if args.prepare:
        prepare()
    if args.capture:
        capture(os.path.join(ROOT, CACHE_REL))
    if args.analyze:
        identity_path = os.path.join(args.analyze, "identity.json")
        identity = {}
        if os.path.exists(identity_path):
            identity = json.load(open(identity_path, encoding="utf-8")).get("identity", {})
        analyze(args.analyze, identity)


if __name__ == "__main__":
    main()
