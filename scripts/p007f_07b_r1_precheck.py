#!/usr/bin/env python3
"""P-007F-07B-R1 사전확인. 중복 ID0가 남아 있으면 본촬영으로 넘어가지 않는다.

카메라/로봇 API는 호출하지 않는다. 이미 받아 둔 RGB만 읽는다.
합격선은 바꾸지 않는다. 이 스크립트는 qualification PASS를 만들지 않는다.
"""
import csv
import hashlib
import importlib.util
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRECHECK_DIR = os.environ.get(
    "P007F07B_R1_PRECHECK_DIR",
    os.path.join(ROOT, ".cache", "p007f07b_r1", "precheck"))
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_static_qualification_r1")
DETECTOR_PATH = os.path.join(ROOT, "scripts", "p007f_07_marker_detector.py")
WIRING_PATH = os.path.join(ROOT, "scripts", "p007f_05_hardware_wiring.py")

# B에서 관측된 M2 ID0 군집. 이 좌표로 합격선을 바꾸지 않고, 다른 위치의 ID0를
# M2로 세지 않기 위한 구분선이다.
M2_CLUSTER_XY = (775.8, 126.6)
M2_CLUSTER_RADIUS_PX = 80.0


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


def main():
    det = _load("p007f_07_marker_detector_pin", DETECTOR_PATH)
    wiring = _load("p007f_05_hardware_wiring_pin", WIRING_PATH)
    cv2 = det.cv2
    meta = json.load(open(os.path.join(PRECHECK_DIR, "frames_meta.json"), encoding="utf-8"))
    detector = det.create_detector()
    rows = []
    max_id0 = 0
    non_m2_id0 = []
    for fm in meta:
        path = os.path.join(PRECHECK_DIR, fm["rgb_filename"])
        image = cv2.imread(path)
        hits = det.detect_markers(image, detector, timestamp=fm["elapsed_s_since_start"],
                                   frame_index=fm["frame_index"])
        detected = [h for h in hits if h.get("marker_detected")]
        id0 = [h for h in detected if h["marker_id"] == 0]
        max_id0 = max(max_id0, len(id0))
        for h in id0:
            dist = ((h["center_x"] - M2_CLUSTER_XY[0]) ** 2 + (h["center_y"] - M2_CLUSTER_XY[1]) ** 2) ** 0.5
            if dist > M2_CLUSTER_RADIUS_PX:
                non_m2_id0.append({
                    "frame_index": fm["frame_index"],
                    "center_x": h["center_x"],
                    "center_y": h["center_y"],
                    "side_length_mean_px": h["quality"]["side_length_mean_px"],
                    "distance_from_b_m2_cluster_px": dist,
                })
        rows.append({
            "frame_index": fm["frame_index"],
            "timestamp_s": fm["elapsed_s_since_start"],
            "ids": [h["marker_id"] for h in detected],
            "id0_count": len(id0),
        })

    baseline = wiring.baseline_b_check(os.path.join(PRECHECK_DIR, meta[0]["rgb_filename"]))
    duplicate = max_id0 >= 2 or len(non_m2_id0) > 0
    decision = "ABORT_DUPLICATE_ID0_STILL_PRESENT" if duplicate else "PRECHECK_SINGLE_ID0_CANDIDATE"
    if not baseline["unchanged"]:
        decision = "ABORT_BASELINE_B_CHANGED"

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "precheck_ids.csv"), "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["frame_index", "timestamp_s", "ids", "id0_count"])
        writer.writeheader()
        for row in rows:
            writer.writerow({
                "frame_index": row["frame_index"],
                "timestamp_s": row["timestamp_s"],
                "ids": " ".join(str(i) for i in row["ids"]),
                "id0_count": row["id0_count"],
            })

    color_names = [fm["rgb_filename"] for fm in meta]
    manifest = [{"file": name, "sha256": sha256(os.path.join(PRECHECK_DIR, name))} for name in color_names]
    record = {
        "gate": "P-007F-07B-R1",
        "stage": "precheck",
        "decision": decision,
        "long_capture_started": False,
        "reason": (
            "decoded ID0 is not the M2 cluster from P-007F-07B; a second printed marker remains in the FOV"
            if duplicate else "no second ID0 decode in this precheck"),
        "frame_count": len(meta),
        "duration_s": meta[-1]["elapsed_s_since_start"],
        "max_id0_in_one_frame": max_id0,
        "non_m2_id0": non_m2_id0,
        "m2_cluster_reference_px": {"x": M2_CLUSTER_XY[0], "y": M2_CLUSTER_XY[1],
                                    "radius_px": M2_CLUSTER_RADIUS_PX,
                                    "source": "P-007F-07B upper cluster, not a new acceptance threshold"},
        "baseline_b_check": baseline,
        "camera_expected_serial": wiring.GEMINI_EXPECTED_SERIAL,
        "camera_expected_firmware": wiring.GEMINI_EXPECTED_FIRMWARE,
        "local_capture_dir": PRECHECK_DIR,
        "color_sha256": manifest,
        "robot_command_count": 0,
        "servo_write_count": 0,
        "torque_command_count": 0,
        "READY_FOR_MOTION_PREREGISTRATION": "NO",
        "MOTION_AUTHORIZED": "NO",
        "MARKER_STATIC_QUALIFICATION_R1": "NOT_ISSUED",
    }
    with open(os.path.join(OUT_DIR, "precheck.json"), "w", encoding="utf-8") as handle:
        json.dump(record, handle, ensure_ascii=False, indent=1)
    print(json.dumps({"decision": decision, "max_id0": max_id0, "non_m2": len(non_m2_id0),
                      "baseline_unchanged": baseline["unchanged"]}, ensure_ascii=False))
    if decision != "PRECHECK_SINGLE_ID0_CANDIDATE":
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
