#!/usr/bin/env python3
"""한 try의 컬러 포즈만 계산해 고정된 분리 조건과 비교한다. threshold 파일은 쓰지 않는다."""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p006c_validate_metric as metric

ROOT = Path("evidence/P-006/camera-target/attempt-04")
FRAMES = ROOT / "frames"


def median_pose(plan, pose, try_index):
    thresholds = metric.load_yaml(Path("evidence/P-006/camera-target/thresholds.yaml"))
    prefix = f"{pose}-t{try_index:02d}-"
    translations = []
    for path in sorted(FRAMES.glob(f"{prefix}*.json")):
        record = json.loads(path.read_text())
        color = metric.read_png(FRAMES / f"{record['stem']}-color.png")
        if color.ndim == 3 and color.shape[2] == 3:
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        else:
            gray = color
        found = metric.detect(gray, thresholds)
        if found is None:
            continue
        cols, rows, corners = found
        obj = metric.object_points(cols, rows)
        matrix = metric.camera_matrix(record["color_info"])
        dist = metric.dist_coeffs(record["color_info"])
        ok, _rvec, tvec = cv2.solvePnP(obj, corners, matrix, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or float(tvec[2]) <= 0.0:
            continue
        translations.append(tvec.reshape(3))
    fixed = np.array(plan["fixed_reference"]["median_t_m"], dtype=np.float64)
    lateral = float(plan["separation"]["lateral_m"])
    range_m = float(plan["separation"]["range_m"])
    minimum = int(plan["frames_per_try"])
    if not translations:
        return {"solved": 0, "delta_x_m": None, "delta_y_m": None, "delta_z_m": None, "rule": "FAIL"}
    median = np.median(np.stack(translations, axis=0), axis=0)
    delta = median - fixed
    passed = len(translations) >= minimum
    if pose == "left":
        passed = passed and float(delta[0]) < -lateral
    elif pose == "right":
        passed = passed and float(delta[0]) > lateral
    elif pose == "near":
        passed = passed and float(delta[2]) < -range_m
    elif pose == "far":
        passed = passed and float(delta[2]) > range_m
    else:
        passed = False
    return {
        "solved": len(translations),
        "delta_x_m": float(delta[0]),
        "delta_y_m": float(delta[1]),
        "delta_z_m": float(delta[2]),
        "rule": "PASS" if passed else "FAIL",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", required=True)
    parser.add_argument("--try-index", type=int, required=True)
    args = parser.parse_args()
    plan = metric.load_yaml(ROOT / "capture-plan.yaml")
    result = median_pose(plan, args.pose, args.try_index)
    result["pose"] = args.pose
    result["try_index"] = args.try_index
    out = ROOT / "feedback"
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{args.pose}-t{args.try_index:02d}.json").write_text(json.dumps(result, indent=2) + "\n")
    dx = "" if result["delta_x_m"] is None else f"{result['delta_x_m']:.8f}"
    dy = "" if result["delta_y_m"] is None else f"{result['delta_y_m']:.8f}"
    dz = "" if result["delta_z_m"] is None else f"{result['delta_z_m']:.8f}"
    print(f"FEEDBACK pose={args.pose} try={args.try_index} solved={result['solved']} delta_x_m={dx} delta_y_m={dy} delta_z_m={dz} rule={result['rule']}")


if __name__ == "__main__":
    main()
