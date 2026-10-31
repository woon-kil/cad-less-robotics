#!/usr/bin/env python3
"""R1I 개발 캡처 1장. evidence/P-008B 에는 쓰지 않는다."""

import sys
import time
from pathlib import Path

import numpy as np
import rclpy

from p008b_acquire import Grab, best_set, parse_identity, save_capture, write_json
from p008b_metrics import extrinsic_rejection, sha256_file


def main():
    if len(sys.argv) != 2:
        raise SystemExit("사용: p008r1i_capture.py <driver_log>")
    output = Path("evidence/P-008/R1I/capture_dev")
    if "P-008B" in output.parts:
        raise SystemExit("Dataset v2 경로에는 쓰지 않는다.")
    log_path = Path(sys.argv[1])
    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 60.0
    chosen = None
    optical = None
    while time.monotonic() < deadline and chosen is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        optical = node.lookup_optical()
        if node.color_info and node.depth_info and node.extrinsic_msg and optical:
            chosen = best_set(node)
    if chosen is None or optical is None:
        raise SystemExit("스트림 또는 extrinsic이 없다.")
    color, depth, cloud, _stamps, span = chosen
    depth_metrics, color_metrics = save_capture(
        output, color, depth, cloud, node.color_info, node.depth_info, optical
    )
    identity = parse_identity(log_path.read_text(errors="replace")) if log_path.exists() else {}
    meta, xyz, _rgb, _rgb_float = cloud
    rotation = np.array(optical["rotation"], dtype=np.float64).reshape(3, 3)
    checks = {
        "serial": identity.get("serial") == "CP263630009D",
        "firmware": identity.get("firmware") == "1.4.60",
        "wrapper": identity.get("ros_wrapper_version") == "2.10.6",
        "color_frame": color[1] == "camera_color_optical_frame",
        "depth_frame": depth[1] == "camera_color_optical_frame",
        "cloud_frame": meta["frame_id"] == "camera_color_optical_frame",
        "fields_rgb": "rgb" in [field["name"] for field in meta["fields"]],
        "stamp_span_sec": span,
        "stamp_span_ok": span <= 0.05,
        "depth_cloud_passed": bool(depth_metrics["passed"]),
        "color_correspondence_passed": bool(color_metrics["passed"]),
        "extrinsic_ok": extrinsic_rejection(rotation, optical["translation_m"]) is None,
        "finite_xyz": int(len(xyz)),
        "registration_accuracy": "NOT_CLAIMED",
    }
    write_json(output / "acquisition_checks.json", {"identity": identity, "checks": checks})
    for name in ("color.png", "depth.png", "cloud.pcd"):
        digest = sha256_file(output / name)
        checks[name + "_sha256"] = digest
    write_json(output / "acquisition_checks.json", {"identity": identity, "checks": checks, "span_sec": span})
    node.destroy_node()
    rclpy.shutdown()
    print("R1I_CAPTURE_DONE", "depth", depth_metrics["passed"], "color", color_metrics["passed"], "n", len(xyz))


if __name__ == "__main__":
    main()
