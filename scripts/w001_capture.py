#!/usr/bin/env python3
"""W-001 작업판 시야 1장. 기존 캡처 경로에는 쓰지 않는다."""

import sys
import time
from pathlib import Path

import numpy as np
import rclpy

from p008b_acquire import Grab, best_set, parse_identity, save_capture, write_json
from p008b_metrics import extrinsic_rejection, sha256_file


def main():
    if len(sys.argv) != 3:
        raise SystemExit("사용: w001_capture.py <driver_log> <output>")
    log_path = Path(sys.argv[1])
    output = Path(sys.argv[2]).resolve()
    if output.name != "W-001":
        raise SystemExit("출력은 evidence/W-001 만 허용한다.")
    if (output / "cloud.pcd").exists():
        raise SystemExit("이미 캡처가 있다. 덮어쓰지 않는다.")
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
        "topic": "/camera/depth_registered/points",
    }
    for name in ("color.png", "depth.png", "cloud.pcd"):
        checks[name + "_sha256"] = sha256_file(output / name)
    passed = all(
        checks[key]
        for key in (
            "serial",
            "firmware",
            "wrapper",
            "color_frame",
            "depth_frame",
            "cloud_frame",
            "fields_rgb",
            "stamp_span_ok",
            "depth_cloud_passed",
            "color_correspondence_passed",
            "extrinsic_ok",
        )
    )
    checks["contract_passed"] = bool(passed)
    write_json(output / "acquisition.json", {"identity": identity, "checks": checks, "span_sec": span})
    write_json(output / "camera_info.json", {
        "color": node.color_info,
        "depth": node.depth_info,
        "frame_id": meta["frame_id"],
        "topic": "/camera/depth_registered/points",
    })
    node.destroy_node()
    rclpy.shutdown()
    print("W001_CAPTURE", "contract", checks["contract_passed"], "n", len(xyz), "serial", identity.get("serial"))
    if not passed:
        raise SystemExit("BLOCKED_ACQUISITION_CONTRACT")


if __name__ == "__main__":
    main()
