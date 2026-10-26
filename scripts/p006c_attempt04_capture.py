#!/usr/bin/env python3
"""Attempt 3에서 left/right/near/far 한 번을 읽기만 한다. 기존 증거는 쓰지 않는다."""

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p006c_capture_streams import Grab, image_array, info_dict, stamp_sec, write_png


ALLOWED = ("near", "far")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", required=True, choices=ALLOWED)
    parser.add_argument("--try-index", type=int, required=True)
    parser.add_argument("--count", type=int, default=4)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--out", default="evidence/P-006/camera-target/attempt-04/frames")
    args = parser.parse_args()
    if args.try_index < 1 or args.count < 1:
        raise SystemExit("try와 frame 수는 1 이상이어야 한다")

    out = Path(args.out)
    if "attempt-04" not in out.parts or out.name != "frames":
        raise SystemExit(f"출력 경로는 attempt-04/frames 만 허용한다: {out}")
    prefix = f"{args.pose}-t{args.try_index:02d}-"
    existing = list(out.glob(f"{prefix}*.json"))
    if existing:
        raise SystemExit(f"이미 같은 try가 있다. 지우지 않는다: {prefix}")
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline and (node.color is None or node.depth is None or node.color_info is None or node.depth_info is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.color is None or node.depth is None or node.color_info is None or node.depth_info is None:
        raise SystemExit("컬러, 깊이, camera_info 중 하나가 없다")

    saved = []
    skipped_sync = 0
    skipped_tf = 0
    next_save = time.monotonic()
    end = time.monotonic() + 8.0
    while len(saved) < args.count and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() < next_save or node.color is None or node.depth is None:
            continue
        color_t = stamp_sec(node.color.header.stamp)
        depth_t = stamp_sec(node.depth.header.stamp)
        delta = abs(color_t - depth_t)
        if delta > 0.05:
            skipped_sync += 1
            continue
        transform = None
        try:
            transform = node.buffer.lookup_transform(
                node.depth.header.frame_id,
                node.color.header.frame_id,
                rclpy.time.Time(),
            )
        except Exception:
            skipped_tf += 1
            transform = None
        index = len(saved)
        stem = f"{args.pose}-t{args.try_index:02d}-{index:02d}"
        color = image_array(node.color)
        depth = image_array(node.depth)
        write_png(out / f"{stem}-color.png", color)
        write_png(out / f"{stem}-depth.png", depth)
        record = {
            "index": index,
            "try_index": args.try_index,
            "stem": stem,
            "pose_label": args.pose,
            "color_stamp": color_t,
            "depth_stamp": depth_t,
            "stamp_delta_sec": delta,
            "color_frame_id": node.color.header.frame_id,
            "depth_frame_id": node.depth.header.frame_id,
            "color_encoding": node.color.encoding,
            "depth_encoding": node.depth.encoding,
            "color_width": int(node.color.width),
            "color_height": int(node.color.height),
            "depth_width": int(node.depth.width),
            "depth_height": int(node.depth.height),
            "color_info": info_dict(node.color_info),
            "depth_info": info_dict(node.depth_info),
            "T_depth_color": None,
        }
        if transform is not None:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            record["T_depth_color"] = {
                "parent": node.depth.header.frame_id,
                "child": node.color.header.frame_id,
                "translation_m": [translation.x, translation.y, translation.z],
                "quaternion_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
            }
        (out / f"{stem}.json").write_text(json.dumps(record, indent=2) + "\n")
        saved.append(stem)
        next_save = time.monotonic() + args.interval

    summary_path = out.parent / "capture-runs.jsonl"
    with summary_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps({
            "pose": args.pose,
            "try_index": args.try_index,
            "saved": saved,
            "skipped_sync": skipped_sync,
            "skipped_tf": skipped_tf,
        }) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    if len(saved) < 1:
        raise SystemExit(f"{args.pose} try {args.try_index} 동기 프레임이 없다")


if __name__ == "__main__":
    main()
