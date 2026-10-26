#!/usr/bin/env python3
"""R2 semantic position의 컬러와 깊이를 읽기만 한다. 로봇 명령은 없다."""

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p006c_capture_streams import Grab, image_array, info_dict, stamp_sec, write_png

POSITIONS = ("CENTER", "LEFT", "RIGHT", "ROBOT_SIDE", "CAMERA_SIDE")


def identity_transform(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    values = (translation.x, translation.y, translation.z)
    norm = sum(value * value for value in values) ** 0.5
    return norm < 1e-6 and abs(rotation.w) > 0.999999


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--position", required=True, choices=POSITIONS)
    parser.add_argument("--try-index", type=int, required=True)
    parser.add_argument("--count", type=int, default=4)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--out", default="evidence/P-006C-R2/captures")
    args = parser.parse_args()
    out = Path(args.out)
    if out.name != "captures" or "P-006C-R2" not in out.parts:
        raise SystemExit(f"출력은 evidence/P-006C-R2/captures 만 허용한다: {out}")
    out.mkdir(parents=True, exist_ok=True)
    stem_prefix = f"{args.position.lower()}-t{args.try_index:02d}-"
    if list(out.glob(f"{stem_prefix}*.json")):
        raise SystemExit(f"이미 같은 배치가 있다. 지우지 않는다: {stem_prefix}")

    rclpy.init()
    node = Grab()
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline and (node.color is None or node.depth is None or node.color_info is None or node.depth_info is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.color is None or node.depth is None or node.color_info is None or node.depth_info is None:
        raise SystemExit("컬러, 깊이, camera_info 중 하나가 없다")

    saved = []
    skipped_sync = 0
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
        tf_reason = "lookup_failed"
        try:
            looked = node.buffer.lookup_transform(node.depth.header.frame_id, node.color.header.frame_id, rclpy.time.Time())
            if identity_transform(looked):
                tf_reason = "identity_rejected"
            else:
                transform = looked
                tf_reason = "ok"
        except Exception:
            transform = None
        index = len(saved)
        stem = f"{stem_prefix}{index:02d}"
        write_png(out / f"{stem}-color.png", image_array(node.color))
        write_png(out / f"{stem}-depth.png", image_array(node.depth))
        record = {
            "semantic_position": args.position,
            "try_index": args.try_index,
            "stem": stem,
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
            "tf_reason": tf_reason,
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
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({"position": args.position, "try_index": args.try_index, "saved": saved, "skipped_sync": skipped_sync}))


if __name__ == "__main__":
    main()
