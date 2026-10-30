#!/usr/bin/env python3
"""저장된 Dataset v2를 다시 읽어 대응과 해시를 재계산한다. 카메라를 열지 않는다."""

import json
import struct
import zlib
from pathlib import Path

import numpy as np

from p008b_metrics import color_correspondence, depth_cloud_metrics, sha256_file, write_png

ROOT = Path(__file__).resolve().parents[1] / "evidence" / "P-008B"


def read_png(path):
    data = Path(path).read_bytes()
    if not data.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError(f"PNG 서명이 아니다: {path}")
    pos = 8
    width = height = bit_depth = color_type = None
    idat = b""
    while pos + 8 <= len(data):
        length = struct.unpack(">I", data[pos:pos + 4])[0]
        tag = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            width, height, bit_depth, color_type = struct.unpack(">IIBB", chunk[:10])
        elif tag == b"IDAT":
            idat += chunk
        elif tag == b"IEND":
            break
    raw = zlib.decompress(idat)
    channels = 3 if color_type == 2 else 1
    sample = 2 if bit_depth == 16 else 1
    stride = width * channels * sample
    rows = []
    offset = 0
    for _ in range(height):
        filt = raw[offset]
        offset += 1
        if filt != 0:
            raise ValueError(f"filter {filt} 는 이 저장기가 만들지 않는다: {path}")
        rows.append(raw[offset:offset + stride])
        offset += stride
    buf = b"".join(rows)
    if bit_depth == 16:
        return np.frombuffer(buf, dtype=">u2").astype(np.uint16).reshape(height, width)
    if channels == 3:
        return np.frombuffer(buf, dtype=np.uint8).reshape(height, width, 3)
    return np.frombuffer(buf, dtype=np.uint8).reshape(height, width)


def read_pcd(path):
    raw = Path(path).read_bytes()
    marker = b"\nDATA binary\n"
    header, body = raw.split(marker, 1)
    text = header.decode("ascii")
    if "FIELDS x y z rgb" not in text:
        raise ValueError(f"FIELDS x y z rgb 가 아니다: {path}")
    count = int([line for line in text.splitlines() if line.startswith("POINTS ")][0].split()[1])
    packed = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")], count=count)
    xyz = np.column_stack((packed["x"], packed["y"], packed["z"])).astype(np.float64)
    rgb_bytes = np.ascontiguousarray(packed["rgb"]).view(np.uint8).reshape(count, 4)
    rgb = np.column_stack((rgb_bytes[:, 2], rgb_bytes[:, 1], rgb_bytes[:, 0])).astype(np.uint8)
    return xyz, rgb


def intrinsics(info):
    k = info["k"]
    return float(k[0]), float(k[4]), float(k[2]), float(k[5])


def depth_view(depth):
    positive = depth[depth > 0]
    view = np.zeros(depth.shape, dtype=np.uint8)
    if positive.size:
        lo, hi = np.percentile(positive, [2, 98])
        scaled = np.clip((depth.astype(np.float64) - lo) / max(hi - lo, 1.0), 0.0, 1.0)
        view[depth > 0] = (32 + 223 * scaled[depth > 0]).astype(np.uint8)
    return view


def paint_cloud(xyz, rgb, fx, fy, cx, cy, shape):
    canvas = np.zeros(shape, dtype=np.uint8)
    z = xyz[:, 2]
    valid = np.isfinite(xyz).all(axis=1) & (z > 0.05) & (z < 3.0)
    ui = np.rint(fx * xyz[:, 0] / z + cx)
    vi = np.rint(fy * xyz[:, 1] / z + cy)
    height, width = shape[:2]
    inside = valid & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
    canvas[vi[inside].astype(np.int64), ui[inside].astype(np.int64)] = rgb[inside]
    return canvas


def alignment_view(color, depth):
    view = color.copy()
    hole = depth == 0
    view[hole] = (view[hole].astype(np.uint16) // 5).astype(np.uint8)
    return view


def correspondence_chart(rows):
    width, height = 900, 360
    image = np.full((height, width, 3), 245, dtype=np.uint8)
    image[40:320, 70:860] = 255
    # 로그 눈금. 아래가 1e-8 m, 위가 1e-2 m.
    def y_of(value):
        log_v = np.log10(max(value, 1e-8))
        return int(320 - (log_v - (-8.0)) / ( -2.0 - (-8.0)) * 280)

    threshold = y_of(0.005)
    image[threshold - 1:threshold + 1, 70:860] = (180, 40, 40)
    for index, row in enumerate(rows):
        x0 = 110 + index * 140
        y = y_of(row["p95_m"])
        image[y:320, x0:x0 + 60] = (30, 90, 160)
    return image


def main():
    captures = []
    chart_rows = []
    first_color = first_depth = None
    first_xyz = first_rgb = None
    first_k = None
    for index in range(1, 6):
        directory = ROOT / f"capture_{index:02d}"
        color = read_png(directory / "color.png")
        depth = read_png(directory / "depth.png")
        xyz, rgb = read_pcd(directory / "cloud.pcd")
        color_info = json.loads((directory / "color_camera_info.json").read_text())
        depth_info = json.loads((directory / "depth_camera_info.json").read_text())
        fx, fy, cx, cy = intrinsics(depth_info)
        color_k = intrinsics(color_info)
        depth_metrics = depth_cloud_metrics(xyz, depth, fx, fy, cx, cy)
        color_metrics = color_correspondence(xyz, rgb, color, *color_k)
        finite = int(np.isfinite(xyz).all(axis=1).sum())
        row = {
            "capture": f"capture_{index:02d}",
            "color_shape": list(color.shape),
            "color_dtype": str(color.dtype),
            "depth_shape": list(depth.shape),
            "depth_dtype": str(depth.dtype),
            "depth_positive_pixel_ratio": float(np.mean(depth > 0)),
            "cloud_point_count": int(xyz.shape[0]),
            "cloud_finite_count": finite,
            "cloud_fields": ["x", "y", "z", "rgb"],
            "depth_frame": depth_info["frame_id"],
            "color_frame": color_info["frame_id"],
            "sha256": {
                "color": sha256_file(directory / "color.png"),
                "depth": sha256_file(directory / "depth.png"),
                "cloud": sha256_file(directory / "cloud.pcd"),
            },
            "timestamps": json.loads((directory / "timestamps.json").read_text()),
            "depth_cloud": depth_metrics,
            "color_correspondence": color_metrics,
            "depth_k": [fx, fy, cx, cy],
            "color_k": list(color_k),
            "readable": True,
        }
        captures.append(row)
        chart_rows.append(depth_metrics)
        if index == 1:
            first_color, first_depth = color, depth
            first_xyz, first_rgb = xyz, rgb
            first_k = (fx, fy, cx, cy)
    figures = ROOT / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    write_png(figures / "01_color.png", first_color)
    write_png(figures / "02_depth.png", depth_view(first_depth))
    write_png(figures / "03_pointcloud.png", paint_cloud(first_xyz, first_rgb, *first_k, first_color.shape))
    write_png(figures / "04_alignment_diagnostic.png", alignment_view(first_color, first_depth))
    write_png(figures / "05_correspondence_diagnostic.png", correspondence_chart(chart_rows))
    payload = {"captures": captures, "registration_accuracy": "NOT_CLAIMED", "hardware_sync": "NOT_VERIFIED"}
    (ROOT / "readback.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    for row in captures:
        print(
            row["capture"],
            "depth_pass", row["depth_cloud"]["passed"],
            "ratio", row["depth_cloud"]["positive_depth_ratio"],
            "p95", row["depth_cloud"]["p95_m"],
            "color_pass", row["color_correspondence"]["passed"],
            "pixel_positive", round(row["depth_positive_pixel_ratio"], 4),
            "points", row["cloud_point_count"],
        )


if __name__ == "__main__":
    main()
