#!/usr/bin/env python3
"""P-008A 저장 파일을 다시 읽고 hash, preview를 만든다. 기하 추출은 하지 않는다."""

import hashlib
import json
import struct
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "evidence" / "P-008A"


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_png(path):
    raw = path.read_bytes()
    if raw[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"PNG signature 실패: {path}")
    length = struct.unpack(">I", raw[8:12])[0]
    if raw[12:16] != b"IHDR" or length != 13:
        raise ValueError(f"IHDR 실패: {path}")
    width, height, bit_depth, color_type = struct.unpack(">IIBB", raw[16:26])
    image = Image.open(path)
    image.load()
    return {"width": int(width), "height": int(height), "bit_depth": int(bit_depth), "color_type": int(color_type), "image": image}


def load_pcd(path):
    data = path.read_bytes()
    marker = b"DATA binary\n"
    index = data.find(marker)
    if index < 0:
        raise ValueError(f"PCD binary marker 없음: {path}")
    header = data[:index].decode("ascii")
    points = None
    for line in header.splitlines():
        if line.startswith("POINTS "):
            points = int(line.split()[1])
    body = data[index + len(marker):]
    xyz = np.frombuffer(body, dtype="<f4")
    if points is None or xyz.size != points * 3:
        raise ValueError(f"PCD 길이 불일치: {path} points={points} floats={xyz.size}")
    xyz = xyz.reshape(points, 3)
    finite = np.isfinite(xyz).all(axis=1)
    return xyz, int(np.count_nonzero(finite))


def depth_preview(depth_path, out_path):
    info = load_png(depth_path)
    depth = np.array(info["image"])
    valid = depth > 0
    preview = np.zeros(depth.shape, dtype=np.uint8)
    if np.any(valid):
        values = depth[valid].astype(np.float32)
        low, high = np.percentile(values, [2, 98])
        if high <= low:
            high = low + 1.0
        scaled = np.clip((depth.astype(np.float32) - low) * (255.0 / (high - low)), 0, 255)
        preview[valid] = scaled[valid].astype(np.uint8)
    Image.fromarray(preview, mode="L").save(out_path)
    return int(depth.shape[1]), int(depth.shape[0]), int(np.count_nonzero(valid))


def cloud_preview(xyz, out_path):
    finite = xyz[np.isfinite(xyz).all(axis=1)]
    if finite.shape[0] > 80000:
        finite = finite[:: max(1, finite.shape[0] // 80000)]
    width, height = 960, 720
    image = Image.new("RGB", (width, height), (16, 16, 18))
    draw = ImageDraw.Draw(image)
    if finite.shape[0] == 0:
        image.save(out_path)
        return
    x = finite[:, 0]
    y = finite[:, 1]
    z = finite[:, 2]
    xmin, xmax = np.percentile(x, [1, 99])
    ymin, ymax = np.percentile(y, [1, 99])
    zmin, zmax = np.percentile(z, [1, 99])
    if xmax <= xmin:
        xmax = xmin + 1e-6
    if ymax <= ymin:
        ymax = ymin + 1e-6
    if zmax <= zmin:
        zmax = zmin + 1e-6
    px = ((x - xmin) / (xmax - xmin) * (width - 40) + 20).astype(np.int32)
    py = ((y - ymin) / (ymax - ymin) * (height - 40) + 20).astype(np.int32)
    tone = ((z - zmin) / (zmax - zmin) * 255.0).astype(np.int32)
    for index in range(0, finite.shape[0], 3):
        color = (int(tone[index]), 140, int(255 - tone[index]))
        draw.point((int(px[index]), int(py[index])), fill=color)
    image.save(out_path)


def main():
    captures = sorted((DATA / "captures").glob("capture_*"))
    if len(captures) != 5:
        raise SystemExit(f"capture 수가 5가 아니다: {len(captures)}")
    lines = []
    rows = []
    stamps = []
    for cap in captures:
        meta = json.loads((cap / "metadata.json").read_text())
        color_info = load_png(cap / "color.png")
        depth_info = load_png(cap / "depth.png")
        xyz, finite = load_pcd(cap / "cloud.pcd")
        if color_info["width"] != meta["color"]["width"] or color_info["height"] != meta["color"]["height"]:
            raise SystemExit(f"{cap.name} color 해상도 불일치")
        if depth_info["width"] != meta["depth"]["width"] or depth_info["height"] != meta["depth"]["height"]:
            raise SystemExit(f"{cap.name} depth 해상도 불일치")
        if xyz.shape[0] != meta["cloud"]["point_count"] or finite != meta["cloud"]["finite_xyz_count"]:
            raise SystemExit(f"{cap.name} cloud read-back 불일치")
        files = {
            "color.png": sha256(cap / "color.png"),
            "depth.png": sha256(cap / "depth.png"),
            "cloud.pcd": sha256(cap / "cloud.pcd"),
            "metadata.json": sha256(cap / "metadata.json"),
        }
        for name, digest in files.items():
            lines.append(f"{digest}  captures/{cap.name}/{name}")
        row = {
            "capture": cap.name,
            "color_stamp_sec": meta["color"]["stamp_sec"],
            "depth_stamp_sec": meta["depth"]["stamp_sec"],
            "cloud_stamp_sec": meta["cloud"]["stamp_sec"],
            "color_width": meta["color"]["width"],
            "color_height": meta["color"]["height"],
            "color_encoding": meta["color"]["encoding"],
            "color_sha256": files["color.png"],
            "depth_width": meta["depth"]["width"],
            "depth_height": meta["depth"]["height"],
            "depth_encoding": meta["depth"]["encoding"],
            "depth_sha256": files["depth.png"],
            "cloud_point_count": meta["cloud"]["point_count"],
            "cloud_finite_xyz_count": meta["cloud"]["finite_xyz_count"],
            "cloud_sha256": files["cloud.pcd"],
            "frame_id_color": meta["color"]["frame_id"],
            "frame_id_depth": meta["depth"]["frame_id"],
            "frame_id_cloud": meta["cloud"]["frame_id"],
            "readback": "PASS",
        }
        rows.append(row)
        stamps.append((row["color_stamp_sec"], row["depth_stamp_sec"], row["cloud_stamp_sec"], row["color_sha256"], row["depth_sha256"], row["cloud_sha256"]))
    distinct = len({item[3] for item in stamps}) == 5 and len({item[5] for item in stamps}) == 5 and len({item[0] for item in stamps}) == 5
    preview = DATA / "preview"
    preview.mkdir(parents=True, exist_ok=True)
    first = captures[0]
    color = load_png(first / "color.png")["image"]
    color.save(preview / "01_color_preview.png")
    depth_preview(first / "depth.png", preview / "02_depth_preview.png")
    cloud_preview(load_pcd(first / "cloud.pcd")[0], preview / "03_pointcloud_preview.png")
    for extra in (DATA / "camera_info").glob("*.json"):
        lines.append(f"{sha256(extra)}  camera_info/{extra.name}")
    manifest = DATA / "manifest.sha256"
    manifest.write_text("\n".join(lines) + "\n")
    mismatch = []
    for line in manifest.read_text().splitlines():
        digest, rel = line.split("  ", 1)
        if sha256(DATA / rel) != digest:
            mismatch.append(rel)
    report = {
        "captures": rows,
        "sha_and_color_stamp_distinct": distinct,
        "manifest_mismatch": mismatch,
        "pcd_readback": "PASS",
        "rgb_readback": "PASS",
        "depth_readback": "PASS",
        "manifest": "PASS" if not mismatch else "FAIL",
    }
    (DATA / "integrity.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"distinct": distinct, "manifest": report["manifest"], "captures": len(rows)}))


if __name__ == "__main__":
    main()
