#!/usr/bin/env python3
"""P-008-R1 그림. 기하를 다시 계산하지 않고 저장된 단계만 그린다."""

import json
import struct
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from p008r1_metrics import distribution, spacing  # noqa: E402


def load_xyz(path):
    raw = Path(path).read_bytes()
    count = struct.unpack_from("<I", raw, 0)[0]
    return np.frombuffer(raw, dtype="<f4", count=count * 3, offset=4).reshape(count, 3).astype(np.float64)


def load_clusters(path):
    raw = Path(path).read_bytes()
    count = struct.unpack_from("<I", raw, 0)[0]
    xyz = np.empty((count, 3), dtype=np.float64)
    ids = np.empty(count, dtype=np.int32)
    offset = 4
    for index in range(count):
        xyz[index] = struct.unpack_from("<fff", raw, offset)
        ids[index] = struct.unpack_from("<i", raw, offset + 12)[0]
        offset += 16
    return xyz, ids


def load_pcd_xyz(path):
    raw = Path(path).read_bytes()
    marker = raw.find(b"DATA binary\n")
    header = raw[:marker].decode("ascii")
    points = int([line for line in header.splitlines() if line.startswith("POINTS ")][0].split()[1])
    body = raw[marker + len(b"DATA binary\n"):]
    return np.frombuffer(body, dtype="<f4", count=points * 3).reshape(points, 3).astype(np.float64)


def canvas(points, color_of, title, path, extra=None):
    width, height = 960, 720
    image = Image.new("RGB", (width, height), (18, 18, 20))
    draw = ImageDraw.Draw(image)
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.shape[0] == 0:
        image.save(path)
        return
    stride = max(1, xyz.shape[0] // 70000)
    xyz = xyz[::stride]
    x = xyz[:, 0]
    y = xyz[:, 1]
    xmin, xmax = np.percentile(x, [1, 99])
    ymin, ymax = np.percentile(y, [1, 99])
    if xmax <= xmin:
        xmax = xmin + 1e-6
    if ymax <= ymin:
        ymax = ymin + 1e-6
    px = ((x - xmin) / (xmax - xmin) * (width - 40) + 20).astype(np.int32)
    py = ((y - ymin) / (ymax - ymin) * (height - 70) + 20).astype(np.int32)
    colors = color_of(xyz)
    for index in range(xyz.shape[0]):
        draw.point((int(px[index]), int(py[index])), fill=colors[index])
    if extra:
        extra(draw, xmin, xmax, ymin, ymax, width, height)
    draw.text((16, height - 28), title, fill=(230, 230, 230))
    image.save(path)


def tone(values):
    low, high = np.percentile(values, [2, 98])
    if high <= low:
        high = low + 1.0
    scale = np.clip((values - low) / (high - low), 0, 1)
    return [(int(40 + 180 * item), int(80 + 100 * (1 - item)), int(220 - 160 * item)) for item in scale]


def main():
    out = Path(sys.argv[1])
    raw_pcd = Path(sys.argv[2])
    figures = out / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    raw = load_pcd_xyz(raw_pcd)
    denoised = load_xyz(out / "stage_voxel_sor.bin")
    inliers = load_xyz(out / "stage_plane_inliers.bin")
    above = load_xyz(out / "stage_plane_removed.bin")
    clusters, ids = load_clusters(out / "stage_clusters.bin")
    shoe = load_xyz(out / "stage_shoe.bin")
    pca = json.loads((out / "pca_frame.json").read_text())
    boundary = np.loadtxt(out / "boundary_3d.csv", delimiter=",", skiprows=1)
    if boundary.ndim == 1:
        boundary = boundary.reshape(1, -1)
    uv = np.loadtxt(out / "boundary_2d.csv", delimiter=",", skiprows=1)
    support = np.loadtxt(out / "boundary_support.csv", delimiter=",", skiprows=1)
    gaps = np.loadtxt(out / "boundary_spacing.csv", delimiter=",", skiprows=1)
    summary = json.loads((out / "boundary_summary.json").read_text())
    worst = int(summary["worst_gap_index"])

    def depth_color(xyz):
        return tone(xyz[:, 2])

    canvas(raw, depth_color, "01 raw cloud", figures / "01_raw_cloud.png")
    canvas(denoised, depth_color, "02 voxel + SOR", figures / "02_voxel_sor.png")
    both = np.vstack([inliers, above]) if above.size and inliers.size else denoised
    def plane_color(xyz):
        colors = []
        split = inliers.shape[0]
        for index in range(xyz.shape[0]):
            colors.append((150, 150, 150) if index < split else (40, 180, 90))
        return colors
    # canvas stride breaks the split. Draw inliers and remaining separately.
    image = Image.new("RGB", (960, 720), (18, 18, 20))
    draw = ImageDraw.Draw(image)
    stack = np.vstack([inliers[::max(1, inliers.shape[0] // 40000)], above[::max(1, max(above.shape[0], 1) // 30000)]])
    flags = np.concatenate([
        np.zeros(inliers[::max(1, inliers.shape[0] // 40000)].shape[0], dtype=np.int32),
        np.ones(above[::max(1, max(above.shape[0], 1) // 30000)].shape[0], dtype=np.int32),
    ])
    x, y = stack[:, 0], stack[:, 1]
    xmin, xmax = np.percentile(x, [1, 99])
    ymin, ymax = np.percentile(y, [1, 99])
    px = ((x - xmin) / (xmax - xmin + 1e-9) * 920 + 20).astype(np.int32)
    py = ((y - ymin) / (ymax - ymin + 1e-9) * 630 + 20).astype(np.int32)
    for index in range(stack.shape[0]):
        color = (140, 140, 140) if flags[index] == 0 else (40, 190, 90)
        draw.point((int(px[index]), int(py[index])), fill=color)
    draw.text((16, 692), "03 support plane gray, remaining green", fill=(230, 230, 230))
    image.save(figures / "03_support_plane.png")
    canvas(above, depth_color, "04 plane removed", figures / "04_plane_removed.png")

    palette = [(220, 80, 70), (70, 140, 230), (240, 190, 60), (160, 90, 220), (80, 200, 190), (240, 130, 60)]
    def cluster_color(xyz):
        # unused, custom draw below
        return [(255, 255, 255)] * xyz.shape[0]
    image = Image.new("RGB", (960, 720), (18, 18, 20))
    draw = ImageDraw.Draw(image)
    sample = clusters[::max(1, clusters.shape[0] // 60000)]
    sample_ids = ids[::max(1, clusters.shape[0] // 60000)]
    x, y = sample[:, 0], sample[:, 1]
    xmin, xmax = np.percentile(x, [1, 99])
    ymin, ymax = np.percentile(y, [1, 99])
    px = ((x - xmin) / (xmax - xmin + 1e-9) * 920 + 20).astype(np.int32)
    py = ((y - ymin) / (ymax - ymin + 1e-9) * 630 + 20).astype(np.int32)
    selected_id = int(summary["selected_cluster_id"])
    for index in range(sample.shape[0]):
        cid = int(sample_ids[index])
        color = (255, 255, 255) if cid == selected_id else palette[cid % len(palette)]
        draw.point((int(px[index]), int(py[index])), fill=color)
    draw.text((16, 692), f"05 clusters, selected id {selected_id} is white", fill=(230, 230, 230))
    image.save(figures / "05_clusters.png")
    canvas(shoe, depth_color, "06 selected shoe", figures / "06_selected_shoe.png")

    centroid = np.array(pca["centroid"], dtype=np.float64)
    axis0 = np.array(pca["axis_0"], dtype=np.float64)
    axis1 = np.array(pca["axis_1"], dtype=np.float64)
    image = Image.new("RGB", (960, 720), (18, 18, 20))
    draw = ImageDraw.Draw(image)
    view = shoe[::max(1, shoe.shape[0] // 40000)]
    x, y = view[:, 0], view[:, 1]
    xmin, xmax = np.percentile(x, [1, 99])
    ymin, ymax = np.percentile(y, [1, 99])
    def map_point(point):
        px = int((point[0] - xmin) / (xmax - xmin + 1e-9) * 920 + 20)
        py = int((point[1] - ymin) / (ymax - ymin + 1e-9) * 630 + 20)
        return px, py
    for point in view:
        draw.point(map_point(point), fill=(80, 160, 210))
    origin = map_point(centroid)
    draw.line([origin, map_point(centroid + axis0 * 0.08)], fill=(240, 80, 80), width=3)
    draw.line([origin, map_point(centroid + axis1 * 0.08)], fill=(80, 220, 120), width=3)
    draw.text((16, 692), "07 PCA frame, red axis0 green axis1", fill=(230, 230, 230))
    image.save(figures / "07_pca_frame.png")

    projected = np.loadtxt(out / "projected_points.csv", delimiter=",", skiprows=1)
    image = Image.new("RGB", (960, 720), (18, 18, 20))
    draw = ImageDraw.Draw(image)
    uu, vv = projected[::max(1, projected.shape[0] // 40000), 0], projected[::max(1, projected.shape[0] // 40000), 1]
    umin, umax = uu.min(), uu.max()
    vmin, vmax = vv.min(), vv.max()
    for u, v in zip(uu, vv):
        px = int((u - umin) / (umax - umin + 1e-9) * 920 + 20)
        py = int((v - vmin) / (vmax - vmin + 1e-9) * 630 + 20)
        draw.point((px, py), fill=(180, 190, 210))
    draw.text((16, 692), "08 PCA projection", fill=(230, 230, 230))
    image.save(figures / "08_projection.png")

    image = Image.new("RGB", (960, 720), (18, 18, 20))
    draw = ImageDraw.Draw(image)
    poly = []
    for u, v in uv[:, 1:3] if uv.shape[1] >= 3 else uv[:, :2]:
        px = int((u - umin) / (umax - umin + 1e-9) * 920 + 20)
        py = int((v - vmin) / (vmax - vmin + 1e-9) * 630 + 20)
        poly.append((px, py))
    if len(poly) >= 2:
        draw.line(poly + [poly[0]], fill=(240, 200, 60), width=2)
    draw.text((16, 692), "09 concave boundary 2d", fill=(230, 230, 230))
    image.save(figures / "09_boundary_2d.png")

    canvas(shoe, depth_color, "10 boundary on shoe", figures / "10_boundary_on_3d_shoe.png")
    # overlay boundary by redrawing on a copy is heavier; draw shoe then boundary in one image
    image = Image.open(figures / "10_boundary_on_3d_shoe.png")
    draw = ImageDraw.Draw(image)
    view = shoe
    x, y = view[:, 0], view[:, 1]
    xmin, xmax = np.percentile(x[::max(1, x.shape[0] // 40000)], [1, 99])
    ymin, ymax = np.percentile(y[::max(1, y.shape[0] // 40000)], [1, 99])
    def map_b(point):
        return (
            int((point[0] - xmin) / (xmax - xmin + 1e-9) * 920 + 20),
            int((point[1] - ymin) / (ymax - ymin + 1e-9) * 630 + 20),
        )
    ring = [map_b(point) for point in boundary[:, 1:4]]
    if len(ring) >= 2:
        draw.line(ring + [ring[0]], fill=(255, 220, 40), width=2)
    image.save(figures / "10_boundary_on_3d_shoe.png")

    image = Image.new("RGB", (960, 720), (18, 18, 20))
    draw = ImageDraw.Draw(image)
    distances = support[:, 1]
    colors = tone(distances)
    for index, point in enumerate(ring):
        draw.ellipse([point[0] - 2, point[1] - 2, point[0] + 2, point[1] + 2], fill=colors[index])
    draw.text((16, 692), "11 boundary support distance", fill=(230, 230, 230))
    image.save(figures / "11_boundary_support.png")

    image = Image.open(figures / "10_boundary_on_3d_shoe.png").copy()
    draw = ImageDraw.Draw(image)
    a = ring[worst]
    b = ring[(worst + 1) % len(ring)]
    draw.line([a, b], fill=(255, 40, 40), width=4)
    draw.text((16, 660), f"12 worst gap index {worst}", fill=(255, 180, 180))
    image.save(figures / "12_boundary_gap_diagnostics.png")

    gap_values = spacing(boundary[:, 1:4], closed=True)
    stats = {
        "support": distribution(distances),
        "spacing": distribution(gap_values),
        "worst_gap_index": worst,
        "worst_gap_matches_summary": abs(float(np.max(gap_values)) - float(summary["worst_gap_m"])) < 1e-9,
    }
    (out / "metric_check.json").write_text(json.dumps(stats, indent=2) + "\n")
    print(json.dumps(stats))


if __name__ == "__main__":
    main()
