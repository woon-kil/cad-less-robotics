#!/usr/bin/env python3
"""R1I 접촉 관측. 띠 정의는 바꾸지 않는다. segmentation은 없다."""

import json
from collections import deque
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/ws/cad-less-robotics/evidence/P-008/R1I")
CAPTURE = ROOT / "capture_dev"


def load_pcd(path):
    blob = path.read_bytes()
    marker = blob.index(b"DATA binary\n") + len(b"DATA binary\n")
    packed = np.frombuffer(blob[marker:], dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")]))
    xyz = np.column_stack((packed["x"], packed["y"], packed["z"])).astype(np.float64)
    raw = np.frombuffer(np.ascontiguousarray(packed["rgb"]).tobytes(), dtype=np.uint8).reshape(-1, 4)
    rgb = np.column_stack((raw[:, 2], raw[:, 1], raw[:, 0])).astype(np.uint8)
    return xyz, rgb


def stats(height):
    if height.size == 0:
        return {"count": 0}
    return {
        "count": int(height.size),
        "mean": float(height.mean()),
        "median": float(np.median(height)),
        "p05": float(np.percentile(height, 5)),
        "p95": float(np.percentile(height, 95)),
    }


def ransac(points):
    rng = np.random.default_rng(1)
    best_count = 0
    best = None
    n = len(points)
    for _ in range(400):
        sample = points[rng.choice(n, 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -float(normal @ sample[0])
        dist = np.abs(points @ normal + d)
        count = int(np.count_nonzero(dist <= 0.004))
        if count > best_count:
            best_count = count
            best = (normal, d)
    normal, d = best
    inliers = np.abs(points @ normal + d) <= 0.004
    centered = points[inliers] - points[inliers].mean(axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    centroid = points[inliers].mean(axis=0)
    if normal @ (np.zeros(3) - centroid) < 0.0:
        normal = -normal
    d = -float(normal @ centroid)
    residual = np.abs(points[inliers] @ normal + d)
    return normal, d, inliers, residual


def plane_basis(normal):
    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    axis_u = np.cross(normal, helper)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    return axis_u, axis_v


def horizontal(points, origin, axis_u, axis_v):
    delta = points - origin
    return np.column_stack((delta @ axis_u, delta @ axis_v))


def near_mask(query, reference, limit):
    if len(query) == 0 or len(reference) == 0:
        return np.zeros(len(query), dtype=bool), np.full(len(query), np.nan)
    scale = 0.005
    keys = np.floor(reference / scale).astype(np.int64)
    buckets = {}
    for key, point in zip(map(tuple, keys), reference):
        buckets.setdefault(key, []).append(point)
    chosen = np.zeros(len(query), dtype=bool)
    nearest = np.full(len(query), np.nan)
    span = int(np.ceil(limit / scale))
    for index, point in enumerate(query):
        base = np.floor(point / scale).astype(np.int64)
        best = None
        for du in range(-span, span + 1):
            for dv in range(-span, span + 1):
                bucket = buckets.get((int(base[0] + du), int(base[1] + dv)))
                if not bucket:
                    continue
                for other in bucket:
                    dist = float(np.linalg.norm(point - other))
                    if best is None or dist < best:
                        best = dist
        if best is not None:
            nearest[index] = best
            chosen[index] = best <= limit
    return chosen, nearest


def largest_component(points, voxel):
    if len(points) == 0:
        return 0
    keys = np.floor(points / voxel).astype(np.int64)
    cells = {}
    for key in map(tuple, keys):
        cells[key] = cells.get(key, 0) + 1
    seen = set()
    best = 0
    for start in cells:
        if start in seen:
            continue
        count = 0
        queue = deque([start])
        seen.add(start)
        while queue:
            current = queue.popleft()
            count += cells[current]
            for step in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
                nxt = (current[0] + step[0], current[1] + step[1], current[2] + step[2])
                if nxt in cells and nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        best = max(best, count)
    return best


def main():
    definition = json.loads((ROOT / "contact_band_definition.json").read_text())
    info = json.loads((CAPTURE / "color_camera_info.json").read_text())
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    xyz, rgb = load_pcd(CAPTURE / "cloud.pcd")
    finite = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0.0)
    xyz, rgb = xyz[finite], rgb[finite]
    u = fx * xyz[:, 0] / xyz[:, 2] + cx
    v = fy * xyz[:, 1] / xyz[:, 2] + cy
    plane_box = definition["plane_fit_window_px"]
    contact_box = definition["pixel_window"]
    plane_sel = (
        (u >= plane_box["u0"]) & (u < plane_box["u1"]) & (v >= plane_box["v0"]) & (v < plane_box["v1"])
    )
    normal, intercept, inlier_local, residual = ransac(xyz[plane_sel])
    height = xyz @ normal + intercept
    support = np.abs(height) <= definition["support_inlier_abs_height_m"]
    shoe_band = (height > definition["shoe_side_height_m"][0]) & (height <= definition["shoe_side_height_m"][1])
    contact = (
        (u >= contact_box["u0"]) & (u < contact_box["u1"]) & (v >= contact_box["v0"]) & (v < contact_box["v1"])
    )
    origin = xyz[support].mean(axis=0)
    axis_u, axis_v = plane_basis(normal)
    support_idx = np.flatnonzero(contact & support)
    shoe_idx = np.flatnonzero(contact & shoe_band)
    support_uv = horizontal(xyz[support_idx], origin, axis_u, axis_v)
    shoe_uv = horizontal(xyz[shoe_idx], origin, axis_u, axis_v)
    shoe_near, shoe_dist = near_mask(shoe_uv, support_uv, definition["horizontal_near_m"])
    support_near, support_dist = near_mask(support_uv, shoe_uv, definition["horizontal_near_m"])
    shoe_pts = xyz[shoe_idx][shoe_near]
    support_pts = xyz[support_idx][support_near]
    shoe_component = largest_component(shoe_pts, definition["contiguity_voxel_m"])
    support_component = largest_component(support_pts, definition["contiguity_voxel_m"])
    both = len(shoe_pts) >= definition["min_points_per_side"] and len(support_pts) >= definition["min_points_per_side"]
    contiguous = shoe_component >= definition["min_points_per_side"] and support_component >= definition["min_points_per_side"]
    report = {
        "plane": {
            "normal": normal.tolist(),
            "d": float(intercept),
            "inlier_count": int(inlier_local.sum()),
            "candidate_count": int(plane_sel.sum()),
            "inlier_ratio": float(inlier_local.sum() / plane_sel.sum()),
            "residual_median_m": float(np.median(residual)),
            "residual_p95_m": float(np.percentile(residual, 95)),
        },
        "shoe_side": {
            **stats(height[shoe_idx][shoe_near]),
            "rgb_available": True,
            "nearest_horizontal_m": {
                "min": None if not np.isfinite(shoe_dist[shoe_near]).any() else float(np.nanmin(shoe_dist[shoe_near])),
                "median": None if not np.isfinite(shoe_dist[shoe_near]).any() else float(np.nanmedian(shoe_dist[shoe_near])),
            },
            "largest_component": int(shoe_component),
        },
        "support_side": {
            **stats(height[support_idx][support_near]),
            "rgb_available": True,
            "nearest_horizontal_m": {
                "min": None if not np.isfinite(support_dist[support_near]).any() else float(np.nanmin(support_dist[support_near])),
                "median": None if not np.isfinite(support_dist[support_near]).any() else float(np.nanmedian(support_dist[support_near])),
            },
            "largest_component": int(support_component),
        },
        "contact_window_support_inliers": int(len(support_idx)),
        "contact_window_shoe_band": int(len(shoe_idx)),
        "contiguous": bool(contiguous),
        "both_sides_present": bool(both and contiguous),
    }
    (ROOT / "contact_metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    color = cv2.imread(str(CAPTURE / "color.png"), cv2.IMREAD_COLOR)
    box = contact_box
    zoom = color[box["v0"]:box["v1"], box["u0"]:box["u1"]].copy()
    def paint(index, flag, bgr):
        uu = np.rint(u[index][flag]).astype(np.int64) - box["u0"]
        vv = np.rint(v[index][flag]).astype(np.int64) - box["v0"]
        ok = (uu >= 0) & (uu < zoom.shape[1]) & (vv >= 0) & (vv < zoom.shape[0])
        zoom[vv[ok], uu[ok]] = bgr
    paint(support_idx, support_near, (0, 180, 255))
    paint(shoe_idx, shoe_near, (255, 80, 0))
    cv2.imwrite(str(ROOT / "contact_rgb_zoom.png"), zoom)
    height_view = np.full(zoom.shape[:2], np.nan, dtype=np.float64)
    sel = contact & np.isfinite(height)
    uu = np.rint(u[sel]).astype(np.int64) - box["u0"]
    vv = np.rint(v[sel]).astype(np.int64) - box["v0"]
    ok = (uu >= 0) & (uu < zoom.shape[1]) & (vv >= 0) & (vv < zoom.shape[0])
    height_view[vv[ok], uu[ok]] = height[sel][ok]
    shown = np.clip((height_view + 0.005) / 0.03, 0, 1)
    shown[~np.isfinite(height_view)] = 0
    heat = cv2.applyColorMap((shown * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    heat[~np.isfinite(height_view)] = 0
    cv2.imwrite(str(ROOT / "contact_height.png"), heat)
    print(json.dumps({
        "shoe": report["shoe_side"]["count"],
        "support": report["support_side"]["count"],
        "both": report["both_sides_present"],
        "inlier_ratio": report["plane"]["inlier_ratio"],
    }))


if __name__ == "__main__":
    main()
