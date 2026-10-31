#!/usr/bin/env python3
"""P-008-R1J. R1I 촬영만 쓴다. R1H·R1I 파일은 쓰지 않는다."""

import json
import struct
import zlib
from collections import deque
from pathlib import Path

import numpy as np

from p008b_metrics import sha256_file
from p008r1h_features import (
    distribution,
    fit_logistic,
    grade,
    histogram_js,
    predict_logistic,
    rgb_to_hsv,
    rgb_to_lab,
    spatial_block_accuracy,
)

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "evidence" / "P-008" / "R1I" / "capture_dev"
OUT = ROOT / "evidence" / "P-008" / "R1J"
EXPECTED_SHA = "68b7b0a08164aeda291349be46dfbb3a6eacd66cd801d04d8b1a9a1025482377"
NAMES = ("RGB", "HSV", "Lab", "Height", "HSV+Height", "Lab+Height", "RGB+Height")
COLOR_NAMES = ("RGB", "HSV", "Lab")


def refuse(path):
    for token in ("capture_02", "capture_03", "capture_04", "capture_05"):
        if token in str(path):
            raise SystemExit(f"holdout 경로는 열 수 없다: {path}")


def read_png(path):
    refuse(path)
    data = Path(path).read_bytes()
    pos = 8
    width = height = None
    idat = b""
    while pos + 8 <= len(data):
        length = struct.unpack(">I", data[pos:pos + 4])[0]
        tag = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            width, height = struct.unpack(">II", chunk[:8])
        elif tag == b"IDAT":
            idat += chunk
        elif tag == b"IEND":
            break
    raw = zlib.decompress(idat)
    rows = []
    offset = 0
    stride = width * 3
    for _ in range(height):
        offset += 1
        rows.append(raw[offset:offset + stride])
        offset += stride
    return np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(height, width, 3)


def read_cloud(path):
    refuse(path)
    raw = path.read_bytes()
    header, body = raw.split(b"\nDATA binary\n", 1)
    text = header.decode("ascii")
    if "FIELDS x y z rgb" not in text:
        raise SystemExit("fields가 x y z rgb가 아니다.")
    count = int([line for line in text.splitlines() if line.startswith("POINTS ")][0].split()[1])
    packed = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")], count=count)
    xyz = np.column_stack((packed["x"], packed["y"], packed["z"])).astype(np.float64)
    rgb_bytes = np.frombuffer(np.ascontiguousarray(packed["rgb"]).tobytes(), dtype=np.uint8).reshape(count, 4)
    rgb = np.column_stack((rgb_bytes[:, 2], rgb_bytes[:, 1], rgb_bytes[:, 0])).astype(np.float64)
    return xyz, rgb


def ransac(points):
    rng = np.random.default_rng(1)
    best_count = -1
    best = None
    for _ in range(400):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        intercept = -float(normal @ sample[0])
        count = int(np.count_nonzero(np.abs(points @ normal + intercept) <= 0.004))
        if count > best_count:
            best_count = count
            best = (normal, intercept)
    normal, _intercept = best
    inliers = np.abs(points @ normal + _intercept) <= 0.004
    centered = points[inliers] - points[inliers].mean(axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    centroid = points[inliers].mean(axis=0)
    if normal @ (np.zeros(3) - centroid) < 0.0:
        normal = -normal
    intercept = -float(normal @ centroid)
    return normal, intercept


def basis(normal):
    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    axis_u = np.cross(normal, helper)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    return axis_u, axis_v


def horiz(points, origin, axis_u, axis_v):
    delta = points - origin
    return np.column_stack((delta @ axis_u, delta @ axis_v))


def near_mask(query, reference, limit):
    scale = 0.005
    buckets = {}
    keys = np.floor(reference / scale).astype(np.int64)
    for key, point in zip(map(tuple, keys), reference):
        buckets.setdefault(key, []).append(point)
    chosen = np.zeros(len(query), dtype=bool)
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
        if best is not None and best <= limit:
            chosen[index] = True
    return chosen


def blocks_of(u, v, mask):
    blocks = np.full(len(u), -1, dtype=np.int64)
    if mask.sum() == 0:
        return blocks
    local = (u[mask] > np.median(u[mask])).astype(np.int64) + 2 * (v[mask] > np.median(v[mask])).astype(np.int64)
    blocks[mask] = local
    return blocks


def features_of(name, rgb, hsv, lab, height):
    hue = hsv[:, 0] * 2.0 * np.pi
    table = {
        "RGB": rgb,
        "HSV": np.column_stack((np.sin(hue), np.cos(hue), hsv[:, 1], hsv[:, 2])),
        "Lab": lab,
        "Height": height.reshape(-1, 1),
        "HSV+Height": np.column_stack((np.sin(hue), np.cos(hue), hsv[:, 1], hsv[:, 2], height)),
        "Lab+Height": np.column_stack((lab, height)),
        "RGB+Height": np.column_stack((rgb, height)),
    }
    return table[name]


def evaluate(name, rgb, hsv, lab, height, u, v, positive, negative):
    mask = positive | negative
    labels = positive[mask].astype(np.int64)
    blocks = np.full(mask.sum(), -1, dtype=np.int64)
    shoe_blocks = blocks_of(u, v, positive)[mask]
    support_blocks = blocks_of(u, v, negative)[mask]
    blocks[labels == 1] = shoe_blocks[labels == 1]
    blocks[labels == 0] = support_blocks[labels == 0]
    matrix = features_of(name, rgb, hsv, lab, height)[mask]
    block = spatial_block_accuracy(matrix, labels, blocks)
    return {
        "spatial_block_balanced_accuracy": block["mean"],
        "valid_folds": block["valid_folds"],
        "folds": block["folds"],
        "grade": grade(block["mean"], block["valid_folds"]),
        "shoe_count": int(positive.sum()),
        "support_count": int(negative.sum()),
    }


def contact_errors(rgb, hsv, lab, height, u, v, positive, negative):
    mask = positive | negative
    labels = positive[mask].astype(np.int64)
    blocks = np.full(mask.sum(), -1, dtype=np.int64)
    shoe_blocks = blocks_of(u, v, positive)[mask]
    support_blocks = blocks_of(u, v, negative)[mask]
    blocks[labels == 1] = shoe_blocks[labels == 1]
    blocks[labels == 0] = support_blocks[labels == 0]
    matrix = features_of("Lab", rgb, hsv, lab, height)[mask]
    finite = np.isfinite(matrix).all(axis=1)
    wrong = np.zeros(mask.sum(), dtype=bool)
    for block_id in range(4):
        test = finite & (blocks == block_id)
        train = finite & (blocks != block_id)
        if test.sum() < 10 or train.sum() < 20:
            continue
        if len(np.unique(labels[train])) < 2 or len(np.unique(labels[test])) < 2:
            continue
        mean = matrix[train].mean(axis=0)
        std = matrix[train].std(axis=0)
        std[std < 1e-8] = 1.0
        weights = fit_logistic((matrix[train] - mean) / std, labels[train])
        predicted = predict_logistic(weights, (matrix[test] - mean) / std)
        wrong[test] = predicted != labels[test].astype(bool)
    full = np.zeros(len(u), dtype=bool)
    full[np.flatnonzero(mask)[wrong]] = True
    return full


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
    return int(best)


def histogram_png(left, right, edges, path):
    image = np.full((360, 720, 3), 245, dtype=np.uint8)
    ha, _ = np.histogram(left[np.isfinite(left)], bins=edges)
    hb, _ = np.histogram(right[np.isfinite(right)], bins=edges)
    peak = max(int(ha.max() if ha.size else 1), int(hb.max() if hb.size else 1), 1)
    width = 640 / max(len(ha), 1)
    for index, (count_a, count_b) in enumerate(zip(ha, hb)):
        x0 = int(40 + index * width)
        x1 = max(x0 + 1, int(40 + (index + 1) * width))
        ya = 300 - int(240 * count_a / peak)
        yb = 300 - int(240 * count_b / peak)
        image[ya:300, x0:x0 + max(1, (x1 - x0) // 2)] = (40, 90, 180)
        image[yb:300, x0 + (x1 - x0) // 2:x1] = (180, 120, 30)
    image[300:302, 40:680] = 40
    # write png via the same zlib path as the rest of the repo
    from p008b_metrics import write_png
    write_png(path, image)


def main():
    cloud_path = CAPTURE / "cloud.pcd"
    digest = sha256_file(cloud_path)
    if digest != EXPECTED_SHA:
        raise SystemExit("BLOCKED_INPUT_CONTRACT")
    info = json.loads((CAPTURE / "color_camera_info.json").read_text())
    if info["frame_id"] != "camera_color_optical_frame" or info["width"] != 1280 or info["height"] != 720:
        raise SystemExit("BLOCKED_INPUT_CONTRACT")
    method = json.loads((OUT / "methodology.json").read_text())
    xyz, rgb = read_cloud(cloud_path)
    finite = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
    if int(finite.sum()) != 756376:
        raise SystemExit(f"BLOCKED_INPUT_CONTRACT finite={int(finite.sum())}")
    xyz, rgb = xyz[finite], rgb[finite]
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    u = fx * xyz[:, 0] / xyz[:, 2] + cx
    v = fy * xyz[:, 1] / xyz[:, 2] + cy
    plane_box = {"u0": 150, "u1": 950, "v0": 250, "v1": 620}
    plane_sel = (u >= plane_box["u0"]) & (u < plane_box["u1"]) & (v >= plane_box["v0"]) & (v < plane_box["v1"])
    normal, intercept = ransac(xyz[plane_sel])
    recorded = np.array([0.004095, -0.687865, -0.725827, 0.527413])
    got = np.array([normal[0], normal[1], normal[2], intercept])
    height = xyz @ normal + intercept
    hsv = rgb_to_hsv(rgb)
    lab = rgb_to_lab(rgb)
    shoe_box = method["shoe_interior_px"]
    wood_box = method["support_interior_px"]
    shoe = (u >= shoe_box["u0"]) & (u < shoe_box["u1"]) & (v >= shoe_box["v0"]) & (v < shoe_box["v1"])
    wood = (u >= wood_box["u0"]) & (u < wood_box["u1"]) & (v >= wood_box["v0"]) & (v < wood_box["v1"])
    window = method["contact_bands"]["window_px"]
    in_window = (u >= window["u0"]) & (u < window["u1"]) & (v >= window["v0"]) & (v < window["v1"])
    support_band = in_window & (np.abs(height) <= 0.004)
    shoe_band = in_window & (height > 0.004) & (height <= 0.020)
    origin = xyz[support_band].mean(axis=0) if support_band.any() else xyz.mean(axis=0)
    axis_u, axis_v = basis(normal)
    support_h = horiz(xyz[support_band], origin, axis_u, axis_v)
    shoe_h = horiz(xyz[shoe_band], origin, axis_u, axis_v)
    shoe_near = near_mask(shoe_h, support_h, 0.015)
    support_near = near_mask(support_h, shoe_h, 0.015)
    contact_shoe = np.zeros(len(xyz), dtype=bool)
    contact_support = np.zeros(len(xyz), dtype=bool)
    contact_shoe[np.flatnonzero(shoe_band)[shoe_near]] = True
    contact_support[np.flatnonzero(support_band)[support_near]] = True
    wood_h = horiz(xyz[wood], origin, axis_u, axis_v)
    shoe_contact_h = horiz(xyz[contact_shoe], origin, axis_u, axis_v)
    if len(wood_h) and len(shoe_contact_h):
        wood_near = near_mask(wood_h, shoe_contact_h, 0.015)
        wood_inside_contact = int(wood_near.sum())
    else:
        wood_inside_contact = 0
    interior = {}
    contact = {}
    for name in NAMES:
        interior[name] = evaluate(name, rgb, hsv, lab, height, u, v, shoe, wood)
        contact[name] = evaluate(name, rgb, hsv, lab, height, u, v, contact_shoe, contact_support)
        contact[name]["height_label_leakage"] = "YES" if name == "Height" else ("PARTIAL" if "Height" in name else "NO")
    errors = contact_errors(rgb, hsv, lab, height, u, v, contact_shoe, contact_support)
    error_component = largest_component(xyz[errors], 0.01)
    color = read_png(CAPTURE / "color.png")
    zoom_box = window
    zoom = color[zoom_box["v0"]:zoom_box["v1"], zoom_box["u0"]:zoom_box["u1"]].copy()
    uu = np.rint(u[errors]).astype(np.int64) - zoom_box["u0"]
    vv = np.rint(v[errors]).astype(np.int64) - zoom_box["v0"]
    ok = (uu >= 0) & (uu < zoom.shape[1]) & (vv >= 0) & (vv < zoom.shape[0])
    zoom[vv[ok], uu[ok]] = (220, 30, 30)
    from p008b_metrics import write_png
    write_png(OUT / "contact_error.png", zoom)
    histogram_png(lab[shoe, 0], lab[wood, 0], np.linspace(0, 100, 33), OUT / "lab_interior.png")
    histogram_png(rgb[contact_shoe, 2], rgb[contact_support, 2], np.linspace(0, 256, 33), OUT / "contact_blue_channel.png")
    low_sat = {
        "contact_shoe_median_s": float(np.median(hsv[contact_shoe, 1])) if contact_shoe.any() else None,
        "contact_support_median_s": float(np.median(hsv[contact_support, 1])) if contact_support.any() else None,
        "contact_low_saturation_fraction": float(np.mean(hsv[contact_shoe | contact_support, 1] < 0.1)),
    }
    best_color = max(COLOR_NAMES, key=lambda name: contact[name]["spatial_block_balanced_accuracy"] or 0.0)
    color_contact_high = contact[best_color]["grade"] == "HIGH" and interior[best_color]["grade"] == "HIGH"
    damage = error_component >= 100
    if wood_inside_contact > 0:
        verdict = "NOT_ROBUSTLY_SEPARABLE"
        family = "NONE"
        reason = "support_interior_intersects_contact_band"
    elif color_contact_high and not damage:
        verdict = "COLOR_SEPARABLE"
        family = best_color
        reason = "color_high_on_pixel_interior_and_contact"
    else:
        verdict = "NOT_ROBUSTLY_SEPARABLE"
        family = "NONE"
        reason = "contact_color_not_independently_high_or_spatial_damage"
    payload = {
        "input_sha256": digest,
        "finite_xyz": 756376,
        "plane": {
            "normal": normal.tolist(),
            "d": float(intercept),
            "abs_delta_vs_r1i": np.abs(got - recorded).tolist(),
        },
        "support_interior_inside_15mm": wood_inside_contact,
        "interior": interior,
        "contact": contact,
        "low_saturation": low_sat,
        "lab_contact_error_points": int(errors.sum()),
        "lab_contact_error_largest_component": error_component,
        "downstream_geometry_risk": bool(damage),
        "height_label_leakage_contact": "YES",
        "feature_separability": verdict,
        "selected_feature_family": family,
        "reason": reason,
        "best_color_space": best_color,
    }
    (OUT / "separability.json").write_text(json.dumps(payload, indent=2) + "\n")
    (OUT / "interior_height.json").write_text(json.dumps({
        "shoe": distribution(height[shoe]),
        "support": distribution(height[wood]),
        "js": histogram_js(height[shoe], height[wood], np.linspace(-0.02, 0.18, 33)),
    }, indent=2) + "\n")
    (OUT / "contact_color.json").write_text(json.dumps({
        "shoe_lab": distribution(lab[contact_shoe, 0]),
        "support_lab_l": distribution(lab[contact_support, 0]),
        "shoe_b": distribution(lab[contact_shoe, 2]),
        "support_b": distribution(lab[contact_support, 2]),
        "low_saturation": low_sat,
    }, indent=2) + "\n")
    print(json.dumps({
        "verdict": verdict,
        "family": family,
        "reason": reason,
        "wood_inside": wood_inside_contact,
        "shoe_box": int(shoe.sum()),
        "wood_box": int(wood.sum()),
        "contact_shoe": int(contact_shoe.sum()),
        "contact_support": int(contact_support.sum()),
        "error_component": error_component,
        "interior_lab": interior["Lab"]["grade"],
        "contact_lab": contact["Lab"]["grade"],
        "contact_height": contact["Height"]["grade"],
        "plane_delta": payload["plane"]["abs_delta_vs_r1i"],
    }))


if __name__ == "__main__":
    main()
