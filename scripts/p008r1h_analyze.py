#!/usr/bin/env python3
"""P-008-R1H. capture_01의 진단 영역만 측정한다.

capture_02~05 경로는 거부한다. 진단 사각형은 바꾸지 않는다.
"""

import json
import struct
import zlib
from pathlib import Path

import numpy as np

from p008b_metrics import sha256_file, write_png
from p008r1h_features import (
    CONTACT_SHOE_HEIGHT,
    CONTACT_SHOE_HORIZONTAL_MAX,
    CONTACT_SUPPORT_HEIGHT_MAX,
    CONTACT_SUPPORT_HORIZONTAL,
    MIN_CONTACT_POINTS,
    bhattacharyya,
    circular_mean_deg,
    distribution,
    grade,
    histogram_js,
    plane_height,
    point_in_polygon,
    rgb_to_hsv,
    rgb_to_lab,
    select_method,
    spatial_block_accuracy,
)

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "evidence" / "P-008B" / "capture_01"
OUT = ROOT / "evidence" / "P-008" / "R1H"
FROZEN_REGION_SHA = "a5a4b6bd125f5bcc1bb2ef5ffa1beb19e40f557f632f08f05fc674c1e84c2a19"
RGB_EDGES = np.linspace(0.0, 256.0, 33)
UNIT_EDGES = np.linspace(0.0, 1.0, 33)
LAB_L_EDGES = np.linspace(0.0, 100.0, 33)
LAB_AB_EDGES = np.linspace(-128.0, 127.0, 33)
HEIGHT_EDGES = np.linspace(-0.02, 0.18, 33)


def refuse(path):
    text = str(path)
    for token in ("capture_02", "capture_03", "capture_04", "capture_05"):
        if token in text:
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
    stride = width * 3
    rows = []
    offset = 0
    for _ in range(height):
        offset += 1
        rows.append(raw[offset:offset + stride])
        offset += stride
    return np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(height, width, 3)


def read_cloud(path):
    refuse(path)
    raw = Path(path).read_bytes()
    header, body = raw.split(b"\nDATA binary\n", 1)
    text = header.decode("ascii")
    if "FIELDS x y z rgb" not in text:
        raise SystemExit("capture_01 cloud fields가 x y z rgb가 아니다.")
    count = int([line for line in text.splitlines() if line.startswith("POINTS ")][0].split()[1])
    packed = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")], count=count)
    xyz = np.column_stack((packed["x"], packed["y"], packed["z"])).astype(np.float64)
    rgb_bytes = np.ascontiguousarray(packed["rgb"]).view(np.uint8).reshape(count, 4)
    rgb = np.column_stack((rgb_bytes[:, 2], rgb_bytes[:, 1], rgb_bytes[:, 0])).astype(np.float64)
    return xyz, rgb


def plane_axes(coefficients):
    normal = np.asarray(coefficients[:3], dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    axis_u = np.cross(helper, normal)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    return axis_u, axis_v


def to_uv(points, origin, axis_u, axis_v):
    delta = points - origin
    return np.column_stack((delta @ axis_u, delta @ axis_v))


def blocks_of(u, v, mask):
    blocks = np.full(u.shape[0], -1, dtype=np.int64)
    if mask.sum() == 0:
        return blocks
    median_u = np.median(u[mask])
    median_v = np.median(v[mask])
    local = (u[mask] > median_u).astype(np.int64) + 2 * (v[mask] > median_v).astype(np.int64)
    blocks[mask] = local
    return blocks


def combine_blocks(u, v, shoe, support):
    blocks = np.full(u.shape[0], -1, dtype=np.int64)
    blocks[shoe] = blocks_of(u, v, shoe)[shoe]
    blocks[support] = blocks_of(u, v, support)[support]
    return blocks


def nearest_horizontal(query, reference, leaf=0.002):
    if len(reference) == 0 or len(query) == 0:
        return np.full(len(query), np.inf)
    keys = np.unique(np.floor(reference / leaf).astype(np.int64), axis=0)
    centers = (keys.astype(np.float64) + 0.5) * leaf
    distances = np.empty(len(query), dtype=np.float64)
    for start in range(0, len(query), 4000):
        chunk = query[start:start + 4000]
        delta = chunk[:, None, :] - centers[None, :, :]
        distances[start:start + 4000] = np.linalg.norm(delta, axis=2).min(axis=1)
    return distances


def channel_metrics(left, right, edges):
    return {
        "js_divergence_log2": histogram_js(left, right, edges),
        "bhattacharyya_distance": bhattacharyya(left, right, edges),
        "shoe": distribution(left),
        "support": distribution(right),
    }


def feature_matrix(name, rgb, hsv, lab, height, mask):
    hue = hsv[:, 0] * 2.0 * np.pi
    columns = {
        "RGB": rgb,
        "HSV": np.column_stack((np.sin(hue), np.cos(hue), hsv[:, 1], hsv[:, 2])),
        "Lab": lab,
        "Height": height.reshape(-1, 1),
        "HSV+Height": np.column_stack((np.sin(hue), np.cos(hue), hsv[:, 1], hsv[:, 2], height)),
        "Lab+Height": np.column_stack((lab, height)),
        "RGB+Height": np.column_stack((rgb, height)),
    }
    return columns[name][mask]


def evaluate_pair(name, rgb, hsv, lab, height, u, v, positive, negative):
    mask = positive | negative
    labels = positive[mask].astype(np.int64)
    blocks = combine_blocks(u, v, positive, negative)[mask]
    matrix = feature_matrix(name, rgb, hsv, lab, height, mask)
    block = spatial_block_accuracy(matrix, labels, blocks)
    return {
        "spatial_block_balanced_accuracy": block["mean"],
        "valid_folds": block["valid_folds"],
        "folds": block["folds"],
        "grade": grade(block["mean"], block["valid_folds"]),
        "point_count": int(mask.sum()),
    }


def draw_box(image, polygon, color):
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    image[y0:y0 + 2, x0:x1] = color
    image[y1 - 2:y1, x0:x1] = color
    image[y0:y1, x0:x0 + 2] = color
    image[y0:y1, x1 - 2:x1] = color


def histogram_figure(left, right, edges, title_rows):
    image = np.full((360, 720, 3), 245, dtype=np.uint8)
    ha, _ = np.histogram(left[np.isfinite(left)], bins=edges)
    hb, _ = np.histogram(right[np.isfinite(right)], bins=edges)
    peak = max(int(ha.max() if ha.size else 1), int(hb.max() if hb.size else 1), 1)
    width = 640 / max(len(ha), 1)
    for index, (count_a, count_b) in enumerate(zip(ha, hb)):
        x0 = int(40 + index * width)
        x1 = int(40 + (index + 1) * width)
        ya = 300 - int(240 * count_a / peak)
        yb = 300 - int(240 * count_b / peak)
        image[ya:300, x0:x0 + max((x1 - x0) // 2, 1)] = (40, 70, 160)
        image[yb:300, x0 + max((x1 - x0) // 2, 1):x1] = (40, 140, 70)
    image[300:302, 40:680] = (20, 20, 20)
    return image


def scatter_figure(x0, y0, x1, y1):
    image = np.full((480, 640, 3), 250, dtype=np.uint8)

    def paint(x, y, color):
        finite = np.isfinite(x) & np.isfinite(y)
        if finite.sum() == 0:
            return
        xs = x[finite]
        ys = y[finite]
        span_x = np.percentile(xs, 99) - np.percentile(xs, 1)
        span_y = np.percentile(ys, 99) - np.percentile(ys, 1)
        span_x = span_x if span_x > 1e-6 else 1.0
        span_y = span_y if span_y > 1e-6 else 1.0
        lo_x, lo_y = np.percentile(xs, 1), np.percentile(ys, 1)
        px = np.clip(((xs - lo_x) / span_x * 560 + 40).astype(int), 0, 639)
        py = np.clip((420 - (ys - lo_y) / span_y * 360).astype(int), 0, 479)
        image[py, px] = color

    paint(x1, y1, (40, 140, 70))
    paint(x0, y0, (40, 70, 160))
    return image


def comparison_figure(names, scores):
    image = np.full((420, 860, 3), 245, dtype=np.uint8)
    image[40:41, 80:820] = (180, 40, 40)
    for index, name in enumerate(names):
        score = scores[name]
        height = 0 if score is None else int(score * 280)
        x0 = 90 + index * 100
        image[320 - height:320, x0:x0 + 50] = (30, 90, 160)
    return image


def contact_separability(contact_ok, decision, contact_rows):
    if not contact_ok:
        return "NOT_IDENTIFIABLE"
    family = decision["selected_feature_family"]
    if family != "NONE":
        return contact_rows[family]["grade"]
    rank = {"LOW": 0, "MEDIUM": 1, "HIGH": 2}
    return max((row["grade"] for row in contact_rows.values()), key=lambda item: rank[item])


def write_json(path, payload):
    Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def main():
    region_path = OUT / "diagnostic_regions.json"
    if sha256_file(region_path) != FROZEN_REGION_SHA:
        raise SystemExit("diagnostic region 파일이 측정 전에 고정한 해시와 다르다.")
    regions = json.loads(region_path.read_text())
    shoe_polygon = regions["regions"][0]["pixel_polygon"]
    support_polygon = regions["regions"][1]["pixel_polygon"]
    xyz, rgb = read_cloud(CAPTURE / "cloud.pcd")
    color = read_png(CAPTURE / "color.png")
    info = json.loads((CAPTURE / "color_camera_info.json").read_text())
    plane = json.loads((OUT / "prism_tool" / "plane.json").read_text())
    coefficients = np.array([plane["viewpoint_a"], plane["viewpoint_b"], plane["viewpoint_c"], plane["viewpoint_d"]])
    hull = np.loadtxt(OUT / "prism_tool" / "plane_hull.csv", delimiter=",", skiprows=1, usecols=(1, 2, 3))
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    height = plane_height(xyz, coefficients)
    axis_u, axis_v = plane_axes(coefficients)
    origin = hull.mean(axis=0)
    uv = to_uv(xyz, origin, axis_u, axis_v)
    hull_uv = to_uv(hull, origin, axis_u, axis_v)
    inside_hull = point_in_polygon(uv[:, 0], uv[:, 1], hull_uv)
    finite = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0.05)
    u = np.full(len(xyz), np.nan)
    v = np.full(len(xyz), np.nan)
    u[finite] = fx * xyz[finite, 0] / xyz[finite, 2] + cx
    v[finite] = fy * xyz[finite, 1] / xyz[finite, 2] + cy
    in_image = finite & (u >= 0) & (u < color.shape[1]) & (v >= 0) & (v < color.shape[0])
    shoe_pixels = in_image & point_in_polygon(u, v, shoe_polygon)
    support_pixels = in_image & point_in_polygon(u, v, support_polygon)
    shoe = shoe_pixels & inside_hull & (height > CONTACT_SHOE_HEIGHT[1]) & (height <= 0.18)
    support = support_pixels & inside_hull & np.isfinite(height) & (height >= -0.02) & (height <= 0.18)
    shoe_uv = uv[shoe]
    window = (
        (u >= min(p[0] for p in shoe_polygon) - 150)
        & (u <= max(p[0] for p in shoe_polygon) + 150)
        & (v >= min(p[1] for p in shoe_polygon) - 150)
        & (v <= max(p[1] for p in shoe_polygon) + 150)
        & inside_hull
        & np.isfinite(height)
    )
    horizontal = np.full(len(xyz), np.inf)
    horizontal[window] = nearest_horizontal(uv[window], shoe_uv)
    contact_shoe = (
        window
        & (height > CONTACT_SHOE_HEIGHT[0])
        & (height <= CONTACT_SHOE_HEIGHT[1])
        & (horizontal <= CONTACT_SHOE_HORIZONTAL_MAX)
        & ~support_pixels
    )
    contact_support = (
        window
        & (height <= CONTACT_SUPPORT_HEIGHT_MAX)
        & (horizontal > CONTACT_SUPPORT_HORIZONTAL[0])
        & (horizontal <= CONTACT_SUPPORT_HORIZONTAL[1])
    )
    contact_ok = int(contact_shoe.sum()) >= MIN_CONTACT_POINTS and int(contact_support.sum()) >= MIN_CONTACT_POINTS
    hsv = rgb_to_hsv(rgb)
    lab = rgb_to_lab(rgb)
    names = ["RGB", "HSV", "Lab", "Height", "HSV+Height", "Lab+Height", "RGB+Height"]
    global_rows = {}
    contact_rows = {}
    for name in names:
        global_rows[name] = evaluate_pair(name, rgb, hsv, lab, height, u, v, shoe, support)
        if contact_ok:
            contact_rows[name] = evaluate_pair(name, rgb, hsv, lab, height, u, v, contact_shoe, contact_support)
        else:
            contact_rows[name] = {
                "spatial_block_balanced_accuracy": None,
                "valid_folds": 0,
                "folds": [],
                "grade": "LOW",
                "point_count": int(contact_shoe.sum() + contact_support.sum()),
            }
    decision = select_method(
        {name: global_rows[name]["grade"] for name in names},
        {name: contact_rows[name]["grade"] for name in names},
        {name: global_rows[name]["spatial_block_balanced_accuracy"] or 0.0 for name in names},
        {name: contact_rows[name]["spatial_block_balanced_accuracy"] or 0.0 for name in names},
        contact_ok,
    )
    hue_deg = hsv[:, 0] * 360.0
    stats = {
        "rgb": {
            "R": channel_metrics(rgb[shoe, 0], rgb[support, 0], RGB_EDGES),
            "G": channel_metrics(rgb[shoe, 1], rgb[support, 1], RGB_EDGES),
            "B": channel_metrics(rgb[shoe, 2], rgb[support, 2], RGB_EDGES),
        },
        "hsv": {
            "H": channel_metrics(hue_deg[shoe], hue_deg[support], np.linspace(0.0, 360.0, 33)),
            "S": channel_metrics(hsv[shoe, 1], hsv[support, 1], UNIT_EDGES),
            "V": channel_metrics(hsv[shoe, 2], hsv[support, 2], UNIT_EDGES),
            "shoe_circular_mean_deg": circular_mean_deg(hsv[shoe, 0]),
            "support_circular_mean_deg": circular_mean_deg(hsv[support, 0]),
        },
        "lab": {
            "L": channel_metrics(lab[shoe, 0], lab[support, 0], LAB_L_EDGES),
            "a": channel_metrics(lab[shoe, 1], lab[support, 1], LAB_AB_EDGES),
            "b": channel_metrics(lab[shoe, 2], lab[support, 2], LAB_AB_EDGES),
        },
        "height": channel_metrics(height[shoe], height[support], HEIGHT_EDGES),
    }
    contact_stats = {
        "identifiable": contact_ok,
        "shoe_side_count": int(contact_shoe.sum()),
        "support_side_count": int(contact_support.sum()),
        "shoe_side_height": distribution(height[contact_shoe]),
        "support_side_height": distribution(height[contact_support]),
        "sets": contact_rows,
        "separability": contact_separability(contact_ok, decision, contact_rows),
    }
    population = {
        "shoe_interior": {
            "point_count": int(shoe.sum()),
            "pixel_polygon": shoe_polygon,
            "height": distribution(height[shoe]),
            "bounds_xyz_min": xyz[shoe].min(axis=0).tolist() if shoe.any() else None,
            "bounds_xyz_max": xyz[shoe].max(axis=0).tolist() if shoe.any() else None,
        },
        "support_interior": {
            "point_count": int(support.sum()),
            "pixel_polygon": support_polygon,
            "height": distribution(height[support]),
            "bounds_xyz_min": xyz[support].min(axis=0).tolist() if support.any() else None,
            "bounds_xyz_max": xyz[support].max(axis=0).tolist() if support.any() else None,
        },
    }
    figures = OUT / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    region_view = color.copy()
    draw_box(region_view, shoe_polygon, (255, 40, 40))
    draw_box(region_view, support_polygon, (40, 220, 40))
    write_png(figures / "01_diagnostic_regions.png", region_view)
    write_png(figures / "02_rgb_distribution.png", histogram_figure(rgb[shoe, 2], rgb[support, 2], RGB_EDGES, None))
    write_png(figures / "03_hsv_distribution.png", histogram_figure(hsv[shoe, 1], hsv[support, 1], UNIT_EDGES, None))
    write_png(figures / "04_lab_distribution.png", histogram_figure(lab[shoe, 1], lab[support, 1], LAB_AB_EDGES, None))
    write_png(figures / "05_height_distribution.png", histogram_figure(height[shoe], height[support], HEIGHT_EDGES, None))
    write_png(figures / "06_color_feature_scatter.png", scatter_figure(lab[shoe, 1], lab[shoe, 2], lab[support, 1], lab[support, 2]))
    write_png(figures / "07_color_height_scatter.png", scatter_figure(lab[shoe, 1], height[shoe], lab[support, 1], height[support]))
    write_png(
        figures / "08_feature_separability_comparison.png",
        comparison_figure(names, {name: global_rows[name]["spatial_block_balanced_accuracy"] for name in names}),
    )
    contact_view = (color.astype(np.uint16) // 2).astype(np.uint8)
    support_paint = contact_support & in_image
    shoe_paint = contact_shoe & in_image
    contact_view[v[support_paint].astype(np.int64), u[support_paint].astype(np.int64)] = (40, 200, 80)
    contact_view[v[shoe_paint].astype(np.int64), u[shoe_paint].astype(np.int64)] = (220, 50, 40)
    write_png(figures / "09_contact_region.png", contact_view)
    family = decision["selected_feature_family"]
    if family in ("Lab", "Lab+Height", "NONE"):
        selected_view = scatter_figure(lab[shoe, 1], lab[shoe, 2], lab[support, 1], lab[support, 2])
    elif family == "Height":
        selected_view = histogram_figure(height[shoe], height[support], HEIGHT_EDGES, None)
    else:
        selected_view = scatter_figure(hsv[shoe, 1], hsv[shoe, 2], hsv[support, 1], hsv[support, 2])
    write_png(figures / "10_selected_feature_visualization.png", selected_view)
    write_json(OUT / "diagnostic_region_population.json", population)
    write_json(OUT / "rgb_statistics.json", stats["rgb"])
    write_json(OUT / "hsv_statistics.json", stats["hsv"])
    write_json(OUT / "lab_statistics.json", stats["lab"])
    write_json(OUT / "height_statistics.json", stats["height"])
    write_json(
        OUT / "separability_metrics.json",
        {
            "decision_metric": "spatial_block_balanced_accuracy",
            "random_point_split_used_for_selection": False,
            "thresholds": {"high": 0.90, "medium": 0.75, "min_valid_folds": 3},
            "global": global_rows,
            "contact": contact_rows,
        },
    )
    write_json(OUT / "contact_analysis.json", contact_stats)
    write_json(OUT / "method_selection.json", decision)
    write_json(
        OUT / "input_provenance.json",
        {
            "dataset": "P-008B Dataset v2",
            "capture": "capture_01",
            "cloud_sha256": sha256_file(CAPTURE / "cloud.pcd"),
            "cloud_fields": ["x", "y", "z", "rgb"],
            "frozen_region_sha256": FROZEN_REGION_SHA,
            "holdout_accessed": False,
            "prism": "scripts/p008r1e_prism.cpp PCL 1.12.1 unchanged parameters",
        },
    )
    print(json.dumps({"population": {key: population[key]["point_count"] for key in population}, "decision": decision, "global": {name: global_rows[name]["grade"] for name in names}, "contact_counts": [int(contact_shoe.sum()), int(contact_support.sum())], "contact_grades": {name: contact_rows[name]["grade"] for name in names}}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
