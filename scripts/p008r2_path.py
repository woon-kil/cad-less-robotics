"""P-008-R2. 신발 장축을 따라 연속 3D 경로 하나를 만든다."""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CANDIDATE = ROOT / "evidence" / "P-008" / "R1GEO" / "05_shoe_candidate.pcd"
PLANE = ROOT / "evidence" / "P-008" / "R1GEO" / "support_plane.json"
PARAMS = {
    "bin_width_m": 0.008,
    "height_quantile": 0.75,
    "min_bin_points": 8,
    "end_fraction": 0.15,
    "jump_reject_m": 0.12,
    "smooth_passes": 1,
    "target_spacing_m": 0.002,
    "surface_reprojection": False,
    "selected_path_semantic": "LONGITUDINAL_SURFACE",
}


def plane_height(points, normal, offset):
    return np.asarray(points, dtype=np.float64) @ np.asarray(normal, dtype=np.float64) + float(offset)


def principal_axes(points):
    points = np.asarray(points, dtype=np.float64)
    centroid = points.mean(axis=0)
    centered = points - centroid
    covariance = (centered.T @ centered) / len(points)
    eigenvalues, vectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    return centroid, eigenvalues[order], vectors[:, order]


def orient_longitudinal(points, axis, centroid, normal, offset, end_fraction):
    """높이 중앙값이 더 큰 끝이 +t가 되도록 축을 한 번만 맞춘다."""
    projection = (np.asarray(points, dtype=np.float64) - centroid) @ axis
    low_cut = np.quantile(projection, end_fraction)
    high_cut = np.quantile(projection, 1.0 - end_fraction)
    height = plane_height(points, normal, offset)
    low = np.median(height[projection <= low_cut])
    high = np.median(height[projection >= high_cut])
    if high < low:
        return -axis
    return axis


def assign_bins(projection, bin_width):
    if bin_width <= 0:
        raise ValueError("bin 폭은 양수여야 합니다.")
    start = float(np.min(projection))
    index = np.floor((projection - start) / bin_width).astype(np.int64)
    index = np.clip(index, 0, None)
    return index, start


def _quantile_point(points, projection, height, mask, quantile):
    target = float(np.quantile(height[mask], quantile))
    local = np.flatnonzero(mask)
    nearest = local[np.argmin(np.abs(height[local] - target))]
    return points[nearest], float(projection[nearest])


def _ordered_chain(points, projection, height, bins, mask_for_bin, quantile, min_count):
    chosen = []
    coordinates = []
    for bin_id in range(int(bins.max()) + 1):
        mask = mask_for_bin(bin_id)
        if int(np.count_nonzero(mask)) < min_count:
            continue
        point, coordinate = _quantile_point(points, projection, height, mask, quantile)
        chosen.append(point)
        coordinates.append(coordinate)
    if not chosen:
        return np.zeros((0, 3)), np.zeros(0)
    order = np.argsort(np.asarray(coordinates))
    return np.asarray(chosen, dtype=np.float64)[order], np.asarray(coordinates, dtype=np.float64)[order]


def representative_points(points, projection, height, bin_width, quantile, min_count, lateral=None):
    """장축 bin마다 위쪽 분위 점을 고른다.

    폭 좌표가 있으면 칼라 개구가 있는 중심선 대신, 장축을 더 길게 덮는 한쪽 측면을 쓴다.
    """
    bins, _ = assign_bins(projection, bin_width)
    full = _ordered_chain(
        points, projection, height, bins, lambda bin_id: bins == bin_id, quantile, min_count)
    if lateral is None:
        return full
    lateral = np.asarray(lateral)
    sides = []
    for sign in (1.0, -1.0):
        chain = _ordered_chain(
            points, projection, height, bins,
            lambda bin_id, sign=sign: (bins == bin_id) & (np.sign(lateral) == sign),
            quantile, min_count)
        if len(chain[0]) >= 2:
            span = float(chain[1][-1] - chain[1][0])
            sides.append((span, len(chain[0]), chain))
    if not sides:
        return full
    sides.sort(key=lambda item: (item[0], item[1]))
    return sides[-1][2]


def reject_jumps(path, limit):
    if len(path) == 0:
        return path.copy()
    kept = [path[0]]
    for point in path[1:]:
        if np.linalg.norm(point - kept[-1]) <= limit:
            kept.append(point)
    return np.asarray(kept, dtype=np.float64)


def smooth_path(path, passes):
    output = np.asarray(path, dtype=np.float64).copy()
    for _ in range(passes):
        if len(output) < 3:
            break
        updated = output.copy()
        updated[1:-1] = 0.25 * output[:-2] + 0.5 * output[1:-1] + 0.25 * output[2:]
        output = updated
    return output


def path_lengths(path):
    if len(path) < 2:
        return np.zeros(0), 0.0
    segments = np.linalg.norm(np.diff(path, axis=0), axis=1)
    return segments, float(segments.sum())


def resample_path(path, spacing):
    if spacing <= 0 or not np.isfinite(spacing):
        raise ValueError("spacing은 양수여야 합니다.")
    path = np.asarray(path, dtype=np.float64)
    if len(path) < 2:
        return path.copy()
    segments, total = path_lengths(path)
    if total < 1e-12:
        return path[:1].copy()
    cumulative = np.concatenate(([0.0], np.cumsum(segments)))
    targets = [0.0]
    step = 1
    while spacing * step < total - 1e-9:
        targets.append(spacing * step)
        step += 1
    if targets[-1] < total - 1e-4:
        targets.append(total)
    output = []
    segment_index = 1
    for target in targets:
        while segment_index + 1 < len(path) and cumulative[segment_index] < target:
            segment_index += 1
        span = cumulative[segment_index] - cumulative[segment_index - 1]
        local = 0.0 if span < 1e-12 else (target - cumulative[segment_index - 1]) / span
        local = min(1.0, max(0.0, local))
        output.append(path[segment_index - 1] + (path[segment_index] - path[segment_index - 1]) * local)
    return np.asarray(output, dtype=np.float64)


def nearest_distances(path, surface):
    path = np.asarray(path, dtype=np.float64)
    surface = np.asarray(surface, dtype=np.float64)
    if len(path) == 0:
        return np.zeros(0), np.zeros((0, 3), dtype=np.int64)
    deltas = path[:, None, :] - surface[None, :, :]
    distances = np.linalg.norm(deltas, axis=2)
    index = np.argmin(distances, axis=1)
    return distances[np.arange(len(path)), index], index


def reproject(path, surface):
    _, index = nearest_distances(path, surface)
    return np.asarray(surface, dtype=np.float64)[index]


def ordering_violations(projection, tolerance=0.002):
    if len(projection) < 2:
        return 0
    return int(np.sum(np.diff(projection) < -tolerance))


def write_evidence(out_dir=None):
    from p008b_metrics import sha256_file, write_png
    from p008r1geo import overlay, paint, project, read_png, read_xyz_pcd, write_xyz_pcd

    out_dir = Path(out_dir) if out_dir else ROOT / "evidence" / "P-008" / "R2"
    out_dir.mkdir(parents=True, exist_ok=True)
    points = read_xyz_pcd(CANDIDATE)
    plane = json.loads(PLANE.read_text())
    normal = np.asarray(plane["normal"], dtype=np.float64)
    offset = float(plane["coefficients"][3])
    built = build_longitudinal_path(points, normal, offset)
    color = read_png(ROOT / "evidence" / "P-008B" / "capture_01" / "color.png")
    camera = json.loads((ROOT / "evidence" / "P-008" / "R1GEO" / "parameters.json").read_text())
    (out_dir / "shoe_candidate.pcd").write_bytes(CANDIDATE.read_bytes())
    write_xyz_pcd(out_dir / "raw_path.pcd", built["raw"])
    write_xyz_pcd(out_dir / "smoothed_path.pcd", built["smoothed"])
    write_xyz_pcd(out_dir / "resampled_path.pcd", built["final"])
    segments, total = built["segments"], built["total_length"]
    cumulative = np.concatenate(([0.0], np.cumsum(segments))) if len(segments) else np.zeros(len(built["final"]))
    with (out_dir / "resampled_path.csv").open("w") as handle:
        handle.write("index,x,y,z,arc_length\n")
        for index, point in enumerate(built["final"]):
            handle.write(f"{index},{point[0]},{point[1]},{point[2]},{cumulative[index]}\n")
    height = built["height"]
    geometry = {
        "point_count": int(len(points)),
        "centroid": built["centroid"].tolist(),
        "aabb_extent": (points.max(0) - points.min(0)).tolist(),
        "pca_eigenvalues": built["eigenvalues"].tolist(),
        "pca_axes": built["axes"].tolist(),
        "longitudinal_axis": built["axis"].tolist(),
        "plane_relative_height": {
            "min": float(height.min()),
            "p05": float(np.quantile(height, 0.05)),
            "median": float(np.median(height)),
            "p95": float(np.quantile(height, 0.95)),
            "max": float(height.max()),
        },
        "eigenvalue_ratio_01": float(built["eigenvalues"][0] / built["eigenvalues"][1]),
    }
    (out_dir / "geometry_summary.json").write_text(json.dumps(geometry, indent=2) + "\n")
    (out_dir / "parameters.json").write_text(json.dumps(PARAMS, indent=2) + "\n")
    (out_dir / "path_selection.json").write_text(json.dumps({
        "selected_path_semantic": "LONGITUDINAL_SURFACE",
        "selection_reason": "첫 고유값이 둘째의 약 3.9배라 장축이 분명하다. 낮은 t 끝은 뒤축, 높은 t 끝은 앞코다. 중심선은 칼라 개구를 지나므로, 장축을 더 길게 덮는 한쪽 측면의 위쪽 점을 잇는다. crest와 boundary는 만들지 않았다.",
    }, indent=2, ensure_ascii=False) + "\n")
    spacing = percentile_summary(segments)
    distance = percentile_summary(built["surface_distance"])
    metrics = {
        "path_point_count_raw": int(len(built["raw"])),
        "path_point_count_final": int(len(built["final"])),
        "target_spacing_m": PARAMS["target_spacing_m"],
        "segment_length": spacing,
        "total_path_length_m": total,
        "surface_distance": distance,
        "ordering_violation_count": built["ordering_violations"],
        "surface_reprojection_used": bool(PARAMS["surface_reprojection"]),
    }
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (out_dir / "input_manifest.json").write_text(json.dumps({
        "dataset": "P-008B_DATASET_V2",
        "capture": "capture_01",
        "input": "evidence/P-008/R1GEO/05_shoe_candidate.pcd",
        "input_sha256": sha256_file(CANDIDATE),
        "support_plane": "evidence/P-008/R1GEO/support_plane.json",
        "capture_02_05_used": False,
    }, indent=2) + "\n")
    figures = out_dir / "figures"
    figures.mkdir(exist_ok=True)
    axis_span = (points - built["centroid"]) @ built["axis"]
    axis_line = built["centroid"] + np.linspace(axis_span.min(), axis_span.max(), 40)[:, None] * built["axis"]
    low = points[axis_span <= np.quantile(axis_span, 0.15)]
    high = points[axis_span >= np.quantile(axis_span, 0.85)]

    def thick(image, path_points, tint):
        if len(path_points) < 2:
            paint(image, path_points, camera, tint)
            return
        u, v = project(np.asarray(path_points, dtype=np.float64), camera)
        height, width = image.shape[:2]
        for index in range(len(u) - 1):
            steps = int(max(abs(u[index + 1] - u[index]), abs(v[index + 1] - v[index]))) + 1
            xs = np.linspace(u[index], u[index + 1], steps)
            ys = np.linspace(v[index], v[index + 1], steps)
            for x, y in zip(xs, ys):
                x0, x1 = max(0, int(round(x)) - 1), min(width, int(round(x)) + 2)
                y0, y1 = max(0, int(round(y)) - 1), min(height, int(round(y)) + 2)
                image[y0:y1, x0:x1] = tint

    base = overlay(color, points, camera, (70, 90, 220))
    write_png(figures / "01_shoe_candidate.png", base)
    axes_view = overlay(color, points, camera, (60, 70, 140))
    paint(axes_view, low, camera, (255, 50, 50))
    paint(axes_view, high, camera, (40, 220, 80))
    thick(axes_view, axis_line, (255, 220, 40))
    write_png(figures / "02_pca_axes.png", axes_view)
    for name, samples in (
        ("03_raw_path.png", built["raw"]),
        ("04_smoothed_path.png", built["smoothed"]),
        ("05_resampled_path.png", built["resampled"]),
        ("07_final_path.png", built["final"]),
    ):
        view = overlay(color, points, camera, (60, 70, 140))
        thick(view, samples, (255, 40, 40))
        write_png(figures / name, view)
    distance_view = overlay(color, points, camera, (40, 40, 40))
    thick(distance_view, built["final"], (40, 220, 80))
    write_png(figures / "06_surface_distance.png", distance_view)
    print(json.dumps(metrics, indent=2))
    return metrics


def percentile_summary(values):
    values = np.asarray(values, dtype=np.float64)
    if len(values) == 0:
        return {"min": None, "median": None, "p95": None, "max": None}
    return {
        "min": float(np.min(values)),
        "median": float(np.median(values)),
        "p95": float(np.quantile(values, 0.95)),
        "max": float(np.max(values)),
    }


def build_longitudinal_path(points, normal, offset, params=None):
    params = dict(PARAMS if params is None else params)
    centroid, eigenvalues, axes = principal_axes(points)
    axis = orient_longitudinal(
        points, axes[:, 0], centroid, normal, offset, params["end_fraction"])
    projection = (points - centroid) @ axis
    lateral = (points - centroid) @ axes[:, 1]
    height = plane_height(points, normal, offset)
    raw, raw_t = representative_points(
        points, projection, height, params["bin_width_m"], params["height_quantile"],
        params["min_bin_points"], lateral)
    raw = reject_jumps(raw, params["jump_reject_m"])
    smoothed = smooth_path(raw, params["smooth_passes"])
    resampled = resample_path(smoothed, params["target_spacing_m"])
    if params["surface_reprojection"]:
        final = reproject(resampled, points)
    else:
        final = resampled
    final_t = (final - centroid) @ axis
    segments, total = path_lengths(final)
    distances, _ = nearest_distances(final, points)
    return {
        "centroid": centroid,
        "eigenvalues": eigenvalues,
        "axes": axes,
        "axis": axis,
        "raw": raw,
        "raw_t": raw_t,
        "smoothed": smoothed,
        "resampled": resampled,
        "final": final,
        "final_t": final_t,
        "segments": segments,
        "total_length": total,
        "surface_distance": distances,
        "ordering_violations": ordering_violations(final_t),
        "height": height,
    }


if __name__ == "__main__":
    write_evidence()
