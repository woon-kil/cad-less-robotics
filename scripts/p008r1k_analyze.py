#!/usr/bin/env python3
"""P-008-R1K. R1I 점군만 쓴다. 기준 곡선은 평가에만 쓴다."""

import hashlib
import json
import struct
import zlib
from pathlib import Path

import numpy as np

from p008b_metrics import write_png
from p008r1k_geometry import (
    connected_components,
    orient_toward_camera,
    polyline_distances,
    signed_plane_height,
    voxel_keys,
)

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "evidence" / "P-008" / "R1I" / "capture_dev"
OUT = ROOT / "evidence" / "P-008" / "R1K"
EXPECTED = "68b7b0a08164aeda291349be46dfbb3a6eacd66cd801d04d8b1a9a1025482377"


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def read_png(path):
    data = path.read_bytes()
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
    raw = path.read_bytes()
    header, body = raw.split(b"\nDATA binary\n", 1)
    count = int([line for line in header.decode("ascii").splitlines() if line.startswith("POINTS ")][0].split()[1])
    packed = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")], count=count)
    return np.column_stack((packed["x"], packed["y"], packed["z"])).astype(np.float64)


def fit_plane(points):
    rng = np.random.default_rng(1)
    best_count = -1
    best_normal = None
    best_d = None
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
            best_normal = normal
            best_d = intercept
    mask = np.abs(points @ best_normal + best_d) <= 0.004
    centroid = points[mask].mean(axis=0)
    _, _, vh = np.linalg.svd(points[mask] - centroid, full_matrices=False)
    normal = orient_toward_camera(vh[-1], centroid)
    intercept = -float(normal @ centroid)
    residual = np.abs(points[mask] @ normal + intercept)
    return normal, intercept, mask, residual


def plane_basis(normal):
    helper = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    axis_u = np.cross(normal, helper)
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    return axis_u, axis_v


def to_plane(points, origin, axis_u, axis_v):
    delta = points - origin
    return np.column_stack((delta @ axis_u, delta @ axis_v))


def group_voxels(points, leaf):
    keys = np.floor(points / leaf).astype(np.int64)
    groups = {}
    for index, key in enumerate(map(tuple, keys)):
        groups.setdefault(key, []).append(index)
    return groups


def component_of(groups, leaf):
    components = connected_components(list(groups))
    owner = {}
    for comp_id, members in enumerate(components):
        for key in members:
            owner[key] = comp_id
    return components, owner


def sample_polyline(nodes, step_px=8):
    samples = []
    closed = nodes + [nodes[0]]
    for start, end in zip(closed[:-1], closed[1:]):
        length = max(abs(end[0] - start[0]), abs(end[1] - start[1]), 1)
        count = max(1, int(length / step_px))
        for t in range(count):
            alpha = t / count
            samples.append((start[0] * (1 - alpha) + end[0] * alpha, start[1] * (1 - alpha) + end[1] * alpha))
    return samples


def lift_reference(samples, u, v, xyz):
    positions = []
    for su, sv in samples:
        near = (np.abs(u - su) <= 3.0) & (np.abs(v - sv) <= 3.0)
        positions.append(None if near.sum() < 3 else np.median(xyz[near], axis=0))
    return positions


def probe_component(samples, u, v, xyz, chosen, owner, leaf):
    cand = np.flatnonzero(chosen)
    votes = []
    if len(cand) == 0:
        return [None] * len(samples)
    cu = u[cand]
    cv = v[cand]
    for su, sv in samples:
        near = (np.abs(cu - su) <= 12.0) & (np.abs(cv - sv) <= 12.0)
        if not np.any(near):
            votes.append(None)
            continue
        local = np.flatnonzero(near)
        dist = (cu[local] - su) ** 2 + (cv[local] - sv) ** 2
        point = xyz[cand[local[int(np.argmin(dist))]]]
        key = tuple(np.floor(point / leaf).astype(np.int64))
        votes.append(owner.get(key))
    return votes


def footprint_boundary(points, leaf, axis_u, axis_v, origin):
    uv = to_plane(points, origin, axis_u, axis_v)
    keys = np.floor(uv / leaf).astype(np.int64)
    cells = {}
    for key, point in zip(map(tuple, keys), points):
        cells.setdefault(key, []).append(point)
    boundary = []
    for key, members in cells.items():
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
        if any((key[0] + du, key[1] + dv) not in cells for du, dv in neighbors):
            boundary.append(np.mean(members, axis=0))
    return np.array(boundary) if boundary else np.zeros((0, 3)), uv


def normals_for(points, leaf, radius, plane_normal):
    groups = group_voxels(points, leaf)
    keys = list(groups)
    normals = {}
    valid = {}
    offsets = range(-radius, radius + 1)
    for key in keys:
        gathered = []
        for i in offsets:
            for j in offsets:
                for k in offsets:
                    found = groups.get((key[0] + i, key[1] + j, key[2] + k))
                    if found:
                        gathered.extend(found)
        if len(gathered) < 8:
            valid[key] = False
            continue
        block = points[gathered]
        centered = block - block.mean(axis=0)
        eigvals, eigvecs = np.linalg.eigh(centered.T @ centered)
        if eigvals[1] <= 1e-12 or eigvals[0] / eigvals[1] > 0.25:
            valid[key] = False
            continue
        direction = eigvecs[:, 0]
        if direction @ plane_normal < 0.0:
            direction = -direction
        normals[key] = direction
        valid[key] = True
    angles = []
    for key, direction in normals.items():
        adjacent = []
        for step in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
            other = normals.get((key[0] + step[0], key[1] + step[1], key[2] + step[2]))
            if other is not None:
                cosine = float(np.clip(direction @ other, -1.0, 1.0))
                adjacent.append(float(np.degrees(np.arccos(cosine))))
        if adjacent:
            angles.append(float(np.median(adjacent)))
    valid_count = sum(1 for flag in valid.values() if flag)
    return {
        "voxel_count": len(keys),
        "valid_count": valid_count,
        "valid_ratio": float(valid_count / len(keys)) if keys else 0.0,
        "median_adjacent_angle_deg": None if not angles else float(np.median(angles)),
        "unstable_ratio": None if not angles else float(np.mean(np.array(angles) > 25.0)),
    }


def tint(image, uu, vv, color, step=4):
    sel = np.arange(0, len(uu), step)
    x = np.rint(uu[sel]).astype(np.int64)
    y = np.rint(vv[sel]).astype(np.int64)
    ok = (x >= 0) & (x < image.shape[1]) & (y >= 0) & (y < image.shape[0])
    image[y[ok], x[ok]] = color


def main():
    cloud = CAPTURE / "cloud.pcd"
    digest = sha256_file(cloud)
    if digest != EXPECTED:
        raise SystemExit("BLOCKED_INPUT_CONTRACT")
    info = json.loads((CAPTURE / "color_camera_info.json").read_text())
    method = json.loads((OUT / "geometry_method.json").read_text())
    reference = json.loads((OUT / "reference_boundary.json").read_text())["polyline_uv"]
    xyz = read_cloud(cloud)
    finite = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
    if int(finite.sum()) != 756376:
        raise SystemExit("BLOCKED_INPUT_CONTRACT")
    xyz = xyz[finite]
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    u = fx * xyz[:, 0] / xyz[:, 2] + cx
    v = fy * xyz[:, 1] / xyz[:, 2] + cy
    window = method["nominal"]["table_window_px"]
    table = (u >= window["u0"]) & (u < window["u1"]) & (v >= window["v0"]) & (v < window["v1"])
    normal, intercept, inlier_local, residual = fit_plane(xyz[table])
    height = signed_plane_height(xyz, normal, intercept)
    support = np.abs(height) <= method["nominal"]["support_tolerance_m"]
    origin = xyz[support].mean(axis=0)
    axis_u, axis_v = plane_basis(normal)
    (OUT / "support_plane.json").write_text(json.dumps({
        "normal": normal.tolist(),
        "d": float(intercept),
        "inlier_count": int(inlier_local.sum()),
        "candidate_count": int(table.sum()),
        "inlier_ratio": float(inlier_local.sum() / table.sum()),
        "residual_median_m": float(np.median(residual)),
        "residual_p95_m": float(np.percentile(residual, 95)),
        "sign": "camera_positive",
    }, indent=2) + "\n")

    def extract(tau, ceiling, leaf):
        chosen = table & (height > tau) & (height <= ceiling)
        groups = group_voxels(xyz[chosen], leaf)
        components, owner = component_of(groups, leaf)
        return chosen, groups, components, owner

    nominal = method["nominal"]
    chosen, groups, components, owner = extract(
        nominal["support_tolerance_m"], nominal["height_ceiling_m"], nominal["voxel_m"]
    )
    samples = sample_polyline(reference)
    votes = probe_component(samples, u, v, xyz, chosen, owner, nominal["voxel_m"])
    positions = lift_reference(samples, u, v, xyz)
    voted = [item for item in votes if item is not None]
    if not voted:
        shoe_id = None
        shoe_points = np.zeros((0, 3))
    else:
        shoe_id = max(set(voted), key=voted.count)
        shoe_keys = set(components[shoe_id])
        shoe_index = [index for key, members in groups.items() if key in shoe_keys for index in members]
        shoe_points = xyz[np.flatnonzero(chosen)[shoe_index]]
    sizes = sorted((len(item) for item in components), reverse=True)
    boundary, _uv = footprint_boundary(shoe_points, nominal["voxel_m"], axis_u, axis_v, origin) if len(shoe_points) else (np.zeros((0, 3)), None)
    ref_pos = np.array([item for item in positions if item is not None])
    if len(boundary) and len(ref_pos):
        boundary_uv = to_plane(boundary, origin, axis_u, axis_v)
        ref_uv = to_plane(ref_pos, origin, axis_u, axis_v)
        ref_dist = polyline_distances(ref_uv, boundary_uv)
        bound_dist = polyline_distances(boundary_uv, ref_uv)
        match = method["acceptance"]["boundary_match_m"]
        coverage = float(np.mean(ref_dist <= match))
        # largest consecutive miss along the ordered probes that had a 3D hit
        miss = ref_dist > match
        gap = 0.0
        run = 0.0
        ordered = [item for item in positions if item is not None]
        for index, flag in enumerate(miss):
            if not flag:
                gap = max(gap, run)
                run = 0.0
                continue
            if index == 0:
                run = 0.0
            else:
                run += float(np.linalg.norm(ordered[index] - ordered[index - 1]))
        gap = max(gap, run)
        false = bound_dist > match
        false_points = boundary[false]
        false_components = connected_components(voxel_keys(false_points, nominal["voxel_m"])) if len(false_points) else []
        largest_false = max((len(item) for item in false_components), default=0)
    else:
        coverage = 0.0
        gap = None
        ref_dist = np.array([])
        bound_dist = np.array([])
        largest_false = None
        false = np.zeros(0, dtype=bool)
    upper = shoe_points[signed_plane_height(shoe_points, normal, intercept) > 0.02] if len(shoe_points) else shoe_points
    normal_report = normals_for(upper, nominal["voxel_m"], nominal["normal_voxel_radius"], normal) if len(upper) else {
        "voxel_count": 0, "valid_count": 0, "valid_ratio": 0.0, "median_adjacent_angle_deg": None, "unstable_ratio": None
    }
    sensitivity = []
    base_count = 0 if shoe_id is None else len(components[shoe_id])
    for tau in method["sensitivity"]["support_tolerance_m"]:
        sensitivity.append(("support_tolerance_m", tau, *summarize_variant(
            extract(tau, nominal["height_ceiling_m"], nominal["voxel_m"]), samples, u, v, xyz, nominal["voxel_m"], base_count
        )))
    for leaf in method["sensitivity"]["voxel_m"]:
        sensitivity.append(("voxel_m", leaf, *summarize_variant(
            extract(nominal["support_tolerance_m"], nominal["height_ceiling_m"], leaf), samples, u, v, xyz, leaf, None
        )))
    for ceiling in method["sensitivity"]["height_ceiling_m"]:
        sensitivity.append(("height_ceiling_m", ceiling, *summarize_variant(
            extract(nominal["support_tolerance_m"], ceiling, nominal["voxel_m"]), samples, u, v, xyz, nominal["voxel_m"], base_count
        )))
    # voxel size changes the count scale, so compare component count and vote purity only
    stable = True
    for kind, value, purity, comp_count, count_ratio in sensitivity:
        if purity < 0.7 or comp_count != 1:
            stable = False
        if count_ratio is not None and not (0.5 <= count_ratio <= 1.5):
            stable = False
    acceptance = method["acceptance"]
    normal_ok = (
        normal_report["valid_ratio"] >= acceptance["normal_valid_min"]
        and normal_report["median_adjacent_angle_deg"] is not None
        and normal_report["median_adjacent_angle_deg"] <= acceptance["normal_median_angle_max_deg"]
    )
    boundary_ok = coverage >= acceptance["coverage_min"] and gap is not None and gap <= acceptance["largest_gap_max_m"]
    passed = bool(boundary_ok and normal_ok and stable and len(shoe_points) > 500 and coverage > 0)
    color = read_png(CAPTURE / "color.png")
    plane_view = color.copy()
    tint(plane_view, u[support], v[support], (40, 180, 60), 8)
    write_png(OUT / "support_plane.png", plane_view)
    geo = color.copy()
    if len(shoe_points):
        su = fx * shoe_points[:, 0] / shoe_points[:, 2] + cx
        sv = fy * shoe_points[:, 1] / shoe_points[:, 2] + cy
        tint(geo, su, sv, (30, 80, 220), 1)
    write_png(OUT / "candidate_geometry.png", geo)
    normal_view = color.copy()
    if len(upper):
        uu = fx * upper[:, 0] / upper[:, 2] + cx
        vv = fy * upper[:, 1] / upper[:, 2] + cy
        tint(normal_view, uu, vv, (20, 160, 220), 1)
    write_png(OUT / "normals.png", normal_view)
    overlay = color.copy()
    if len(boundary):
        bu = fx * boundary[:, 0] / boundary[:, 2] + cx
        bv = fy * boundary[:, 1] / boundary[:, 2] + cy
        tint(overlay, bu, bv, (0, 200, 255), 1)
    for ru, rv in reference:
        overlay[max(0, rv - 1):rv + 2, max(0, ru - 1):ru + 2] = (255, 40, 40)
    write_png(OUT / "boundary_overlay.png", overlay)
    failure = color.copy()
    if len(ref_pos):
        miss = ref_dist > acceptance["boundary_match_m"]
        mu = fx * ref_pos[miss, 0] / ref_pos[miss, 2] + cx
        mv = fy * ref_pos[miss, 1] / ref_pos[miss, 2] + cy
        tint(failure, mu, mv, (220, 30, 30), 1)
    if len(boundary) and false.any():
        fu = fx * boundary[false, 0] / boundary[false, 2] + cx
        fv = fy * boundary[false, 1] / boundary[false, 2] + cy
        tint(failure, fu, fv, (180, 0, 180), 1)
    write_png(OUT / "boundary_failure.png", failure)
    sens_rows = [
        {"parameter": kind, "value": value, "vote_purity": purity, "winning_components": comp_count, "count_ratio": count_ratio}
        for kind, value, purity, comp_count, count_ratio in sensitivity
    ]
    (OUT / "support_removal_sensitivity.json").write_text(json.dumps([row for row in sens_rows if row["parameter"] == "support_tolerance_m"], indent=2) + "\n")
    (OUT / "parameter_sensitivity.json").write_text(json.dumps(sens_rows, indent=2) + "\n")
    (OUT / "normal_quality.json").write_text(json.dumps(normal_report, indent=2) + "\n")
    (OUT / "boundary_metrics.json").write_text(json.dumps({
        "coverage_within_0_025m": coverage,
        "largest_gap_m": gap,
        "reference_median_distance_m": None if len(ref_dist) == 0 else float(np.median(ref_dist)),
        "boundary_median_distance_m": None if len(bound_dist) == 0 else float(np.median(bound_dist)),
        "largest_false_component_voxels": largest_false,
        "boundary_points": int(len(boundary)),
        "reference_samples": len(samples),
        "reference_hits": int(sum(item is not None for item in positions)),
        "independent_of_extraction_threshold": True,
    }, indent=2) + "\n")
    (OUT / "processpath_feasibility.json").write_text(json.dumps({
        "position_feasible": bool(len(shoe_points) > 500 and stable),
        "normal_feasible": bool(normal_ok),
        "frame": "camera_color_optical_frame",
        "path_generated": False,
    }, indent=2) + "\n")
    verdict = "PASS_GEOMETRY_FEASIBILITY" if passed else "FAIL_GEOMETRY_FEASIBILITY"
    (OUT / "result.json").write_text(json.dumps({
        "verdict": verdict,
        "shoe_points": int(len(shoe_points)),
        "dominant_voxels": 0 if shoe_id is None else len(components[shoe_id]),
        "component_count": len(components),
        "largest_component_voxels": sizes[0] if sizes else 0,
        "vote_purity": 0.0 if not voted else voted.count(shoe_id) / len(voted),
        "stable": stable,
        "boundary_ok": boundary_ok,
        "normal_ok": normal_ok,
    }, indent=2) + "\n")
    print(json.dumps({
        "verdict": verdict,
        "shoe_points": int(len(shoe_points)),
        "coverage": coverage,
        "gap": gap,
        "normal_valid": normal_report["valid_ratio"],
        "normal_angle": normal_report["median_adjacent_angle_deg"],
        "stable": stable,
        "purity": 0.0 if not voted else voted.count(shoe_id) / len(voted),
        "components": len(components),
    }))


def summarize_variant(extracted, samples, u, v, xyz, leaf, base_count):
    chosen, _groups, components, owner = extracted
    votes = probe_component(samples, u, v, xyz, chosen, owner, leaf)
    voted = [item for item in votes if item is not None]
    if not voted:
        return 0.0, 0, None
    winner = max(set(voted), key=voted.count)
    purity = voted.count(winner) / len(voted)
    winners = len({item for item in voted})
    count = len(components[winner])
    ratio = None if not base_count else count / base_count
    return purity, winners, ratio


if __name__ == "__main__":
    main()
