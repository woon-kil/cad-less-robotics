#!/usr/bin/env python3
"""P-008-R1L. R1I 점군에서 분사 작업면만 가른다. R1K 기준 곡선은 읽지 않는다."""

import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from p008b_metrics import write_png
from p008r1k_analyze import (
    fit_plane,
    group_voxels,
    component_of,
    plane_basis,
    read_cloud,
    read_png,
    sha256_file,
    to_plane,
)
from p008r1k_geometry import connected_components, signed_plane_height
from p008r1l_geometry import (
    boundary_clearance,
    graph_extent,
    normal_consistency,
    usable_region_mask,
)

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "evidence" / "P-008" / "R1I" / "capture_dev"
OUT = ROOT / "evidence" / "P-008" / "R1L"
R1K = ROOT / "evidence" / "P-008" / "R1K"
R1I = ROOT / "evidence" / "P-008" / "R1I"
EXPECTED = "68b7b0a08164aeda291349be46dfbb3a6eacd66cd801d04d8b1a9a1025482377"
FACE = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))


def load_rules():
    process = json.loads((OUT / "process_definition.json").read_text())
    confidence = json.loads((OUT / "geometry_confidence.json").read_text())
    table = json.loads((R1K / "geometry_method.json").read_text())["nominal"]["table_window_px"]
    contact = json.loads((R1I / "contact_band_definition.json").read_text())["pixel_window"]
    if process["table_window_px"] != table or process["contact_window_px"] != contact:
        raise SystemExit("창 정의가 기존 evidence와 다르다.")
    if process["min_usable_path_extent_m"] != 0.1 or confidence["boundary_clearance_m"] != 0.01:
        raise SystemExit("명목 기준이 바뀌었다.")
    return process, confidence


def describe_voxels(points, leaf, radius, plane_normal, min_points, eigen_max):
    groups = group_voxels(points, leaf)
    info = {}
    offsets = range(-radius, radius + 1)
    for key in groups:
        gathered = []
        for i in offsets:
            for j in offsets:
                for k in offsets:
                    found = groups.get((key[0] + i, key[1] + j, key[2] + k))
                    if found:
                        gathered.extend(found)
        count = len(gathered)
        direction = None
        valid = False
        if count >= min_points:
            block = points[np.asarray(gathered, dtype=np.int64)]
            centered = block - block.mean(axis=0)
            eigvals, eigvecs = np.linalg.eigh(centered.T @ centered)
            if eigvals[1] > 1e-12 and eigvals[0] / eigvals[1] <= eigen_max:
                direction = eigvecs[:, 0]
                if float(direction @ plane_normal) < 0.0:
                    direction = -direction
                valid = True
        info[key] = {"count": count, "valid": valid, "direction": None if direction is None else direction.copy()}
    for key, item in info.items():
        if not item["valid"]:
            item["angle"] = None
            continue
        adjacent = []
        for step in FACE:
            other = info.get((key[0] + step[0], key[1] + step[1], key[2] + step[2]))
            if other is not None and other["valid"]:
                cosine = float(np.clip(item["direction"] @ other["direction"], -1.0, 1.0))
                adjacent.append(float(np.degrees(np.arccos(cosine))))
        item["angle"] = None if not adjacent else float(np.median(adjacent))
    return info


def enclosed_holes(uv, leaf):
    keys = {tuple(row) for row in np.floor(np.asarray(uv) / leaf).astype(np.int64)}
    if not keys:
        return []
    xs = [key[0] for key in keys]
    ys = [key[1] for key in keys]
    minx, maxx = min(xs) - 1, max(xs) + 1
    miny, maxy = min(ys) - 1, max(ys) + 1
    outside = set()
    stack = [(x, miny) for x in range(minx, maxx + 1)]
    stack += [(x, maxy) for x in range(minx, maxx + 1)]
    stack += [(minx, y) for y in range(miny, maxy + 1)]
    stack += [(maxx, y) for y in range(miny, maxy + 1)]
    while stack:
        x, y = stack.pop()
        if (x, y) in outside or (x, y) in keys:
            continue
        if x < minx or x > maxx or y < miny or y > maxy:
            continue
        outside.add((x, y))
        stack.extend(((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)))
    seen = set()
    holes = []
    for x in range(minx, maxx + 1):
        for y in range(miny, maxy + 1):
            if (x, y) in keys or (x, y) in outside or (x, y) in seen:
                continue
            comp = []
            queue = [(x, y)]
            seen.add((x, y))
            while queue:
                cx, cy = queue.pop()
                comp.append((cx, cy))
                for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                    if (nx, ny) in seen or (nx, ny) in keys or (nx, ny) in outside:
                        continue
                    if nx < minx or nx > maxx or ny < miny or ny > maxy:
                        continue
                    seen.add((nx, ny))
                    queue.append((nx, ny))
            hx = [cell[0] for cell in comp]
            hy = [cell[1] for cell in comp]
            holes.append({
                "cells": len(comp),
                "span_m": float(leaf * math.hypot(max(hx) - min(hx), max(hy) - min(hy))),
            })
    holes.sort(key=lambda item: item["span_m"], reverse=True)
    return holes


def paint(image, uu, vv, color, step=2):
    if len(uu) == 0:
        return
    sel = np.arange(0, len(uu), step)
    x = np.rint(uu[sel]).astype(np.int64)
    y = np.rint(vv[sel]).astype(np.int64)
    ok = (x >= 0) & (x < image.shape[1]) & (y >= 0) & (y < image.shape[0])
    image[y[ok], x[ok]] = color


def draw_line(image, x0, y0, x1, y1, color):
    count = int(max(abs(x1 - x0), abs(y1 - y0), 1))
    for step in range(count + 1):
        x = int(round(x0 + (x1 - x0) * step / count))
        y = int(round(y0 + (y1 - y0) * step / count))
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            image[y, x] = color


def project(points, fx, fy, cx, cy):
    z = np.maximum(points[:, 2], 1e-6)
    return fx * points[:, 0] / z + cx, fy * points[:, 1] / z + cy


def main():
    process, confidence = load_rules()
    cloud = CAPTURE / "cloud.pcd"
    digest = sha256_file(cloud)
    xyz_all = read_cloud(cloud)
    finite = np.isfinite(xyz_all).all(axis=1) & (xyz_all[:, 2] > 0)
    if digest != EXPECTED or int(finite.sum()) != 756376:
        result = {"verdict": "BLOCKED_INPUT_CONTRACT", "sha256": digest, "finite_xyz": int(finite.sum())}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        raise SystemExit("BLOCKED_INPUT_CONTRACT")
    xyz = xyz_all[finite]
    info = json.loads((CAPTURE / "color_camera_info.json").read_text())
    fx, fy, cx, cy = (info["k"][0], info["k"][4], info["k"][2], info["k"][5])
    u, v = project(xyz, fx, fy, cx, cy)
    window = process["table_window_px"]
    table = (u >= window["u0"]) & (u < window["u1"]) & (v >= window["v0"]) & (v < window["v1"])
    normal, intercept, _, _ = fit_plane(xyz[table])
    recorded = json.loads((R1K / "support_plane.json").read_text())
    if np.max(np.abs(normal - np.asarray(recorded["normal"]))) > 1e-6 or abs(intercept - recorded["d"]) > 1e-6:
        raise SystemExit("지지 평면이 R1K와 다르다.")
    height = signed_plane_height(xyz, normal, intercept)
    leaf = process["voxel_m"]
    low, high = process["height_process_m"]
    band_low, band_high = process["contact_band_m"]
    chosen = table & (height > process["support_tolerance_m"]) & (height <= high)
    chosen_index = np.flatnonzero(chosen)
    groups = group_voxels(xyz[chosen_index], leaf)
    components, _owner = component_of(groups, leaf)
    contact_box = process["contact_window_px"]
    in_contact = (
        (u >= contact_box["u0"]) & (u < contact_box["u1"])
        & (v >= contact_box["v0"]) & (v < contact_box["v1"])
        & (height > band_low) & (height <= band_high)
    )
    scored = []
    for comp_id, members in enumerate(components):
        point_ids = np.concatenate([np.asarray(groups[key], dtype=np.int64) for key in members])
        scored.append((int(in_contact[chosen_index[point_ids]].sum()), len(members), comp_id, point_ids))
    scored.sort(key=lambda item: (-item[0], -item[1], item[2]))
    ambiguous = len(scored) >= 2 and scored[0][0] == scored[1][0]
    winner = scored[0]
    body_ids = chosen_index[winner[3]]
    body_height = height[body_ids]
    elevated_ids = body_ids[body_height > low]
    contact_ids = body_ids[body_height <= low]
    origin = xyz[np.abs(height) <= process["support_tolerance_m"]].mean(axis=0)
    axis_u, axis_v = plane_basis(normal)
    elevated = xyz[elevated_ids]
    uv = to_plane(elevated, origin, axis_u, axis_v)
    distances = boundary_clearance(uv, leaf)
    valid_rule = confidence["valid_normal"]
    nominal_info = describe_voxels(
        elevated, leaf, valid_rule["radius_voxels"], normal,
        valid_rule["min_points"], valid_rule["eigen_ratio_max"],
    )
    info_by_radius = {valid_rule["radius_voxels"]: nominal_info}

    def voxel_key(points):
        return list(map(tuple, np.floor(points / leaf).astype(np.int64)))

    def evaluate(clearance, radius, angle_max, voxel_info):
        keys = voxel_key(elevated)
        count = np.array([voxel_info[key]["count"] for key in keys])
        valid = np.array([voxel_info[key]["valid"] for key in keys])
        angles = np.array([
            np.nan if voxel_info[key]["angle"] is None else voxel_info[key]["angle"] for key in keys
        ])
        density_ok = count >= confidence["density_min_points"]
        clearance_ok = distances >= clearance
        stable = valid & np.isfinite(angles) & (angles <= angle_max)
        safe = usable_region_mask(valid, stable, density_ok, clearance_ok)
        pre = clearance_ok & density_ok
        pre_keys = list(dict.fromkeys(key for key, flag in zip(keys, pre) if flag))
        pre_valid = [voxel_info[key]["valid"] for key in pre_keys]
        pre_angles = [
            voxel_info[key]["angle"] for key in pre_keys
            if voxel_info[key]["valid"] and voxel_info[key]["angle"] is not None
        ]
        quality = normal_consistency(pre_angles, confidence["unstable_angle_deg"])
        safe_keys = list(dict.fromkeys(key for key, flag in zip(keys, safe) if flag))
        safe_angles = [
            voxel_info[key]["angle"] for key in safe_keys if voxel_info[key]["angle"] is not None
        ]
        safe_quality = normal_consistency(safe_angles, confidence["unstable_angle_deg"])
        parts = connected_components(safe_keys)
        best_length = 0.0
        best_path = []
        best_part = []
        for part in parts:
            length, path = graph_extent(part, leaf)
            if length > best_length:
                best_length = length
                best_path = path
                best_part = part
        part_set = set(best_part)
        in_best = np.array([key in part_set for key in keys]) & safe
        if best_part:
            centers = (np.asarray(best_part, dtype=np.float64) + 0.5) * leaf
            spacing = []
            for index, center in enumerate(centers):
                delta = centers - center
                delta[index] = np.inf
                spacing.append(float(np.min(np.linalg.norm(delta, axis=1))))
            spacing_median = float(np.median(spacing))
        else:
            spacing_median = None
        return {
            "clearance_m": clearance,
            "radius": radius,
            "angle_max_deg": angle_max,
            "safe_points": int(safe.sum()),
            "safe_component_count": len(parts),
            "largest_points": int(in_best.sum()),
            "largest_voxels": len(best_part),
            "extent_m": best_length,
            "spacing_median_m": spacing_median,
            "pre_valid_ratio": float(np.mean(pre_valid)) if pre_valid else 0.0,
            "pre_quality": quality,
            "safe_quality": safe_quality,
            "safe_mask": safe,
            "pre_mask": pre,
            "best_mask": in_best,
            "path": best_path,
            "uncertain_points": int((pre & ~safe).sum()),
        }

    def info_for(radius):
        if radius not in info_by_radius:
            info_by_radius[radius] = describe_voxels(
                elevated, leaf, radius, normal, valid_rule["min_points"], valid_rule["eigen_ratio_max"],
            )
        return info_by_radius[radius]

    nominal = evaluate(
        confidence["boundary_clearance_m"], valid_rule["radius_voxels"],
        confidence["normal_stable_max_deg"], nominal_info,
    )
    variants = []
    for clearance in confidence["sensitivity"]["boundary_clearance_m"]:
        variants.append(evaluate(clearance, valid_rule["radius_voxels"], confidence["normal_stable_max_deg"], nominal_info))
    for radius in confidence["sensitivity"]["normal_voxel_radius"]:
        variants.append(evaluate(confidence["boundary_clearance_m"], radius, confidence["normal_stable_max_deg"], info_for(radius)))
    for angle in confidence["sensitivity"]["normal_stable_max_deg"]:
        variants.append(evaluate(confidence["boundary_clearance_m"], valid_rule["radius_voxels"], angle, nominal_info))
    min_extent = process["min_usable_path_extent_m"]
    stable = (not ambiguous) and all(item["extent_m"] >= min_extent for item in variants)
    pre_quality = nominal["pre_quality"]
    safe_quality = nominal["safe_quality"]
    normal_ok = (
        nominal["pre_valid_ratio"] >= confidence["normal_valid_ratio_min"]
        and pre_quality["median_deg"] is not None
        and pre_quality["median_deg"] <= confidence["normal_median_max_deg"]
        and pre_quality["p95_deg"] <= confidence["normal_p95_max_deg"]
        and pre_quality["unstable_ratio"] <= confidence["unstable_ratio_max"]
    )
    spacing_ok = (
        nominal["spacing_median_m"] is not None
        and nominal["spacing_median_m"] <= confidence["spacing_median_max_m"]
    )
    extent_ok = (not ambiguous) and nominal["extent_m"] >= min_extent
    continuity = "PASS" if extent_ok and spacing_ok else "FAIL"
    best_uv = uv[nominal["best_mask"]]
    holes = enclosed_holes(best_uv, leaf) if len(best_uv) else []
    elevated_holes = enclosed_holes(uv, leaf)
    hole_span = 0.0 if not holes else holes[0]["span_m"]
    elevated_hole = 0.0 if not elevated_holes else elevated_holes[0]["span_m"]
    # 구멍 크기만으로 합격선을 만들지 않는다. 경로는 안전 voxel만 지난다.
    passed = bool(
        (not ambiguous)
        and nominal["largest_points"] > 0
        and extent_ok
        and continuity == "PASS"
        and normal_ok
        and stable
    )
    blocking = []
    if ambiguous:
        blocking.append("접촉 성분이 한 개로 정해지지 않음")
    if not extent_ok:
        blocking.append("작업면 그래프 길이가 0.10 m 미만")
    if not spacing_ok:
        blocking.append("안전 성분 간격이 0.010 m를 넘음")
    if not normal_ok:
        blocking.append("각도 문턱 적용 전 법선이 기준 미달")
    if not stable:
        blocking.append("민감 조합에서 0.10 m 작업면이 사라짐")
    verdict = "PASS_PROCESS_SURFACE_FEASIBILITY" if passed else "FAIL_PROCESS_SURFACE_FEASIBILITY"
    plane_span = None
    if len(best_uv):
        plane_span = {
            "u_m": float(best_uv[:, 0].max() - best_uv[:, 0].min()),
            "v_m": float(best_uv[:, 1].max() - best_uv[:, 1].min()),
            "height_m": float(height[elevated_ids[nominal["best_mask"]]].max() - height[elevated_ids[nominal["best_mask"]]].min()),
        }
    projected_area = None
    if len(best_uv):
        cells = {tuple(row) for row in np.floor(best_uv / leaf).astype(np.int64)}
        projected_area = float(len(cells) * leaf * leaf)

    def public_variant(item):
        return {
            "clearance_m": item["clearance_m"],
            "radius": item["radius"],
            "angle_max_deg": item["angle_max_deg"],
            "extent_m": item["extent_m"],
            "safe_points": item["safe_points"],
            "largest_points": item["largest_points"],
            "pre_valid_ratio": item["pre_valid_ratio"],
            "pre_median_deg": item["pre_quality"]["median_deg"],
            "survives_0_10m": item["extent_m"] >= min_extent,
        }

    # 같은 명목 조합이 세 민감 축에 반복된다. 기록은 축마다 한 번씩만 남긴다.
    unique_variants = []
    seen = set()
    for item in variants:
        key = (item["clearance_m"], item["radius"], item["angle_max_deg"])
        if key in seen:
            continue
        seen.add(key)
        unique_variants.append(public_variant(item))

    keys = voxel_key(elevated)
    unstable_local = []
    for key, flag in zip(keys, nominal["pre_mask"]):
        angle = nominal_info[key]["angle"]
        if flag and angle is not None and angle > confidence["unstable_angle_deg"]:
            unstable_local.append(key)
    unstable_local = list(dict.fromkeys(unstable_local))

    exclusion = {
        "rule": "작업 높이 점 중 평면 경계 셀까지 거리가 명목 간격 미만인 점, 그리고 h ≤ 0.020 m 인 접촉 띠. 기준 곡선은 사용하지 않는다.",
        "nominal_clearance_m": confidence["boundary_clearance_m"],
        "excluded_contact_points": int(len(contact_ids)),
        "excluded_clearance_points": int((distances < confidence["boundary_clearance_m"]).sum()),
        "uncertain_points": nominal["uncertain_points"],
        "unknown_interpolated": False,
        "elevated_enclosed_hole_span_m": elevated_hole,
        "safe_enclosed_hole_span_m": hole_span,
    }
    components_out = {
        "association": "R1I_CONTACT_WINDOW_MAX_COUNT",
        "ambiguous": ambiguous,
        "selected_contact_points": winner[0],
        "selected_voxels": winner[1],
        "runner_up_contact_points": None if len(scored) < 2 else scored[1][0],
        "largest_candidate_voxels": max(item[1] for item in scored),
        "safe_component_count": nominal["safe_component_count"],
        "largest_safe_points": nominal["largest_points"],
        "largest_safe_voxels": nominal["largest_voxels"],
        "graph_extent_m": nominal["extent_m"],
        "plane_aligned_span_m": plane_span,
        "projected_area_m2": projected_area,
        "surface_area": "NOT_CLAIMED",
        "elevated_points": int(len(elevated_ids)),
    }
    normal_out = {
        "population": "각도 문턱을 적용하기 전의 간격·밀도 집합",
        "valid_ratio": nominal["pre_valid_ratio"],
        "median_deg": pre_quality["median_deg"],
        "p95_deg": pre_quality["p95_deg"],
        "unstable_ratio": pre_quality["unstable_ratio"],
        "unstable_voxels": len(unstable_local),
        "safe_population": {
            "valid_ratio": 1.0 if nominal["largest_voxels"] else 0.0,
            "median_deg": safe_quality["median_deg"],
            "p95_deg": safe_quality["p95_deg"],
            "unstable_ratio": safe_quality["unstable_ratio"],
            "note": "SAFE 집합은 이미 25° 문턱을 통과한 점이다. 합격 판단은 그 이전 집합을 쓴다.",
        },
    }
    continuity_out = {
        "position_continuity": continuity,
        "graph_extent_m": nominal["extent_m"],
        "spacing_median_m": nominal["spacing_median_m"],
        "internal_max_gap_m": hole_span,
        "elevated_enclosed_hole_span_m": elevated_hole,
        "path_crosses_hole": False,
        "path_generated": False,
    }
    feasibility = {
        "verdict": verdict,
        "spray_position_feasible": bool(extent_ok),
        "spray_orientation_feasible": bool(normal_ok),
        "parameter_sensitivity": "STABLE" if stable else "FRAGILE",
        "unknown_geometry_interpolated": False,
        "process_path_blocking_errors": blocking,
        "r1k_status_changed": False,
        "color_used_for_extraction": False,
    }
    (OUT / "input_manifest.json").write_text(json.dumps({
        "capture": "evidence/P-008/R1I/capture_dev/cloud.pcd",
        "sha256": digest,
        "frame": "camera_color_optical_frame",
        "fields": ["x", "y", "z", "rgb"],
        "finite_xyz": 756376,
        "color_used_for_extraction": False,
        "holdout": "NOT_USED",
        "reference_polyline": "NOT_READ",
    }, indent=2) + "\n")
    (OUT / "boundary_exclusion.json").write_text(json.dumps(exclusion, indent=2) + "\n")
    (OUT / "usable_components.json").write_text(json.dumps(components_out, indent=2) + "\n")
    (OUT / "normal_quality.json").write_text(json.dumps(normal_out, indent=2) + "\n")
    (OUT / "continuity_metrics.json").write_text(json.dumps(continuity_out, indent=2) + "\n")
    (OUT / "sensitivity.json").write_text(json.dumps({
        "parameter_sensitivity": "STABLE" if stable else "FRAGILE",
        "variants": unique_variants,
    }, indent=2) + "\n")
    (OUT / "process_surface_feasibility.json").write_text(json.dumps(feasibility, indent=2) + "\n")

    color = read_png(CAPTURE / "color.png").copy()
    eu, ev = project(elevated, fx, fy, cx, cy)
    safe_view = color.copy()
    paint(safe_view, eu[nominal["best_mask"]], ev[nominal["best_mask"]], (40, 170, 60), 1)
    write_png(OUT / "safe_surface.png", safe_view)
    excluded_view = color.copy()
    cu, cv = project(xyz[contact_ids], fx, fy, cx, cy)
    paint(excluded_view, cu, cv, (40, 40, 220), 2)
    clearance_fail = distances < confidence["boundary_clearance_m"]
    paint(excluded_view, eu[clearance_fail], ev[clearance_fail], (40, 40, 220), 2)
    uncertain = nominal["pre_mask"] & ~nominal["safe_mask"]
    paint(excluded_view, eu[uncertain], ev[uncertain], (0, 180, 255), 1)
    write_png(OUT / "excluded_geometry.png", excluded_view)

    normal_view = color.copy()
    means = {}
    for key, point in zip(keys, elevated):
        means.setdefault(key, []).append(point)
    means = {key: np.mean(value, axis=0) for key, value in means.items()}
    shown = 0
    for key in dict.fromkeys(key for key, flag in zip(keys, nominal["best_mask"]) if flag):
        shown += 1
        if shown % 8 != 0:
            continue
        direction = nominal_info[key]["direction"]
        if direction is None:
            continue
        start = means[key]
        end = start + 0.02 * direction
        x0, y0 = project(start.reshape(1, 3), fx, fy, cx, cy)
        x1, y1 = project(end.reshape(1, 3), fx, fy, cx, cy)
        draw_line(normal_view, x0[0], y0[0], x1[0], y1[0], (40, 220, 80))
    if unstable_local:
        unstable_points = np.array([means[key] for key in unstable_local if key in means])
        if len(unstable_points):
            ux, uy = project(unstable_points, fx, fy, cx, cy)
            paint(normal_view, ux, uy, (30, 30, 230), 1)
    write_png(OUT / "normal_overlay.png", normal_view)

    continuity_view = color.copy()
    paint(continuity_view, eu[nominal["best_mask"]], ev[nominal["best_mask"]], (80, 160, 40), 2)
    path_points = [means[key] for key in nominal["path"] if key in means]
    for start, end in zip(path_points, path_points[1:]):
        x0, y0 = project(start.reshape(1, 3), fx, fy, cx, cy)
        x1, y1 = project(end.reshape(1, 3), fx, fy, cx, cy)
        draw_line(continuity_view, x0[0], y0[0], x1[0], y1[0], (0, 220, 255))
    write_png(OUT / "continuity.png", continuity_view)

    chart = Image.new("RGB", (900, 420), (255, 255, 255))
    draw = ImageDraw.Draw(chart)
    font_path = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
    font = ImageFont.truetype(font_path, 14) if Path(font_path).exists() else ImageFont.load_default()
    draw.text((20, 12), "그래프 경로 길이 (m). 빨간 선 = 0.10 m", fill=(0, 0, 0), font=font)
    max_extent = max([item["extent_m"] for item in unique_variants] + [0.12])
    for index, item in enumerate(unique_variants):
        y = 40 + index * 28
        label = "간격 %.0f mm, 반경 %d, 각 %.0f°" % (item["clearance_m"] * 1000, item["radius"], item["angle_max_deg"])
        draw.text((16, y), label, fill=(0, 0, 0), font=font)
        width = int(320 * item["extent_m"] / max_extent)
        color_bar = (40, 140, 70) if item["survives_0_10m"] else (180, 40, 40)
        draw.rectangle((280, y, 280 + width, y + 16), fill=color_bar)
        draw.text((290 + width, y), "%.3f" % item["extent_m"], fill=(0, 0, 0), font=font)
    line_x = 280 + int(320 * 0.1 / max_extent)
    draw.line((line_x, 36, line_x, 40 + len(unique_variants) * 28), fill=(220, 30, 30), width=2)
    chart.save(OUT / "sensitivity.png")

    report = [
        "# P-008-R1L — 분사에 쓸 수 있는 신발 표면",
        "",
        "입력은 R1I 점군이다. SHA256은 `%s`이다. 색은 추출에 쓰지 않았다. R1K 기준 곡선은 읽지 않았다." % digest,
        "",
        "작업은 신발 표면에 접착제나 처리제를 뿌리는 경로를 나중에 만들기 위한 표면이 있는지를 보는 것이다. 전체 둘레는 요구하지 않는다. R1K는 `FAIL_GEOMETRY_FEASIBILITY` 그대로다.",
        "",
        "성분은 테이블 창의 평면 위 연결 성분 가운데, R1I 접촉 창 안의 접촉 띠 점이 가장 많은 것이다. 접촉 점 %d, 다음 성분 %s. 가장 큰 성분은 %d voxel이라 이 성분이 아니다." % (
            winner[0],
            "없음" if len(scored) < 2 else str(scored[1][0]),
            max(item[1] for item in scored),
        ),
        "",
        "작업 높이는 0.020 m 초과 0.100 m 이하이다. 그 아래 접촉 띠 %d 점은 제외했다. 평면 경계에서 10 mm 안쪽 %d 점도 제외했다. 빈 칸은 채우지 않았다." % (
            len(contact_ids), int((distances < confidence["boundary_clearance_m"]).sum()),
        ),
        "",
        "안전 점 %d. 그중 가장 긴 그래프 경로는 %.3f m, voxel %d, 점 %d. 성분 수 %d. 이웃 간격 중앙값은 %s m." % (
            nominal["safe_points"], nominal["extent_m"], nominal["largest_voxels"], nominal["largest_points"],
            nominal["safe_component_count"],
            "없음" if nominal["spacing_median_m"] is None else "%.4f" % nominal["spacing_median_m"],
        ),
        "",
        "법선은 각도 문턱을 걸기 전에 재었다. 유효 비율 %.3f, 중앙값 %s°, P95 %s°, 불안정 비율 %s. R1K의 0.981을 가져오지 않았다." % (
            nominal["pre_valid_ratio"],
            "없음" if pre_quality["median_deg"] is None else "%.2f" % pre_quality["median_deg"],
            "없음" if pre_quality["p95_deg"] is None else "%.2f" % pre_quality["p95_deg"],
            "없음" if pre_quality["unstable_ratio"] is None else "%.3f" % pre_quality["unstable_ratio"],
        ),
        "",
        "작업 높이 전체의 닫힌 구멍 길이는 %.3f m이다. 안전 집합의 닫힌 구멍 길이는 %.3f m이다. 노란 경로는 안전 voxel만 지난다." % (
            elevated_hole, hole_span,
        ),
        "",
        "민감 결과는 %s이다. 막힌 항목: %s." % (
            "STABLE" if stable else "FRAGILE",
            "없음" if not blocking else ", ".join(blocking),
        ),
        "",
        "```text",
        "P-008-R1L = %s" % verdict,
        "UNKNOWN_GEOMETRY_INTERPOLATED = NO",
        "R1K_STATUS_CHANGED = NO",
        "```",
        "",
        "다음 Gate는 열지 않는다.",
        "",
    ]
    (OUT / "report.md").write_text("\n".join(report))
    (OUT / "result.json").write_text(json.dumps({
        "verdict": verdict,
        "sha256": digest,
        "color_used_for_extraction": False,
        "safe_points": nominal["safe_points"],
        "safe_component_count": nominal["safe_component_count"],
        "largest_safe_points": nominal["largest_points"],
        "usable_physical_extent_m": nominal["extent_m"],
        "position_continuity": continuity,
        "internal_max_gap_m": hole_span,
        "normal_valid_ratio": nominal["pre_valid_ratio"],
        "normal_angle_median_deg": pre_quality["median_deg"],
        "normal_angle_p95_deg": pre_quality["p95_deg"],
        "unstable_normal_ratio": pre_quality["unstable_ratio"],
        "safe_normal_median_deg": safe_quality["median_deg"],
        "safe_normal_p95_deg": safe_quality["p95_deg"],
        "boundary_clearance_m": confidence["boundary_clearance_m"],
        "unknown_geometry_interpolated": False,
        "parameter_sensitivity": "STABLE" if stable else "FRAGILE",
        "spray_position_feasible": bool(extent_ok),
        "spray_orientation_feasible": bool(normal_ok),
        "process_path_blocking_errors": blocking,
        "r1k_status_changed": False,
        "ambiguous_component": ambiguous,
    }, indent=2) + "\n")
    print(json.dumps({
        "verdict": verdict,
        "extent": nominal["extent_m"],
        "safe_points": nominal["safe_points"],
        "normal_valid": nominal["pre_valid_ratio"],
        "median": pre_quality["median_deg"],
        "p95": pre_quality["p95_deg"],
        "unstable": pre_quality["unstable_ratio"],
        "stable": stable,
        "hole": hole_span,
        "elevated_hole": elevated_hole,
        "blocking": blocking,
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
