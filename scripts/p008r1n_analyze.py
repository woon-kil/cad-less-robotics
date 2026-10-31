#!/usr/bin/env python3
"""P-008-R1N. R1I 점군에서 단면 특징만 본다. 규칙은 이미 고정돼 있다."""

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from p008b_metrics import write_png
from p008r1k_analyze import (
    component_of,
    fit_plane,
    group_voxels,
    plane_basis,
    read_cloud,
    read_png,
    sha256_file,
    to_plane,
)
from p008r1k_geometry import connected_components, signed_plane_height
from p008r1l_analyze import describe_voxels, draw_line, paint, project
from p008r1l_geometry import normal_consistency
from p008r1n_geometry import (
    frame_coordinates,
    interval_coverage,
    jump_limit,
    max_consecutive,
    orient_axis,
    pca_in_plane,
    side_transition,
    slice_centers,
    spacing_stats,
)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "evidence" / "P-008" / "R1N"
CAPTURE = ROOT / "evidence" / "P-008" / "R1I" / "capture_dev"
EXPECTED = "68b7b0a08164aeda291349be46dfbb3a6eacd66cd801d04d8b1a9a1025482377"


def load_rules():
    method = json.loads((OUT / "methodology.json").read_text())
    slices = json.loads((OUT / "slice_definition.json").read_text())
    feature = json.loads((OUT / "feature_rule.json").read_text())
    if not (method["fixed_before_scoring"] and slices["fixed_before_scoring"] and feature["fixed_before_scoring"]):
        raise SystemExit("규칙이 채점 전에 고정되지 않았다.")
    if method["expected_sha256"] != EXPECTED:
        raise SystemExit("입력 계약이 바뀌었다.")
    return method, slices, feature


def shoe_ids(xyz, u, v, height, rules):
    window = {"u0": 150, "u1": 950, "v0": 250, "v1": 620}
    table = (u >= window["u0"]) & (u < window["u1"]) & (v >= window["v0"]) & (v < window["v1"])
    chosen = table & (height > 0.004) & (height <= 0.1)
    index = np.flatnonzero(chosen)
    groups = group_voxels(xyz[index], rules["voxel_m"])
    components, _owner = component_of(groups, rules["voxel_m"])
    box = {"u0": 420, "u1": 660, "v0": 340, "v1": 540}
    contact = (u >= box["u0"]) & (u < box["u1"]) & (v >= box["v0"]) & (v < box["v1"]) & (height > 0.004) & (height <= 0.02)
    scored = []
    for comp_id, members in enumerate(components):
        local = np.concatenate([np.asarray(groups[key], dtype=np.int64) for key in members])
        full = index[local]
        scored.append((int(np.count_nonzero(contact[full])), comp_id, full))
    scored.sort(key=lambda item: (-item[0], item[1]))
    if len(scored) >= 2 and scored[0][0] == scored[1][0]:
        return None
    return scored[0][2]


def axis_from(points, origin, axis_u, axis_v):
    uv = to_plane(points, origin, axis_u, axis_v)
    direction = pca_in_plane(uv - uv.mean(axis=0))
    vector = orient_axis(direction[0] * axis_u + direction[1] * axis_v)
    lateral = np.cross(axis_u, axis_v)
    # lateral은 지지 법선과 세로축으로 다시 만든다.
    del lateral
    return vector


def envelope(across, height, indices, bin_m):
    keys = np.floor(across / bin_m).astype(np.int64)
    chosen = {}
    for local, key in enumerate(keys):
        if key not in chosen or height[local] > height[chosen[key]]:
            chosen[key] = local
    ordered = sorted(chosen)
    local_index = [chosen[key] for key in ordered]
    position = np.array(ordered, dtype=np.float64) * bin_m + bin_m / 2.0
    return position, height[local_index], indices[local_index]


def evaluate(points, height, origin, longitudinal, lateral, normal, intercept, slices, feature, spacing, thickness, neighborhood):
    along, across, _ = frame_coordinates(points, origin, longitudinal, lateral, normal, intercept)
    low, high = np.percentile(along, slices["longitudinal_percentile"])
    centers = slice_centers(float(low), float(high), spacing)
    half = thickness / 2.0
    rows = []
    for index, center in enumerate(centers):
        chosen = np.abs(along - center) <= half
        raw = int(np.count_nonzero(chosen))
        row = {
            "index": index,
            "t_m": float(center),
            "raw_points": raw,
            "eligible_points": 0,
            "contour_points": 0,
            "valid": False,
            "label": "missing",
            "reason": "too_few_points",
        }
        if raw < slices["min_points"]:
            rows.append(row)
            continue
        local_points = points[chosen]
        keys = list(map(tuple, np.floor(local_points / slices["voxel_m"]).astype(np.int64)))
        parts = connected_components(keys)
        largest = max(parts, key=len)
        keep = np.array([key in set(largest) for key in keys])
        eligible = int(np.count_nonzero(keep))
        row["eligible_points"] = eligible
        if eligible < slices["min_points"]:
            row["reason"] = "fragmented"
            rows.append(row)
            continue
        local_index = np.flatnonzero(chosen)[keep]
        position, curve, point_index = envelope(across[local_index], height[local_index], local_index, slices["envelope_bin_m"])
        row["contour_points"] = int(len(position))
        if len(position) < slices["min_envelope_bins"] or float(position[-1] - position[0]) < slices["min_lateral_extent_m"]:
            row["reason"] = "short_envelope"
            rows.append(row)
            continue
        row["valid"] = True
        found = side_transition(position, curve, neighborhood, feature["steep_deg"], feature["shallow_deg"])
        scores = {name: item["score_deg"] for name, item in found.items()}
        if len(found) == 1:
            side = next(iter(found))
            label = "unique"
        elif len(found) == 2:
            names = list(found)
            gap = abs(found[names[0]]["score_deg"] - found[names[1]]["score_deg"])
            if gap < feature["dominance_margin_deg"]:
                side = None
                label = "ambiguous"
            else:
                side = max(names, key=lambda name: found[name]["score_deg"])
                label = "unique"
        else:
            side = None
            label = "missing"
        row["label"] = label
        row["reason"] = None if label == "unique" else label
        row["scores_deg"] = scores
        if side is not None:
            point = int(point_index[found[side]["index"]])
            row["point_index"] = point
            row["side"] = side
            row["xyz"] = points[point].tolist()
            row["height_m"] = float(height[point])
            row["across_m"] = float(across[point])
        rows.append(row)
    unique = [row for row in rows if row["label"] == "unique"]
    labels = [row["label"] for row in rows]
    total = len(rows)
    valid = sum(1 for row in rows if row["valid"])
    stats = spacing_stats([row["xyz"] for row in unique])
    limit = jump_limit(spacing)
    jumps = 0
    if stats["max_m"] is not None:
        gaps = np.linalg.norm(np.diff(np.array([row["xyz"] for row in unique]), axis=0), axis=1) if len(unique) >= 2 else []
        jumps = int(np.count_nonzero(np.array(gaps) > limit)) if len(unique) >= 2 else 0
    coverage = interval_coverage([row["t_m"] for row in unique], spacing, float(low), float(high))
    summary = {
        "evaluated": total,
        "valid": valid,
        "valid_ratio": float(valid / total) if total else 0.0,
        "unique": len(unique),
        "unique_ratio": float(len(unique) / total) if total else 0.0,
        "ambiguous_ratio": float(labels.count("ambiguous") / total) if total else 0.0,
        "missing_ratio": float(labels.count("missing") / total) if total else 0.0,
        "max_consecutive_missing": max_consecutive(labels, "missing"),
        "coverage": coverage,
        "spacing": stats,
        "jump_limit_m": limit,
        "unexplained_jumps": jumps,
        "range_m": [float(low), float(high)],
    }
    passed = bool(
        summary["valid_ratio"] >= slices["valid_slice_ratio_min"]
        and summary["unique_ratio"] >= slices["unique_feature_slice_ratio_min"]
        and summary["max_consecutive_missing"] <= slices["max_consecutive_missing_slices"]
        and summary["coverage"] >= slices["ordered_coverage_min"]
        and jumps == 0
        and total > 0
    )
    return rows, summary, passed


def main():
    _method, slices, feature = load_rules()
    digest = sha256_file(CAPTURE / "cloud.pcd")
    xyz_all = read_cloud(CAPTURE / "cloud.pcd")
    finite = np.isfinite(xyz_all).all(axis=1) & (xyz_all[:, 2] > 0)
    if digest != EXPECTED or int(finite.sum()) != 756376:
        (OUT / "result.json").write_text(json.dumps({"verdict": "BLOCKED_INPUT_CONTRACT", "sha256": digest}) + "\n")
        raise SystemExit("BLOCKED_INPUT_CONTRACT")
    xyz = xyz_all[finite]
    info = json.loads((CAPTURE / "color_camera_info.json").read_text())
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    u, v = project(xyz, fx, fy, cx, cy)
    window = (u >= 150) & (u < 950) & (v >= 250) & (v < 620)
    normal, intercept, inliers, residual = fit_plane(xyz[window])
    recorded = json.loads((ROOT / "evidence" / "P-008" / "R1K" / "support_plane.json").read_text())
    if np.max(np.abs(normal - np.asarray(recorded["normal"]))) > 1e-6 or abs(float(intercept) - recorded["d"]) > 1e-6:
        raise SystemExit("지지 평면이 R1K와 다르다.")
    height = signed_plane_height(xyz, normal, intercept)
    ids = shoe_ids(xyz, u, v, height, slices)
    if ids is None:
        raise SystemExit("신발 성분이 하나로 정해지지 않는다.")
    shoe = xyz[ids]
    shoe_h = height[ids]
    support = xyz[np.abs(height) <= 0.004]
    origin = support.mean(axis=0)
    axis_u, axis_v = plane_basis(normal)
    full_axis = axis_from(shoe, origin, axis_u, axis_v)
    upper = shoe[shoe_h > 0.02]
    upper_axis = axis_from(upper, origin, axis_u, axis_v) if len(upper) >= 2 else full_axis
    axis_angle = float(np.degrees(np.arccos(np.clip(abs(float(full_axis @ upper_axis)), -1.0, 1.0))))
    axis_stable = axis_angle <= 15.0
    lateral = orient_axis(np.cross(normal, full_axis))
    along, _, _ = frame_coordinates(shoe, origin, full_axis, lateral, normal, intercept)
    rows, summary, passed_features = evaluate(
        shoe, shoe_h, origin, full_axis, lateral, normal, intercept, slices, feature,
        slices["slice_spacing_m"], slices["slab_thickness_m"], feature["neighborhood_m"],
    )
    variants = []
    for spacing in slices["sensitivity"]["slice_spacing_m"]:
        _rows, variant, ok = evaluate(
            shoe, shoe_h, origin, full_axis, lateral, normal, intercept, slices, feature,
            spacing, slices["slab_thickness_m"], feature["neighborhood_m"],
        )
        variants.append({"parameter": "slice_spacing_m", "value": spacing, "passed": ok, "summary": variant})
    for thickness in slices["sensitivity"]["slab_thickness_m"]:
        _rows, variant, ok = evaluate(
            shoe, shoe_h, origin, full_axis, lateral, normal, intercept, slices, feature,
            slices["slice_spacing_m"], thickness, feature["neighborhood_m"],
        )
        variants.append({"parameter": "slab_thickness_m", "value": thickness, "passed": ok, "summary": variant})
    for neighborhood in slices["sensitivity"]["neighborhood_m"]:
        _rows, variant, ok = evaluate(
            shoe, shoe_h, origin, full_axis, lateral, normal, intercept, slices, feature,
            slices["slice_spacing_m"], slices["slab_thickness_m"], neighborhood,
        )
        variants.append({"parameter": "neighborhood_m", "value": neighborhood, "passed": ok, "summary": variant})
    # 같은 명목은 세 축에 반복된다.
    unique_variants = []
    seen = set()
    for item in variants:
        key = (item["parameter"], item["value"])
        if key in seen:
            continue
        seen.add(key)
        unique_variants.append(item)
    others = [item["passed"] for item in unique_variants if not (
        item["parameter"] == "slice_spacing_m" and item["value"] == slices["slice_spacing_m"]
        or item["parameter"] == "slab_thickness_m" and item["value"] == slices["slab_thickness_m"]
        or item["parameter"] == "neighborhood_m" and item["value"] == feature["neighborhood_m"]
    )]
    if passed_features and all(others):
        sensitivity = "STABLE"
    elif (not passed_features) and (not any(others)):
        sensitivity = "STABLE"
    else:
        sensitivity = "FRAGILE"
    passed = bool(passed_features and axis_stable and sensitivity == "STABLE")
    verdict = "PASS_CUTTING_PLANE_FEATURE_FEASIBILITY" if passed else "FAIL_CUTTING_PLANE_FEATURE_FEASIBILITY"
    blockers = []
    if not axis_stable:
        blockers.append("세로축이 15° 넘게 달라짐")
    if summary["valid_ratio"] < slices["valid_slice_ratio_min"]:
        blockers.append("유효 단면 비율")
    if summary["unique_ratio"] < slices["unique_feature_slice_ratio_min"]:
        blockers.append("단일 특징 비율")
    if summary["max_consecutive_missing"] > slices["max_consecutive_missing_slices"]:
        blockers.append("연속 결손")
    if summary["coverage"] < slices["ordered_coverage_min"]:
        blockers.append("세로 덮임")
    if summary["unexplained_jumps"] > 0:
        blockers.append("간격 초과 점프")
    if sensitivity == "FRAGILE":
        blockers.append("파라미터에 따라 합격 여부가 갈림")
    feature_xyz = np.array([row["xyz"] for row in rows if row["label"] == "unique"]) if any(row["label"] == "unique" for row in rows) else np.zeros((0, 3))
    voxels = describe_voxels(shoe, slices["voxel_m"], 2, normal, 8, 0.25) if len(shoe) else {}
    feature_angles = []
    feature_valid = 0
    feature_keys = []
    if len(feature_xyz):
        feature_keys = list(map(tuple, np.floor(feature_xyz / slices["voxel_m"]).astype(np.int64)))
        for key in feature_keys:
            item = voxels.get(key)
            if item is None:
                continue
            if item["valid"]:
                feature_valid += 1
            if item["angle"] is not None:
                feature_angles.append(item["angle"])
    normal_report = normal_consistency(feature_angles, 25.0)
    normal_report["valid_ratio"] = float(feature_valid / len(feature_keys)) if feature_keys else 0.0
    public_rows = []
    for row in rows:
        public_rows.append({key: value for key, value in row.items() if key != "point_index"})
    (OUT / "input_manifest.json").write_text(json.dumps({
        "capture": "evidence/P-008/R1I/capture_dev/cloud.pcd",
        "sha256": digest,
        "finite_xyz": 756376,
        "frame": "camera_color_optical_frame",
        "fields": ["x", "y", "z", "rgb"],
        "color_used_for_extraction": False,
        "r1m_used": False,
        "holdout": "NOT_USED",
    }, indent=2) + "\n")
    (OUT / "support_plane.json").write_text(json.dumps({
        "normal": normal.tolist(),
        "d": float(intercept),
        "inlier_count": int(inliers.sum()),
        "residual_median_m": float(np.median(residual)),
        "residual_p95_m": float(np.percentile(residual, 95)),
        "sign": "camera_positive",
    }, indent=2) + "\n")
    (OUT / "longitudinal_axis.json").write_text(json.dumps({
        "method": "plane_pca",
        "vector": full_axis.tolist(),
        "extent_m": float(along.max() - along.min()),
        "evaluated_range_m": summary["range_m"],
        "axis_angle_deg": axis_angle,
        "stable": axis_stable,
    }, indent=2) + "\n")
    (OUT / "slice_metrics.json").write_text(json.dumps({"summary": summary, "slices": public_rows}, indent=2) + "\n")
    (OUT / "feature_candidates.json").write_text(json.dumps(public_rows, indent=2) + "\n")
    (OUT / "feature_continuity.json").write_text(json.dumps({
        "interpolation_used": False,
        "spacing": summary["spacing"],
        "max_consecutive_missing": summary["max_consecutive_missing"],
        "coverage": summary["coverage"],
        "jump_limit_m": summary["jump_limit_m"],
        "unexplained_jumps": summary["unexplained_jumps"],
    }, indent=2) + "\n")
    (OUT / "normal_quality.json").write_text(json.dumps(normal_report, indent=2) + "\n")
    (OUT / "parameter_sensitivity.json").write_text(json.dumps({
        "parameter_sensitivity": sensitivity,
        "variants": [{
            "parameter": item["parameter"],
            "value": item["value"],
            "passed": item["passed"],
            "valid_ratio": item["summary"]["valid_ratio"],
            "unique_ratio": item["summary"]["unique_ratio"],
            "coverage": item["summary"]["coverage"],
            "max_consecutive_missing": item["summary"]["max_consecutive_missing"],
            "unexplained_jumps": item["summary"]["unexplained_jumps"],
        } for item in unique_variants],
    }, indent=2) + "\n")
    (OUT / "feasibility.json").write_text(json.dumps({
        "verdict": verdict,
        "ordered_process_feature": passed,
        "true_manufacturing_boundary": False,
        "terminology": "ORDERED_PROCESS_FEATURE",
        "blockers": blockers,
    }, indent=2) + "\n")
    color = read_png(CAPTURE / "color.png").copy()
    shoe_u, shoe_v = project(shoe, fx, fy, cx, cy)
    paint(color, shoe_u, shoe_v, (40, 90, 200), 3)
    end_a = origin + full_axis * along.min()
    end_b = origin + full_axis * along.max()
    ax, ay = project(np.vstack((end_a, end_b)), fx, fy, cx, cy)
    draw_line(color, ax[0], ay[0], ax[1], ay[1], (255, 220, 0))
    write_png(OUT / "longitudinal_axis.png", color)
    overview = read_png(CAPTURE / "color.png").copy()
    for row in rows:
        mark = shoe[np.abs(along - row["t_m"]) <= slices["slab_thickness_m"] / 2.0]
        if len(mark) == 0:
            continue
        mu, mv = project(mark, fx, fy, cx, cy)
        tone = (40, 180, 60) if row["valid"] else (40, 40, 220)
        paint(overview, mu, mv, tone, 4)
    write_png(OUT / "slice_overview.png", overview)
    feature_view = read_png(CAPTURE / "color.png").copy()
    previous = None
    for row in rows:
        if row["label"] != "unique":
            continue
        point = np.array(row["xyz"])
        pu, pv = project(point.reshape(1, 3), fx, fy, cx, cy)
        paint(feature_view, pu, pv, (255, 180, 0), 1)
        if previous is not None and row["index"] == previous["index"] + 1:
            qu, qv = project(np.array(previous["xyz"]).reshape(1, 3), fx, fy, cx, cy)
            draw_line(feature_view, qu[0], qv[0], pu[0], pv[0], (255, 180, 0))
        previous = row
    write_png(OUT / "ordered_feature.png", feature_view)
    failure = read_png(CAPTURE / "color.png").copy()
    for row in rows:
        if row["label"] == "unique":
            continue
        mark = shoe[np.abs(along - row["t_m"]) <= slices["slab_thickness_m"] / 2.0]
        if len(mark) == 0:
            continue
        mu, mv = project(mark, fx, fy, cx, cy)
        tone = (0, 140, 255) if row["label"] == "ambiguous" else (30, 30, 220)
        paint(failure, mu, mv, tone, 3)
    write_png(OUT / "feature_failures.png", failure)
    write_png(OUT / "feature_candidates.png", feature_view)
    examples = [row for row in rows if row["valid"]]
    picked = []
    if examples:
        picked = [examples[0], examples[len(examples) // 2], examples[-1]]
    chart = Image.new("RGB", (900, 320), (255, 255, 255))
    draw = ImageDraw.Draw(chart)
    font_path = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
    font = ImageFont.truetype(font_path, 14) if Path(font_path).exists() else ImageFont.load_default()
    for panel, row in enumerate(picked):
        local = np.abs(along - row["t_m"]) <= slices["slab_thickness_m"] / 2.0
        s = frame_coordinates(shoe[local], origin, full_axis, lateral, normal, intercept)[1]
        h = shoe_h[local]
        if len(s) == 0:
            continue
        s0, s1 = float(s.min()), float(s.max())
        h0, h1 = float(h.min()), float(h.max())
        left = 20 + panel * 300
        for sv, hv in zip(s[::3], h[::3]):
            x = left + int(260 * (sv - s0) / max(s1 - s0, 1e-6))
            y = 280 - int(240 * (hv - h0) / max(h1 - h0, 1e-6))
            draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=(20, 90, 180))
        draw.text((left, 8), "slice %d %s" % (row["index"], row["label"]), fill=(0, 0, 0), font=font)
    chart.save(OUT / "cross_section_examples.png")
    bars = Image.new("RGB", (880, 420), (255, 255, 255))
    pen = ImageDraw.Draw(bars)
    pen.text((16, 12), "민감 조합의 단일 특징 비율. 빨간 선 = 0.80", fill=(0, 0, 0), font=font)
    for index, item in enumerate(unique_variants):
        y = 44 + index * 28
        pen.text((16, y), "%s %.3f" % (item["parameter"], item["value"]), fill=(0, 0, 0), font=font)
        width = int(360 * item["summary"]["unique_ratio"])
        pen.rectangle((280, y, 280 + width, y + 16), fill=(40, 140, 70) if item["passed"] else (180, 40, 40))
    pen.line((280 + int(360 * 0.8), 36, 280 + int(360 * 0.8), 44 + len(unique_variants) * 28), fill=(220, 30, 30), width=2)
    bars.save(OUT / "sensitivity.png")
    (OUT / "report.md").write_text("\n".join([
        "# P-008-R1N — 절단 단면의 순서 있는 기하 특징",
        "",
        "입력은 R1I 점군이다. SHA256은 `%s`이다. 색은 추출에 쓰지 않았다. R1M의 B, C는 쓰지 않았다." % digest,
        "",
        "세로축은 지지 평면에 투영한 PCA다. 전체와 2 cm 위 부분의 축 차이는 %.2f°이다." % axis_angle,
        "",
        "평가 단면 %d, 유효 %d (%.3f), 단일 특징 %d (%.3f), 모호 %.3f, 없음 %.3f." % (
            summary["evaluated"], summary["valid"], summary["valid_ratio"], summary["unique"],
            summary["unique_ratio"], summary["ambiguous_ratio"], summary["missing_ratio"],
        ),
        "",
        "연속 결손 %d. 세로 덮임 %.3f. 간격 중앙값 %s m. 설명하지 않은 점프 %d. 민감 결과는 %s." % (
            summary["max_consecutive_missing"], summary["coverage"],
            "없음" if summary["spacing"]["median_m"] is None else "%.4f" % summary["spacing"]["median_m"],
            summary["unexplained_jumps"], sensitivity,
        ),
        "",
        "이 점은 접착선이 아니다. 제조 경계라고 하지 않는다. 빠진 점은 잇지 않았다.",
        "",
        "```text",
        "P-008-R1N = %s" % verdict,
        "FEATURE_INTERPOLATION_USED = NO",
        "TRUE_MANUFACTURING_BOUNDARY_CLAIMED = NO",
        "```",
        "",
        "다음 Gate는 열지 않는다.",
        "",
    ]))
    (OUT / "result.json").write_text(json.dumps({
        "verdict": verdict,
        "sha256": digest,
        "axis_vector": full_axis.tolist(),
        "axis_extent_m": float(along.max() - along.min()),
        "axis_angle_deg": axis_angle,
        "axis_stable": axis_stable,
        "summary": summary,
        "normal": normal_report,
        "parameter_sensitivity": sensitivity,
        "blockers": blockers,
        "feature_interpolation_used": False,
        "true_manufacturing_boundary_claimed": False,
    }, indent=2) + "\n")
    print(json.dumps({
        "verdict": verdict,
        "valid_ratio": summary["valid_ratio"],
        "unique_ratio": summary["unique_ratio"],
        "missing_run": summary["max_consecutive_missing"],
        "coverage": summary["coverage"],
        "jumps": summary["unexplained_jumps"],
        "axis_angle": axis_angle,
        "sensitivity": sensitivity,
        "blockers": blockers,
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
