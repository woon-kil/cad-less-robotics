#!/usr/bin/env python3
"""P-008-R1M. 세 뷰를 각각 재고 정합하지 않는다."""

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
    read_cloud,
    read_png,
    sha256_file,
)
from p008r1k_geometry import connected_components, signed_plane_height
from p008r1l_analyze import describe_voxels, paint, project
from p008r1l_geometry import normal_consistency
from p008r1m_geometry import complementary_observation, surface_class

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "evidence" / "P-008" / "R1M"
VIEWS = ("view_a", "view_b", "view_c")
OFFSETS = [
    (i, j, k)
    for i in (-1, 0, 1)
    for j in (-1, 0, 1)
    for k in (-1, 0, 1)
    if not (i == 0 and j == 0 and k == 0)
]


def support_angle(direction, plane_normal):
    cosine = float(np.clip(np.asarray(direction) @ np.asarray(plane_normal), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def measure_view(name, rules):
    directory = OUT / "capture" / name
    checks = json.loads((directory / "acquisition_checks.json").read_text())
    cloud_path = directory / "cloud.pcd"
    digest = sha256_file(cloud_path)
    if digest != checks["checks"]["cloud.pcd_sha256"] or not checks["checks"]["contract_passed"]:
        raise SystemExit("BLOCKED_ACQUISITION_CONTRACT")
    info = json.loads((directory / "color_camera_info.json").read_text())
    xyz = read_cloud(cloud_path)
    finite = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
    xyz = xyz[finite]
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    u, v = project(xyz, fx, fy, cx, cy)
    window = rules["support"]["table_window_px"]
    table = (u >= window["u0"]) & (u < window["u1"]) & (v >= window["v0"]) & (v < window["v1"])
    plane_normal, intercept, _, _ = fit_plane(xyz[table])
    height = signed_plane_height(xyz, plane_normal, intercept)
    leaf = rules["voxel_m"]
    band_low, band_high = rules["contact_band_m"]
    high = rules["height_process_m"][1]
    chosen = table & (height > rules["support"]["ransac_distance_m"]) & (height <= high)
    chosen_index = np.flatnonzero(chosen)
    groups = group_voxels(xyz[chosen_index], leaf)
    components, _owner = component_of(groups, leaf)
    robot = rules["robot_exclusion_px"]
    candidates = []
    for comp_id, members in enumerate(components):
        point_ids = np.concatenate([np.asarray(groups[key], dtype=np.int64) for key in members])
        full = chosen_index[point_ids]
        hh = height[full]
        contact = int(np.count_nonzero((hh > band_low) & (hh <= band_high)))
        elevated = int(np.count_nonzero(hh > rules["height_process_m"][0]))
        center_u = float(np.mean(u[full]))
        center_v = float(np.mean(v[full]))
        distance = math.hypot(center_u - robot["u"], center_v - robot["v"])
        candidates.append({
            "id": comp_id,
            "voxels": len(members),
            "contact": contact,
            "elevated": elevated,
            "u": center_u,
            "v": center_v,
            "robot": distance <= robot["radius"],
            "ids": full,
        })
    outside = [item for item in candidates if (not item["robot"]) and item["contact"] >= 100]
    outside.sort(key=lambda item: (-item["elevated"], item["id"]))
    ambiguous = len(outside) >= 2 and outside[0]["elevated"] == outside[1]["elevated"]
    shoe = None if ambiguous or not outside else outside[0]
    report = {
        "view": name,
        "sha256": digest,
        "finite_xyz": int(checks["checks"]["finite_xyz"]),
        "contract_passed": True,
        "plane_normal": plane_normal.tolist(),
        "plane_d": float(intercept),
        "component_count": len(components),
        "shoe_found": shoe is not None,
        "shoe_ambiguous": ambiguous,
    }
    if shoe is None:
        report.update({
            "elevated_points": 0,
            "side_points": 0,
            "side_voxels": 0,
            "side_valid_ratio": 0.0,
            "side_neighbor": normal_consistency([], rules["normal"]["unstable_angle_deg"]),
            "elevated_valid_ratio": 0.0,
            "elevated_voxels": 0,
            "largest_elevated_voxels": 0,
            "elevated_neighbor": normal_consistency([], rules["normal"]["unstable_angle_deg"]),
            "mean_support_angle_deg": None,
            "top_points": 0,
        })
        return report, None
    body = shoe["ids"]
    elevated_ids = body[height[body] > rules["height_process_m"][0]]
    elevated = xyz[elevated_ids]
    normal_rule = rules["normal"]
    voxels = describe_voxels(
        elevated, leaf, normal_rule["radius_voxels"], plane_normal,
        normal_rule["min_points"], normal_rule["eigen_ratio_max"],
    )
    keys = list(map(tuple, np.floor(elevated / leaf).astype(np.int64)))
    class_of = rules["surface_class_deg"]
    side_keys = set()
    top_points = 0
    side_points = 0
    support_angles = []
    for key, item in voxels.items():
        if not item["valid"] or item["direction"] is None:
            item["support_angle"] = None
            item["klass"] = "unknown"
            continue
        angle = support_angle(item["direction"], plane_normal)
        item["support_angle"] = angle
        item["klass"] = surface_class(angle, class_of["top_max"], class_of["side_min"], class_of["side_max"])
        support_angles.append(angle)
        if item["klass"] == "side":
            side_keys.add(key)
    for key, flag in zip(keys, [voxels[key]["klass"] == "top" for key in keys]):
        if flag:
            top_points += 1
    for key in keys:
        if voxels[key]["klass"] == "side":
            side_points += 1
    invalid_near = set()
    for key, item in voxels.items():
        if item["valid"]:
            continue
        if any((key[0] + step[0], key[1] + step[1], key[2] + step[2]) in side_keys for step in OFFSETS):
            invalid_near.add(key)
    side_population = len(side_keys) + len(invalid_near)
    side_parts = connected_components(list(side_keys))
    largest_side = max((len(part) for part in side_parts), default=0)
    elevated_parts = connected_components(list(voxels))
    largest_elevated = max((len(part) for part in elevated_parts), default=0)
    valid_count = sum(1 for item in voxels.values() if item["valid"])
    side_angles = [voxels[key]["angle"] for key in side_keys if voxels[key]["angle"] is not None]
    elevated_angles = [item["angle"] for item in voxels.values() if item["angle"] is not None]
    report.update({
        "shoe_voxels": shoe["voxels"],
        "shoe_contact_points": shoe["contact"],
        "shoe_u": shoe["u"],
        "shoe_v": shoe["v"],
        "elevated_points": int(len(elevated_ids)),
        "top_points": int(top_points),
        "side_points": int(side_points),
        "side_voxels": len(side_keys),
        "largest_side_voxels": largest_side,
        "side_valid_ratio": float(len(side_keys) / side_population) if side_population else 0.0,
        "side_neighbor": normal_consistency(side_angles, normal_rule["unstable_angle_deg"]),
        "elevated_valid_ratio": float(valid_count / len(voxels)) if voxels else 0.0,
        "elevated_voxels": len(voxels),
        "largest_elevated_voxels": largest_elevated,
        "elevated_neighbor": normal_consistency(elevated_angles, normal_rule["unstable_angle_deg"]),
        "mean_support_angle_deg": None if not support_angles else float(np.mean(support_angles)),
    })
    drawing = {
        "color": directory / "color.png",
        "elevated_uv": np.column_stack(project(elevated, fx, fy, cx, cy)),
        "classes": [voxels[key]["klass"] for key in keys],
    }
    return report, drawing


def panel(path, label, font):
    image = Image.fromarray(read_png(path))
    image.thumbnail((420, 280))
    canvas = Image.new("RGB", (image.width, image.height + 28), (255, 255, 255))
    canvas.paste(image, (0, 28))
    ImageDraw.Draw(canvas).text((8, 4), label, fill=(0, 0, 0), font=font)
    return canvas


def main():
    rules = json.loads((OUT / "observation_rules.json").read_text())
    measured = []
    drawings = {}
    for name in VIEWS:
        report, drawing = measure_view(name, rules)
        measured.append(report)
        drawings[name] = drawing
        (OUT / f"{name}_metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    baseline = measured[0]
    decisions = {}
    for report in measured[1:]:
        decisions[report["view"]] = complementary_observation(
            report["side_points"],
            report["largest_side_voxels"],
            report["side_valid_ratio"],
            report["side_neighbor"]["median_deg"],
            report["elevated_points"],
            report["largest_elevated_voxels"],
            report["elevated_valid_ratio"],
            report["elevated_neighbor"]["median_deg"],
            report["mean_support_angle_deg"],
            baseline["side_points"],
            baseline["mean_support_angle_deg"],
            rules["complementary"],
        )
    useful = [name for name, item in decisions.items() if item["complementary"]]
    passed = bool(useful) and all(item["contract_passed"] and item["shoe_found"] for item in measured)
    verdict = "PASS_ADDITIONAL_VIEW_OBSERVABILITY" if passed else "FAIL_ADDITIONAL_VIEW_VALUE"
    primary = None
    if useful:
        primary = max(useful, key=lambda name: next(item["side_points"] for item in measured if item["view"] == name))
    primary_report = next((item for item in measured if item["view"] == primary), None)
    comparison = {
        "registration_performed": False,
        "fusion_performed": False,
        "known_exact_object_transform": False,
        "r1l_100mm_retested": False,
        "gap_fill_mm": "NOT_CLAIMED",
        "decisions": decisions,
        "view_a_side_points": baseline["side_points"],
        "view_a_mean_support_angle_deg": baseline["mean_support_angle_deg"],
    }
    (OUT / "additional_view_comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    normal_out = {
        name: {
            "population": "신발 작업 높이의 옆면 voxel. 유효 비율은 그 voxel과 맞닿은 무효 voxel을 포함한다.",
            "valid_ratio": item["side_valid_ratio"],
            "median_deg": item["side_neighbor"]["median_deg"],
            "p95_deg": item["side_neighbor"]["p95_deg"],
            "unstable_ratio": item["side_neighbor"]["unstable_ratio"],
            "mean_support_angle_deg": item["mean_support_angle_deg"],
            "elevated_valid_ratio": item["elevated_valid_ratio"],
            "elevated_neighbor_median_deg": item["elevated_neighbor"]["median_deg"],
        }
        for name, item in ((item["view"], item) for item in measured)
    }
    (OUT / "normal_quality.json").write_text(json.dumps(normal_out, indent=2) + "\n")
    (OUT / "observability.json").write_text(json.dumps({
        "verdict": verdict,
        "complementary_views": useful,
        "primary_view": primary,
        "camera_mount_moved": False,
        "usb_device_note": "컨테이너를 다시 만들 때 USB 번호가 038, 039, 040으로 바뀌었다. 카메라는 옮기지 않았다.",
    }, indent=2) + "\n")
    (OUT / "input_manifest.json").write_text(json.dumps({
        "views": {
            item["view"]: {
                "sha256": item["sha256"],
                "finite_xyz": item["finite_xyz"],
                "contract_passed": item["contract_passed"],
            }
            for item in measured
        },
        "frame": "camera_color_optical_frame",
        "fields": ["x", "y", "z", "rgb"],
        "serial": "CP263630009D",
        "firmware": "1.4.60",
        "driver": "OrbbecSDK_ROS2 2.10.6",
        "registration_accuracy": "NOT_CLAIMED",
        "holdout": "NOT_USED",
        "color_used_for_extraction": False,
    }, indent=2) + "\n")
    font_path = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
    font = ImageFont.truetype(font_path, 18) if Path(font_path).exists() else ImageFont.load_default()
    panels = []
    for item in measured:
        color = read_png(drawings[item["view"]]["color"]).copy() if drawings[item["view"]] else read_png(OUT / "capture" / item["view"] / "color.png")
        if drawings[item["view"]]:
            uv = drawings[item["view"]]["elevated_uv"]
            classes = np.asarray(drawings[item["view"]]["classes"])
            paint(color, uv[classes == "top", 0], uv[classes == "top", 1], (40, 170, 60), 2)
            paint(color, uv[classes == "side", 0], uv[classes == "side", 1], (0, 140, 255), 1)
        write_png(OUT / f"{item['view']}.png", color)
        panels.append(panel(OUT / f"{item['view']}.png", item["view"], font))
    width = sum(image.width for image in panels)
    height = max(image.height for image in panels)
    comparison_image = Image.new("RGB", (width, height), (255, 255, 255))
    cursor = 0
    for image in panels:
        comparison_image.paste(image, (cursor, 0))
        cursor += image.width
    comparison_image.save(OUT / "view_comparison.png")
    normal_panels = []
    for item in measured:
        normal_panels.append(panel(OUT / f"{item['view']}.png", item["view"] + " 초록=위 주황=옆", font))
    normal_image = Image.new("RGB", (sum(image.width for image in normal_panels), max(image.height for image in normal_panels)), (255, 255, 255))
    cursor = 0
    for image in normal_panels:
        normal_image.paste(image, (cursor, 0))
        cursor += image.width
    normal_image.save(OUT / "normal_comparison.png")
    complement = Image.new("RGB", (840, 300), (255, 255, 255))
    draw = ImageDraw.Draw(complement)
    for index, name in enumerate(("view_b", "view_c")):
        source = Image.open(OUT / f"{name}.png")
        source.thumbnail((400, 250))
        complement.paste(source, (index * 420, 40))
        mark = "보완" if decisions[name]["complementary"] else "보완 아님"
        draw.text((index * 420 + 8, 8), f"{name} {mark}", fill=(0, 0, 0), font=font)
    complement.save(OUT / "complementary_geometry.png")
    lines = [
        "# P-008-R1M — 추가 시선이 보이는 기하",
        "",
        "카메라는 고정이고 신발만 돌렸다. 세 점군은 맞추지 않았다. 빈 곳을 mm로 메운다고 말하지 않는다.",
        "",
        "View A 옆면 점 %d, 지지 법선과의 평균각 %s°." % (
            baseline["side_points"],
            "없음" if baseline["mean_support_angle_deg"] is None else "%.1f" % baseline["mean_support_angle_deg"],
        ),
        "",
    ]
    for item in measured[1:]:
        decision = decisions[item["view"]]
        lines.append(
            "%s 작업 높이 %d점, 옆면 %d점, 옆면 유효 비율 %.3f, 이웃 각 중앙값 %s°, 평균 기울기 %s°. 옆면 규칙 %s, 기울기 규칙 %s." % (
                item["view"],
                item["elevated_points"],
                item["side_points"],
                item["side_valid_ratio"],
                "없음" if item["side_neighbor"]["median_deg"] is None else "%.2f" % item["side_neighbor"]["median_deg"],
                "없음" if item["mean_support_angle_deg"] is None else "%.1f" % item["mean_support_angle_deg"],
                "예" if decision["side_rule"] else "아니오",
                "예" if decision["tilt_rule"] else "아니오",
            )
        )
    lines.extend([
        "",
        "```text",
        "P-008-R1M = %s" % verdict,
        "REGISTRATION_PERFORMED = NO",
        "FUSION_PERFORMED = NO",
        "R1L_100MM_RETESTED = NO",
        "```",
        "",
        "다음 Gate는 열지 않는다.",
        "",
    ])
    (OUT / "report.md").write_text("\n".join(lines))
    result = {
        "verdict": verdict,
        "view_a_sha256": measured[0]["sha256"],
        "view_b_sha256": measured[1]["sha256"],
        "view_c_sha256": measured[2]["sha256"],
        "view_b_complementary": decisions["view_b"]["complementary"],
        "view_c_complementary": decisions["view_c"]["complementary"],
        "primary_view": primary,
        "primary_side_points": None if primary_report is None else primary_report["side_points"],
        "primary_side_valid_ratio": None if primary_report is None else primary_report["side_valid_ratio"],
        "primary_side_median_deg": None if primary_report is None else primary_report["side_neighbor"]["median_deg"],
        "primary_side_p95_deg": None if primary_report is None else primary_report["side_neighbor"]["p95_deg"],
        "primary_largest_side_voxels": None if primary_report is None else primary_report["largest_side_voxels"],
        "registration_performed": False,
        "fusion_performed": False,
        "multiview_registration_justified": passed,
    }
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({
        "verdict": verdict,
        "a_side": baseline["side_points"],
        "a_angle": baseline["mean_support_angle_deg"],
        "b": decisions["view_b"],
        "b_side": measured[1]["side_points"],
        "b_angle": measured[1]["mean_support_angle_deg"],
        "c": decisions["view_c"],
        "c_side": measured[2]["side_points"],
        "c_angle": measured[2]["mean_support_angle_deg"],
        "shoe": [item["shoe_found"] for item in measured],
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
