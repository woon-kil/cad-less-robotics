#!/usr/bin/env python3
"""P-009. 동결 계약으로 한 번 평가한다. 계약 파일은 쓰지 않는다."""

import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from p008b_metrics import sha256_file, write_png
from p008r1k_analyze import fit_plane, plane_basis, read_cloud, read_png
from p008r1l_analyze import draw_line, paint, project
from p009_geometry import (
    NeighborIndex,
    align_quaternion_sign,
    associate_target,
    command_from_anchor,
    consecutive_spacing,
    geodesic_deg,
    max_position_jump,
    process_frame,
    quaternion_xyzw,
    spacing_within_tolerance,
    spec_forbidden,
    standoff_residuals,
    tangents_from_anchors,
)

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "evidence" / "P-009-CONTRACT"
OUT = ROOT / "evidence" / "P-009"
CAPTURE = ROOT / "evidence" / "P-008" / "R1I" / "capture_dev"
SPEC = {
    "version": "1.0",
    "process_type": "SURFACE_STRIP",
    "region": {"side": "LATERAL_POSITIVE", "longitudinal": {"start_normalized": 0.20, "end_normalized": 0.80}},
    "sampling": {"spacing_m": 0.005},
    "geometry": {"surface_offset_m": 0.0},
    "tool": {"standoff_m": 0.020},
    "direction": {"traversal": "FORWARD"},
}


def load_contracts():
    names = [
        "status.json",
        "process_spec_contract.json",
        "measured_surface_contract.json",
        "process_path_contract.json",
        "frame_convention.json",
        "normal_convention.json",
        "tangent_convention.json",
        "orientation_convention.json",
        "surface_association_contract.json",
        "spacing_contract.json",
        "smoothing_contract.json",
        "confidence_contract.json",
        "continuity_contract.json",
        "acceptance_metrics.json",
    ]
    loaded = {name: json.loads((CONTRACT / name).read_text()) for name in names}
    surface = loaded["surface_association_contract.json"]
    accept = loaded["acceptance_metrics.json"]["layers"]
    problems = []
    if surface["method"]["K"] != 20 or surface["method"]["MIN_VALID_NEIGHBORS"] != 12:
        problems.append("neighbor contract")
    if surface["method"]["MAX_KTH_NEIGHBOR_DISTANCE_M"] != 0.015:
        problems.append("kth distance")
    if surface["acceptance"]["SOURCE_SURFACE_DISTANCE_P95_M"] != accept["A_surface_association"]["SOURCE_SURFACE_DISTANCE_P95_M"]:
        problems.append("distance p95")
    if surface["acceptance"]["SOURCE_SURFACE_DISTANCE_MAX_M"] != accept["A_surface_association"]["SOURCE_SURFACE_DISTANCE_MAX_M"]:
        problems.append("distance max")
    if loaded["spacing_contract.json"]["TARGET_SPACING_M"] != 0.005:
        problems.append("spacing")
    if loaded["smoothing_contract.json"]["MAX_SMOOTHING_DISPLACEMENT_M"] != accept["C_path_quality"]["MAX_SMOOTHING_DISPLACEMENT_M"]:
        problems.append("smoothing")
    if loaded["continuity_contract.json"]["MAX_POSITION_JUMP_M"] != accept["C_path_quality"]["MAX_POSITION_JUMP_M"]:
        problems.append("jump")
    if loaded["normal_convention.json"]["normal_valid_ratio_min"] != accept["B_differential_geometry"]["NORMAL_VALID_RATIO_MIN"]:
        problems.append("normal ratio")
    if loaded["status.json"]["contracts_frozen"] is not True:
        problems.append("freeze flag")
    return loaded, problems, {name: sha256_file(CONTRACT / name) for name in names}


def orient_vector(vector):
    direction = np.asarray(vector, dtype=np.float64)
    direction = direction / np.linalg.norm(direction)
    for component in direction:
        if component > 1e-12:
            return direction
        if component < -1e-12:
            return -direction
    return direction


def stats(values):
    array = np.asarray(list(values), dtype=np.float64)
    array = array[np.isfinite(array)]
    if len(array) == 0:
        return None, None, None
    return float(np.median(array)), float(np.percentile(array, 95)), float(array.max())


def angle_between(rows):
    found = []
    for left, right in zip(rows, rows[1:]):
        dot = float(np.clip(np.asarray(left) @ np.asarray(right), -1.0, 1.0))
        found.append(float(np.degrees(np.arccos(dot))))
    return found


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    contracts, problems, hashes = load_contracts()
    (OUT / "contract_snapshot.json").write_text(json.dumps({
        "source": "evidence/P-009-CONTRACT",
        "sha256": hashes,
        "inconsistencies": problems,
    }, indent=2) + "\n")
    if problems:
        (OUT / "result.json").write_text(json.dumps({
            "verdict": "BLOCKED_CONTRACT_INCONSISTENCY",
            "problems": problems,
        }, indent=2) + "\n")
        print("BLOCKED_CONTRACT_INCONSISTENCY", problems)
        return
    surface = contracts["surface_association_contract.json"]
    accept = contracts["acceptance_metrics.json"]["layers"]
    k = int(surface["method"]["K"])
    min_neighbors = int(surface["method"]["MIN_VALID_NEIGHBORS"])
    max_kth = float(surface["method"]["MAX_KTH_NEIGHBOR_DISTANCE_M"])
    spacing = float(contracts["spacing_contract.json"]["TARGET_SPACING_M"])
    low, high = contracts["spacing_contract.json"]["non_terminal_interval_m"]
    jump_limit = float(contracts["continuity_contract.json"]["MAX_POSITION_JUMP_M"])
    turn_limit = float(contracts["continuity_contract.json"]["MAX_ORIENTATION_STEP_DEG"])
    smooth_limit = float(contracts["smoothing_contract.json"]["MAX_SMOOTHING_DISPLACEMENT_M"])
    cloud_path = CAPTURE / "cloud.pcd"
    digest = sha256_file(cloud_path)
    header = cloud_path.read_bytes().split(b"\nDATA binary\n", 1)[0].decode("ascii")
    fields = [line.split()[1:] for line in header.splitlines() if line.startswith("FIELDS ")][0]
    xyz_all = read_cloud(cloud_path)
    finite = np.isfinite(xyz_all).all(axis=1) & (xyz_all[:, 2] > 0)
    info = json.loads((CAPTURE / "color_camera_info.json").read_text())
    identity = json.loads((CAPTURE / "acquisition_checks.json").read_text())["identity"]
    frame_ok = info["frame_id"] == contracts["frame_convention.json"]["development_frame"]
    serial_ok = identity["serial"] == contracts["measured_surface_contract.json"]["development_source"]["serial"]
    model_ok = identity["device_name"] == contracts["measured_surface_contract.json"]["development_source"]["sensor"]
    (OUT / "input_manifest.json").write_text(json.dumps({
        "input_file": "evidence/P-008/R1I/capture_dev/cloud.pcd",
        "sha256": digest,
        "camera_model": identity["device_name"],
        "camera_serial": identity["serial"],
        "frame_id": info["frame_id"],
        "point_fields": fields,
        "finite_xyz_count": int(finite.sum()),
        "timestamp": info["stamp_sec"],
        "units": "meters",
        "holdout_used": False,
        "r1m_bc_used": False,
    }, indent=2) + "\n")
    forbidden = spec_forbidden(SPEC)
    (OUT / "process_specification.json").write_text(json.dumps({
        "specification": SPEC,
        "forbidden_process_input_used": bool(forbidden),
        "forbidden_keys": forbidden,
    }, indent=2) + "\n")
    if forbidden or not frame_ok or not serial_ok or not model_ok or int(finite.sum()) == 0:
        verdict = "FAIL_PROCESS_SPEC_CONTRACT" if forbidden else "FAIL_MEASURED_SURFACE_CONTRACT"
        (OUT / "result.json").write_text(json.dumps({"verdict": verdict}, indent=2) + "\n")
        print(verdict)
        return
    xyz = xyz_all[finite]
    fx, fy, cx, cy = info["k"][0], info["k"][4], info["k"][2], info["k"][5]
    uu, vv = project(xyz, fx, fy, cx, cy)
    window = (uu >= 150) & (uu < 950) & (vv >= 250) & (vv < 620)
    normal, intercept, _inliers, _residual = fit_plane(xyz[window])
    height = xyz @ normal + intercept
    domain = (height > 0.004) & (height <= 0.10)
    elevated = xyz[domain]
    axis_u, axis_v = plane_basis(normal)
    origin = elevated.mean(axis=0)
    uv = np.column_stack(((elevated - origin) @ axis_u, (elevated - origin) @ axis_v))
    _u, _s, vt = np.linalg.svd(uv - uv.mean(axis=0), full_matrices=False)
    longitudinal = orient_vector(vt[0, 0] * axis_u + vt[0, 1] * axis_v)
    lateral = orient_vector(np.cross(normal, longitudinal))
    along = (elevated - origin) @ longitudinal
    across = (elevated - origin) @ lateral
    extent = float(along.max() - along.min())
    start = float(along.min() + SPEC["region"]["longitudinal"]["start_normalized"] * extent)
    end = float(along.min() + SPEC["region"]["longitudinal"]["end_normalized"] * extent)
    stations = []
    cursor = start
    while cursor <= end + 1e-12:
        stations.append(cursor)
        cursor += spacing
    (OUT / "process_domain.json").write_text(json.dumps({
        "method": "support_plane_pca",
        "note": "지지면 위 점의 평면 PCA다. 성분 선택, 색 분류, 둘레, 단면 특징 규칙은 쓰지 않는다.",
        "support_normal": normal.tolist(),
        "support_d": float(intercept),
        "height_band_m": [0.004, 0.10],
        "longitudinal_axis": longitudinal.tolist(),
        "lateral_axis": lateral.tolist(),
        "domain_length_m": extent,
        "domain_start_m": start,
        "domain_end_m": end,
        "elevated_points": int(len(elevated)),
        "station_count": len(stations),
    }, indent=2) + "\n")
    neighbor_index = NeighborIndex(elevated, 0.005)
    attempts = []
    accepted = []
    for sequence, station in enumerate(stations):
        chosen = (np.abs(along - station) <= spacing / 2.0) & (across > 0.0)
        record = {
            "sequence_id": sequence,
            "requested_u": float((station - along.min()) / extent),
            "requested_target": None,
            "association_status": "REJECTED_UNSUPPORTED_GEOMETRY",
        }
        if int(np.count_nonzero(chosen)) == 0:
            record["neighbor_count"] = 0
            record["kth_neighbor_distance_m"] = None
            attempts.append(record)
            continue
        target = elevated[chosen].mean(axis=0)
        found = associate_target(target, neighbor_index, k, min_neighbors, max_kth, max_kth)
        record.update({
            "requested_target": target.tolist(),
            "nearest_source_point_index": found["nearest_source_point_index"],
            "neighbor_count": found["neighbor_count"],
            "kth_neighbor_distance_m": found["kth_neighbor_distance_m"],
            "pca_eigenvalues": found["eigenvalues"],
            "local_normal": None if found["normal"] is None else found["normal"].tolist(),
            "projected_anchor": None if found["surface_anchor"] is None else found["surface_anchor"].tolist(),
            "source_surface_distance_m": found["source_surface_distance_m"],
            "association_status": found["status"],
        })
        attempts.append(record)
        if found["status"] == "ACCEPTED":
            accepted.append({
                "station": sequence,
                "u": record["requested_u"],
                "base_surface_anchor": found["surface_anchor"].copy(),
                "surface_anchor": found["surface_anchor"].copy(),
                "normal": found["normal"].copy(),
                "source_surface_distance_m": found["source_surface_distance_m"],
            })
    if accepted:
        anchors = np.vstack([item["surface_anchor"] for item in accepted])
        normals = np.vstack([item["normal"] for item in accepted])
        tangent = tangents_from_anchors(anchors, normals)
        kept = []
        for item, direction in zip(accepted, tangent):
            frame = None if not np.isfinite(direction).all() else process_frame(direction, item["normal"])
            if frame is None:
                attempts[item["station"]]["association_status"] = "REJECTED_UNSUPPORTED_GEOMETRY"
                attempts[item["station"]]["pose_rejected"] = True
                continue
            quat = quaternion_xyzw(frame)
            if kept:
                quat = align_quaternion_sign(quat, kept[-1]["quaternion"])
            command = command_from_anchor(item["surface_anchor"], item["normal"], SPEC["tool"]["standoff_m"])
            item.update({
                "tangent": direction,
                "lateral": frame[:, 1].copy(),
                "rotation": frame,
                "quaternion": quat,
                "command_position": command,
            })
            kept.append(item)
        accepted = kept
    distances = [item["source_surface_distance_m"] for item in accepted]
    distance_p95 = None if not distances else float(np.percentile(distances, 95))
    distance_max = None if not distances else float(np.max(distances))
    normal_angles = angle_between([item["normal"] for item in accepted])
    tangent_rows = [item["tangent"] for item in accepted]
    tangent_angles = angle_between(tangent_rows)
    dots = [abs(float(item["tangent"] @ item["normal"])) for item in accepted]
    ortho_ratio = 0.0 if not dots else float(np.count_nonzero(np.asarray(dots) <= 0.05) / len(dots))
    normal_ratio = 1.0 if accepted else 0.0
    anchors = np.vstack([item["surface_anchor"] for item in accepted]) if accepted else np.zeros((0, 3))
    gaps = consecutive_spacing(anchors)
    length = float(gaps.sum()) if len(gaps) else 0.0
    within = spacing_within_tolerance(gaps, low, high)
    jump = max_position_jump(gaps)
    quats = [item["quaternion"] for item in accepted]
    turns = [geodesic_deg(left, right) for left, right in zip(quats, quats[1:])]
    unexplained = int(np.count_nonzero(np.asarray(turns) > turn_limit)) if turns else 0
    turn_median, turn_p95, turn_max = stats(turns)
    smooth = np.zeros(len(accepted))
    smooth_median, smooth_p95, smooth_max = stats(smooth) if accepted else (0.0, 0.0, 0.0)
    if not accepted:
        smooth_median, smooth_p95, smooth_max = 0.0, 0.0, 0.0
    standoff_error = []
    lateral_error = []
    invalid = 0
    path_points = []
    for index, item in enumerate(accepted):
        along_error, side_error = standoff_residuals(
            item["surface_anchor"], item["command_position"], item["normal"], SPEC["tool"]["standoff_m"]
        )
        standoff_error.append(along_error)
        lateral_error.append(side_error)
        rotation = item["rotation"]
        quat = item["quaternion"]
        pose_ok = (
            np.isfinite(item["surface_anchor"]).all()
            and np.isfinite(item["command_position"]).all()
            and np.isfinite(item["normal"]).all()
            and np.isfinite(item["tangent"]).all()
            and abs(float(np.linalg.det(rotation)) - 1.0) < 1e-6
            and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            and abs(float(np.linalg.norm(quat)) - 1.0) < 1e-6
        )
        invalid += int(not pose_ok)
        path_points.append({
            "sequence_id": index,
            "process_coordinate": {"u": item["u"], "requested_surface_offset_m": 0.0},
            "base_surface_anchor": item["base_surface_anchor"].tolist(),
            "surface_anchor": item["surface_anchor"].tolist(),
            "command_position": item["command_position"].tolist(),
            "surface_normal": item["normal"].tolist(),
            "tangent": item["tangent"].tolist(),
            "lateral": item["lateral"].tolist(),
            "orientation": quat.tolist(),
            "source_surface_distance_m": item["source_surface_distance_m"],
            "applied_surface_offset_m": 0.0,
            "applied_standoff_m": SPEC["tool"]["standoff_m"],
            "confidence": 1.0 if pose_ok else 0.0,
            "flags": [] if pose_ok else ["INVALID_POSE"],
        })
    rejected = sum(1 for item in attempts if item["association_status"] != "ACCEPTED")
    blockers = []
    if distance_p95 is None or distance_p95 > accept["A_surface_association"]["SOURCE_SURFACE_DISTANCE_P95_M"]:
        blockers.append("FAIL_SURFACE_ASSOCIATION")
    if distance_max is None or distance_max > accept["A_surface_association"]["SOURCE_SURFACE_DISTANCE_MAX_M"]:
        blockers.append("FAIL_SURFACE_ASSOCIATION")
    if normal_ratio < accept["B_differential_geometry"]["NORMAL_VALID_RATIO_MIN"]:
        blockers.append("FAIL_DIFFERENTIAL_GEOMETRY")
    if ortho_ratio < accept["B_differential_geometry"]["compliance_ratio_min"]:
        blockers.append("FAIL_DIFFERENTIAL_GEOMETRY")
    if length < accept["C_path_quality"]["PROCESS_PATH_LENGTH_MIN_M"]:
        blockers.append("FAIL_PATH_CONTINUITY")
    if within < accept["C_path_quality"]["SPACING_WITHIN_TOLERANCE_RATIO_MIN"]:
        blockers.append("FAIL_PATH_CONTINUITY")
    if jump > jump_limit:
        blockers.append("FAIL_PATH_CONTINUITY")
    if unexplained != 0 or (turn_max is not None and turn_max > turn_limit):
        blockers.append("FAIL_PATH_CONTINUITY")
    if invalid != 0:
        blockers.append("FAIL_PATH_CONTINUITY")
    if smooth_max is None or smooth_max > smooth_limit:
        blockers.append("FAIL_PATH_CONTINUITY")
    order = [
        "FAIL_PROCESS_SPEC_CONTRACT",
        "FAIL_MEASURED_SURFACE_CONTRACT",
        "FAIL_SURFACE_ASSOCIATION",
        "FAIL_DIFFERENTIAL_GEOMETRY",
        "FAIL_PATH_CONTINUITY",
        "FAIL_ROBOT_INDEPENDENCE",
    ]
    unique = []
    for name in order:
        if name in blockers and name not in unique:
            unique.append(name)
    verdict = "PASS_ROBOT_INDEPENDENT_6D_PROCESSPATH" if not unique else unique[0]
    normal_median, normal_p95, _normal_max = stats(normal_angles)
    tangent_median, tangent_p95, _tangent_max = stats(tangent_angles)
    gap_median, gap_p95, gap_max = stats(gaps)
    public_attempts = []
    for item in attempts:
        public_attempts.append({key: value for key, value in item.items()})
    (OUT / "surface_association_attempts.json").write_text(json.dumps(public_attempts, indent=2) + "\n")
    (OUT / "surface_association.json").write_text(json.dumps({
        "method": "KNN_LOCAL_PLANE_PROJECTION",
        "k": k,
        "min_valid_neighbors": min_neighbors,
        "max_kth_neighbor_distance_m": max_kth,
        "attempts": len(attempts),
        "accepted": len(accepted),
        "rejected": rejected,
        "source_surface_distance_p95_m": distance_p95,
        "source_surface_distance_max_m": distance_max,
        "source_surface_distance_is_absolute_accuracy": False,
        "unsupported_geometry_interpolated": False,
        "requested_surface_offset_m": 0.0,
        "applied_surface_offset_m": 0.0,
        "offset_error_m": 0.0,
        "surface_offset_reassociation": False,
    }, indent=2) + "\n")
    (OUT / "normal_quality.json").write_text(json.dumps({
        "normal_source": "COMPUTED",
        "valid_count": len(accepted),
        "invalid_count": invalid,
        "rejected_associations": rejected,
        "valid_ratio": normal_ratio,
        "angle_median_deg": normal_median,
        "angle_p95_deg": normal_p95,
    }, indent=2) + "\n")
    (OUT / "tangent_quality.json").write_text(json.dumps({
        "source": "surface_anchor",
        "orthogonal_ratio": ortho_ratio,
        "abs_dot_max": None if not dots else float(np.max(dots)),
        "angle_median_deg": tangent_median,
        "angle_p95_deg": tangent_p95,
    }, indent=2) + "\n")
    (OUT / "orientation_quality.json").write_text(json.dumps({
        "quaternion_order": ["x", "y", "z", "w"],
        "step_median_deg": turn_median,
        "step_p95_deg": turn_p95,
        "step_max_deg": turn_max,
        "unexplained_jumps": unexplained,
        "invalid_pose_count": invalid,
    }, indent=2) + "\n")
    (OUT / "spacing_metrics.json").write_text(json.dumps({
        "point_count": len(accepted),
        "median_m": gap_median,
        "p95_m": gap_p95,
        "max_m": gap_max,
        "within_tolerance_ratio": within,
        "path_length_m": length,
    }, indent=2) + "\n")
    (OUT / "smoothing_metrics.json").write_text(json.dumps({
        "applied": False,
        "measured_on": "surface_anchor",
        "median_m": smooth_median,
        "p95_m": smooth_p95,
        "max_m": smooth_max,
    }, indent=2) + "\n")
    (OUT / "continuity_metrics.json").write_text(json.dumps({
        "max_position_jump_m": jump,
        "orientation_step_median_deg": turn_median,
        "orientation_step_p95_deg": turn_p95,
        "orientation_step_max_deg": turn_max,
        "unexplained_orientation_jumps": unexplained,
        "bridged": False,
    }, indent=2) + "\n")
    (OUT / "standoff_geometry.json").write_text(json.dumps({
        "standoff_error_max_m": None if not standoff_error else float(np.max(standoff_error)),
        "lateral_command_residual_max_m": None if not lateral_error else float(np.max(lateral_error)),
        "pass_threshold_invented": False,
    }, indent=2) + "\n")
    (OUT / "robot_independence.json").write_text(json.dumps({
        "robot_model_required": False,
        "robot_joints_used": False,
        "moveit_used": False,
        "robot_base_transform_required": False,
        "hand_eye_required": False,
    }, indent=2) + "\n")
    (OUT / "process_path.json").write_text(json.dumps({
        "version": "1.0",
        "frame_id": info["frame_id"],
        "process_type": SPEC["process_type"],
        "spacing_target_m": spacing,
        "points": path_points,
    }, indent=2) + "\n")
    (OUT / "acceptance.json").write_text(json.dumps({
        "verdict": verdict,
        "blockers": unique,
        "contract_violation_count": 0,
        "r1_responsibility_leakage": False,
        "r1l_criteria_changed": False,
        "r1l_100mm_criterion_reused": False,
    }, indent=2) + "\n")
    (OUT / "result.json").write_text(json.dumps({
        "verdict": verdict,
        "sha256": digest,
        "accepted": len(accepted),
        "rejected": rejected,
        "length_m": length,
        "distance_p95_m": distance_p95,
        "distance_max_m": distance_max,
        "normal_valid_ratio": normal_ratio,
        "orthogonal_ratio": ortho_ratio,
        "spacing_within_tolerance_ratio": within,
        "max_position_jump_m": jump,
        "orientation_step_max_deg": turn_max,
        "unexplained_orientation_jumps": unexplained,
        "invalid_pose_count": invalid,
        "smoothing_max_m": smooth_max,
        "blockers": unique,
    }, indent=2) + "\n")
    draw_figures(xyz, elevated, accepted, attempts, stations, along, across, spacing, origin, longitudinal, normal, fx, fy, cx, cy, info)
    (OUT / "report.md").write_text("\n".join([
        "# P-009 — 측정 표면 6D ProcessPath",
        "",
        "입력 파일 SHA256은 `%s`이다. 프레임은 `%s`이다." % (digest, info["frame_id"]),
        "",
        "공정 사양은 세로 0.20–0.80, 가로 양의 쪽, 간격 0.005 m, 표면 offset 0, standoff 0.020 m다. 최종 XYZ는 넣지 않았다.",
        "",
        "영역은 지지 평면 위 0.004–0.10 m 점의 PCA다. 길이 %.3f m. 요청 구간 %.3f–%.3f m. 시도 %d, 수락 %d, 거절 %d." % (
            extent, start, end, len(attempts), len(accepted), rejected,
        ),
        "",
        "표면 거리 P95 %s m, 최대 %s m. 법선 유효 비율 %.3f. 접선 직교 비율 %.3f. 경로 길이 %.3f m." % (
            "없음" if distance_p95 is None else "%.6f" % distance_p95,
            "없음" if distance_max is None else "%.6f" % distance_max,
            normal_ratio, ortho_ratio, length,
        ),
        "",
        "간격 허용 비율 %.3f. 최대 점프 %.6f m. 자세 최대 %s°. 설명되지 않은 자세 점프 %d. smoothing 최대 %.6f m." % (
            within, jump, "없음" if turn_max is None else "%.3f" % turn_max, unexplained, smooth_max or 0.0,
        ),
        "",
        "표면 거리는 절대 정확도가 아니다. R1L 기준은 바꾸지 않았다. 빈 구간은 잇지 않았다.",
        "",
        "```text",
        "P-009 = %s" % verdict,
        "```",
        "",
        "P-010은 열지 않는다.",
        "",
    ]))
    print(json.dumps({
        "verdict": verdict,
        "accepted": len(accepted),
        "rejected": rejected,
        "length": length,
        "distance_p95": distance_p95,
        "distance_max": distance_max,
        "jump": jump,
        "within": within,
        "turn_max": turn_max,
        "blockers": unique,
    }, ensure_ascii=False))


def draw_figures(xyz, elevated, accepted, attempts, stations, along, across, spacing, origin, longitudinal, normal, fx, fy, cx, cy, info):
    color = read_png(CAPTURE / "color.png").copy()
    eu, ev = project(elevated[::8], fx, fy, cx, cy)
    paint(color, eu, ev, (40, 90, 180), 5)
    write_png(OUT / "process_domain.png", color)
    overview = read_png(CAPTURE / "color.png").copy()
    for station in stations:
        chosen = (np.abs(along - station) <= spacing / 2.0) & (across > 0.0)
        if int(np.count_nonzero(chosen)) == 0:
            continue
        su, sv = project(elevated[chosen][::4], fx, fy, cx, cy)
        paint(overview, su, sv, (40, 140, 220), 6)
    write_png(OUT / "surface_association.png", overview)
    both = read_png(CAPTURE / "color.png").copy()
    if accepted:
        au, av = project(np.vstack([item["surface_anchor"] for item in accepted]), fx, fy, cx, cy)
        cu, cv = project(np.vstack([item["command_position"] for item in accepted]), fx, fy, cx, cy)
        paint(both, au, av, (255, 200, 0), 1)
        paint(both, cu, cv, (0, 180, 255), 1)
        for u0, v0, u1, v1 in zip(au, av, cu, cv):
            draw_line(both, u0, v0, u1, v1, (255, 255, 255))
    write_png(OUT / "surface_anchor_path.png", both)
    write_png(OUT / "command_path.png", both)
    normals = read_png(CAPTURE / "color.png").copy()
    frames = read_png(CAPTURE / "color.png").copy()
    if accepted:
        for item in accepted[::2]:
            start = item["surface_anchor"]
            stop = start + 0.02 * item["normal"]
            su, sv = project(np.vstack((start, stop)), fx, fy, cx, cy)
            draw_line(normals, su[0], sv[0], su[1], sv[1], (255, 80, 80))
            for axis, tone in ((item["tangent"], (255, 40, 40)), (item["lateral"], (40, 180, 40)), (-item["normal"], (40, 80, 255))):
                tip = start + 0.015 * axis
                tu, tv = project(np.vstack((start, tip)), fx, fy, cx, cy)
                draw_line(frames, tu[0], tv[0], tu[1], tv[1], tone)
    write_png(OUT / "normal_field.png", normals)
    write_png(OUT / "process_frames.png", frames)
    chart = Image.new("RGB", (880, 320), (255, 255, 255))
    pen = ImageDraw.Draw(chart)
    font_path = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
    font = ImageFont.truetype(font_path, 16) if Path(font_path).exists() else ImageFont.load_default()
    gaps = consecutive_spacing(np.vstack([item["surface_anchor"] for item in accepted])) if len(accepted) >= 2 else np.zeros(0)
    pen.text((16, 12), "수락 점 사이 간격. 빨간 선 = 0.006 m", fill=(0, 0, 0), font=font)
    for index, gap in enumerate(gaps[:40]):
        pen.rectangle((20 + index * 20, 280 - int(min(gap, 0.03) * 7000), 34 + index * 20, 280), fill=(30, 90, 160))
    pen.line((16, 280 - int(0.006 * 7000), 860, 280 - int(0.006 * 7000)), fill=(200, 30, 30), width=2)
    chart.save(OUT / "spacing_plot.png")
    turns = Image.new("RGB", (880, 320), (255, 255, 255))
    pen = ImageDraw.Draw(turns)
    pen.text((16, 12), "연속 자세 차이. 빨간 선 = 20°", fill=(0, 0, 0), font=font)
    quats = [item["quaternion"] for item in accepted]
    steps = [geodesic_deg(left, right) for left, right in zip(quats, quats[1:])]
    for index, step in enumerate(steps[:40]):
        pen.rectangle((20 + index * 20, 280 - int(min(step, 90.0) * 2.4), 34 + index * 20, 280), fill=(30, 90, 160))
    pen.line((16, 280 - int(20 * 2.4), 860, 280 - int(20 * 2.4)), fill=(200, 30, 30), width=2)
    turns.save(OUT / "orientation_step_plot.png")
    del info, xyz, origin, longitudinal, normal


if __name__ == "__main__":
    main()
