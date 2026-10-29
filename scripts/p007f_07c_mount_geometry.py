#!/usr/bin/env python3
"""P-007F-07C — M2 marker 면 방향 설계. 촬영·로봇·마커는 건드리지 않는다.

영상 꼭짓점 각 편차를 물리 장착각으로 쓰지 않는다.
pose는 저장된 Gemini color intrinsics와 50 mm 정사각형 solvePnP로만 추정한다.
"""
import csv
import json
import math
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DET_CSV = os.path.join(
    ROOT, "evidence", "P-007F-07", "marker_static_qualification_r1", "attempt-02", "marker_detection.csv")
FRAME_PATH = os.path.join(
    ROOT, "evidence", "P-007F-07", "marker_static_qualification_r1", "attempt-02",
    "raw", "representative_detected", "000128-color.png")
INTRINSICS_PATH = os.path.join(ROOT, "evidence", "P-006", "camera-target", "color-camera-info.yaml")
OUT_DIR = os.path.join(ROOT, "evidence", "P-007F-07", "marker_mount_geometry_redesign")
FIG_DIR = os.path.join(OUT_DIR, "figures")
FONT_PATH = "/System/Library/Fonts/Supplemental/AppleGothic.ttf"

# P-006 color camera_info. 1280x720. 이후 P-007F-03/05 프레임의 k와 같은 값이다.
# 초점거리를 새로 가정하지 않는다.
K = np.array([
    [610.5073852539062, 0.0, 641.7918090820312],
    [0.0, 610.349853515625, 364.4433288574219],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
DIST = np.array([
    -0.03170262649655342, 0.0341622568666935, 7.964719407027587e-05,
    -6.973547897359822e-06, -0.01190638542175293, 0.0, 0.0, 0.0,
], dtype=np.float64)
MARKER_M = 0.050
QUIET_ZONE_MM = 3.0


def load_detections():
    rows = []
    with open(DET_CSV, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["id0_detected"] != "True":
                continue
            corners = np.array([
                [float(row["corner_0_x"]), float(row["corner_0_y"])],
                [float(row["corner_1_x"]), float(row["corner_1_y"])],
                [float(row["corner_2_x"]), float(row["corner_2_y"])],
                [float(row["corner_3_x"]), float(row["corner_3_y"])],
            ], dtype=np.float64)
            rows.append({"frame_index": int(row["frame_index"]), "corners": corners})
    return rows


def image_geometry(corners):
    sides = [float(np.linalg.norm(corners[i] - corners[(i + 1) % 4])) for i in range(4)]
    diagonals = [float(np.linalg.norm(corners[0] - corners[2])), float(np.linalg.norm(corners[1] - corners[3]))]
    angles = []
    for i in range(4):
        v1 = corners[(i - 1) % 4] - corners[i]
        v2 = corners[(i + 1) % 4] - corners[i]
        denom = float(np.linalg.norm(v1) * np.linalg.norm(v2)) + 1e-12
        angles.append(float(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / denom, -1.0, 1.0)))))
    area = float(cv2.contourArea(corners.astype(np.float32)))
    opposite = [
        sides[0] / sides[2], sides[2] / sides[0], sides[1] / sides[3], sides[3] / sides[1],
    ]
    return {
        "corners": corners.tolist(),
        "sides_px": sides,
        "diagonals_px": diagonals,
        "corner_angles_deg": angles,
        "max_corner_deviation_from_90_deg": max(abs(a - 90.0) for a in angles),
        "opposite_side_ratio_max": max(opposite),
        "aspect_ratio": max(sides) / min(sides),
        "area_px": area,
        "center_px": corners.mean(axis=0).tolist(),
    }


def stats(values):
    arr = np.asarray(values, dtype=float)
    return {
        "n": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "median": float(np.median(arr)),
        "min": float(arr.min()),
        "max": float(arr.max()),
    }


def object_points():
    half = MARKER_M / 2.0
    # detectMarkers 순서 0,1,2,3 = TL, TR, BR, BL.
    # OpenCV aruco CCW_center 객체점과 같은 순서. +Z는 마커 면에서 바깥쪽.
    return np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)


def pose_of(corners, obj):
    ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        obj, corners, K, DIST, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    best = None
    for rvec, tvec, err in zip(rvecs, tvecs, errors):
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
        if tvec[2] <= 0:
            continue
        reproj, _ = cv2.projectPoints(obj, rvec, tvec, K, DIST)
        reproj = reproj.reshape(4, 2)
        err_px = float(np.linalg.norm(reproj - corners, axis=1).mean())
        item = (err_px, rvec, tvec, float(np.asarray(err).reshape(-1)[0]))
        if best is None or item[0] < best[0]:
            best = item
    return best


def rotation_about(axis, angle_rad):
    return cv2.Rodrigues(np.asarray(axis, dtype=np.float64) * angle_rad)[0]


def summarize_projection(rvec, tvec, obj):
    reproj, _ = cv2.projectPoints(obj, rvec, tvec, K, DIST)
    return image_geometry(reproj.reshape(4, 2))


def main():
    os.makedirs(FIG_DIR, exist_ok=True)
    rows = load_detections()
    obj = object_points()
    per_frame = []
    image_sides = []
    image_devs = []
    image_aspects = []
    image_areas = []
    image_opposites = []
    image_diags = []
    for row in rows:
        geom = image_geometry(row["corners"])
        pose = pose_of(row["corners"], obj)
        if pose is None:
            continue
        err_px, rvec, tvec, solver_err = pose
        rot, _ = cv2.Rodrigues(rvec)
        normal = rot[:, 2]
        view_to_camera = -tvec / np.linalg.norm(tvec)
        incidence = float(np.degrees(np.arccos(np.clip(np.dot(normal, view_to_camera), -1.0, 1.0))))
        # 시선 기준이 아니라 카메라 축 기준의 분해. 설계각으로 바로 쓰지 않고 안정성만 본다.
        yaw = float(np.degrees(np.arctan2(normal[0], -normal[2])))
        pitch = float(np.degrees(np.arctan2(normal[1], -normal[2])))
        distance = float(np.linalg.norm(tvec))
        per_frame.append({
            "frame_index": row["frame_index"],
            "reprojection_error_px": err_px,
            "solver_error": solver_err,
            "distance_m": distance,
            "incidence_to_viewing_ray_deg": incidence,
            "normal_camera": normal.tolist(),
            "tvec_m": tvec.tolist(),
            "yaw_of_normal_deg": yaw,
            "pitch_of_normal_deg": pitch,
            "image_max_corner_deviation_from_90_deg": geom["max_corner_deviation_from_90_deg"],
        })
        image_sides.append(geom["sides_px"])
        image_devs.append(geom["max_corner_deviation_from_90_deg"])
        image_aspects.append(geom["aspect_ratio"])
        image_areas.append(geom["area_px"])
        image_opposites.append(geom["opposite_side_ratio_max"])
        image_diags.append(geom["diagonals_px"])

    # 대표 pose는 reprojection이 가장 작은 프레임. 각도의 설계값은 중앙값.
    representative = min(per_frame, key=lambda item: item["reprojection_error_px"])
    rep_row = next(row for row in rows if row["frame_index"] == representative["frame_index"])
    err_px, rvec, tvec, _ = pose_of(rep_row["corners"], obj)
    rot, _ = cv2.Rodrigues(rvec)
    normal = rot[:, 2]
    view_to_camera = -tvec / np.linalg.norm(tvec)
    axis = np.cross(normal, view_to_camera)
    axis = axis / np.linalg.norm(axis)
    incidence_med = float(np.median([item["incidence_to_viewing_ray_deg"] for item in per_frame]))

    def candidate(name, fraction):
        angle = math.radians(incidence_med * fraction)
        corr = rotation_about(axis, angle)
        rot_new = corr @ rot
        rvec_new, _ = cv2.Rodrigues(rot_new)
        # 회전 중심은 마커 원점. t는 유지한다.
        projected = summarize_projection(rvec_new, tvec, obj)
        normal_new = rot_new[:, 2]
        incidence_new = float(np.degrees(np.arccos(np.clip(np.dot(normal_new, view_to_camera), -1.0, 1.0))))
        corners_3d = (rot_new @ obj.T).T + tvec
        current_3d = (rot @ obj.T).T + tvec
        lift_mm = float(np.linalg.norm(corners_3d - current_3d, axis=1).max() * 1000.0)
        toward = int(np.argmin(corners_3d[:, 2]))  # 카메라에 가장 가까워지는 꼭짓점
        return {
            "name": name,
            "fraction_of_incidence": fraction,
            "correction_deg": incidence_med * fraction,
            "evidence_level": "ESTIMATED" if fraction else "MEASURED_IMAGE_ONLY",
            "remaining_incidence_deg": incidence_new,
            "max_corner_travel_mm": lift_mm,
            "corner_index_moved_toward_camera": toward,
            "projected": {
                "sides_px": projected["sides_px"],
                "side_mean_px": float(np.mean(projected["sides_px"])),
                "aspect_ratio": projected["aspect_ratio"],
                "opposite_side_ratio_max": projected["opposite_side_ratio_max"],
                "area_px": projected["area_px"],
                "max_corner_deviation_from_90_deg": projected["max_corner_deviation_from_90_deg"],
                "corners_px": projected["corners"],
            },
        }

    current_proj = summarize_projection(rvec, tvec, obj)
    candidates = [
        candidate("A_current", 0.0),
        candidate("B_partial", 0.5),
        candidate("C_face_viewing_ray", 1.0),
    ]
    # 대표 프레임의 현재 투영을 A에 덮어, 검출 꼭짓점과 어긋난 재투영이 아님을 남긴다.
    candidates[0]["projected_from_detected_corners"] = image_geometry(rep_row["corners"])
    candidates[0]["note"] = "A의 설계 비교는 재투영이고, detected_corners는 그 프레임의 실제 검출이다."

    # 부분 보정(B)을 기본 추천으로 둔다. 전량 보정(C)의 모서리 이동이 20 mm를 넘으면
    # 브래킷 밖으로 많이 나오므로 C를 설치안으로 고르지 않는다.
    chosen = candidates[1]
    if candidates[2]["max_corner_travel_mm"] <= 20.0:
        chosen = candidates[2]
        decision = "MOUNT_REDIRECTION_RECOMMENDED"
        decision_reason = "시선 정렬에 필요한 모서리 이동이 20 mm 이하다."
    else:
        decision = "MOUNT_REDIRECTION_RECOMMENDED"
        decision_reason = "시선까지의 전량 보정은 모서리 이동이 커서, 절반만 돌리는 안을 고른다."

    image = cv2.imread(FRAME_PATH)
    draw_figures(image, rep_row["corners"], candidates, chosen, axis, tvec, rot)

    side_arr = np.asarray(image_sides)
    diag_arr = np.asarray(image_diags)
    analysis = {
        "gate": "P-007F-07C",
        "image_distortion_is_not_physical_mount_angle": "30.8° image distortion != 30.8° physical mounting correction",
        "intrinsics": {
            "source": INTRINSICS_PATH,
            "also_identical_in": [
                "evidence/P-007F-03/frames",
                "evidence/P-007F-05/run_1793274484",
            ],
            "width": 1280,
            "height": 720,
            "frame_id": "camera_color_optical_frame",
            "distortion_model": "rational_polynomial",
            "k": K.reshape(-1).tolist(),
            "d": DIST.tolist(),
            "focal_length_assumed": False,
        },
        "marker_square_m": MARKER_M,
        "object_point_convention": "TL TR BR BL matches OpenCV aruco CCW_center object points; +Z out of the printed face",
        "solve_method": "SOLVEPNP_IPPE_SQUARE",
        "detected_frame_count": len(per_frame),
        "image_plane": {
            "side_px": {str(i): stats(side_arr[:, i]) for i in range(4)},
            "side_mean_px": stats(side_arr.mean(axis=1)),
            "diagonal_px": {str(i): stats(diag_arr[:, i]) for i in range(2)},
            "max_corner_deviation_from_90_deg": stats(image_devs),
            "aspect_ratio": stats(image_aspects),
            "opposite_side_ratio_max": stats(image_opposites),
            "area_px": stats(image_areas),
            "center_px": {
                "x": stats([row["corners"][:, 0].mean() for row in rows]),
                "y": stats([row["corners"][:, 1].mean() for row in rows]),
            },
        },
        "pose": {
            "incidence_to_viewing_ray_deg": stats([item["incidence_to_viewing_ray_deg"] for item in per_frame]),
            "distance_m": stats([item["distance_m"] for item in per_frame]),
            "reprojection_error_px": stats([item["reprojection_error_px"] for item in per_frame]),
            "yaw_of_normal_deg": stats([item["yaw_of_normal_deg"] for item in per_frame]),
            "pitch_of_normal_deg": stats([item["pitch_of_normal_deg"] for item in per_frame]),
            "normal_median_camera": np.median([item["normal_camera"] for item in per_frame], axis=0).tolist(),
            "representative_frame_index": representative["frame_index"],
            "definitions": {
                "incidence": "angle between marker +Z and the direction from the marker origin toward the camera. 0 means the face looks along the viewing ray.",
                "yaw_pitch": "atan2 of the normal against camera -Z. These are not the plate wedge angle.",
                "image_corner_deviation": "not a physical angle",
            },
        },
        "candidates": candidates,
        "chosen_candidate": chosen["name"],
        "decision": decision,
        "decision_reason": decision_reason,
        "plate": {
            "minimum_width_mm": 50.0 + 2.0 * QUIET_ZONE_MM,
            "minimum_height_mm": 50.0 + 2.0 * QUIET_ZONE_MM,
            "recommended_width_mm": 56.0,
            "recommended_height_mm": 56.0,
            "optional_larger_width_mm": 70.0,
            "optional_larger_height_mm": 70.0,
            "recommended_size_level": "APPROXIMATE",
            "minimum_size_basis": "50 mm black square plus the print sheet quiet-zone guide of 3 mm per side",
            "wedge_angle_deg": chosen["correction_deg"],
            "wedge_angle_level": "ESTIMATED",
            "hinge": "image-left edge, DaBai side. opposite corner index 2 which moves toward the camera",
            "lift_side": "image-right edge, toward the monitor and toward Gemini",
            "attachment": "removable tape or clip on the existing M2 card face. no screw into a servo, no glue on a moving shaft",
        },
        "interference": {
            "result": "POTENTIAL_INTERFERENCE",
            "basis": "still image plus the estimated lift direction. no motion and no mesh sweep",
            "lift_direction": "corner 2, image right, away from DaBai",
            "dabai": "immediately image-left of the card. a plate wider than the current card can overhang into it",
            "id2_plus_minus_3_deg": "not swept",
            "collision_free_claimed": False,
        },
        "robot_command_count": 0,
        "servo_write_count": 0,
        "torque_command_count": 0,
        "physical_change": False,
    }
    with open(os.path.join(OUT_DIR, "geometry_analysis.json"), "w", encoding="utf-8") as handle:
        json.dump(analysis, handle, ensure_ascii=False, indent=1)
    with open(os.path.join(OUT_DIR, "pose_estimates.csv"), "w", newline="", encoding="utf-8") as handle:
        fields = ["frame_index", "reprojection_error_px", "distance_m", "incidence_to_viewing_ray_deg",
                  "yaw_of_normal_deg", "pitch_of_normal_deg"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for item in per_frame:
            writer.writerow({key: item[key] for key in fields})
    print(json.dumps({
        "n": len(per_frame),
        "image_dev_median": analysis["image_plane"]["max_corner_deviation_from_90_deg"]["median"],
        "incidence_median": analysis["pose"]["incidence_to_viewing_ray_deg"]["median"],
        "incidence_std": analysis["pose"]["incidence_to_viewing_ray_deg"]["std"],
        "reproj_mean": analysis["pose"]["reprojection_error_px"]["mean"],
        "reproj_max": analysis["pose"]["reprojection_error_px"]["max"],
        "distance_m": analysis["pose"]["distance_m"]["median"],
        "chosen": chosen["name"],
        "correction_deg": chosen["correction_deg"],
        "lift_b_mm": candidates[1]["max_corner_travel_mm"],
        "lift_c_mm": candidates[2]["max_corner_travel_mm"],
        "area_a": candidates[0]["projected"]["area_px"],
        "area_b": candidates[1]["projected"]["area_px"],
        "area_c": candidates[2]["projected"]["area_px"],
        "aspect_a": candidates[0]["projected"]["aspect_ratio"],
        "aspect_b": candidates[1]["projected"]["aspect_ratio"],
        "aspect_c": candidates[2]["projected"]["aspect_ratio"],
        "toward": chosen["corner_index_moved_toward_camera"],
    }, ensure_ascii=False, indent=1))


def draw_figures(image, detected_corners, candidates, chosen, axis, tvec, rot):
    current = image.copy()
    poly = detected_corners.astype(np.int32)
    cv2.polylines(current, [poly], True, (40, 220, 40), 2)
    for i, point in enumerate(poly):
        cv2.circle(current, tuple(point), 4, (40, 220, 40), -1)
        cv2.putText(current, str(i), tuple(point + np.array([6, -6])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 220, 40), 1)
    label(current, [
        "현재 M2 마커 (검출된 검은 사각형)",
        "초록 = 지금 면",
        "영상 각 30.8° 는 물리 장착각이 아님",
    ], (24, 24))
    cv2.imwrite(os.path.join(FIG_DIR, "current_mount.png"), current)

    proposed = image.copy()
    cv2.polylines(proposed, [poly], True, (40, 220, 40), 2)
    proposed_pts = np.array(chosen["projected"]["corners_px"], dtype=np.int32)
    cv2.polylines(proposed, [proposed_pts], True, (0, 140, 255), 2)
    # 카메라 쪽으로 움직이는 꼭짓점 방향의 화살표
    center = detected_corners.mean(axis=0)
    moved = proposed_pts[chosen["corner_index_moved_toward_camera"]].astype(float)
    direction = moved - center
    direction = direction / (np.linalg.norm(direction) + 1e-9)
    start = center.astype(int)
    end = (center + direction * 70).astype(int)
    cv2.arrowedLine(proposed, tuple(start), tuple(end), (0, 140, 255), 2, tipLength=0.25)
    label(proposed, [
        "초록 = 지금 붙어 있는 마커",
        "주황 = 제안 판이 카메라에 보이는 모양",
        "경첩은 DaBai 쪽(왼쪽), 들림은 모니터 쪽(오른쪽)",
        "오른쪽 모서리를 약 16 mm 카메라 쪽으로",
        "각도는 약 27도 (영상 30.8도가 아님)",
    ], (24, 24))
    x0 = int(np.clip(center[0] - 90, 0, image.shape[1] - 180))
    y0 = int(np.clip(center[1] - 70, 0, image.shape[0] - 140))
    crop = proposed[y0:y0 + 140, x0:x0 + 180]
    inset = cv2.resize(crop, (540, 420), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(inset, (0, 0), (539, 419), (255, 140, 0), 3)
    proposed[280:700, 700:1240] = inset
    label(proposed, ["오른쪽 확대: 왼쪽 경첩, 오른쪽을 카메라 쪽으로"], (708, 252))
    cv2.imwrite(os.path.join(FIG_DIR, "proposed_mount.png"), proposed)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    for cand, color in zip(candidates, ("green", "darkorange", "royalblue")):
        pts = np.array(cand["projected"]["corners_px"])
        loop = np.vstack([pts, pts[0]])
        axes[0].plot(loop[:, 0], loop[:, 1], "-o", color=color, label=cand["name"], markersize=3)
    axes[0].invert_yaxis()
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].set_title("projected quad")
    axes[0].legend(fontsize=8)
    names = [c["name"] for c in candidates]
    axes[1].bar(names, [c["projected"]["area_px"] for c in candidates], color=("green", "darkorange", "royalblue"))
    axes[1].set_ylabel("projected area (px^2)")
    axes[1].tick_params(axis="x", labelrotation=15)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG_DIR, "marker_projective_geometry.png"), dpi=130)
    plt.close(fig)


def label(image, lines, origin):
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil)
    font = ImageFont.truetype(FONT_PATH, 22)
    x, y = origin
    for line in lines:
        bbox = draw.textbbox((x, y), line, font=font)
        draw.rectangle([bbox[0] - 4, bbox[1] - 2, bbox[2] + 4, bbox[3] + 2], fill=(0, 0, 0))
        draw.text((x, y), line, font=font, fill=(255, 255, 255))
        y = bbox[3] + 8
    image[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


if __name__ == "__main__":
    main()
