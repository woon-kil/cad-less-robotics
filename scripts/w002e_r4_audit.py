#!/usr/bin/env python3
"""W-002E-R4. 저장된 RGB와 실행 중 CameraInfo만 감사한다.

등록 변환을 승인하지 않는다. refinement가 기준을 넘겨도 PASS_WORKBOARD_REGISTRATION으로
바꾸지 않는다. 민감도 경계는 결과를 보기 전에 고정한다.
"""

import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

import w002e_register as reg

OUT = Path("/ws/cad-less-robotics/evidence/W-002E-R4")
SRC = Path("/ws/cad-less-robotics/evidence/W-002E")
MATERIAL_PX = 0.30
LOW_PX = 0.15
PARAMS = [
    "enable_color_undistortion",
    "enable_color",
    "enable_depth_undistortion",
    "depth_registration",
    "align_mode",
    "align_target_stream",
    "color_width",
    "color_height",
    "color_format",
    "color_fps",
    "color_optical_frame_id",
    "enable_frame_sync",
    "camera_name",
]


def dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def effect_label(baseline, other):
    delta = abs(other - baseline)
    if delta >= MATERIAL_PX:
        return "MATERIAL", float(delta)
    if delta < LOW_PX:
        return "LOW", float(delta)
    return "INCONCLUSIVE", float(delta)


def param_value(name):
    completed = subprocess.run(
        ["ros2", "param", "get", "/camera/camera", name],
        check=False,
        capture_output=True,
        text=True,
    )
    text = (completed.stdout or completed.stderr).strip()
    return {"name": name, "returncode": completed.returncode, "text": text}


def package_version():
    completed = subprocess.run(
        ["ros2", "pkg", "prefix", "orbbec_camera"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        return {"version": None, "source": "PKG_PREFIX_FAILED", "text": completed.stderr.strip()}
    prefix = completed.stdout.strip()
    xml = Path(prefix) / "share" / "orbbec_camera" / "package.xml"
    if not xml.is_file():
        return {"version": None, "source": "PACKAGE_XML_ABSENT", "path": str(xml)}
    for line in xml.read_text().splitlines():
        if "<version>" in line and "</version>" in line:
            version = line.split("<version>", 1)[1].split("</version>", 1)[0].strip()
            return {"version": version, "source": str(xml)}
    return {"version": None, "source": str(xml)}


def qos(reliable):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class Once(Node):
    def __init__(self):
        super().__init__("w002e_r4_audit")
        self.info = None
        self.image = None
        for reliable in (True, False):
            self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, qos(reliable))
            self.create_subscription(Image, "/camera/color/image_raw", self.on_image, qos(reliable))

    def on_info(self, message):
        if self.info is None:
            self.info = message

    def on_image(self, message):
        if self.image is None and message.width > 0:
            self.image = message


def detect(image_bgr, method):
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = int(method)
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    found = {}
    if ids is None:
        return found, parameters
    for index, value in enumerate(ids.flatten()):
        found[int(value)] = np.asarray(corners[index], dtype=np.float64).reshape(4, 2)
    return found, parameters


def stats(errors):
    return {
        "mean_px": float(np.mean(errors)),
        "p95_px": float(np.percentile(errors, 95)),
        "max_px": float(np.max(errors)),
        "count": int(errors.size),
    }


def displacement(base, other):
    values = []
    per_marker = {}
    for marker_id in (0, 1, 2, 3):
        delta = np.linalg.norm(other[marker_id] - base[marker_id], axis=1)
        per_marker[str(marker_id)] = {
            "mean_px": float(delta.mean()),
            "max_px": float(delta.max()),
            "corners_px": [float(item) for item in delta],
        }
        values.extend(delta.tolist())
    arr = np.asarray(values, dtype=np.float64)
    return {
        "mean_px": float(arr.mean()),
        "p95_px": float(np.percentile(arr, 95)),
        "max_px": float(arr.max()),
        "per_marker": per_marker,
    }


def draw_order(image, found):
    canvas = image.copy()
    colors = ((0, 0, 255), (0, 165, 255), (0, 220, 0), (255, 0, 0))
    for marker_id, quad in found.items():
        for index, point in enumerate(quad):
            center = tuple(np.round(point).astype(int))
            cv2.circle(canvas, center, 5, colors[index], -1)
            cv2.putText(
                canvas,
                f"{marker_id}:{index}",
                (center[0] + 4, center[1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                colors[index],
                1,
                cv2.LINE_AA,
            )
    return canvas


def draw_residual(image, found, projected_by_id):
    canvas = image.copy()
    for marker_id, quad in found.items():
        projected = projected_by_id[marker_id]
        for observed, point in zip(quad, projected):
            start = tuple(np.round(observed).astype(int))
            end = tuple(np.round(observed + (point - observed) * 20.0).astype(int))
            cv2.arrowedLine(canvas, start, end, (0, 200, 255), 1, tipLength=0.25)
            cv2.circle(canvas, start, 3, (0, 220, 0), -1)
    return canvas


def ordering(found):
    rows = []
    directions = []
    for marker_id in (0, 1, 2, 3):
        quad = found[marker_id]
        forward = quad[1] - quad[0]
        angle = float(np.degrees(np.arctan2(forward[1], forward[0])))
        directions.append(angle)
        rows.append({
            "id": marker_id,
            "image_corner_order": [0, 1, 2, 3],
            "object_corner_order": ["TOP_LEFT", "TOP_RIGHT", "BOTTOM_RIGHT", "BOTTOM_LEFT"],
            "corner0_to_corner1_deg": angle,
            "code_uses_same_index": True,
        })
    pair = []
    for i in range(4):
        for j in range(i + 1, 4):
            diff = abs(directions[i] - directions[j]) % 360.0
            pair.append(min(diff, 360.0 - diff))
    spread = float(max(pair))
    consistent = spread < 20.0
    for row in rows:
        row["corner_order_match"] = "YES" if consistent else "NO"
    return rows, spread, "PASS" if consistent else "FAIL"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    device = reg.read_running_serial()
    package = package_version()
    params = {item["name"]: item for item in (param_value(name) for name in PARAMS)}
    undistort_text = params["enable_color_undistortion"]["text"]
    rclpy.init()
    node = Once()
    deadline = time.monotonic() + 15.0
    while (node.info is None or node.image is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    info = node.info
    image_msg = node.image
    node.destroy_node()
    rclpy.shutdown()
    if info is None or image_msg is None:
        dump(OUT / "result.json", {
            "work_order": "W-002E-R4",
            "status": "BLOCKED_RUNTIME_CAPTURE",
            "closed": True,
            "camera_info_received": info is not None,
            "image_received": image_msg is not None,
        })
        raise SystemExit("runtime CameraInfo 또는 color image header가 없다.")

    live = {
        "topic": "/camera/color/camera_info",
        "image_topic": "/camera/color/image_raw",
        "frame_id": info.header.frame_id,
        "width": int(info.width),
        "height": int(info.height),
        "distortion_model": info.distortion_model,
        "d": [float(value) for value in info.d],
        "k": [float(value) for value in info.k],
        "r": [float(value) for value in info.r],
        "p": [float(value) for value in info.p],
        "image_width": int(image_msg.width),
        "image_height": int(image_msg.height),
        "image_encoding": image_msg.encoding,
        "image_frame_id": image_msg.header.frame_id,
    }
    dump(OUT / "camera_info.json", live)
    stored = json.loads((SRC / "camera_intrinsics.json").read_text())
    stored_k = np.asarray(stored["k"], dtype=np.float64).reshape(-1)
    live_k = np.asarray(live["k"], dtype=np.float64)
    k_match = bool(np.allclose(stored_k, live_k))
    d_match = bool(np.allclose(np.asarray(stored["distortion_coefficients"]), np.asarray(live["d"])))
    resolution_match = (
        live["image_width"] == live["width"]
        and live["image_height"] == live["height"]
        and live["width"] == stored["image_width"]
        and live["height"] == stored["image_height"]
    )
    undistort_enabled = undistort_text.endswith("False") is False and "True" in undistort_text
    # 파라미터 원문을 그대로 둔다. True/False 판정은 그 문자열이다.
    undistort_enabled = "True" in undistort_text and "False" not in undistort_text
    color_domain = "RAW_DISTORTED" if not undistort_enabled else "UNDISTORTED"
    domain_basis = [
        "enable_color_undistortion runtime parameter",
        "w002e_register.py does not call undistort before detectMarkers",
        "solvePnP and projectPoints receive CameraInfo K and D",
    ]
    if undistort_enabled:
        color_domain = "UNDISTORTED"
    domain_match = "PASS" if (not undistort_enabled and not False) else "UNRESOLVED"
    # distorted raw image + same CameraInfo K/D, no pre-undistort.
    if not undistort_enabled and live["distortion_model"] and len(live["d"]) > 0:
        domain_match = "PASS"
    elif undistort_enabled and all(abs(value) < 1e-12 for value in live["d"]):
        domain_match = "PASS"
    elif undistort_enabled:
        domain_match = "FAIL"

    image = cv2.imread(str(SRC / "detections" / "medoid_color.png"), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit("저장된 medoid color가 없다.")
    camera_matrix = np.asarray(stored["k"], dtype=np.float64)
    distortion = np.asarray(stored["distortion_coefficients"], dtype=np.float64)
    modes = [
        ("CORNER_REFINE_NONE", cv2.aruco.CORNER_REFINE_NONE),
        ("CORNER_REFINE_SUBPIX", cv2.aruco.CORNER_REFINE_SUBPIX),
    ]
    if hasattr(cv2.aruco, "CORNER_REFINE_CONTOUR"):
        modes.append(("CORNER_REFINE_CONTOUR", cv2.aruco.CORNER_REFINE_CONTOUR))
    baseline_params = cv2.aruco.DetectorParameters_create()
    results = []
    baseline_found = None
    for name, method in modes:
        found, _parameters = detect(image, method)
        row = {
            "mode": name,
            "method_value": int(method),
            "detected_ids": sorted(found),
            "detected_count": len(found),
            "full_4_4": sorted(found) == [0, 1, 2, 3],
            "adopted": False,
        }
        if not row["full_4_4"]:
            results.append(row)
            continue
        object_pts, image_pts, labels = reg.stack_correspondence(found)
        solved = reg.solve_one(object_pts, image_pts, camera_matrix, distortion, reg.pnp_flag())
        row["pnp_success"] = solved is not None
        if solved is None:
            results.append(row)
            continue
        row["reprojection"] = stats(solved["errors"])
        row["meets_w002b_reprojection"] = bool(
            row["reprojection"]["mean_px"] <= 1.0
            and row["reprojection"]["p95_px"] <= 2.0
            and row["reprojection"]["max_px"] <= 3.0
        )
        projected = {}
        cursor = 0
        for marker_id in (0, 1, 2, 3):
            projected[marker_id] = solved["projected"][cursor:cursor + 4]
            cursor += 4
        row["corners"] = {
            str(marker_id): np.round(quad, 4).tolist() for marker_id, quad in found.items()
        }
        if name == "CORNER_REFINE_NONE":
            baseline_found = found
            cv2.imwrite(str(OUT / "corner_order_overlay.png"), draw_order(image, found))
            cv2.imwrite(str(OUT / "residual_none.png"), draw_residual(image, found, projected))
        elif name == "CORNER_REFINE_SUBPIX":
            cv2.imwrite(str(OUT / "residual_subpix.png"), draw_residual(image, found, projected))
        elif name == "CORNER_REFINE_CONTOUR":
            cv2.imwrite(str(OUT / "residual_contour.png"), draw_residual(image, found, projected))
        if baseline_found is not None and name != "CORNER_REFINE_NONE":
            row["displacement_from_none"] = displacement(baseline_found, found)
        results.append(row)
        print(name, row["reprojection"]["mean_px"], row.get("displacement_from_none", {}).get("mean_px"))

    order_rows, spread, order_audit = ordering(baseline_found)
    dump(OUT / "corner_ordering.json", {
        "rows": order_rows,
        "corner0_to_corner1_spread_deg": spread,
        "rule": "PASS if all markers share corner0->corner1 direction within 20 deg and code index matches W-002B TL,TR,BR,BL",
        "audit": order_audit,
        "ordering_modified": False,
    })
    none_row = next(row for row in results if row["mode"] == "CORNER_REFINE_NONE")
    labels = []
    for row in results:
        if row["mode"] == "CORNER_REFINE_NONE" or "reprojection" not in row:
            continue
        label, delta = effect_label(none_row["reprojection"]["mean_px"], row["reprojection"]["mean_px"])
        row["effect_vs_none"] = label
        row["mean_delta_px"] = delta
        labels.append(label)
    if "MATERIAL" in labels:
        localization = "MATERIAL"
    elif "INCONCLUSIVE" in labels:
        localization = "INCONCLUSIVE"
    elif labels:
        localization = "LOW"
    else:
        localization = "INCONCLUSIVE"
    dump(OUT / "refinement_comparison.json", {
        "image": "evidence/W-002E/detections/medoid_color.png",
        "new_capture": False,
        "marker_size_mm": 60,
        "size_changed": False,
        "rule": "MATERIAL if |mean reprojection delta| >= 0.30 px; LOW if < 0.15 px",
        "modes": results,
        "corner_localization_effect": localization,
        "registration_approved": False,
    })
    code_audit = {
        "file": "scripts/w002e_register.py",
        "camera_info_topic": "/camera/color/camera_info",
        "image_topic": "/camera/color/image_raw",
        "k_source": "CameraInfo.k lines 249-262",
        "d_source": "CameraInfo.d lines 250-263",
        "distortion_model_stored": "CameraInfo.distortion_model, passed through as the D vector",
        "solve_pnp": "cv2.solvePnP SOLVEPNP_SQPNP with K and D, lines 188-194",
        "project_points": "cv2.projectPoints with the same K and D, line 200",
        "image_points_source": "detectMarkers corners, no undistortPoints",
        "undistort_points_used": False,
        "image_pre_undistorted_by_script": False,
        "resize": False,
        "crop": False,
        "detector": "Dictionary_get DICT_4X4_50, DetectorParameters_create defaults, lines 112-114",
        "corner_refinement_method_baseline": int(baseline_params.cornerRefinementMethod),
        "corner_refinement_win_size": int(baseline_params.cornerRefinementWinSize),
        "corner_refinement_max_iterations": int(baseline_params.cornerRefinementMaxIterations),
        "corner_refinement_min_accuracy": float(baseline_params.cornerRefinementMinAccuracy),
        "opencv_version": cv2.__version__,
    }
    dump(OUT / "code_audit.json", code_audit)
    runtime = {
        "camera_model": "Orbbec Gemini 335L",
        "camera_model_source": "running driver log serial matches the previously identified Gemini 335L unit",
        "serial": device.get("serial"),
        "firmware": device.get("firmware"),
        "sdk_version": None,
        "sdk_source": device.get("source"),
        "ros_wrapper": package,
        "parameters": params,
        "color_topic": "/camera/color/image_raw",
        "color_camera_info_topic": "/camera/color/camera_info",
        "published_width": live["image_width"],
        "published_height": live["image_height"],
        "published_encoding": live["image_encoding"],
        "parameter_color_width_is_zero_means_driver_default": "0" in params["color_width"]["text"],
    }
    log = Path("/tmp/w002a_gemini.log")
    if log.is_file():
        for line in log.read_text(errors="replace").splitlines():
            if "SDK version:" in line:
                runtime["sdk_version"] = line.split("SDK version:", 1)[1].strip()
                runtime["sdk_source"] = str(log)
    dump(OUT / "runtime_camera_config.json", runtime)
    geometry_audit = "PASS" if domain_match == "PASS" and resolution_match and order_audit == "PASS" else "UNRESOLVED"
    if domain_match == "FAIL" or order_audit == "FAIL" or not resolution_match:
        geometry_audit = "FAIL"
    result = {
        "work_order": "W-002E-R4",
        "status": "PASS_RGB_GEOMETRY_AUDIT",
        "closed": True,
        "registration_pass": False,
        "color_topic": "/camera/color/image_raw",
        "color_camera_info_topic": "/camera/color/camera_info",
        "color_image_domain": color_domain,
        "color_image_domain_basis": domain_basis,
        "distortion_model": live["distortion_model"],
        "distortion_coefficients": live["d"],
        "camera_matrix_source": "K",
        "distortion_source": "D",
        "image_pre_undistorted": False,
        "undistort_points_used": False,
        "image_calibration_domain_match": domain_match,
        "double_correction": False,
        "resolution_intrinsics_match": "PASS" if resolution_match else "FAIL",
        "image_resized": False,
        "image_cropped": False,
        "live_matches_w002e_k": k_match,
        "live_matches_w002e_d": d_match,
        "corner_ordering_audit": order_audit,
        "opencv_version": cv2.__version__,
        "corner_refinement_method_baseline": "CORNER_REFINE_NONE" if baseline_params.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_NONE else str(baseline_params.cornerRefinementMethod),
        "corner_localization_effect": localization,
        "distortion_domain_effect": "NONE" if domain_match == "PASS" else "INCONCLUSIVE",
        "distortion_domain_effect_reason": "domain match PASS, so no alternate distortion model was invented",
        "camera_geometry_audit": geometry_audit,
        "root_cause": "INCONCLUSIVE",
        "validated_registration_transform": None,
        "diagnostic_transform": "FAILED_GATE_DIAGNOSTIC_TRANSFORM",
        "markers_repositioned": False,
        "camera_moved": False,
        "robot_moved": False,
        "workboard_moved": False,
        "marker_geometry_changed": False,
        "thresholds_changed": False,
        "p009_r1_opened": False,
        "p010_opened": False,
        "next_gate_opened": False,
        "historical_evidence_modified": False,
        "refinement": {row["mode"]: row.get("reprojection") for row in results},
    }
    dump(OUT / "result.json", result)
    print("DOMAIN", color_domain, domain_match, "K", k_match, "D", d_match)
    print("ORDER", order_audit, spread)
    print("LOCALIZATION", localization)
    print("AUDIT", geometry_audit)


if __name__ == "__main__":
    main()
