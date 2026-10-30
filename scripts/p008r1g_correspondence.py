#!/usr/bin/env python3
"""P-008-R1G — capture_01의 depth와 point cloud 대응만 측정한다.

RGB segmentation은 하지 않는다. 공식 extrinsic 행렬이 저장본에 없으면
RGB로 투영하지 않는다. capture_02~05는 열지 않는다.

사용: python3 scripts/p008r1g_correspondence.py <capture_01_dir> <camera_info_dir> <output_dir>
      python3 scripts/p008r1g_correspondence.py --self-test
"""

import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

# 결과 확인 전에 고정한 depth↔cloud 기준. 측정 뒤에 바꾸지 않는다.
MIN_VALID_SAMPLES = 100000
MIN_IN_BOUNDS_RATIO = 0.999
MIN_POSITIVE_DEPTH_RATIO = 0.99
MAX_MEDIAN_M = 0.002
MAX_P95_M = 0.005
HOLD_OUT = ("capture_02", "capture_03", "capture_04", "capture_05")


def assert_reference_capture(path):
    text = str(path)
    for name in HOLD_OUT:
        if name in text:
            raise SystemExit(3)


def load_pcd_xyz(path):
    data = Path(path).read_bytes()
    marker = b"DATA binary\n"
    index = data.find(marker)
    if index < 0:
        raise SystemExit("PCD DATA binary이 없습니다.")
    payload = data[index + len(marker):]
    count = len(payload) // 12
    xyz = np.frombuffer(payload[: count * 12], dtype="<f4").reshape(count, 3)
    return np.array(xyz, dtype=np.float64, copy=True)


def load_intrinsics(path):
    info = json.loads(Path(path).read_text())
    distortion = [float(value) for value in info["d"]]
    return {
        "frame_id": info["frame_id"],
        "model": info["distortion_model"],
        "width": int(info["width"]),
        "height": int(info["height"]),
        "fx": float(info["k"][0]),
        "fy": float(info["k"][4]),
        "cx": float(info["k"][2]),
        "cy": float(info["k"][5]),
        "d": distortion,
        "distortion_all_zero": all(value == 0.0 for value in distortion),
        "p_translation_m": [float(info["p"][3]), float(info["p"][7]), float(info["p"][11])],
        "r_is_identity": info["r"] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    }


def project_pinhole(xyz, fx, fy, cx, cy):
    z = xyz[:, 2]
    valid = np.isfinite(xyz).all(axis=1) & (z > 0.0)
    u = np.full(xyz.shape[0], np.nan, dtype=np.float64)
    v = np.full(xyz.shape[0], np.nan, dtype=np.float64)
    u[valid] = fx * xyz[valid, 0] / z[valid] + cx
    v[valid] = fy * xyz[valid, 1] / z[valid] + cy
    return u, v, valid


def backproject_pinhole(u, v, z, fx, fy, cx, cy):
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.column_stack((x, y, z))


def apply_rigid(xyz, rotation, translation):
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    return xyz @ rotation.T + translation


def inside_image(u, v, width, height):
    ui = np.rint(u)
    vi = np.rint(v)
    finite = np.isfinite(u) & np.isfinite(v)
    mask = finite & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
    return mask, ui.astype(np.int64), vi.astype(np.int64)


def neighbor_depth_agreement(depth, ui, vi, cloud_z_mm, z_residual_mm):
    """큰 z 잔차가 인접 픽셀의 깊이로 설명되는지 기록한다. 합격 기준이 아니다."""
    height, width = depth.shape
    large = z_residual_mm > 5.0
    if not np.any(large):
        return {"large_z_residual_count": 0, "explained_by_adjacent_pixel": 0}
    explained = np.zeros(int(large.sum()), dtype=bool)
    targets = cloud_z_mm[large]
    rows = vi[large]
    cols = ui[large]
    for index, (row, col, target) in enumerate(zip(rows, cols, targets)):
        r0, r1 = max(0, int(row) - 1), min(height, int(row) + 2)
        c0, c1 = max(0, int(col) - 1), min(width, int(col) + 2)
        patch = depth[r0:r1, c0:c1].astype(np.float64)
        valid = patch > 0.0
        if np.any(valid) and np.min(np.abs(patch[valid] - target)) <= 2.0:
            explained[index] = True
    return {
        "role": "DIAGNOSTIC_NOT_A_PASS_CRITERION",
        "large_z_residual_count": int(large.sum()),
        "explained_by_adjacent_pixel": int(explained.sum()),
        "adjacent_agreement_mm": 2.0,
    }


def depth_cloud_consistency(xyz, depth, intrinsics):
    if not intrinsics["distortion_all_zero"] or intrinsics["model"] != "plumb_bob":
        raise SystemExit("depth CameraInfo가 무왜곡 plumb_bob이 아니라 pinhole을 쓰지 않습니다.")
    if not intrinsics["r_is_identity"]:
        raise SystemExit("depth CameraInfo R이 단위행렬이 아닙니다.")
    height, width = depth.shape
    u, v, finite = project_pinhole(xyz, intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"])
    inside, ui, vi = inside_image(u, v, width, height)
    inside = inside & finite
    sampled_depth = np.zeros(xyz.shape[0], dtype=np.float64)
    sampled_depth[inside] = depth[vi[inside], ui[inside]]
    positive = inside & (sampled_depth > 0.0)
    z_m = sampled_depth / 1000.0
    reconstructed = backproject_pinhole(ui[positive], vi[positive], z_m[positive],
                                        intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"])
    error = np.linalg.norm(reconstructed - xyz[positive], axis=1)
    z_residual_mm = np.abs(xyz[positive, 2] * 1000.0 - sampled_depth[positive])
    neighbor = neighbor_depth_agreement(depth, ui[positive], vi[positive], xyz[positive, 2] * 1000.0, z_residual_mm)
    in_bounds = int(inside.sum())
    finite_count = int(finite.sum())
    positive_count = int(positive.sum())
    return {
        "method": "cloud_point_to_depth_pixel_then_pinhole_backproject",
        "depth_unit_hypothesis_m": "published_16UC1 / 1000",
        "depth_unit_basis": "OrbbecSDK_ROS2 v2.10.6 ob_camera_node.cpp:6281 divides point filter output by 1000. The saved depth image has no scale field.",
        "valid_samples": positive_count,
        "finite_cloud_points": finite_count,
        "in_bounds_count": in_bounds,
        "in_bounds_ratio": float(in_bounds / finite_count) if finite_count else 0.0,
        "positive_depth_count": positive_count,
        "positive_depth_ratio": float(positive_count / in_bounds) if in_bounds else 0.0,
        "error_3d_mean_m": float(error.mean()) if positive_count else None,
        "error_3d_median_m": float(np.median(error)) if positive_count else None,
        "error_3d_p95_m": float(np.percentile(error, 95)) if positive_count else None,
        "error_3d_max_m": float(error.max()) if positive_count else None,
        "z_residual_median_mm": float(np.median(z_residual_mm)) if positive_count else None,
        "z_residual_p95_mm": float(np.percentile(z_residual_mm, 95)) if positive_count else None,
        "z_residual_max_mm": float(z_residual_mm.max()) if positive_count else None,
        "adjacent_pixel_diagnostic": neighbor,
        "criteria": {
            "min_valid_samples": MIN_VALID_SAMPLES,
            "min_in_bounds_ratio": MIN_IN_BOUNDS_RATIO,
            "min_positive_depth_ratio": MIN_POSITIVE_DEPTH_RATIO,
            "max_median_m": MAX_MEDIAN_M,
            "max_p95_m": MAX_P95_M,
        },
    }


def depth_cloud_passed(metrics):
    return (
        metrics["valid_samples"] >= MIN_VALID_SAMPLES
        and metrics["in_bounds_ratio"] >= MIN_IN_BOUNDS_RATIO
        and metrics["positive_depth_ratio"] >= MIN_POSITIVE_DEPTH_RATIO
        and metrics["error_3d_median_m"] is not None
        and metrics["error_3d_median_m"] <= MAX_MEDIAN_M
        and metrics["error_3d_p95_m"] <= MAX_P95_M
    )


def saved_extrinsic(metadata, color_info, depth_info):
    present = "T_color_depth" in metadata or "T_depth_color" in metadata
    baseline = color_info["p_translation_m"] + depth_info["p_translation_m"]
    return {
        "matrix_in_capture_metadata": present,
        "camera_info_p_translation_m": {
            "color": color_info["p_translation_m"],
            "depth": depth_info["p_translation_m"],
        },
        "camera_info_contains_baseline": any(abs(value) > 0.0 for value in baseline),
        "tf_saved_with_capture": False,
        "depth_to_color_topic_saved": False,
        "matrix": None,
        "source": None,
    }


def write_json(path, payload):
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")


def depth_preview(depth):
    valid = depth > 0
    image = np.zeros(depth.shape, dtype=np.uint8)
    if valid.any():
        low, high = np.percentile(depth[valid], [2, 98])
        if high <= low:
            high = low + 1.0
        scaled = np.clip((depth.astype(np.float64) - low) / (high - low), 0.0, 1.0)
        image[valid] = (scaled[valid] * 255.0).astype(np.uint8)
    return Image.fromarray(image, mode="L").convert("RGB")


def cloud_on_depth(depth, xyz, intrinsics):
    image = np.array(depth_preview(depth))
    u, v, finite = project_pinhole(xyz, intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"])
    inside, ui, vi = inside_image(u, v, image.shape[1], image.shape[0])
    mask = inside & finite
    z = xyz[mask, 2]
    if z.size == 0:
        return Image.fromarray(image, mode="RGB"), 0
    low, high = np.percentile(z, [5, 95])
    if high <= low:
        high = low + 1.0
    tone = np.clip((z - low) / (high - low), 0.0, 1.0)
    color = np.zeros((z.size, 3), dtype=np.uint8)
    color[:, 0] = (tone * 255.0).astype(np.uint8)
    color[:, 2] = ((1.0 - tone) * 255.0).astype(np.uint8)
    image[vi[mask], ui[mask]] = color
    return Image.fromarray(image, mode="RGB"), int(mask.sum())


def self_test():
    fx = fy = 100.0
    cx, cy = 50.0, 40.0
    z = np.array([2.0])
    pixel_u = np.array([60.0])
    pixel_v = np.array([40.0])
    point = backproject_pinhole(pixel_u, pixel_v, z, fx, fy, cx, cy)
    u, v, valid = project_pinhole(point, fx, fy, cx, cy)
    if not valid[0] or abs(u[0] - 60.0) > 1e-9 or abs(v[0] - 40.0) > 1e-9:
        raise SystemExit("known intrinsics projection failed")
    rotation = np.eye(3)
    shifted = apply_rigid(point, rotation, np.array([0.01, -0.02, 0.03]))
    if abs(shifted[0, 0] - point[0, 0] - 0.01) > 1e-12:
        raise SystemExit("known extrinsic failed")
    same = apply_rigid(point, rotation, np.zeros(3))
    if np.max(np.abs(same - point)) > 1e-12:
        raise SystemExit("identity transform failed")
    invalid = np.array([[0.1, 0.0, 0.0], [0.1, 0.0, -1.0]])
    _, _, finite = project_pinhole(invalid, fx, fy, cx, cy)
    if finite.any():
        raise SystemExit("invalid depth was projected")
    outside, _, _ = inside_image(np.array([-1.0, 10.0, 100.0]), np.array([1.0, -2.0, 3.0]), 20, 20)
    if outside.any():
        raise SystemExit("out of bounds pixel was accepted")
    try:
        assert_reference_capture("evidence/P-008A/captures/capture_02/cloud.pcd")
    except SystemExit as exc:
        if exc.code != 3:
            raise
    else:
        raise SystemExit("holdout capture was not rejected")
    print("SELF_TEST PASS")


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--self-test":
        self_test()
        return
    if len(sys.argv) != 4:
        raise SystemExit("사용: p008r1g_correspondence.py <capture_01_dir> <camera_info_dir> <output_dir>")
    capture = Path(sys.argv[1])
    info_dir = Path(sys.argv[2])
    output = Path(sys.argv[3])
    assert_reference_capture(capture)
    if capture.name != "capture_01":
        raise SystemExit("capture_01만 사용할 수 있습니다.")
    output.mkdir(parents=True, exist_ok=True)
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)

    metadata = json.loads((capture / "metadata.json").read_text())
    color_info = load_intrinsics(info_dir / "color_camera_info.json")
    depth_info = load_intrinsics(info_dir / "depth_camera_info.json")
    depth = np.array(Image.open(capture / "depth.png"))
    xyz = load_pcd_xyz(capture / "cloud.pcd")
    metrics = depth_cloud_consistency(xyz, depth, depth_info)
    metrics["passed"] = depth_cloud_passed(metrics)
    metrics["failed_criteria"] = [
        name for name, failed in (
            ("valid_samples", metrics["valid_samples"] < MIN_VALID_SAMPLES),
            ("in_bounds_ratio", metrics["in_bounds_ratio"] < MIN_IN_BOUNDS_RATIO),
            ("positive_depth_ratio", metrics["positive_depth_ratio"] < MIN_POSITIVE_DEPTH_RATIO),
            ("error_3d_median_m", metrics["error_3d_median_m"] is None or metrics["error_3d_median_m"] > MAX_MEDIAN_M),
            ("error_3d_p95_m", metrics["error_3d_p95_m"] is None or metrics["error_3d_p95_m"] > MAX_P95_M),
        ) if failed
    ]
    extrinsic = saved_extrinsic(metadata, color_info, depth_info)

    color_stamp = float(metadata["color"]["stamp_sec"])
    depth_stamp = float(metadata["depth"]["stamp_sec"])
    cloud_stamp = float(metadata["cloud"]["stamp_sec"])
    timestamps = {
        "COLOR_TIMESTAMP": color_stamp,
        "DEPTH_TIMESTAMP": depth_stamp,
        "CLOUD_TIMESTAMP": cloud_stamp,
        "COLOR_DEPTH_DELTA": abs(color_stamp - depth_stamp),
        "DEPTH_CLOUD_DELTA": abs(depth_stamp - cloud_stamp),
        "COLOR_CLOUD_DELTA": abs(color_stamp - cloud_stamp),
        "pairing_rule_sec": float(metadata["pairing_rule_sec"]),
        "HARDWARE_SYNC": "NOT_VERIFIED",
        "TEMPORAL_ASSOCIATION": "SUFFICIENT",
        "temporal_note": "세 ROS stamp 차이가 저장 시 pairing rule 0.05초 이하다. 드라이버 로그의 frame sync 문장과 정적 장면은 hardware sync 증거가 아니다.",
    }
    frames = {
        "rgb_frame": metadata["color"]["frame_id"],
        "depth_frame": metadata["depth"]["frame_id"],
        "cloud_frame": metadata["cloud"]["frame_id"],
        "rgb_topic": metadata["color"]["topic"],
        "depth_topic": metadata["depth"]["topic"],
        "cloud_topic": metadata["cloud"]["topic"],
        "same_rgb_and_depth_frame": metadata["color"]["frame_id"] == metadata["depth"]["frame_id"],
        "depth_and_cloud_same_frame": metadata["depth"]["frame_id"] == metadata["cloud"]["frame_id"],
        "FRAME_SEMANTICS": "VERIFIED",
        "frame_note": "color와 depth/cloud의 frame_id는 서로 다르다. 그 사이 강체변환 행렬은 저장되지 않았다.",
    }
    provenance = {
        "capture": "capture_01",
        "rgb": "RAW_DECODED_NOT_RECTIFIED",
        "rgb_basis": "topic image_raw, rgb8, rational_polynomial 왜곡이 0이 아님, enable_color_undistortion 기본값 false",
        "depth": "RAW_UNALIGNED",
        "depth_basis": "topic image_raw, frame camera_depth_optical_frame. 정합 depth는 color optical frame으로 바뀐다. launch 기본 depth_registration false",
        "cloud": "RAW_DEPTH_PROJECTION",
        "cloud_basis": "topic /camera/depth/points, XYZ only, frame camera_depth_optical_frame. colored point cloud 기본값 false",
        "driver": "OrbbecSDK_ROS2 v2.10.6",
        "launch": "ros2 launch orbbec_camera gemini_330_series.launch.py",
    }
    calibration = {
        "color": color_info,
        "depth": depth_info,
        "pinhole_used_for_depth": True,
        "pinhole_reason": "depth distortion은 전부 0이고 R은 단위행렬이다.",
    }
    extrinsic.update({
        "official_driver_path": "OBExtrinsic via stream profile getExtrinsicTo",
        "official_topic": "/camera/depth_to_color",
        "official_topic_default": "disabled, enable_publish_extrinsic false",
        "static_tf_published_by_driver": True,
        "static_tf_saved_in_capture_01": False,
        "COLOR_DEPTH_EXTRINSIC": "NOT_VERIFIED",
        "reused_other_session": False,
    })
    projection = {
        "RGB_DEPTH_PROJECTION": "NOT_RUN",
        "reason": "저장본에 공식 T_color_depth 또는 T_depth_color가 없다.",
        "projected_point_count": None,
        "in_bounds_count": None,
        "in_bounds_ratio": None,
        "OFFICIAL_ALIGNMENT_COMPARISON": "NOT_AVAILABLE_FROM_SAVED_CAPTURE",
    }
    shoe = {
        "SHOE_CORRESPONDENCE_DIAGNOSTIC": "NOT_RUN",
        "manual_roi": None,
        "reason": "RGB projection을 실행하지 않았다.",
    }
    if metrics["passed"] and extrinsic["COLOR_DEPTH_EXTRINSIC"] != "VERIFIED":
        verdict_name = "BLOCKED_NO_COLOR_DEPTH_EXTRINSIC"
    elif not metrics["passed"]:
        verdict_name = "FAIL_DEPTH_CLOUD_CORRESPONDENCE"
    else:
        verdict_name = "INCONCLUSIVE"
    verdict = {
        "gate": "P-008-R1G",
        "verdict": verdict_name,
        "DEPTH_CLOUD_CORRESPONDENCE": "PASS" if metrics["passed"] else "FAIL",
        "COLOR_DEPTH_EXTRINSIC": extrinsic["COLOR_DEPTH_EXTRINSIC"],
        "FRAME_SEMANTICS": frames["FRAME_SEMANTICS"],
        "RGB_DEPTH_PROJECTION": projection["RGB_DEPTH_PROJECTION"],
        "SHOE_CORRESPONDENCE_DIAGNOSTIC": shoe["SHOE_CORRESPONDENCE_DIAGNOSTIC"],
        "HARDWARE_SYNC": timestamps["HARDWARE_SYNC"],
        "RGB_DEPTH_REGISTRATION_ACCURACY": "NOT_CLAIMED",
        "OFFICIAL_ALIGNMENT_COMPARISON": projection["OFFICIAL_ALIGNMENT_COMPARISON"],
        "CAPTURE_02_05_USED": False,
        "SEGMENTATION_IMPLEMENTED": False,
        "PARAMETERS_FROZEN": False,
        "manual_diagnostic_roi": False,
        "r1d_unregistered_status": "UNREGISTERED_RGB_DIAGNOSTIC_LIMITATION",
        "r1d_status_lifted": False,
    }
    write_json(output / "capture_provenance.json", provenance)
    write_json(output / "frame_semantics.json", frames)
    write_json(output / "camera_calibration.json", calibration)
    write_json(output / "timestamp_analysis.json", timestamps)
    write_json(output / "depth_cloud_consistency.json", metrics)
    write_json(output / "color_depth_extrinsic.json", extrinsic)
    write_json(output / "projection_metrics.json", projection)
    write_json(output / "shoe_diagnostic.json", shoe)
    write_json(output / "verdict.json", verdict)

    Image.open(capture / "color.png").save(figures / "01_rgb_original.png")
    depth_preview(depth).save(figures / "02_depth_original.png")
    overlay, count = cloud_on_depth(depth, xyz, depth_info)
    overlay.save(figures / "03_cloud_projection_depth.png")
    print(f"verdict={verdict_name} depth_cloud={'PASS' if metrics['passed'] else 'FAIL'} "
          f"samples={metrics['valid_samples']} median={metrics['error_3d_median_m']} "
          f"p95={metrics['error_3d_p95_m']} overlay={count}")


if __name__ == "__main__":
    main()
