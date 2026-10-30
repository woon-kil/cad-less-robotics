"""P-008B 측정 함수. ROS와 촬영 루프는 여기 두지 않는다."""

import hashlib
import struct
import zlib
from pathlib import Path

import numpy as np

POSITIVE_DEPTH_RATIO_MIN = 0.99
P95_MAX_M = 0.005
COLOR_IN_BOUNDS_MIN = 0.999
COLOR_MEDIAN_ABS_MAX = 15.0
PAIR_MAX_SEC = 0.05


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rotation_valid(rotation):
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        return False
    if abs(np.linalg.det(matrix) - 1.0) > 0.01:
        return False
    return np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-2)


def extrinsic_rejection(rotation, translation):
    translation = np.asarray(translation, dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        return "invalid_translation"
    if not rotation_valid(rotation):
        return "invalid_matrix"
    identity = np.allclose(rotation, np.eye(3), atol=1e-6) and np.linalg.norm(translation) < 1e-4
    if identity:
        return "identity_fallback"
    return None


def matrix_4x4(rotation, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def quaternion_xyzw(rotation):
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([x, y, z, w], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return quat


def project_pinhole(xyz, fx, fy, cx, cy):
    z = xyz[:, 2]
    valid = np.isfinite(xyz).all(axis=1) & (z > 0.0)
    u = np.full(xyz.shape[0], np.nan)
    v = np.full(xyz.shape[0], np.nan)
    u[valid] = fx * xyz[valid, 0] / z[valid] + cx
    v[valid] = fy * xyz[valid, 1] / z[valid] + cy
    return u, v, valid


def depth_cloud_metrics(xyz, depth, fx, fy, cx, cy):
    height, width = depth.shape
    u, v, finite = project_pinhole(xyz, fx, fy, cx, cy)
    ui = np.rint(u)
    vi = np.rint(v)
    inside = finite & np.isfinite(ui) & np.isfinite(vi) & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
    ui = ui.astype(np.int64)
    vi = vi.astype(np.int64)
    sampled = np.zeros(xyz.shape[0], dtype=np.float64)
    sampled[inside] = depth[vi[inside], ui[inside]]
    positive = inside & (sampled > 0.0)
    z_m = sampled[positive] / 1000.0
    reconstructed_x = (ui[positive] - cx) * z_m / fx
    reconstructed_y = (vi[positive] - cy) * z_m / fy
    reconstructed = np.column_stack((reconstructed_x, reconstructed_y, z_m))
    error = np.linalg.norm(reconstructed - xyz[positive], axis=1) if positive.any() else np.array([])
    in_bounds = int(inside.sum())
    positive_count = int(positive.sum())
    ratio = float(positive_count / in_bounds) if in_bounds else 0.0
    p95 = float(np.percentile(error, 95)) if positive_count else None
    return {
        "sample_count": positive_count,
        "in_bounds_count": in_bounds,
        "positive_depth_ratio": ratio,
        "mean_m": float(error.mean()) if positive_count else None,
        "median_m": float(np.median(error)) if positive_count else None,
        "p95_m": p95,
        "max_m": float(error.max()) if positive_count else None,
        "passed": bool(positive_count > 0 and ratio >= POSITIVE_DEPTH_RATIO_MIN and p95 is not None and p95 <= P95_MAX_M),
        "criteria": {
            "positive_depth_ratio_min": POSITIVE_DEPTH_RATIO_MIN,
            "p95_max_m": P95_MAX_M,
        },
    }


def unpack_rgb_little_endian(blob, offset, point_step, count):
    """sensor_msgs little-endian rgb float: b, g, r, pad."""
    raw = np.frombuffer(blob, dtype=np.uint8).reshape(count, point_step)
    rgb = raw[:, offset:offset + 4]
    return np.column_stack((rgb[:, 2], rgb[:, 1], rgb[:, 0])).astype(np.uint8)


def color_correspondence(xyz, rgb, image, fx, fy, cx, cy):
    height, width = image.shape[:2]
    u, v, finite = project_pinhole(xyz, fx, fy, cx, cy)
    ui = np.rint(u)
    vi = np.rint(v)
    inside = finite & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
    finite_count = int(finite.sum())
    in_bounds = int(inside.sum())
    ratio = float(in_bounds / finite_count) if finite_count else 0.0
    if not inside.any():
        median_abs = None
    else:
        pixels = image[vi[inside].astype(np.int64), ui[inside].astype(np.int64)]
        median_abs = float(np.median(np.max(np.abs(pixels.astype(np.int16) - rgb[inside].astype(np.int16)), axis=1)))
    return {
        "projected_point_count": finite_count,
        "in_bounds_count": in_bounds,
        "in_bounds_ratio": ratio,
        "median_abs_channel": median_abs,
        "passed": bool(ratio >= COLOR_IN_BOUNDS_MIN and median_abs is not None and median_abs <= COLOR_MEDIAN_ABS_MAX),
        "registration_accuracy": "NOT_CLAIMED",
        "criteria": {
            "in_bounds_ratio_min": COLOR_IN_BOUNDS_MIN,
            "median_abs_channel_max": COLOR_MEDIAN_ABS_MAX,
        },
    }


def write_png(path, image):
    height, width = image.shape[:2]
    if image.ndim == 2:
        if image.dtype == np.uint16:
            color_type = 0
            bit_depth = 16
            rows = image.astype(">u2")
        else:
            color_type = 0
            bit_depth = 8
            rows = image
    else:
        color_type = 2
        bit_depth = 8
        rows = image

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + np.ascontiguousarray(rows[row]).tobytes() for row in range(height))
    header = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    payload = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    Path(path).write_bytes(payload)


def write_xyzrgb_pcd(path, xyz, rgb_float):
    count = int(xyz.shape[0])
    header = (
        "# .PCD v0.7\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    packed = np.empty(count, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")]))
    packed["x"] = xyz[:, 0]
    packed["y"] = xyz[:, 1]
    packed["z"] = xyz[:, 2]
    packed["rgb"] = rgb_float
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(packed.tobytes())
