"""P-008-R1GEO. 평면 거리와 군집 선택. RGB는 분할에 쓰지 않는다."""

import json
import struct
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "evidence" / "P-008B" / "capture_01"
PARAMS_PATH = ROOT / "evidence" / "P-008" / "R1GEO" / "parameters.json"
EXPECTED_SHA = "2891c0e16ff602d6e7dfac867e7bfd610b4f6b8683b1d23d7c06baae6b847a1c"


def refuse(path):
    text = str(path)
    for name in ("capture_02", "capture_03", "capture_04", "capture_05"):
        if name in text:
            raise SystemExit("holdout capture는 열지 않습니다.")


def load_params(path=PARAMS_PATH):
    return json.loads(Path(path).read_text())


def finite_mask(points):
    points = np.asarray(points, dtype=np.float64)
    return np.isfinite(points).all(axis=1)


def normalize_plane(normal, offset):
    normal = np.asarray(normal, dtype=np.float64)
    length = float(np.linalg.norm(normal))
    if not np.isfinite(length) or length < 1e-12:
        raise ValueError("평면 법선 길이가 0입니다.")
    return normal / length, float(offset) / length


def signed_distance(points, normal, offset):
    points = np.asarray(points, dtype=np.float64)
    normal = np.asarray(normal, dtype=np.float64)
    return points @ normal + float(offset)


def orient_toward_camera(normal, offset, centroid, viewpoint=(0.0, 0.0, 0.0)):
    """카메라 쪽이 양이 되도록 계수를 최대 한 번 뒤집는다."""
    normal, offset = normalize_plane(normal, offset)
    toward = np.asarray(viewpoint, dtype=np.float64) - np.asarray(centroid, dtype=np.float64)
    flipped = False
    if float(np.dot(toward, normal)) < 0.0:
        normal = -normal
        offset = -offset
        flipped = True
    return normal, offset, flipped


def select_cluster(clusters, params):
    """가장 큰 유효 군집을 고른다. 없으면 None."""
    valid = []
    for cluster in clusters:
        extent = np.asarray(cluster["extent"], dtype=np.float64)
        longest = float(np.max(extent))
        if int(cluster["point_count"]) < int(params["select_min_points"]):
            continue
        if float(cluster["height_median"]) < float(params["select_min_median_height"]):
            continue
        if float(cluster["height_max"]) > float(params["max_object_height"]):
            continue
        if longest < float(params["select_min_extent"]) or longest > float(params["select_max_extent"]):
            continue
        valid.append(cluster)
    if not valid:
        return None
    valid.sort(key=lambda item: (
        -int(item["point_count"]),
        float(item["centroid"][0]),
        float(item["centroid"][1]),
        float(item["centroid"][2]),
        int(item["id"]),
    ))
    return valid[0]


def read_xyz_pcd(path):
    refuse(path)
    raw = Path(path).read_bytes()
    header, body = raw.split(b"\nDATA binary\n", 1)
    text = header.decode("ascii")
    if "FIELDS x y z" not in text or "rgb" in text.split("DATA", 1)[0]:
        raise SystemExit("xyz PCD가 아닙니다.")
    count = int([line for line in text.splitlines() if line.startswith("POINTS ")][0].split()[1])
    points = np.frombuffer(body, dtype="<f4", count=count * 3).reshape(count, 3).astype(np.float64)
    return points


def write_xyz_pcd(path, points):
    points = np.asarray(points, dtype=np.float32)
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n"
    )
    Path(path).write_bytes(header.encode("ascii") + np.ascontiguousarray(points).tobytes())


def read_labels(path, count):
    raw = Path(path).read_bytes()
    stored = struct.unpack_from("<i", raw, 0)[0]
    if stored != count:
        raise SystemExit("라벨 개수가 점 개수와 다릅니다.")
    return np.frombuffer(raw, dtype="<i4", count=count, offset=4).copy()


def read_png(path):
    refuse(path)
    data = Path(path).read_bytes()
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
    stride = width * 3
    rows = []
    offset = 0
    for _ in range(height):
        offset += 1
        rows.append(raw[offset:offset + stride])
        offset += stride
    return np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(height, width, 3).copy()


def project(points, params):
    z = points[:, 2]
    u = params["fx"] * points[:, 0] / z + params["cx"]
    v = params["fy"] * points[:, 1] / z + params["cy"]
    return u, v


def paint(image, points, params, color):
    if len(points) == 0:
        return
    u, v = project(points, params)
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    keep = (ui >= 0) & (vi >= 0) & (ui < image.shape[1]) & (vi < image.shape[0])
    image[vi[keep], ui[keep]] = np.asarray(color, dtype=np.uint8)


def overlay(color, points, params, tint):
    view = (color.astype(np.float32) * 0.45).astype(np.uint8)
    paint(view, points, params, tint)
    return view


def height_colors(points, normal, offset):
    height = signed_distance(points, normal, offset)
    span = np.clip((height - 0.0) / 0.12, 0.0, 1.0)
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (span * 255).astype(np.uint8)
    colors[:, 2] = ((1.0 - span) * 255).astype(np.uint8)
    return colors


def paint_per_point(image, points, params, colors):
    if len(points) == 0:
        return
    u, v = project(points, params)
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    keep = (ui >= 0) & (vi >= 0) & (ui < image.shape[1]) & (vi < image.shape[0])
    image[vi[keep], ui[keep]] = colors[keep]


def write_figures(out_dir, params):
    from p008b_metrics import sha256_file, write_png

    color = read_png(CAPTURE / "color.png")
    workspace = read_xyz_pcd(out_dir / "02_workspace.pcd")
    plane = read_xyz_pcd(out_dir / "03_support_plane.pcd")
    objects = read_xyz_pcd(out_dir / "04_non_plane.pcd")
    plane_info = json.loads((out_dir / "support_plane.json").read_text())
    normal = np.asarray(plane_info["normal"], dtype=np.float64)
    offset = float(plane_info["coefficients"][3])
    figures = out_dir / "figures"
    figures.mkdir(exist_ok=True)
    write_png(figures / "01_input.png", overlay(color, workspace, params, (180, 180, 255)))
    write_png(figures / "02_support_plane.png", overlay(color, plane, params, (40, 220, 80)))
    height_view = (color.astype(np.float32) * 0.45).astype(np.uint8)
    if plane_info["identifiable"] and len(objects):
        paint_per_point(height_view, objects, params, height_colors(objects, normal, offset))
    write_png(figures / "03_plane_relative_height.png", height_view)
    write_png(figures / "04_non_plane_objects.png", overlay(color, objects, params, (255, 160, 40)))
    labels = read_labels(out_dir / "04_non_plane_labels.bin", len(objects)) if len(objects) else np.zeros(0, dtype=np.int32)
    cluster_view = (color.astype(np.float32) * 0.45).astype(np.uint8)
    palette = np.array([
        [255, 80, 80],
        [80, 160, 255],
        [255, 220, 40],
        [180, 80, 255],
        [40, 220, 200],
        [255, 140, 200],
    ], dtype=np.uint8)
    if len(objects):
        colors = np.full((len(objects), 3), 80, dtype=np.uint8)
        known = labels >= 0
        colors[known] = palette[labels[known] % len(palette)]
        paint_per_point(cluster_view, objects, params, colors)
    write_png(figures / "05_clusters.png", cluster_view)
    candidate_path = out_dir / "05_shoe_candidate.pcd"
    candidate = read_xyz_pcd(candidate_path) if candidate_path.exists() else np.zeros((0, 3))
    write_png(figures / "06_selected_shoe_candidate.png", overlay(color, candidate, params, (255, 40, 40)))
    return sha256_file(CAPTURE / "cloud.pcd")


def finish(out_dir):
    from p008b_metrics import sha256_file

    out_dir = Path(out_dir)
    params = load_params()
    plane = json.loads((out_dir / "support_plane.json").read_text())
    clusters = json.loads((out_dir / "clusters.json").read_text())
    chosen = select_cluster(clusters, params) if plane.get("identifiable") else None
    objects = read_xyz_pcd(out_dir / "04_non_plane.pcd")
    labels = read_labels(out_dir / "04_non_plane_labels.bin", len(objects)) if len(objects) else np.zeros(0, dtype=np.int32)
    if chosen is not None:
        selected_points = objects[labels == int(chosen["id"])]
        write_xyz_pcd(out_dir / "05_shoe_candidate.pcd", selected_points)
    cloud_sha = sha256_file(CAPTURE / "cloud.pcd")
    if cloud_sha != EXPECTED_SHA:
        raise SystemExit("capture_01 SHA가 고정값과 다릅니다.")
    manifest = {
        "dataset": "P-008B Dataset v2",
        "capture": "capture_01",
        "cloud_sha256": cloud_sha,
        "fields": ["x", "y", "z", "rgb"],
        "rgb_used_as_feature": False,
        "holdout_accessed": False,
    }
    (out_dir / "input_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    write_figures(out_dir, params)
    result = {
        "gate": "P-008-R1GEO",
        "support_plane_identifiable": bool(plane.get("identifiable")),
        "plane_coefficients": plane.get("coefficients"),
        "plane_inliers": plane.get("inlier_count"),
        "plane_inlier_ratio": plane.get("inlier_ratio"),
        "plane_residual_median": plane.get("residual_median"),
        "plane_residual_p95": plane.get("residual_p95"),
        "plane_relative_height_used": bool(plane.get("identifiable")),
        "object_cluster_count": len(clusters),
        "selected_cluster": None if chosen is None else int(chosen["id"]),
        "selected_cluster_points": 0 if chosen is None else int(chosen["point_count"]),
        "shoe_candidate_generated": chosen is not None,
        "geometry_only": True,
        "color_feature_used": False,
        "ml_segmentation_used": False,
        "capture_02_05_used": False,
        "production_robot_command_count": 0,
        "servo_write_count": 0,
        "motion_authorized": False,
    }
    (out_dir / "measurement.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    finish(ROOT / "evidence" / "P-008" / "R1GEO")
