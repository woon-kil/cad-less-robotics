"""P-008-R1GEO-V. 동결된 기하 분할을 hold-out에 그대로 적용한다."""

import hashlib
import json
import struct
import zlib
from pathlib import Path

import numpy as np

from p008b_metrics import write_png
from p008r1geo import (
    height_colors,
    overlay,
    paint_per_point,
    select_cluster,
    write_xyz_pcd,
)

ROOT = Path(__file__).resolve().parents[1]
PARAMETER_PATH = ROOT / "evidence" / "P-008" / "R1GEO" / "parameters.json"
HOLDOUT = ("capture_02", "capture_03", "capture_04", "capture_05")
STAGE_NAMES = {
    "capture_02": "holdout_b",
    "capture_03": "holdout_c",
    "capture_04": "holdout_d",
    "capture_05": "holdout_e",
}


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def point_count(path):
    header = Path(path).read_bytes().split(b"\nDATA ", 1)[0].decode("ascii")
    for line in header.splitlines():
        if line.startswith("POINTS "):
            return int(line.split()[1])
    raise SystemExit("POINTS 행이 없습니다.")


def load_frozen_parameters():
    return json.loads(PARAMETER_PATH.read_text())


def read_xyz_pcd(path):
    raw = Path(path).read_bytes()
    header, body = raw.split(b"\nDATA binary\n", 1)
    text = header.decode("ascii")
    count = int([line for line in text.splitlines() if line.startswith("POINTS ")][0].split()[1])
    return np.frombuffer(body, dtype="<f4", count=count * 3).reshape(count, 3).astype(np.float64)


def read_labels(path, count):
    raw = Path(path).read_bytes()
    stored = struct.unpack_from("<i", raw, 0)[0]
    if stored != count:
        raise SystemExit("라벨 개수가 점 개수와 다릅니다.")
    return np.frombuffer(raw, dtype="<i4", count=count, offset=4)


def read_png(path):
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


def build_metrics(capture, input_sha, input_count, plane, clusters, chosen):
    if capture not in HOLDOUT:
        raise SystemExit("hold-out 집계는 capture_02–05만 받는다.")
    selected = chosen or {}
    return {
        "capture": capture,
        "input_sha256": input_sha,
        "input_point_count": int(input_count),
        "support_plane_identifiable": bool(plane.get("identifiable")),
        "plane_coefficients": plane.get("coefficients"),
        "plane_inliers": plane.get("inlier_count"),
        "plane_inlier_ratio": plane.get("inlier_ratio"),
        "plane_residual_median": plane.get("residual_median"),
        "plane_residual_p95": plane.get("residual_p95"),
        "object_cluster_count": len(clusters),
        "selected_cluster": None if chosen is None else int(chosen["id"]),
        "selected_cluster_points": 0 if chosen is None else int(chosen["point_count"]),
        "selected_cluster_centroid": None if chosen is None else selected["centroid"],
        "selected_cluster_bounds": None if chosen is None else {
            "min": selected["aabb_min"],
            "max": selected["aabb_max"],
        },
        "selected_cluster_height_min": None if chosen is None else selected["height_min"],
        "selected_cluster_height_median": None if chosen is None else selected["height_median"],
        "selected_cluster_height_p95": None if chosen is None else selected["height_p95"],
        "selected_cluster_height_max": None if chosen is None else selected["height_max"],
        "shoe_candidate_generated": chosen is not None,
        "next_stage_input_usable": None,
        "capture_result": None,
        "failure_class": None,
    }


def judge(metrics, surface_kept, blocking_contamination):
    """시각 판단은 작업면 유지와 오염 두 가지뿐이다. 파라미터는 받지 않는다."""
    if not metrics["support_plane_identifiable"]:
        return "FAIL", "SUPPORT_PLANE_FAILURE", False
    if not metrics["shoe_candidate_generated"]:
        return "FAIL", "CANDIDATE_SELECTION_FAILURE", False
    if not surface_kept:
        return "FAIL", "SHOE_SURFACE_LOSS", False
    if blocking_contamination:
        return "FAIL", "BACKGROUND_CONTAMINATION", False
    return "PASS", None, True


def gate_from_results(results):
    passed = sum(1 for item in results if item == "PASS")
    if passed == 4:
        return "PASS_HOLDOUT_VALIDATION", "CLOSED", True, passed
    if passed == 3:
        return "PASS_WITH_LIMITATION", "CLOSED_WITH_KNOWN_LIMITATION", True, passed
    return "FAIL_HOLDOUT_GENERALIZATION", "STOPPED", False, passed


def write_manifest(evidence_root):
    evidence_root = Path(evidence_root)
    evidence_root.mkdir(parents=True, exist_ok=True)
    parameter_bytes = PARAMETER_PATH.read_bytes()
    captures = []
    for name in HOLDOUT:
        cloud = ROOT / "evidence" / "P-008B" / name / "cloud.pcd"
        captures.append({
            "capture": name,
            "cloud": str(cloud.relative_to(ROOT)),
            "sha256": sha256_file(cloud),
            "input_point_count": point_count(cloud),
            "stage_name": STAGE_NAMES[name],
        })
    manifest = {
        "dataset": "P-008B_DATASET_V2",
        "development_capture_excluded": "capture_01",
        "baseline_commit": "dd06f59b03497680dd97f34456815794d23db3ea",
        "parameter_path": "evidence/P-008/R1GEO/parameters.json",
        "parameter_sha256": hashlib.sha256(parameter_bytes).hexdigest(),
        "captures": captures,
    }
    (evidence_root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (evidence_root / "frozen_parameters.json").write_bytes(parameter_bytes)
    return manifest


def assemble_capture(evidence_root, capture):
    evidence_root = Path(evidence_root)
    directory = evidence_root / capture
    params = load_frozen_parameters()
    frozen = json.loads((evidence_root / "frozen_parameters.json").read_text())
    if frozen != params:
        raise SystemExit("동결 파라미터가 R1GEO 원본과 다릅니다.")
    manifest = json.loads((evidence_root / "manifest.json").read_text())
    recorded = next(item for item in manifest["captures"] if item["capture"] == capture)
    cloud = ROOT / "evidence" / "P-008B" / capture / "cloud.pcd"
    if sha256_file(cloud) != recorded["sha256"]:
        raise SystemExit("실행 전 SHA와 현재 클라우드가 다릅니다.")
    plane = json.loads((directory / "support_plane.json").read_text())
    clusters = json.loads((directory / "clusters.json").read_text())
    chosen = select_cluster(clusters, params) if plane.get("identifiable") else None
    objects = read_xyz_pcd(directory / "04_non_plane.pcd")
    labels = read_labels(directory / "04_non_plane_labels.bin", len(objects)) if len(objects) else np.zeros(0, dtype=np.int32)
    if chosen is not None:
        write_xyz_pcd(directory / "shoe_candidate.pcd", objects[labels == int(chosen["id"])])
    metrics = build_metrics(capture, recorded["sha256"], recorded["input_point_count"], plane, clusters, chosen)
    (directory / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    color = read_png(ROOT / "evidence" / "P-008B" / capture / "color.png")
    workspace = read_xyz_pcd(directory / "02_workspace.pcd")
    support = read_xyz_pcd(directory / "03_support_plane.pcd")
    figures = directory / "figures"
    figures.mkdir(exist_ok=True)
    write_png(figures / "01_input.png", overlay(color, workspace, params, (180, 180, 255)))
    write_png(figures / "02_support_plane.png", overlay(color, support, params, (40, 220, 80)))
    write_png(figures / "03_non_plane.png", overlay(color, objects, params, (255, 160, 40)))
    cluster_view = (color.astype(np.float32) * 0.45).astype(np.uint8)
    palette = np.array([
        [255, 80, 80], [80, 160, 255], [255, 220, 40], [180, 80, 255], [40, 220, 200], [255, 140, 200],
    ], dtype=np.uint8)
    if len(objects):
        colors = np.full((len(objects), 3), 80, dtype=np.uint8)
        known = labels >= 0
        colors[known] = palette[np.mod(labels[known], len(palette))]
        paint_per_point(cluster_view, objects, params, colors)
    write_png(figures / "04_clusters.png", cluster_view)
    candidate = read_xyz_pcd(directory / "shoe_candidate.pcd") if chosen is not None else np.zeros((0, 3))
    write_png(figures / "05_selected_candidate.png", overlay(color, candidate, params, (255, 40, 40)))
    if plane.get("identifiable") and len(objects):
        height_view = (color.astype(np.float32) * 0.45).astype(np.uint8)
        normal = np.asarray(plane["normal"], dtype=np.float64)
        paint_per_point(height_view, objects, params, height_colors(objects, normal, float(plane["coefficients"][3])))
        write_png(figures / "03_plane_relative_height.png", height_view)
    return metrics


def apply_visual(evidence_root, capture, surface_kept, blocking_contamination):
    path = Path(evidence_root) / capture / "metrics.json"
    metrics = json.loads(path.read_text())
    result, failure, usable = judge(metrics, surface_kept, blocking_contamination)
    metrics["capture_result"] = result
    metrics["failure_class"] = failure
    metrics["next_stage_input_usable"] = usable
    path.write_text(json.dumps(metrics, indent=2) + "\n")
    return metrics
