#!/usr/bin/env python3
"""W-002F-R3-D2. 저장된 R3 RGB에서 마커 화소와 칸 밝기만 측정한다.

새 촬영, 검출 파라미터 변경, solvePnP는 하지 않는다.
"""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/ws/cad-less-robotics")
PNG = ROOT / "evidence/W-002F-R2/generated/charuco_7x5_30mm_15mm_opencv454.png"
FRAME = ROOT / "evidence/W-002F-R3/frames/color/000.png"
OUT = ROOT / "evidence/W-002F-R3-D2"
CANONICAL_SHA = "dd37b453f71ccb86be32d506fad85bb1673c4240b3004889da6aa056f35d29d6"
HISTORICAL_SHA = "a67dc7283002096aa41380c5a2111d766dc77b9ec551844910e9f1ad662d1ba9"
CELL_PX = 16
GRID = 7
MARKER_BITS = 5
BORDER_BITS = 1
DIAGNOSTIC_THRESHOLD = 127


def parameters():
    detector = cv2.aruco.DetectorParameters_create()
    detector.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return detector


def detect(image, dictionary):
    corners, ids, rejected = cv2.aruco.detectMarkers(
        image, dictionary, parameters=parameters()
    )
    marker_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    return corners, marker_ids, rejected or []


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentile(values, fraction):
    ordered = np.sort(np.asarray(values, dtype=np.float64))
    if len(ordered) == 0:
        return None
    if len(ordered) == 1:
        return float(ordered[0])
    position = (len(ordered) - 1) * fraction
    low = int(np.floor(position))
    high = int(np.ceil(position))
    weight = position - low
    return float(ordered[low] * (1.0 - weight) + ordered[high] * weight)


def pearson(xs, ys):
    x = np.asarray(xs, dtype=np.float64)
    y = np.asarray(ys, dtype=np.float64)
    if len(x) < 3 or float(np.std(x)) < 1e-9 or float(np.std(y)) < 1e-9:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def edge_lengths(points):
    lengths = []
    for index in range(4):
        delta = points[(index + 1) % 4] - points[index]
        lengths.append(float(np.linalg.norm(delta)))
    return lengths


def warp_marker(gray, points):
    side = GRID * CELL_PX
    destination = np.array(
        [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype=np.float32
    )
    matrix = cv2.getPerspectiveTransform(points.astype(np.float32), destination)
    return cv2.warpPerspective(gray, matrix, (side, side), flags=cv2.INTER_LINEAR)


def cell_means(normalized):
    means = np.zeros((GRID, GRID), dtype=np.float64)
    for row in range(GRID):
        for col in range(GRID):
            patch = normalized[
                row * CELL_PX:(row + 1) * CELL_PX,
                col * CELL_PX:(col + 1) * CELL_PX,
            ]
            means[row, col] = float(patch.mean())
    payload = means[BORDER_BITS:BORDER_BITS + MARKER_BITS, BORDER_BITS:BORDER_BITS + MARKER_BITS]
    border = means.copy()
    border[BORDER_BITS:BORDER_BITS + MARKER_BITS, BORDER_BITS:BORDER_BITS + MARKER_BITS] = np.nan
    return means, payload, float(np.nanmean(border))


def observed_bits(payload):
    return (payload > DIAGNOSTIC_THRESHOLD).astype(np.uint8)


def popcount(values):
    return int(np.unpackbits(np.ascontiguousarray(values, dtype=np.uint8)).sum())


def nearest_code(bits, dictionary):
    packed = np.asarray(dictionary.getByteListFromBits(bits), dtype=np.uint8).reshape(4, 4)[0]
    table = np.asarray(dictionary.bytesList, dtype=np.uint8)
    best = None
    for marker_id in range(table.shape[0]):
        for rotation in range(table.shape[1]):
            distance = popcount(np.bitwise_xor(packed, table[marker_id, rotation].reshape(-1)))
            if best is None or distance < best[0]:
                best = (distance, marker_id, rotation)
    return {"nearest_id": best[1], "rotation": best[2], "hamming": best[0]}


def expected_bits(dictionary, marker_id):
    row = np.asarray(dictionary.bytesList[marker_id, 0], dtype=np.uint8).reshape(1, -1)
    return np.asarray(dictionary.getBitsFromByteList(row, MARKER_BITS), dtype=np.uint8)


def disagreement(observed, expected):
    best = None
    current = observed.copy()
    for _rotation in range(4):
        distance = int(np.count_nonzero(current != expected))
        if best is None or distance < best:
            best = distance
        current = np.rot90(current, k=-1)
    return best


def laplacian_variance(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def raw_crop(image, points):
    x, y, width, height = cv2.boundingRect(points.astype(np.float32))
    pad = 4
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(image.shape[1], x + width + pad)
    y1 = min(image.shape[0], y + height + pad)
    return image[y0:y1, x0:x1].copy()


def montage(tiles, columns=7):
    if not tiles:
        return np.zeros((32, 32, 3), dtype=np.uint8)
    sample = tiles[0]
    if sample.ndim == 2:
        tiles = [cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR) for tile in tiles]
    thumb = 80
    resized = [cv2.resize(tile, (thumb, thumb), interpolation=cv2.INTER_NEAREST) for tile in tiles]
    rows = []
    for start in range(0, len(resized), columns):
        group = resized[start:start + columns]
        if len(group) < columns:
            group = group + [np.zeros_like(group[0]) for _ in range(columns - len(group))]
        rows.append(np.hstack(group))
    return np.vstack(rows)


def main():
    if cv2.__version__ != "4.5.4":
        raise RuntimeError(cv2.__version__)
    if sha256(PNG) != CANONICAL_SHA:
        raise RuntimeError("canonical png hash mismatch")
    for folder in ("metrics", "figures", "figures/candidates"):
        (OUT / folder).mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    image = cv2.imread(str(FRAME), cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _corners, _ids, rejected = detect(image, dictionary)
    center = np.array([image.shape[1] / 2.0, image.shape[0] / 2.0], dtype=np.float64)
    records = []
    normalized_tiles = []
    payload_tiles = []
    for index, quad in enumerate(rejected):
        points = np.asarray(quad, dtype=np.float64).reshape(4, 2)
        lengths = edge_lengths(points)
        centroid = points.mean(axis=0)
        normalized = warp_marker(gray, points)
        means, payload, border_mean = cell_means(normalized)
        bits = observed_bits(payload)
        nearest = nearest_code(bits, dictionary)
        black = payload[bits == 0]
        white = payload[bits == 1]
        black_mean = None if black.size == 0 else float(black.mean())
        white_mean = None if white.size == 0 else float(white.mean())
        separation = None
        if black_mean is not None and white_mean is not None:
            separation = white_mean - black_mean
        crop = raw_crop(image, points)
        crop_gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        binary = (normalized > DIAGNOSTIC_THRESHOLD).astype(np.uint8) * 255
        stem = f"{index:02d}"
        cv2.imwrite(str(OUT / "figures/candidates" / f"{stem}_raw_crop.png"), crop)
        cv2.imwrite(str(OUT / "figures/candidates" / f"{stem}_perspective_normalized_crop.png"), normalized)
        cv2.imwrite(str(OUT / "figures/candidates" / f"{stem}_grayscale_crop.png"), normalized)
        cv2.imwrite(str(OUT / "figures/candidates" / f"{stem}_binary_diagnostic_crop.png"), binary)
        normalized_tiles.append(normalized)
        view = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
        cv2.putText(view, str(index), (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        payload_tiles.append(view)
        records.append({
            "candidate_index": index,
            "center_px": [round(float(centroid[0]), 2), round(float(centroid[1]), 2)],
            "edge_01_px": round(lengths[0], 3),
            "edge_12_px": round(lengths[1], 3),
            "edge_23_px": round(lengths[2], 3),
            "edge_30_px": round(lengths[3], 3),
            "mean_edge_px": round(float(np.mean(lengths)), 3),
            "min_edge_px": round(float(np.min(lengths)), 3),
            "max_edge_px": round(float(np.max(lengths)), 3),
            "polygon_area_px2": round(float(cv2.contourArea(points.astype(np.float32))), 3),
            "image_center_distance_px": round(float(np.linalg.norm(centroid - center)), 3),
            "border_cell_mean": round(border_mean, 3),
            "payload_cell_means": np.round(payload, 3).tolist(),
            "capture_observed_bits": bits.tolist(),
            "source_expected_bits": "NOT_COMPARED",
            "bit_disagreement_count": "NOT_COMPARED",
            "nearest_dictionary_id": nearest["nearest_id"],
            "nearest_dictionary_hamming": nearest["hamming"],
            "correspondence_to_source_marker": "NOT_ESTABLISHED",
            "black_cell_intensity_mean": None if black_mean is None else round(black_mean, 3),
            "white_cell_intensity_mean": None if white_mean is None else round(white_mean, 3),
            "black_white_separation": None if separation is None else round(separation, 3),
            "local_contrast": None if separation is None or white_mean in (None, 0) else round(
                separation / (white_mean + black_mean), 4
            ),
            "laplacian_variance_raw_crop": round(laplacian_variance(crop_gray), 3),
            "diagnostic_threshold": DIAGNOSTIC_THRESHOLD,
            "bit_reader_is_opencv_identify": False,
        })
    mean_edges = [item["mean_edge_px"] for item in records]
    separations = [item["black_white_separation"] for item in records if item["black_white_separation"] is not None]
    black_values = [item["black_cell_intensity_mean"] for item in records if item["black_cell_intensity_mean"] is not None]
    white_values = [item["white_cell_intensity_mean"] for item in records if item["white_cell_intensity_mean"] is not None]
    sharp = [item["laplacian_variance_raw_crop"] for item in records]
    hamming = [item["nearest_dictionary_hamming"] for item in records]
    distances = [item["image_center_distance_px"] for item in records]
    geometry = {
        "candidate_count": len(records),
        "edge_statistic": "per-candidate mean of four polygon edges",
        "capture_marker_edge_mean_px": None if not mean_edges else round(float(np.mean(mean_edges)), 3),
        "capture_marker_edge_min_px": None if not mean_edges else round(float(np.min(mean_edges)), 3),
        "capture_marker_edge_max_px": None if not mean_edges else round(float(np.max(mean_edges)), 3),
        "capture_marker_edge_p05_px": None if not mean_edges else round(percentile(mean_edges, 0.05), 3),
        "capture_marker_edge_p50_px": None if not mean_edges else round(percentile(mean_edges, 0.50), 3),
        "capture_marker_edge_p95_px": None if not mean_edges else round(percentile(mean_edges, 0.95), 3),
        "candidates": records,
        "pass_threshold_created": False,
    }
    quality = {
        "black_cell_intensity_mean": None if not black_values else round(float(np.mean(black_values)), 3),
        "white_cell_intensity_mean": None if not white_values else round(float(np.mean(white_values)), 3),
        "black_white_separation": None if not separations else round(float(np.mean(separations)), 3),
        "laplacian_variance_mean": None if not sharp else round(float(np.mean(sharp)), 3),
        "nearest_dictionary_hamming_median": None if not hamming else float(np.median(hamming)),
        "nearest_dictionary_hamming_min": None if not hamming else int(np.min(hamming)),
        "nearest_dictionary_hamming_max": None if not hamming else int(np.max(hamming)),
        "true_marker_bit_disagreement": "NOT_COMPARED",
        "reason_not_compared": "Accepted IDs are absent, so a capture candidate is not tied to one source marker.",
    }
    associations = {
        "method": "Pearson correlation. Not a cause.",
        "pixel_size_vs_nearest_hamming": pearson(mean_edges, hamming),
        "contrast_vs_nearest_hamming": pearson(separations, [item["nearest_dictionary_hamming"] for item in records if item["black_white_separation"] is not None]),
        "sharpness_vs_nearest_hamming": pearson(sharp, hamming),
        "image_center_distance_vs_nearest_hamming": pearson(distances, hamming),
    }
    (OUT / "metrics/marker_pixel_geometry.json").write_text(
        json.dumps(geometry, indent=2) + "\n", encoding="utf-8"
    )
    (OUT / "metrics/cell_payload_audit.json").write_text(
        json.dumps({
            "diagnostic_threshold": DIAGNOSTIC_THRESHOLD,
            "bit_reader_is_opencv_identify": False,
            "candidates": [
                {
                    "candidate_index": item["candidate_index"],
                    "border_cell_mean": item["border_cell_mean"],
                    "payload_cell_means": item["payload_cell_means"],
                    "capture_observed_bits": item["capture_observed_bits"],
                    "source_expected_bits": item["source_expected_bits"],
                    "bit_disagreement_count": item["bit_disagreement_count"],
                    "nearest_dictionary_id": item["nearest_dictionary_id"],
                    "nearest_dictionary_hamming": item["nearest_dictionary_hamming"],
                }
                for item in records
            ],
        }, indent=2) + "\n",
        encoding="utf-8",
    )
    (OUT / "metrics/image_quality.json").write_text(
        json.dumps({"summary": quality, "associations": associations, "candidates": records}, indent=2) + "\n",
        encoding="utf-8",
    )
    canvas = image.copy()
    for item, quad in zip(records, rejected):
        polygon = np.round(np.asarray(quad).reshape(4, 2)).astype(np.int32)
        cv2.polylines(canvas, [polygon], True, (0, 255, 255), 1)
        cv2.putText(
            canvas,
            f"{item['candidate_index']}:{item['mean_edge_px']:.0f}",
            (int(item["center_px"][0]), int(item["center_px"][1])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    cv2.imwrite(str(OUT / "figures/candidate_geometry.png"), canvas)
    cv2.imwrite(str(OUT / "figures/normalized_candidates.png"), montage(normalized_tiles))
    cv2.imwrite(str(OUT / "figures/payload_comparison.png"), montage(payload_tiles))
    plot = np.full((420, 640, 3), 255, dtype=np.uint8)
    if mean_edges:
        x0, x1 = min(mean_edges), max(mean_edges)
        span = max(x1 - x0, 1.0)
        for edge, ham, dist in zip(mean_edges, hamming, distances):
            px = int(40 + (edge - x0) / span * 560)
            py = int(360 - ham / 25.0 * 300)
            cv2.circle(plot, (px, py), 5, (40, 40, 180), -1)
            cv2.putText(plot, f"{dist:.0f}", (px + 4, py), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
    cv2.putText(plot, "edge px vs nearest hamming", (20, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    cv2.imwrite(str(OUT / "figures/spatial_diagnostics.png"), plot)
    print(json.dumps({
        "sha": sha256(FRAME),
        "png": sha256(PNG),
        "count": len(records),
        "edges": geometry["capture_marker_edge_mean_px"],
        "min": geometry["capture_marker_edge_min_px"],
        "max": geometry["capture_marker_edge_max_px"],
        "p50": geometry["capture_marker_edge_p50_px"],
        "quality": quality,
        "associations": associations,
    }, indent=2))


if __name__ == "__main__":
    main()
