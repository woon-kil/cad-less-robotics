#!/usr/bin/env python3
"""W-002F-R3-D1. 생성 PNG와 촬영 RGB의 ChArUco 검출 원인을 가른다.

검출 파라미터는 바꾸지 않는다. solvePnP는 호출하지 않는다.
"""

import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/ws/cad-less-robotics")
PNG = ROOT / "evidence/W-002F-R2/generated/charuco_7x5_30mm_15mm_opencv454.png"
CAPTURE = ROOT / "evidence/W-002F-R3/frames/color/000.png"
OUT = ROOT / "evidence/W-002F-R3-D1"
FROZEN_SHA = "a67dc7283002096aa41380c5a2111d766dc77b9ec551844910e9f1ad662d1ba9"


def parameters():
    detector = cv2.aruco.DetectorParameters_create()
    detector.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return detector


def detect(image, dictionary, detector):
    corners, ids, rejected = cv2.aruco.detectMarkers(
        image, dictionary, parameters=detector
    )
    marker_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    rejected_count = 0 if rejected is None else len(rejected)
    return corners, marker_ids, rejected_count, rejected


def interpolate(image, board, corners, marker_ids):
    if not marker_ids:
        return [], 0
    ids = np.array(marker_ids, dtype=np.int32).reshape(-1, 1)
    count, _points, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, image, board
    )
    if count is None or int(count) <= 0 or charuco_ids is None:
        return [], 0
    found = [int(value) for value in np.asarray(charuco_ids).reshape(-1)]
    return found, len(found)


def draw(image, corners, ids, rejected):
    canvas = image.copy()
    if canvas.ndim == 2:
        canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    if rejected:
        cv2.aruco.drawDetectedMarkers(canvas, rejected, borderColor=(0, 0, 255))
    if corners:
        marker_ids = np.array(ids, dtype=np.int32).reshape(-1, 1)
        cv2.aruco.drawDetectedMarkers(canvas, corners, marker_ids)
    return canvas


def popcount(values):
    return int(np.unpackbits(np.ascontiguousarray(values, dtype=np.uint8)).sum())


def nearest_id(bits, dictionary):
    packed = np.asarray(dictionary.getByteListFromBits(bits), dtype=np.uint8).reshape(4, 4)
    candidate = packed[0]
    table = np.asarray(dictionary.bytesList, dtype=np.uint8)
    best = None
    for marker_id in range(table.shape[0]):
        for rotation in range(table.shape[1]):
            distance = popcount(np.bitwise_xor(candidate, table[marker_id, rotation].reshape(-1)))
            if best is None or distance < best[0]:
                best = (distance, marker_id, rotation)
    return {
        "nearest_id": best[1],
        "rotation": best[2],
        "hamming": best[0],
        "max_correction_bits": int(dictionary.maxCorrectionBits),
    }


def extract_bits(gray, quad, marker_size=5, border_bits=1):
    cells = marker_size + 2 * border_bits
    side = cells * 8
    destination = np.array(
        [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype=np.float32
    )
    source = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    warped = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(source, destination), (side, side))
    cell = side // cells
    bits = np.zeros((marker_size, marker_size), dtype=np.uint8)
    for row in range(marker_size):
        for col in range(marker_size):
            patch = warped[
                (row + border_bits) * cell:(row + border_bits + 1) * cell,
                (col + border_bits) * cell:(col + border_bits + 1) * cell,
            ]
            bits[row, col] = 1 if float(patch.mean()) > 127.0 else 0
    return bits


def bit_reader_self_check(dictionary):
    drawn = np.full((220, 220), 255, dtype=np.uint8)
    marker = np.zeros((140, 140), dtype=np.uint8)
    dictionary.drawMarker(0, 140, marker, 1)
    drawn[40:180, 40:180] = marker
    corners, ids, _rejected_count, _rejected = detect(drawn, dictionary, parameters())
    if ids != [0] or not corners:
        return {"verified": False, "reason": "canonical marker was not detected"}
    bits = extract_bits(drawn, corners[0])
    distance = nearest_id(bits, dictionary)
    verified = distance["nearest_id"] == 0 and distance["hamming"] == 0
    return {"verified": verified, "canonical_distance": distance}


def audit_rejected(image, rejected, dictionary):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    check = bit_reader_self_check(dictionary)
    rows = []
    canvas = image.copy()
    if canvas.ndim == 2:
        canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    for index, quad in enumerate(rejected or []):
        points = np.asarray(quad, dtype=np.float32).reshape(4, 2)
        perimeter = float(cv2.arcLength(points, True))
        _x, _y, width, height = cv2.boundingRect(points)
        center = [float(points[:, 0].mean()), float(points[:, 1].mean())]
        row = {
            "index": index,
            "polygon": points.round(2).tolist(),
            "perimeter_px": round(perimeter, 2),
            "bbox_px": [int(width), int(height)],
            "center_px": [round(center[0], 2), round(center[1], 2)],
        }
        if check["verified"]:
            bits = extract_bits(gray, points)
            distance = nearest_id(bits, dictionary)
            row["bits"] = bits.tolist()
            row.update(distance)
            color = (0, 0, 255)
            cv2.polylines(canvas, [np.round(points).astype(np.int32)], True, color, 1)
            cv2.putText(
                canvas,
                f"{distance['nearest_id']}:{distance['hamming']}",
                (int(center[0]), int(center[1])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        rows.append(row)
    return check, rows, canvas


def dictionary_identity():
    constant = int(cv2.aruco.DICT_5X5_100)
    created = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    detected = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(7, 5, 0.030, 0.015, created)
    same = np.array_equal(created.bytesList, detected.bytesList)
    board_same = np.array_equal(board.dictionary.bytesList, detected.bytesList)
    return {
        "requested_dictionary": "DICT_5X5_100",
        "actual_dictionary_constant": "cv2.aruco.DICT_5X5_100",
        "constant_value": constant,
        "bytes_list_shape": list(created.bytesList.shape),
        "create_and_detect_bytes_list_equal": bool(same and board_same),
        "dictionary_adopted_from_cross_check": False,
    }


def main():
    if cv2.__version__ != "4.5.4":
        raise RuntimeError(cv2.__version__)
    OUT.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(PNG), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError("png missing")
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(7, 5, 0.030, 0.015, dictionary)
    detector = parameters()
    corners, marker_ids, rejected_count, rejected = detect(image, dictionary, detector)
    charuco_ids, charuco_count = interpolate(image, board, corners, marker_ids)
    cv2.imwrite(str(OUT / "source_png_detection.png"), draw(image, corners, marker_ids, rejected))
    result = {
        "opencv_version": cv2.__version__,
        "png_path": str(PNG.relative_to(ROOT)),
        "frozen_sha256_before_dpi": FROZEN_SHA,
        "detected_file_note": "On-disk PNG includes the later pHYs chunk. Detection uses that file.",
        "source_png_marker_ids": marker_ids,
        "source_png_marker_count": len(marker_ids),
        "source_png_rejected_count": rejected_count,
        "source_png_charuco_corner_ids": charuco_ids,
        "source_png_charuco_corner_count": charuco_count,
        "camera_matrix_used_on_png": False,
        "detector_parameters_changed": False,
        "solve_pnp_executed": False,
    }
    if len(marker_ids) == 0:
        result["diagnosis_branch"] = "SOURCE_ARTIFACT_OR_GENERATION_CONTRACT"
        result["camera_cause"] = "NOT_APPLICABLE"
        result["test_3_executed"] = False
        result["dictionary_identity"] = dictionary_identity()
        (OUT / "result_partial.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        print("SOURCE_PNG_MARKER_COUNT 0")
        return
    capture = cv2.imread(str(CAPTURE), cv2.IMREAD_COLOR)
    if capture is None:
        raise RuntimeError("capture missing")
    capture_corners, capture_ids, capture_rejected, capture_rejected_quads = detect(
        capture, dictionary, parameters()
    )
    cv2.imwrite(
        str(OUT / "captured_rgb_detection.png"),
        draw(capture, capture_corners, capture_ids, capture_rejected_quads),
    )
    result["capture_marker_ids"] = capture_ids
    result["capture_marker_count"] = len(capture_ids)
    result["capture_rejected_count"] = capture_rejected
    result["test_3_executed"] = True
    result["dictionary_identity"] = dictionary_identity()
    if len(capture_ids) == 0:
        result["diagnosis_branch"] = "PRINT_OR_IMAGING_PATH"
        result["print_cause_established"] = False
        check, rows, canvas = audit_rejected(capture, capture_rejected_quads, dictionary)
        result["bit_reader_self_check"] = check
        result["rejected_candidates"] = rows
        cv2.imwrite(str(OUT / "rejected_candidate_audit.png"), canvas)
    else:
        result["diagnosis_branch"] = "BOTH_DETECTABLE"
    (OUT / "result_partial.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print(f"SOURCE_PNG_MARKER_COUNT {len(marker_ids)} CAPTURE {len(capture_ids)}")


if __name__ == "__main__":
    main()
