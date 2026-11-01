#!/usr/bin/env python3
"""W-002F-R3-R2. 스마트폰 사진에서 DICT_5X5_100만 읽고 생성 PNG와 비교한다.

Gemini 재촬영, 보드 이동, solvePnP, 검출 파라미터 변경은 하지 않는다.
"""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/ws/cad-less-robotics")
PNG = ROOT / "evidence/W-002F-R2/generated/charuco_7x5_30mm_15mm_opencv454.png"
OUT = ROOT / "evidence/W-002F-R3-R2"
CANONICAL_SHA = "dd37b453f71ccb86be32d506fad85bb1673c4240b3004889da6aa056f35d29d6"
PHOTOS = ("01_oblique.jpg", "02_close.jpg", "03_overhead.jpg")
BOARD_IDS = set(range(17))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parameters():
    detector = cv2.aruco.DetectorParameters_create()
    detector.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return detector


def detect(image, dictionary):
    corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters())
    marker_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    return corners or [], marker_ids, rejected or []


def popcount(values):
    return int(np.unpackbits(np.ascontiguousarray(values, dtype=np.uint8)).sum())


def extract_bits(gray, quad, marker_size=5, border_bits=1):
    cells = marker_size + 2 * border_bits
    side = cells * 8
    destination = np.array(
        [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]], dtype=np.float32
    )
    warped = cv2.warpPerspective(
        gray, cv2.getPerspectiveTransform(np.asarray(quad, dtype=np.float32).reshape(4, 2), destination), (side, side)
    )
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


def hamming_to_id(bits, dictionary, marker_id):
    packed = np.asarray(dictionary.getByteListFromBits(bits), dtype=np.uint8).reshape(4, 4)[0]
    row = np.asarray(dictionary.bytesList[marker_id], dtype=np.uint8)
    return min(
        popcount(np.bitwise_xor(packed, row[rotation].reshape(-1)))
        for rotation in range(row.shape[0])
    )


def centers_by_id(corners, marker_ids):
    found = {}
    for quad, marker_id in zip(corners, marker_ids):
        center = np.asarray(quad, dtype=np.float64).reshape(4, 2).mean(axis=0)
        found[marker_id] = center
    return found


def layout_residual(source_centers, photo_centers):
    shared = sorted(set(source_centers) & set(photo_centers))
    if len(shared) < 4:
        return {"shared_id_count": len(shared), "median_residual_px": None, "max_residual_px": None}
    source = np.array([source_centers[marker_id] for marker_id in shared], dtype=np.float32).reshape(-1, 1, 2)
    photo = np.array([photo_centers[marker_id] for marker_id in shared], dtype=np.float32).reshape(-1, 1, 2)
    homography, _mask = cv2.findHomography(source, photo, 0)
    if homography is None:
        return {"shared_id_count": len(shared), "median_residual_px": None, "max_residual_px": None}
    projected = cv2.perspectiveTransform(source, homography).reshape(-1, 2)
    errors = np.linalg.norm(projected - photo.reshape(-1, 2), axis=1)
    return {
        "shared_id_count": len(shared),
        "median_residual_px": round(float(np.median(errors)), 3),
        "max_residual_px": round(float(np.max(errors)), 3),
        "method": "2D homography from source PNG centers to photo centers. Not solvePnP.",
    }


def main():
    if cv2.__version__ != "4.5.4":
        raise RuntimeError(cv2.__version__)
    if sha256(PNG) != CANONICAL_SHA:
        raise RuntimeError("canonical png hash mismatch")
    (OUT / "metrics").mkdir(parents=True, exist_ok=True)
    (OUT / "figures").mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    source = cv2.imread(str(PNG), cv2.IMREAD_COLOR)
    source_corners, source_ids, _rejected = detect(source, dictionary)
    source_centers = centers_by_id(source_corners, source_ids)
    photos = []
    for name in PHOTOS:
        path = OUT / "photos" / name
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, marker_ids, rejected = detect(image, dictionary)
        photo_centers = centers_by_id(corners, marker_ids)
        markers = []
        for quad, marker_id in zip(corners, marker_ids):
            bits = extract_bits(gray, quad)
            markers.append({
                "id": marker_id,
                "center_px": [round(float(photo_centers[marker_id][0]), 2), round(float(photo_centers[marker_id][1]), 2)],
                "diagnostic_hamming_to_accepted_id": hamming_to_id(bits, dictionary, marker_id),
                "id_in_source_board": marker_id in BOARD_IDS,
                "bit_reader_is_opencv_identify": False,
            })
        canvas = image.copy()
        if corners:
            cv2.aruco.drawDetectedMarkers(canvas, corners, np.array(marker_ids, dtype=np.int32).reshape(-1, 1))
        cv2.imwrite(str(OUT / "figures" / f"{path.stem}_detection.png"), canvas)
        photos.append({
            "file": f"photos/{name}",
            "sha256": sha256(path),
            "resolution": [int(image.shape[1]), int(image.shape[0])],
            "accepted_marker_ids": marker_ids,
            "accepted_marker_count": len(marker_ids),
            "rejected_candidate_count": len(rejected),
            "ids_outside_source_board": sorted(set(marker_ids) - BOARD_IDS),
            "source_ids_missing": sorted(BOARD_IDS - set(marker_ids)),
            "layout": layout_residual(source_centers, photo_centers),
            "markers": markers,
        })
        print(name, "accepted", marker_ids, "rejected", len(rejected), flush=True)
    (OUT / "metrics/phone_detection.json").write_text(json.dumps({
        "opencv": cv2.__version__,
        "dictionary": "DICT_5X5_100",
        "detector": "DetectorParameters_create with CORNER_REFINE_NONE",
        "source_png_sha256": CANONICAL_SHA,
        "source_accepted_marker_ids": source_ids,
        "source_accepted_marker_count": len(source_ids),
        "existing_aruco_dictionary_queried": False,
        "solve_pnp": False,
        "photos": photos,
    }, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
