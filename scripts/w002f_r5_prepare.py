#!/usr/bin/env python3
"""W-002F-R5. 기존 Candidate C PNG를 검사하고 같은 픽셀의 실측 인쇄 PDF만 만든다.

보드를 다시 그리지 않는다. 촬영, solvePnP, 실측값 생성은 하지 않는다.
"""

import hashlib
import json
import zlib
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/ws/cad-less-robotics")
SOURCE = ROOT / "evidence/W-002F-R5/source/candidate_c.png"
OUT = ROOT / "evidence/W-002F-R5"
EXPECTED_SHA = None


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parameters():
    detector = cv2.aruco.DetectorParameters_create()
    detector.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return detector


def detect(image, dictionary):
    corners, ids, _rejected = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters())
    marker_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    return corners or [], marker_ids


def longest_axis_pair(corners):
    points = np.asarray(corners, dtype=np.float64)
    horizontal = None
    vertical = None
    for first in range(len(points)):
        for second in range(first + 1, len(points)):
            delta = points[second] - points[first]
            if abs(delta[1]) < 1e-9 and abs(delta[0]) > 1e-9:
                distance = abs(float(delta[0]))
                if horizontal is None or distance > horizontal[0]:
                    horizontal = (distance, first, second)
            if abs(delta[0]) < 1e-9 and abs(delta[1]) > 1e-9:
                distance = abs(float(delta[1]))
                if vertical is None or distance > vertical[0]:
                    vertical = (distance, first, second)
    if horizontal is None or vertical is None:
        raise RuntimeError("직교 장거리 코너를 찾지 못했다")
    return horizontal, vertical


def write_pdf(image, pdf_path):
    height, width = image.shape
    if (width, height) != (2200, 1800):
        raise RuntimeError(f"{width}x{height}")
    raw = np.ascontiguousarray(image).tobytes()
    compressed = zlib.compress(raw, 9)
    page_w = 297.0 * 72.0 / 25.4
    page_h = 210.0 * 72.0 / 25.4
    image_w = 220.0 * 72.0 / 25.4
    image_h = 180.0 * 72.0 / 25.4
    origin_x = (page_w - image_w) / 2.0
    origin_y = (page_h - image_h) / 2.0
    content = (
        f"q\n{image_w:.6f} 0 0 {image_h:.6f} {origin_x:.6f} {origin_y:.6f} cm\n/Im0 Do\nQ\n"
    ).encode("ascii")
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Count 1 /Kids [3 0 R] >>",
        (
            f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {page_w:.6f} {page_h:.6f}] "
            f"/Contents 4 0 R /Resources << /XObject << /Im0 5 0 R >> >> >>"
        ).encode("ascii"),
        b"<< /Length " + str(len(content)).encode("ascii") + b" >>\nstream\n" + content + b"endstream",
        (
            f"<< /Type /XObject /Subtype /Image /Width {width} /Height {height} "
            f"/ColorSpace /DeviceGray /BitsPerComponent 8 /Filter /FlateDecode "
            f"/Length {len(compressed)} >>\nstream\n"
        ).encode("ascii") + compressed + b"\nendstream",
    ]
    pdf = bytearray(b"%PDF-1.4\n")
    offsets = [0]
    for number, body in enumerate(objects, start=1):
        offsets.append(len(pdf))
        pdf.extend(f"{number} 0 obj\n".encode("ascii"))
        pdf.extend(body)
        pdf.extend(b"\nendobj\n")
    xref = len(pdf)
    pdf.extend(f"xref\n0 {len(objects) + 1}\n".encode("ascii"))
    pdf.extend(b"0000000000 65535 f \n")
    for offset in offsets[1:]:
        pdf.extend(f"{offset:010d} 00000 n \n".encode("ascii"))
    pdf.extend(
        f"trailer << /Size {len(objects) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode("ascii")
    )
    pdf_path.write_bytes(pdf)
    extracted = zlib.decompress(compressed)
    if extracted != raw:
        raise RuntimeError("pdf pixels diverged")


def main():
    if cv2.__version__ != "4.5.4":
        raise RuntimeError(cv2.__version__)
    if hasattr(cv2.aruco, "CharucoDetector"):
        raise RuntimeError("CharucoDetector가 있다")
    image = cv2.imread(str(SOURCE), cv2.IMREAD_GRAYSCALE)
    if image is None or image.shape != (1800, 2200):
        raise RuntimeError("candidate C shape")
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(5, 4, 0.040, 0.030, dictionary)
    if hasattr(board, "setLegacyPattern"):
        raise RuntimeError("setLegacyPattern이 있다")
    corners, marker_ids = detect(image, dictionary)
    count, _points, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners,
        np.array(marker_ids, dtype=np.int32).reshape(-1, 1),
        image,
        board,
    )
    found_corners = [] if charuco_ids is None else [int(value) for value in np.asarray(charuco_ids).reshape(-1)]
    horizontal, vertical = longest_axis_pair(board.chessboardCorners)
    (OUT / "fabrication").mkdir(parents=True, exist_ok=True)
    (OUT / "metrology").mkdir(parents=True, exist_ok=True)
    pdf_path = OUT / "fabrication/candidate_c_a4_landscape.pdf"
    write_pdf(image, pdf_path)
    payload = {
        "opencv": cv2.__version__,
        "source_sha256": sha256(SOURCE),
        "source_shape_px": [2200, 1800],
        "source_redrawn": False,
        "dictionary": "DICT_5X5_100",
        "squares": [int(board.getChessboardSize()[0]), int(board.getChessboardSize()[1])],
        "square_length_mm": round(float(board.getSquareLength()) * 1000.0, 6),
        "marker_length_mm": round(float(board.getMarkerLength()) * 1000.0, 6),
        "board_marker_count": int(np.asarray(board.ids).size),
        "board_corner_count": int(np.asarray(board.chessboardCorners).shape[0]),
        "detected_marker_count": len(marker_ids),
        "detected_marker_ids": marker_ids,
        "detected_charuco_corner_count": 0 if count is None else int(count),
        "detected_charuco_corner_ids": found_corners,
        "digital_detection": "PASS" if len(marker_ids) == 10 and int(count) == 12 else "FAIL",
        "reference_distance_1": {
            "axis": "board_x",
            "corner_ids": [horizontal[1], horizontal[2]],
            "nominal_mm": round(horizontal[0] * 1000.0, 6),
            "measured_mm": None,
        },
        "reference_distance_2": {
            "axis": "board_y",
            "corner_ids": [vertical[1], vertical[2]],
            "nominal_mm": round(vertical[0] * 1000.0, 6),
            "measured_mm": None,
        },
        "pdf_sha256": sha256(pdf_path),
        "pdf_page": "A4 landscape",
        "pdf_image_mm": [220.0, 180.0],
        "pdf_scaled_to_page": False,
        "print_executed": False,
        "physical_measurements": None,
    }
    (OUT / "metrology/nominal_geometry.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "sha": payload["source_sha256"],
        "markers": payload["detected_marker_count"],
        "corners": payload["detected_charuco_corner_count"],
        "digital": payload["digital_detection"],
        "d1": payload["reference_distance_1"],
        "d2": payload["reference_distance_2"],
    }, indent=2))


if __name__ == "__main__":
    main()
