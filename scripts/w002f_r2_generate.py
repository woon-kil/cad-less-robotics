#!/usr/bin/env python3
"""W-002F-R2 ChArUco 인쇄 원본 생성.

OpenCV 4.5.4 CharucoBoard_create만 사용한다.
detectMarkers, interpolateCornersCharuco, solvePnP는 호출하지 않는다.
"""

import hashlib
import json
import struct
import sys
import zlib
from pathlib import Path

import cv2
import numpy as np

SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH_M = 0.030
MARKER_LENGTH_M = 0.015
PIXELS_PER_MM = 10
MARGIN_MM = 10
BORDER_BITS = 1
PATTERN_WIDTH_MM = SQUARES_X * 30
PATTERN_HEIGHT_MM = SQUARES_Y * 30
MARGIN_PX = MARGIN_MM * PIXELS_PER_MM
PATTERN_WIDTH_PX = PATTERN_WIDTH_MM * PIXELS_PER_MM
PATTERN_HEIGHT_PX = PATTERN_HEIGHT_MM * PIXELS_PER_MM
IMAGE_WIDTH_PX = PATTERN_WIDTH_PX + 2 * MARGIN_PX
IMAGE_HEIGHT_PX = PATTERN_HEIGHT_PX + 2 * MARGIN_PX
SQUARE_PX = 30 * PIXELS_PER_MM
MARKER_PX = 15 * PIXELS_PER_MM


def corner_distance_mm(board, first, second):
    corners = board.chessboardCorners
    delta = corners[second] - corners[first]
    return float(np.linalg.norm(delta) * 1000.0)


def assert_corner_contract(board):
    corners = np.asarray(board.chessboardCorners, dtype=np.float64)
    if corners.shape != (24, 3):
        raise RuntimeError(f"corner count {corners.shape}")
    expected = {
        0: (0.03, 0.03, 0.0),
        5: (0.18, 0.03, 0.0),
        6: (0.03, 0.06, 0.0),
        18: (0.03, 0.12, 0.0),
        23: (0.18, 0.12, 0.0),
    }
    for index, point in expected.items():
        if np.max(np.abs(corners[index] - np.array(point))) > 1e-5:
            raise RuntimeError(f"corner {index} is {corners[index].tolist()}")
    distance_0_5 = corner_distance_mm(board, 0, 5)
    distance_0_18 = corner_distance_mm(board, 0, 18)
    if abs(distance_0_5 - 150.0) > 1e-3 or abs(distance_0_18 - 90.0) > 1e-3:
        raise RuntimeError(f"reference distances {distance_0_5}, {distance_0_18}")
    return distance_0_5, distance_0_18


def assert_raster(image):
    if image.shape != (IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX):
        raise RuntimeError(f"image shape {image.shape}")
    if image.dtype != np.uint8:
        raise RuntimeError(f"image dtype {image.dtype}")
    margins = (
        image[:MARGIN_PX, :],
        image[-MARGIN_PX:, :],
        image[:, :MARGIN_PX],
        image[:, -MARGIN_PX:],
    )
    if any(int(region.min()) != 255 for region in margins):
        raise RuntimeError("margin is not white")
    solid = 0
    marked = 0
    for row in range(SQUARES_Y):
        for col in range(SQUARES_X):
            y0 = MARGIN_PX + row * SQUARE_PX
            x0 = MARGIN_PX + col * SQUARE_PX
            block = image[y0:y0 + SQUARE_PX, x0:x0 + SQUARE_PX]
            rim_ink = int(block[5, 5])
            neighbor = None
            if col + 1 < SQUARES_X:
                neighbor = int(image[y0 + 5, x0 + SQUARE_PX + 5])
            elif row + 1 < SQUARES_Y:
                neighbor = int(image[y0 + SQUARE_PX + 5, x0 + 5])
            if neighbor is not None and neighbor == rim_ink:
                raise RuntimeError(f"squares do not alternate at {col},{row}")
            if rim_ink == 0:
                if int(block.min()) != 0 or int(block.max()) != 0:
                    raise RuntimeError(f"black square is not solid at {col},{row}")
                solid += 1
            elif rim_ink == 255:
                inset = (SQUARE_PX - MARKER_PX) // 2
                rim = np.concatenate((
                    block[:inset - 5, :].reshape(-1),
                    block[-(inset - 5):, :].reshape(-1),
                    block[:, :inset - 5].reshape(-1),
                    block[:, -(inset - 5):].reshape(-1),
                ))
                center = block[inset:inset + MARKER_PX, inset:inset + MARKER_PX]
                if int(rim.min()) != 255 or int(center.min()) == 255:
                    raise RuntimeError(f"marker square mismatch at {col},{row}")
                marked += 1
            else:
                raise RuntimeError(f"unexpected ink {rim_ink} at {col},{row}")
    if (solid, marked) != (18, 17):
        raise RuntimeError(f"square counts solid={solid} marked={marked}")


def png_chunk(tag, data):
    crc = zlib.crc32(tag + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)


def insert_png_pixels_per_meter(png_path, pixels_per_meter):
    raw = png_path.read_bytes()
    if not raw.startswith(b"\x89PNG\r\n\x1a\n"):
        raise RuntimeError("not a png")
    chunks = []
    index = 8
    while index < len(raw):
        length = struct.unpack(">I", raw[index:index + 4])[0]
        tag = raw[index + 4:index + 8]
        data = raw[index + 8:index + 8 + length]
        chunks.append((tag, data))
        index += 12 + length
        if tag == b"IEND":
            break
    if any(tag == b"pHYs" for tag, _data in chunks):
        raise RuntimeError("png already has pHYs")
    phys = struct.pack(">IIB", pixels_per_meter, pixels_per_meter, 1)
    out = [raw[:8]]
    inserted = False
    for tag, data in chunks:
        if tag == b"IDAT" and not inserted:
            out.append(png_chunk(b"pHYs", phys))
            inserted = True
        out.append(png_chunk(tag, data))
    png_path.write_bytes(b"".join(out))


def write_a4_landscape_pdf(image, pdf_path):
    """패턴 픽셀은 유지하고 A4 가로 페이지 한가운데에 230×170 mm로 놓는다."""
    height, width = image.shape
    if (width, height) != (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX):
        raise RuntimeError(f"pdf image size {width}x{height}")
    raw = np.ascontiguousarray(image).tobytes()
    compressed = zlib.compress(raw, 9)
    page_w = 297.0 * 72.0 / 25.4
    page_h = 210.0 * 72.0 / 25.4
    image_w = 230.0 * 72.0 / 25.4
    image_h = 170.0 * 72.0 / 25.4
    origin_x = (page_w - image_w) / 2.0
    origin_y = (page_h - image_h) / 2.0
    content = (
        f"q\n{image_w:.6f} 0 0 {image_h:.6f} {origin_x:.6f} {origin_y:.6f} cm\n/Im0 Do\nQ\n"
    ).encode("ascii")
    objects = []
    objects.append(b"<< /Type /Catalog /Pages 2 0 R >>")
    objects.append(b"<< /Type /Pages /Count 1 /Kids [3 0 R] >>")
    objects.append(
        (
            f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {page_w:.6f} {page_h:.6f}] "
            f"/Contents 4 0 R /Resources << /XObject << /Im0 5 0 R >> >> >>"
        ).encode("ascii")
    )
    objects.append(b"<< /Length " + str(len(content)).encode("ascii") + b" >>\nstream\n" + content + b"endstream")
    objects.append(
        (
            f"<< /Type /XObject /Subtype /Image /Width {width} /Height {height} "
            f"/ColorSpace /DeviceGray /BitsPerComponent 8 /Filter /FlateDecode "
            f"/Length {len(compressed)} >>\nstream\n"
        ).encode("ascii") + compressed + b"\nendstream"
    )
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
        (
            f"trailer << /Size {len(objects) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n"
        ).encode("ascii")
    )
    pdf_path.write_bytes(pdf)


def main():
    if cv2.__version__ != "4.5.4":
        raise RuntimeError(f"OpenCV {cv2.__version__}")
    if hasattr(cv2.aruco, "CharucoDetector"):
        raise RuntimeError("unexpected CharucoDetector")
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        "/ws/cad-less-robotics/evidence/W-002F-R2/generated"
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard_create(
        SQUARES_X, SQUARES_Y, SQUARE_LENGTH_M, MARKER_LENGTH_M, dictionary
    )
    if hasattr(board, "setLegacyPattern"):
        raise RuntimeError("unexpected setLegacyPattern")
    distance_0_5, distance_0_18 = assert_corner_contract(board)
    image = board.draw(
        (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX),
        marginSize=MARGIN_PX,
        borderBits=BORDER_BITS,
    )
    assert_raster(image)
    png_path = out_dir / "charuco_7x5_30mm_15mm_opencv454.png"
    if not cv2.imwrite(str(png_path), image):
        raise RuntimeError("png write failed")
    sha256_before_dpi = hashlib.sha256(png_path.read_bytes()).hexdigest()
    insert_png_pixels_per_meter(png_path, PIXELS_PER_MM * 1000)
    pdf_path = out_dir / "charuco_7x5_30mm_15mm_opencv454.pdf"
    write_a4_landscape_pdf(image, pdf_path)
    sha256 = hashlib.sha256(png_path.read_bytes()).hexdigest()
    pdf_sha256 = hashlib.sha256(pdf_path.read_bytes()).hexdigest()
    manifest = {
        "opencv_version": cv2.__version__,
        "charuco_legacy_pattern": "YES",
        "legacy_pattern_meaning": "OpenCV 4.5.4 CharucoBoard_create only pattern. setLegacyPattern was not called.",
        "dictionary": "DICT_5X5_100",
        "squares_x": SQUARES_X,
        "squares_y": SQUARES_Y,
        "square_length_m": SQUARE_LENGTH_M,
        "marker_length_m": MARKER_LENGTH_M,
        "board_square_length_m": float(board.getSquareLength()),
        "board_marker_length_m": float(board.getMarkerLength()),
        "output_pixel_width": IMAGE_WIDTH_PX,
        "output_pixel_height": IMAGE_HEIGHT_PX,
        "pattern_pixel_width": PATTERN_WIDTH_PX,
        "pattern_pixel_height": PATTERN_HEIGHT_PX,
        "margin_px": MARGIN_PX,
        "margin_mm": MARGIN_MM,
        "border_bits": BORDER_BITS,
        "pixels_per_mm": PIXELS_PER_MM,
        "png_pixels_per_meter": PIXELS_PER_MM * 1000,
        "png_has_dpi_metadata": True,
        "pdf_page": "A4_LANDSCAPE_297x210_MM",
        "pdf_image_placement_mm": "230 x 170 centered, pattern pixels unchanged",
        "nominal_pattern_width_mm": PATTERN_WIDTH_MM,
        "nominal_pattern_height_mm": PATTERN_HEIGHT_MM,
        "corner_index_order": "row_major_x_then_y",
        "model_corner_0_to_5_mm": distance_0_5,
        "model_corner_0_to_18_mm": distance_0_18,
        "detect_markers_called": False,
        "interpolate_corners_charuco_called": False,
        "solve_pnp_called": False,
        "sha256_png_before_dpi": sha256_before_dpi,
        "sha256": sha256,
        "pdf_sha256": pdf_sha256,
        "file": png_path.name,
        "pdf_file": pdf_path.name,
    }
    (out_dir / "generation.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(sha256)
    print(pdf_sha256)


if __name__ == "__main__":
    main()
