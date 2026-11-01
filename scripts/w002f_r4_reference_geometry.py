#!/usr/bin/env python3
"""W-002F-R4. OpenCV 4.5.4에서 ChArUco 후보를 만들고 검출만 비교한다.

인쇄, 촬영, solvePnP, 검출 파라미터 변경은 하지 않는다.
"""

import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/ws/cad-less-robotics")
OUT = ROOT / "evidence/W-002F-R4"
FRAME = ROOT / "evidence/W-002F-R3/frames/color/000.png"
PIXELS_PER_MM = 10
MARGIN_MM = 10
BORDER_BITS = 1
BASELINE_MARKER_MM = 15.0
BASELINE_EDGE_P50_PX = 23.486
PX_PER_MM = BASELINE_EDGE_P50_PX / BASELINE_MARKER_MM
PLANNING_MARGIN_MM = 10.0
CANDIDATES = (
    {"name": "A", "squares_x": 7, "squares_y": 5, "square_m": 0.030, "marker_m": 0.015},
    {"name": "B", "squares_x": 6, "squares_y": 4, "square_m": 0.035, "marker_m": 0.025},
    {"name": "C", "squares_x": 5, "squares_y": 4, "square_m": 0.040, "marker_m": 0.030},
)


def parameters():
    detector = cv2.aruco.DetectorParameters_create()
    detector.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return detector


def detect(image, dictionary):
    corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters())
    marker_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    return corners or [], marker_ids, rejected or []


def board_geometry(board):
    size = board.getChessboardSize()
    squares = (int(size[0]), int(size[1]))
    square_m = float(board.getSquareLength())
    marker_m = float(board.getMarkerLength())
    ids = [int(value) for value in np.asarray(board.ids).reshape(-1)]
    corners = np.asarray(board.chessboardCorners, dtype=np.float64)
    return {
        "squares_x": squares[0],
        "squares_y": squares[1],
        "square_length_mm": round(square_m * 1000.0, 6),
        "marker_length_mm": round(marker_m * 1000.0, 6),
        "marker_count": len(ids),
        "marker_ids": ids,
        "charuco_corner_count": int(corners.shape[0]),
        "pattern_width_mm": round(squares[0] * square_m * 1000.0, 6),
        "pattern_height_mm": round(squares[1] * square_m * 1000.0, 6),
    }


def draw_board(board, geometry):
    square_px = int(round(geometry["square_length_mm"] * PIXELS_PER_MM))
    margin_px = MARGIN_MM * PIXELS_PER_MM
    width = geometry["squares_x"] * square_px + 2 * margin_px
    height = geometry["squares_y"] * square_px + 2 * margin_px
    return board.draw((width, height), marginSize=margin_px, borderBits=BORDER_BITS)


def interpolate(image, board, corners, marker_ids):
    if not marker_ids:
        return []
    count, _points, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners,
        np.array(marker_ids, dtype=np.int32).reshape(-1, 1),
        image,
        board,
    )
    if count is None or int(count) <= 0 or charuco_ids is None:
        return []
    return [int(value) for value in np.asarray(charuco_ids).reshape(-1)]


def main():
    if cv2.__version__ != "4.5.4":
        raise RuntimeError(cv2.__version__)
    if hasattr(cv2.aruco, "CharucoDetector"):
        raise RuntimeError("CharucoDetector가 있다")
    for folder in ("candidates", "metrics", "figures"):
        (OUT / folder).mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    records = []
    thumbs = []
    for spec in CANDIDATES:
        board = cv2.aruco.CharucoBoard_create(
            spec["squares_x"], spec["squares_y"], spec["square_m"], spec["marker_m"], dictionary
        )
        if hasattr(board, "setLegacyPattern"):
            raise RuntimeError("setLegacyPattern이 있다")
        geometry = board_geometry(board)
        image = draw_board(board, geometry)
        path = OUT / "candidates" / f"candidate_{spec['name'].lower()}.png"
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(path.name)
        corners, marker_ids, _rejected = detect(image, dictionary)
        charuco_ids = interpolate(image, board, corners, marker_ids)
        estimated = geometry["marker_length_mm"] * PX_PER_MM
        backing_w = geometry["pattern_width_mm"] + 2.0 * PLANNING_MARGIN_MM
        backing_h = geometry["pattern_height_mm"] + 2.0 * PLANNING_MARGIN_MM
        record = {
            "candidate": spec["name"],
            "opencv_geometry": geometry,
            "generated_marker_count": len(marker_ids),
            "generated_marker_ids": marker_ids,
            "generated_charuco_corner_count": len(charuco_ids),
            "generated_charuco_corner_ids": charuco_ids,
            "digital_ids_match_board": sorted(marker_ids) == sorted(geometry["marker_ids"]),
            "digital_corners_match_board": len(charuco_ids) == geometry["charuco_corner_count"],
            "estimated_marker_edge_px": round(estimated, 3),
            "estimate_is_detection_threshold": False,
            "proposed_backing_mm": [backing_w, backing_h],
            "proposed_backing_status": "PLANNING_MARGIN_ONLY",
            "planning_margin_per_side_mm": PLANNING_MARGIN_MM,
        }
        records.append(record)
        thumb = cv2.resize(image, (280, int(280 * image.shape[0] / image.shape[1])))
        thumbs.append(thumb)
        print(spec["name"], geometry["marker_count"], geometry["charuco_corner_count"], len(marker_ids), len(charuco_ids), flush=True)
    comparison = np.full((max(item.shape[0] for item in thumbs) + 30, sum(item.shape[1] for item in thumbs) + 20, 3), 255, dtype=np.uint8)
    x = 0
    for thumb, record in zip(thumbs, records):
        color = cv2.cvtColor(thumb, cv2.COLOR_GRAY2BGR) if thumb.ndim == 2 else thumb
        comparison[30:30 + color.shape[0], x:x + color.shape[1]] = color
        cv2.putText(comparison, record["candidate"], (x + 8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        x += color.shape[1] + 10
    cv2.imwrite(str(OUT / "figures/candidate_comparison.png"), comparison)
    frame = cv2.imread(str(FRAME), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("R3 frame이 없다")
    panels = []
    origin = (180, 160)
    for record in records:
        panel = frame.copy()
        width = int(round(record["opencv_geometry"]["pattern_width_mm"] * PX_PER_MM))
        height = int(round(record["opencv_geometry"]["pattern_height_mm"] * PX_PER_MM))
        p1 = (origin[0] + width, origin[1] + height)
        inside = 0 <= origin[0] and 0 <= origin[1] and p1[0] < frame.shape[1] and p1[1] < frame.shape[0]
        cv2.rectangle(panel, origin, p1, (0, 180, 255), 2)
        marker_px = int(round(record["estimated_marker_edge_px"]))
        cv2.rectangle(panel, origin, (origin[0] + marker_px, origin[1] + marker_px), (0, 0, 255), 2)
        cv2.putText(panel, record["candidate"], (24, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(panel, "SIMULATED_NOT_OBSERVED", (24, frame.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        record["simulated_pattern_px"] = [width, height]
        record["simulated_rectangle_inside_frame"] = inside
        panels.append(cv2.resize(panel, (426, 240)))
    cv2.imwrite(str(OUT / "figures/gemini_footprint_simulation.png"), np.hstack(panels))
    (OUT / "metrics/geometry_comparison.json").write_text(json.dumps({
        "opencv": cv2.__version__,
        "dictionary": "DICT_5X5_100",
        "empirical_px_per_mm": PX_PER_MM,
        "empirical_basis": "baseline marker edge p50 23.486 px at 15 mm",
        "empirical_is_not_intrinsics": True,
        "simulated_not_observed": True,
        "detector": "DetectorParameters_create with CORNER_REFINE_NONE",
        "candidates": records,
    }, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
