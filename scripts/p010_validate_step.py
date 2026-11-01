#!/usr/bin/env python3
"""P-010. FreeCAD/OpenCascade로 같은 STEP을 두 번 읽고 수치를 기록한다.

CAD를 고치지 않는다. Robot, ROS, MoveIt, Gemini는 사용하지 않는다.
"""

import json
import math
from pathlib import Path

import FreeCAD as App
import Part

ROOT = Path("/ws") if False else Path("/Users/woonkil/projects/cad-less-robotics")
STEP = ROOT / "assets/cad/shoe_last.step"
OUT = ROOT / "evidence/P-010"
REQUIRED_FACES = 10


def finite(value):
    return math.isfinite(float(value))


def read_shape():
    shape = Part.Shape()
    shape.read(str(STEP))
    return shape


def measure(shape):
    box = shape.BoundBox
    coordinates = [box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax]
    nan_free = all(finite(value) for value in coordinates)
    faces = []
    normal_failures = 0
    for index, face in enumerate(shape.Faces):
        u0, u1, v0, v1 = face.ParameterRange
        samples = []
        for u_ratio in (0.25, 0.5, 0.75):
            for v_ratio in (0.25, 0.5, 0.75):
                u = u0 + (u1 - u0) * u_ratio
                v = v0 + (v1 - v0) * v_ratio
                point = face.valueAt(u, v)
                normal = face.normalAt(u, v)
                length = math.sqrt(normal.x ** 2 + normal.y ** 2 + normal.z ** 2)
                point_ok = finite(point.x) and finite(point.y) and finite(point.z)
                normal_ok = finite(normal.x) and finite(normal.y) and finite(normal.z) and abs(length - 1.0) < 1e-6
                if not normal_ok:
                    normal_failures += 1
                samples.append({
                    "u": u,
                    "v": v,
                    "point_mm": [point.x, point.y, point.z],
                    "normal": [normal.x, normal.y, normal.z],
                    "normal_length": length,
                    "point_finite": point_ok,
                    "normal_unit": normal_ok,
                })
        faces.append({
            "face_index": index,
            "surface_type": face.Surface.TypeId,
            "valid": bool(face.isValid()),
            "samples": samples,
        })
    shell_closed = []
    for shell in shape.Shells:
        shell_closed.append(bool(shell.isClosed()) if hasattr(shell, "isClosed") else None)
    return {
        "shape_type": shape.ShapeType,
        "solid_count": len(shape.Solids),
        "shell_count": len(shape.Shells),
        "face_count": len(shape.Faces),
        "edge_count": len(shape.Edges),
        "vertex_count": len(shape.Vertexes),
        "shape_valid": bool(shape.isValid()),
        "shell_closed": shell_closed,
        "bounding_box_mm": {
            "unit": "mm",
            "x_extent": box.XLength,
            "y_extent": box.YLength,
            "z_extent": box.ZLength,
            "min": [box.XMin, box.YMin, box.ZMin],
            "max": [box.XMax, box.YMax, box.ZMax],
        },
        "coordinates_finite": nan_free,
        "normal_failure_count": normal_failures,
        "faces": faces,
    }


def comparable(record):
    return {
        "shape_type": record["shape_type"],
        "solid_count": record["solid_count"],
        "shell_count": record["shell_count"],
        "face_count": record["face_count"],
        "edge_count": record["edge_count"],
        "vertex_count": record["vertex_count"],
        "shape_valid": record["shape_valid"],
        "bounding_box_mm": record["bounding_box_mm"],
        "normal_failure_count": record["normal_failure_count"],
        "samples": [
            [sample["point_mm"], sample["normal"]]
            for face in record["faces"]
            for sample in face["samples"]
        ],
    }


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    first = measure(read_shape())
    second = measure(read_shape())
    same = comparable(first) == comparable(second)
    check_messages = []
    try:
        checked = read_shape().check(True)
        check_messages = [] if checked is None else [str(item) for item in checked]
    except Exception as error:
        check_messages = [f"{type(error).__name__}: {error}"]
    topology = {
        "step_read_status": "PASS",
        "reader": "FreeCAD Part.Shape.read",
        "opencascade_binding": f"FreeCAD {'.'.join(App.Version()[:3])}",
        "shape_type": first["shape_type"],
        "solid_count": first["solid_count"],
        "shell_count": first["shell_count"],
        "face_count": first["face_count"],
        "edge_count": first["edge_count"],
        "vertex_count": first["vertex_count"],
    }
    extents = first["bounding_box_mm"]
    geometry = {
        "unit": "mm",
        "unit_source": "STEP SI_UNIT(.MILLI.,.METRE.)",
        "bounding_box_mm": extents,
        "coordinates_finite": first["coordinates_finite"],
        "dimension_plausible": bool(
            first["coordinates_finite"]
            and 150.0 <= extents["x_extent"] <= 400.0
            and 40.0 <= extents["y_extent"] <= 180.0
            and 40.0 <= extents["z_extent"] <= 200.0
        ),
    }
    geometry["plausibility_screen_mm"] = {
        "x": [150.0, 400.0],
        "y": [40.0, 180.0],
        "z": [40.0, 200.0],
        "meaning": "Adult shoe-last order of magnitude. Not a manufacturing tolerance.",
    }
    surface_query = "PASS" if first["face_count"] >= REQUIRED_FACES and all(
        sample["point_finite"] for face in first["faces"] for sample in face["samples"]
    ) else "FAIL"
    normal_query = "PASS" if first["face_count"] >= REQUIRED_FACES and first["normal_failure_count"] == 0 else "FAIL"
    brep_valid = first["shape_valid"] and first["shell_closed"] == [True]
    bop_clean = check_messages == []
    validity = "VALID" if brep_valid and bop_clean else "INVALID"
    (OUT / "cad_topology.json").write_text(json.dumps(topology, indent=2) + "\n", encoding="utf-8")
    (OUT / "cad_geometry.json").write_text(json.dumps(geometry, indent=2) + "\n", encoding="utf-8")
    (OUT / "cad_validation.json").write_text(json.dumps({
        "runs": [first, second],
        "reproducible": same,
        "brep_check_messages": check_messages,
        "cad_validity": validity,
        "surface_query": surface_query,
        "surface_query_reason": None if surface_query == "PASS" else f"faces {first['face_count']} < {REQUIRED_FACES}",
        "normal_query": normal_query,
        "normal_failure_count": first["normal_failure_count"],
        "required_distinct_faces": REQUIRED_FACES,
        "role": "CAD branch input. Not measured-surface ground truth.",
    }, indent=2) + "\n", encoding="utf-8")
    mesh = []
    shape = read_shape()
    for index, face in enumerate(shape.Faces):
        points, facets = face.tessellate(1.0)
        mesh.append({
            "face_index": index,
            "points": [[point.x, point.y, point.z] for point in points],
            "facets": [list(facet) for facet in facets],
        })
    (OUT / "mesh_faces.json").write_text(json.dumps(mesh) + "\n", encoding="utf-8")
    print(json.dumps({
        "validity": validity,
        "faces": first["face_count"],
        "surface": surface_query,
        "normal": normal_query,
        "same": same,
        "plausible": geometry["dimension_plausible"],
        "check": check_messages,
    }, indent=2))


main()
