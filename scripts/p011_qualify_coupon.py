#!/usr/bin/env python3
"""P-011. 새 FreeCAD 세션에서 STEP을 읽고 qualification 한다.

P-012 Geometry Engine은 만들지 않는다. 곡률 질의도 하지 않는다.
"""

import hashlib
import json
import math
import os

import MeshPart
import Part

ROOT = "/Users/woonkil/projects/cad-less-robotics"
EVIDENCE = os.path.join(ROOT, "evidence", "P-011")
SAMPLES = (0.2, 0.5, 0.8)


def finite(point):
    return math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)


def face_samples(face):
    umin, umax, vmin, vmax = face.ParameterRange
    rows = []
    for u_ratio in SAMPLES:
        row = []
        for v_ratio in SAMPLES:
            u = umin + (umax - umin) * u_ratio
            v = vmin + (vmax - vmin) * v_ratio
            point = face.valueAt(u, v)
            normal = face.normalAt(u, v)
            row.append((point, normal, u, v))
        rows.append(row)
    return rows


def classify(face, samples):
    box = face.BoundBox
    center = samples[1][1][0]
    normal = samples[1][1][1]
    length = math.sqrt(normal.x ** 2 + normal.y ** 2 + normal.z ** 2)
    upward = length > 0.0 and normal.z / length > 0.5 and center.z > 18.0 and box.XMax <= 169.0
    if upward:
        return "process"
    if box.XMin >= 169.0 or (box.ZMax <= 16.0 and box.XMax > 170.0):
        return "landmark"
    if normal.z < -0.5 and center.z < 1.0:
        return "base"
    return "side"


def surface_grid(face):
    umin, umax, vmin, vmax = face.ParameterRange
    points = []
    for u_index in range(21):
        for v_index in range(21):
            u = umin + (umax - umin) * u_index / 20.0
            v = vmin + (vmax - vmin) * v_index / 20.0
            points.append(face.valueAt(u, v))
    return points


def nearest(points, x, y):
    return min(points, key=lambda point: (point.x - x) ** 2 + (point.y - y) ** 2)


def region_of(face):
    """점의 높이로 판정한다. face 이름이나 X 구간으로 이름을 붙이지 않는다."""
    points = surface_grid(face)
    center_x = sum(point.x for point in points) / len(points)
    center_y = sum(point.y for point in points) / len(points)
    interior = [
        point for point in points
        if center_x - 18.0 < point.x < center_x + 18.0 and 8.0 < point.y < 92.0
    ]
    if not interior:
        interior = points

    def opposite(point):
        return (
            nearest(points, point.x - 18.0, point.y),
            nearest(points, point.x + 18.0, point.y),
            nearest(points, point.x, point.y - 18.0),
            nearest(points, point.x, point.y + 18.0),
        )

    margin = 1.5
    high = max(interior, key=lambda point: point.z)
    low = min(interior, key=lambda point: point.z)
    peak = all(high.z > item.z + margin for item in opposite(high))
    valley = all(low.z < item.z - margin for item in opposite(low))
    saddle_point = nearest(points, center_x, center_y)
    west, east, south, north = opposite(saddle_point)
    x_valley = saddle_point.z < west.z - margin and saddle_point.z < east.z - margin
    y_peak = saddle_point.z > south.z + margin and saddle_point.z > north.z + margin
    x_peak = saddle_point.z > west.z + margin and saddle_point.z > east.z + margin
    y_valley = saddle_point.z < south.z - margin and saddle_point.z < north.z - margin
    span = max(point.z for point in points) - min(point.z for point in points)
    if (x_valley and y_peak) or (x_peak and y_valley):
        return "saddle"
    if peak:
        return "convex"
    if valley:
        return "concave"
    if span < 3.0:
        return "low_curvature"
    return "transition"


def read_shape(path):
    shape = Part.Shape()
    shape.read(path)
    return shape


def unit_from_step(path):
    text = open(path, encoding="utf-8", errors="replace").read()
    if "SI_UNIT(.MILLI.,.METRE.)" in text:
        return "mm"
    return "UNKNOWN"


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def measure(shape):
    bop_error = None
    try:
        shape.check(True)
        bop = "PASS"
    except Exception as error:
        bop = "FAIL"
        bop_error = str(error)[:2000]
    degenerate = 0
    for edge in shape.Edges:
        if edge.Length < 1e-6:
            degenerate += 1
    null_faces = 0
    surface_fail = 0
    normal_fail = 0
    surface_ok = 0
    faces = []
    regions = {}
    landmark_z_max = None
    process_z_min = None
    for index, face in enumerate(shape.Faces):
        if face.Area < 1e-8:
            null_faces += 1
        samples = face_samples(face)
        role = classify(face, samples)
        region = region_of(face) if role == "process" else None
        if region:
            regions[region] = regions.get(region, 0) + 1
        for row in samples:
            for point, normal, _u, _v in row:
                if not finite(point):
                    surface_fail += 1
                else:
                    surface_ok += 1
                length = math.sqrt(normal.x ** 2 + normal.y ** 2 + normal.z ** 2)
                if length < 1e-8 or not finite(normal):
                    normal_fail += 1
        if role == "landmark":
            landmark_z_max = face.BoundBox.ZMax if landmark_z_max is None else max(landmark_z_max, face.BoundBox.ZMax)
        if role == "process":
            process_z_min = face.BoundBox.ZMin if process_z_min is None else min(process_z_min, face.BoundBox.ZMin)
        center = samples[1][1]
        faces.append({
            "index": index,
            "role": role,
            "region": region,
            "area_mm2": face.Area,
            "center_mm": [center[0].x, center[0].y, center[0].z],
            "normal": [center[1].x, center[1].y, center[1].z],
            "responsibility": responsibility(role, region),
        })
    box = shape.BoundBox
    landmark_faces = [item for item in faces if item["role"] == "landmark"]
    return {
        "shape_type": shape.ShapeType,
        "solid_count": len(shape.Solids),
        "shell_count": len(shape.Shells),
        "closed_shell": bool(shape.Shells) and all(shell.isClosed() for shell in shape.Shells),
        "face_count": len(shape.Faces),
        "edge_count": len(shape.Edges),
        "vertex_count": len(shape.Vertexes),
        "is_valid": bool(shape.isValid()),
        "bop_check": bop,
        "bop_error": bop_error,
        "degenerate_edge_count": degenerate,
        "null_face_count": null_faces,
        "bounding_box_mm": {
            "x": box.XLength,
            "y": box.YLength,
            "z": box.ZLength,
            "xmin": box.XMin,
            "ymin": box.YMin,
            "zmin": box.ZMin,
        },
        "faces": faces,
        "regions": regions,
        "surface_query_ok": surface_ok,
        "surface_query_fail": surface_fail,
        "normal_query_fail": normal_fail,
        "landmark_face_count": len(landmark_faces),
        "landmark_z_max_mm": landmark_z_max,
        "process_z_min_mm": process_z_min,
        "landmark_outside_process_surface": (
            landmark_z_max is not None and process_z_min is not None and landmark_z_max < process_z_min - 1.0
        ),
    }


def responsibility(role, region):
    if region == "low_curvature":
        return "낮은 곡률의 공정 표면. 완만한 기준면이다."
    if region == "convex":
        return "볼록 공정 표면. 중심이 주변보다 높다."
    if region == "concave":
        return "오목 공정 표면. 중심이 주변보다 낮다."
    if region == "saddle":
        return "안장 공정 표면. X에서는 골, Y에서는 등이다."
    if region == "transition":
        return "영역 사이 전이 공정 표면이다."
    if role == "landmark":
        return "비대칭 등록 landmark. 공정 표면 아래의 옆면 원통이다."
    if role == "base":
        return "평평한 바닥이다."
    return "영역 경계를 닫는 옆면이다."


def mesh_payload(shape):
    faces = []
    probes = []
    for index, face in enumerate(shape.Faces):
        mesh = MeshPart.meshFromShape(Shape=face, LinearDeflection=0.4, AngularDeflection=0.3)
        triangles = []
        for facet in mesh.Facets:
            triangles.append([list(point) for point in facet.Points])
        samples = face_samples(face)
        role = classify(face, samples)
        faces.append({
            "index": index,
            "role": role,
            "region": region_of(face) if role == "process" else None,
            "triangles": triangles,
        })
        if role == "process":
            point, normal, _u, _v = samples[1][1]
            probes.append({
                "face": index,
                "point": [point.x, point.y, point.z],
                "normal": [normal.x, normal.y, normal.z],
            })
    return {"faces": faces, "probes": probes}


def main():
    first = measure(read_shape(os.environ["P011_STEP_A"]))
    second = measure(read_shape(os.environ["P011_STEP_B"]))
    compared = ["solid_count", "shell_count", "face_count", "edge_count", "vertex_count", "bop_check", "is_valid"]
    same = all(first[key] == second[key] for key in compared)
    box_same = first["bounding_box_mm"] == second["bounding_box_mm"]
    sample_same = [face["center_mm"] for face in first["faces"]] == [face["center_mm"] for face in second["faces"]]
    regions = first["regions"]
    within = (
        150.0 <= first["bounding_box_mm"]["x"] <= 200.0
        and 80.0 <= first["bounding_box_mm"]["y"] <= 120.0
        and 25.0 <= first["bounding_box_mm"]["z"] <= 60.0
    )
    meaningful = 10 <= first["face_count"] <= 20
    checks = {
        "step_read": first["shape_type"] == "Solid",
        "unit_mm": unit_from_step(os.environ["P011_STEP_A"]) == "mm",
        "solid_count": first["solid_count"] >= 1,
        "closed_shell": first["closed_shell"],
        "cad_validity": first["is_valid"] and first["bop_check"] == "PASS" and first["degenerate_edge_count"] == 0 and first["null_face_count"] == 0,
        "bounding_box": within,
        "meaningful_face_count": meaningful,
        "low_curvature": regions.get("low_curvature", 0) >= 1,
        "convex": regions.get("convex", 0) >= 1,
        "concave": regions.get("concave", 0) >= 1,
        "saddle": regions.get("saddle", 0) >= 1,
        "landmark": first["landmark_face_count"] >= 1,
        "landmark_outside": first["landmark_outside_process_surface"],
        "finite_geometry": first["surface_query_fail"] == 0,
        "surface_query": first["surface_query_fail"] == 0 and first["surface_query_ok"] > 0,
        "normal_query": first["normal_query_fail"] == 0,
        "reproducibility": same and box_same and sample_same and second["bop_check"] == "PASS",
    }
    passed = all(checks.values())
    os.makedirs(EVIDENCE, exist_ok=True)
    with open(os.path.join(EVIDENCE, "cad_topology.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "faces": first["faces"],
            "counts": {key: first[key] for key in ("solid_count", "shell_count", "face_count", "edge_count", "vertex_count")},
        }, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(EVIDENCE, "cad_geometry.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "unit": "mm",
            "bounding_box_mm": first["bounding_box_mm"],
            "regions": regions,
            "landmark_z_max_mm": first["landmark_z_max_mm"],
            "process_z_min_mm": first["process_z_min_mm"],
            "landmark_outside_process_surface": first["landmark_outside_process_surface"],
            "surface_query_ok": first["surface_query_ok"],
            "surface_query_fail": first["surface_query_fail"],
            "normal_query_fail": first["normal_query_fail"],
        }, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(EVIDENCE, "cad_validation.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "step_read": checks["step_read"],
            "is_valid": first["is_valid"],
            "bop_check": first["bop_check"],
            "bop_error": first["bop_error"],
            "closed_shell": first["closed_shell"],
            "degenerate_edge_count": first["degenerate_edge_count"],
            "null_face_count": first["null_face_count"],
            "cad_validity": "PASS" if checks["cad_validity"] else "FAIL",
            "checks": checks,
            "final_gate": "PASS_COMPLEX_SURFACE_CAD_TEST_COUPON" if passed else "FAIL_COMPLEX_SURFACE_CAD_TEST_COUPON",
        }, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(EVIDENCE, "reproducibility.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "run_a_sha256": sha256(os.environ["P011_STEP_A"]),
            "run_b_sha256": sha256(os.environ["P011_STEP_B"]),
            "hash_equal": sha256(os.environ["P011_STEP_A"]) == sha256(os.environ["P011_STEP_B"]),
            "topology_equal": same,
            "bounding_box_equal": box_same,
            "sample_equal": sample_same,
            "run_b_bop": second["bop_check"],
            "result": "PASS" if checks["reproducibility"] else "FAIL",
            "hash_note": "export metadata 때문에 hash가 달라도 geometry가 같으면 FAIL로 보지 않는다.",
        }, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(EVIDENCE, "mesh_faces.json"), "w", encoding="utf-8") as handle:
        json.dump(mesh_payload(read_shape(os.environ["P011_STEP_A"])), handle)
    print("FINAL", "PASS_COMPLEX_SURFACE_CAD_TEST_COUPON" if passed else "FAIL")
    for name, value in checks.items():
        print(name, value)
    print("REGIONS", regions)
    print("LANDMARK", first["landmark_z_max_mm"], first["process_z_min_mm"], first["landmark_face_count"])
    print("NORMAL_FAIL", first["normal_query_fail"])


main()
