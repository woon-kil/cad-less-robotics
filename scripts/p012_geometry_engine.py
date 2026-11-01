#!/usr/bin/env python3
"""P-012. 동결된 STEP에서 곡면 미분기하를 읽는다.

영역 이름과 P-011 좌표는 쓰지 않는다. ProcessPath는 만들지 않는다.
"""

import hashlib
import json
import math
import os
import sys

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepClass import BRepClass_FaceClassifier
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepLProp import BRepLProp_SLProps
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GeomAbs import (
    GeomAbs_BezierSurface,
    GeomAbs_BSplineSurface,
    GeomAbs_Cone,
    GeomAbs_Cylinder,
    GeomAbs_OffsetSurface,
    GeomAbs_OtherSurface,
    GeomAbs_Plane,
    GeomAbs_Sphere,
    GeomAbs_SurfaceOfExtrusion,
    GeomAbs_SurfaceOfRevolution,
    GeomAbs_Torus,
)
from OCC.Core.GProp import GProp_GProps
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import (
    TopAbs_FACE,
    TopAbs_IN,
    TopAbs_ON,
    TopAbs_REVERSED,
    TopAbs_SHELL,
    TopAbs_SOLID,
)
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import topods
from OCC.Core.gp import gp_Pnt2d

ROOT = "/Users/woonkil/projects/cad-less-robotics"
STEP_PATH = os.path.join(ROOT, "assets/cad/complex_surface_coupon.step")
EXPECTED_SHA256 = "2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa"
EXPECTED_SOLID_COUNT = 1
EXPECTED_SHELL_COUNT = 1
EXPECTED_FACE_COUNT = 20

GRID_N = 9
CLASSIFIER_TOLERANCE = 1.0e-6
DERIVATIVE_RESOLUTION_MM = 1.0e-7
DEGENERATE_CROSS_MM2 = 1.0e-8
EPS_PRINCIPAL_PER_MM = 5.0e-3
EPS_GAUSSIAN_PER_MM2 = 1.0e-5
NORMAL_LENGTH_TOLERANCE = 1.0e-6

SURFACE_TYPES = {
    GeomAbs_Plane: "Plane",
    GeomAbs_Cylinder: "Cylinder",
    GeomAbs_Cone: "Cone",
    GeomAbs_Sphere: "Sphere",
    GeomAbs_Torus: "Torus",
    GeomAbs_BezierSurface: "Bezier",
    GeomAbs_BSplineSurface: "BSpline",
    GeomAbs_SurfaceOfRevolution: "Revolution",
    GeomAbs_SurfaceOfExtrusion: "Extrusion",
    GeomAbs_OffsetSurface: "Offset",
    GeomAbs_OtherSurface: "Other",
}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite(value):
    return math.isfinite(value)


def rounded(value):
    if value is None:
        return None
    return round(float(value), 10)


def count_shapes(shape, kind):
    total = 0
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        total += 1
        explorer.Next()
    return total


def unit_of(path):
    text = open(path, encoding="utf-8", errors="replace").read()
    if "SI_UNIT(.MILLI.,.METRE.)" in text:
        return "mm"
    return "UNKNOWN"


def classify_curvature(k1, k2, gaussian):
    if abs(k1) < EPS_PRINCIPAL_PER_MM and abs(k2) < EPS_PRINCIPAL_PER_MM:
        return "LOW_CURVATURE"
    if gaussian < -EPS_GAUSSIAN_PER_MM2:
        return "SADDLE"
    if gaussian > EPS_GAUSSIAN_PER_MM2:
        return "ELLIPTIC"
    return "PARABOLIC_OR_TRANSITION"


def load_shape(path):
    reader = STEPControl_Reader()
    status = reader.ReadFile(path)
    if status != IFSelect_RetDone:
        raise RuntimeError("STEP_READ_FAILED")
    reader.TransferRoots()
    return reader.OneShape()


def face_records(shape):
    records = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    traversal = 0
    while explorer.More():
        face = topods.Face(explorer.Current())
        adaptor = BRepAdaptor_Surface(face, True)
        mass = GProp_GProps()
        brepgprop.SurfaceProperties(face, mass)
        center = mass.CentreOfMass()
        kind = SURFACE_TYPES.get(adaptor.GetType(), "Other")
        orientation = "REVERSED" if face.Orientation() == TopAbs_REVERSED else "FORWARD"
        records.append({
            "traversal_index": traversal,
            "face": face,
            "adaptor": adaptor,
            "surface_type": kind,
            "orientation": orientation,
            "orientation_corrected": orientation == "REVERSED",
            "u_min": adaptor.FirstUParameter(),
            "u_max": adaptor.LastUParameter(),
            "v_min": adaptor.FirstVParameter(),
            "v_max": adaptor.LastVParameter(),
            "area_mm2": mass.Mass(),
            "centroid_mm": [center.X(), center.Y(), center.Z()],
        })
        traversal += 1
        explorer.Next()
    records.sort(key=lambda item: (
        item["surface_type"],
        round(item["area_mm2"], 4),
        round(item["centroid_mm"][0], 4),
        round(item["centroid_mm"][1], 4),
        round(item["centroid_mm"][2], 4),
        item["orientation"],
    ))
    for index, record in enumerate(records):
        record["face_id"] = f"F{index:02d}"
    return records


def parameter_grid(start, end):
    if GRID_N == 1:
        return [(start + end) / 2.0]
    return [start + (end - start) * index / (GRID_N - 1) for index in range(GRID_N)]


def inside_trim(face, u, v):
    state = BRepClass_FaceClassifier(face, gp_Pnt2d(u, v), CLASSIFIER_TOLERANCE).State()
    return state == TopAbs_IN or state == TopAbs_ON


def sample_face(record):
    samples = []
    face = record["face"]
    adaptor = record["adaptor"]
    corrected = record["orientation_corrected"]
    for u in parameter_grid(record["u_min"], record["u_max"]):
        for v in parameter_grid(record["v_min"], record["v_max"]):
            base = {
                "face_id": record["face_id"],
                "uv": [rounded(u), rounded(v)],
                "position_mm": None,
                "normal": None,
                "k1_per_mm": None,
                "k2_per_mm": None,
                "gaussian_per_mm2": None,
                "mean_per_mm": None,
                "classification": None,
                "validity": None,
                "orientation_corrected": corrected,
            }
            if not inside_trim(face, u, v):
                base["validity"] = "OUTSIDE_TRIM"
                samples.append(base)
                continue
            props = BRepLProp_SLProps(adaptor, u, v, 2, DERIVATIVE_RESOLUTION_MM)
            point = props.Value()
            position = [point.X(), point.Y(), point.Z()]
            if not all(finite(item) for item in position):
                base["validity"] = "NONFINITE_GEOMETRY"
                samples.append(base)
                continue
            base["position_mm"] = [rounded(item) for item in position]
            if not props.IsNormalDefined():
                base["validity"] = "DEGENERATE_DIFFERENTIAL"
                samples.append(base)
                continue
            su = props.D1U()
            sv = props.D1V()
            raw = su.Crossed(sv)
            raw_length = raw.Magnitude()
            derivatives = [su.X(), su.Y(), su.Z(), sv.X(), sv.Y(), sv.Z(), raw.X(), raw.Y(), raw.Z()]
            if not all(finite(item) for item in derivatives) or raw_length < DEGENERATE_CROSS_MM2:
                base["validity"] = "DEGENERATE_DIFFERENTIAL"
                samples.append(base)
                continue
            normal = [raw.X() / raw_length, raw.Y() / raw_length, raw.Z() / raw_length]
            if corrected:
                normal = [-item for item in normal]
            length = math.sqrt(sum(item * item for item in normal))
            if not all(finite(item) for item in normal) or abs(length - 1.0) > NORMAL_LENGTH_TOLERANCE:
                base["normal"] = [rounded(item) for item in normal]
                base["validity"] = "NORMAL_FAILURE"
                samples.append(base)
                continue
            base["normal"] = [rounded(item) for item in normal]
            if not props.IsCurvatureDefined():
                base["validity"] = "CURVATURE_UNDEFINED"
                samples.append(base)
                continue
            k_min = props.MinCurvature()
            k_max = props.MaxCurvature()
            if not finite(k_min) or not finite(k_max):
                base["validity"] = "NONFINITE_GEOMETRY"
                samples.append(base)
                continue
            if corrected:
                k_min, k_max = -k_max, -k_min
            k1, k2 = sorted((k_min, k_max))
            gaussian = k1 * k2
            mean = (k1 + k2) / 2.0
            if not all(finite(item) for item in (gaussian, mean)):
                base["validity"] = "NONFINITE_GEOMETRY"
                samples.append(base)
                continue
            base["k1_per_mm"] = rounded(k1)
            base["k2_per_mm"] = rounded(k2)
            base["gaussian_per_mm2"] = rounded(gaussian)
            base["mean_per_mm"] = rounded(mean)
            base["classification"] = classify_curvature(k1, k2, gaussian)
            base["validity"] = "VALID"
            samples.append(base)
    return samples


def mesh_payload(shape, records):
    BRepMesh_IncrementalMesh(shape, 0.6, False, 0.5, True)
    by_traversal = {record["traversal_index"]: record["face_id"] for record in records}
    faces = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    traversal = 0
    while explorer.More():
        face = topods.Face(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation(face, location)
        triangles = []
        if triangulation is not None:
            transform = location.Transformation()
            for index in range(1, triangulation.NbTriangles() + 1):
                triangle = triangulation.Triangle(index)
                nodes = triangle.Get()
                points = []
                for node_index in nodes:
                    point = triangulation.Node(node_index).Transformed(transform)
                    points.append([rounded(point.X()), rounded(point.Y()), rounded(point.Z())])
                triangles.append(points)
        faces.append({"face_id": by_traversal[traversal], "triangles": triangles})
        traversal += 1
        explorer.Next()
    return {"faces": faces}


def inventory(records):
    exported = []
    for record in records:
        exported.append({
            "face_id": record["face_id"],
            "surface_type": record["surface_type"],
            "u_min": rounded(record["u_min"]),
            "u_max": rounded(record["u_max"]),
            "v_min": rounded(record["v_min"]),
            "v_max": rounded(record["v_max"]),
            "orientation": record["orientation"],
            "orientation_corrected": record["orientation_corrected"],
            "area_mm2": rounded(record["area_mm2"]),
            "centroid_mm": [rounded(item) for item in record["centroid_mm"]],
            "raw_normal_dot_final": rounded(orientation_dot(record)),
            "uv_note": "u_min/u_max/v_min/v_max는 trim의 bounding rectangle이다. 사각형 내부가 face 내부는 아니다.",
        })
    return exported


def orientation_dot(record):
    """Su×Sv와 보정 후 법선의 내적. REVERSED면 -1이어야 한다."""
    face = record["face"]
    adaptor = record["adaptor"]
    for u in parameter_grid(record["u_min"], record["u_max"]):
        for v in parameter_grid(record["v_min"], record["v_max"]):
            if not inside_trim(face, u, v):
                continue
            props = BRepLProp_SLProps(adaptor, u, v, 1, DERIVATIVE_RESOLUTION_MM)
            if not props.IsNormalDefined():
                continue
            raw = props.D1U().Crossed(props.D1V())
            length = raw.Magnitude()
            if length < DEGENERATE_CROSS_MM2:
                continue
            raw_n = [raw.X() / length, raw.Y() / length, raw.Z() / length]
            final = [-item for item in raw_n] if record["orientation_corrected"] else raw_n
            return sum(raw_n[index] * final[index] for index in range(3))
    return None


def write_parameters(directory):
    text = """# P-012 geometry parameters. scripts/p012_geometry_engine.py 와 같다.
# 분류는 곡률 수치만 사용한다. P-011 좌표는 없다.

unit: mm
curvature_unit: 1/mm
gaussian_unit: 1/mm^2
grid_n: 9
classifier_tolerance: 1.0e-6
derivative_resolution_mm: 1.0e-7
degenerate_cross_mm2: 1.0e-8
eps_principal_per_mm: 5.0e-3
eps_gaussian_per_mm2: 1.0e-5
normal_length_tolerance: 1.0e-6
orientation_convention: >-
  N_raw = Su × Sv. TopoDS_Face가 REVERSED이면 N = -N_raw이고
  principal curvature의 부호를 바꾼다. Gaussian curvature는 곱이라 부호가 유지된다.
  BRepLProp_SLProps의 normal은 이 빌드에서 face orientation을 뒤집지 않는다.
classification: >-
  |k1| < eps_principal and |k2| < eps_principal -> LOW_CURVATURE.
  K < -eps_gaussian -> SADDLE.
  K > eps_gaussian -> ELLIPTIC.
  else PARABOLIC_OR_TRANSITION.
  CONVEX와 CONCAVE는 곡률 부호만으로 정하지 않는다.
"""
    with open(os.path.join(directory, "geometry_parameters.yaml"), "w", encoding="utf-8") as handle:
        handle.write(text)


def main():
    output = os.environ["P012_OUT"]
    os.makedirs(output, exist_ok=True)
    digest = sha256(STEP_PATH)
    if digest != EXPECTED_SHA256:
        print("INPUT_INTEGRITY BLOCKED")
        print(digest)
        sys.exit(2)
    shape = load_shape(STEP_PATH)
    solid_count = count_shapes(shape, TopAbs_SOLID)
    shell_count = count_shapes(shape, TopAbs_SHELL)
    records = face_records(shape)
    if (
        unit_of(STEP_PATH) != "mm"
        or solid_count != EXPECTED_SOLID_COUNT
        or shell_count != EXPECTED_SHELL_COUNT
        or len(records) != EXPECTED_FACE_COUNT
    ):
        print("TOPOLOGY_MISMATCH", solid_count, shell_count, len(records))
        sys.exit(3)
    samples = []
    for record in records:
        samples.extend(sample_face(record))
    with open(os.path.join(output, "face_inventory.json"), "w", encoding="utf-8") as handle:
        json.dump({"faces": inventory(records)}, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(output, "cad_surface_samples.json"), "w", encoding="utf-8") as handle:
        json.dump({"samples": samples}, handle, ensure_ascii=False)
    with open(os.path.join(output, "mesh.json"), "w", encoding="utf-8") as handle:
        json.dump(mesh_payload(shape, records), handle)
    write_parameters(output)
    counts = {}
    for sample in samples:
        counts[sample["validity"]] = counts.get(sample["validity"], 0) + 1
    print("SAMPLES", len(samples), counts)
    print("EXPORTED", output)


main()
