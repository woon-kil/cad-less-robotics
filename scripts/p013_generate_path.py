#!/usr/bin/env python3
"""P-013. 동결된 계약으로 CAD ProcessPath를 만든다.

P-009, P-011 STEP, P-012 엔진은 수정하지 않는다.
"""

import hashlib
import json
import math
import os
import sys

from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCC.Core.BRepClass import BRepClass_FaceClassifier
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepLProp import BRepLProp_SLProps
from OCC.Core.GProp import GProp_GProps
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
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_ON, TopAbs_REVERSED
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods
from OCC.Core.gp import gp_Pnt, gp_Pnt2d

ROOT = "/Users/woonkil/projects/cad-less-robotics"
STEP_PATH = os.path.join(ROOT, "assets/cad/complex_surface_coupon.step")
EXPECTED_SHA256 = "2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa"

TARGET_SPACING_MM = 5.0
PASS_PITCH_MM = 5.0
TOOL_STANDOFF_MM = 20.0
CAD_X = (1.0, 0.0, 0.0)
EPS_TOOL_TANGENT = 1.0e-4
NZ_PROCESS_MIN = 0.5
CLASSIFIER_TOLERANCE = 1.0e-6
DERIVATIVE_RESOLUTION_MM = 1.0e-7
XY_ACCEPT_MM = 0.05
DENSE_STEP_MM = 0.5

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


def rounded(value):
    if value is None:
        return None
    return round(float(value), 10)


def finite(values):
    return all(math.isfinite(item) for item in values)


def dot(left, right):
    return sum(left[index] * right[index] for index in range(len(left)))


def cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def length(vector):
    return math.sqrt(sum(item * item for item in vector))


def sub(left, right):
    return tuple(left[index] - right[index] for index in range(len(left)))


def add(left, right):
    return tuple(left[index] + right[index] for index in range(len(left)))


def scale(vector, factor):
    return tuple(item * factor for item in vector)


def normalize(vector, eps):
    magnitude = length(vector)
    if magnitude < eps or not math.isfinite(magnitude):
        return None
    return scale(vector, 1.0 / magnitude)


def load_shape(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError("STEP_READ_FAILED")
    reader.TransferRoots()
    return reader.OneShape()


def inside_trim(face, u, v):
    state = BRepClass_FaceClassifier(face, gp_Pnt2d(u, v), CLASSIFIER_TOLERANCE).State()
    return state == TopAbs_IN or state == TopAbs_ON


def face_records(shape):
    records = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = topods.Face(explorer.Current())
        adaptor = BRepAdaptor_Surface(face, True)
        mass = GProp_GProps()
        brepgprop.SurfaceProperties(face, mass)
        center = mass.CentreOfMass()
        records.append({
            "face": face,
            "adaptor": adaptor,
            "surface_type": SURFACE_TYPES.get(adaptor.GetType(), "Other"),
            "orientation": "REVERSED" if face.Orientation() == TopAbs_REVERSED else "FORWARD",
            "u_min": adaptor.FirstUParameter(),
            "u_max": adaptor.LastUParameter(),
            "v_min": adaptor.FirstVParameter(),
            "v_max": adaptor.LastVParameter(),
            "area_mm2": mass.Mass(),
            "centroid_mm": (center.X(), center.Y(), center.Z()),
        })
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


def evaluate(record, u, v):
    props = BRepLProp_SLProps(record["adaptor"], u, v, 1, DERIVATIVE_RESOLUTION_MM)
    point = props.Value()
    position = (point.X(), point.Y(), point.Z())
    if not props.IsNormalDefined() or not finite(position):
        return None
    raw = props.D1U().Crossed(props.D1V())
    raw_n = (raw.X(), raw.Y(), raw.Z())
    if not finite(raw_n) or length(raw_n) < 1.0e-8:
        return None
    normal = normalize(raw_n, 1.0e-8)
    if record["orientation"] == "REVERSED":
        normal = scale(normal, -1.0)
    return {"position": position, "normal": normal, "u": u, "v": v}


def project_face(record, x, y):
    u_span = record["u_max"] - record["u_min"]
    v_span = record["v_max"] - record["v_min"]
    u = (record["u_min"] + record["u_max"]) / 2.0
    v = (record["v_min"] + record["v_max"]) / 2.0
    hit = None
    for _ in range(16):
        hit = evaluate(record, u, v)
        if hit is None:
            return None
        px, py, _pz = hit["position"]
        props = BRepLProp_SLProps(record["adaptor"], u, v, 1, DERIVATIVE_RESOLUTION_MM)
        su = props.D1U()
        sv = props.D1V()
        ex = x - px
        ey = y - py
        det = su.X() * sv.Y() - sv.X() * su.Y()
        if abs(det) < 1.0e-12:
            break
        du = (ex * sv.Y() - ey * sv.X()) / det
        dv = (su.X() * ey - su.Y() * ex) / det
        step = max(abs(du), abs(dv))
        if step > 0.2 * max(abs(u_span), abs(v_span), 1.0):
            scale_step = 0.2 * max(abs(u_span), abs(v_span), 1.0) / step
            du *= scale_step
            dv *= scale_step
        u = min(max(u + du, record["u_min"]), record["u_max"])
        v = min(max(v + dv, record["v_min"]), record["v_max"])
        if abs(ex) < XY_ACCEPT_MM and abs(ey) < XY_ACCEPT_MM:
            break
    if hit is None or not inside_trim(record["face"], u, v):
        return None
    hit = evaluate(record, u, v)
    if hit is None:
        return None
    error = math.hypot(hit["position"][0] - x, hit["position"][1] - y)
    if error > XY_ACCEPT_MM:
        return None
    hit["xy_error_mm"] = error
    hit["face_id"] = record["face_id"]
    hit["face"] = record["face"]
    return hit


def project_assembly(records, x, y):
    best = None
    for record in records:
        hit = project_face(record, x, y)
        if hit is None:
            continue
        if best is None or hit["xy_error_mm"] < best["xy_error_mm"] or (
            hit["xy_error_mm"] == best["xy_error_mm"] and hit["face_id"] < best["face_id"]
        ):
            best = hit
    return best


def surface_distance(hit):
    vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(*hit["position"])).Shape()
    tool = BRepExtrema_DistShapeShape(vertex, hit["face"])
    if not tool.IsDone():
        return None
    return tool.Value()


def mean_nz(record):
    values = []
    for i in range(5):
        for j in range(5):
            u = record["u_min"] + (record["u_max"] - record["u_min"]) * i / 4.0
            v = record["v_min"] + (record["v_max"] - record["v_min"]) * j / 4.0
            if not inside_trim(record["face"], u, v):
                continue
            hit = evaluate(record, u, v)
            if hit is not None:
                values.append(hit["normal"][2])
    if not values:
        return 0.0
    return sum(values) / len(values)


def select_process_faces(records):
    chosen = []
    report = []
    for record in records:
        nz = mean_nz(record)
        selected = record["surface_type"] == "BSpline" and nz > NZ_PROCESS_MIN
        report.append({
            "face_id": record["face_id"],
            "surface_type": record["surface_type"],
            "mean_nz": rounded(nz),
            "selected": selected,
        })
        if selected:
            chosen.append(record)
    return chosen, report


def domain_of(records):
    xs = []
    ys = []
    for record in records:
        for i in range(7):
            for j in range(7):
                u = record["u_min"] + (record["u_max"] - record["u_min"]) * i / 6.0
                v = record["v_min"] + (record["v_max"] - record["v_min"]) * j / 6.0
                if not inside_trim(record["face"], u, v):
                    continue
                hit = evaluate(record, u, v)
                if hit is not None:
                    xs.append(hit["position"][0])
                    ys.append(hit["position"][1])
    return min(xs), max(xs), min(ys), max(ys)


def pass_ys(y_min, y_max):
    values = []
    y = y_min
    while y <= y_max + 1.0e-6:
        values.append(min(y, y_max))
        if y >= y_max - 1.0e-6:
            break
        y += PASS_PITCH_MM
    if values[-1] != y_max and abs(values[-1] - y_max) > 1.0e-6:
        values.append(y_max)
    return values


def dense_xs(x_min, x_max):
    count = max(2, int(math.ceil((x_max - x_min) / DENSE_STEP_MM)) + 1)
    return [x_min + (x_max - x_min) * index / (count - 1) for index in range(count)]


def arc_samples(records, y, x_min, x_max):
    samples = []
    distance = 0.0
    previous = None
    for x in dense_xs(x_min, x_max):
        hit = project_assembly(records, x, y)
        if hit is None:
            continue
        if previous is not None:
            distance += length(sub(hit["position"], previous))
        previous = hit["position"]
        samples.append((distance, x))
    return samples


def x_at_distance(samples, target):
    if target <= samples[0][0]:
        return samples[0][1]
    if target >= samples[-1][0]:
        return samples[-1][1]
    for index in range(1, len(samples)):
        left_s, left_x = samples[index - 1]
        right_s, right_x = samples[index]
        if left_s <= target <= right_s:
            span = right_s - left_s
            ratio = 0.0 if span == 0.0 else (target - left_s) / span
            return left_x + (right_x - left_x) * ratio
    return samples[-1][1]


def build_passes(records, x_min, x_max, y_min, y_max):
    passes = []
    for pass_index, y in enumerate(pass_ys(y_min, y_max)):
        samples = arc_samples(records, y, x_min, x_max)
        if len(samples) < 2:
            continue
        intervals = max(1, int(round(samples[-1][0] / TARGET_SPACING_MM)))
        direction = 1 if pass_index % 2 == 0 else -1
        targets = [index * samples[-1][0] / intervals for index in range(intervals + 1)]
        if direction < 0:
            targets.reverse()
        points = []
        for target in targets:
            hit = project_assembly(records, x_at_distance(samples, target), y)
            if hit is None:
                continue
            hit["pass_index"] = pass_index
            hit["surface_distance_mm"] = surface_distance(hit)
            points.append(hit)
        if points:
            passes.append(points)
    return passes


def tool_frame(normal, previous_tool):
    z_tool = scale(normal, -1.0)
    raw = sub(CAD_X, scale(z_tool, dot(CAD_X, z_tool)))
    if length(raw) < EPS_TOOL_TANGENT:
        return None
    tool = normalize(raw, EPS_TOOL_TANGENT)
    if previous_tool is not None and dot(tool, previous_tool) < 0.0:
        tool = scale(tool, -1.0)
    x_axis = normalize(sub(tool, scale(z_tool, dot(tool, z_tool))), EPS_TOOL_TANGENT)
    if x_axis is None:
        return None
    y_axis = normalize(cross(z_tool, x_axis), EPS_TOOL_TANGENT)
    if y_axis is None:
        return None
    x_axis = normalize(cross(y_axis, z_tool), EPS_TOOL_TANGENT)
    if x_axis is None:
        return None
    return {"tool_tangent": tool, "x": x_axis, "y": y_axis, "z": z_tool}


def matrix_det(x_axis, y_axis, z_axis):
    return (
        x_axis[0] * (y_axis[1] * z_axis[2] - y_axis[2] * z_axis[1])
        - x_axis[1] * (y_axis[0] * z_axis[2] - y_axis[2] * z_axis[0])
        + x_axis[2] * (y_axis[0] * z_axis[1] - y_axis[1] * z_axis[0])
    )


def quaternion_from_axes(x_axis, y_axis, z_axis):
    trace = x_axis[0] + y_axis[1] + z_axis[2]
    if trace > 0.0:
        scale_q = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale_q
        qx = (y_axis[2] - z_axis[1]) / scale_q
        qy = (z_axis[0] - x_axis[2]) / scale_q
        qz = (x_axis[1] - y_axis[0]) / scale_q
    elif x_axis[0] > y_axis[1] and x_axis[0] > z_axis[2]:
        scale_q = math.sqrt(1.0 + x_axis[0] - y_axis[1] - z_axis[2]) * 2.0
        qw = (y_axis[2] - z_axis[1]) / scale_q
        qx = 0.25 * scale_q
        qy = (y_axis[0] + x_axis[1]) / scale_q
        qz = (z_axis[0] + x_axis[2]) / scale_q
    elif y_axis[1] > z_axis[2]:
        scale_q = math.sqrt(1.0 + y_axis[1] - x_axis[0] - z_axis[2]) * 2.0
        qw = (z_axis[0] - x_axis[2]) / scale_q
        qx = (y_axis[0] + x_axis[1]) / scale_q
        qy = 0.25 * scale_q
        qz = (z_axis[1] + y_axis[2]) / scale_q
    else:
        scale_q = math.sqrt(1.0 + z_axis[2] - x_axis[0] - y_axis[1]) * 2.0
        qw = (x_axis[1] - y_axis[0]) / scale_q
        qx = (z_axis[0] + x_axis[2]) / scale_q
        qy = (z_axis[1] + y_axis[2]) / scale_q
        qz = 0.25 * scale_q
    quat = (qx, qy, qz, qw)
    if not finite(quat):
        return None
    unit = normalize(quat, 1.0e-12)
    return unit


def traversal_tangents(points):
    tangents = []
    for index, point in enumerate(points):
        if len(points) == 1:
            raw = CAD_X
        elif index == 0:
            raw = sub(points[1]["position"], point["position"])
        elif index == len(points) - 1:
            raw = sub(point["position"], points[index - 1]["position"])
        else:
            raw = sub(points[index + 1]["position"], points[index - 1]["position"])
        tangent = normalize(raw, 1.0e-8)
        tangents.append(tangent if tangent is not None else (0.0, 0.0, 0.0))
    return tangents


def vector_json(vector):
    if vector is None:
        return None
    return [rounded(item) for item in vector]


def assemble(passes):
    waypoints = []
    failures = {
        "SURFACE_QUERY_FAILURE": 0,
        "OUTSIDE_PROCESS_SURFACE": 0,
        "NORMAL_FAILURE": 0,
        "TOOL_TANGENT_DEGENERATE": 0,
        "FRAME_INVALID": 0,
        "QUATERNION_INVALID": 0,
        "NONFINITE": 0,
    }
    previous_quat = None
    previous_tool = None
    sequence = 0
    for pass_index, points in enumerate(passes):
        tangents = traversal_tangents(points)
        for index, point in enumerate(points):
            segment = "TRANSITION" if index == 0 and pass_index > 0 else "PROCESS"
            normal = point["normal"]
            if not finite(normal) or abs(length(normal) - 1.0) > 1.0e-6:
                failures["NORMAL_FAILURE"] += 1
            frame = tool_frame(normal, previous_tool)
            degenerate = frame is None
            if degenerate:
                failures["TOOL_TANGENT_DEGENERATE"] += 1
                tool_tangent = None
                quat = None
                axes = None
            else:
                tool_tangent = frame["tool_tangent"]
                previous_tool = tool_tangent
                axes = (frame["x"], frame["y"], frame["z"])
                quat = quaternion_from_axes(*axes)
                if quat is None:
                    failures["QUATERNION_INVALID"] += 1
                elif previous_quat is not None and dot(quat, previous_quat) < 0.0:
                    quat = scale(quat, -1.0)
                if quat is not None:
                    previous_quat = quat
            position = point["position"]
            if not finite(position):
                failures["NONFINITE"] += 1
            command = add(position, scale(normal, TOOL_STANDOFF_MM))
            distance = point["surface_distance_mm"]
            if distance is None or not math.isfinite(distance):
                failures["SURFACE_QUERY_FAILURE"] += 1
            waypoint = {
                "sequence_id": sequence,
                "pass_index": pass_index,
                "surface_anchor": vector_json(position),
                "command_position": vector_json(command),
                "surface_normal": vector_json(normal),
                "traversal_tangent": vector_json(tangents[index]),
                "tool_tangent": vector_json(tool_tangent),
                "orientation_quaternion": vector_json(quat),
                "process_offset_mm": TOOL_STANDOFF_MM,
                "source": "CAD",
                "geometry_role": "REFERENCE_GEOMETRY",
                "face_id": point["face_id"],
                "segment_type": segment,
                "confidence": "NOT_DEFINED",
                "surface_distance_mm": rounded(distance),
                "frame_id": "CAD_MODEL_FRAME",
            }
            if axes is not None and quat is not None:
                waypoint["frame_det"] = rounded(matrix_det(*axes))
                waypoint["frame_error"] = rounded(max(
                    abs(dot(axes[0], axes[1])),
                    abs(dot(axes[1], axes[2])),
                    abs(dot(axes[2], axes[0])),
                    abs(length(axes[0]) - 1.0),
                    abs(length(axes[1]) - 1.0),
                    abs(length(axes[2]) - 1.0),
                ))
            else:
                waypoint["frame_det"] = None
                waypoint["frame_error"] = None
                failures["FRAME_INVALID"] += 1
            waypoints.append(waypoint)
            sequence += 1
    return waypoints, failures


def pair_span(waypoints):
    process_gaps = []
    transition_gaps = []
    process_angles = []
    transition_angles = []
    process_normals = []
    crossings = []
    for index in range(1, len(waypoints)):
        left = waypoints[index - 1]
        right = waypoints[index]
        gap = length(sub(right["surface_anchor"], left["surface_anchor"]))
        q_left = left["orientation_quaternion"]
        q_right = right["orientation_quaternion"]
        angle = None
        if q_left and q_right:
            cosine = abs(max(-1.0, min(1.0, dot(q_left, q_right))))
            angle = math.degrees(2.0 * math.acos(cosine))
        normal_dot = dot(left["surface_normal"], right["surface_normal"])
        same_pass = left["pass_index"] == right["pass_index"]
        if same_pass:
            process_gaps.append(gap)
            process_normals.append(normal_dot)
            if angle is not None:
                process_angles.append(angle)
            if left["face_id"] != right["face_id"]:
                crossings.append({
                    "sequence_id": right["sequence_id"],
                    "from_face": left["face_id"],
                    "to_face": right["face_id"],
                    "anchor_distance_mm": rounded(gap),
                    "normal_dot": rounded(normal_dot),
                    "orientation_delta_deg": rounded(angle),
                    "spacing_mm": rounded(gap),
                })
        else:
            transition_gaps.append(gap)
            if angle is not None:
                transition_angles.append(angle)
    return process_gaps, transition_gaps, process_angles, transition_angles, process_normals, crossings


def percentile(values, ratio):
    if not values:
        return None
    ordered = sorted(values)
    position = min(len(ordered) - 1, max(0, int(math.ceil(ratio * len(ordered)) - 1)))
    return ordered[position]


def write_parameters(directory):
    text = """# P-013 구현 파라미터. 동결 계약을 바꾸지 않는다.
unit: mm
frame_id: CAD_MODEL_FRAME
target_spacing_mm: 5.0
pass_pitch_mm: 5.0
tool_standoff_mm: 20.0
cad_x: [1.0, 0.0, 0.0]
eps_tool_tangent: 1.0e-4
process_surface_rule: BSpline and mean oriented Nz > 0.5
quaternion_order: [x, y, z, w]
confidence: NOT_DEFINED
tool_tangent_degenerate: TOOL_TANGENT_DEGENERATE
no_axis_fallback: true
"""
    with open(os.path.join(directory, "generation_parameters.yaml"), "w", encoding="utf-8") as handle:
        handle.write(text)


def main():
    output = os.environ["P013_OUT"]
    os.makedirs(output, exist_ok=True)
    digest = sha256(STEP_PATH)
    if digest != EXPECTED_SHA256:
        print("BLOCKED_INPUT_INTEGRITY")
        print(digest)
        sys.exit(2)
    shape = load_shape(STEP_PATH)
    records = face_records(shape)
    chosen, selection = select_process_faces(records)
    if len(chosen) != 4:
        print("PROCESS_SURFACE_COUNT", len(chosen))
    x_min, x_max, y_min, y_max = domain_of(chosen)
    passes = build_passes(chosen, x_min, x_max, y_min, y_max)
    waypoints, failures = assemble(passes)
    process_gaps, transition_gaps, process_angles, transition_angles, process_normals, crossings = pair_span(waypoints)
    distances = [item["surface_distance_mm"] for item in waypoints if item["surface_distance_mm"] is not None]
    spacing_errors = [abs(gap - TARGET_SPACING_MM) for gap in process_gaps]
    out_of_band = sum(1 for gap in process_gaps if gap < 4.0 or gap > 6.0)
    flips = sum(1 for value in process_normals if value < 0.0)
    quats = [item["orientation_quaternion"] for item in waypoints if item["orientation_quaternion"]]
    quat_norms = [length(item) for item in quats]
    dets = [item["frame_det"] for item in waypoints if item["frame_det"] is not None]
    frame_errors = [item["frame_error"] for item in waypoints if item["frame_error"] is not None]
    over_45 = sum(1 for angle in process_angles if angle > 45.0)
    bad_crossings = [
        item for item in crossings
        if item["anchor_distance_mm"] is None
        or item["anchor_distance_mm"] < 4.0
        or item["anchor_distance_mm"] > 6.0
        or item["normal_dot"] < 0.0
        or (item["orientation_delta_deg"] or 0.0) > 45.0
    ]
    failures["SPACING_VIOLATION"] = out_of_band
    failures["ORIENTATION_DISCONTINUITY"] = over_45
    metrics = {
        "process_surface_count": len(chosen),
        "process_faces": [record["face_id"] for record in chosen],
        "domain_mm": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
        "waypoint_count": len(waypoints),
        "process_segment_count": len(process_gaps),
        "transition_segment_count": len(transition_gaps),
        "process_path_length_mm": sum(process_gaps),
        "spacing_median_mm": percentile(process_gaps, 0.50),
        "spacing_p95_error_mm": percentile(spacing_errors, 0.95),
        "spacing_min_mm": min(process_gaps) if process_gaps else None,
        "spacing_max_mm": max(process_gaps) if process_gaps else None,
        "spacing_out_of_band_count": out_of_band,
        "transition_spacing_mm": transition_gaps,
        "surface_distance_median_mm": percentile(distances, 0.50),
        "surface_distance_p95_mm": percentile(distances, 0.95),
        "surface_distance_max_mm": max(distances) if distances else None,
        "normal_dot_min": min(process_normals) if process_normals else None,
        "normal_flip_count": flips,
        "tool_tangent_degenerate_count": failures["TOOL_TANGENT_DEGENERATE"],
        "orientation_delta_median_deg": percentile(process_angles, 0.50),
        "orientation_delta_p95_deg": percentile(process_angles, 0.95),
        "orientation_delta_max_deg": max(process_angles) if process_angles else None,
        "orientation_over_45_count": over_45,
        "transition_orientation_max_deg": max(transition_angles) if transition_angles else None,
        "quaternion_norm_min": min(quat_norms) if quat_norms else None,
        "quaternion_norm_max": max(quat_norms) if quat_norms else None,
        "frame_orthogonality_max_error": max(frame_errors) if frame_errors else None,
        "rotation_det_min": min(dets) if dets else None,
        "rotation_det_max": max(dets) if dets else None,
        "face_boundary_crossing_count": len(crossings),
        "face_boundary_continuity": "PASS" if crossings and not bad_crossings else ("FAIL" if bad_crossings else "PASS"),
        "nonfinite_count": failures["NONFINITE"],
        "failures": failures,
    }
    with open(os.path.join(output, "cad_process_path.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "frame_id": "CAD_MODEL_FRAME",
            "position_unit": "mm",
            "quaternion_order": ["x", "y", "z", "w"],
            "source": "CAD",
            "geometry_role": "REFERENCE_GEOMETRY",
            "ground_truth": "NONE",
            "process_surface_selection": selection,
            "waypoints": waypoints,
            "boundary_crossings": crossings,
        }, handle)
    with open(os.path.join(output, "path_metrics.json"), "w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2)
    write_parameters(output)
    print("SURFACES", metrics["process_surface_count"], metrics["process_faces"])
    print("WAYPOINTS", metrics["waypoint_count"], "PROCESS", metrics["process_segment_count"], "TRANSITION", metrics["transition_segment_count"])
    print("SPACING", metrics["spacing_min_mm"], metrics["spacing_median_mm"], metrics["spacing_max_mm"], "OUT", metrics["spacing_out_of_band_count"])
    print("DIST", metrics["surface_distance_p95_mm"], metrics["surface_distance_max_mm"])
    print("ORI", metrics["orientation_delta_max_deg"], "OVER45", metrics["orientation_over_45_count"])
    print("DEGEN", metrics["tool_tangent_degenerate_count"], "FLIP", metrics["normal_flip_count"], "CROSS", metrics["face_boundary_crossing_count"])
    print("EXPORTED", output)


main()
