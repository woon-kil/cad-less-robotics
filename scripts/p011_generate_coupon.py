#!/usr/bin/env python3
"""P-011. 동결된 형상 계약으로 Complex Surface Coupon을 만든다.

FreeCAD Part(OpenCascade)만 사용한다. GUI는 쓰지 않는다.
곡률 엔진, ProcessPath, 출력, 촬영, registration은 하지 않는다.
"""

import math
import os

import Part
from FreeCAD import Vector
from OCC.Core.Interface import Interface_Static

BODY_X_MM = 168.0
BODY_Y_MM = 100.0
BASE_Z_MM = 28.0
PATCH_EDGES_MM = (0.0, 40.0, 82.0, 124.0, 168.0)
POLES_U = 6
POLES_V = 8
FLAT_BLEND_START_MM = 156.0
FLAT_EDGE_Z_MM = 28.0
CONVEX_CENTER_MM = (64.0, 50.0)
CONVEX_SIGMA_MM = (11.0, 20.0)
CONVEX_AMPLITUDE_MM = 14.0
SADDLE_CENTER_MM = (104.0, 50.0)
SADDLE_SCALE_MM = (16.0, 18.0)
SADDLE_AMPLITUDE_MM = 14.0
CONCAVE_CENTER_MM = (146.0, 38.0)
CONCAVE_SIGMA_MM = (10.0, 10.0)
CONCAVE_AMPLITUDE_MM = 8.0
LOW_CURVATURE_MM = 0.00008
LANDMARK_AXIS_MM = (162.0, 84.0, 10.0)
LANDMARK_RADIUS_MM = 5.0
LANDMARK_LENGTH_MM = 18.0


def window(dx, dy, sx, sy):
    return math.exp(-0.5 * (dx / sx) ** 2 - 0.5 * (dy / sy) ** 2)


def z_top(x, y):
    """공정 표면 높이(mm). 바닥은 z=0이다."""
    height = BASE_Z_MM
    height += LOW_CURVATURE_MM * (y - 50.0) ** 2
    height += CONVEX_AMPLITUDE_MM * window(
        x - CONVEX_CENTER_MM[0], y - CONVEX_CENTER_MM[1],
        CONVEX_SIGMA_MM[0], CONVEX_SIGMA_MM[1],
    )
    sx = (x - SADDLE_CENTER_MM[0]) / SADDLE_SCALE_MM[0]
    sy = (y - SADDLE_CENTER_MM[1]) / SADDLE_SCALE_MM[1]
    height += SADDLE_AMPLITUDE_MM * (sx * sx - sy * sy) * math.exp(-(sx * sx) - (sy * sy))
    height -= CONCAVE_AMPLITUDE_MM * window(
        x - CONCAVE_CENTER_MM[0], y - CONCAVE_CENTER_MM[1],
        CONCAVE_SIGMA_MM[0], CONCAVE_SIGMA_MM[1],
    )
    if x >= FLAT_BLEND_START_MM:
        blend = (x - FLAT_BLEND_START_MM) / (BODY_X_MM - FLAT_BLEND_START_MM)
        height = (1.0 - blend) * height + blend * FLAT_EDGE_Z_MM
    return height


def knots(pole_count, degree):
    unique = pole_count - degree + 1
    span = unique - 1
    values = [index / span for index in range(unique)]
    mults = [degree + 1] + [1] * (unique - 2) + [degree + 1]
    return values, mults


def bspline_face(poles, udegree, vdegree):
    uknots, umults = knots(len(poles), udegree)
    vknots, vmults = knots(len(poles[0]), vdegree)
    surface = Part.BSplineSurface()
    surface.buildFromPolesMultsKnots(
        poles, umults, vmults, uknots, vknots, False, False, udegree, vdegree,
    )
    return Part.Face(surface)


def region_solid(x0, x1):
    xs = [x0 + (x1 - x0) * index / (POLES_U - 1) for index in range(POLES_U)]
    ys = [BODY_Y_MM * index / (POLES_V - 1) for index in range(POLES_V)]
    top = bspline_face(
        [[Vector(x, y, z_top(x, y)) for y in ys] for x in xs],
        3,
        3,
    )
    sides = []
    for edge in top.Edges:
        start = edge.valueAt(edge.FirstParameter)
        end = edge.valueAt(edge.LastParameter)
        bottom = Part.makeLine(Vector(start.x, start.y, 0.0), Vector(end.x, end.y, 0.0))
        sides.append(Part.makeRuledSurface(edge, bottom))
    floor = Part.Face(Part.makePolygon([
        Vector(x0, 0.0, 0.0),
        Vector(x1, 0.0, 0.0),
        Vector(x1, BODY_Y_MM, 0.0),
        Vector(x0, BODY_Y_MM, 0.0),
        Vector(x0, 0.0, 0.0),
    ]))
    return Part.makeSolid(Part.makeShell([top] + sides + [floor]))


def build_shape():
    shape = region_solid(PATCH_EDGES_MM[0], PATCH_EDGES_MM[1])
    for index in range(1, len(PATCH_EDGES_MM) - 1):
        shape = shape.fuse(region_solid(PATCH_EDGES_MM[index], PATCH_EDGES_MM[index + 1]))
    body = shape.Solids[0]
    landmark = Part.makeCylinder(
        LANDMARK_RADIUS_MM,
        LANDMARK_LENGTH_MM,
        Vector(*LANDMARK_AXIS_MM),
        Vector(1.0, 0.0, 0.0),
    )
    return body.fuse(landmark).Solids[0]


def main():
    # pcurve를 STEP에 써야 다시 읽은 B-spline 경계가 BOP 검사를 통과한다.
    Interface_Static.SetIVal("write.surfacecurve.mode", 1)
    shape = build_shape()
    shape.exportStep(os.environ["P011_STEP"])
    box = shape.BoundBox
    print("FACES", len(shape.Faces))
    print("BBOX", box.XLength, box.YLength, box.ZLength)
    print("EXPORTED", os.environ["P011_STEP"])


main()
