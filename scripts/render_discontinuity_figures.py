#!/usr/bin/env python3
"""Attempt #3 진단 그림. 법선이나 경로를 바꾸지 않는다."""

import csv
import math
from pathlib import Path


def load(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def vec(row, keys):
    return tuple(float(row[key]) for key in keys)


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(value):
    return math.sqrt(dot(value, value))


def unit(value):
    length = norm(value)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return tuple(component / length for component in value)


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def project(point, origin, axis_u, axis_v):
    delta = sub(point, origin)
    return (dot(delta, axis_u), dot(delta, axis_v))


def render(root, index):
    name = f"{index:03d}"
    waypoints = load(root / f"index-{name}-neighborhood.csv")
    surface = [
        row for row in load(root / f"index-{name}-surface.csv") if row["set"] == "viz_radius_0_02"
    ]
    focus = next(row for row in waypoints if int(row["index"]) == index)
    origin = vec(focus, ("x", "y", "z"))
    tangent = unit(vec(focus, ("tx", "ty", "tz")))
    normal = unit(vec(focus, ("nx", "ny", "nz")))
    side = unit(cross(tangent, normal))
    if norm(side) < 0.5:
        side = unit(cross(tangent, (0.0, 0.0, 1.0)))
    up = unit(cross(side, tangent))
    frames = ((tangent, up, "경로 × 법선"), (tangent, side, "경로 × 측면"))

    width = 760
    height = 390
    panel = 360
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white"/>',
    ]
    for panel_index, (axis_u, axis_v, title) in enumerate(frames):
        left = 20 + panel_index * 380
        cloud = [project(vec(row, ("x", "y", "z")), origin, axis_u, axis_v) for row in surface]
        path = [project(vec(row, ("x", "y", "z")), origin, axis_u, axis_v) for row in waypoints]
        points = cloud + path
        min_x = min(point[0] for point in points)
        max_x = max(point[0] for point in points)
        min_y = min(point[1] for point in points)
        max_y = max(point[1] for point in points)
        span = max(max_x - min_x, max_y - min_y, 1e-6)
        scale = (panel - 40) / span

        def place(point):
            return (
                left + 20 + (point[0] - min_x) * scale,
                40 + (max_y - point[1]) * scale,
            )

        parts.append(
            f'<text x="{left}" y="24" font-family="sans-serif" font-size="14">{title} index {index}</text>'
        )
        for point in cloud:
            x, y = place(point)
            parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="1.3" fill="#9aa0a6"/>')
        mapped = [place(point) for point in path]
        poly = " ".join(f"{x:.2f},{y:.2f}" for x, y in mapped)
        parts.append(f'<polyline points="{poly}" fill="none" stroke="#1a73e8" stroke-width="1.4"/>')
        for row, (x, y) in zip(waypoints, mapped):
            waypoint_index = int(row["index"])
            color = "#d93025" if waypoint_index in (index, index + 1) else "#1a73e8"
            parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2.4" fill="{color}"/>')
            direction = project(unit(vec(row, ("nx", "ny", "nz"))), (0, 0, 0), axis_u, axis_v)
            parts.append(
                f'<line x1="{x:.2f}" y1="{y:.2f}" x2="{x + direction[0] * 18:.2f}" '
                f'y2="{y - direction[1] * 18:.2f}" stroke="#188038" stroke-width="1.2"/>'
            )
        camera = project(unit((-origin[0], -origin[1], -origin[2])), (0, 0, 0), axis_u, axis_v)
        cx, cy = place((0.0, 0.0))
        parts.append(
            f'<line x1="{cx:.2f}" y1="{cy:.2f}" x2="{cx + camera[0] * 28:.2f}" '
            f'y2="{cy - camera[1] * 28:.2f}" stroke="#9334e6" stroke-width="1.4" stroke-dasharray="3 2"/>'
        )
    parts.append("</svg>\n")
    (root / f"normal-discontinuity-{name}.svg").write_text("\n".join(parts))


def main():
    root = Path(__file__).resolve().parents[1] / "evidence" / "P-004" / "attempt-03"
    for index in (43, 44, 311):
        render(root, index)


if __name__ == "__main__":
    main()
