#!/usr/bin/env python3
"""P-013 경로 그림. 배경 mesh는 읽기만 한다."""

import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = "/Users/woonkil/projects/cad-less-robotics"
OUT = os.path.join(ROOT, "evidence/P-013")
PATH = os.path.join(OUT, "cad_process_path.json")
MESH = os.path.join(ROOT, "evidence/P-012/mesh.json")
PROCESS_FACES = {"F10", "F11", "F12", "F13"}


def load():
    with open(PATH, encoding="utf-8") as handle:
        return json.load(handle)


def mesh_triangles():
    with open(MESH, encoding="utf-8") as handle:
        payload = json.load(handle)
    triangles = []
    for face in payload["faces"]:
        if face["face_id"] in PROCESS_FACES:
            triangles.extend(face["triangles"])
    return triangles


def axes():
    figure = plt.figure(figsize=(9, 5.2))
    axis = figure.add_subplot(111, projection="3d")
    triangles = mesh_triangles()
    axis.add_collection3d(Poly3DCollection(
        triangles, facecolor="#d7d2c8", edgecolor="none", alpha=0.35
    ))
    axis.set_xlabel("X mm")
    axis.set_ylabel("Y mm")
    axis.set_zlabel("Z mm")
    axis.set_xlim(0, 180)
    axis.set_ylim(0, 100)
    axis.set_zlim(20, 50)
    axis.view_init(elev=28, azim=-58)
    return figure, axis


def anchors(path):
    return [item["surface_anchor"] for item in path["waypoints"]]


def save(figure, name):
    figure.tight_layout()
    figure.savefig(os.path.join(OUT, name), dpi=140)
    plt.close(figure)


def draw_path(path):
    figure, axis = axes()
    points = anchors(path)
    xs, ys, zs = zip(*points)
    axis.plot(xs, ys, zs, color="#1f4e79", linewidth=1.2)
    save(figure, "cad_process_path.png")


def draw_anchors(path):
    figure, axis = axes()
    colors = {"F10": "#4c78a8", "F11": "#f58518", "F12": "#54a24b", "F13": "#e45756"}
    for face_id, color in colors.items():
        points = [
            item["surface_anchor"]
            for item in path["waypoints"]
            if item["face_id"] == face_id
        ]
        xs, ys, zs = zip(*points)
        axis.scatter(xs, ys, zs, s=6, color=color, label=face_id)
    axis.legend(loc="upper left", fontsize=8)
    save(figure, "cad_process_anchor.png")


def unit(vector):
    magnitude = sum(item * item for item in vector) ** 0.5
    return [item / magnitude for item in vector]


def cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def draw_frames(path):
    figure, axis = axes()
    points = anchors(path)
    xs, ys, zs = zip(*points)
    axis.plot(xs, ys, zs, color="#9aa3ad", linewidth=0.6)
    length = 6.0
    colors = ("#c0392b", "#1e8449", "#2471a3")
    for waypoint in path["waypoints"][::24]:
        if waypoint["tool_tangent"] is None:
            continue
        origin = waypoint["surface_anchor"]
        normal = waypoint["surface_normal"]
        z_axis = [-item for item in normal]
        tangent = waypoint["tool_tangent"]
        projected = [
            tangent[index] - sum(tangent[i] * z_axis[i] for i in range(3)) * z_axis[index]
            for index in range(3)
        ]
        x_axis = unit(projected)
        y_axis = unit(cross(z_axis, x_axis))
        for axis_vector, color in zip((x_axis, y_axis, z_axis), colors):
            axis.plot(
                [origin[0], origin[0] + axis_vector[0] * length],
                [origin[1], origin[1] + axis_vector[1] * length],
                [origin[2], origin[2] + axis_vector[2] * length],
                color=color,
                linewidth=1.1,
            )
    save(figure, "cad_tool_frames.png")


def draw_segments(path):
    figure, axis = axes()
    waypoints = path["waypoints"]
    for index in range(1, len(waypoints)):
        left = waypoints[index - 1]["surface_anchor"]
        right = waypoints[index]["surface_anchor"]
        kind = waypoints[index]["segment_type"]
        color = "#1f4e79" if kind == "PROCESS" else "#d35400"
        width = 1.0 if kind == "PROCESS" else 2.2
        axis.plot(
            [left[0], right[0]],
            [left[1], right[1]],
            [left[2], right[2]],
            color=color,
            linewidth=width,
        )
    save(figure, "cad_path_segments.png")


def main():
    path = load()
    draw_path(path)
    draw_anchors(path)
    draw_frames(path)
    draw_segments(path)
    print("RENDERED")


if __name__ == "__main__":
    main()
