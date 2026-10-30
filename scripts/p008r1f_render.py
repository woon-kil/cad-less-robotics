# P-008-R1F 그림. capture_01 depth와 연결 진단 산출만 사용한다.

import csv
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def load_cloud(path):
    file_path = Path(path)
    if not file_path.exists():
        return np.zeros((0, 3), dtype=np.float32)
    raw = file_path.read_bytes()
    count = int(np.frombuffer(raw, dtype="<u4", count=1)[0])
    if count == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return np.frombuffer(raw, dtype="<f4", count=count * 3, offset=4).reshape(count, 3)


def read_sweep(path):
    with Path(path).open(newline="") as handle:
        return list(csv.DictReader(handle))


def chart(path, title, xs, series):
    width, height = 960, 540
    image = Image.new("RGB", (width, height), (16, 18, 22))
    draw = ImageDraw.Draw(image)
    left, right, top, bottom = 70, 930, 40, 470
    draw.rectangle((left, top, right, bottom), outline=(80, 80, 80))
    ymax = max(max(item[0]) for item in series.values())
    ymax = ymax if ymax > 0 else 1.0
    for name, values, color in (
        (key, series[key][0], series[key][1]) for key in series
    ):
        coords = []
        for x_value, y_value in zip(xs, values):
            px = left + (x_value - xs[0]) / (xs[-1] - xs[0]) * (right - left)
            py = bottom - (y_value / ymax) * (bottom - top)
            coords.append((px, py))
        draw.line(coords, fill=color, width=2)
        draw.text((left, 8), title, fill=(255, 255, 255))
        draw.text((left, 500), name, fill=color)
    for index, x_value in enumerate(xs):
        px = left + (x_value - xs[0]) / (xs[-1] - xs[0]) * (right - left)
        draw.text((px - 12, bottom + 4), f"{x_value * 1000:.0f}", fill=(180, 180, 180))
    image.save(path)


def main():
    out_dir = Path(sys.argv[1])
    depth_path = Path(sys.argv[2])
    fx = fy = 410.4136657714844
    cx = 423.66876220703125
    cy = 240.9937744140625
    depth = np.array(Image.open(depth_path))
    view = depth.astype(np.float32)
    low, high = np.percentile(view[view > 0], [2, 98])
    image = np.clip((view - low) / (high - low), 0, 1)
    image[view == 0] = 0
    base = (np.dstack([image, image, image]) * 255).astype(np.uint8)

    def project(points):
        xyz = np.asarray(points, dtype=np.float64)
        return np.rint(fx * xyz[:, 0] / xyz[:, 2] + cx).astype(int), np.rint(
            fy * xyz[:, 1] / xyz[:, 2] + cy
        ).astype(int)

    def paint(canvas, points, color, step):
        if len(points) == 0:
            return
        pixels = canvas.load()
        xs, ys = project(points)
        for x_coord, y_coord in zip(xs[::step], ys[::step]):
            if 0 <= x_coord < 848 and 0 <= y_coord < 480:
                pixels[int(x_coord), int(y_coord)] = color

    def save(name, draw_calls):
        canvas = Image.fromarray(base.copy(), "RGB")
        for points, color, step in draw_calls:
            paint(canvas, points, color, step)
        draw = ImageDraw.Draw(canvas)
        draw.text((8, 460), name, fill=(255, 255, 255))
        canvas.save(out_dir / "figures" / f"{name}.png")

    selected = load_cloud(out_dir / "stage_selected.bin")
    seed = load_cloud(out_dir / "stage_seed.bin")
    tracked = load_cloud(out_dir / "stage_tracked_10mm.bin")
    shoe_side = load_cloud(out_dir / "stage_bridge_shoe.bin")
    env_side = load_cloud(out_dir / "stage_bridge_env.bin")
    path = load_cloud(out_dir / "stage_path.bin")
    figure_dir = out_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    save("01_prism_cloud", [(selected, (40, 210, 80), 3), (seed, (255, 220, 0), 1)])
    save("02_10mm_shoe_component", [(tracked, (255, 255, 255), 2), (seed, (255, 220, 0), 1)])
    endpoints = path[-2:] if path.shape[0] >= 2 else path
    save(
        "03_shoe_environment_bridge",
        [(shoe_side, (80, 160, 255), 2), (env_side, (220, 60, 60), 2), (endpoints, (255, 220, 0), 1)],
    )
    save(
        "04_connectivity_path",
        [(shoe_side, (30, 60, 90), 4), (env_side, (90, 30, 30), 4), (path, (255, 255, 255), 1)],
    )
    rows = read_sweep(out_dir / "tolerance_sweep.csv")
    xs = [float(row["tolerance_m"]) for row in rows]
    chart(
        figure_dir / "05_tolerance_sweep.png",
        "component count",
        xs,
        {"components": ([float(row["component_count"]) for row in rows], (80, 180, 255))},
    )
    chart(
        figure_dir / "06_component_size_vs_tolerance.png",
        "tracked component points",
        xs,
        {"points": ([float(row["tracked_component_points"]) for row in rows], (80, 220, 120))},
    )
    chart(
        figure_dir / "07_shoe_membership_vs_tolerance.png",
        "shoe seed membership",
        xs,
        {"membership": ([float(row["shoe_membership"]) for row in rows], (255, 200, 60))},
    )
    chart(
        figure_dir / "08_fragmentation_diagnostics.png",
        "seed component count",
        xs,
        {"fragments": ([float(row["shoe_fragment_count"]) for row in rows], (255, 100, 80))},
    )
    normal = json.loads((out_dir / "normal_suitability.json").read_text())
    if normal["reason"] == "MEASURED" or normal["reason"].startswith("ANGLE"):
        save("09_contact_normal_diagnostics", [(seed, (255, 220, 0), 2), (endpoints, (255, 40, 40), 1)])


if __name__ == "__main__":
    main()
