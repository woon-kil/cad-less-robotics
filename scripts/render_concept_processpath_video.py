#!/usr/bin/env python3
"""Offline Concept / ProcessPath visualization (presentation only).

Reads frozen P-013 ProcessPath and frozen P-022 simulation datum.
Does NOT reopen gates, recompute paths, run IK, or write evidence/.

VIDEO_ROLE = CONCEPT / PROCESSPATH VISUALIZATION
VALIDATED_ROBOTTRAJECTORY = NO
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

ROOT = Path(__file__).resolve().parents[1]
P013 = ROOT / "evidence/P-013/cad_process_path.json"
P022 = ROOT / "evidence/P-022/robot_planning_input.json"
COUPON_PNG = ROOT / "evidence/P-011/coupon_overview.png"
URDF = ROOT / ".cache/presentation/ur5e/urdf/ur5e.urdf"
STL_DIR = ROOT / ".cache/presentation/ur5e/visual_stl"
OUT_DEFAULT = ROOT / "presentation/concept_processpath"

FPS = 12
DURATION_S = 25
N_FRAMES = FPS * DURATION_S  # 300
BADGE = "ProcessPath 개념 시각화 · 검증된 RobotTrajectory가 아닙니다"

# Display-only static arm pose (not from P-022 planning). Radians.
STATIC_Q = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.2,
    "elbow_joint": 1.4,
    "wrist_1_joint": -1.7,
    "wrist_2_joint": -1.57,
    "wrist_3_joint": 0.0,
}

CHAIN = [
    ("shoulder_pan_joint", "shoulder_link", "shoulder.stl"),
    ("shoulder_lift_joint", "upper_arm_link", "upperarm.stl"),
    ("elbow_joint", "forearm_link", "forearm.stl"),
    ("wrist_1_joint", "wrist_1_link", "wrist1.stl"),
    ("wrist_2_joint", "wrist_2_link", "wrist2.stl"),
    ("wrist_3_joint", "wrist_3_link", "wrist3.stl"),
]


def load_p013() -> dict:
    data = json.loads(P013.read_text())
    wps = data["waypoints"]
    if len(wps) != 741:
        raise SystemExit(f"P-013 waypoint count {len(wps)} != 741")
    return data


def load_p022_datum() -> tuple[np.ndarray, list]:
    data = json.loads(P022.read_text())
    if data.get("robot_trajectory") is not False:
        raise SystemExit("P-022 robot_trajectory must be false (frozen NOT ACHIEVED)")
    t = np.array(data["translation_m"], dtype=float)
    if data.get("rotation") != "identity":
        raise SystemExit(f"unexpected rotation {data.get('rotation')!r}")
    poses = data["poses"]
    if len(poses) != 741:
        raise SystemExit(f"P-022 poses {len(poses)} != 741")
    return t, poses


def quat_to_rot(q: list[float]) -> np.ndarray:
    """quaternion order x,y,z,w (P-013 contract)."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def load_stl_tris(path: Path, max_tris: int = 400) -> np.ndarray:
    """Return (N,3,3) triangle vertices; downsample for Agg speed."""
    raw = path.read_bytes()
    if raw[:5] == b"solid" and b"\x00" not in raw[:80]:
        # ASCII — rare for these packages; skip if huge
        raise SystemExit(f"ASCII STL not supported: {path}")
    n = struct.unpack_from("<I", raw, 80)[0]
    tris = []
    off = 84
    step = max(1, n // max_tris)
    for i in range(0, n, step):
        o = off + i * 50
        # skip normal
        v0 = struct.unpack_from("<fff", raw, o + 12)
        v1 = struct.unpack_from("<fff", raw, o + 24)
        v2 = struct.unpack_from("<fff", raw, o + 36)
        tris.append([v0, v1, v2])
        if len(tris) >= max_tris:
            break
    return np.asarray(tris, dtype=float)


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _origin_xyz_rpy(el) -> tuple[np.ndarray, list[float]]:
    xyz = np.zeros(3)
    rpy = [0.0, 0.0, 0.0]
    if el is None:
        return xyz, rpy
    if el.get("xyz"):
        xyz = np.array([float(v) for v in el.get("xyz").split()])
    if el.get("rpy"):
        rpy = [float(v) for v in el.get("rpy").split()]
    return xyz, rpy


def parse_urdf(urdf_path: Path) -> tuple[dict, dict]:
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        xyz, rpy = _origin_xyz_rpy(j.find("origin"))
        axis = np.array([0.0, 0.0, 1.0])
        ax = j.find("axis")
        if ax is not None and ax.get("xyz"):
            axis = np.array([float(v) for v in ax.get("xyz").split()])
        joints[name] = {
            "type": j.get("type"),
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
        }
    collision_origin: dict[str, tuple[np.ndarray, list[float]]] = {}
    for link in root.findall("link"):
        name = link.get("name")
        coll = link.find("collision")
        if coll is None:
            continue
        collision_origin[name] = _origin_xyz_rpy(coll.find("origin"))
    return joints, collision_origin


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def link_world_transforms(joints: dict) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """FK for static visualization pose. Returns child_link -> (R, t) in base_link."""
    # Start from base_link
    T: dict[str, tuple[np.ndarray, np.ndarray]] = {
        "base_link": (np.eye(3), np.zeros(3)),
    }
    # fixed base_link -> base_link_inertia
    j_in = joints["base_link-base_link_inertia"]
    R0 = rpy_matrix(*j_in["rpy"])
    T["base_link_inertia"] = (R0, j_in["xyz"].copy())

    parent = "base_link"
    for jname, link, _stl in CHAIN:
        j = joints[jname]
        Rp, tp = T[j["parent"]]
        R_origin = rpy_matrix(*j["rpy"])
        R_joint = axis_angle(j["axis"], STATIC_Q[jname])
        R = Rp @ R_origin @ R_joint
        t = tp + Rp @ j["xyz"]
        T[link] = (R, t)
    return T


def transform_tris(
    tris: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    origin_xyz: np.ndarray | None = None,
    origin_rpy: list[float] | None = None,
) -> np.ndarray:
    """Place mesh in link frame then map to world: T_world_link * T_link_mesh."""
    o_xyz = origin_xyz if origin_xyz is not None else np.zeros(3)
    o_rpy = origin_rpy if origin_rpy is not None else [0.0, 0.0, 0.0]
    Rm = rpy_matrix(*o_rpy)
    out = []
    for tri in tris:
        local = (Rm @ tri.T).T + o_xyz
        pts = (R @ local.T).T + t
        out.append(pts)
    return np.asarray(out)


def setup_axes(ax, elev=28, azim=-60, *, hide_cube: bool = False):
    ax.view_init(elev=elev, azim=azim)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.set_facecolor("#f7f4ee")
    for spine in ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane:
        spine.set_facecolor("#f7f4ee")
        spine.set_alpha(0.0 if hide_cube else 1.0)
        spine.set_edgecolor("#f7f4ee" if hide_cube else "#d9d3c7")
    if hide_cube:
        ax.xaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
        ax.yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
        ax.zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
        ax.grid(False)


def equal_aspect(ax, pts: np.ndarray, pad=0.15, zoom: float = 1.0):
    """Cubic fit (Scene D robot context). zoom>1 crops closer."""
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    c = (mins + maxs) / 2
    r = max((maxs - mins).max() * 0.55, 1e-3) * (1 + pad) / max(zoom, 1e-6)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)


def fit_surface_view(
    ax,
    pts: np.ndarray,
    *,
    zoom: float = 1.6,
    margin: float = 0.06,
    z_visual: float = 3.2,
) -> None:
    """Tight non-cubic framing for flat CAD coupons (presentation only).

    Uses data XY extents + mild Z box-aspect emphasis so the surface fills the
    viewport instead of floating as a thin sheet inside an empty cube.
    Coordinates remain frozen mm; only matplotlib box aspect changes.
    """
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    spans = np.maximum(maxs - mins, 1e-3)
    c = (mins + maxs) / 2.0
    m = margin / max(zoom, 1e-6)
    half = spans * (0.5 + m)
    # Extra XY crop via zoom
    half[:2] /= max(zoom, 1e-6)
    half[2] = max(half[2], spans[0] * 0.04)
    ax.set_xlim(c[0] - half[0], c[0] + half[0])
    ax.set_ylim(c[1] - half[1], c[1] + half[1])
    ax.set_zlim(c[2] - half[2], c[2] + half[2])
    bx, by, bz = float(spans[0]), float(spans[1]), float(max(spans[2] * z_visual, spans[0] * 0.22))
    ax.set_box_aspect((bx, by, bz))


def make_safe_3d_axes(fig):
    """Title / 3D / footer subplot rows (layout); PIL burn seals text-safe area."""
    import matplotlib.gridspec as gridspec

    gs = gridspec.GridSpec(
        3,
        1,
        figure=fig,
        height_ratios=[0.14, 0.72, 0.14],
        hspace=0.0,
        left=0.02,
        right=0.98,
        top=0.98,
        bottom=0.02,
    )
    ax_top = fig.add_subplot(gs[0])
    ax = fig.add_subplot(gs[1], projection="3d")
    ax_bot = fig.add_subplot(gs[2])
    for band in (ax_top, ax_bot):
        band.set_axis_off()
        band.set_facecolor("#f7f4ee")
        band.set_xlim(0, 1)
        band.set_ylim(0, 1)
    return ax_top, ax, ax_bot


def fill_axes(fig, ax) -> None:
    """No-op kept for call-site compatibility; layout comes from make_safe_3d_axes."""
    del fig, ax


def draw_text_safe_bands(fig) -> None:
    """No-op: safe bands sealed by burn_text_safe_area after savefig."""
    del fig


def _pil_font(size: int, *, mono: bool = False):
    from PIL import ImageFont

    candidates = (
        (
            "/System/Library/Fonts/Supplemental/Courier New.ttf",
            "/System/Library/Fonts/Menlo.ttc",
        )
        if mono
        else (
            "/System/Library/Fonts/AppleSDGothicNeo.ttc",
            "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
            "/Library/Fonts/Arial Unicode.ttf",
            "/System/Library/Fonts/Helvetica.ttc",
        )
    )
    for path in candidates:
        try:
            return ImageFont.truetype(path, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


def burn_text_safe_area(
    png_path: Path,
    *,
    title: str = "",
    subtitle: str = "",
    meta: str = "",
    meta_color: tuple[int, int, int] = (63, 58, 50),
    badge: str = BADGE,
) -> None:
    """Final presentation seal: opaque title/footer bands + text on top of PNG.

    matplotlib 3D can spill past Axes; this post-pass guarantees claim/UI text
    is never covered by mesh (TEXT_SAFE_AREA_ONLY repair).
    """
    from PIL import Image, ImageDraw

    im = Image.open(png_path).convert("RGB")
    w, h = im.size
    top_h = int(round(h * 0.14))
    bot_h = int(round(h * 0.14))
    cream = (247, 244, 238)
    draw = ImageDraw.Draw(im)
    draw.rectangle([0, 0, w, top_h], fill=cream)
    draw.rectangle([0, h - bot_h, w, h], fill=cream)

    font_title = _pil_font(28)
    font_sub = _pil_font(20, mono=True)
    font_meta = _pil_font(18, mono=True)
    font_badge = _pil_font(20)

    if title:
        draw.text((36, int(top_h * 0.22)), title, fill=(28, 25, 21), font=font_title)
    if subtitle:
        draw.text((36, int(top_h * 0.58)), subtitle, fill=(92, 86, 76), font=font_sub)
    if meta:
        draw.text((36, h - bot_h + int(bot_h * 0.18)), meta, fill=meta_color, font=font_meta)
    # badge centered
    bw = draw.textlength(badge, font=font_badge) if hasattr(draw, "textlength") else font_badge.getlength(badge)
    draw.text(
        ((w - bw) / 2.0, h - bot_h + int(bot_h * 0.55)),
        badge,
        fill=(42, 38, 31),
        font=font_badge,
    )
    im.save(png_path)


def split_anchor_passes(anchors: np.ndarray, y_jump_mm: float = 1.0) -> list[np.ndarray]:
    """Group frozen serpentine anchors into Y-passes (presentation mesh only)."""
    dy = np.diff(anchors[:, 1])
    cuts = np.where(np.abs(dy) > y_jump_mm)[0] + 1
    starts = np.concatenate(([0], cuts))
    ends = np.concatenate((cuts, [len(anchors)]))
    return [anchors[s:e] for s, e in zip(starts, ends)]


def surface_tris_from_passes(passes: list[np.ndarray]) -> np.ndarray:
    """Triangle strip between adjacent frozen passes — no new geometry samples."""
    tris: list[list[np.ndarray]] = []
    for a, b in zip(passes[:-1], passes[1:]):
        n = min(len(a), len(b))
        if n < 2:
            continue
        # Align serpentine direction so adjacent samples share similar X.
        if abs(float(a[0, 0] - b[0, 0])) > abs(float(a[0, 0] - b[-1, 0])):
            b = b[::-1]
        for i in range(n - 1):
            p00, p01 = a[i], a[i + 1]
            p10, p11 = b[i], b[i + 1]
            tris.append([p00, p01, p11])
            tris.append([p00, p11, p10])
    if not tris:
        return np.zeros((0, 3, 3), dtype=float)
    return np.asarray(tris, dtype=float)


def draw_reference_surface(ax, anchors: np.ndarray, *, wire: bool = True, face_alpha: float = 0.72) -> None:
    """Visual-only surface from frozen surface_anchor samples."""
    passes = split_anchor_passes(anchors)
    tris = surface_tris_from_passes(passes)
    if len(tris):
        ax.add_collection3d(
            Poly3DCollection(
                tris,
                facecolors="#4f5d6c",
                edgecolors="#1f2933" if wire else "none",
                linewidths=0.35 if wire else 0.0,
                alpha=face_alpha,
            )
        )
    # Sample points on top of mesh (readability, not new samples)
    step = max(1, len(anchors) // 180)
    ax.scatter(
        anchors[::step, 0],
        anchors[::step, 1],
        anchors[::step, 2],
        s=14,
        c="#0c0a09",
        alpha=0.7,
        depthshade=False,
    )


def _font():
    from matplotlib import font_manager

    for name in ("AppleGothic", "Apple SD Gothic Neo", "NanumGothic", "Malgun Gothic", "DejaVu Sans"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            return name
        except Exception:
            continue
    return "DejaVu Sans"


FONT = _font()


def draw_badge(fig, ax_bot=None):
    if ax_bot is not None:
        ax_bot.text(
            0.5,
            0.28,
            BADGE,
            ha="center",
            va="center",
            fontsize=11,
            fontweight="600",
            color="#2a261f",
            fontfamily=FONT,
            transform=ax_bot.transAxes,
        )
        return
    fig.text(
        0.5,
        0.018,
        BADGE,
        ha="center",
        va="bottom",
        fontsize=11,
        fontweight="600",
        color="#2a261f",
        fontfamily=FONT,
        zorder=30,
    )


def draw_title(fig, title: str, subtitle: str = "", ax_top=None):
    if ax_top is not None:
        ax_top.text(
            0.02,
            0.72,
            title,
            ha="left",
            va="center",
            fontsize=13,
            fontweight="600",
            color="#1c1915",
            fontfamily=FONT,
            transform=ax_top.transAxes,
        )
        if subtitle:
            ax_top.text(
                0.02,
                0.28,
                subtitle,
                ha="left",
                va="center",
                fontsize=10,
                color="#5c564c",
                fontfamily="monospace",
                transform=ax_top.transAxes,
            )
        return
    fig.text(
        0.03,
        0.96,
        title,
        ha="left",
        va="top",
        fontsize=13,
        fontweight="600",
        color="#1c1915",
        fontfamily=FONT,
        zorder=30,
    )
    if subtitle:
        fig.text(
            0.03,
            0.915,
            subtitle,
            ha="left",
            va="top",
            fontsize=10,
            color="#5c564c",
            fontfamily="monospace",
            zorder=30,
        )


def draw_footer_meta(fig, text: str, *, color: str = "#3f3a32", ax_bot=None) -> None:
    """Scene metadata in the reserved bottom band (above persistent badge)."""
    if ax_bot is not None:
        ax_bot.text(
            0.02,
            0.72,
            text,
            ha="left",
            va="center",
            fontsize=9,
            fontfamily="monospace",
            color=color,
            transform=ax_bot.transAxes,
        )
        return
    fig.text(
        0.03,
        0.055,
        text,
        ha="left",
        va="bottom",
        fontsize=9,
        fontfamily="monospace",
        color=color,
        zorder=30,
    )


def draw_tcp_frame(ax, origin, R, scale_mm=12.0, lw=1.6):
    colors = ("#b91c1c", "#15803d", "#1d4ed8")  # X Y Z
    for i, c in enumerate(colors):
        end = origin + R[:, i] * scale_mm
        ax.plot([origin[0], end[0]], [origin[1], end[1]], [origin[2], end[2]], color=c, lw=lw)


def scene_time(frame: int) -> str:
    t = frame / FPS
    if t < 4:
        return "A"
    if t < 11:
        return "B"
    if t < 17:
        return "C"
    if t < 21:
        return "D"
    return "E"


# Representative still frames for presentation repair review (seconds → frame index).
STILL_SPECS = (
    ("A_geometry.png", 2.0),
    ("B_processpath.png", 10.0),
    ("C_tcp_6d.png", 14.0),
    ("D_static_ur5e.png", 19.0),
    ("E_boundary.png", 23.0),
)

# TEXT_SAFE_AREA gate — only these four stills before final MP4.
SAFE_AREA_STILL_SPECS = (
    ("A_t1s_safe.png", 1.0),
    ("C_t12s_safe.png", 12.0),
    ("C_t14s_safe.png", 14.0),
    ("C_t16s_safe.png", 16.0),
)


def render(out_dir: Path, *, stills_only: bool = False, still_specs: tuple | None = None) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = out_dir / "frames"
    stills_dir = out_dir / "review_stills"
    if not stills_only:
        frames_dir.mkdir(exist_ok=True)
    stills_dir.mkdir(exist_ok=True)

    p013 = load_p013()
    wps = p013["waypoints"]
    anchors = np.array([w["surface_anchor"] for w in wps], dtype=float)
    commands = np.array([w["command_position"] for w in wps], dtype=float)
    quats = [w["orientation_quaternion"] for w in wps]
    normals = np.array([w["surface_normal"] for w in wps], dtype=float)

    translation_m, poses = load_p022_datum()
    cmd_m = np.array([p["command_position_m"] for p in poses], dtype=float)
    anc_m = np.array([p["surface_anchor_m"] for p in poses], dtype=float)

    joints, coll_origin = parse_urdf(URDF)
    Tw = link_world_transforms(joints)
    base_tris = load_stl_tris(STL_DIR / "base.stl", 250)
    link_meshes = {}
    for jname, link, stl in CHAIN:
        link_meshes[link] = load_stl_tris(STL_DIR / stl, 280)

    # provenance
    prov = {
        "VIDEO_ROLE": "CONCEPT / PROCESSPATH VISUALIZATION",
        "VALIDATED_ROBOTTRAJECTORY": "NO",
        "PROCESSPATH_SOURCE": str(P013.relative_to(ROOT)),
        "SIMULATION_DATUM_SOURCE": str(P022.relative_to(ROOT)),
        "translation_m": translation_m.tolist(),
        "rotation": "identity",
        "UR5E_MODEL": "ur_description 3.5.1 collision STL + xacro-generated URDF (cache only)",
        "ARTICULATED_ROBOT_EXECUTION": "NO",
        "STATIC_Q_DISPLAY_ONLY": STATIC_Q,
        "waypoint_count": 741,
        "PRESENTATION_REPAIR": "camera/zoom/surface-alpha/line-width only; frozen data untouched",
        "github": "https://github.com/woon-kil/cad-less-robotics",
    }
    (out_dir / "provenance.json").write_text(json.dumps(prov, indent=2, ensure_ascii=False) + "\n")

    specs = still_specs if still_specs is not None else STILL_SPECS
    if stills_only:
        frame_list = [(int(round(sec * FPS)), name) for name, sec in specs]
    else:
        frame_list = [(fi, None) for fi in range(N_FRAMES)]

    for fi, still_name in frame_list:
        t = fi / FPS
        scene = scene_time(fi)
        fig = plt.figure(figsize=(10.0, 6.25), dpi=120, facecolor="#f7f4ee")

        if scene == "E":
            ax = fig.add_axes([0.08, 0.12, 0.84, 0.78])
            ax.set_axis_off()
            ax.set_facecolor("#f7f4ee")
            lines = [
                "Verified Engineering Boundary",
                "",
                "CAD Geometry          PASS",
                "6D ProcessPath        PASS",
                "Robot Adapter         PASS",
                "MoveIt Integration    EXECUTED",
                "",
                "P-022                 CLOSED_FAIL",
                "RobotTrajectory       NOT ACHIEVED",
            ]
            ax.text(0.05, 0.92, "\n".join(lines), va="top", ha="left", fontsize=14, fontfamily="monospace", color="#1c1915")
            draw_title(fig, "Engineering boundary", "presentation · not a planning result")
            draw_badge(fig)
            if still_name:
                fig.savefig(stills_dir / still_name)
                print("still", still_name, f"t={t:.1f}s")
            else:
                fig.savefig(frames_dir / f"frame_{fi:04d}.png")
            plt.close(fig)
            continue

        ax_top, ax, ax_bot = make_safe_3d_axes(fig)
        setup_axes(ax)
        burn_title = ""
        burn_subtitle = ""
        burn_meta = ""
        burn_meta_color = (63, 58, 50)

        if scene in ("A", "B", "C"):
            # CAD_MODEL_FRAME mm — frozen anchors only; presentation mesh/wire/points
            if scene == "A":
                draw_reference_surface(ax, anchors, wire=True, face_alpha=0.82)
                # Higher elev so the flat coupon fills the frame (presentation).
                setup_axes(ax, elev=58, azim=-40, hide_cube=True)
                fit_surface_view(ax, anchors, zoom=1.35, margin=0.02, z_visual=3.6)
                burn_title = "Complex Surface CAD Coupon"
                burn_subtitle = "REFERENCE_GEOMETRY"
                burn_meta = "GROUND_TRUTH = NONE"
                burn_meta_color = (140, 47, 47)

            if scene == "B":
                draw_reference_surface(ax, anchors, wire=True, face_alpha=0.58)
                # reveal path progressively 4–11s
                frac = (t - 4) / 7.0
                n_show = max(2, int(741 * min(1.0, max(0.0, frac))))
                segs = np.stack([commands[: n_show - 1], commands[1:n_show]], axis=1)
                ax.add_collection3d(Line3DCollection(segs, colors="#0a3d57", linewidths=2.8))
                # sparse orientation frames
                for idx in range(0, n_show, 40):
                    R = quat_to_rot(quats[idx])
                    draw_tcp_frame(ax, commands[idx], R, scale_mm=18, lw=2.4)
                setup_axes(ax, elev=52, azim=-40, hide_cube=True)
                fit_surface_view(
                    ax,
                    np.vstack([anchors, commands[:n_show]]),
                    zoom=1.3,
                    margin=0.02,
                    z_visual=3.6,
                )
                burn_title = "Robot-independent 6D ProcessPath"
                burn_subtitle = "741-waypoint · CAD_MODEL_FRAME · mm"

            if scene == "C":
                # local surface patch from frozen anchors near TCP
                frac = (t - 11) / 6.0
                idx = int(min(740, max(0, frac * 740)))
                trail = max(0, idx - 80)
                # Full frozen surface under local camera zoom (no resample).
                draw_reference_surface(ax, anchors, wire=True, face_alpha=0.68)
                segs = np.stack([commands[trail:idx], commands[trail + 1 : idx + 1]], axis=1) if idx > trail else []
                if len(segs):
                    ax.add_collection3d(Line3DCollection(segs, colors="#0a3d57", linewidths=3.0))
                # standoff segment
                a, c = anchors[idx], commands[idx]
                ax.plot([a[0], c[0]], [a[1], c[1]], [a[2], c[2]], color="#b45309", lw=3.4)
                ax.scatter([a[0]], [a[1]], [a[2]], c="#0c0a09", s=90, zorder=5)
                ax.scatter([c[0]], [c[1]], [c[2]], c="#0a3d57", s=100, zorder=5)
                R = quat_to_rot(quats[idx])
                draw_tcp_frame(ax, c, R, scale_mm=26, lw=3.2)
                # Local crop around TCP (~2× tighter than first draft r=45)
                center = (a + c) / 2.0
                r_xy = 18.0
                ax.set_xlim(center[0] - r_xy, center[0] + r_xy)
                ax.set_ylim(center[1] - r_xy, center[1] + r_xy)
                ax.set_zlim(center[2] - 8.0, center[2] + 26.0)
                ax.set_box_aspect((1.0, 1.0, 0.9))
                setup_axes(ax, elev=42, azim=-35, hide_cube=True)
                burn_title = "TCP meaning on ProcessPath"
                burn_subtitle = "surface_anchor → standoff → command_position → orientation"
                burn_meta = f"sequence_id={idx}  process_offset_mm=20.0 (frozen)"

        if scene == "D":
            # path in base_link meters — coupon emphasis
            segs = np.stack([cmd_m[:-1], cmd_m[1:]], axis=1)
            ax.add_collection3d(Line3DCollection(segs, colors="#0f4c6b", linewidths=1.8, alpha=0.95))
            ax.scatter(anc_m[::4, 0], anc_m[::4, 1], anc_m[::4, 2], s=6, c="#3f4a55", alpha=0.7)

            # static robot meshes (collision STL + URDF collision origin) — higher contrast
            Rb, tb = Tw["base_link_inertia"]
            ox, orpy = coll_origin.get("base_link_inertia", (np.zeros(3), [0.0, 0.0, math.pi]))
            tris_b = transform_tris(base_tris, Rb, tb, ox, orpy)
            ax.add_collection3d(
                Poly3DCollection(
                    tris_b,
                    facecolors="#8a8680",
                    edgecolors="#4a4640",
                    linewidths=0.15,
                    alpha=0.72,
                )
            )
            for _j, link, _s in CHAIN:
                R, tvec = Tw[link]
                ox, orpy = coll_origin.get(link, (np.zeros(3), [0.0, 0.0, 0.0]))
                tris = transform_tris(link_meshes[link], R, tvec, ox, orpy)
                ax.add_collection3d(
                    Poly3DCollection(
                        tris,
                        facecolors="#6f6a64",
                        edgecolors="#3a3632",
                        linewidths=0.12,
                        alpha=0.68,
                    )
                )

            pts = np.vstack([cmd_m, anc_m, np.array([tvec for _, tvec in Tw.values()])])
            setup_axes(ax, elev=24, azim=-55)
            equal_aspect(ax, pts, pad=0.08, zoom=1.35)
            burn_title = "ProcessPath → Robot Adapter → UR5e Planning Context"
            burn_subtitle = "STATIC ur_description 3.5.1 · CONFIGURED_SIMULATION_DATUM · no RobotTrajectory"
            burn_meta = "ARTICULATED_EXECUTION=NO · ROBOTTRAJECTORY_PLAYBACK=NO"
            burn_meta_color = (140, 47, 47)

        # Matplotlib labels optional; PIL burn is the authoritative safe-area text.
        draw_title(fig, burn_title, burn_subtitle, ax_top=ax_top)
        if burn_meta:
            draw_footer_meta(fig, burn_meta, color="#%02x%02x%02x" % burn_meta_color, ax_bot=ax_bot)
        draw_badge(fig, ax_bot=ax_bot)
        if still_name:
            out_png = stills_dir / still_name
            fig.savefig(out_png)
            burn_text_safe_area(
                out_png,
                title=burn_title,
                subtitle=burn_subtitle,
                meta=burn_meta,
                meta_color=burn_meta_color,
            )
            print("still", still_name, f"t={t:.1f}s scene={scene}")
        else:
            out_png = frames_dir / f"frame_{fi:04d}.png"
            fig.savefig(out_png)
            burn_text_safe_area(
                out_png,
                title=burn_title,
                subtitle=burn_subtitle,
                meta=burn_meta,
                meta_color=burn_meta_color,
            )
            if fi % 30 == 0:
                print(f"frame {fi}/{N_FRAMES} scene={scene}")
        plt.close(fig)

    if stills_only:
        # Prefer C@14s safe-area gate still; fall back to general review still.
        for candidate in ("C_t14s_safe.png", "C_tcp_6d.png"):
            poster_src = stills_dir / candidate
            if poster_src.exists():
                poster_dst = out_dir / "concept-processpath-poster.png"
                poster_dst.write_bytes(poster_src.read_bytes())
                print("wrote", poster_dst, "from", candidate)
                break
        print("wrote stills →", stills_dir)
        return

    # poster = frame at ~14s (TCP highlight)
    poster_src = frames_dir / f"frame_{14 * FPS:04d}.png"
    poster_dst = out_dir / "concept-processpath-poster.png"
    poster_dst.write_bytes(poster_src.read_bytes())
    print("wrote", poster_dst)


def encode(out_dir: Path) -> None:
    import subprocess

    frames = out_dir / "frames" / "frame_%04d.png"
    mp4 = out_dir / "concept-processpath.mp4"
    webm = out_dir / "concept-processpath.webm"
    subprocess.check_call(
        [
            "ffmpeg",
            "-y",
            "-framerate",
            str(FPS),
            "-i",
            str(frames),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "20",
            str(mp4),
        ]
    )
    subprocess.check_call(
        [
            "ffmpeg",
            "-y",
            "-framerate",
            str(FPS),
            "-i",
            str(frames),
            "-pix_fmt",
            "yuv420p",
            "-c:v",
            "libvpx-vp9",
            "-b:v",
            "0",
            "-crf",
            "32",
            "-an",
            str(webm),
        ]
    )
    print("wrote", mp4, webm)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--encode-only", action="store_true")
    ap.add_argument("--skip-encode", action="store_true")
    ap.add_argument(
        "--stills-only",
        action="store_true",
        help="Render review_stills A–E only (presentation repair gate; no MP4)",
    )
    ap.add_argument(
        "--stills-safe-area",
        action="store_true",
        help="TEXT_SAFE_AREA gate: A@1s + C@12/14/16s only (no MP4)",
    )
    args = ap.parse_args()
    if args.stills_safe_area:
        render(args.out, stills_only=True, still_specs=SAFE_AREA_STILL_SPECS)
        return
    if args.stills_only:
        render(args.out, stills_only=True)
        return
    if not args.encode_only:
        render(args.out)
    if not args.skip_encode:
        encode(args.out)


if __name__ == "__main__":
    main()
