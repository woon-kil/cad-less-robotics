#!/usr/bin/env python3
"""채택 944관절의 표시용 분사 데모 모델을 만든다.

재탐색, TOTG, 도포량 계산, 충돌 검증은 하지 않는다.
기존 manifest와 P-022는 읽기만 한다.
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import socket
import struct
import subprocess
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RESTORE = ROOT / "evidence/UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH-RESTORE/restore.json"
URDF = ROOT / "reproduction/inputs/ur5e.urdf"
MESH = ROOT / "evidence/UR-ANALYTIC-IK-PROCESS-COLLISION-COUPON-R2/mesh.bin"
MESH_SHA = "ea78058ca33866ef3c69764be48f90da4a6f2491455c9077c514ccecd6acbc16"
OUT_DIR = ROOT / "evidence/UR-ANALYTIC-IK-SPRAY-DEMO"
VIEWER = OUT_DIR / "viewer/index.html"
BASELINE = "fdfa9c9"
RESTORE_SHA = "35f72af9e3b23b9cb2a4e6133eb24623489ed00762f8ccf40000e618a0998943"
URDF_SHA = "a21bf5fb70b3a1745bf2e4816e0f43654539737ba7f2585939ec773d159778d8"
DATUM = np.array([0.3244333936553, -0.0499999998932, 0.11133404082435])
DT_S = 0.04
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
CHAIN = [
    ("base_link", None),
    ("base_link_inertia", "base_link-base_link_inertia"),
    ("shoulder_link", "shoulder_pan_joint"),
    ("upper_arm_link", "shoulder_lift_joint"),
    ("forearm_link", "elbow_joint"),
    ("wrist_1_link", "wrist_1_joint"),
    ("wrist_2_link", "wrist_2_joint"),
    ("wrist_3_link", "wrist_3_joint"),
    ("flange", "wrist_3-flange"),
    ("tool0", "flange-tool0"),
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    one = 1.0 - c
    return np.array(
        [
            [c + x * x * one, x * y * one - z * s, x * z * one + y * s],
            [y * x * one + z * s, c + y * y * one, y * z * one - x * s],
            [z * x * one - y * s, z * y * one + x * s, c + z * z * one],
        ]
    )


def parse_urdf(path: Path) -> dict:
    joints = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        origin = joint.find("origin")
        xyz = np.zeros(3)
        rpy = [0.0, 0.0, 0.0]
        if origin is not None and origin.get("xyz"):
            xyz = np.array([float(v) for v in origin.get("xyz").split()])
        if origin is not None and origin.get("rpy"):
            rpy = [float(v) for v in origin.get("rpy").split()]
        axis = np.array([0.0, 0.0, 1.0])
        axis_el = joint.find("axis")
        if axis_el is not None and axis_el.get("xyz"):
            axis = np.array([float(v) for v in axis_el.get("xyz").split()])
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
        }
    return joints


def fk(joints: dict, q: np.ndarray) -> tuple[list[list[float]], list[float], list[float]]:
    values = {name: float(q[index]) for index, name in enumerate(JOINTS)}
    frames: dict[str, tuple[np.ndarray, np.ndarray]] = {"base_link": (np.eye(3), np.zeros(3))}
    points = [[0.0, 0.0, 0.0]]
    for link, joint_name in CHAIN[1:]:
        joint = joints[joint_name]
        parent_r, parent_t = frames[list(frames)[-1]]
        rotation = parent_r @ rpy_matrix(*joint["rpy"])
        if joint["type"] == "revolute":
            rotation = rotation @ axis_angle(joint["axis"], values[joint_name])
        translation = parent_t + parent_r @ joint["xyz"]
        frames[link] = (rotation, translation)
        points.append([float(v) for v in translation])
    tool_r, tool_t = frames["tool0"]
    axis = tool_r[:, 2]
    axis = axis / np.linalg.norm(axis)
    return points, [float(v) for v in tool_t], [float(v) for v in axis]


def segment_of(left: dict, right: dict, by_pair: dict) -> dict:
    if left["inserted"]:
        pair = (left["from_sequence_id"], left["to_sequence_id"])
    elif right["inserted"]:
        pair = (right["from_sequence_id"], right["to_sequence_id"])
    else:
        pair = (left["sequence_id"], right["sequence_id"])
    segment = by_pair.get(pair)
    if segment is None:
        raise SystemExit(f"BLOCKED_SEGMENT {pair}")
    if (left["inserted"] or right["inserted"]) and segment["kind"] != "PROCESS":
        raise SystemExit(f"BLOCKED_MIDPOINT {pair} {segment['kind']}")
    return segment


def load_coupon_mesh() -> list[list[list[float]]]:
    if sha256(MESH) != MESH_SHA:
        raise SystemExit("BLOCKED_MESH")
    raw = MESH.read_bytes()
    accepted, vertex_count, triangle_count = struct.unpack_from("<III", raw, 8)
    if raw[:5] != b"COUP1" or accepted != 1 or vertex_count != 413 or triangle_count != 564:
        raise SystemExit("BLOCKED_MESH_HEADER")
    offset = 20
    coordinates = struct.unpack_from("<" + "d" * (vertex_count * 3), raw, offset)
    offset += vertex_count * 3 * 8
    indices = struct.unpack_from("<" + "I" * (triangle_count * 3), raw, offset)
    triangles = []
    for triangle_index in range(triangle_count):
        corners = []
        for corner in range(3):
            vertex = indices[triangle_index * 3 + corner]
            corners.append(
                [
                    round(coordinates[vertex * 3] + float(DATUM[0]), 6),
                    round(coordinates[vertex * 3 + 1] + float(DATUM[1]), 6),
                    round(coordinates[vertex * 3 + 2] + float(DATUM[2]), 6),
                ]
            )
        triangles.append(corners)
    return triangles


def main() -> None:
    if sha256(RESTORE) != RESTORE_SHA or sha256(URDF) != URDF_SHA:
        raise SystemExit("BLOCKED_INPUT_INTEGRITY")
    data = json.loads(RESTORE.read_text())
    rows = data["joints"]
    segments = data["segments"]
    if len(rows) != 944 or len(segments) != 740:
        raise SystemExit("BLOCKED_COUNT")
    by_pair = {(item["original_from"], item["original_to"]): item for item in segments}
    urdf = parse_urdf(URDF)
    frames = []
    process_edges = 0
    transition_edges = 0
    process_pairs = set()
    transition_pairs = set()
    for index, row in enumerate(rows):
        points, tool, spray_axis = fk(urdf, np.array(row["joints_rad"], dtype=float))
        if index < len(rows) - 1:
            segment = segment_of(row, rows[index + 1], by_pair)
            kind = segment["kind"]
            pair = [segment["original_from"], segment["original_to"]]
            spray_on = kind == "PROCESS"
            if kind == "PROCESS":
                process_edges += 1
                process_pairs.add(tuple(pair))
            elif kind == "TRANSITION":
                transition_edges += 1
                transition_pairs.add(tuple(pair))
            else:
                raise SystemExit(f"BLOCKED_KIND {kind}")
        else:
            kind = "END"
            pair = [rows[-1]["sequence_id"], rows[-1]["sequence_id"]]
            spray_on = False
        frames.append(
            {
                "rung": row["rung"],
                "sequence_id": row["sequence_id"],
                "inserted": row["inserted"],
                "from_sequence_id": row["from_sequence_id"],
                "to_sequence_id": row["to_sequence_id"],
                "joints_rad": row["joints_rad"],
                "links_m": points,
                "tool0_m": tool,
                "spray_axis": spray_axis,
                "kind": kind,
                "original_from": pair[0],
                "original_to": pair[1],
                "spray_on": spray_on,
                "playback_time_s": round(index * DT_S, 6),
            }
        )
    if process_edges != 923 or transition_edges != 20:
        raise SystemExit(f"BLOCKED_EDGES {process_edges} {transition_edges}")
    if len(process_pairs) != 720 or len(transition_pairs) != 20:
        raise SystemExit(f"BLOCKED_PAIRS {len(process_pairs)} {len(transition_pairs)}")
    if frames[-1]["spray_on"] or any(not math.isfinite(v) for frame in frames for v in frame["tool0_m"]):
        raise SystemExit("BLOCKED_SPRAY")
    model = {
        "schema": "spray_demo_display",
        "baseline_commit": BASELINE,
        "role": "DISPLAY_ONLY",
        "playback_time": "VISUALIZATION_TIME",
        "spray": "VISUAL_EFFECT",
        "nozzle": "DISPLAY_ONLY_NO_COLLISION_CHECK",
        "discrete_verification": "UNCHANGED_PRIOR_RECORD",
        "demo_verification": "SEPARATE",
        "time_parameterization": "NOT_USED",
        "deposition": "NOT_CLAIMED",
        "approach_retract": "OUT_OF_SCOPE",
        "p022": "UNCHANGED",
        "dt_s": DT_S,
        "datum_translation_m": DATUM.tolist(),
        "datum_rotation": "IDENTITY",
        "coupon_local_m": {"min": [0.0, 0.0, 0.0], "max": [0.180, 0.100, 0.039819660146195091]},
        "coupon_display": "EXISTING_MESH",
        "mesh_sha256": MESH_SHA,
        "mesh_triangles": load_coupon_mesh(),
        "link_names": [name for name, _joint in CHAIN],
        "frame_count": 944,
        "frames": frames,
    }
    checks = {
        "baseline_commit": BASELINE,
        "restore_sha256": RESTORE_SHA,
        "urdf_sha256": URDF_SHA,
        "frames": 944,
        "process_graph_edges": process_edges,
        "transition_graph_edges": transition_edges,
        "process_original_segments": len(process_pairs),
        "transition_original_segments": len(transition_pairs),
        "final_spray_on": frames[-1]["spray_on"],
        "inserted_frames_keep_process": all(
            frame["kind"] == "PROCESS"
            for frame in frames
            if frame["inserted"] and frame["kind"] != "END"
        ),
        "tool0_frame0_m": frames[0]["tool0_m"],
        "spray_axis_frame0": frames[0]["spray_axis"],
        "p022_modified": False,
        "manifest_modified": False,
        "coupon_display": "EXISTING_MESH",
        "mesh_sha256": MESH_SHA,
        "mesh_vertices": 413,
        "mesh_triangles": 564,
        "robot_display": "STICK_KINEMATICS",
        "default_view": "CLOSE_TOOL_COUPON",
    }
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "demo_model.json").write_text(json.dumps(model, ensure_ascii=False))
    (OUT_DIR / "checks.json").write_text(json.dumps(checks, indent=2) + "\n")
    write_viewer(model)
    print(json.dumps(checks, indent=2))


def write_viewer(model: dict) -> None:
    payload = json.dumps(
        {
            "link_names": model["link_names"],
            "datum_translation_m": model["datum_translation_m"],
            "coupon_local_m": model["coupon_local_m"],
            "mesh_m": model["mesh_triangles"],
            "dt_s": model["dt_s"],
            "frames": [
                {
                    "rung": frame["rung"],
                    "sequence_id": frame["sequence_id"],
                    "inserted": frame["inserted"],
                    "kind": frame["kind"],
                    "original_from": frame["original_from"],
                    "original_to": frame["original_to"],
                    "spray_on": frame["spray_on"],
                    "playback_time_s": frame["playback_time_s"],
                    "links_m": frame["links_m"],
                    "tool0_m": frame["tool0_m"],
                    "spray_axis": frame["spray_axis"],
                }
                for frame in model["frames"]
            ],
        },
        separators=(",", ":"),
    )
    VIEWER.parent.mkdir(parents=True, exist_ok=True)
    VIEWER.write_text(VIEWER_HTML.replace("__MODEL__", payload))


VIEWER_HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <title>표시용 분사 데모</title>
  <style>
    body { margin: 0; font-family: "Iowan Old Style", Palatino, Georgia, serif; background: #f7f4ee; color: #1c1915; }
    main { display: grid; grid-template-columns: minmax(0, 1fr) 320px; min-height: 100vh; }
    canvas { width: 100%; height: 100vh; background: #fbfaf6; display: block; }
    aside { padding: 20px 18px; border-left: 1px solid #d9d3c7; }
    h1 { font-size: 1.25rem; margin: 0 0 8px; }
    p, li { font-size: 0.92rem; line-height: 1.4; }
    .muted { color: #5c564c; }
    button { margin: 0 6px 8px 0; padding: 6px 10px; }
    dl { display: grid; grid-template-columns: 92px 1fr; gap: 4px 8px; font-family: ui-monospace, Menlo, monospace; font-size: 0.78rem; }
    dt { color: #5c564c; }
    dd { margin: 0; }
    #sprayState.on { color: #2f5d45; }
    #sprayState.off { color: #8c2f2f; }
  </style>
</head>
<body>
  <main>
    <canvas id="view" width="1100" height="800"></canvas>
    <aside>
      <h1>표시용 분사 데모</h1>
      <p class="muted">기준 커밋 fdfa9c9. 저장된 944개 관절을 재생한다. 새 경로가 아니다. 기본 화면은 공구와 coupon 근접 구도다.</p>
      <p>
        <button id="play" type="button">재생</button>
        <button id="pause" type="button">일시정지</button>
        <button id="reset" type="button">처음부터</button>
        <button id="closeView" type="button">근접</button>
        <button id="armView" type="button">팔 전체</button>
      </p>
      <dl>
        <dt>재생 시간</dt><dd id="time">0 s</dd>
        <dt>자세</dt><dd id="pose">0 / 943</dd>
        <dt>구간</dt><dd id="segment">-</dd>
        <dt>분사</dt><dd id="sprayState">OFF</dd>
        <dt>tool0</dt><dd id="tool">-</dd>
        <dt>분사 축</dt><dd id="axis">-</dd>
      </dl>
      <p id="selftest"></p>
      <ul>
        <li id="meshLabel">coupon은 기존 충돌 mesh 표시다.</li>
        <li>로봇은 막대 기구학 표시다.</li>
        <li><span style="color:#1d6aa5">공구 방향</span>과 <span style="color:#c45c12">공정 경로</span>는 서로 다른 색이다.</li>
        <li>시간은 시각화용 재생 시간이다.</li>
        <li>분사는 시각 효과이며 실제 도포 품질 예측이 아니다.</li>
        <li>노즐은 표시용이다. 새 공구 형상의 충돌 검증은 하지 않았다.</li>
        <li>기존 고정 표본 이산 검증과 이 화면의 데모 기능 검증은 서로 다르다.</li>
      </ul>
    </aside>
  </main>
  <script id="model" type="application/json">__MODEL__</script>
  <script>
    const model = JSON.parse(document.getElementById("model").textContent);
    const canvas = document.getElementById("view");
    const ctx = canvas.getContext("2d");
    const views = {
      close: { eye: [0.58, -0.16, 0.34], target: [0.40, 0.01, 0.15], scale: 1280 },
      arm: { eye: [-0.15, -0.85, 0.72], target: [0.42, 0.02, 0.18], scale: 520 }
    };
    let viewName = "close";
    let index = 0;
    let playing = false;
    let last = 0;

    function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
    function norm(a) {
      const n = Math.hypot(a[0], a[1], a[2]) || 1;
      return [a[0] / n, a[1] / n, a[2] / n];
    }
    function cross(a, b) {
      return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    }
    function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    function basis() {
      const view = views[viewName];
      const forward = norm(sub(view.target, view.eye));
      const right = norm(cross(forward, [0, 0, 1]));
      return { eye: view.eye, forward, right, up: cross(right, forward), scale: view.scale };
    }

    function project(p) {
      const cam = basis();
      const d = sub(p, cam.eye);
      const x = dot(d, cam.right);
      const y = dot(d, cam.up);
      const z = dot(d, cam.forward);
      const s = cam.scale / Math.max(z, 0.02);
      return [canvas.width * 0.5 + x * s, canvas.height * 0.58 - y * s, z];
    }

    function line(a, b, color, width) {
      const p = project(a);
      const q = project(b);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      ctx.moveTo(p[0], p[1]);
      ctx.lineTo(q[0], q[1]);
      ctx.stroke();
    }

    function drawMesh() {
      const light = norm([-0.2, -0.35, 0.9]);
      const faces = [];
      for (const tri of model.mesh_m) {
        const n = norm(cross(sub(tri[1], tri[0]), sub(tri[2], tri[0])));
        const shade = Math.max(0.18, Math.abs(dot(n, light)));
        const p0 = project(tri[0]);
        const p1 = project(tri[1]);
        const p2 = project(tri[2]);
        faces.push({ p0, p1, p2, shade, depth: (p0[2] + p1[2] + p2[2]) / 3 });
      }
      faces.sort((a, b) => b.depth - a.depth);
      for (const face of faces) {
        const tone = Math.round(118 + 110 * face.shade);
        ctx.fillStyle = "rgb(" + (tone - 18) + "," + (tone - 6) + "," + (tone - 28) + ")";
        ctx.beginPath();
        ctx.moveTo(face.p0[0], face.p0[1]);
        ctx.lineTo(face.p1[0], face.p1[1]);
        ctx.lineTo(face.p2[0], face.p2[1]);
        ctx.closePath();
        ctx.fill();
      }
    }

    function drawPath() {
      ctx.lineWidth = viewName === "close" ? 2.5 : 1.5;
      ctx.strokeStyle = "#c45c12";
      ctx.beginPath();
      let open = false;
      for (let i = 0; i < model.frames.length - 1; i += 1) {
        if (model.frames[i].kind !== "PROCESS") {
          open = false;
          continue;
        }
        const a = project(model.frames[i].tool0_m);
        const b = project(model.frames[i + 1].tool0_m);
        if (!open) {
          ctx.moveTo(a[0], a[1]);
          open = true;
        }
        ctx.lineTo(b[0], b[1]);
      }
      ctx.stroke();
    }

    function draw() {
      const frame = model.frames[index];
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fbfaf6";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      drawMesh();
      drawPath();
      const links = frame.links_m;
      const startLink = viewName === "close" ? 6 : 1;
      for (let i = startLink; i < links.length; i += 1) line(links[i - 1], links[i], "#1c1915", viewName === "close" ? 4 : 3);
      const tool = frame.tool0_m;
      const axis = frame.spray_axis;
      const back = [tool[0] - axis[0] * 0.018, tool[1] - axis[1] * 0.018, tool[2] - axis[2] * 0.018];
      line(back, tool, "#2f5d45", 5);
      const reach = frame.spray_on ? 0.045 : 0.02;
      const tip = [tool[0] + axis[0] * reach, tool[1] + axis[1] * reach, tool[2] + axis[2] * reach];
      line(tool, tip, "#1d6aa5", viewName === "close" ? 6 : 3);
      const mark = project(tool);
      ctx.fillStyle = "#1d6aa5";
      ctx.beginPath();
      ctx.arc(mark[0], mark[1], viewName === "close" ? 6 : 3, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#5c564c";
      ctx.font = "16px sans-serif";
      ctx.fillText("기존 충돌 mesh 표시", 16, 28);
      ctx.fillText("로봇은 막대 기구학 표시", 16, 50);
      document.getElementById("time").textContent = frame.playback_time_s.toFixed(2) + " s";
      document.getElementById("pose").textContent = frame.rung + " / 943";
      const label = frame.kind === "END" ? "종료" : frame.kind + " " + frame.original_from + "→" + frame.original_to;
      document.getElementById("segment").textContent = label + (frame.inserted ? " 중간" : "");
      const spray = document.getElementById("sprayState");
      spray.textContent = frame.spray_on ? "ON" : "OFF";
      spray.className = frame.spray_on ? "on" : "off";
      document.getElementById("tool").textContent = frame.tool0_m.map((v) => v.toFixed(3)).join(", ");
      document.getElementById("axis").textContent = frame.spray_axis.map((v) => v.toFixed(3)).join(", ");
    }

    function tick(now) {
      if (playing) {
        if (!last) last = now;
        if (now - last >= model.dt_s * 1000) {
          last = now;
          if (index < model.frames.length - 1) index += 1;
          else playing = false;
        }
      }
      draw();
      requestAnimationFrame(tick);
    }

    function play() {
      if (index >= model.frames.length - 1) index = 0;
      playing = true;
      last = 0;
    }
    function pause() { playing = false; }
    function reset() {
      playing = false;
      index = 0;
      last = 0;
      draw();
    }
    document.getElementById("play").addEventListener("click", play);
    document.getElementById("pause").addEventListener("click", pause);
    document.getElementById("reset").addEventListener("click", reset);
    document.getElementById("closeView").addEventListener("click", () => { viewName = "close"; draw(); });
    document.getElementById("armView").addEventListener("click", () => { viewName = "arm"; draw(); });
    window.__demo = {
      play, pause, reset,
      state() {
        return { index, playing, kind: model.frames[index].kind, spray: model.frames[index].spray_on, time: model.frames[index].playback_time_s };
      },
      show(i) { playing = false; index = i; draw(); }
    };
    function runSelfTest() {
      reset();
      const start = window.__demo.state();
      play();
      tick(1000);
      tick(1000 + model.dt_s * 1000 + 5);
      const moved = window.__demo.state();
      pause();
      tick(9000);
      const held = window.__demo.state();
      reset();
      const back = window.__demo.state();
      const pass = start.index === 0 && !start.playing && moved.playing && moved.index === 1 && !held.playing && held.index === 1 && back.index === 0 && !back.playing;
      document.getElementById("selftest").textContent = pass ? "SELFTEST_PASS" : "SELFTEST_FAIL";
    }
    const params = new URLSearchParams(location.search);
    if (params.get("view") === "arm") viewName = "arm";
    if (params.has("f")) window.__demo.show(Number(params.get("f")));
    else draw();
    if (params.get("selftest") === "1") runSelfTest();
    requestAnimationFrame(tick);
  </script>
</body>
</html>
"""


def click_buttons() -> None:
    """재생·일시정지·처음부터 버튼을 마우스 이벤트로 누른다."""
    chrome = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
    port = 9333
    profile = "/tmp/spray-demo-click"
    proc = subprocess.Popen(
        [
            chrome,
            "--headless=new",
            "--disable-gpu",
            f"--remote-debugging-port={port}",
            "--force-device-scale-factor=1",
            "--window-size=1440,900",
            f"--user-data-dir={profile}",
            VIEWER.resolve().as_uri(),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        page = None
        for _ in range(50):
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/json") as response:
                    tabs = json.load(response)
                page = next(item for item in tabs if item.get("type") == "page")
                break
            except (OSError, StopIteration, json.JSONDecodeError):
                time.sleep(0.1)
        if page is None:
            raise SystemExit("CDP_NO_PAGE")
        cdp = Cdp(page["webSocketDebuggerUrl"])
        cdp.call("Runtime.enable")

        def shown(element_id: str) -> str:
            message = cdp.call(
                "Runtime.evaluate",
                {
                    "expression": f"document.getElementById('{element_id}').textContent",
                    "returnByValue": True,
                },
            )
            return message["result"]["result"]["value"]

        def press(element_id: str) -> None:
            message = cdp.call(
                "Runtime.evaluate",
                {
                    "expression": (
                        "(() => { const r = document.getElementById("
                        f"'{element_id}').getBoundingClientRect(); "
                        "return {x: r.x + r.width / 2, y: r.y + r.height / 2, w: r.width}; })()"
                    ),
                    "returnByValue": True,
                },
            )
            box = message["result"]["result"]["value"]
            if box["w"] < 1:
                raise SystemExit("BUTTON_NOT_VISIBLE")
            for kind in ("mousePressed", "mouseReleased"):
                cdp.call(
                    "Input.dispatchMouseEvent",
                    {"type": kind, "x": box["x"], "y": box["y"], "button": "left", "clickCount": 1},
                )

        for _ in range(30):
            if shown("pose") == "0 / 943":
                break
            time.sleep(0.1)
        start = shown("pose")
        press("play")
        time.sleep(0.25)
        moved = shown("pose")
        press("pause")
        held = shown("pose")
        time.sleep(0.3)
        held_later = shown("pose")
        press("reset")
        time.sleep(0.05)
        back = shown("pose")
        back_time = shown("time")
        labels = cdp.call(
            "Runtime.evaluate",
            {"expression": "document.body.innerText", "returnByValue": True},
        )["result"]["result"]["value"]
        record = {
            "method": "CHROME_MOUSE_EVENT",
            "start_pose": start,
            "pose_after_play": moved,
            "pose_after_pause": held,
            "pose_after_pause_wait": held_later,
            "pose_after_reset": back,
            "time_after_reset": back_time,
            "coupon_label_visible": "기존 충돌 mesh 표시" in labels,
            "stick_label_visible": "로봇은 막대 기구학 표시" in labels,
            "tool_direction_visible": "공구 방향" in labels,
            "process_path_visible": "공정 경로" in labels,
            "pass": (
                start == "0 / 943"
                and int(moved.split(" / ")[0]) > 0
                and held_later == held
                and int(held.split(" / ")[0]) >= int(moved.split(" / ")[0])
                and back == "0 / 943"
                and back_time == "0.00 s"
                and "기존 충돌 mesh 표시" in labels
                and "로봇은 막대 기구학 표시" in labels
                and "공구 방향" in labels
                and "공정 경로" in labels
            ),
        }
        checks = json.loads((OUT_DIR / "checks.json").read_text())
        checks["button_click"] = record
        (OUT_DIR / "checks.json").write_text(json.dumps(checks, indent=2) + "\n")
        print(json.dumps(record, ensure_ascii=False, indent=2))
        if not record["pass"]:
            raise SystemExit("BUTTON_CLICK_FAIL")
    finally:
        proc.terminate()
        proc.wait(timeout=5)


class Cdp:
    def __init__(self, url: str) -> None:
        host_path = url.split("://", 1)[1]
        host, path = host_path.split("/", 1)
        hostname, port = host.split(":")
        self.sock = socket.create_connection((hostname, int(port)))
        key = base64.b64encode(os.urandom(16)).decode()
        request = (
            f"GET /{path} HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(request.encode())
        buffer = b""
        while b"\r\n\r\n" not in buffer:
            buffer += self.sock.recv(4096)
        self.next_id = 1

    def call(self, method: str, params: dict | None = None) -> dict:
        payload = json.dumps({"id": self.next_id, "method": method, "params": params or {}}).encode()
        want = self.next_id
        self.next_id += 1
        mask = os.urandom(4)
        header = bytearray([0x81])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        else:
            header.append(0x80 | 126)
            header += struct.pack(">H", length)
        header += mask
        self.sock.sendall(bytes(header) + bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload)))
        while True:
            message = json.loads(self._read())
            if message.get("id") == want:
                return message

    def _read(self) -> bytes:
        def take(count: int) -> bytes:
            buffer = b""
            while len(buffer) < count:
                chunk = self.sock.recv(count - len(buffer))
                if not chunk:
                    raise SystemExit("CDP_CLOSED")
                buffer += chunk
            return buffer

        first, second = take(2)
        length = second & 0x7F
        if length == 126:
            length = struct.unpack(">H", take(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", take(8))[0]
        data = take(length)
        if first & 0x0F == 8:
            raise SystemExit("CDP_CLOSE")
        return data


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "click":
        click_buttons()
    else:
        main()
