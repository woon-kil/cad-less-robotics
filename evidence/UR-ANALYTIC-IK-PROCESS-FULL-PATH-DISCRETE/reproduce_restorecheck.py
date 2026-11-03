#!/usr/bin/env python3
"""보존 입력 생성, 빌드, restorecheck.

fullgraph 모드는 실행하지 않는다. 과거 바이너리 바이트를 재현하지 않는다.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import resource
import subprocess
import sys
from pathlib import Path

LIMIT = 8589934592
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
RUNS_SHA = "4d51b28f0851114b8d36036d0fb96bb1681bf03f89ac31a47bf7d3a18eecfb26"
INSERTS_SHA = "ff1f59d4275cdee2e41f5b8ef0da3182480204eb32d2372e1eb65ef4dd40f153"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require(path: Path, digest: str) -> None:
    actual = sha256(path)
    if actual != digest:
        raise SystemExit(f"BLOCKED_INPUT_INTEGRITY {path} {actual}")


def generate_inputs(out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    runs = out / "runs.bin"
    join = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/ur_analytic_ik_process_join.py"),
            str(ROOT / "evidence/P-013/cad_process_path.json"),
            str(ROOT / "evidence/P-022/robot_planning_input.json"),
            str(ROOT / "evidence/UR-ANALYTIC-IK-WAYPOINT-SURVEY-R1/search_candidate_set.json"),
            str(ROOT / "reproduction/inputs/ur5e.urdf"),
            str(ROOT / "reproduction/inputs/ur5e.srdf"),
            str(runs),
        ],
        check=False,
    )
    if join.returncode != 0:
        raise SystemExit(join.returncode)
    require(runs, RUNS_SHA)

    spec = importlib.util.spec_from_file_location(
        "full_parent",
        ROOT / "evidence/UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH-CORRESPONDENCE/full_graph_parent.py",
    )
    if spec is None or spec.loader is None:
        raise SystemExit("BLOCKED_INPUT_INTEGRITY parent")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    json_dir = out / "json"
    json_dir.mkdir(parents=True, exist_ok=True)
    source = ROOT / "evidence/UR-ANALYTIC-IK-PROCESS-CONNECT-R3"
    for run_id in range(6, 12):
        name = f"run_{run_id:02d}.json"
        (json_dir / name).write_bytes((source / name).read_bytes())
    module.JSON_DIR = json_dir
    module.OUT = out
    digest, size = module.write_inserts()
    if digest != INSERTS_SHA or size != 2768164:
        raise SystemExit(f"BLOCKED_INPUT_INTEGRITY inserts {size} {digest}")
    print(runs.stat().st_size, RUNS_SHA)
    print(size, digest)


def build(build_dir: Path) -> None:
    build_dir.mkdir(parents=True, exist_ok=True)
    raw = os.environ.get("CMAKE_PREFIX_PATH", "/opt/ros/jazzy")
    parts = [item for item in raw.replace(";", ":").split(":") if item]
    if "/tmp/descartes-light/prefix" not in parts:
        parts.append("/tmp/descartes-light/prefix")
    configure = subprocess.run(
        [
            "cmake",
            "-S",
            str(HERE),
            "-B",
            str(build_dir),
            "-DCMAKE_PREFIX_PATH=" + ";".join(parts),
        ],
        check=False,
    )
    if configure.returncode != 0:
        raise SystemExit(configure.returncode)
    compiled = subprocess.run(
        [
            "cmake",
            "--build",
            str(build_dir),
            "--target",
            "ur_analytic_ik_fullgraph",
            "ur_analytic_ik_restorecheck",
        ],
        check=False,
    )
    if compiled.returncode != 0:
        raise SystemExit(compiled.returncode)
    print(sha256(build_dir / "ur_analytic_ik_restorecheck"))


def restorecheck(binary: Path, generated: Path, output: Path) -> None:
    runs = generated / "runs.bin"
    inserts = generated / "inserts.bin"
    require(runs, RUNS_SHA)
    require(inserts, INSERTS_SHA)
    urdf = ROOT / "reproduction/inputs/ur5e.urdf"
    srdf = ROOT / "reproduction/inputs/ur5e.srdf"
    mesh = ROOT / "evidence/UR-ANALYTIC-IK-PROCESS-COLLISION-COUPON-R2/mesh.bin"
    acceptance = ROOT / "evidence/UR-ANALYTIC-IK-PROCESS-COLLISION-COUPON-R2/mesh_acceptance.json"
    transitions = ROOT / "evidence/UR-ANALYTIC-IK-PROCESS-TRANSITION/transitions.bin"
    selected = ROOT / "evidence/UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH/fullgraph.json"
    manifest = json.loads((HERE / "dependencies.json").read_text())
    require(urdf, manifest["inputs"]["ur5e.urdf"]["sha256"])
    require(srdf, manifest["inputs"]["ur5e.srdf"]["sha256"])
    require(mesh, manifest["inputs"]["mesh.bin"]["sha256"])
    require(acceptance, manifest["inputs"]["mesh_acceptance.json"]["sha256"])
    require(transitions, manifest["inputs"]["transitions.bin"]["sha256"])
    resource.setrlimit(resource.RLIMIT_AS, (LIMIT, LIMIT))
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    if soft != LIMIT or hard != LIMIT:
        raise SystemExit("EXECUTION_ERROR address_limit")
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(binary),
        "restorecheck",
        str(urdf),
        str(srdf),
        str(runs),
        str(mesh),
        str(acceptance),
        str(transitions),
        str(inserts),
        str(selected),
        str(output),
    ]
    os.execv(command[0], command)


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    inputs = sub.add_parser("inputs")
    inputs.add_argument("out", type=Path)
    built = sub.add_parser("build")
    built.add_argument("build_dir", type=Path)
    check = sub.add_parser("check")
    check.add_argument("binary", type=Path)
    check.add_argument("generated", type=Path)
    check.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.command == "inputs":
        generate_inputs(args.out)
        return 0
    if args.command == "build":
        build(args.build_dir)
        return 0
    restorecheck(args.binary, args.generated, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
