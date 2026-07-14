#!/usr/bin/env python3
"""Run dynamic drop FEM formulation comparisons for bunny or dragon."""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

EXPERIMENT_DIR = Path(__file__).resolve().parent
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(EXPERIMENT_DIR))

import pypgo as pgo
import pypgo.parallel as pp
from pypgo.mesh.volume import read_veg
from pypgo.tools.sim._config import load_config
from pypgo.tools.sim._runners import run_dynamic
from pypgo.tools.sim._scene import build_scene

from common import (
    DYNAMIC_SETTINGS as SETTINGS,
    FORMULATION_CASES,
    STUDIES,
    asset_id,
    build_cases,
    output_root,
)
from summarize import summarize_dynamic


def _initialize_parallelism() -> None:
    max_concurrency = SETTINGS["num_threads"]
    if not max_concurrency:
        return
    effective_concurrency = pp.initialize(max_concurrency=max_concurrency)
    print(f"[parallel] effective_concurrency={effective_concurrency}")


def _surface_volume(surface) -> float:
    vertices = np.asarray(surface.vertices, dtype=np.float64)
    triangles = vertices[np.asarray(surface.elements, dtype=np.int64)]
    return float(
        abs(np.einsum("ij,ij->", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])))
        / 6.0
    )


def _signature(
    name: str,
    study: dict,
    cases: dict,
    write_abc: bool,
) -> dict:
    case = cases[name]
    settings = dict(SETTINGS)
    settings["write_abc"] = bool(write_abc)
    signature = {
        "case": name,
        "study": study["name"],
        "surface_mesh": asset_id(study["surface"]),
        "obstacle_mesh": asset_id(study["obstacle"]),
        "volume_mesh": asset_id(case["volume"]) if case["volume"] is not None else None,
        "formulation": case["formulation"],
        "settings": settings,
    }
    if name == "tet_ref":
        signature["tet_reference_selection"] = case["selection"]
    return signature


def _completed_summary(
    name: str,
    study: dict,
    output_dir: Path,
    cases: dict,
    write_abc: bool,
):
    summary_path = output_dir / "summary.json"
    if not summary_path.exists():
        return None
    summary = json.loads(summary_path.read_text())
    if any(
        summary.get(key) != value
        for key, value in _signature(name, study, cases, write_abc).items()
    ):
        return None
    if summary.get("final_timestep_id") != SETTINGS["num_steps"]:
        return None
    return summary


def run_case(
    name: str,
    study: dict,
    cases: dict,
    output: Path,
    force: bool = False,
    write_abc: bool = True,
) -> dict:
    output_dir = output / name
    if not force:
        completed = _completed_summary(name, study, output_dir, cases, write_abc)
        if completed is not None:
            print(f"[{study['name']}/{name}] reusing {output_dir}")
            return completed

    case = cases[name]
    if name == "tet_ref" and (case["selection"] is None or case["volume"] is None):
        raise FileNotFoundError(
            "tet reference selection is missing; run mesh/tune_tet_reference.py first"
        )
    overrides = {
        "mesh.volume": str(case["volume"]),
        "mesh.surface": str(study["surface"]),
        "mesh.formulation": case["formulation"],
        "material.model": "stable_neo",
        "material.enable_material_max_step": False,
        "loads.gravity": SETTINGS["gravity"],
        "initial_state.velocity": SETTINGS["initial_velocity"],
        "contact": [
            {
                "model": "ipc",
                "dhat": SETTINGS["dhat"],
                "dhat_external": SETTINGS["dhat_external"],
                "kappa": SETTINGS["kappa"],
                "obstacles": [{"mesh": str(study["obstacle"])}],
            }
        ],
        "solver.max_iterations": SETTINGS["max_iterations"],
        "solver.gradient_tolerance": SETTINGS["gradient_tolerance"],
        "dynamic.timestep": SETTINGS["timestep"],
        "dynamic.num_steps": SETTINGS["num_steps"],
        "dynamic.integrator": SETTINGS["integrator"],
        "dynamic.damping": SETTINGS["damping"],
        "output.directory": str(output_dir),
        "output.write_surfaces": True,
        "output.write_abc": bool(write_abc),
        "output.write_checkpoints": True,
        "output.dump_interval": SETTINGS["dump_interval"],
    }
    if "sparse_solver" in case:
        overrides["solver.sparse_solver"] = case["sparse_solver"]
    # Forced runs are fresh wall-time benchmarks; recovery runs resume.
    if not force and any((output_dir / "checkpoints").glob("state*.npz")):
        overrides["dynamic.resume"] = "latest"

    cfg = load_config(mesh_type=case["mesh_type"], mode="dynamic", overrides=overrides)
    started = time.perf_counter()
    summary = run_dynamic(build_scene(cfg), cfg)
    summary.update(_signature(name, study, cases, write_abc))
    summary["wall_seconds"] = time.perf_counter() - started
    summary["volume_ratio"] = read_veg(str(case["volume"])).mesh_data.volume / _surface_volume(
        pgo.mesh.read_obj(str(study["surface"]))
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(
        f"[{study['name']}/{name}] frames={summary['final_timestep_id']}/"
        f"{SETTINGS['num_steps']} dofs={summary['num_dofs']} "
        f"wall={summary['wall_seconds']:.1f}s"
    )
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=STUDIES, default="bunny")
    parser.add_argument(
        "--cases",
        nargs="+",
        choices=FORMULATION_CASES,
        default=list(FORMULATION_CASES),
    )
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--no-write-abc", dest="write_abc", action="store_false")
    parser.set_defaults(write_abc=True)
    args = parser.parse_args(argv)

    _initialize_parallelism()

    study = STUDIES[args.study]
    cases = build_cases(study)
    output = args.output_root or output_root(args.study, "dynamic")
    output.mkdir(parents=True, exist_ok=True)

    summaries = []
    for name in args.cases:
        try:
            summaries.append(
                run_case(name, study, cases, output, args.force, args.write_abc)
            )
        except Exception as exc:
            case = cases[name]
            failure = {
                **_signature(name, study, cases, args.write_abc),
                "mode": "dynamic",
                "mesh_type": case["mesh_type"],
                "status": "exception",
                "error": str(exc),
                "num_frames": 0,
                "final_timestep_id": 0,
                "target_timestep_id": SETTINGS["num_steps"],
            }
            output_dir = output / name
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / "summary.json").write_text(json.dumps(failure, indent=2) + "\n")
            summaries.append(failure)
            print(f"[{study['name']}/{name}] failed: {exc}", file=sys.stderr)

    summarize_dynamic(args.study, output)
    return 2 if any(
        summary.get("final_timestep_id") != SETTINGS["num_steps"]
        for summary in summaries
    ) else 0


if __name__ == "__main__":
    raise SystemExit(main())
