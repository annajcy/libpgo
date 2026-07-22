#!/usr/bin/env python3
"""Run independent static tet-convergence and cubic x64 cases."""

from __future__ import annotations

import argparse
import json
import sys
import time
from contextlib import nullcontext
from pathlib import Path

import numpy as np


CONVERGENCE_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = CONVERGENCE_DIR.parent
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(CONVERGENCE_DIR))

import pypgo as pgo  # noqa: E402
import pypgo.parallel as pp  # noqa: E402
from pypgo.mesh.volume import read_veg  # noqa: E402
from pypgo.tools.sim._config import load_config  # noqa: E402
from pypgo.tools.sim._runners import run_static as solve_static  # noqa: E402
from pypgo.tools.sim._scene import build_scene  # noqa: E402

from convergence_common import (  # noqa: E402
    RUN_CASES,
    STATIC_SETTINGS as SETTINGS,
    STUDIES,
    build_convergence_cases,
    convergence_output,
)


def _parallelism_scope():
    max_concurrency = SETTINGS["num_threads"]
    if not max_concurrency:
        return nullcontext()
    print(f"[parallel] max_allowed_parallelism={max_concurrency}")
    return pp.GlobalTbbControl(max_concurrency)


def _surface_volume(surface) -> float:
    vertices = np.asarray(surface.vertices, dtype=np.float64)
    triangles = vertices[np.asarray(surface.elements, dtype=np.int64)]
    return float(
        abs(np.einsum("ij,ij->", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])))
        / 6.0
    )


def _mesh_stat(path: Path) -> dict:
    stat = path.stat()
    return {
        "path": str(path.resolve()),
        "size": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }


def _signature(name: str, study: dict, case: dict) -> dict:
    return {
        "experiment": "tet_convergence",
        "schema_version": 1,
        "case": name,
        "study": study["name"],
        "surface_mesh": str(study["surface"].resolve()),
        "volume_mesh": _mesh_stat(case["volume"]),
        "formulation": case["formulation"],
        "source": case["source"],
        "gradient_tolerance": case["gradient_tolerance"],
        "settings": SETTINGS,
        "mesh_manifest": case["manifest"],
    }


def _completed_summary(name: str, study: dict, case: dict, output_dir: Path):
    summary_path = output_dir / "summary.json"
    surface_path = output_dir / "final_surface.obj"
    if not summary_path.exists() or not surface_path.exists():
        return None
    summary = json.loads(summary_path.read_text())
    signature = _signature(name, study, case)
    return summary if all(summary.get(k) == v for k, v in signature.items()) else None


def run_case(name: str, study: dict, case: dict, output: Path, force: bool = False) -> dict:
    output_dir = output / name
    if not force:
        completed = _completed_summary(name, study, case, output_dir)
        if completed is not None:
            print(f"[{study['name']}/{name}] reusing {output_dir}")
            return completed

    overrides = {
        "mesh.volume": str(case["volume"]),
        "mesh.surface": str(study["surface"]),
        "mesh.formulation": case["formulation"],
        "material.model": "stable_neo",
        "constraints.surface_attachments": [
            {
                "vertices": {"file": str(study["fixed"])},
                "coeff": SETTINGS["attachment_coeff"],
            },
        ],
        "loads.gravity": SETTINGS["gravity"],
        "solver.max_iterations": SETTINGS["max_iterations"],
        "solver.gradient_tolerance": case["gradient_tolerance"],
        "output.directory": str(output_dir),
        "output.write_surfaces": True,
    }
    cfg = load_config(mesh_type=case["mesh_type"], mode="static", overrides=overrides)
    started = time.perf_counter()
    summary = solve_static(build_scene(cfg), cfg)
    summary.update(_signature(name, study, case))
    summary["wall_seconds"] = time.perf_counter() - started
    summary["volume_ratio"] = read_veg(str(case["volume"])).mesh_data.volume / _surface_volume(
        pgo.mesh.read_obj(str(study["surface"]))
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(
        f"[{study['name']}/{name}] converged={summary['converged']} "
        f"iterations={summary['iterations']} dofs={summary['num_dofs']} "
        f"wall={summary['wall_seconds']:.1f}s"
    )
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=STUDIES, required=True)
    parser.add_argument("--cases", nargs="+", choices=RUN_CASES, default=list(RUN_CASES))
    parser.add_argument("--output-root", type=Path)
    parser.add_argument(
        "--num-threads",
        type=int,
        help="Override the canonical TBB concurrency for execution only.",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)
    if args.num_threads is not None:
        if args.num_threads < 1:
            parser.error("--num-threads must be positive")
        SETTINGS["num_threads"] = args.num_threads

    study = STUDIES[args.study]
    cases = build_convergence_cases(args.study)
    output = args.output_root or convergence_output(args.study)
    output.mkdir(parents=True, exist_ok=True)
    summaries = []
    with _parallelism_scope():
        for name in args.cases:
            try:
                summaries.append(run_case(name, study, cases[name], output, args.force))
            except Exception as exc:
                failure = {
                    **_signature(name, study, cases[name]),
                    "converged": False,
                    "status": "exception",
                    "error": str(exc),
                }
                output_dir = output / name
                output_dir.mkdir(parents=True, exist_ok=True)
                (output_dir / "summary.json").write_text(json.dumps(failure, indent=2) + "\n")
                summaries.append(failure)
                print(f"[{study['name']}/{name}] failed: {exc}", file=sys.stderr)

    pin_failed = any(
        summary.get("pin_residual_max", 0.0) > SETTINGS["pin_residual_limit"]
        for summary in summaries
    )
    return 2 if pin_failed or any(not row.get("converged") for row in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
