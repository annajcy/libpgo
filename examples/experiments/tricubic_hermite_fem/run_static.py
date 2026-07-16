#!/usr/bin/env python3
"""Run static FEM formulation comparisons for bunny or dragon."""

import argparse
import json
import sys
import time
from contextlib import nullcontext
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
from pypgo.tools.sim._runners import run_static
from pypgo.tools.sim._scene import build_scene

from common import (
    FORMULATION_CASES,
    STATIC_SETTINGS as SETTINGS,
    STUDIES,
    asset_id,
    build_cases,
    output_root,
)
from summarize import summarize_static


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


def _signature(name: str, study: dict, cases: dict) -> dict:
    case = cases[name]
    signature = {
        "case": name,
        "study": study["name"],
        "surface_mesh": asset_id(study["surface"]),
        "volume_mesh": asset_id(case["volume"]) if case["volume"] is not None else None,
        "formulation": case["formulation"],
        "settings": SETTINGS,
    }
    if name == "tet_ref":
        signature["tet_reference_selection"] = case["selection"]
    return signature


def _completed_summary(name: str, study: dict, output_dir: Path, cases: dict):
    summary_path = output_dir / "summary.json"
    surface_path = output_dir / "final_surface.obj"
    if not summary_path.exists() or not surface_path.exists():
        return None
    summary = json.loads(summary_path.read_text())
    return (
        summary
        if all(summary.get(key) == value for key, value in _signature(name, study, cases).items())
        else None
    )


def run_case(
    name: str,
    study: dict,
    cases: dict,
    output: Path,
    force: bool = False,
) -> dict:
    output_dir = output / name
    if not force:
        completed = _completed_summary(name, study, output_dir, cases)
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
        "constraints.surface_attachments": [
            {
                "vertices": {"file": str(study["fixed"])},
                "coeff": SETTINGS["attachment_coeff"],
            },
        ],
        "loads.gravity": SETTINGS["gravity"],
        "solver.max_iterations": SETTINGS["max_iterations"],
        "solver.gradient_tolerance": SETTINGS["gradient_tolerance"],
        "output.directory": str(output_dir),
        "output.write_surfaces": True,
    }
    if "sparse_solver" in case:
        overrides["solver.sparse_solver"] = case["sparse_solver"]
    cfg = load_config(mesh_type=case["mesh_type"], mode="static", overrides=overrides)
    started = time.perf_counter()
    summary = run_static(build_scene(cfg), cfg)
    summary.update(_signature(name, study, cases))
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
    parser.add_argument("--study", choices=STUDIES, default="dragon")
    parser.add_argument(
        "--cases",
        nargs="+",
        choices=FORMULATION_CASES,
        default=list(FORMULATION_CASES),
    )
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)

    with _parallelism_scope():
        study = STUDIES[args.study]
        cases = build_cases(study)
        output = args.output_root or output_root(args.study, "static")
        output.mkdir(parents=True, exist_ok=True)

        summaries = []
        for name in args.cases:
            try:
                summaries.append(run_case(name, study, cases, output, args.force))
            except Exception as exc:
                failure = {
                    **_signature(name, study, cases),
                    "converged": False,
                    "status": "exception",
                    "error": str(exc),
                }
                output_dir = output / name
                output_dir.mkdir(parents=True, exist_ok=True)
                (output_dir / "summary.json").write_text(
                    json.dumps(failure, indent=2) + "\n"
                )
                summaries.append(failure)
                print(f"[{study['name']}/{name}] failed: {exc}", file=sys.stderr)

        comparison = summarize_static(args.study, output)
        pin_failed = comparison is not None and any(
            row.get("pin_residual_max", 0.0) > SETTINGS["pin_residual_limit"]
            for row in comparison["cases"]
        )
        return (
            2
            if pin_failed
            or any(not summary.get("converged") for summary in summaries)
            else 0
        )


if __name__ == "__main__":
    raise SystemExit(main())
