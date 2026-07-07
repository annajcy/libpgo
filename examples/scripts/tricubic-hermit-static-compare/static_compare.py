#!/usr/bin/env python3
"""Run and compare static FEM formulations for a selected asset."""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[2]
sys.path.insert(0, str(ROOT))

import pypgo as pgo
import pypgo.parallel as pp
from pypgo.mesh.volume import read_veg
from pypgo.tools.sim._config import load_config
from pypgo.tools.sim._runners import run_static
from pypgo.tools.sim._scene import build_scene


ASSETS = SCRIPT_DIR / "assets"
FORMULATION_CASES = ("tet_ref", "cubic_linear", "cubic_linear_x8", "cubic_hermite")
STUDIES = {
    "dragon": {
        "surface": ASSETS / "obj" / "dragon.obj",
        "patch": ASSETS / "fixed" / "dragon-surface-fixed.txt",
        "output": ROOT / "examples" / "outputs" / "dragon-static-compare-conservative-r15-x5",
        "prefix": "dragon-conservative-r15",
        "tet_a": "1.47683e-7",
    },
    "bunny": {
        "surface": ASSETS / "obj" / "bunny.obj",
        "patch": ASSETS / "fixed" / "bunny-surface-fixed-ear-tip.txt",
        "output": ROOT / "examples" / "outputs" / "bunny-static-compare-conservative-r15-ear-tip-x5",
        "prefix": "bunny-conservative-r15",
        "tet_a": "2.89036e-9",
    },
}
SETTINGS = {
    "attachment_coeff": 1e5,
    "gravity": [0.0, -9.81, 0.0],
    "max_iterations": 300,
    "gradient_tolerance": 1e-5,
    "pin_residual_limit": 1e-3,
    "num_threads": 4,
}


def build_cases(study: dict) -> dict:
    prefix = study["prefix"]
    return {
        "tet_ref": {
            "mesh_type": "tet",
            "volume": ASSETS / "veg" / "tet" / f"{prefix}-tet-a{study['tet_a']}.veg",
            "formulation": "tet-linear",
            "sparse_solver": "mkl_pardiso",
        },
        "cubic_linear": {
            "mesh_type": "cubic",
            "volume": ASSETS / "veg" / "cubic" / f"{prefix}.veg",
            "formulation": "cubic-linear",
        },
        "cubic_linear_x8": {
            "mesh_type": "cubic",
            "volume": ASSETS / "veg" / "cubic" / f"{prefix}-subdiv2.veg",
            "formulation": "cubic-linear",
        },
        "cubic_hermite": {
            "mesh_type": "cubic",
            "volume": ASSETS / "veg" / "cubic" / f"{prefix}.veg",
            "formulation": "cubic-tricubic-hermite",
        },
    }


def _rel_l2(displacement, reference) -> float:
    denominator = np.linalg.norm(reference)
    return float(np.linalg.norm(displacement - reference) / denominator) if denominator else 0.0


def displacement_metrics(displacement, reference, fixed_vertices) -> dict:
    displacement = np.asarray(displacement, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    free = np.ones(len(reference), dtype=bool)
    free[np.asarray(fixed_vertices, dtype=np.int64)] = False
    point_error = np.linalg.norm(displacement[free] - reference[free], axis=1)
    tip_index = np.flatnonzero(free)[np.argmax(np.linalg.norm(reference[free], axis=1))]
    return {
        "free_surface_rel_l2": _rel_l2(displacement[free], reference[free]),
        "all_surface_rel_l2": _rel_l2(displacement, reference),
        "free_surface_y_rel_l2": _rel_l2(displacement[free, 1], reference[free, 1]),
        "free_surface_error_mean": float(point_error.mean()),
        "free_surface_error_p95": float(np.percentile(point_error, 95)),
        "free_surface_error_max": float(point_error.max()),
        "surface_displacement_rms": float(np.sqrt(np.mean(np.sum(displacement[free] ** 2, axis=1)))),
        "pin_residual_max": float(np.linalg.norm(displacement[fixed_vertices], axis=1).max()),
        "reference_tip_vertex": int(tip_index),
        "tip_displacement": displacement[tip_index].tolist(),
        "reference_tip_displacement": reference[tip_index].tolist(),
    }


def _surface_volume(surface) -> float:
    vertices = np.asarray(surface.vertices, dtype=np.float64)
    triangles = vertices[np.asarray(surface.elements, dtype=np.int64)]
    return float(abs(np.einsum("ij,ij->", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2]))) / 6.0)


def _signature(name, cases: dict) -> dict:
    case = cases[name]
    return {
        "case": name,
        "volume_mesh": str(case["volume"]),
        "formulation": case["formulation"],
        "settings": SETTINGS,
    }


def _completed_summary(name, output_dir, cases: dict):
    summary_path = output_dir / "summary.json"
    surface_path = output_dir / "final_surface.obj"
    if not summary_path.exists() or not surface_path.exists():
        return None
    summary = json.loads(summary_path.read_text())
    return summary if all(summary.get(key) == value for key, value in _signature(name, cases).items()) else None


def run_case(name, study: dict, cases: dict, output_root, force=False) -> dict:
    output_dir = output_root / name
    if not force:
        completed = _completed_summary(name, output_dir, cases)
        if completed is not None:
            print(f"[{name}] reusing {output_dir}")
            return completed

    case = cases[name]
    overrides = {
        "mesh.volume": str(case["volume"]),
        "mesh.surface": str(study["surface"]),
        "mesh.formulation": case["formulation"],
        "material.model": "stable_neo",
        "constraints.surface_attachments": [
            {"vertices": {"file": str(study["patch"])}, "coeff": SETTINGS["attachment_coeff"]},
        ],
        "loads.gravity": SETTINGS["gravity"],
        "solver.max_iterations": SETTINGS["max_iterations"],
        "solver.gradient_tolerance": SETTINGS["gradient_tolerance"],
        "output.directory": str(output_dir),
        "output.write_surfaces": True,
    }
    if "sparse_solver" in case:
        overrides["solver.sparse_solver"] = case["sparse_solver"]
    if SETTINGS["num_threads"]:
        pp.set_worker_limit(SETTINGS["num_threads"])
    cfg = load_config(mesh_type=case["mesh_type"], mode="static", overrides=overrides)
    started = time.perf_counter()
    summary = run_static(build_scene(cfg), cfg)
    summary.update(_signature(name, cases))
    summary["wall_seconds"] = time.perf_counter() - started
    summary["volume_ratio"] = read_veg(str(case["volume"])).mesh_data.volume / _surface_volume(
        pgo.mesh.read_obj(str(study["surface"]))
    )
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"[{name}] converged={summary['converged']} iterations={summary['iterations']} "
          f"dofs={summary['num_dofs']} wall={summary['wall_seconds']:.1f}s")
    return summary


def analyze(study: dict, cases: dict, output_root) -> dict | None:
    available = {}
    for name in cases:
        summary = _completed_summary(name, output_root / name, cases)
        if summary is not None:
            available[name] = summary
    if "tet_ref" not in available or not available["tet_ref"].get("converged"):
        print("comparison skipped: converged tet_ref output is unavailable")
        return None

    rest = np.asarray(pgo.mesh.read_obj(str(study["surface"])).vertices, dtype=np.float64)
    fixed = np.atleast_1d(np.loadtxt(study["patch"], dtype=np.int64))
    displacements = {
        name: np.asarray(pgo.mesh.read_obj(str(output_root / name / "final_surface.obj")).vertices) - rest
        for name, summary in available.items()
        if summary.get("converged")
    }
    reference = displacements["tet_ref"]
    rows = []
    for name in cases:
        if name not in available:
            continue
        row = dict(available[name])
        if name in displacements:
            row.update(displacement_metrics(displacements[name], reference, fixed))
        rows.append(row)

    comparison = {
        "reference": "tet_ref",
        "reference_status": "numerical_baseline_not_convergence_verified",
        "fixed_surface_vertices": int(fixed.size),
        "cases": rows,
    }
    (output_root / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    fields = [
        "case", "formulation", "num_dofs", "converged", "iterations", "wall_seconds",
        "volume_ratio", "free_surface_rel_l2", "all_surface_rel_l2",
        "free_surface_y_rel_l2", "free_surface_error_mean", "free_surface_error_p95",
        "free_surface_error_max", "pin_residual_max",
    ]
    with (output_root / "comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return comparison


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=STUDIES, default="dragon")
    parser.add_argument("--cases", nargs="+", choices=FORMULATION_CASES, default=list(FORMULATION_CASES))
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)

    study = STUDIES[args.case]
    cases = build_cases(study)
    output_root = args.output_root or study["output"]
    output_root.mkdir(parents=True, exist_ok=True)

    summaries = []
    for name in args.cases:
        try:
            summaries.append(run_case(name, study, cases, output_root, args.force))
        except Exception as exc:
            failure = {**_signature(name, cases), "converged": False, "status": "exception", "error": str(exc)}
            output_dir = output_root / name
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / "summary.json").write_text(json.dumps(failure, indent=2) + "\n")
            summaries.append(failure)
            print(f"[{name}] failed: {exc}", file=sys.stderr)

    comparison = analyze(study, cases, output_root)
    pin_failed = comparison is not None and any(
        row.get("pin_residual_max", 0.0) > SETTINGS["pin_residual_limit"]
        for row in comparison["cases"]
    )
    return 2 if pin_failed or any(not summary.get("converged") for summary in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
