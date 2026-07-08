#!/usr/bin/env python3
"""Run and compare dynamic bunny-drop FEM formulations with IPC contact."""

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
from pypgo.tools.sim._runners import run_dynamic
from pypgo.tools.sim._scene import build_scene


ASSETS = SCRIPT_DIR / "assets"
FORMULATION_CASES = ("tet_ref", "cubic_linear", "cubic_linear_x8", "cubic_hermite")
STUDY = {
    "surface": ASSETS / "obj" / "bunny.obj",
    "obstacle": ASSETS / "obj" / "bottom.1.obj",
    "output": ROOT / "examples" / "outputs" / "bunny-dynamic-drop-compare-conservative-r15",
    "prefix": "bunny-conservative-r15",
    "tet_a": "2.8768e-9",
}
SETTINGS = {
    "gravity": [0.0, -9.81, 0.0],
    "initial_velocity": [0.0, 0.0, 0.0],
    "timestep": 0.001,
    "num_steps": 800,
    "integrator": "implicit_euler",
    "damping": [0.0, 0.0],
    "dhat": 0.002,
    "dhat_external": 0.005,
    "kappa": 3000.0,
    "max_iterations": 200,
    "gradient_tolerance": 1e-4,
    "dump_interval": 10,
    "num_threads": 12,
}


def build_cases(study: dict) -> dict:
    prefix = study["prefix"]
    return {
        "tet_ref": {
            "mesh_type": "tet",
            "volume": ASSETS / "veg" / "tet" / f"{prefix}-tet-a{study['tet_a']}.veg",
            "formulation": "tet-linear",
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


def _signature(name, study: dict, cases: dict, write_abc: bool) -> dict:
    case = cases[name]
    settings = dict(SETTINGS)
    settings["write_abc"] = bool(write_abc)
    return {
        "case": name,
        "surface_mesh": str(study["surface"]),
        "obstacle_mesh": str(study["obstacle"]),
        "volume_mesh": str(case["volume"]),
        "formulation": case["formulation"],
        "settings": settings,
    }


def _surface_volume(surface) -> float:
    vertices = np.asarray(surface.vertices, dtype=np.float64)
    triangles = vertices[np.asarray(surface.elements, dtype=np.int64)]
    return float(abs(np.einsum("ij,ij->", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2]))) / 6.0)


def _completed_summary(name, study: dict, output_dir: Path, cases: dict, write_abc: bool):
    summary_path = output_dir / "summary.json"
    if not summary_path.exists():
        return None
    summary = json.loads(summary_path.read_text())
    if any(summary.get(key) != value for key, value in _signature(name, study, cases, write_abc).items()):
        return None
    if summary.get("final_timestep_id") != SETTINGS["num_steps"]:
        return None
    return summary


def run_case(name, study: dict, cases: dict, output_root: Path, force=False, write_abc=True) -> dict:
    output_dir = output_root / name
    if not force:
        completed = _completed_summary(name, study, output_dir, cases, write_abc)
        if completed is not None:
            print(f"[{name}] reusing {output_dir}")
            return completed

    case = cases[name]
    overrides = {
        "mesh.volume": str(case["volume"]),
        "mesh.surface": str(study["surface"]),
        "mesh.formulation": case["formulation"],
        "material.model": "stable_neo",
        "material.enable_material_max_step": False,
        "loads.gravity": SETTINGS["gravity"],
        "initial_state.velocity": SETTINGS["initial_velocity"],
        "contact": [{
            "model": "ipc",
            "dhat": SETTINGS["dhat"],
            "dhat_external": SETTINGS["dhat_external"],
            "kappa": SETTINGS["kappa"],
            "obstacles": [{"mesh": str(study["obstacle"])}],
        }],
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
    if any((output_dir / "checkpoints").glob("state*.npz")):
        overrides["dynamic.resume"] = "latest"
    if SETTINGS["num_threads"]:
        pp.set_worker_limit(SETTINGS["num_threads"])

    cfg = load_config(mesh_type=case["mesh_type"], mode="dynamic", overrides=overrides)
    started = time.perf_counter()
    with pp.cpu_affinity_limit(SETTINGS["num_threads"]):
        summary = run_dynamic(build_scene(cfg), cfg)
    summary.update(_signature(name, study, cases, write_abc))
    summary["wall_seconds"] = time.perf_counter() - started
    summary["volume_ratio"] = read_veg(str(case["volume"])).mesh_data.volume / _surface_volume(
        pgo.mesh.read_obj(str(study["surface"]))
    )
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(
        f"[{name}] frames={summary['final_timestep_id']}/{SETTINGS['num_steps']} "
        f"dofs={summary['num_dofs']} wall={summary['wall_seconds']:.1f}s"
    )
    return summary


def _surface_frame_index(path: Path) -> int:
    return int(path.stem.removeprefix("surface"))


def _load_surface_positions(case_dir: Path) -> dict[int, np.ndarray]:
    surface_dir = case_dir / "surface"
    return {
        _surface_frame_index(path): np.asarray(pgo.mesh.read_obj(str(path)).vertices, dtype=np.float64)
        for path in sorted(surface_dir.glob("surface*.obj"), key=_surface_frame_index)
    }


def _rel_l2(value: np.ndarray, reference: np.ndarray) -> float:
    denominator = np.linalg.norm(reference)
    return float(np.linalg.norm(value - reference) / denominator) if denominator else 0.0


def _impact_frame(positions: dict[int, np.ndarray], obstacle_top_y: float) -> int | None:
    threshold = obstacle_top_y + SETTINGS["dhat_external"]
    for frame in sorted(positions):
        if float(positions[frame][:, 1].min()) <= threshold:
            return int(frame)
    return None


def trajectory_metrics(
    positions: dict[int, np.ndarray],
    reference_positions: dict[int, np.ndarray],
    rest: np.ndarray,
    obstacle_top_y: float,
) -> dict:
    common = sorted(set(positions) & set(reference_positions))
    if not common:
        return {}

    displacement = np.stack([positions[i] - rest for i in common])
    reference = np.stack([reference_positions[i] - rest for i in common])
    rel = np.array([_rel_l2(d.ravel(), r.ravel()) for d, r in zip(displacement, reference)])
    y_rel = np.array([_rel_l2(d[:, 1], r[:, 1]) for d, r in zip(displacement, reference)])
    final_error = np.linalg.norm(displacement[-1] - reference[-1], axis=1)
    impact = _impact_frame(positions, obstacle_top_y)
    reference_impact = _impact_frame(reference_positions, obstacle_top_y)
    min_y = min(float(pos[:, 1].min()) for pos in positions.values())
    return {
        "matched_frames": int(len(common)),
        "first_matched_frame": int(common[0]),
        "last_matched_frame": int(common[-1]),
        "trajectory_rel_l2_mean": float(rel.mean()),
        "trajectory_rel_l2_p95": float(np.percentile(rel, 95)),
        "trajectory_rel_l2_max": float(rel.max()),
        "trajectory_y_rel_l2_mean": float(y_rel.mean()),
        "final_surface_rel_l2": float(rel[-1]),
        "final_surface_y_rel_l2": float(y_rel[-1]),
        "final_surface_error_mean": float(final_error.mean()),
        "final_surface_error_p95": float(np.percentile(final_error, 95)),
        "final_surface_error_max": float(final_error.max()),
        "impact_frame": impact,
        "impact_time": impact * SETTINGS["timestep"] if impact is not None else None,
        "impact_frame_delta": impact - reference_impact
        if impact is not None and reference_impact is not None else None,
        "min_surface_y": min_y,
    }


def _matching_summary(name, study: dict, output_dir: Path, cases: dict, write_abc: bool):
    summary_path = output_dir / "summary.json"
    if not summary_path.exists():
        return None
    summary = json.loads(summary_path.read_text())
    return summary if all(summary.get(k) == v for k, v in _signature(name, study, cases, write_abc).items()) else None


def analyze(study: dict, cases: dict, output_root: Path, write_abc=True) -> dict | None:
    reference_summary = _matching_summary("tet_ref", study, output_root / "tet_ref", cases, write_abc)
    if reference_summary is None:
        print("comparison skipped: tet_ref output is unavailable")
        return None

    rest = np.asarray(pgo.mesh.read_obj(str(study["surface"])).vertices, dtype=np.float64)
    reference_positions = _load_surface_positions(output_root / "tet_ref")
    if not reference_positions:
        print("comparison skipped: tet_ref surface frames are unavailable")
        return None

    obstacle = pgo.mesh.read_obj(str(study["obstacle"]))
    obstacle_top_y = float(np.asarray(obstacle.vertices, dtype=np.float64)[:, 1].max())
    rows = []
    for name in cases:
        summary = _matching_summary(name, study, output_root / name, cases, write_abc)
        if summary is None:
            continue
        row = dict(summary)
        row["completed"] = summary.get("final_timestep_id") == SETTINGS["num_steps"]
        row.update(trajectory_metrics(_load_surface_positions(output_root / name), reference_positions, rest, obstacle_top_y))
        rows.append(row)

    comparison = {
        "reference": "tet_ref",
        "reference_status": "same_domain_dynamic_numerical_baseline_not_ground_truth",
        "case": "bunny",
        "title": "Bunny Dynamic Drop Compare Conservative R15",
        "surface_mesh": str(study["surface"]),
        "obstacle_mesh": str(study["obstacle"]),
        "obstacle_top_y": obstacle_top_y,
        "settings": SETTINGS,
        "cases": rows,
    }
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    fields = [
        "case", "formulation", "num_dofs", "completed", "final_timestep_id", "target_timestep_id",
        "num_frames", "wall_seconds", "volume_ratio", "matched_frames", "trajectory_rel_l2_mean",
        "trajectory_rel_l2_p95", "trajectory_rel_l2_max", "trajectory_y_rel_l2_mean",
        "final_surface_rel_l2", "final_surface_y_rel_l2", "final_surface_error_p95",
        "impact_frame", "impact_time", "impact_frame_delta", "min_surface_y",
    ]
    with (output_root / "comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return comparison


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", choices=FORMULATION_CASES, default=list(FORMULATION_CASES))
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--no-write-abc", dest="write_abc", action="store_false")
    parser.set_defaults(write_abc=True)
    args = parser.parse_args(argv)

    cases = build_cases(STUDY)
    output_root = args.output_root or STUDY["output"]
    output_root.mkdir(parents=True, exist_ok=True)

    summaries = []
    for name in args.cases:
        try:
            summaries.append(run_case(name, STUDY, cases, output_root, args.force, args.write_abc))
        except Exception as exc:
            case = cases[name]
            failure = {
                **_signature(name, STUDY, cases, args.write_abc),
                "mode": "dynamic",
                "mesh_type": case["mesh_type"],
                "status": "exception",
                "error": str(exc),
                "num_frames": 0,
                "final_timestep_id": 0,
                "target_timestep_id": SETTINGS["num_steps"],
            }
            output_dir = output_root / name
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / "summary.json").write_text(json.dumps(failure, indent=2) + "\n")
            summaries.append(failure)
            print(f"[{name}] failed: {exc}", file=sys.stderr)

    analyze(STUDY, cases, output_root, args.write_abc)
    return 2 if any(s.get("final_timestep_id") != SETTINGS["num_steps"] for s in summaries) else 0


if __name__ == "__main__":
    raise SystemExit(main())
