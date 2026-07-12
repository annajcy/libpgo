#!/usr/bin/env python3
"""Summarize completed tricubic Hermite FEM static and dynamic experiments."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

EXPERIMENT_DIR = Path(__file__).resolve().parent
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(EXPERIMENT_DIR))

import pypgo as pgo

from common import (
    DYNAMIC_SETTINGS,
    FORMULATION_CASES,
    STUDIES,
    output_root,
)


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def _rel_l2(value: np.ndarray, reference: np.ndarray) -> float:
    denominator = np.linalg.norm(reference)
    return float(np.linalg.norm(value - reference) / denominator) if denominator else 0.0


def static_displacement_metrics(
    displacement: np.ndarray,
    reference: np.ndarray,
    fixed_vertices: np.ndarray,
) -> dict:
    """Compute visible-surface error outside the attached static patch."""
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
        "surface_displacement_rms": float(
            np.sqrt(np.mean(np.sum(displacement[free] ** 2, axis=1)))
        ),
        "pin_residual_max": float(np.linalg.norm(displacement[fixed_vertices], axis=1).max()),
        "reference_tip_vertex": int(tip_index),
        "tip_displacement": displacement[tip_index].tolist(),
        "reference_tip_displacement": reference[tip_index].tolist(),
    }


def _surface_frame_index(path: Path) -> int:
    return int(path.stem.removeprefix("surface"))


def _load_surface_positions(case_dir: Path) -> dict[int, np.ndarray]:
    surface_dir = case_dir / "surface"
    return {
        _surface_frame_index(path): np.asarray(
            pgo.mesh.read_obj(str(path)).vertices,
            dtype=np.float64,
        )
        for path in sorted(surface_dir.glob("surface*.obj"), key=_surface_frame_index)
    }


def _impact_frame(positions: dict[int, np.ndarray], obstacle_top_y: float) -> int | None:
    threshold = obstacle_top_y + DYNAMIC_SETTINGS["dhat_external"]
    for frame in sorted(positions):
        if float(positions[frame][:, 1].min()) <= threshold:
            return int(frame)
    return None


def _window_stats(values: dict[int, float], frames: list[int], prefix: str) -> dict:
    selected = np.asarray([values[frame] for frame in frames], dtype=np.float64)
    if selected.size == 0:
        return {
            f"{prefix}_samples": 0,
            f"{prefix}_rel_l2_mean": None,
            f"{prefix}_rel_l2_p95": None,
        }
    return {
        f"{prefix}_samples": int(selected.size),
        f"{prefix}_rel_l2_mean": float(selected.mean()),
        f"{prefix}_rel_l2_p95": float(np.percentile(selected, 95)),
    }


def trajectory_metrics(
    positions: dict[int, np.ndarray],
    reference_positions: dict[int, np.ndarray],
    rest: np.ndarray,
    obstacle_top_y: float,
) -> dict:
    """Compute trajectory, contact, final-frame, and reference-relative windows."""
    common = sorted(set(positions) & set(reference_positions))
    if not common:
        return {}

    displacement = np.stack([positions[i] - rest for i in common])
    reference = np.stack([reference_positions[i] - rest for i in common])
    rel_values = {
        frame: _rel_l2(d.ravel(), r.ravel())
        for frame, d, r in zip(common, displacement, reference)
    }
    y_rel_values = {
        frame: _rel_l2(d[:, 1], r[:, 1])
        for frame, d, r in zip(common, displacement, reference)
    }
    rel = np.asarray([rel_values[i] for i in common])
    y_rel = np.asarray([y_rel_values[i] for i in common])
    final_error = np.linalg.norm(displacement[-1] - reference[-1], axis=1)
    impact = _impact_frame(positions, obstacle_top_y)
    reference_impact = _impact_frame(reference_positions, obstacle_top_y)
    min_y = min(float(pos[:, 1].min()) for pos in positions.values())

    positive = [frame for frame in common if frame > 0]
    pre_contact = (
        [frame for frame in positive if frame >= 100 and frame < reference_impact]
        if reference_impact is not None
        else []
    )
    if reference_impact is not None and not pre_contact:
        pre_contact = [frame for frame in positive if frame < reference_impact]
    post_contact = (
        [frame for frame in positive if frame >= reference_impact]
        if reference_impact is not None
        else []
    )
    late_start = max(
        reference_impact or 0,
        DYNAMIC_SETTINGS["num_steps"] - 200,
    )
    late = [frame for frame in positive if frame >= late_start]

    metrics = {
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
        "impact_time": impact * DYNAMIC_SETTINGS["timestep"] if impact is not None else None,
        "impact_frame_delta": (
            impact - reference_impact
            if impact is not None and reference_impact is not None
            else None
        ),
        "min_surface_y": min_y,
    }
    metrics.update(_window_stats(rel_values, positive, "full_trajectory"))
    metrics.update(_window_stats(rel_values, pre_contact, "pre_contact"))
    metrics.update(_window_stats(rel_values, post_contact, "post_contact"))
    metrics.update(_window_stats(rel_values, late, "late"))
    return metrics


def _available_summaries(output: Path) -> dict[str, dict]:
    available = {}
    for name in FORMULATION_CASES:
        summary_path = output / name / "summary.json"
        if summary_path.exists():
            available[name] = _load_json(summary_path)
    return available


def _write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _fmt(value, digits: int = 4) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int):
        return f"{value:,}"
    return f"{float(value):.{digits}g}"


def _write_markdown(path: Path, comparison: dict) -> None:
    rows = comparison["cases"]
    mode = comparison["mode"]
    lines = [
        f"# {comparison['title']}",
        "",
        f"Reference: `{comparison['reference']}` ({comparison['reference_status']}).",
        "",
        f"Complete five-case matrix: {'yes' if comparison['complete'] else 'no'}.",
        "",
    ]
    if mode == "static":
        lines += [
            "| case | DOFs | converged | wall s | free rel L2 | y rel L2 | p95 error | pin max |",
            "|---|---:|---|---:|---:|---:|---:|---:|",
        ]
        for row in rows:
            lines.append(
                "| {case} | {dofs} | {converged} | {wall} | {rel} | {yrel} | {p95} | {pin} |".format(
                    case=row["case"],
                    dofs=_fmt(row.get("num_dofs"), 0),
                    converged=_fmt(row.get("converged")),
                    wall=_fmt(row.get("wall_seconds"), 6),
                    rel=_fmt(row.get("free_surface_rel_l2"), 6),
                    yrel=_fmt(row.get("free_surface_y_rel_l2"), 6),
                    p95=_fmt(row.get("free_surface_error_p95"), 6),
                    pin=_fmt(row.get("pin_residual_max"), 6),
                )
            )
    else:
        lines += [
            "| case | DOFs | complete | fresh | wall s | impact | post mean | late mean | final rel L2 |",
            "|---|---:|---|---|---:|---:|---:|---:|---:|",
        ]
        for row in rows:
            lines.append(
                "| {case} | {dofs} | {complete} | {fresh} | {wall} | {impact} | {post} | {late} | {final} |".format(
                    case=row["case"],
                    dofs=_fmt(row.get("num_dofs"), 0),
                    complete=_fmt(row.get("completed")),
                    fresh=_fmt(row.get("fresh_run")),
                    wall=_fmt(row.get("wall_seconds"), 7),
                    impact=_fmt(row.get("impact_frame"), 0),
                    post=_fmt(row.get("post_contact_rel_l2_mean"), 7),
                    late=_fmt(row.get("late_rel_l2_mean"), 7),
                    final=_fmt(row.get("final_surface_rel_l2"), 7),
                )
            )
    lines += [
        "",
        "Notes:",
        "- The tet result is a same-domain numerical reference, not certified ground truth.",
        "- `cubic_linear_x8` is the comparable-DOF baseline; `cubic_linear_x27` is over-resolved.",
        "",
    ]
    path.write_text("\n".join(lines))


def summarize_static(study_name: str, output: Path | None = None) -> dict | None:
    study = STUDIES[study_name]
    output = output or output_root(study_name, "static")
    available = _available_summaries(output)
    reference_summary = available.get("tet_ref")
    reference_surface = output / "tet_ref" / "final_surface.obj"
    if not reference_summary or not reference_summary.get("converged") or not reference_surface.exists():
        print(f"[{study_name}/static] summary skipped: converged tet_ref is unavailable")
        return None

    rest = np.asarray(pgo.mesh.read_obj(str(study["surface"])).vertices, dtype=np.float64)
    fixed = np.atleast_1d(np.loadtxt(study["fixed"], dtype=np.int64))
    reference = np.asarray(pgo.mesh.read_obj(str(reference_surface)).vertices, dtype=np.float64) - rest
    rows = []
    for name in FORMULATION_CASES:
        summary = available.get(name)
        if summary is None:
            continue
        row = dict(summary)
        surface_path = output / name / "final_surface.obj"
        if summary.get("converged") and surface_path.exists():
            displacement = np.asarray(
                pgo.mesh.read_obj(str(surface_path)).vertices,
                dtype=np.float64,
            ) - rest
            row.update(static_displacement_metrics(displacement, reference, fixed))
        rows.append(row)

    comparison = {
        "reference": "tet_ref",
        "reference_status": "same_domain_numerical_baseline_not_ground_truth",
        "study": study_name,
        "mode": "static",
        "title": f"{study['title']} Static FEM Comparison",
        "surface_mesh": str(study["surface"]),
        "fixed_surface_vertices": int(fixed.size),
        "expected_cases": list(FORMULATION_CASES),
        "complete": (
            set(available) == set(FORMULATION_CASES)
            and all(
                row.get("converged")
                and (output / row["case"] / "final_surface.obj").exists()
                for row in rows
            )
        ),
        "cases": rows,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    _write_csv(
        output / "comparison.csv",
        rows,
        [
            "case", "formulation", "num_dofs", "converged", "iterations", "wall_seconds",
            "volume_ratio", "free_surface_rel_l2", "all_surface_rel_l2",
            "free_surface_y_rel_l2", "free_surface_error_mean", "free_surface_error_p95",
            "free_surface_error_max", "pin_residual_max",
        ],
    )
    _write_markdown(output / "summary.md", comparison)
    return comparison


def summarize_dynamic(study_name: str, output: Path | None = None) -> dict | None:
    study = STUDIES[study_name]
    output = output or output_root(study_name, "dynamic")
    available = _available_summaries(output)
    reference_summary = available.get("tet_ref")
    reference_positions = _load_surface_positions(output / "tet_ref")
    if not reference_summary or not reference_positions:
        print(f"[{study_name}/dynamic] summary skipped: tet_ref surfaces are unavailable")
        return None

    rest = np.asarray(pgo.mesh.read_obj(str(study["surface"])).vertices, dtype=np.float64)
    obstacle = pgo.mesh.read_obj(str(study["obstacle"]))
    obstacle_top_y = float(np.asarray(obstacle.vertices, dtype=np.float64)[:, 1].max())
    rows = []
    for name in FORMULATION_CASES:
        summary = available.get(name)
        if summary is None:
            continue
        row = dict(summary)
        row["completed"] = summary.get("final_timestep_id") == DYNAMIC_SETTINGS["num_steps"]
        row["fresh_run"] = (
            summary.get("resumed_from") is None
            and summary.get("initial_timestep_id", 0) == 0
        )
        row.update(
            trajectory_metrics(
                _load_surface_positions(output / name),
                reference_positions,
                rest,
                obstacle_top_y,
            )
        )
        rows.append(row)

    comparison = {
        "reference": "tet_ref",
        "reference_status": "same_domain_dynamic_numerical_baseline_not_ground_truth",
        "study": study_name,
        "mode": "dynamic",
        "title": f"{study['title']} Dynamic Drop FEM Comparison",
        "surface_mesh": str(study["surface"]),
        "obstacle_mesh": str(study["obstacle"]),
        "obstacle_top_y": obstacle_top_y,
        "settings": DYNAMIC_SETTINGS,
        "expected_cases": list(FORMULATION_CASES),
        "complete": (
            set(available) == set(FORMULATION_CASES)
            and all(row.get("completed") and row.get("matched_frames", 0) > 0 for row in rows)
        ),
        "cases": rows,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    _write_csv(
        output / "comparison.csv",
        rows,
        [
            "case", "formulation", "num_dofs", "completed", "fresh_run",
            "final_timestep_id", "target_timestep_id", "num_frames", "wall_seconds",
            "volume_ratio", "matched_frames", "trajectory_rel_l2_mean",
            "trajectory_rel_l2_p95", "trajectory_rel_l2_max", "trajectory_y_rel_l2_mean",
            "full_trajectory_rel_l2_mean", "full_trajectory_rel_l2_p95",
            "pre_contact_rel_l2_mean", "pre_contact_rel_l2_p95",
            "post_contact_rel_l2_mean", "post_contact_rel_l2_p95",
            "late_rel_l2_mean", "late_rel_l2_p95", "final_surface_rel_l2",
            "final_surface_y_rel_l2", "final_surface_error_mean",
            "final_surface_error_p95", "final_surface_error_max", "impact_frame",
            "impact_time", "impact_frame_delta", "min_surface_y",
        ],
    )
    _write_markdown(output / "summary.md", comparison)
    return comparison


def summarize(study_name: str, mode: str, output: Path | None = None) -> dict | None:
    if mode == "static":
        return summarize_static(study_name, output)
    if mode == "dynamic":
        return summarize_dynamic(study_name, output)
    raise ValueError(f"unknown mode: {mode}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=STUDIES, default="bunny")
    parser.add_argument("--mode", choices=("static", "dynamic", "all"), default="all")
    parser.add_argument("--all", action="store_true", help="Summarize both studies and modes")
    parser.add_argument("--output-root", type=Path, help="Override output for one study/mode")
    args = parser.parse_args(argv)

    all_runs = args.all or args.mode == "all"
    if all_runs and args.output_root is not None:
        parser.error("--output-root requires one explicit --study and --mode")
    targets = (
        [(study, mode) for study in STUDIES for mode in ("static", "dynamic")]
        if all_runs
        else [(args.study, args.mode)]
    )
    incomplete = False
    for study_name, mode in targets:
        comparison = summarize(study_name, mode, args.output_root)
        if comparison is None or not comparison["complete"]:
            incomplete = True
        else:
            destination = args.output_root or output_root(study_name, mode)
            print(f"[{study_name}/{mode}] wrote summary under {destination}")
    return 2 if incomplete else 0


if __name__ == "__main__":
    raise SystemExit(main())
