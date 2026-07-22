#!/usr/bin/env python3
"""Analyze the pre-registered static tet spatial-convergence experiment."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np


CONVERGENCE_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = CONVERGENCE_DIR.parent
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(CONVERGENCE_DIR))

import pypgo as pgo  # noqa: E402

from convergence_common import (  # noqa: E402
    CANONICAL_CONTROLS,
    OUTPUT_ROOT,
    RUN_CASES,
    STATIC_SETTINGS,
    STUDIES,
    convergence_output,
)


RHO_LIMIT = 0.8
DIRECTION_COSINE_LIMIT = 0.9
SOLVER_NOISE_FRACTION_LIMIT = 0.1


def lumped_vertex_areas(vertices: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int64)
    tri = vertices[triangles]
    areas = 0.5 * np.linalg.norm(
        np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1
    )
    weights = np.zeros(len(vertices), dtype=np.float64)
    np.add.at(weights, triangles.ravel(), np.repeat(areas / 3.0, 3))
    return weights


def weighted_inner(a: np.ndarray, b: np.ndarray, weights: np.ndarray) -> float:
    return float(np.einsum("i,ij,ij->", weights, a, b))


def weighted_norm(value: np.ndarray, weights: np.ndarray) -> float:
    return math.sqrt(max(0.0, weighted_inner(value, value, weights)))


def weighted_cosine(a: np.ndarray, b: np.ndarray, weights: np.ndarray) -> float | None:
    denominator = weighted_norm(a, weights) * weighted_norm(b, weights)
    return weighted_inner(a, b, weights) / denominator if denominator else None


def distance_interval(distance: float, uncertainty: float) -> dict:
    return {
        "distance": distance,
        "lower": max(0.0, distance - uncertainty),
        "upper": distance + uncertainty,
    }


def classify_smaller(left: dict, right: dict) -> str:
    """Classify whether the left distance is smaller than the right distance."""
    if left["upper"] < right["lower"]:
        return "supports"
    if right["upper"] < left["lower"]:
        return "falsifies"
    return "inconclusive"


def _load_case(case_dir: Path, rest: np.ndarray) -> tuple[dict, np.ndarray]:
    summary_path = case_dir / "summary.json"
    surface_path = case_dir / "final_surface.obj"
    if not summary_path.exists() or not surface_path.exists():
        raise FileNotFoundError(f"missing completed case artifacts under {case_dir}")
    summary = json.loads(summary_path.read_text())
    if not summary.get("converged"):
        raise RuntimeError(f"case did not converge: {case_dir.name}")
    positions = np.asarray(pgo.mesh.read_obj(str(surface_path)).vertices, dtype=np.float64)
    if positions.shape != rest.shape:
        raise ValueError(f"surface shape mismatch for {case_dir.name}: {positions.shape}")
    return summary, positions - rest


def _case_metrics(
    displacement: np.ndarray,
    reference: np.ndarray,
    *,
    weights: np.ndarray,
    free: np.ndarray,
    fixed: np.ndarray,
) -> dict:
    diff = displacement - reference
    weighted_denominator = weighted_norm(reference, weights)
    vertex_denominator = float(np.linalg.norm(reference[free]))
    point_error = np.linalg.norm(diff[free], axis=1)
    return {
        "area_distance_abs": weighted_norm(diff, weights),
        "area_rel_l2_to_L2": (
            weighted_norm(diff, weights) / weighted_denominator
            if weighted_denominator else 0.0
        ),
        "vertex_rel_l2_to_L2": (
            float(np.linalg.norm(diff[free])) / vertex_denominator
            if vertex_denominator else 0.0
        ),
        "point_error_mean_to_L2": float(point_error.mean()),
        "point_error_p95_to_L2": float(np.percentile(point_error, 95)),
        "point_error_max_to_L2": float(point_error.max()),
        "surface_displacement_rms": float(
            np.sqrt(np.mean(np.sum(displacement[free] ** 2, axis=1)))
        ),
        "pin_residual_max": float(np.linalg.norm(displacement[fixed], axis=1).max()),
    }


def _write_csv(path: Path, rows: list[dict]) -> None:
    fields = [
        "case", "num_dofs", "wall_seconds", "area_distance_abs",
        "area_rel_l2_to_L2", "distance_interval_lower", "distance_interval_upper",
        "vertex_rel_l2_to_L2", "point_error_mean_to_L2",
        "point_error_p95_to_L2", "point_error_max_to_L2",
        "surface_displacement_rms", "pin_residual_max",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _fmt(value, digits: int = 6) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int):
        return f"{value:,}"
    return f"{float(value):.{digits}g}"


def _write_report(path: Path, result: dict) -> None:
    checks = result["eligibility"]
    lines = [
        f"# {result['title']}",
        "",
        f"Pre-registered primary decision: **{result['decision']['x27_vs_current_tet']}**.",
        "",
        f"Hermite versus x27: **{result['decision']['hermite_vs_x27']}**.",
        "",
        "## Tet convergence eligibility",
        "",
        "| check | value | passed |",
        "|---|---:|---|",
        f"| rho = d21/d10 | {_fmt(checks['rho'])} | {_fmt(checks['rho_passed'])} |",
        f"| update direction cosine | {_fmt(checks['direction_cosine'])} | {_fmt(checks['direction_passed'])} |",
        f"| solver noise / d21 | {_fmt(checks['solver_noise_fraction'])} | {_fmt(checks['solver_noise_passed'])} |",
        f"| pin residuals | {_fmt(checks['max_pin_residual'])} | {_fmt(checks['pin_passed'])} |",
        f"| all preconditions |  | {_fmt(checks['eligible'])} |",
        "",
        "## Distances to tet L2",
        "",
        "| case | DOFs | area rel L2 | interval lower | interval upper | vertex rel L2 | RMS | pin max |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in result["cases"]:
        lines.append(
            "| {case} | {dofs} | {area} | {lo} | {hi} | {vertex} | {rms} | {pin} |".format(
                case=row["case"],
                dofs=_fmt(row.get("num_dofs"), 0),
                area=_fmt(row.get("area_rel_l2_to_L2")),
                lo=_fmt(row.get("distance_interval_lower")),
                hi=_fmt(row.get("distance_interval_upper")),
                vertex=_fmt(row.get("vertex_rel_l2_to_L2")),
                rms=_fmt(row.get("surface_displacement_rms")),
                pin=_fmt(row.get("pin_residual_max")),
            )
        )
    lines += [
        "",
        "## Interpretation",
        "",
        result["interpretation"],
        "",
        "The decision uses the area-weighted displacement field and the uncertainty "
        "interval fixed in `PLAN.md`; RMS is diagnostic only.",
        "",
    ]
    path.write_text("\n".join(lines))


def analyze(study_name: str, output: Path | None = None) -> dict:
    study = STUDIES[study_name]
    output = output or convergence_output(study_name)
    canonical_output = OUTPUT_ROOT / study_name / "static"
    surface = pgo.mesh.read_obj(str(study["surface"]))
    rest = np.asarray(surface.vertices, dtype=np.float64)
    fixed_indices = np.atleast_1d(np.loadtxt(study["fixed"], dtype=np.int64))
    fixed = np.asarray(fixed_indices, dtype=np.int64)
    free = np.ones(len(rest), dtype=bool)
    free[fixed] = False
    weights = lumped_vertex_areas(rest, np.asarray(surface.elements, dtype=np.int64))
    weights[~free] = 0.0

    summaries = {}
    displacements = {}
    for name in RUN_CASES:
        summaries[name], displacements[name] = _load_case(output / name, rest)
    for name in CANONICAL_CONTROLS:
        summaries[name], displacements[name] = _load_case(canonical_output / name, rest)

    u0 = displacements["tet_L0"]
    u1 = displacements["tet_L1"]
    u2 = displacements["tet_L2"]
    update10 = u1 - u0
    update21 = u2 - u1
    d10 = weighted_norm(update10, weights)
    d21 = weighted_norm(update21, weights)
    rho = d21 / d10 if d10 else None
    direction_cosine = weighted_cosine(update10, update21, weights)
    solver_noise = weighted_norm(displacements["tet_L0_tight"] - u0, weights)
    solver_noise_fraction = solver_noise / d21 if d21 else None

    raw_rows = []
    for name in (*RUN_CASES, *CANONICAL_CONTROLS):
        row = {
            "case": name,
            "num_dofs": summaries[name].get("num_dofs"),
            "wall_seconds": summaries[name].get("wall_seconds"),
        }
        row.update(
            _case_metrics(
                displacements[name], u2,
                weights=weights, free=free, fixed=fixed,
            )
        )
        raw_rows.append(row)

    max_pin = max(row["pin_residual_max"] for row in raw_rows)
    rho_passed = rho is not None and rho < RHO_LIMIT
    direction_passed = (
        direction_cosine is not None and direction_cosine >= DIRECTION_COSINE_LIMIT
    )
    solver_noise_passed = (
        solver_noise_fraction is not None
        and solver_noise_fraction <= SOLVER_NOISE_FRACTION_LIMIT
    )
    pin_passed = max_pin <= STATIC_SETTINGS["pin_residual_limit"]
    eligible = rho_passed and direction_passed and solver_noise_passed and pin_passed
    tail = (rho / (1.0 - rho) * d21) if eligible else None
    uncertainty_abs = tail + solver_noise if tail is not None else None
    l2_norm = weighted_norm(u2, weights)
    uncertainty_rel = uncertainty_abs / l2_norm if uncertainty_abs is not None and l2_norm else None

    intervals = {}
    for row in raw_rows:
        if uncertainty_rel is None:
            row["distance_interval_lower"] = None
            row["distance_interval_upper"] = None
            continue
        interval = distance_interval(row["area_rel_l2_to_L2"], uncertainty_rel)
        row["distance_interval_lower"] = interval["lower"]
        row["distance_interval_upper"] = interval["upper"]
        intervals[row["case"]] = interval

    if eligible:
        x27_vs_tet = classify_smaller(intervals["cubic_linear_x27"], intervals["tet_L0"])
        hermite_vs_x27 = classify_smaller(intervals["cubic_hermite"], intervals["cubic_linear_x27"])
    else:
        x27_vs_tet = "inconclusive"
        hermite_vs_x27 = "inconclusive"

    if x27_vs_tet == "supports":
        interpretation = (
            "The x27 interval is wholly below the current tet-reference interval, "
            "supporting the hypothesis that x27 is closer to the continuous limit."
        )
    elif x27_vs_tet == "falsifies":
        interpretation = (
            "The current tet-reference interval is wholly below the x27 interval, "
            "materially weakening the hypothesis that x27 is more accurate."
        )
    else:
        interpretation = (
            "The pre-registered convergence checks or interval separation are "
            "insufficient; the experiment remains inconclusive and must not be "
            "resolved from RMS ordering alone."
        )

    x8_update = displacements["cubic_linear_x27"] - displacements["cubic_linear_x8"]
    x64_update = displacements["cubic_linear_x64"] - displacements["cubic_linear_x27"]
    result = {
        "schema_version": 1,
        "study": study_name,
        "title": f"{study['title']} Tet Spatial Convergence",
        "primary_metric": "free_surface_lumped_area_weighted_displacement_l2",
        "thresholds": {
            "rho_limit": RHO_LIMIT,
            "direction_cosine_limit": DIRECTION_COSINE_LIMIT,
            "solver_noise_fraction_limit": SOLVER_NOISE_FRACTION_LIMIT,
            "pin_residual_limit": STATIC_SETTINGS["pin_residual_limit"],
        },
        "tet_updates": {"d10": d10, "d21": d21},
        "eligibility": {
            "rho": rho,
            "rho_passed": rho_passed,
            "direction_cosine": direction_cosine,
            "direction_passed": direction_passed,
            "solver_noise": solver_noise,
            "solver_noise_fraction": solver_noise_fraction,
            "solver_noise_passed": solver_noise_passed,
            "max_pin_residual": max_pin,
            "pin_passed": pin_passed,
            "eligible": eligible,
        },
        "tail_uncertainty_abs": uncertainty_abs,
        "tail_uncertainty_rel": uncertainty_rel,
        "cubic_control": {
            "x27_to_x64_update_abs": weighted_norm(x64_update, weights),
            "x8_to_x27_vs_x27_to_x64_direction_cosine": weighted_cosine(
                x8_update, x64_update, weights
            ),
        },
        "decision": {
            "x27_vs_current_tet": x27_vs_tet,
            "hermite_vs_x27": hermite_vs_x27,
        },
        "interpretation": interpretation,
        "cases": raw_rows,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "convergence.json").write_text(json.dumps(result, indent=2) + "\n")
    _write_csv(output / "distances.csv", raw_rows)
    _write_report(output / "report.md", result)
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=STUDIES, required=True)
    parser.add_argument("--output-root", type=Path)
    args = parser.parse_args(argv)
    result = analyze(args.study, args.output_root)
    print(json.dumps({
        "study": args.study,
        "eligible": result["eligibility"]["eligible"],
        "x27_vs_current_tet": result["decision"]["x27_vs_current_tet"],
        "hermite_vs_x27": result["decision"]["hermite_vs_x27"],
        "report": str((args.output_root or convergence_output(args.study)) / "report.md"),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
