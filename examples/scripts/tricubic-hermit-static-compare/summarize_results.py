#!/usr/bin/env python3
"""Summarize conservative r15 static comparison outputs."""

import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[2]
sys.path.insert(0, str(ROOT))

import pypgo as pgo

ASSETS = SCRIPT_DIR / "assets"
CASES = {
    "dragon": {
        "surface": ASSETS / "obj" / "dragon.obj",
        "patch": ASSETS / "fixed" / "dragon-surface-fixed.txt",
        "reference_root": ROOT / "examples" / "outputs" / "dragon-static-compare-conservative-r15",
        "hermite_root": ROOT / "examples" / "outputs" / "dragon-static-compare-p0p1-r15",
        "title": "Dragon Static Compare Conservative R15",
    },
    "bunny": {
        "surface": ASSETS / "obj" / "bunny.obj",
        "patch": ASSETS / "fixed" / "bunny-surface-fixed-ear-tip.txt",
        "reference_root": ROOT / "examples" / "outputs" / "bunny-static-compare-conservative-r15-ear-tip-x5",
        "hermite_root": ROOT / "examples" / "outputs" / "bunny-static-compare-conservative-r15-ear-tip-x5",
        "title": "Bunny Static Compare Conservative R15",
    },
}


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def _load_displacement(surface_path: Path, rest_vertices: np.ndarray) -> np.ndarray:
    return np.asarray(pgo.mesh.read_obj(str(surface_path)).vertices, dtype=np.float64) - rest_vertices


def _rel_l2(value: np.ndarray, reference: np.ndarray) -> float:
    denom = np.linalg.norm(reference)
    return float(np.linalg.norm(value - reference) / denom) if denom else 0.0


def _metrics(displacement: np.ndarray, reference: np.ndarray, fixed_vertices: np.ndarray) -> dict:
    free = np.ones(len(reference), dtype=bool)
    free[fixed_vertices] = False
    err = np.linalg.norm(displacement[free] - reference[free], axis=1)
    return {
        "free_surface_rel_l2": _rel_l2(displacement[free], reference[free]),
        "free_surface_y_rel_l2": _rel_l2(displacement[free, 1], reference[free, 1]),
        "free_surface_error_mean": float(err.mean()),
        "free_surface_error_p95": float(np.percentile(err, 95)),
        "free_surface_error_max": float(err.max()),
        "surface_displacement_rms": float(np.sqrt(np.mean(np.sum(displacement[free] ** 2, axis=1)))),
        "pin_residual_max": float(np.linalg.norm(displacement[fixed_vertices], axis=1).max()),
    }


def _peak_mib(log_path: Path) -> float | None:
    if not log_path.exists():
        return None
    peaks = [
        float(match.group(1))
        for match in re.finditer(r"peakMiB=([0-9.]+)", log_path.read_text(errors="replace"))
    ]
    return max(peaks) if peaks else None


def _case_row(name: str, root: Path, reference: np.ndarray, rest: np.ndarray, fixed: np.ndarray) -> dict:
    case_dir = root / name
    summary = _load_json(case_dir / "summary.json")
    log_path = root / f"{name}.log"
    if not log_path.exists():
        log_path = root / "tmux.log"
    row = {
        "case": name,
        "formulation": summary.get("formulation"),
        "volume_mesh": summary.get("volume_mesh"),
        "num_dofs": summary.get("num_dofs"),
        "converged": summary.get("converged"),
        "iterations": summary.get("iterations"),
        "wall_seconds": summary.get("wall_seconds"),
        "final_gradient_max_norm": summary.get("final_gradient_max_norm"),
        "max_abs_u": summary.get("max_abs_u"),
        "volume_ratio": summary.get("volume_ratio"),
        "peak_process_mib": _peak_mib(log_path),
    }
    surface_path = case_dir / "final_surface.obj"
    if surface_path.exists() and summary.get("converged"):
        row.update(_metrics(_load_displacement(surface_path, rest), reference, fixed))
    return row


def _write_csv(path: Path, rows: list[dict]) -> None:
    fields = [
        "case", "formulation", "num_dofs", "converged", "iterations", "wall_seconds",
        "final_gradient_max_norm", "volume_ratio", "peak_process_mib",
        "free_surface_rel_l2", "free_surface_y_rel_l2", "free_surface_error_mean",
        "free_surface_error_p95", "free_surface_error_max", "pin_residual_max",
    ]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _fmt(value, digits=4):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int):
        return f"{value:,}"
    return f"{float(value):.{digits}g}"


def _write_markdown(path: Path, comparison: dict) -> None:
    rows = comparison["cases"]
    lines = [
        f"# {comparison['title']}",
        "",
        f"Reference: `{comparison['reference']}`.",
        "",
        "| case | formulation | DOFs | converged | iterations | wall s | peak MiB | free rel L2 | y rel L2 | p95 err | pin max |",
        "|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {case} | {formulation} | {dofs} | {conv} | {iters} | {wall} | {peak} | {rel} | {yrel} | {p95} | {pin} |".format(
                case=row["case"],
                formulation=row.get("formulation", ""),
                dofs=_fmt(row.get("num_dofs"), 0),
                conv=_fmt(row.get("converged")),
                iters=_fmt(row.get("iterations"), 0),
                wall=_fmt(row.get("wall_seconds"), 5),
                peak=_fmt(row.get("peak_process_mib"), 5),
                rel=_fmt(row.get("free_surface_rel_l2"), 5),
                yrel=_fmt(row.get("free_surface_y_rel_l2"), 5),
                p95=_fmt(row.get("free_surface_error_p95"), 5),
                pin=_fmt(row.get("pin_residual_max"), 5),
            )
        )
    lines += [
        "",
        "Notes:",
        "- `tet_ref` is a same-domain numerical baseline, not certified ground truth.",
        "- `max_abs_u` is omitted from the table because Hermite stores derivative DOFs in the same vector.",
        "",
    ]
    path.write_text("\n".join(lines))


def summarize(reference_root: Path, hermite_root: Path, output_prefix: Path,
              surface: Path, patch: Path, case: str, title: str) -> dict:
    _surface = surface
    _patch = patch
    rest = np.asarray(pgo.mesh.read_obj(str(_surface)).vertices, dtype=np.float64)
    fixed = np.atleast_1d(np.loadtxt(_patch, dtype=np.int64))
    reference = _load_displacement(reference_root / "tet_ref" / "final_surface.obj", rest)
    rows = [
        _case_row("tet_ref", reference_root, reference, rest, fixed),
        _case_row("cubic_linear", reference_root, reference, rest, fixed),
        _case_row("cubic_linear_x8", reference_root, reference, rest, fixed),
        _case_row("cubic_hermite", hermite_root, reference, rest, fixed),
    ]
    comparison = {
        "reference": "tet_ref",
        "reference_status": "same_domain_numerical_baseline_not_ground_truth",
        "case": case,
        "title": title,
        "surface_mesh": str(_surface),
        "fixed_surface_vertices": int(fixed.size),
        "cases": rows,
    }
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    (output_prefix.with_suffix(".json")).write_text(json.dumps(comparison, indent=2) + "\n")
    _write_csv(output_prefix.with_suffix(".csv"), rows)
    _write_markdown(output_prefix.with_suffix(".md"), comparison)
    return comparison


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, default="dragon")
    parser.add_argument("--reference-root", type=Path)
    parser.add_argument("--hermite-root", type=Path)
    parser.add_argument("--output-prefix", type=Path)
    parser.add_argument("--surface", type=Path, help="Original surface OBJ mesh")
    parser.add_argument("--patch", type=Path, help="Fixed surface vertices file")
    args = parser.parse_args(argv)
    defaults = CASES[args.case]
    reference_root = args.reference_root or defaults["reference_root"]
    comparison = summarize(
        reference_root,
        args.hermite_root or defaults["hermite_root"],
        args.output_prefix or reference_root / "comparison",
        args.surface or defaults["surface"],
        args.patch or defaults["patch"],
        args.case,
        defaults["title"],
    )
    print(json.dumps(comparison, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
