"""Reproducible dragon static formulation comparison.

Solves the hanging-dragon static equilibrium with three discretizations and
compares the resulting surface displacement fields on the shared display mesh
(`dragon.obj`, 12847 vertices):

  tet_ref        dragon_big.veg, TetLinear        (fine reference, ~120k DOFs)
  cubic_linear   cubic/dragon.veg, CubicLinear    (coarse voxel mesh)
  cubic_hermite  cubic/dragon.veg, tricubic Hermite (same voxel mesh, 24 DOFs/vtx)

All three use the IDENTICAL constraint: the canonical surface patch
`assets/fixed/dragon-surface-fixed.txt` (289 dragon.obj vertex indices),
soft-pinned at rest through each formulation's surface embedding matrix
(`constraints.surface_attachments`, i.e. pypgo.energy.EmbeddedVertexAttachment).
Constraints defined on the shared surface make the comparison free of
cross-simulation-mesh mapping error and of DOF-clamping artifacts — hard-fixing
Hermite vertices would clamp derivative DOFs and strangle the formulation.

Reported metric: relative L2 = RMS(d - d_ref) / RMS(d_ref) over per-vertex
surface displacement 3-vectors, plus Linf, max/mean |u|, pin residual,
iterations, and wall time.

Expected full-run results (2026-06, see the sim-cli plan addenda): hermite
rel L2 ~52% vs linear ~68% on the coarse voxel mesh; the remaining gap is
dominated by voxelization geometry (the voxel dragon has 2.44x the true
volume), which shape-function order cannot fix.

Usage:
  python examples/scripts/dragon_formulation_comparison.py --output-root /tmp/dragon-cmp
  # subset / pipeline check:
  python ... --cases cubic_linear --quick

Full run takes ~45 min (tet_ref ~30 min, hermite ~15 min, linear ~1 s).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
ASSETS = REPO / "examples" / "assets"
PATCH_FILE = ASSETS / "fixed" / "dragon-surface-fixed.txt"

CASES = {
    "tet_ref": {
        "mesh_type": "tet",
        "overrides": {"mesh.volume": str(ASSETS / "veg" / "tet" / "dragon_big.veg")},
    },
    "cubic_linear": {
        "mesh_type": "cubic",
        "overrides": {"mesh.volume": str(ASSETS / "veg" / "cubic" / "dragon.veg")},
    },
    "cubic_hermite": {
        "mesh_type": "cubic",
        "overrides": {
            "mesh.volume": str(ASSETS / "veg" / "cubic" / "dragon.veg"),
            "mesh.formulation": "cubic-tricubic-hermite",
        },
    },
}


def _read_obj_vertices(path: Path) -> np.ndarray:
    return np.array([
        [float(x) for x in line.split()[1:4]]
        for line in open(path)
        if line.startswith("v ")
    ])


def run_case(name: str, spec: dict, output_root: Path, max_iterations: int) -> dict:
    from pypgo.tools.sim._config import load_config
    from pypgo.tools.sim._runners import run_static
    from pypgo.tools.sim._scene import build_scene

    overrides = {
        "mesh.surface": str(ASSETS / "obj" / "dragon.obj"),
        "constraints.surface_attachments": [
            {"vertices": {"file": str(PATCH_FILE)}, "coeff": 1e5},
        ],
        "loads.gravity": (0.0, -9.81, 0.0),
        "solver.max_iterations": max_iterations,
        "solver.gradient_tolerance": 1e-4,
        "output.directory": str(output_root / name),
        "output.write_surfaces": True,
        **spec["overrides"],
    }
    cfg = load_config(mesh_type=spec["mesh_type"], mode="static", overrides=overrides)
    t0 = time.perf_counter()
    summary = run_static(build_scene(cfg), cfg)
    summary["wall_seconds"] = time.perf_counter() - t0
    return summary


def compare(output_root: Path, case_names: list[str], runs: dict) -> dict:
    rest = _read_obj_vertices(ASSETS / "obj" / "dragon.obj")
    patch = np.loadtxt(PATCH_FILE, dtype=np.int64)
    disp = {
        name: _read_obj_vertices(output_root / name / "final_surface.obj") - rest
        for name in case_names
    }
    ref = disp.get("tet_ref")
    ref_rms = np.sqrt((np.linalg.norm(ref, axis=1) ** 2).mean()) if ref is not None else None

    report = {}
    for name in case_names:
        d = disp[name]
        nn = np.linalg.norm(d, axis=1)
        entry = {
            "num_dofs": runs[name]["num_dofs"],
            "iterations": runs[name]["iterations"],
            "converged": runs[name]["converged"],
            "wall_seconds": round(runs[name]["wall_seconds"], 1),
            "max_u": float(nn.max()),
            "mean_u": float(nn.mean()),
            "pin_residual": float(np.linalg.norm(d[patch], axis=1).max()),
        }
        if ref is not None:
            err = d - ref
            entry["rel_l2_vs_ref"] = float(
                np.sqrt((np.linalg.norm(err, axis=1) ** 2).mean()) / ref_rms)
            entry["linf_vs_ref"] = float(np.abs(err).max())
        report[name] = entry
    return report


def format_table(report: dict) -> str:
    has_ref = any("rel_l2_vs_ref" in e for e in report.values())
    header = (f"{'case':14s} {'dofs':>7s} {'iters':>5s} {'wall':>8s} "
              f"{'max|u|':>8s} {'mean|u|':>8s} {'pin res':>8s}")
    if has_ref:
        header += f" {'rel L2':>7s} {'Linf':>8s}"
    lines = [header]
    for name, e in report.items():
        line = (f"{name:14s} {e['num_dofs']:>7d} {e['iterations']:>5d} "
                f"{e['wall_seconds']:>7.1f}s {e['max_u']:>8.4f} {e['mean_u']:>8.4f} "
                f"{e['pin_residual'] * 1000:>6.2f}mm")
        if "rel_l2_vs_ref" in e:
            line += f" {e['rel_l2_vs_ref']:>6.1%} {e['linf_vs_ref']:>8.4f}"
        lines.append(line)
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Dragon static formulation comparison "
                    "(tet reference vs cubic-linear vs tricubic Hermite).")
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", choices=sorted(CASES),
                        default=["cubic_linear", "cubic_hermite", "tet_ref"],
                        help="subset of cases to run (rel-L2 needs tet_ref)")
    parser.add_argument("--quick", action="store_true",
                        help="cap Newton at 3 iterations — pipeline check only, "
                             "numbers are NOT converged results")
    args = parser.parse_args(argv)

    max_iterations = 3 if args.quick else 200
    args.output_root.mkdir(parents=True, exist_ok=True)

    runs = {}
    for name in args.cases:
        print(f"running {name} ...", flush=True)
        runs[name] = run_case(name, dict(CASES[name]), args.output_root, max_iterations)
        print(f"  converged={runs[name]['converged']} "
              f"iters={runs[name]['iterations']} "
              f"wall={runs[name]['wall_seconds']:.1f}s", flush=True)

    report = compare(args.output_root, args.cases, runs)
    table = format_table(report)
    print("\n" + table)
    if args.quick:
        print("\n[--quick] results are NOT converged; use a full run for real numbers.")

    (args.output_root / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    (args.output_root / "report.md").write_text(
        "# Dragon static formulation comparison\n\n```\n" + table + "\n```\n")
    print(f"\nwrote {args.output_root / 'report.json'} and report.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
