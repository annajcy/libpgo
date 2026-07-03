#!/usr/bin/env python3
"""Tune TetGen max volume to hit a target tet/Hermite DOF ratio."""

import argparse
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[2]
sys.path.insert(0, str(ROOT))

import pypgo as pgo
from pypgo.mesh.volume import ENuMaterial, VegFile, read_veg, write_veg

ASSETS = SCRIPT_DIR / "assets"
MATERIAL = ENuMaterial(E=1e6, nu=0.45, density=1000.0)
CASES = {
    "bunny": {
        "boundary": ASSETS / "obj" / "bunny-conservative-r15-surface.obj",
        "cubic": ASSETS / "veg" / "cubic" / "bunny-conservative-r15.veg",
        "output_template": str(ASSETS / "veg" / "tet" / "bunny-conservative-r15-tet-a{a}.veg"),
        "lo": 1.7e-9,
        "hi": 1.0e-8,
    },
    "dragon": {
        "boundary": ASSETS / "obj" / "dragon-conservative-r15-surface.obj",
        "cubic": ASSETS / "veg" / "cubic" / "dragon-conservative-r15.veg",
        "output_template": str(ASSETS / "veg" / "tet" / "dragon-conservative-r15-tet-a{a}.veg"),
        "lo": 1.0e-7,
        "hi": 3.0e-7,
    },
}


def format_a(value: float) -> str:
    return f"{value:g}".replace("e-0", "e-").replace("e+0", "e+")


def midpoint_a(lo: float, hi: float) -> float:
    return (lo * hi) ** 0.5


def update_bracket(lo: float, hi: float, a: float, actual_ratio: float, target_ratio: float):
    return (a, hi) if actual_ratio > target_ratio else (lo, a)


def output_path(template: str, a: float) -> Path:
    return Path(template.format(a=format_a(a)))


def generate_or_read(boundary, cubic_volume: float, path: Path, a: float):
    if path.exists():
        return read_veg(str(path)).mesh_data
    tet = pgo.mesh.tet_mesher(
        boundary,
        backend="tetgen",
        config={"command": f"pq1.414a{a:.17g}"},
    )
    pts = np.asarray(tet.vertices)[np.asarray(tet.elements)]
    signed = np.linalg.det(np.stack([
        pts[:, 1] - pts[:, 0],
        pts[:, 2] - pts[:, 0],
        pts[:, 3] - pts[:, 0],
    ], axis=2)) / 6.0
    bad = int(np.count_nonzero(signed <= 0.0))
    if bad:
        raise RuntimeError(f"TetGen produced {bad} non-positive tetrahedra")
    if abs(tet.volume / cubic_volume - 1.0) > 1e-6:
        raise RuntimeError(f"tet/cubic volume ratio is {tet.volume / cubic_volume:.12g}")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp.veg")
    write_veg(str(tmp), VegFile.from_single_material(tet, MATERIAL))
    tmp.replace(path)
    return tet


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, default="bunny")
    parser.add_argument("--boundary", type=Path)
    parser.add_argument("--cubic", type=Path)
    parser.add_argument("--output-template")
    parser.add_argument("--target-ratio", type=float, default=5.0)
    parser.add_argument("--a", type=float, help="Generate one exact TetGen max-volume value and exit.")
    parser.add_argument("--lo", type=float, help="Small a with ratio above target.")
    parser.add_argument("--hi", type=float, help="Large a with ratio below target.")
    parser.add_argument("--tolerance", type=float, default=0.05)
    parser.add_argument("--max-iters", type=int, default=6)
    args = parser.parse_args(argv)

    defaults = CASES[args.case]
    boundary_path = args.boundary or defaults["boundary"]
    cubic_path = args.cubic or defaults["cubic"]
    output_template = args.output_template or defaults["output_template"]
    lo = args.lo if args.lo is not None else defaults["lo"]
    hi = args.hi if args.hi is not None else defaults["hi"]

    boundary = pgo.mesh.read_obj(str(boundary_path))
    cubic = read_veg(str(cubic_path)).mesh_data
    hermite_dofs = cubic.num_vertices * 24
    best = None

    def eval_a(a: float):
        path = output_path(output_template, a)
        tet = generate_or_read(boundary, cubic.volume, path, a)
        ratio = (tet.num_vertices * 3) / hermite_dofs
        row = (abs(ratio - args.target_ratio), a, ratio, tet.num_vertices, tet.num_elements, path)
        print(f"a={format_a(a)} ratio={ratio:.4g} tet_vertices={tet.num_vertices} tets={tet.num_elements} path={path}")
        return row

    if args.a is not None:
        eval_a(args.a)
        return 0

    lo_row = eval_a(lo)
    hi_row = eval_a(hi)
    if lo_row[2] < args.target_ratio or hi_row[2] > args.target_ratio:
        raise RuntimeError("lo/hi do not bracket target; use smaller --lo or larger --hi")
    best = min(lo_row, hi_row)
    for _ in range(args.max_iters):
        a = midpoint_a(lo, hi)
        row = eval_a(a)
        best = min(best, row)
        if row[0] <= args.tolerance:
            break
        lo, hi = update_bracket(lo, hi, a, row[2], args.target_ratio)

    _, a, ratio, vertices, elements, path = best
    print(f"best a={format_a(a)} ratio={ratio:.4g} tet_vertices={vertices} tets={elements} path={path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
