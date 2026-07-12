#!/usr/bin/env python3
"""Tune TetGen max volume to hit a target tet/Hermite DOF ratio."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = SCRIPT_DIR.parent
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(EXPERIMENT_DIR))

import pypgo as pgo
from pypgo.mesh.volume import ENuMaterial, VegFile, read_veg, write_veg

from common import ASSETS, STUDIES, asset_id, tet_reference_selection_path

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


def _path_id(path: Path) -> str:
    try:
        return asset_id(path)
    except ValueError:
        return str(path.resolve())


def file_fingerprint(path: Path) -> dict:
    """Return a content fingerprint for a tuning input."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {
        "path": _path_id(path),
        "size": path.stat().st_size,
        "sha256": digest.hexdigest(),
    }


def candidate_metadata_path(path: Path) -> Path:
    return path.with_suffix(".meta.json")


def candidate_input_signature(
    *,
    boundary_fingerprint: dict,
    cubic_fingerprint: dict,
    a: float,
) -> dict:
    return {
        "schema_version": 1,
        "boundary": boundary_fingerprint,
        "cubic": cubic_fingerprint,
        "tetgen_command": f"pq1.414a{a:.17g}",
        "material": {"E": 1e6, "nu": 0.45, "density": 1000.0},
    }


def publish_selection(
    *,
    study: str,
    candidate: Path,
    selection_output: Path,
    a: float,
    target_ratio: float,
    actual_ratio: float,
    vertices: int,
    elements: int,
    input_signature: dict,
) -> dict:
    """Atomically publish the binary-search manifest as the single source of truth."""
    metadata_path = candidate_metadata_path(candidate)
    if not candidate.exists() or not metadata_path.exists():
        raise FileNotFoundError(f"cannot publish missing tet candidate assets: {candidate}")
    candidate_metadata = json.loads(metadata_path.read_text())
    if candidate_metadata.get("input_signature") != input_signature:
        raise ValueError(f"candidate metadata does not match selected inputs: {candidate}")
    selection = {
        "study": study,
        "a": a,
        "a_formatted": format_a(a),
        "target_dof_ratio": target_ratio,
        "actual_dof_ratio": actual_ratio,
        "tet_vertices": vertices,
        "tet_elements": elements,
        "candidate_mesh": _path_id(candidate),
        "candidate_metadata": _path_id(metadata_path),
        "input_signature": input_signature,
    }
    selection_output.parent.mkdir(parents=True, exist_ok=True)
    selection_tmp = selection_output.with_suffix(".tmp.json")
    selection_tmp.write_text(json.dumps(selection, indent=2) + "\n")
    selection_tmp.replace(selection_output)
    return selection


def _validate_tet(tet, cubic_volume: float) -> None:
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


def generate_or_read(
    boundary,
    cubic_volume: float,
    path: Path,
    a: float,
    input_signature: dict,
):
    metadata_path = candidate_metadata_path(path)
    if path.exists() and metadata_path.exists():
        metadata = json.loads(metadata_path.read_text())
        if metadata.get("input_signature") == input_signature:
            tet = read_veg(str(path)).mesh_data
            _validate_tet(tet, cubic_volume)
            return tet
    tet = pgo.mesh.tet_mesher(
        boundary,
        backend="tetgen",
        config={"command": f"pq1.414a{a:.17g}"},
    )
    _validate_tet(tet, cubic_volume)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp.veg")
    write_veg(str(tmp), VegFile.from_single_material(tet, MATERIAL))
    tmp.replace(path)
    metadata_tmp = metadata_path.with_suffix(".tmp.json")
    metadata_tmp.write_text(json.dumps({"input_signature": input_signature}, indent=2) + "\n")
    metadata_tmp.replace(metadata_path)
    return tet


def require_tolerance(best: tuple, tolerance: float) -> None:
    if best[0] > tolerance:
        raise RuntimeError(
            f"best tet/Hermite DOF ratio error {best[0]:.6g} exceeds tolerance {tolerance:.6g}"
        )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=CASES, default="bunny")
    parser.add_argument("--boundary", type=Path)
    parser.add_argument("--cubic", type=Path)
    parser.add_argument("--output-template")
    parser.add_argument("--selection-output", type=Path)
    parser.add_argument("--target-ratio", type=float, default=5.0)
    parser.add_argument("--a", type=float, help="Generate one exact TetGen max-volume value and exit.")
    parser.add_argument("--lo", type=float, help="Small a with ratio above target.")
    parser.add_argument("--hi", type=float, help="Large a with ratio below target.")
    parser.add_argument("--tolerance", type=float, default=0.05)
    parser.add_argument("--max-iters", type=int, default=6)
    args = parser.parse_args(argv)

    defaults = CASES[args.study]
    boundary_path = args.boundary or defaults["boundary"]
    cubic_path = args.cubic or defaults["cubic"]
    output_template = args.output_template or defaults["output_template"]
    selection_output = args.selection_output or tet_reference_selection_path(STUDIES[args.study])
    lo = args.lo if args.lo is not None else defaults["lo"]
    hi = args.hi if args.hi is not None else defaults["hi"]

    boundary = pgo.mesh.read_obj(str(boundary_path))
    cubic = read_veg(str(cubic_path)).mesh_data
    hermite_dofs = cubic.num_vertices * 24
    boundary_fingerprint = file_fingerprint(boundary_path)
    cubic_fingerprint = file_fingerprint(cubic_path)
    best = None

    def eval_a(a: float):
        path = output_path(output_template, a)
        input_signature = candidate_input_signature(
            boundary_fingerprint=boundary_fingerprint,
            cubic_fingerprint=cubic_fingerprint,
            a=a,
        )
        tet = generate_or_read(boundary, cubic.volume, path, a, input_signature)
        ratio = (tet.num_vertices * 3) / hermite_dofs
        row = (
            abs(ratio - args.target_ratio),
            a,
            ratio,
            tet.num_vertices,
            tet.num_elements,
            path,
            input_signature,
        )
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

    require_tolerance(best, args.tolerance)
    _, a, ratio, vertices, elements, path, input_signature = best
    publish_selection(
        study=args.study,
        candidate=path,
        selection_output=selection_output,
        a=a,
        target_ratio=args.target_ratio,
        actual_ratio=ratio,
        vertices=vertices,
        elements=elements,
        input_signature=input_signature,
    )
    print(f"best a={format_a(a)} ratio={ratio:.4g} tet_vertices={vertices} tets={elements} path={path}")
    print(f"selection metadata={selection_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
