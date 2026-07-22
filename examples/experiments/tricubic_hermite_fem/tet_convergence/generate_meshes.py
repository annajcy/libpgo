#!/usr/bin/env python3
"""Generate independent finer tet levels and the cubic-linear x64 control."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


CONVERGENCE_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = CONVERGENCE_DIR.parent
MESH_DIR = EXPERIMENT_DIR / "mesh"
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(EXPERIMENT_DIR))
sys.path.insert(0, str(MESH_DIR))
sys.path.insert(0, str(CONVERGENCE_DIR))

import pypgo as pgo  # noqa: E402
from pypgo.mesh.volume import VegFile, read_veg, write_veg  # noqa: E402

from convergence_common import (  # noqa: E402
    STUDIES,
    TET_LEVEL_FACTORS,
    build_cases,
    level_manifest_path,
    mesh_output,
)
from subdivide_cubic_mesh import subdivide_cubic_data  # noqa: E402
from tune_tet_reference import (  # noqa: E402
    candidate_input_signature,
    candidate_metadata_path,
    file_fingerprint,
    format_a,
    generate_or_read,
)


def relative_ratio_error(actual: float, target: float) -> float:
    return abs(actual - target) / target


def next_a(a: float, actual_ratio: float, target_ratio: float) -> float:
    """Update max volume using the leading-order N ~ 1/a relation."""
    if min(a, actual_ratio, target_ratio) <= 0.0:
        raise ValueError("a and ratios must be positive")
    return a * actual_ratio / target_ratio


def _repo_id(path: Path) -> str:
    try:
        return path.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def _write_manifest(path: Path, manifest: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp.json")
    tmp.write_text(json.dumps(manifest, indent=2) + "\n")
    tmp.replace(path)


def generate_tet_level(
    study_name: str,
    level: str,
    *,
    relative_tolerance: float,
    max_iters: int,
) -> dict:
    study = STUDIES[study_name]
    canonical = build_cases(study)["tet_ref"]
    selection = canonical["selection"]
    if selection is None or canonical["volume"] is None:
        raise FileNotFoundError("canonical tet reference selection is unavailable")

    factor = TET_LEVEL_FACTORS[level]
    base_ratio = float(selection["actual_dof_ratio"])
    base_a = float(selection["a"])
    target_ratio = base_ratio * factor
    boundary_path = EXPERIMENT_DIR / "assets" / "obj" / f"{study['prefix']}-surface.obj"
    cubic_path = EXPERIMENT_DIR / "assets" / "veg" / "cubic" / f"{study['prefix']}.veg"
    boundary = pgo.mesh.read_obj(str(boundary_path))
    cubic = read_veg(str(cubic_path)).mesh_data
    hermite_dofs = cubic.num_vertices * 24
    boundary_fingerprint = file_fingerprint(boundary_path)
    cubic_fingerprint = file_fingerprint(cubic_path)

    out_dir = mesh_output(study_name)
    out_dir.mkdir(parents=True, exist_ok=True)
    a = base_a / factor
    best = None
    for iteration in range(max_iters):
        candidate = out_dir / f"{study['prefix']}-{level}-a{format_a(a)}.veg"
        signature = candidate_input_signature(
            boundary_fingerprint=boundary_fingerprint,
            cubic_fingerprint=cubic_fingerprint,
            a=a,
        )
        tet = generate_or_read(boundary, cubic.volume, candidate, a, signature)
        actual_ratio = tet.num_vertices * 3 / hermite_dofs
        rel_error = relative_ratio_error(actual_ratio, target_ratio)
        row = {
            "iteration": iteration,
            "a": a,
            "actual_ratio": actual_ratio,
            "relative_ratio_error": rel_error,
            "vertices": tet.num_vertices,
            "elements": tet.num_elements,
            "candidate": candidate,
            "input_signature": signature,
        }
        if best is None or rel_error < best["relative_ratio_error"]:
            best = row
        print(
            f"[{study_name}/{level}] iteration={iteration} a={format_a(a)} "
            f"target={target_ratio:.6g} actual={actual_ratio:.6g} "
            f"relative_error={rel_error:.3%} vertices={tet.num_vertices} "
            f"tets={tet.num_elements}"
        )
        if rel_error <= relative_tolerance:
            break
        updated = next_a(a, actual_ratio, target_ratio)
        if abs(updated / a - 1.0) < 1e-4:
            break
        a = updated

    if best is None or best["relative_ratio_error"] > relative_tolerance:
        raise RuntimeError(
            f"{study_name}/{level} best relative DOF error "
            f"{best['relative_ratio_error'] if best else float('inf'):.3%} exceeds "
            f"{relative_tolerance:.3%}"
        )
    candidate = best["candidate"]
    metadata_path = candidate_metadata_path(candidate)
    manifest = {
        "schema_version": 1,
        "study": study_name,
        "level": level,
        "base_case": "canonical_tet_ref",
        "base_candidate_mesh": _repo_id(Path(canonical["volume"])),
        "base_a": base_a,
        "base_actual_dof_ratio": base_ratio,
        "target_factor": factor,
        "target_dof_ratio": target_ratio,
        "a": best["a"],
        "actual_dof_ratio": best["actual_ratio"],
        "relative_ratio_error": best["relative_ratio_error"],
        "tet_vertices": best["vertices"],
        "tet_elements": best["elements"],
        "candidate_mesh": _repo_id(candidate),
        "candidate_metadata": _repo_id(metadata_path),
        "input_signature": best["input_signature"],
    }
    manifest_path = level_manifest_path(study_name, level)
    _write_manifest(manifest_path, manifest)
    print(f"[{study_name}/{level}] manifest={manifest_path}")
    return manifest


def generate_x64(study_name: str) -> Path:
    study = STUDIES[study_name]
    input_path = EXPERIMENT_DIR / "assets" / "veg" / "cubic" / f"{study['prefix']}.veg"
    output_path = mesh_output(study_name) / f"{study['prefix']}-subdiv4.veg"
    if output_path.exists():
        print(f"[{study_name}/cubic_linear_x64] reusing {output_path}")
        return output_path
    veg = read_veg(str(input_path))
    refined = subdivide_cubic_data(veg.mesh_data, factor=4)
    volume_ratio = refined.volume / veg.mesh_data.volume
    if abs(volume_ratio - 1.0) > 1e-10:
        raise RuntimeError(f"x64 subdivision changed volume: ratio={volume_ratio}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = output_path.with_suffix(".tmp.veg")
    write_veg(str(tmp), VegFile.from_single_material(refined, veg.materials[0]))
    tmp.replace(output_path)
    print(
        f"[{study_name}/cubic_linear_x64] vertices={refined.num_vertices} "
        f"cubes={refined.num_elements} dofs={refined.num_vertices * 3} "
        f"path={output_path}"
    )
    return output_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=STUDIES, required=True)
    parser.add_argument(
        "--levels",
        nargs="+",
        choices=(*TET_LEVEL_FACTORS, "cubic_linear_x64"),
        default=[*TET_LEVEL_FACTORS, "cubic_linear_x64"],
    )
    parser.add_argument("--relative-tolerance", type=float, default=0.03)
    parser.add_argument("--max-iters", type=int, default=5)
    args = parser.parse_args(argv)
    if not 0.0 < args.relative_tolerance < 1.0:
        parser.error("--relative-tolerance must be in (0, 1)")

    for level in args.levels:
        if level == "cubic_linear_x64":
            generate_x64(args.study)
        else:
            generate_tet_level(
                args.study,
                level,
                relative_tolerance=args.relative_tolerance,
                max_iters=args.max_iters,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
