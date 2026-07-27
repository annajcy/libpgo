#!/usr/bin/env python3
"""Generate a connected conservative cubic mesh for an experiment study."""

import argparse
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = SCRIPT_DIR.parent
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(EXPERIMENT_DIR))

import pypgo as pgo
from pypgo.mesh.volume import ENuMaterial, VegFile, VolumeMesh, write_veg

from common import ASSETS, STUDIES

OUTPUT_DIR = ASSETS / "veg" / "cubic"
BOUNDARY_DIR = ASSETS / "obj"
MATERIAL = ENuMaterial(E=1e6, nu=0.45, density=1000.0)


def surface_volume(surface) -> float:
    vertices = np.asarray(surface.vertices, dtype=np.float64)
    triangles = vertices[np.asarray(surface.elements, dtype=np.int64)]
    return float(abs(np.einsum("ij,ij->", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2]))) / 6.0)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=STUDIES, default="dragon")
    parser.add_argument("--resolution", type=int, default=15)
    parser.add_argument("--occupancy", choices=("center", "conservative"), default="conservative")
    parser.add_argument("--surface", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--boundary-output", type=Path)
    parser.add_argument("--max-volume-error", type=float)
    args = parser.parse_args(argv)

    surface_path = args.surface or STUDIES[args.study]["surface"]
    stem = f"{args.study}-{args.occupancy}-r{args.resolution}"
    output = args.output or OUTPUT_DIR / f"{stem}.veg"
    surface = pgo.mesh.read_obj(str(surface_path))
    raw = pgo.mesh.cubic_mesher(surface, resolution=args.resolution, occupancy=args.occupancy)
    mesh = pgo.mesh.filter_mesh_components(raw, keep_largest=1)
    ratio = mesh.volume / surface_volume(surface)
    error = abs(ratio - 1.0)
    if args.max_volume_error is not None and error > args.max_volume_error:
        raise RuntimeError(f"volume error {error:.3%} exceeds {args.max_volume_error:.3%}")

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".tmp.veg")
    write_veg(str(temporary), VegFile.from_single_material(mesh, MATERIAL))
    temporary.replace(output)
    boundary_output = args.boundary_output
    if args.occupancy == "conservative" and boundary_output is None:
        boundary_output = BOUNDARY_DIR / f"{stem}-surface.obj"
    if boundary_output is not None:
        boundary_output.parent.mkdir(parents=True, exist_ok=True)
        boundary_tmp = boundary_output.with_suffix(".tmp.obj")
        boundary = VolumeMesh(mesh, MATERIAL).extract_surface_mesh()
        pgo.mesh.write_obj(str(boundary_tmp), boundary)
        boundary_tmp.replace(boundary_output)

    print(f"resolution: {args.resolution}")
    print(f"occupancy: {args.occupancy}")
    print(f"components filtered: {raw.num_elements - mesh.num_elements} elements removed")
    print(f"mesh: {mesh.num_vertices} vertices, {mesh.num_elements} cubes")
    print(f"volume ratio: {ratio:.9g} ({error:.3%} error)")
    print(f"DOFs: linear={mesh.num_vertices * 3}, Hermite={mesh.num_vertices * 24}")
    print(f"wrote: {output}")
    if boundary_output is not None:
        print(f"wrote: {boundary_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
