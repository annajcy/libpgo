#!/usr/bin/env python3
"""Split each cubic element into factor x factor x factor cubic elements."""

import argparse
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = SCRIPT_DIR.parent
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(ROOT))

import pypgo as pgo
from pypgo.mesh.volume import VegFile, read_veg, write_veg

LOCAL_CORNERS = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 1.0],
    [1.0, 1.0, 1.0],
    [0.0, 1.0, 1.0],
])


def trilinear(corners: np.ndarray, uvw: tuple[float, float, float]) -> np.ndarray:
    u, v, w = uvw
    weights = (
        (1 - u) * (1 - v) * (1 - w),
        u * (1 - v) * (1 - w),
        u * v * (1 - w),
        (1 - u) * v * (1 - w),
        (1 - u) * (1 - v) * w,
        u * (1 - v) * w,
        u * v * w,
        (1 - u) * v * w,
    )
    return np.tensordot(np.asarray(weights), corners, axes=1)


def subdivide_cubic_data(mesh, *, factor: int = 2, digits: int = 12):
    if factor < 2:
        raise ValueError("factor must be at least 2")
    vertices = []
    vertex_ids = {}
    elements = []

    def add_vertex(point):
        key = tuple(np.round(point, digits))
        idx = vertex_ids.get(key)
        if idx is None:
            idx = len(vertices)
            vertex_ids[key] = idx
            vertices.append(np.asarray(key, dtype=np.float64))
        return idx

    for element in np.asarray(mesh.elements, dtype=np.int64):
        corners = np.asarray(mesh.vertices, dtype=np.float64)[element]
        grid = {
            (i, j, k): add_vertex(trilinear(corners, (i / factor, j / factor, k / factor)))
            for i in range(factor + 1)
            for j in range(factor + 1)
            for k in range(factor + 1)
        }
        for i in range(factor):
            for j in range(factor):
                for k in range(factor):
                    elements.append([
                        grid[(i, j, k)],
                        grid[(i + 1, j, k)],
                        grid[(i + 1, j + 1, k)],
                        grid[(i, j + 1, k)],
                        grid[(i, j, k + 1)],
                        grid[(i + 1, j, k + 1)],
                        grid[(i + 1, j + 1, k + 1)],
                        grid[(i, j + 1, k + 1)],
                    ])

    return pgo.mesh.CubicMeshData(np.asarray(vertices), np.asarray(elements, dtype=np.int64))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--factor", type=int, default=2, help="Subdivisions per cube axis (default: 2)")
    parser.add_argument("--digits", type=int, default=12)
    args = parser.parse_args(argv)

    veg = read_veg(str(args.input))
    refined = subdivide_cubic_data(veg.mesh_data, factor=args.factor, digits=args.digits)
    if abs(refined.volume / veg.mesh_data.volume - 1.0) > 1e-10:
        raise RuntimeError(f"volume changed from {veg.mesh_data.volume} to {refined.volume}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.output.with_suffix(".tmp.veg")
    write_veg(str(tmp), VegFile.from_single_material(refined, veg.materials[0]))
    tmp.replace(args.output)
    print(f"input: {veg.mesh_data.num_vertices} vertices, {veg.mesh_data.num_elements} cubes")
    print(f"output: {refined.num_vertices} vertices, {refined.num_elements} cubes")
    print(f"linear DOFs: {refined.num_vertices * 3}")
    print(f"volume ratio: {refined.volume / veg.mesh_data.volume:.12g}")
    print(f"wrote: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
