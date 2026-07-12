#!/usr/bin/env python3
"""Mesh sanity report for a static formulation comparison."""

from __future__ import annotations

import argparse
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
from pypgo.mesh.volume import read_veg

from common import ASSETS, STUDIES

CASES = {
    name: {
        "surface": study["surface"],
        "tet": ASSETS / "veg" / "tet" / f"{study['prefix']}-tet-a{study['tet_a']}.veg",
        "cubic": ASSETS / "veg" / "cubic" / f"{study['prefix']}.veg",
    }
    for name, study in STUDIES.items()
}


def _bbox(mesh) -> dict:
    lo, hi = mesh.bbox
    return {
        "min": lo.tolist(),
        "max": hi.tolist(),
        "diagonal": float(np.linalg.norm(hi - lo)),
    }


def _percentiles(values: np.ndarray, points=(0, 1, 5, 10, 50, 90, 95, 99, 100)) -> dict:
    vals = np.percentile(np.asarray(values, dtype=np.float64), points)
    return {f"p{p:g}": float(v) for p, v in zip(points, vals)}


def _surface_signed_volumes(vertices: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    tri = vertices[triangles]
    return np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])) / 6.0


def _tet_signed_volumes(vertices: np.ndarray, tets: np.ndarray) -> np.ndarray:
    pts = vertices[tets]
    return np.linalg.det(np.stack([pts[:, 1] - pts[:, 0], pts[:, 2] - pts[:, 0], pts[:, 3] - pts[:, 0]], axis=2)) / 6.0


def _tet_quality(mesh) -> dict:
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    tets = np.asarray(mesh.elements, dtype=np.int64)
    pts = vertices[tets]
    signed = _tet_signed_volumes(vertices, tets)
    volumes = np.abs(signed)
    edge_pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
    edges = np.stack([np.linalg.norm(pts[:, i] - pts[:, j], axis=1) for i, j in edge_pairs], axis=1)

    mean_ratio = 12.0 * (3.0 * volumes) ** (2.0 / 3.0) / np.sum(edges * edges, axis=1)

    face_triples = [(1, 2, 3), (0, 3, 2), (0, 1, 3), (0, 2, 1)]
    area = np.zeros(len(tets), dtype=np.float64)
    for i, j, k in face_triples:
        area += 0.5 * np.linalg.norm(np.cross(pts[:, j] - pts[:, i], pts[:, k] - pts[:, i]), axis=1)
    inradius = np.divide(3.0 * volumes, area, out=np.zeros_like(volumes), where=area > 0.0)

    # ponytail: looped solve is fine for a 186k-tet offline sanity check.
    a, b, c, d = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
    lhs = 2.0 * np.stack([b - a, c - a, d - a], axis=1)
    rhs = np.stack([
        np.sum(b * b - a * a, axis=1),
        np.sum(c * c - a * a, axis=1),
        np.sum(d * d - a * a, axis=1),
    ], axis=1)
    circumradius = np.full(len(tets), np.nan, dtype=np.float64)
    for idx in range(len(tets)):
        try:
            center = np.linalg.solve(lhs[idx], rhs[idx])
        except np.linalg.LinAlgError:
            continue
        circumradius[idx] = np.linalg.norm(center - a[idx])
    radius_ratio = np.divide(
        3.0 * inradius,
        circumradius,
        out=np.zeros_like(volumes),
        where=(circumradius > 0.0) & np.isfinite(circumradius),
    )

    return {
        "volume": float(mesh.volume),
        "negative_orientation_count": int(np.count_nonzero(signed < 0.0)),
        "element_volume_percentiles": _percentiles(volumes),
        "edge_length_percentiles": _percentiles(edges.ravel(), (0, 1, 5, 50, 95, 99, 100)),
        "mean_ratio_quality_percentiles": _percentiles(mean_ratio),
        "radius_ratio_quality_percentiles": _percentiles(radius_ratio),
        "low_quality_counts": {
            "mean_ratio_lt_0.1": int(np.count_nonzero(mean_ratio < 0.1)),
            "mean_ratio_lt_0.2": int(np.count_nonzero(mean_ratio < 0.2)),
            "mean_ratio_lt_0.3": int(np.count_nonzero(mean_ratio < 0.3)),
            "radius_ratio_lt_0.05": int(np.count_nonzero(radius_ratio < 0.05)),
            "radius_ratio_lt_0.1": int(np.count_nonzero(radius_ratio < 0.1)),
            "radius_ratio_lt_0.2": int(np.count_nonzero(radius_ratio < 0.2)),
        },
    }


def _cubic_quality(mesh) -> dict:
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    cubes = np.asarray(mesh.elements, dtype=np.int64)
    pts = vertices[cubes]
    distances = []
    for i in range(8):
        for j in range(i + 1, 8):
            distances.append(np.linalg.norm(pts[:, i] - pts[:, j], axis=1))
    distances = np.stack(distances, axis=1)
    edge_lengths = np.sort(distances, axis=1)[:, :12]
    edge_min = edge_lengths.min(axis=1)
    edge_max = edge_lengths.max(axis=1)
    return {
        "edge_length_percentiles": _percentiles(edge_lengths.ravel(), (0, 1, 5, 50, 95, 99, 100)),
        "max_aspect_ratio": float(np.max(edge_max / edge_min)),
    }


def build_report(surface_path: Path, tet_path: Path, cubic_path: Path) -> dict:
    surface = pgo.mesh.read_obj(str(surface_path))
    tet = read_veg(str(tet_path)).mesh_data
    cubic = read_veg(str(cubic_path)).mesh_data
    tet_info = pgo.mesh.volume_mesh_info(tet)
    cubic_info = pgo.mesh.volume_mesh_info(cubic)
    surface_quality = pgo.mesh.check_surface_quality(surface)

    surface_vertices = np.asarray(surface.vertices, dtype=np.float64)
    surface_triangles = np.asarray(surface.elements, dtype=np.int64)

    surface_signed = _surface_signed_volumes(surface_vertices, surface_triangles)
    surface_volume = float(abs(surface_signed.sum()))
    tet_quality = _tet_quality(tet)

    return {
        "paths": {
            "surface": str(surface_path),
            "tet": str(tet_path),
            "cubic": str(cubic_path),
        },
        "surface": {
            "num_vertices": int(surface.num_vertices),
            "num_triangles": int(surface.num_elements),
            "bbox": _bbox(surface),
            "signed_volume": float(surface_signed.sum()),
            "volume": surface_volume,
            "abs_tetra_sum_volume": float(np.abs(surface_signed).sum()),
            "is_clean": bool(surface_quality.is_clean),
            "degenerate_triangles": len(surface_quality.degenerate_tris),
            "non_manifold_edges": len(surface_quality.non_manifold_edges),
            "has_self_intersections": bool(surface_quality.has_self_intersections),
        },
        "tet": {
            "num_vertices": int(tet_info.num_vertices),
            "num_elements": int(tet_info.num_elements),
            "dofs": int(tet_info.num_vertices * 3),
            "bbox": _bbox(tet),
            "center_of_mass": tet_info.center_of_mass.tolist(),
            **tet_quality,
        },
        "cubic": {
            "num_vertices": int(cubic_info.num_vertices),
            "num_elements": int(cubic_info.num_elements),
            "linear_dofs": int(cubic_info.num_vertices * 3),
            "hermite_dofs": int(cubic_info.num_vertices * 24),
            "bbox": _bbox(cubic),
            "center_of_mass": cubic_info.center_of_mass.tolist(),
            "volume": float(cubic_info.total_volume),
            **_cubic_quality(cubic),
        },
        "ratios": {
            "tet_volume_over_surface_volume": float(tet_quality["volume"] / surface_volume),
            "cubic_volume_over_surface_volume": float(cubic_info.total_volume / surface_volume),
            "cubic_volume_over_tet_volume": float(cubic_info.total_volume / tet_quality["volume"]),
        },
    }


def print_report(report: dict, case: str) -> None:
    print(f"{case.title()} static mesh quality")
    print(f"surface: {report['surface']['num_vertices']} vertices, {report['surface']['num_triangles']} triangles")
    print(f"tet:     {report['tet']['num_vertices']} vertices, {report['tet']['num_elements']} tets, {report['tet']['dofs']} DOFs")
    print(
        "cubic:   "
        f"{report['cubic']['num_vertices']} vertices, {report['cubic']['num_elements']} cubes, "
        f"{report['cubic']['linear_dofs']} linear DOFs, {report['cubic']['hermite_dofs']} Hermite DOFs"
    )
    print()
    print(f"surface volume: {report['surface']['volume']:.9g}")
    print(f"tet volume:     {report['tet']['volume']:.9g}")
    print(f"cubic volume:   {report['cubic']['volume']:.9g}")
    print(f"tet/surface:    {report['ratios']['tet_volume_over_surface_volume']:.6g}")
    print(f"cubic/surface:  {report['ratios']['cubic_volume_over_surface_volume']:.6g}")
    print(f"cubic/tet:      {report['ratios']['cubic_volume_over_tet_volume']:.6g}")
    print()
    print(f"tet negative orientations: {report['tet']['negative_orientation_count']}")
    print(f"tet mean-ratio quality:    {report['tet']['mean_ratio_quality_percentiles']}")
    print(f"tet radius-ratio quality:  {report['tet']['radius_ratio_quality_percentiles']}")
    print(f"tet low-quality counts:    {report['tet']['low_quality_counts']}")
    print(f"cubic max aspect ratio:    {report['cubic']['max_aspect_ratio']:.6g}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=CASES, default="dragon")
    parser.add_argument("--surface", type=Path)
    parser.add_argument("--tet", type=Path)
    parser.add_argument("--cubic", type=Path)
    parser.add_argument("--json", type=Path, default=None, help="Optional path for the JSON report.")
    args = parser.parse_args(argv)

    defaults = CASES[args.study]
    report = build_report(
        args.surface or defaults["surface"],
        args.tet or defaults["tet"],
        args.cubic or defaults["cubic"],
    )
    print_report(report, args.study)
    if args.json is not None:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
