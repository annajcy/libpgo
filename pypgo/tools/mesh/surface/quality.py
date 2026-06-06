"""CLI for checking basic OBJ surface quality."""

from __future__ import annotations

import argparse
import json

import pypgo.mesh as _mesh
from pypgo.mesh import QualityReport


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pypgo-surface-quality", description="Check basic OBJ surface quality.")
    parser.add_argument("input_obj", help="input OBJ surface")
    parser.add_argument("--short-edge-threshold", type=float, default=0.0)
    parser.add_argument("--degenerate-area-threshold", type=float, default=1e-12)
    parser.add_argument("--output", "-o", help="optional output JSON path; defaults to stdout")
    return parser


def _quality_to_dict(report: QualityReport) -> dict:
    return {
        "is_clean": report.is_clean,
        "degenerate_tris": report.degenerate_tris,
        "short_edges": report.short_edges,
        "non_manifold_edges": report.non_manifold_edges,
        "flipped_tris": report.flipped_tris,
        "has_self_intersections": report.has_self_intersections,
    }


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    report = _mesh.check_surface_quality(
        _mesh.read_obj(args.input_obj),
        short_edge_threshold=args.short_edge_threshold,
        degenerate_area_threshold=args.degenerate_area_threshold,
    )
    payload = json.dumps(_quality_to_dict(report), indent=2) + "\n"
    if args.output:
        with open(args.output, "w") as f:
            f.write(payload)
    else:
        print(payload, end="")
    return 0
