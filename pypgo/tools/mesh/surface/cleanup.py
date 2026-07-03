"""CLI for conservative cleanup of an OBJ surface mesh."""

from __future__ import annotations

import argparse

import pypgo.mesh as _mesh

from ._common import add_surface_io_args, read_surface, write_json, write_surface


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pypgo-surface-cleanup",
        description="Conservatively clean an OBJ surface mesh.",
    )
    add_surface_io_args(parser)
    parser.add_argument("--json", required=True, help="output JSON cleanup report")
    parser.add_argument(
        "--expected-components",
        type=int,
        help="expected edge-connected component count; defaults to the input count",
    )
    parser.add_argument("--short-edge-threshold", type=float, default=1e-5)
    parser.add_argument("--max-passes", type=int, default=3)
    parser.add_argument("--max-collapses", type=int, default=10000)
    parser.add_argument("--merge-close-vertices", action="store_true", help="merge nearby vertices before cleanup")
    parser.add_argument("--eps", type=float, help="merge distance for --merge-close-vertices")
    parser.add_argument("--repair-self-intersections", action="store_true", help="repair self-intersections before cleanup")
    parser.add_argument(
        "--repair-method",
        default="autorefine",
        choices=["autorefine", "autorefine-only", "remove"],
        help="self-intersection repair method",
    )
    parser.add_argument("--remove-isolated-vertices", action="store_true", help="remove isolated vertices after cleanup")
    parser.add_argument("--dry-run", action="store_true", help="write report without writing output mesh")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    surface = read_surface(args.input_mesh)
    pre_steps = {}

    if args.merge_close_vertices:
        merged = _mesh.merge_close_vertices(surface, eps=args.eps)
        surface = merged.surface
        pre_steps["merge_close_vertices"] = {
            "enabled": True,
            "merged_vertices": merged.merged_vertices,
            "eps": merged.eps,
        }
    else:
        pre_steps["merge_close_vertices"] = {"enabled": False}

    if args.repair_self_intersections:
        surface, all_fixed = _mesh.cgal_repair_self_intersections(surface, method=args.repair_method)
        pre_steps["repair_self_intersections"] = {
            "enabled": True,
            "method": args.repair_method,
            "all_fixed": all_fixed,
        }
    else:
        pre_steps["repair_self_intersections"] = {"enabled": False}

    result = _mesh.raw_surface_cleanup(
        surface,
        expected_components=args.expected_components,
        short_edge_threshold=args.short_edge_threshold,
        max_passes=args.max_passes,
        max_collapses=args.max_collapses,
    )
    output_surface = result.surface
    if args.remove_isolated_vertices:
        output_surface = _mesh.remove_isolated_vertices(output_surface)

    if not args.dry_run:
        write_surface(args.output_mesh, output_surface)
    report = result.report.to_dict()
    report["input"] = args.input_mesh
    report["output"] = args.output_mesh
    report["dry_run"] = args.dry_run
    report.update(pre_steps)
    report["remove_isolated_vertices"] = {"enabled": args.remove_isolated_vertices}
    write_json(args.json, report)
    return 0
