"""CLI for remeshing an OBJ surface mesh."""

from __future__ import annotations

import argparse

import pypgo.mesh as _mesh

from ._common import average_triangle_edge_length, add_surface_io_args, read_surface, write_surface


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pypgo-surface-remesh",
        description="Remesh an OBJ surface mesh.",
    )
    add_surface_io_args(parser)
    edge_length = parser.add_mutually_exclusive_group(required=True)
    edge_length.add_argument("--target-edge-length", type=float, help="absolute target edge length")
    edge_length.add_argument(
        "-l",
        "--edge-length-scale",
        type=float,
        help="target edge length as a multiplier of the input average edge length",
    )
    parser.add_argument("--iterations", type=int, default=10, help="number of remeshing iterations")
    parser.add_argument(
        "-s",
        "--sharp-edge-angle",
        type=float,
        default=180.0,
        help="feature edge angle threshold in degrees",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    surface = read_surface(args.input_mesh)
    target_edge_length = args.target_edge_length
    if target_edge_length is None:
        target_edge_length = args.edge_length_scale * average_triangle_edge_length(surface)
    output = _mesh.cgal_isotropic_remesh(
        surface,
        target_edge_length=target_edge_length,
        num_iter=args.iterations,
        sharp_angle=args.sharp_edge_angle,
    )
    write_surface(args.output_mesh, output)
    return 0
