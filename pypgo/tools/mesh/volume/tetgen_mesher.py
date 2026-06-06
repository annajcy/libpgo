"""CLI for tetrahedralizing an OBJ surface with TetGen."""

from __future__ import annotations

import argparse

import pypgo.mesh as _mesh

from ._common import add_material_args, write_volume_outputs


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pypgo-tetgen-mesher",
        description="Tetrahedralize a closed OBJ surface into a tet .veg mesh with TetGen.",
    )
    parser.add_argument("input_obj", help="input closed OBJ surface")
    parser.add_argument("output_veg", help="output .veg file")
    parser.add_argument("--command", default="pq1.414", help="TetGen command string")
    add_material_args(parser)
    parser.add_argument("--output-surface", help="optional output OBJ surface extracted from the generated volume")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    surface = _mesh.read_obj(args.input_obj)
    mesh_data = _mesh.tet_mesher(
        surface,
        backend="tetgen",
        config={"command": args.command},
    )
    write_volume_outputs(mesh_data, args)
    return 0
