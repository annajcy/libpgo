"""CLI for converting a Gmsh .msh file to a Vega .veg volume mesh."""

from __future__ import annotations

import argparse

import pypgo.mesh as _mesh
from pypgo.mesh.volume import read_msh, VolumeMesh, write_veg
from pypgo.mesh.volume import ENuMaterial, VegFile

from ._common import add_material_args, material_from_args, write_volume_outputs


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pypgo-msh-converter",
        description="Convert a Gmsh .msh tet mesh to a Vega .veg file.",
    )
    parser.add_argument("input_msh", help="input Gmsh .msh file")
    parser.add_argument("output_veg", help="output .veg file")
    add_material_args(parser)
    parser.add_argument(
        "--output-surface",
        help="optional output OBJ surface extracted from the volume mesh",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    mesh_data = read_msh(args.input_msh)
    write_volume_outputs(mesh_data, args)
    return 0
