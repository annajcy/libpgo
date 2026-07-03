"""CLI for voxelizing an OBJ surface into a cubic volume mesh."""

from __future__ import annotations

import argparse
import pypgo.mesh as _mesh

from ._common import add_material_args, write_volume_outputs


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pypgo-cubic-mesher",
        description="Voxelize a closed OBJ surface into a cubic .veg mesh.",
    )
    parser.add_argument("input_obj", help="input closed OBJ surface")
    parser.add_argument("output_veg", help="output .veg file")
    parser.add_argument("--resolution", type=int, required=True, help="voxel resolution along the shortest axis")
    parser.add_argument("--occupancy", choices=("conservative", "center"), default="conservative")
    add_material_args(parser)
    parser.add_argument("--output-surface", help="optional output OBJ surface extracted from the generated volume")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    surface = _mesh.read_obj(args.input_obj)
    mesh_data = _mesh.cubic_mesher(
        surface,
        resolution=args.resolution,
        occupancy=args.occupancy,
        E=args.E,
        nu=args.nu,
        density=args.density,
    )
    write_volume_outputs(mesh_data, args)
    return 0
