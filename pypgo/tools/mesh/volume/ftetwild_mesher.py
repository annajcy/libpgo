"""CLI for tetrahedralizing an OBJ surface with fTetWild."""

from __future__ import annotations

import argparse

import pypgo.mesh as _mesh

from ._common import add_material_args, write_volume_outputs


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pypgo-ftetwild-mesher",
        description="Tetrahedralize a closed OBJ surface into a tet .veg mesh with fTetWild.",
    )
    parser.add_argument("input_obj", help="input closed OBJ surface")
    parser.add_argument("output_veg", help="output .veg file")
    parser.add_argument("--lr", type=float, default=0.05, help="relative ideal edge length")
    parser.add_argument("--la", type=float, help="absolute ideal edge length")
    parser.add_argument("--epsr", type=float, default=0.001, help="relative epsilon")
    parser.add_argument("--stop-energy", type=float, default=10.0, help="stop energy")
    parser.add_argument("--max-threads", type=int, default=0, help="max threads; 0 uses backend default")
    add_material_args(parser)
    parser.add_argument("--output-surface", help="optional output OBJ surface extracted from the generated volume")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    config = {
        "lr": args.lr,
        "epsr": args.epsr,
        "stop_energy": args.stop_energy,
        "max_threads": args.max_threads,
    }
    if args.la is not None:
        config["la"] = args.la
    surface = _mesh.read_obj(args.input_obj)
    mesh_data = _mesh.tet_mesher(surface, backend="tetwild", config=config)
    write_volume_outputs(mesh_data, args)
    return 0
