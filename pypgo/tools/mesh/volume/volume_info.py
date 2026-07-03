"""CLI for printing volumetric mesh summaries."""

from __future__ import annotations

import argparse

from pypgo.mesh import volume_mesh_info
from pypgo.mesh.volume import read_veg


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pypgo-volume-info", description="Print a volumetric mesh summary.")
    parser.add_argument("veg", help="input .veg file")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    print(volume_mesh_info(read_veg(args.veg)))
    return 0
