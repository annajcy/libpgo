"""CLI for converting animation sequences to Alembic .abc files."""

from __future__ import annotations

import argparse

from pypgo.animation import dump_animation


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pypgo-animation-convert",
        description="Convert animation sequences defined in a JSON config to Alembic .abc files.",
    )
    parser.add_argument("config", help="JSON config file path")
    parser.add_argument(
        "-o", "--output-folder", default=None,
        help="output directory (default: config file's output-folder field or its directory)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    dump_animation(args.config, args.output_folder)
    return 0
