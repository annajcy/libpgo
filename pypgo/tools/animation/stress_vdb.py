"""CLI for exporting per-tet von Mises stress to an OpenVDB sequence."""

from __future__ import annotations

import argparse

from pypgo.animation import dump_stress_vdb


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="pypgo-stress-vdb",
        description="Splat per-tet von Mises stress into a per-frame OpenVDB sequence.",
    )
    parser.add_argument("veg_path", help="input .veg tet mesh file")
    parser.add_argument("sim_output", help="simulation output folder (contains states/ and stress/)")
    parser.add_argument("output_dir", help="destination folder for .vdb files")
    parser.add_argument("--prefix", default="vonMises", help="output filename prefix")
    parser.add_argument("--voxel-size", type=float, default=0.0,
                        help="VDB voxel size (<=0 auto-derives)")
    parser.add_argument("--frame-start", type=int, default=0,
                        help="first frame index (inclusive)")
    parser.add_argument("--frame-end", type=int, default=-1,
                        help="last frame index (exclusive, -1 auto-detects)")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    n = dump_stress_vdb(
        args.veg_path,
        args.sim_output,
        args.output_dir,
        prefix=args.prefix,
        voxel_size=args.voxel_size,
        frame_start=args.frame_start,
        frame_end=args.frame_end,
    )
    print(f"Wrote {n} frame(s) to {args.output_dir}")
    return 0
