"""Shared argparse layer for the pypgo-sim CLI family.

Precedence: defaults < JSON (--config) < explicit CLI flags.
CLI-given paths resolve against the CWD; JSON paths against the JSON dir.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pypgo.tools.sim._config import (
    ConfigError, INTEGRATORS, VOLUME_FORMULATIONS, load_config,
)


def build_parser(*, prog: str, mesh_type: str, mode: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=prog,
        description=f"Run a {mesh_type} {mode} simulation "
                    f"(defaults < JSON --config < CLI flags).",
    )
    parser.add_argument("--config", type=Path, default=None,
                        help="JSON scene config; paths inside resolve relative to it")
    if mesh_type == "shell":
        parser.add_argument("--surface", type=Path, default=None,
                            help="shell OBJ mesh")
    else:
        parser.add_argument("--volume", type=Path, default=None,
                            help="input .veg volume mesh")
        parser.add_argument("--surface", type=Path, default=None,
                            help="embedded/contact OBJ surface")
        parser.add_argument("--formulation", choices=VOLUME_FORMULATIONS,
                            default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--write-surfaces", action="store_true", default=None,
                        help="write deformed surface OBJ output")
    parser.add_argument("--write-checkpoints", action="store_true", default=None,
                        help="write restart checkpoints at every dynamic frame")
    parser.add_argument("--gravity", nargs=3, type=float, default=None,
                        metavar=("GX", "GY", "GZ"))
    parser.add_argument("--solver-max-iterations", type=int, default=None)
    parser.add_argument("--solver-gradient-tolerance", type=float, default=None)
    parser.add_argument("--solver-verbose", type=int, default=None)
    parser.add_argument("--solver-damping", action="store_true", default=None)
    parser.add_argument("--solver-damping-scale", type=float, default=None)
    if mode == "dynamic":
        parser.add_argument("--timestep", type=float, default=None)
        parser.add_argument("--num-steps", type=int, default=None)
        parser.add_argument("--integrator", choices=INTEGRATORS, default=None)
        parser.add_argument("--damping", nargs=2, type=float, default=None,
                            metavar=("MASS", "STIFFNESS"))
        parser.add_argument("--resume", default=None,
                            help="'latest' or a checkpoint .npz path")
    return parser


def _overrides_from_args(args, *, mesh_type: str, mode: str) -> dict:
    overrides: dict = {}

    def put(dotted, value):
        if value is not None:
            overrides[dotted] = value

    def put_path(dotted, value):
        if value is not None:
            overrides[dotted] = str(Path(value).resolve())

    if mesh_type != "shell":
        put_path("mesh.volume", args.volume)
        put("mesh.formulation", args.formulation)
    put_path("mesh.surface", args.surface)
    put_path("output.directory", args.output_dir)
    put("output.write_surfaces", args.write_surfaces)
    put("output.write_checkpoints", args.write_checkpoints)
    put("loads.gravity", tuple(args.gravity) if args.gravity is not None else None)
    put("solver.max_iterations", args.solver_max_iterations)
    put("solver.gradient_tolerance", args.solver_gradient_tolerance)
    put("solver.verbose", args.solver_verbose)
    put("solver.damping", args.solver_damping)
    put("solver.damping_scale", args.solver_damping_scale)
    if mode == "dynamic":
        put("dynamic.timestep", args.timestep)
        put("dynamic.num_steps", args.num_steps)
        put("dynamic.integrator", args.integrator)
        put("dynamic.damping",
            tuple(args.damping) if args.damping is not None else None)
        if args.resume is not None:
            resume = "latest" if args.resume == "latest" else str(Path(args.resume).resolve())
            put("dynamic.resume", resume)
    return overrides


def run_cli(*, mesh_type: str, mode: str, prog: str, argv=None) -> int:
    parser = build_parser(prog=prog, mesh_type=mesh_type, mode=mode)
    args = parser.parse_args(argv)
    from pypgo.tools.sim._runners import run_dynamic, run_static
    from pypgo.tools.sim._scene import build_scene

    try:
        cfg = load_config(
            mesh_type=mesh_type, mode=mode, json_path=args.config,
            overrides=_overrides_from_args(args, mesh_type=mesh_type, mode=mode))
        bundle = build_scene(cfg)
        runner = run_static if mode == "static" else run_dynamic
        summary = runner(bundle, cfg)
    except ConfigError as exc:
        parser.error(str(exc))  # prints to stderr and exits with code 2
    print(f"{prog}: wrote {cfg.output.directory / 'summary.json'}")
    if mode == "dynamic":
        print(f"{prog}: {summary['num_frames']} frames, "
              f"final time {summary['final_time']:.6f}")
    else:
        print(f"{prog}: converged={summary['converged']} "
              f"iterations={summary['iterations']}")
    return 0
