"""CLI for running a Python volume IPC simulation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pypgo as _pgo
import pypgo.fem as _fem
import pypgo.mesh as _mesh
from pypgo.mesh.volume import VolumeMesh, read_veg
from pypgo.sim import IPCContactSpec, RuntimeConfig, VolumeIPCSimulationSpec, build_volume_ipc_simulation


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pypgo-volume-ipc", description="Run a Python volume IPC simulation.")
    parser.add_argument("--veg", required=True, help="input volume .veg file")
    parser.add_argument("--surface", required=True, help="input embedded/contact OBJ surface")
    parser.add_argument("--output-dir", required=True, help="output directory")
    parser.add_argument("--num-steps", type=int, default=1, help="number of dynamic steps to run")
    parser.add_argument("--timestep", type=float, required=True, help="time step size")
    parser.add_argument("--gravity", nargs=3, type=float, default=(0.0, 0.0, 0.0), metavar=("GX", "GY", "GZ"))
    parser.add_argument(
        "--formulation",
        choices=("auto", "tet-p1", "linear-cubic", "tricubic-hermite"),
        default="auto",
    )
    parser.add_argument("--dhat", type=float, default=0.001, help="IPC self-contact dhat")
    parser.add_argument("--dhat-external", type=float, default=0.001, help="IPC external dhat")
    parser.add_argument("--kappa", type=float, default=1000.0, help="IPC barrier stiffness")
    parser.add_argument("--solver-max-iterations", type=int, default=50)
    parser.add_argument("--solver-gradient-tolerance", type=float, default=1e-6)
    parser.add_argument("--write-surfaces", action="store_true", help="write deformed surface OBJ per frame")
    return parser


def _formulation(name: str):
    if name == "auto":
        return None
    if name == "tet-p1":
        return _fem.TetP1()
    if name == "linear-cubic":
        return _fem.LinearCubic()
    if name == "tricubic-hermite":
        return _fem.TricubicHermite()
    raise ValueError(f"unsupported formulation: {name}")


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    volume = VolumeMesh.from_veg_file(read_veg(args.veg))
    surface = _mesh.read_obj(args.surface)
    spec = VolumeIPCSimulationSpec(
        volume=volume,
        surface=surface,
        formulation=_formulation(args.formulation),
        runtime=RuntimeConfig(
            timestep=args.timestep,
            num_steps=args.num_steps,
            gravity=args.gravity,
            solver_max_iterations=args.solver_max_iterations,
            solver_gradient_tolerance=args.solver_gradient_tolerance,
        ),
        contact=IPCContactSpec(
            parameters=_pgo.contact.IPCParameters(
                dhat=args.dhat,
                dhat_external=args.dhat_external,
                kappa=args.kappa,
            )
        ),
    )
    runner = build_volume_ipc_simulation(spec)
    frames = runner.run()

    if args.write_surfaces:
        surface_dir = output_dir / "surface"
        surface_dir.mkdir(parents=True, exist_ok=True)
        for frame in frames:
            _mesh.write_obj(
                str(surface_dir / f"surface{frame.frame_index:04d}.obj"),
                runner.deformed_surface(frame.displacement),
            )

    with open(output_dir / "summary.json", "w") as f:
        json.dump(
            {
                "num_steps": int(args.num_steps),
                "num_dofs": runner.num_dofs,
                "num_frames": len(frames),
                "final_time": runner.state.time,
                "final_timestep_id": runner.state.timestep_id,
            },
            f,
            indent=2,
        )
        f.write("\n")
    return 0
