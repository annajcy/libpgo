"""CLI for running a volume IPC simulation — a self-contained usage example.

This module demonstrates how the public pypgo APIs (fem / contact / energy /
solver / sim) are wired together for a volumetric dynamic IPC scene.  The
orchestration lives here rather than in a separate builder layer so every step
is visible.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import pypgo as _pgo
import pypgo.contact as _contact
import pypgo.energy as _energy
import pypgo.fem as _fem
import pypgo.solver as _solver
from pypgo._utils import vec3_finite
from pypgo.mesh import TriMeshData, read_obj
from pypgo.mesh.volume import VolumeMesh, read_veg
from pypgo.fem import SimulationMesh
from pypgo.sim import DynamicSimulation, DynamicState


# -- helper: auto-detect formulation from element width ------------------------

def _default_formulation(volume: VolumeMesh):
    element_width = int(volume.mesh_data.elements.shape[1])
    if element_width == 4:
        return _fem.TetLinear()
    if element_width == 8:
        return _fem.CubicLinear()
    raise ValueError(f"unsupported volume element width: {element_width}")


# -- helper: tile or Hermite-pad a 3-vector to n-DOFs -------------------------

def _initial_vector(volume: VolumeMesh, num_dofs: int, values, formulation) -> np.ndarray:
    vec = vec3_finite("initial vector", values)
    if num_dofs == volume.num_vertices * 3:
        return np.tile(vec, volume.num_vertices).astype(np.float64, copy=False)
    out = np.zeros(num_dofs, dtype=np.float64)
    if isinstance(formulation, _fem.CubicTricubicHermite):
        for vertex_id in range(volume.num_vertices):
            out[vertex_id * 24:vertex_id * 24 + 3] = vec
    return out


# -- main orchestration --------------------------------------------------------

def run_volume_ipc(
    veg_path: str | Path,
    surface_path: str | Path,
    output_dir: str | Path,
    *,
    timestep: float,
    num_steps: int = 1,
    gravity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    formulation: str = "auto",
    dhat: float = 0.001,
    dhat_external: float = 0.001,
    kappa: float = 1000.0,
    solver_max_iterations: int = 50,
    solver_gradient_tolerance: float = 1e-6,
    write_surfaces: bool = False,
) -> int:
    """Run a dynamic volumetric IPC simulation and write a summary JSON.

    Returns the number of frames simulated.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Load meshes
    volume = VolumeMesh.from_veg_file(read_veg(veg_path))
    surface = read_obj(surface_path)

    # 2. Pick formulation
    fm = {
        "auto": _default_formulation(volume),
        "tet-linear": _fem.TetLinear(),
        "cubic-linear": _fem.CubicLinear(),
        "cubic-tricubic-hermite": _fem.CubicTricubicHermite(),
    }[formulation]

    # 3. Build deformation energy
    sim_mesh = SimulationMesh.create_volumetric(volume)
    deformation = _fem.deformation_energy(
        sim_mesh,
        elastic=_fem.StableNeo(),
        elastic_field=_fem.ElementwiseField(),
        plastic=_fem.VolumetricPlasticity(dofs=0),
        plastic_field=_fem.ElementwiseField(),
        formulation=fm,
    )

    # 4. Mass, surface embedding, contact surface
    mass = fm.mass_matrix(volume)
    surface_map = fm.surface_embedding_matrix(volume, surface.vertices)
    contact_surface = _contact.ContactSurface.embedded(surface.vertices, surface_map)

    # 5. IPC contact energy
    ipc_params = _contact.IPCParameters(dhat=dhat, dhat_external=dhat_external, kappa=kappa)
    ipc = _contact.IPCEnergy(contact_surface, surface.elements, params=ipc_params)

    # 6. Combine energies
    energy = _energy.EnergySet([(deformation, 1.0), (ipc, 1.0)])

    # 7. Body force (gravity)
    external_force = fm.body_force(volume, np.array(gravity, dtype=np.float64))

    # 8. Initial state
    n = deformation.num_dofs
    initial_state = DynamicState(
        displacement=_initial_vector(volume, n, (0.0, 0.0, 0.0), fm),
        velocity=_initial_vector(volume, n, (0.0, 0.0, 0.0), fm),
        acceleration=np.zeros(n, dtype=np.float64),
    )

    # 9. Optimizer
    optimizer = _solver.NewtonOptimizer(
        max_iterations=solver_max_iterations,
        gradient_tolerance=solver_gradient_tolerance,
    )

    # 10. Warm-start IPC state
    if hasattr(ipc, "begin_step"):
        ipc.begin_step(time=0.0, timestep=timestep, previous_x=initial_state.displacement)

    # 11. Dynamic simulation
    sim = DynamicSimulation(
        mass=mass,
        state=initial_state,
        timestep=timestep,
        energy=energy,
    )

    # 12. Step loop
    frames = []
    for _ in range(num_steps):
        if ipc.obstacles:
            ipc.set_moving_obstacle_time(sim.state.time + timestep)
        frame = sim.step(external_force=external_force, optimizer=optimizer)
        frames.append(frame)

    # 13. Optional surface export
    if write_surfaces:
        surface_dir = output_dir / "surface"
        surface_dir.mkdir(parents=True, exist_ok=True)
        for frame in frames:
            disp = frame.displacement
            mapped = np.asarray(surface_map @ disp.reshape(-1), dtype=np.float64).reshape((-1, 3))
            deformed = TriMeshData(surface.vertices + mapped, surface.elements)
            _pgo.mesh.write_obj(
                str(surface_dir / f"surface{frame.frame_index:04d}.obj"), deformed,
            )

    # 14. Write summary
    with open(output_dir / "summary.json", "w") as f:
        json.dump(
            {
                "num_steps": num_steps,
                "num_dofs": n,
                "num_frames": len(frames),
                "final_time": sim.state.time,
                "final_timestep_id": sim.state.timestep_id,
            },
            f,
            indent=2,
        )
        f.write("\n")
    return len(frames)


# -- CLI entry point -----------------------------------------------------------

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
        choices=("auto", "tet-linear", "cubic-linear", "cubic-tricubic-hermite"),
        default="auto",
    )
    parser.add_argument("--dhat", type=float, default=0.001, help="IPC self-contact dhat")
    parser.add_argument("--dhat-external", type=float, default=0.001, help="IPC external dhat")
    parser.add_argument("--kappa", type=float, default=1000.0, help="IPC barrier stiffness")
    parser.add_argument("--solver-max-iterations", type=int, default=50)
    parser.add_argument("--solver-gradient-tolerance", type=float, default=1e-6)
    parser.add_argument("--write-surfaces", action="store_true", help="write deformed surface OBJ per frame")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    return run_volume_ipc(
        veg_path=args.veg,
        surface_path=args.surface,
        output_dir=args.output_dir,
        timestep=args.timestep,
        num_steps=args.num_steps,
        gravity=tuple(args.gravity),
        formulation=args.formulation,
        dhat=args.dhat,
        dhat_external=args.dhat_external,
        kappa=args.kappa,
        solver_max_iterations=args.solver_max_iterations,
        solver_gradient_tolerance=args.solver_gradient_tolerance,
        write_surfaces=args.write_surfaces,
    )
