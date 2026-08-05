"""Drop a Koiter shell onto an obstacle with IPC contact."""

from importlib.util import find_spec
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.contact as pc
import pypgo.energy as pe
import pypgo.fem as pf
import pypgo.sim as pgs
import pypgo.solver as ps


CASE_DIR = Path(__file__).resolve().parent
EXAMPLES_DIR = CASE_DIR.parents[2]
ASSET_DIR = EXAMPLES_DIR / "assets"
OUTPUT_DIR = CASE_DIR / "output"
TIMESTEP = 5.0e-4
NUM_STEPS = 80
DUMP_INTERVAL = 10


def _material(mesh, E, nu, thickness):
    elastic = pf.KoiterStVKDefinition()
    plastic = pf.ShellPlasticityDefinition(dofs=0)
    binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(
            elastic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
        pf.PlasticMaterialBinding(
            plastic, mesh.num_elements,
            np.empty((mesh.num_elements, 0), dtype=np.float64)),
    )
    element_values = np.broadcast_to(
        np.array([E, nu, E, nu, thickness], dtype=np.float64),
        (mesh.num_elements, elastic.num_optimizable_channels),
    ).copy()
    state = pf.MaterialState(
        element_values, np.empty(0))
    return binding, state


def main() -> None:
    # Load the shell and its obstacle.
    surface = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "shell.obj"))
    obstacle = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "bottom.obj"))

    # Build Koiter shell deformation, mass, gravity, and IPC contact.
    E, nu, thickness = 1.0e6, 0.4, 1.0e-3
    mesh = pf.SimulationMesh(surface)
    formulation = pf.KoiterShell()
    material_binding, material_state = _material(mesh, E, nu, thickness)
    deformation_operator = pf.DeformationEnergyOperator(
        mesh, material_binding,
        formulation=formulation,
    )
    deformation = pf.DeformationPotentialEnergy(
        deformation_operator, material_state)
    areal_density = 1000.0 * 1.0e-3
    mass = formulation.mass_matrix(mesh, areal_density)
    gravity_force = formulation.body_force(
        mesh,
        np.array([0.0, -9.81, 0.0]),
        areal_density,
    )
    contact_surface = pc.ContactSurface.identity(surface.vertices)
    ipc = pc.IPCEnergy(
        contact_surface,
        surface.elements,
        params=pc.IPCParameters(dhat=5.0e-3, kappa=1.0e3),
        obstacles=[pc.ObstacleSpec.static(obstacle.vertices, obstacle.elements)],
    )

    # Advance the shell with a downward initial velocity.
    state = pgs.DynamicState(
        displacement=np.zeros(deformation.num_dofs),
        velocity=np.tile(np.array([0.0, -2.0, 0.0]), surface.num_vertices),
        acceleration=np.zeros(deformation.num_dofs),
    )
    simulation = pgs.DynamicSimulation(
        mass=mass,
        energy=pe.EnergySet([(deformation, 1.0), (ipc, 1.0)]),
        state=state,
        timestep=TIMESTEP,
        integrator=pgs.BackwardEulerDynamicStepper(),
    )
    optimizer = ps.NewtonOptimizer(
        max_iterations=200,
        gradient_tolerance=1.0e-4,
        line_search=ps.Simple(),
    )
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    final_surface = surface

    for step in range(1, NUM_STEPS + 1):
        frame = simulation.step(external_force=gravity_force, optimizer=optimizer)
        if not frame.accepted:
            raise RuntimeError(
                f"frame {frame.frame_index} rejected: {frame.solver_result.status.name}"
            )
        if step % DUMP_INTERVAL == 0 or step == NUM_STEPS:
            final_surface = pgo.mesh.TriMeshData(
                surface.vertices + frame.displacement.reshape((-1, 3)),
                surface.elements,
            )
            pgo.mesh.write_obj(
                str(OUTPUT_DIR / f"surface{step:04d}.obj"),
                final_surface,
            )
            print(f"step {step:3d}  iterations={frame.solver_result.iterations}")

    if find_spec("pyvista") is None:
        print("PyVista is not installed; skipping visualization.")
    else:
        pgo.mesh.plot_surface(
            [surface, final_surface, obstacle],
            titles=["rest", "final", "obstacle"],
            colors=["lightgray", "salmon", "steelblue"],
            show_edges=False,
            window_size=(1200, 420),
        )
    print("saved frames ->", OUTPUT_DIR)


if __name__ == "__main__":
    main()
