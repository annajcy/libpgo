"""Drop a cubic box-with-sphere model onto an obstacle with IPC contact."""

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
TIMESTEP = 1.0e-3
NUM_STEPS = 2000
DUMP_INTERVAL = 10


def main() -> None:
    # Load the cubic simulation mesh, render surface, and obstacle.
    volume = pgo.mesh.volume.VolumeMesh.from_veg_file(
        pgo.mesh.volume.read_veg(
            str(ASSET_DIR / "veg" / "cubic" / "box-with-sphere.veg")
        )
    )
    surface = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "box-with-sphere.obj"))
    obstacle = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "bottom.obj"))

    # Build deformation, mass, gravity, and embedded IPC contact.
    simulation_mesh = pf.SimulationMesh.create_volumetric(volume)
    formulation = pf.CubicLinear()
    deformation = pf.deformation_energy(
        simulation_mesh,
        elastic=pf.StableNeo(),
        elastic_field=pf.ElementwiseField(),
        plastic=pf.VolumetricPlasticity(dofs=0),
        plastic_field=pf.ElementwiseField(),
        formulation=formulation,
        options=pf.DeformationOptions(enable_material_max_step=False),
    )
    mass_field = pf.volume_density(volume)
    mass = formulation.mass_matrix(simulation_mesh, mass_field)
    gravity_force = formulation.body_force(
        simulation_mesh,
        np.array([0.0, -9.81, 0.0]),
        mass_field,
    )
    surface_map = formulation.surface_embedding_matrix(volume, surface.vertices)
    contact_surface = pc.ContactSurface.embedded(surface.vertices, surface_map)
    ipc = pc.IPCEnergy(
        contact_surface,
        surface.elements,
        params=pc.IPCParameters(
            dhat=2.0e-3,
            dhat_external=5.0e-3,
            kappa=3.0e3,
        ),
        obstacles=[pc.ObstacleSpec.static(obstacle.vertices, obstacle.elements)],
    )

    # Advance implicit dynamics and write selected surface frames.
    state = pgs.DynamicState(
        displacement=np.zeros(deformation.num_dofs),
        velocity=np.zeros(deformation.num_dofs),
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
        gradient_tolerance=1.0e-3,
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
            frame_vertices = surface.vertices + (
                surface_map @ frame.displacement
            ).reshape((-1, 3))
            final_surface = pgo.mesh.TriMeshData(frame_vertices, surface.elements)
            pgo.mesh.write_obj(
                str(OUTPUT_DIR / f"surface{step:04d}.obj"),
                final_surface,
            )
            print(f"step {step:4d}  iterations={frame.solver_result.iterations}")

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
