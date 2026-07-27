"""Drop a cubic box-with-sphere model onto an obstacle with IPC contact."""

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
TIMESTEP = 1.0e-3
NUM_STEPS = 2000
DUMP_INTERVAL = 10


def _material_state(asset):
    elastic = pf.StableNeoDefinition()
    plastic = pf.VolumetricPlasticityDefinition(dofs=0)
    def identity_field(field_type, names):
        count = len(names)
        return field_type(
            names,
            pf.ElementwiseParameterLayout(asset.num_elements, count),
            pf.IdentityMaterialChannelMapping(count))

    elastic_fixed = identity_field(pf.FixedParameterField, elastic.fixed_channel_names)
    plastic_fixed = identity_field(pf.FixedParameterField, plastic.fixed_channel_names)
    elastic_opt = identity_field(
        pf.OptimizableParameterField, elastic.optimizable_channel_names)
    plastic_opt = identity_field(
        pf.OptimizableParameterField, plastic.optimizable_channel_names)
    parameterization = pf.MaterialParameterization(
        pf.ElasticParameterization(elastic, elastic_fixed, elastic_opt),
        pf.PlasticParameterization(plastic, plastic_fixed, plastic_opt))

    def parameter_block(fixed_field, optimizable_field):
        return pf.MaterialParameterDataBlock(
            fixed_values=np.asarray(
                pf.project_imported_material_inputs(
                    asset.material_catalog, fixed_field),
                dtype=np.float64,
            ).reshape(-1),
            initial_optimizable_values=np.zeros(
                optimizable_field.num_global_parameters, dtype=np.float64),
        )

    parameter_data = pf.MaterialParameterData(
        elastic=parameter_block(elastic_fixed, elastic_opt),
        plastic=parameter_block(plastic_fixed, plastic_opt),
    )
    parameterization.validate(parameter_data)
    return parameterization, parameter_data


def main() -> None:
    # Load the cubic simulation mesh, render surface, and obstacle.
    volume = pgo.mesh.volume.VolumeMesh(
        pgo.mesh.volume.read_veg(
            str(ASSET_DIR / "veg" / "cubic" / "box-with-sphere.veg")
        )
    )
    surface = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "box-with-sphere.obj"))
    obstacle = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "bottom.obj"))

    # Build deformation, mass, gravity, and embedded IPC contact.
    asset = pf.SimulationImportResult(volume)
    formulation = pf.CubicLinear()
    parameterization, parameter_data = _material_state(asset)
    assignment = pf.MaterialAssignment(
        mesh=asset.mesh,
        parameterization=parameterization,
        parameter_data=parameter_data,
        material_frames=pf.GlobalAxesMaterialFrameField(asset.num_elements))
    deformation = pf.DeformationEnergy(
        assignment,
        formulation=formulation,
        options=pf.DeformationOptions(enable_material_max_step=False),
    )
    mass_field = pf.volume_density(volume)
    mass = formulation.mass_matrix(asset.mesh, mass_field)
    gravity_force = formulation.body_force(
        asset.mesh,
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
