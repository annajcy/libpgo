"""Solve a tetrahedral dragon under gravity with soft surface pins."""

from importlib.util import find_spec
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.energy as pe
import pypgo.fem as pf
import pypgo.solver as ps


CASE_DIR = Path(__file__).resolve().parent
EXAMPLES_DIR = CASE_DIR.parents[2]
ASSET_DIR = EXAMPLES_DIR / "assets"
OUTPUT_DIR = CASE_DIR / "output"


def _material(asset):
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
    def fixed_parameters(field):
        return pf.FixedMaterialParameters(
            field,
            np.asarray(pf.project_imported_material_inputs(
                asset.material_catalog, field), dtype=np.float64).reshape(-1),
        )

    binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(
            elastic, fixed_parameters(elastic_fixed), elastic_opt),
        pf.PlasticMaterialBinding(
            plastic, fixed_parameters(plastic_fixed), plastic_opt),
        pf.GlobalAxesMaterialFrameField(asset.num_elements),
    )
    state = pf.MaterialState(
        np.zeros(elastic_opt.num_global_parameters),
        np.zeros(plastic_opt.num_global_parameters),
    )
    return binding, state


def main() -> None:
    # Load the tetrahedral volume and its render surface.
    volume = pgo.mesh.volume.VolumeMesh(
        pgo.mesh.volume.read_veg(str(ASSET_DIR / "veg" / "tet" / "dragon.veg"))
    )
    surface = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "dragon.obj"))
    fixed_surface_vertices = np.loadtxt(
        ASSET_DIR / "fixed" / "dragon-surface-fixed.txt",
        dtype=np.int64,
    ).reshape(-1)

    # Build the FEM energy, gravity, and surface attachment.
    asset = pf.SimulationImportResult(volume)
    formulation = pf.TetLinear()
    material_binding, material_state = _material(asset)
    deformation_operator = pf.DeformationEnergyOperator(
        asset.mesh, material_binding,
        formulation=formulation,
    )
    deformation = pf.DeformationPotentialEnergy(
        deformation_operator, material_state)
    mass_field = pf.volume_density(volume)
    gravity_force = formulation.body_force(
        asset.mesh,
        np.array([0.0, -9.81, 0.0]),
        mass_field,
    )
    surface_map = formulation.surface_embedding_matrix(volume, surface.vertices)
    attachment = pe.EmbeddedVertexAttachment(
        embedding=surface_map,
        vertex_indices=fixed_surface_vertices,
        coeff=1.0e5,
    )

    # Minimize elastic, gravity, and attachment energies.
    objective = pe.EnergySet(
        [
            (deformation, 1.0),
            (pe.LinearEnergy(-gravity_force), 1.0),
            (attachment, 1.0),
        ]
    )
    result = ps.NewtonOptimizer(
        max_iterations=200,
        gradient_tolerance=1.0e-4,
        line_search=ps.Simple(),
    ).solve(ps.OptimizationProblem(objective=objective), deformation.zero_state())

    # Map the simulation displacement back to the render surface.
    deformed_vertices = surface.vertices + (surface_map @ result.x).reshape((-1, 3))
    deformed_surface = pgo.mesh.TriMeshData(deformed_vertices, surface.elements)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "final_surface.obj"
    pgo.mesh.write_obj(str(output_path), deformed_surface)
    if find_spec("pyvista") is None:
        print("PyVista is not installed; skipping visualization.")
    else:
        pgo.mesh.plot_surface(
            [surface, deformed_surface],
            titles=["rest", "gravity equilibrium"],
            colors=["lightgray", "salmon"],
            show_edges=False,
        )

    print(f"status: {result.status.name}, iterations: {result.iterations}")
    print("saved surface ->", output_path)


if __name__ == "__main__":
    main()
