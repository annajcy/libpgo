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


def main() -> None:
    # Load the tetrahedral volume and its render surface.
    volume = pgo.mesh.volume.VolumeMesh.from_veg_file(
        pgo.mesh.volume.read_veg(str(ASSET_DIR / "veg" / "tet" / "dragon.veg"))
    )
    surface = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "dragon.obj"))
    fixed_surface_vertices = np.loadtxt(
        ASSET_DIR / "fixed" / "dragon-surface-fixed.txt",
        dtype=np.int64,
    ).reshape(-1)

    # Build the FEM energy, gravity, and surface attachment.
    simulation_mesh = pf.SimulationMesh.create_volumetric(volume)
    formulation = pf.TetLinear()
    deformation = pf.deformation_energy(
        simulation_mesh,
        elastic=pf.StableNeo(),
        elastic_field=pf.ElementwiseField(),
        plastic=pf.VolumetricPlasticity(dofs=0),
        plastic_field=pf.ElementwiseField(),
        formulation=formulation,
    )
    mass_field = pf.volume_density(volume)
    gravity_force = formulation.body_force(
        simulation_mesh,
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
