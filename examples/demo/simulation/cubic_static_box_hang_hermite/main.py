"""Solve a hanging cubic box with the tricubic Hermite formulation."""

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
    # Load the cubic volume and its embedded render surface.
    volume = pgo.mesh.volume.VolumeMesh.from_veg_file(
        pgo.mesh.volume.read_veg(str(ASSET_DIR / "veg" / "cubic" / "box.veg"))
    )
    surface = pgo.mesh.read_obj(str(ASSET_DIR / "obj" / "box.obj"))
    rest_vertices = volume.mesh_data.vertices

    # Build a 24-DOF-per-vertex tricubic Hermite model.
    simulation_mesh = pf.SimulationMesh.create_volumetric(volume)
    formulation = pf.CubicTricubicHermite()
    deformation = pf.deformation_energy(
        simulation_mesh,
        elastic=pf.StableNeo(),
        plastic=pf.VolumetricPlasticity(dofs=0),
        formulation=formulation,
    )
    mass_field = pf.volume_density(volume)
    gravity_force = formulation.body_force(
        simulation_mesh,
        np.array([0.0, -9.81, 0.0]),
        mass_field,
    )

    # Clamp every Hermite DOF on the top face.
    top_vertices = np.flatnonzero(
        np.abs(rest_vertices[:, 1] - rest_vertices[:, 1].max()) <= 1.0e-3
    )
    dofs_per_vertex = deformation.num_dofs // volume.num_vertices
    fixed_dofs = (
        top_vertices[:, None] * dofs_per_vertex + np.arange(dofs_per_vertex)
    ).ravel()
    problem = ps.OptimizationProblem(
        objective=pe.EnergySet(
            [(deformation, 1.0), (pe.LinearEnergy(-gravity_force), 1.0)]
        )
    )
    problem.fix_variables(
        fixed_dofs.tolist(),
        np.zeros(fixed_dofs.size),
        num_dofs=deformation.num_dofs,
    )
    result = ps.NewtonOptimizer(
        max_iterations=200,
        gradient_tolerance=1.0e-4,
        line_search=ps.Simple(),
    ).solve(problem, deformation.zero_state())

    # Interpolate Hermite displacement onto the render surface.
    surface_map = formulation.surface_embedding_matrix(volume, surface.vertices)
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
            titles=["rest", "Hermite equilibrium"],
            colors=["lightgray", "salmon"],
            show_edges=True,
        )

    print(f"DOFs per vertex: {dofs_per_vertex}")
    print(f"status: {result.status.name}, iterations: {result.iterations}")
    print("saved surface ->", output_path)


if __name__ == "__main__":
    main()
