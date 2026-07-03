#!/usr/bin/env python3
"""Generate examples/static_solve_dragon_gravity_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python examples/scripts/notebook_generators/generate_static_solve_dragon_gravity_demo.py
"""

from __future__ import annotations

from pathlib import Path
import sys

try:
    from notebook_builder import code, md, repo_root, write_notebook
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # Static Solve — Dragon Under Gravity

        This notebook builds a static equilibrium solve for the dragon tet
        volume mesh under gravity, using the display `dragon.obj` and the
        provided `dragon-surface-fixed.txt` pinned set.

        The pinned set is authored on the DISPLAY SURFACE, not on any
        particular volume mesh: the constraint is a soft
        `EmbeddedVertexAttachment` acting on embedded surface points through
        the formulation's surface embedding matrix. Defining constraints on
        the shared surface keeps the scene portable across simulation meshes
        and formulations (tet, cubic, tricubic Hermite).
        """
    ),
    code(
        """
        from pathlib import Path

        import numpy as np
        import pypgo as pgo
        import pypgo.energy as pe
        import pypgo.fem as pf
        import pypgo.solver as ps
        from pypgo.mesh import visualize as vis
        from pypgo.mesh.volume import VolumeMesh, read_veg
        """
    ),
    md(
        """
        ## 1. Locate the Dragon Assets

        The volume mesh is tetrahedral. The surface mesh doubles as the
        display mesh and the carrier of the pinned vertex set
        (`dragon-surface-fixed.txt` stores `dragon.obj` vertex IDs).
        """
    ),
    code(
        """
        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent.parent
        ASSET_DIR = PACKAGE_ROOT / "examples" / "assets"
        OUTPUT_DIR = PACKAGE_ROOT / "examples" / "outputs" / "static_solve_dragon_gravity"

        DRAGON_TET = ASSET_DIR / "veg" / "tet" / "dragon_big.veg"
        DRAGON_SURFACE = ASSET_DIR / "obj" / "dragon.obj"
        DRAGON_FIXED = ASSET_DIR / "fixed" / "dragon-surface-fixed.txt"

        print("volume asset:", DRAGON_TET)
        print("surface asset:", DRAGON_SURFACE)
        print("fixed asset:", DRAGON_FIXED)
        print("output dir:", OUTPUT_DIR)
        """
    ),
    md(
        """
        ## 2. Load the Tet Volume and Display Surface

        `SurfaceEmbedding` creates the interpolation from volume displacement
        DOFs to the high-resolution display surface.
        """
    ),
    code(
        """
        veg = read_veg(str(DRAGON_TET))
        volume = VolumeMesh.from_veg_file(veg)
        tet_data = volume.mesh_data

        rest_surface = pgo.mesh.read_obj(str(DRAGON_SURFACE))
        surface_embedding = pgo.mesh.SurfaceEmbedding(rest_surface, volume)

        print(volume)
        print("tet geometry:", tet_data.num_vertices, "vertices,", tet_data.num_elements, "tets")
        print("material:", volume.material)
        print("tet bbox:", tet_data.bbox)
        print("surface:", rest_surface.num_vertices, "vertices,", rest_surface.num_elements, "triangles")
        print(
            "surface interpolation:",
            surface_embedding.interpolation_matrix.shape,
            "nnz:",
            surface_embedding.interpolation_matrix.nnz,
        )

        vis.plot_surface(
            rest_surface,
            titles=["dragon rest surface"],
            colors=["lightgray"],
            show_edges=False,
            window_size=(720, 520),
        )
        """
    ),
    md(
        """
        ## 3. Load the Pinned Surface Vertices

        `dragon-surface-fixed.txt` stores zero-based vertex IDs for
        `dragon.obj`. These points are not hard constrained. They become a
        soft embedded attachment energy with coefficient `1e5`.
        """
    ),
    code(
        """
        fixed_vertices = np.loadtxt(DRAGON_FIXED, dtype=np.int64).reshape(-1)
        if fixed_vertices.size == 0:
            raise ValueError(f"{DRAGON_FIXED} does not contain any fixed IDs")
        fixed_vertices = np.unique(fixed_vertices)
        if fixed_vertices.min() < 0:
            raise ValueError("fixed vertex IDs must be non-negative")
        if fixed_vertices.max() >= rest_surface.num_vertices:
            raise ValueError(
                "fixed vertex IDs do not fit dragon.obj: "
                f"max={fixed_vertices.max()}, num_surface_vertices={rest_surface.num_vertices}"
            )

        fixed_positions = np.asarray(rest_surface.vertices)[fixed_vertices]
        print("pinned surface vertices:", fixed_vertices.size)
        print("pinned vertex ID range:", int(fixed_vertices.min()), int(fixed_vertices.max()))
        print("first pinned vertices:", fixed_vertices[:30].tolist())
        print("pinned patch bbox:", fixed_positions.min(axis=0), fixed_positions.max(axis=0))

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        fixed_points_obj = OUTPUT_DIR / "dragon_fixed_points.obj"
        vis.write_points_obj(fixed_points_obj, fixed_positions)
        print("wrote fixed points:", fixed_points_obj)

        vis.plot_points_on_mesh(
            tet_data,
            fixed_positions,
            title="pinned surface vertices over dragon_big.veg",
            mesh_opacity=0.28,
            point_color="red",
            point_size=10,
            show_edges=False,
            window_size=(900, 650),
        )
        """
    ),
    md(
        """
        ## 4. Build the Static Objective

        The tet mesh uses `TetLinear()`. Gravity is converted to a generalized
        force with the same formulation, then represented as a linear potential
        energy `-f^T u`. The pinned surface vertices are kept near their rest
        positions with `EmbeddedVertexAttachment(coeff=1e5)`, which acts
        through the formulation's surface embedding matrix — the same
        constraint definition works unchanged on cubic or tricubic Hermite
        simulation meshes.
        """
    ),
    code(
        """
        sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
        formulation = pf.TetLinear()
        deformation = pf.deformation_energy(
            sim_mesh,
            elastic=pf.StableNeo(),
            elastic_field=pf.ElementwiseField(),
            plastic=pf.VolumetricPlasticity(dofs=0),
            plastic_field=pf.ElementwiseField(),
            formulation=formulation,
        )

        mass_field = pf.volume_density(volume)
        gravity_accel = np.array([0.0, -9.81, 0.0], dtype=np.float64)
        gravity_force = formulation.body_force(sim_mesh, gravity_accel, mass_field)
        gravity_energy = pe.LinearEnergy(-gravity_force)

        surface_map = formulation.surface_embedding_matrix(volume, rest_surface.vertices)

        attachment_coeff = 1e5
        surface_attachment = pe.EmbeddedVertexAttachment(
            embedding=surface_map,
            vertex_indices=fixed_vertices,
            coeff=attachment_coeff,
        )

        total_energy = pe.EnergySet([
            (deformation, 1.0),
            (gravity_energy, 1.0),
            (surface_attachment, 1.0),
        ])

        print("mesh_type:", sim_mesh.mesh_type)
        print("vertices:", sim_mesh.num_vertices)
        print("elements:", sim_mesh.num_elements)
        print("DOFs:", deformation.num_dofs)
        print("state_kind:", deformation.state_kind)
        print("gravity force norm:", float(np.linalg.norm(gravity_force)))
        print("linear energy DOFs:", gravity_energy.num_dofs)
        print("surface attachment coeff:", attachment_coeff)
        print("surface attachment:", surface_attachment)
        """
    ),
    md(
        """
        ## 5. Initialize the Static State

        There are no hard fixed DOFs in this scene. The pinned surface
        vertices are soft constraints in the objective with coefficient `1e5`.
        """
    ),
    code(
        """
        x0 = np.zeros(deformation.num_dofs, dtype=np.float64)

        print("initial DOFs:", x0.size)
        print("surface attachment energy at x0:", surface_attachment.value(x0))
        """
    ),
    md(
        """
        ## 6. Solve the Static Equilibrium

        The static problem minimizes elastic energy, gravity potential, and
        the surface soft attachment energy. `dragon_big.veg` is a large mesh,
        so this solve can take substantially longer than the smaller example
        notebooks.
        """
    ),
    code(
        """
        problem = ps.OptimizationProblem(objective=total_energy)

        optimizer = ps.NewtonOptimizer(
            max_iterations=200,
            gradient_tolerance=1e-4,
            damping=ps.FixedDamping(),
            line_search=ps.Backtrack(),
        )
        result = optimizer.solve(problem, x0)

        print("status:", result.status.name)
        print("converged:", result.converged)
        print("iterations:", result.iterations)
        print("final objective:", result.final_objective)
        print("final gradient max norm:", result.final_gradient_max_norm)
        print("max |u|:", float(np.max(np.abs(result.x))))
        pinned_displacement = np.asarray(surface_map @ result.x).reshape((-1, 3))[fixed_vertices]
        print("surface attachment energy after solve:", surface_attachment.value(result.x))
        print("max pinned surface |u|:", float(np.max(np.abs(pinned_displacement))))
        """
    ),
    md(
        """
        ## 7. Export and Visualize the Deformed Dragon

        The optimized state is a volume displacement vector. `SurfaceEmbedding`
        maps it back onto `dragon.obj` so the high-resolution display mesh can
        be written and visualized.
        """
    ),
    code(
        """
        deformed_surface = surface_embedding.deform(result.x)

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        deformed_obj = OUTPUT_DIR / "dragon_static_solve_deformed.obj"
        pgo.mesh.write_obj(str(deformed_obj), deformed_surface)

        displacement = result.x.reshape((-1, 3))
        deformed_volume_vertices = tet_data.vertices + displacement

        print("rest tet bbox:    ", tet_data.bbox)
        print("deformed tet bbox:", (deformed_volume_vertices.min(axis=0), deformed_volume_vertices.max(axis=0)))
        print("deformed surface:", deformed_surface.num_vertices, "vertices,", deformed_surface.num_elements, "triangles")
        print("wrote OBJ:", deformed_obj)

        vis.plot_surface(
            [rest_surface, deformed_surface],
            titles=["dragon rest surface", "dragon static solve"],
            colors=["lightgray", "salmon"],
            show_edges=False,
            window_size=(1000, 520),
        )
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "examples" / "static_solve_dragon_gravity_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
