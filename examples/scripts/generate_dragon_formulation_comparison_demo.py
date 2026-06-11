"""Generate the dragon formulation comparison notebook."""

from notebook_builder import code, md, repo_root, write_notebook

CELLS = [
    md(
        """
        # Dragon Static Formulation Comparison

        Solves the hanging-dragon static equilibrium with three discretizations
        and compares the surface displacement fields on the shared display mesh
        `dragon.obj` (12847 vertices):

        | case | mesh | formulation | DOFs |
        |---|---|---|---|
        | `tet_ref` | `dragon_big.veg` (186k tets) | TetLinear | ~120k |
        | `cubic_linear` | `cubic/dragon.veg` (759 voxels) | CubicLinear | ~3.7k |
        | `cubic_hermite` | same voxel mesh | CubicTricubicHermite | ~30k |

        **Constraint design.** All three use the IDENTICAL constraint: the
        canonical surface patch `dragon-surface-fixed.txt` (289 `dragon.obj`
        vertex indices), soft-pinned at rest through each formulation's surface
        embedding matrix (`pypgo.energy.EmbeddedVertexAttachment`). Defining
        the constraint on the shared surface removes two classic comparison
        artifacts:

        1. **cross-mesh mapping error** — proximity-mapping a fixed set onto a
           different simulation mesh dilates the clamped footprint (one grid
           cell of dilation tripled the apparent stiffness in our experiments);
        2. **DOF-clamping bias** — hard-fixing tricubic Hermite vertices clamps
           all 24 DOFs including derivatives, which strangles the formulation
           (hermite degenerated to linear-level accuracy under hard clamps).

        Expected full-run result: hermite rel-L2 ~52% vs linear ~68% against
        the tet reference. The remaining gap is dominated by voxelization
        geometry (the voxel dragon has 2.44x the true volume), which
        shape-function order cannot fix.

        Runtime: `cubic_linear` ~1 s, `cubic_hermite` ~15 min, `tet_ref` ~30 min.
        A headless/CI variant of this study lives in
        `examples/scripts/dragon_formulation_comparison.py`.
        """
    ),
    code(
        """
        import json
        import time
        from pathlib import Path

        import numpy as np
        import pypgo as pgo
        from pypgo.mesh import visualize as vis
        from pypgo.tools.sim._config import load_config
        from pypgo.tools.sim._runners import run_static
        from pypgo.tools.sim._scene import build_scene
        """
    ),
    md(
        """
        ## 1. Assets and the Canonical Pinned Patch

        `dragon-surface-fixed.txt` is the single source of truth for the
        constraint. Its 289 indices address `dragon.obj` vertices, which
        coincide exactly (to 1e-16) with the legacy `dragon_big.veg` fixed
        vertices they were derived from.
        """
    ),
    code(
        """
        PACKAGE_ROOT = Path(pgo.__file__).resolve().parent.parent
        ASSET_DIR = PACKAGE_ROOT / "examples" / "assets"
        OUTPUT_ROOT = PACKAGE_ROOT / "examples" / "outputs" / "dragon_formulation_comparison"

        DRAGON_SURFACE = ASSET_DIR / "obj" / "dragon.obj"
        PATCH_FILE = ASSET_DIR / "fixed" / "dragon-surface-fixed.txt"

        rest_surface = pgo.mesh.read_obj(str(DRAGON_SURFACE))
        rest_vertices = np.asarray(rest_surface.vertices, dtype=np.float64)
        patch = np.loadtxt(PATCH_FILE, dtype=np.int64)

        print("surface:", rest_surface.num_vertices, "vertices")
        print("pinned patch:", patch.size, "vertices")
        print("patch bbox:", rest_vertices[patch].min(axis=0), rest_vertices[patch].max(axis=0))

        vis.plot_points_on_mesh(
            rest_surface,
            rest_vertices[patch],
            title="canonical pinned surface patch (289 vertices)",
            mesh_opacity=0.3,
            point_color="red",
            point_size=8,
            show_edges=False,
            window_size=(900, 650),
        )
        """
    ),
    md(
        """
        ## 2. Case Definitions

        Each case is a static solve through the sim-CLI scene pipeline. The
        ONLY differences between cases are the volume mesh and the
        formulation; the surface, the constraint, gravity, and the solver
        settings are shared.
        """
    ),
    code(
        """
        CASES = {
            "cubic_linear": {
                "mesh_type": "cubic",
                "overrides": {"mesh.volume": str(ASSET_DIR / "veg" / "cubic" / "dragon.veg")},
            },
            "cubic_hermite": {
                "mesh_type": "cubic",
                "overrides": {
                    "mesh.volume": str(ASSET_DIR / "veg" / "cubic" / "dragon.veg"),
                    "mesh.formulation": "cubic-tricubic-hermite",
                },
            },
            "tet_ref": {
                "mesh_type": "tet",
                "overrides": {"mesh.volume": str(ASSET_DIR / "veg" / "tet" / "dragon_big.veg")},
            },
        }


        def run_case(name):
            spec = CASES[name]
            overrides = {
                "mesh.surface": str(DRAGON_SURFACE),
                "constraints.surface_attachments": [
                    {"vertices": {"file": str(PATCH_FILE)}, "coeff": 1e5},
                ],
                "loads.gravity": (0.0, -9.81, 0.0),
                "solver.max_iterations": 200,
                "solver.gradient_tolerance": 1e-4,
                "output.directory": str(OUTPUT_ROOT / name),
                "output.write_surfaces": True,
                **spec["overrides"],
            }
            cfg = load_config(mesh_type=spec["mesh_type"], mode="static", overrides=overrides)
            t0 = time.perf_counter()
            summary = run_static(build_scene(cfg), cfg)
            summary["wall_seconds"] = time.perf_counter() - t0
            print(f"[{name}] converged={summary['converged']} "
                  f"iters={summary['iterations']} dofs={summary['num_dofs']} "
                  f"wall={summary['wall_seconds']:.1f}s")
            return summary
        """
    ),
    md(
        """
        ## 3. Run the Three Solves

        Ordered cheapest-first so partial results are available early.
        """
    ),
    code(
        """
        runs = {}
        runs["cubic_linear"] = run_case("cubic_linear")
        """
    ),
    code(
        """
        runs["cubic_hermite"] = run_case("cubic_hermite")   # ~15 min
        """
    ),
    code(
        """
        runs["tet_ref"] = run_case("tet_ref")               # ~30 min
        """
    ),
    md(
        """
        ## 4. Surface Displacement Metrics

        Relative L2 = RMS of the per-vertex displacement error vs the tet
        reference, normalized by the reference's own RMS. The pin residual
        verifies all three constraints behaved identically (soft pins hold
        within ~1 mm at coeff 1e5).
        """
    ),
    code(
        """
        disp = {
            name: np.asarray(
                pgo.mesh.read_obj(str(OUTPUT_ROOT / name / "final_surface.obj")).vertices,
                dtype=np.float64) - rest_vertices
            for name in runs
        }
        ref = disp["tet_ref"]
        ref_rms = np.sqrt((np.linalg.norm(ref, axis=1) ** 2).mean())

        print(f"{'case':14s} {'dofs':>7s} {'iters':>5s} {'wall':>8s} "
              f"{'max|u|':>8s} {'mean|u|':>8s} {'rel L2':>7s} {'pin res':>8s}")
        for name in ("tet_ref", "cubic_linear", "cubic_hermite"):
            d, s = disp[name], runs[name]
            nn = np.linalg.norm(d, axis=1)
            rel = np.sqrt((np.linalg.norm(d - ref, axis=1) ** 2).mean()) / ref_rms
            pin = np.linalg.norm(d[patch], axis=1).max()
            print(f"{name:14s} {s['num_dofs']:>7d} {s['iterations']:>5d} "
                  f"{s['wall_seconds']:>7.1f}s {nn.max():>8.4f} {nn.mean():>8.4f} "
                  f"{rel:>6.1%} {pin*1000:>6.2f}mm")
        """
    ),
    md(
        """
        ## 5. Visualize the Deformed Surfaces

        The reference sags visibly further than both voxel-mesh solves; the
        hermite solve recovers noticeably more of the deformation than linear
        on the same voxel mesh.
        """
    ),
    code(
        """
        deformed = {
            name: pgo.mesh.TriMeshData(rest_vertices + disp[name], rest_surface.elements)
            for name in ("tet_ref", "cubic_linear", "cubic_hermite")
        }
        vis.plot_surface(
            [deformed["tet_ref"], deformed["cubic_linear"], deformed["cubic_hermite"]],
            titles=["tet reference", "cubic linear", "cubic tricubic Hermite"],
            colors=["lightsteelblue", "lightcoral", "lightgreen"],
            show_edges=False,
            window_size=(1500, 520),
        )
        """
    ),
    md(
        """
        ## 6. Takeaways

        1. **Formulation effect (same mesh, same constraint):** tricubic
           Hermite recovers substantially more deformation than trilinear
           (rel L2 ~52% vs ~68% in our runs) at ~900x the solve cost.
        2. **Constraint design matters more than it looks:** hard-fixing
           Hermite vertices (24 DOFs incl. derivatives) erased the entire
           formulation advantage in earlier experiments; surface-embedded soft
           pins are the fair construction.
        3. **Geometry dominates the remaining gap:** the voxelized dragon
           carries 2.44x the true volume, inflating bending stiffness — a
           finer voxelization, not a higher-order formulation, is the lever
           for closing the rest.
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "examples" / "dragon_formulation_comparison_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
