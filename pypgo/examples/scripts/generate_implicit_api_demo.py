#!/usr/bin/env python3
"""Generate pypgo/examples/implicit_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_implicit_api_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # pypgo.implicit API Demo

        This notebook introduces the lazy implicit surface API:
        `GridSpec`, analytic fields, CSG operators, grid sampling,
        marching-cubes extraction, and mesh surface thickening.
        """
    ),
    code(
        """
        import numpy as np
        import pypgo as pgo
        from pypgo import implicit
        """
    ),
    md(
        """
        ## 1. Analytic fields

        `SphereField` and `BoxField` are `ImplicitField` objects. They can be
        queried at points without allocating a sampled grid.
        """
    ),
    code(
        """
        sphere = implicit.SphereField([0.0, 0.0, 0.0], 1.0)
        box = implicit.BoxField([0.0, 0.0, 0.0], [0.75, 0.75, 0.75])

        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.5, 0.0, 0.0],
            ],
            dtype=np.float64,
        )

        print("sphere eval:", [sphere.eval(p) for p in points])
        print("box eval:", [box.eval(p) for p in points])
        print("sphere bounds:", sphere.bounds())
        """
    ),
    md(
        """
        ## 2. Lazy CSG

        The `|`, `&`, and `-` operators create lazy composed fields. No dense
        grid is allocated until `sample_to_grid` is called.
        """
    ),
    code(
        """
        left = implicit.SphereField([-0.45, 0.0, 0.0], 0.9)
        right = implicit.SphereField([0.45, 0.0, 0.0], 0.9)

        union = left | right
        intersection = left & right
        difference = left - right

        p = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        print("union at origin:", union.eval(p))
        print("intersection at origin:", intersection.eval(p))
        print("difference at origin:", difference.eval(p))
        print("lazy type:", type(union).__name__)
        """
    ),
    md(
        """
        ## 3. Sampling to `GridField`

        `sample_to_grid` materializes a field on a uniform grid. `GridField`
        still inherits `ImplicitField`, so point queries use trilinear
        interpolation and CSG operators continue to work.
        """
    ),
    code(
        """
        spec = implicit.GridSpec([-1.5, -1.5, -1.5], [1.5, 1.5, 1.5], resolution=48)
        grid = union.sample_to_grid(spec, num_threads=1)

        values = grid.values
        print("grid shape:", values.shape)
        print("grid dtype:", values.dtype)
        print("min/max:", float(values.min()), float(values.max()))
        print("grid eval at origin:", grid.eval([0.0, 0.0, 0.0]))
        print("values shares memory:", np.shares_memory(values, grid.values))
        """
    ),
    md(
        """
        ## 4. Marching cubes

        Extraction accepts a materialized `GridField`. For an unsampled lazy
        field, call `sample_to_grid` first.
        """
    ),
    code(
        """
        surface = implicit.extract_marching_cubes(grid, iso_offset=0.0)
        print("vertices:", surface.num_vertices)
        print("triangles:", surface.num_elements)
        print("bbox:", surface.bbox)
        """
    ),
    md(
        """
        ## 5. Mesh unsigned distance and shell thickening

        `MeshUnsignedDistanceField` turns a triangle surface into a distance
        field. The convenience function below performs the common pipeline:
        mesh -> distance field -> offset -> grid -> marching cubes.
        """
    ),
    code(
        """
        tri = pgo.mesh.TriMeshData(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [[0, 1, 2]],
        )

        shell = implicit.thicken_mesh_surface(
            tri,
            thickness=0.1,
            resolution=24,
            padding=0.25,
        )

        print("shell vertices:", shell.num_vertices)
        print("shell triangles:", shell.num_elements)
        """
    ),
    md(
        """
        ## 6. OpenVDB availability

        OpenVDB support depends on how libpgo was built. Use `has_openvdb`
        before constructing OpenVDB level sets.
        """
    ),
    code(
        """
        print("OpenVDB available:", implicit.has_openvdb())

        if implicit.has_openvdb():
            opts = implicit.OpenVDBOptions(voxel_size=0.05)
            levelset = implicit.build_openvdb_from_grid_field(grid, opts)
            vdb_surface = implicit.extract_openvdb(levelset, opts)
            print("OpenVDB surface:", vdb_surface.num_vertices, vdb_surface.num_elements)
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "implicit_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
