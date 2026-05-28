#!/usr/bin/env python3
"""Generate pypgo/examples/numpy_interoperate.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_numpy_interoperate.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # NumPy Interoperate with pypgo

        This tutorial explains how pypgo's Python API exchanges data with NumPy arrays.

        **Audience:** users writing scripts, notebooks, or tests around pypgo mesh data.

        **Prerequisites:** basic NumPy indexing and familiarity with triangle/tetrahedral mesh arrays.

        **Learning goals:**

        1. Build pypgo mesh objects from NumPy arrays.
        2. Understand dtype normalization and copy boundaries.
        3. Use NumPy vectorization on mesh outputs.
        4. Export sparse matrices as COO arrays.
        5. Recognize common shape and dtype errors.
        """
    ),
    md(
        """
        ## Outline

        1. Array shapes and dtypes
        2. Ownership: snapshots, not mutable views
        3. Vectorized mesh queries
        4. Dense binding smoke tests
        5. Sparse COO export
        6. Common validation failures
        7. Exercise
        """
    ),
    code(
        """
        import numpy as np
        import pypgo as pgo
        """
    ),
    md(
        """
        ## 1. Array shapes and dtypes

        Mesh constructors accept array-like input. Vertices are shaped `(n, 3)` and normalized to `float64`; elements are shaped `(m, K)` and normalized to `int64`, where `K` is 3, 4, or 8.
        """
    ),
    code(
        """
        vertices32 = np.array(
            [
                [0, 0, 0],
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )
        tets32 = np.array([[0, 1, 2, 3]], dtype=np.int32)

        tet = pgo.mesh.TetMeshData(vertices32, tets32)

        print("input dtypes:", vertices32.dtype, tets32.dtype)
        print("pypgo dtypes:", tet.vertices.dtype, tet.elements.dtype)
        print("shapes:", tet.vertices.shape, tet.elements.shape)
        """
    ),
    md(
        """
        ## 2. Ownership: snapshots, not mutable views

        pypgo copies constructor inputs into its C++ object. The `.vertices` and `.elements` properties return NumPy arrays that are convenient snapshots. Mutating those snapshots does not mutate the underlying mesh; rebuild a mesh when you want changed geometry.
        """
    ),
    code(
        """
        source_vertices = vertices32.astype(np.float64)
        source_elements = tets32.astype(np.int64)
        owned = pgo.mesh.TetMeshData(source_vertices, source_elements)

        source_vertices[0, 0] = 99.0
        source_elements[0, 0] = 3
        print("mesh after source mutation:")
        print(owned.vertices)
        print(owned.elements)

        snapshot = owned.vertices
        snapshot[0, 0] = 123.0
        print("snapshot changed:", snapshot[0, 0])
        print("mesh property re-read:", owned.vertices[0, 0])
        """
    ),
    md(
        """
        To modify geometry, use NumPy to create new arrays and construct a new `MeshData` object.
        """
    ),
    code(
        """
        translated_vertices = owned.vertices + np.array([10.0, 0.0, 0.0])
        translated = pgo.mesh.TetMeshData(translated_vertices, owned.elements)

        print("old bbox:", owned.bbox)
        print("new bbox:", translated.bbox)
        """
    ),
    md(
        """
        ## 3. Vectorized mesh queries

        Once you have NumPy arrays, use regular indexing and broadcasting. The expression `vertices[elements]` gathers each element's vertex coordinates.
        """
    ),
    code(
        """
        element_vertices = tet.vertices[tet.elements]
        centroids = element_vertices.mean(axis=1)

        print("element_vertices shape:", element_vertices.shape)
        print("centroids:", centroids)
        print("volume:", tet.volume)
        print("center of mass:", tet.center_of_mass)
        """
    ),
    code(
        """
        surface = pgo.mesh.create_box(bmin=(0, 0, 0), bmax=(1, 1, 1))
        surface_geo = pgo.mesh.geo.TriMeshGeo.from_mesh_data(surface)

        print("surface bbox:", surface.bbox)
        print("face area sum:", surface_geo.face_areas.sum())
        print("first vertex normal:", surface_geo.vertex_normals[0])
        """
    ),
    md(
        """
        ## 4. Dense binding smoke tests

        These `_core` helpers are internal test hooks, but they make the interop rules visible: contiguous `float64` arrays can be read directly by C++; non-contiguous slices are accepted through a copy fallback for owned outputs.
        """
    ),
    code(
        """
        vector = np.array([1.0, 2.0, 3.5], dtype=np.float64)
        matrix = np.arange(12.0, dtype=np.float64).reshape(3, 4)
        sliced = matrix[:, ::2]

        print("vector roundtrip:", pgo._core._test_vector_roundtrip(vector))
        print("vector sum:", pgo._core._test_vector_sum(vector))
        print("matrix sum:", pgo._core._test_matrix_sum(matrix))
        print("sliced contiguous?", sliced.flags.c_contiguous)
        print("sliced roundtrip:\\n", pgo._core._test_matrix_roundtrip(sliced))
        """
    ),
    md(
        """
        ## 5. Sparse COO export

        Sparse matrices are owned by pypgo and exported as `(rows, cols, values)` NumPy arrays. Duplicate entries are accumulated by the C++ sparse matrix before export.
        """
    ),
    code(
        """
        core_sparse = pgo._core.create_sparse_matrix(
            3,
            4,
            [2, 0, 2],
            [1, 3, 1],
            [5.0, 1.5, 2.5],
        )
        sparse = pgo.sparse.SparseMatrix(core_sparse)
        rows, cols, values = sparse.to_coo()

        print("shape:", sparse.shape)
        print("nnz:", sparse.nnz)
        print("rows:", rows, rows.dtype)
        print("cols:", cols, cols.dtype)
        print("values:", values, values.dtype)
        """
    ),
    code(
        """
        material = pgo.mesh.veg.ENuMaterial("rubber", E=1e6, nu=0.35, density=1000.0)
        volume = pgo.mesh.veg.VolumeMesh.create_from_single_material(tet, material)
        embedding = pgo.mesh.geo.BarycentricEmbedding(
            np.array([[0.25, 0.25, 0.25]], dtype=np.float64),
            volume,
        )
        rows, cols, values = embedding.interpolation_matrix.to_coo()

        print("embedding matrix:", embedding.interpolation_matrix.shape, embedding.interpolation_matrix.nnz)
        print("coo rows:", rows)
        print("coo cols:", cols)
        print("coo values:", values)
        """
    ),
    md(
        """
        ## 6. Common validation failures

        Element arrays must have the right width and integer dtype. Indices must be in range.
        """
    ),
    code(
        """
        examples = [
            ("wrong tet width", lambda: pgo.mesh.TetMeshData(vertices32, np.array([[0, 1, 2]], dtype=np.int64))),
            ("float element dtype", lambda: pgo.mesh.TetMeshData(vertices32, np.array([[0.0, 1.0, 2.0, 3.0]]))),
            ("out-of-range index", lambda: pgo.mesh.TetMeshData(vertices32, np.array([[0, 1, 2, 99]], dtype=np.int64))),
        ]

        for label, fn in examples:
            try:
                fn()
            except Exception as exc:
                print(f"{label}: {type(exc).__name__}: {exc}")
        """
    ),
    md(
        """
        ## 7. Exercise

        Build two translated copies of a triangle surface, concatenate them, and compute the merged bounding box.
        """
    ),
    code(
        """
        base = pgo.mesh.create_box(bmin=(0, 0, 0), bmax=(1, 1, 1))
        shifted_vertices = base.vertices + np.array([2.0, 0.0, 0.0])
        shifted = pgo.mesh.TriMeshData(shifted_vertices, base.elements)

        merged = pgo.mesh.TriMeshData.concatenate([base, shifted])
        print("merged vertices/elements:", merged.num_vertices, merged.num_elements)
        print("merged bbox:", merged.bbox)
        """
    ),
    md(
        """
        ## Pitfall and extension

        **Pitfall:** changing `mesh.vertices` in-place changes only the returned snapshot. Reconstruct a `MeshData` object when you want persistent edits.

        **Extension:** convert COO arrays to SciPy sparse matrices if your environment has SciPy installed:

        ```python
        from scipy.sparse import coo_matrix
        scipy_matrix = coo_matrix((values, (rows, cols)), shape=sparse.shape)
        ```
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "numpy_interoperate.ipynb", CELLS)


if __name__ == "__main__":
    main()
