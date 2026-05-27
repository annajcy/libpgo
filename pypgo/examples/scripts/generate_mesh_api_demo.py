#!/usr/bin/env python3
"""Generate pypgo/examples/mesh_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_mesh_api_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # pypgo Mesh API Demo

        This notebook is a compact tour of the current M1 mesh API:

        - `pypgo.mesh`: `TriMeshData`, `TetMeshData`, `CubicMeshData`, OBJ I/O, and shape factories
        - `pypgo.mesh.geo`: geometry facades, normals, and barycentric embedding
        - `pypgo.mesh.veg`: Vega volume materials, `.veg` I/O, `VolumeMesh`, and surface extraction
        - `pypgo.tools.mesh`: quality checks and mesher wrappers
        - `pypgo.sparse` / `pypgo.sim`: sparse COO export and solver-ready mesh factories
        """
    ),
    code(
        """
        import os
        import shutil
        import tempfile

        import numpy as np
        import pypgo as pgo

        from pypgo.mesh import CubicMeshData, MeshDataType, TetMeshData, TriMeshData
        from pypgo.mesh.geo import BarycentricEmbedding, CubicMeshGeo, TetMeshGeo, TriMeshGeo
        from pypgo.mesh.veg import (
            ENuMaterial,
            MeshRegion,
            MeshSet,
            MooneyRivlinMaterial,
            VegFile,
            VolumeMesh,
        )
        from pypgo.tools.mesh import check_surface_quality, cubic_mesher, has_tetwild, tet_mesher
        """
    ),
    md(
        """
        ## 1. MeshData containers

        `MeshData` is the canonical data representation at public API boundaries. Constructors accept list-like or NumPy input and normalize vertices to `float64` and elements to `int64`.
        """
    ),
    code(
        """
        tri_data = TriMeshData(
            np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=np.float32),
            np.array([[0, 1, 2], [1, 3, 2]], dtype=np.int32),
        )

        tet_data = TetMeshData(
            np.array(
                [
                    [0, 0, 0],
                    [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1],
                ],
                dtype=np.float64,
            ),
            np.array([[0, 1, 2, 3]], dtype=np.int64),
        )

        cubic_data = CubicMeshData(
            np.array(
                [
                    [0, 0, 0],
                    [1, 0, 0],
                    [1, 1, 0],
                    [0, 1, 0],
                    [0, 0, 1],
                    [1, 0, 1],
                    [1, 1, 1],
                    [0, 1, 1],
                ],
                dtype=np.float64,
            ),
            np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64),
        )

        print("tri:", tri_data.num_vertices, tri_data.num_elements, tri_data.vertices.dtype, tri_data.elements.dtype)
        print("tet:", tet_data.num_vertices, tet_data.num_elements, tet_data.mesh_type == MeshDataType.Tet)
        print("cubic:", cubic_data.num_vertices, cubic_data.num_elements, cubic_data.mesh_type == MeshDataType.Cubic)
        """
    ),
    md(
        """
        ## 2. NumPy properties and shape factories

        Derived properties are intentionally NumPy-friendly: bounding boxes, volumes, centers of mass, subsetting, concatenation, and triangle normals all come back as arrays or scalars.
        """
    ),
    code(
        """
        box = pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 2.0, 3.0))
        sphere = pgo.mesh.create_sphere(radius=1.0, axis_subdiv=12, height_subdiv=6)

        bmin, bmax = box.bbox
        box_geo = TriMeshGeo.from_mesh_data(box)

        print("box vertices/elements:", box.num_vertices, box.num_elements)
        print("box bbox:", bmin, bmax)
        print("sphere vertices/elements:", sphere.num_vertices, sphere.num_elements)
        print("tet volume:", tet_data.volume)
        print("cubic volume:", cubic_data.volume)
        print("first 3 face normals:\\n", box_geo.face_normals[:3])
        """
    ),
    code(
        """
        left = box.take_elements(np.arange(0, box.num_elements, 2))
        right = pgo.mesh.create_box(bmin=(2.0, 0.0, 0.0), bmax=(3.0, 1.0, 1.0))
        merged = TriMeshData.concatenate([left, right])

        print("subset elements:", left.num_elements)
        print("merged vertices/elements:", merged.num_vertices, merged.num_elements)
        print("merged bbox:", merged.bbox)
        """
    ),
    md(
        """
        ## 3. MeshGeo facades and explicit conversion

        `MeshGeo` provides typed geometry accessors. Crossing between facade and data container is explicit: `.to_mesh_data()` and `.from_mesh_data(...)`.
        """
    ),
    code(
        """
        tri_geo = TriMeshGeo.from_mesh_data(tri_data)
        tet_geo = TetMeshGeo.from_mesh_data(tet_data)
        cubic_geo = CubicMeshGeo.from_mesh_data(cubic_data)

        print("triangles:\\n", tri_geo.triangles)
        print("tets:\\n", tet_geo.tets)
        print("cubes:\\n", cubic_geo.cubes)
        print("roundtrip:", np.array_equal(tri_geo.to_mesh_data().elements, tri_data.elements))
        """
    ),
    md(
        """
        ## 4. Surface quality checks

        `check_surface_quality` combines pure NumPy checks with the `_core` self-intersection bridge.
        """
    ),
    code(
        """
        quality = check_surface_quality(box, short_edge_threshold=1e-6)
        quality
        """
    ),
    md(
        """
        ## 5. Vega volume meshes and `.veg` I/O

        `.veg` I/O preserves mesh data, material payloads, sets, and regions. The public object is `VegFile`; `VolumeMesh` is constructed from `TetMeshData` or `CubicMeshData`.
        """
    ),
    code(
        """
        soft = ENuMaterial("soft", density=1000.0, E=2e6, nu=0.35)
        insert = MooneyRivlinMaterial("insert", density=1200.0, mu01=3.0, mu10=4.0, v1=0.2)

        two_tet_data = TetMeshData(
            np.array(
                [
                    [0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [1.0, 1.0, 1.0],
                ],
                dtype=np.float64,
            ),
            np.array([[0, 1, 2, 3], [1, 2, 3, 4]], dtype=np.int64),
        )

        veg = VegFile(
            mesh_data=two_tet_data,
            materials=[soft, insert],
            sets=[
                MeshSet("allElements", [0, 1]),
                MeshSet("softSet", [0]),
                MeshSet("insertSet", [1]),
            ],
            regions=[
                MeshRegion(0, 1),
                MeshRegion(1, 2),
            ],
        )

        volume = VolumeMesh(veg.mesh_data, regions=veg.to_volume_regions())
        print("volume:", volume)
        print("surface:", volume.extract_surface_mesh().num_elements, "triangles")
        """
    ),
    code(
        """
        tmpdir = tempfile.mkdtemp()
        try:
            veg_path = os.path.join(tmpdir, "two_materials.veg")
            pgo.mesh.veg.write_veg(veg_path, veg)
            loaded = pgo.mesh.veg.read_veg(veg_path)

            print("loaded mesh:", type(loaded.mesh_data).__name__, loaded.mesh_data.num_elements)
            print("materials:", [type(m).__name__ + ':' + m.name for m in loaded.materials])
            print("sets:", [(s.name, s.elements) for s in loaded.sets])
            print("regions:", [(r.material_index, r.set_index) for r in loaded.regions])
        finally:
            shutil.rmtree(tmpdir)
        """
    ),
    md(
        """
        ## 6. Barycentric embedding and sparse COO

        `BarycentricEmbedding` maps volume displacements to target locations. Its interpolation matrix is exposed through `pypgo.sparse.SparseMatrix.to_coo()`.
        """
    ),
    code(
        """
        single_volume = VolumeMesh(tet_data, soft)
        embedding = BarycentricEmbedding(np.array([[0.25, 0.25, 0.25]], dtype=np.float64), single_volume)
        matrix = embedding.interpolation_matrix
        rows, cols, values = matrix.to_coo()

        displacement = np.zeros((single_volume.num_vertices, 3), dtype=np.float64)
        displacement[:, 2] = np.linspace(0.0, 1.0, single_volume.num_vertices)

        print("matrix shape/nnz:", matrix.shape, matrix.nnz)
        print("coo:", rows, cols, values)
        print("embedded displacement:", embedding.deform(displacement.ravel()))
        """
    ),
    md(
        """
        ## 7. Solver-ready meshes

        Use explicit factories for solver-facing meshes. Volume and shell paths are separate on purpose.
        """
    ),
    code(
        """
        sim_volume = pgo.sim.SimulationMesh.create_volumetric(single_volume)
        shell_material = pgo.sim.KoiterStVKShellMaterial(
            "cloth", thickness=0.01, E_membrane=1e6, nu_membrane=0.35
        )
        sim_shell = pgo.sim.SimulationMesh.create_shell(tri_data, shell_material)

        print("volume sim:", sim_volume.mesh_type, sim_volume.num_vertices, sim_volume.num_elements)
        print("shell sim:", sim_shell.mesh_type, sim_shell.num_vertices, sim_shell.num_elements)
        """
    ),
    md(
        """
        ## 8. Mesher wrappers

        The wrappers accept `TriMeshData` and return volume `MeshData`. `tetwild` is optional; `has_tetwild()` reflects the build configuration.
        """
    ),
    code(
        """
        mesher_surface = pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 1.0, 1.0))
        cubic_from_surface = cubic_mesher(mesher_surface, resolution=2)
        tet_from_surface = tet_mesher(mesher_surface, backend="tetgen", config={"command": "pq1.414a0.05"})

        print("cubic mesher:", cubic_from_surface.num_vertices, cubic_from_surface.num_elements)
        print("tet mesher:", tet_from_surface.num_vertices, tet_from_surface.num_elements)
        print("tetwild enabled:", has_tetwild())
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "mesh_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
