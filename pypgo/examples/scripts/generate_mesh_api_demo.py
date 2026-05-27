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
        from pathlib import Path

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
        ## Notebook visualization helpers

        The demo uses real OBJ assets from `pypgo/examples/assets/obj`. PyVista is optional and only needed for interactive 3D views; install it with `pip install -e .[examples]`. The heavier `dragon.obj` asset is included for experimentation, but the default cells use smaller assets so the notebook stays quick to run.
        """
    ),
    code(
        """
        def _find_repo_root() -> Path:
            cwd = Path.cwd().resolve()
            for candidate in (cwd, *cwd.parents):
                if (candidate / "setup.py").exists() and (candidate / "pypgo").exists():
                    return candidate
            raise RuntimeError("Could not find repository root from the current working directory")


        REPO_ROOT = _find_repo_root()
        ASSET_DIR = REPO_ROOT / "pypgo" / "examples" / "assets" / "obj"
        PYVISTA_INSTALL_HINT = "Install visualization dependencies with: pip install -e .[examples]"

        try:
            import pyvista as pv
        except ModuleNotFoundError:
            pv = None
            print(f"PyVista is not installed. {PYVISTA_INSTALL_HINT}")


        def _require_pyvista() -> bool:
            if pv is None:
                print(f"Skipping PyVista view. {PYVISTA_INSTALL_HINT}")
                return False
            return True


        def to_pyvista_surface(surface_data: TriMeshData):
            if not isinstance(surface_data, TriMeshData):
                raise TypeError(f"surface_data must be a TriMeshData, got {type(surface_data).__name__}")
            if pv is None:
                raise RuntimeError(PYVISTA_INSTALL_HINT)

            faces = np.column_stack(
                [
                    np.full(surface_data.num_elements, 3, dtype=np.int64),
                    surface_data.elements,
                ]
            ).ravel()
            return pv.PolyData(surface_data.vertices, faces)


        def to_pyvista_volume(volume_data):
            if pv is None:
                raise RuntimeError(PYVISTA_INSTALL_HINT)
            if not isinstance(volume_data, (TetMeshData, CubicMeshData)):
                raise TypeError(
                    f"volume_data must be a TetMeshData or CubicMeshData, got {type(volume_data).__name__}"
                )

            elements = volume_data.elements
            width = elements.shape[1]
            if width == 4:
                cell_type = pv.CellType.TETRA
            elif width == 8:
                cell_type = pv.CellType.HEXAHEDRON
            else:
                raise ValueError(f"Unsupported volume element width: {width}")

            cells = np.column_stack(
                [
                    np.full(volume_data.num_elements, width, dtype=np.int64),
                    elements,
                ]
            ).ravel()
            cell_types = np.full(volume_data.num_elements, cell_type, dtype=np.uint8)
            return pv.UnstructuredGrid(cells, cell_types, volume_data.vertices)


        def plot_surface(meshes, *, titles=None, show_edges=True, colors=None, window_size=(900, 360)):
            if not _require_pyvista():
                return None

            if isinstance(meshes, TriMeshData):
                meshes = [meshes]
            meshes = list(meshes)
            titles = titles or [None] * len(meshes)
            colors = colors or ["lightgray"] * len(meshes)

            plotter = pv.Plotter(shape=(1, len(meshes)), window_size=window_size)
            for index, mesh in enumerate(meshes):
                if len(meshes) > 1:
                    plotter.subplot(0, index)
                plotter.add_mesh(
                    to_pyvista_surface(mesh),
                    color=colors[index % len(colors)],
                    show_edges=show_edges,
                    smooth_shading=False,
                )
                if titles[index]:
                    plotter.add_text(titles[index], position="upper_left", font_size=10)
                plotter.view_isometric()
                plotter.camera.zoom(1.2)
            return plotter.show()


        def plot_volume_surface(meshes, *, titles=None, show_edges=True, colors=None, window_size=(900, 360)):
            if not _require_pyvista():
                return None

            if isinstance(meshes, (TetMeshData, CubicMeshData)):
                meshes = [meshes]
            meshes = list(meshes)
            titles = titles or [None] * len(meshes)
            colors = colors or ["lightsteelblue"] * len(meshes)

            plotter = pv.Plotter(shape=(1, len(meshes)), window_size=window_size)
            for index, mesh in enumerate(meshes):
                if len(meshes) > 1:
                    plotter.subplot(0, index)
                plotter.add_mesh(
                    to_pyvista_volume(mesh).extract_surface(),
                    color=colors[index % len(colors)],
                    show_edges=show_edges,
                    smooth_shading=False,
                )
                if titles[index]:
                    plotter.add_text(titles[index], position="upper_left", font_size=10)
                plotter.view_isometric()
                plotter.camera.zoom(1.2)
            return plotter.show()


        print("asset directory:", ASSET_DIR)
        print("available OBJ assets:", sorted(path.name for path in ASSET_DIR.glob("*.obj")))
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

        Derived properties are intentionally NumPy-friendly: bounding boxes, volumes, centers of mass, subsetting, concatenation, and triangle normals all come back as arrays or scalars. The surface examples below load real OBJ assets from `pypgo/examples/assets/obj`.
        """
    ),
    code(
        """
        box = pgo.mesh.read_obj(str(ASSET_DIR / "box.obj"))
        bunny = pgo.mesh.read_obj(str(ASSET_DIR / "bunny.obj"))
        sphere = pgo.mesh.create_sphere(radius=1.0, axis_subdiv=12, height_subdiv=6)

        bmin, bmax = box.bbox
        box_geo = TriMeshGeo.from_mesh_data(box)

        print("box.obj vertices/elements:", box.num_vertices, box.num_elements)
        print("box bbox:", bmin, bmax)
        print("bunny.obj vertices/elements:", bunny.num_vertices, bunny.num_elements)
        print("sphere vertices/elements:", sphere.num_vertices, sphere.num_elements)
        print("tet volume:", tet_data.volume)
        print("cubic volume:", cubic_data.volume)
        print("first 3 face normals:\\n", box_geo.face_normals[:3])

        plot_surface([box, bunny], titles=["box.obj", "bunny.obj"], show_edges=False)
        """
    ),
    code(
        """
        bunny_subset = bunny.take_elements(np.arange(0, bunny.num_elements, 4))
        bunny_width = bunny.bbox[1][0] - bunny.bbox[0][0]
        shifted_subset = TriMeshData(
            bunny_subset.vertices + np.array([1.4 * bunny_width, 0.0, 0.0]),
            bunny_subset.elements,
        )
        merged = TriMeshData.concatenate([bunny_subset, shifted_subset])

        print("bunny subset elements:", bunny_subset.num_elements)
        print("merged vertices/elements:", merged.num_vertices, merged.num_elements)
        print("merged bbox:", merged.bbox)

        plot_surface(
            [bunny, bunny_subset],
            titles=["original bunny.obj", "take_elements every 4th face"],
            show_edges=True,
            colors=["lightgray", "cornflowerblue"],
        )
        plot_surface(merged, titles=["concatenated shifted subsets"], show_edges=True, colors=["plum"])
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

        The wrappers accept `TriMeshData` and return volume `MeshData`. `tetwild` is optional; `has_tetwild()` reflects the build configuration. This section uses the real `box-with-sphere.obj` asset as input.
        """
    ),
    code(
        """
        mesher_surface = pgo.mesh.read_obj(str(ASSET_DIR / "box-with-sphere.obj"))
        cubic_from_surface = cubic_mesher(mesher_surface, resolution=4)
        tet_from_surface = tet_mesher(mesher_surface, backend="tetgen", config={"command": "pq1.414a0.1"})

        cubic_surface = VolumeMesh(cubic_from_surface, soft).extract_surface_mesh()
        tet_surface = VolumeMesh(tet_from_surface, soft).extract_surface_mesh()

        print("mesher input surface:", mesher_surface.num_vertices, mesher_surface.num_elements)
        print("cubic mesher:", cubic_from_surface.num_vertices, cubic_from_surface.num_elements)
        print("tet mesher:", tet_from_surface.num_vertices, tet_from_surface.num_elements)
        print("cubic extracted surface:", cubic_surface.num_vertices, cubic_surface.num_elements)
        print("tet extracted surface:", tet_surface.num_vertices, tet_surface.num_elements)
        print("tetwild enabled:", has_tetwild())

        plot_surface(mesher_surface, titles=["box-with-sphere.obj input"], show_edges=False)
        plot_volume_surface(
            [cubic_from_surface, tet_from_surface],
            titles=["cubic_mesher volume surface", "tet_mesher volume surface"],
            show_edges=True,
            colors=["lightsteelblue", "palegreen"],
        )
        plot_surface(
            [cubic_surface, tet_surface],
            titles=["VolumeMesh(cubic).extract_surface_mesh()", "VolumeMesh(tet).extract_surface_mesh()"],
            show_edges=True,
            colors=["lightskyblue", "mediumseagreen"],
        )
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "mesh_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
