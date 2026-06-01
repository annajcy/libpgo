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
        from pypgo.mesh.geo import (
            BarycentricEmbedding, CubicMeshGeo, TetMeshGeo, TriMeshGeo,
            surface_to_volume_interpolation_matrix,
            triangle_component_ids, connected_components_by_edge,
            connected_components_by_vertex, filter_small_components, get_outer_component,
            split_components, minimum_bounding_sphere,
        )
        from pypgo.mesh.veg import (
            ENuMaterial,
            MeshRegion,
            MeshSet,
            MooneyRivlinMaterial,
            VegFile,
            VolumeMesh,
        )
        from pypgo.tools.mesh import (
            check_surface_quality, cubic_mesher, has_tetwild, tet_mesher,
            has_cgal_remesher, has_geogram_remesher,
            cgal_smooth, cgal_isotropic_remesh, cgal_simplify,
            cgal_repair_self_intersections, geogram_remesh, remove_isolated_vertices,
            volume_mesh_info,
        )
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

        from pypgo.vis import plot_surface, plot_volume_surface

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

        volume = VolumeMesh(veg.mesh_data, veg.to_volume_regions())
        print("volume:", volume)
        print("surface:", volume.extract_surface_mesh().num_elements, "triangles")

        info = volume_mesh_info(volume)
        print(info)
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
        single_volume = VolumeMesh.create_from_single_material(tet_data, soft)
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
        ## 7. Surface-to-volume interpolation matrix

        `surface_to_volume_interpolation_matrix` is a convenience wrapper around `BarycentricEmbedding` for the common case where the target locations are the vertices of a surface mesh embedded inside a volume mesh. The returned matrix `W` has shape `(3 × n_surf_verts, 3 × n_vol_verts)`, so `W @ vol_disp.ravel()` gives the interpolated surface displacements directly.
        """
    ),
    code(
        """
        surf = single_volume.extract_surface_mesh()
        W = surface_to_volume_interpolation_matrix(surf, single_volume)

        vol_disp = np.zeros((single_volume.num_vertices, 3), dtype=np.float64)
        vol_disp[:, 2] = np.linspace(0.0, 1.0, single_volume.num_vertices)

        surf_disp = (W @ vol_disp.ravel()).reshape(-1, 3)

        print("W shape:", W.shape, "  nnz:", W.nnz)
        print("surface vertices:", surf.num_vertices, "  volume vertices:", single_volume.num_vertices)
        print("interpolated z-displacements on surface vertices:\\n", surf_disp[:, 2])
        """
    ),
    md(
        """
        ## 8. Mass matrix

        `VolumeMesh.mass_matrix()` returns the consistent mass matrix as a `SparseMatrix`.

        - `inflate3dim=True` (default): shape `(3n, 3n)` — the standard displacement-DOF mass matrix used by solvers and IPC.
        - `inflate3dim=False`: shape `(n, n)` — scalar mass per vertex, useful for lumped-mass approximations.

        The consistent mass matrix satisfies `M @ ones = lumped_mass_per_vertex`, so the total mass equals the sum of **all** entries of `M1` (not just the diagonal).
        """
    ),
    code(
        """
        M3 = single_volume.mass_matrix()                   # (3n, 3n)
        M1 = single_volume.mass_matrix(inflate3dim=False)  # (n, n)

        M3_dense = M3.to_dense()
        M1_dense = M1.to_dense()

        print("M3 shape:", M3.shape, "  nnz:", M3.nnz)
        print("M3 (3n×3n) dense:\\n", M3_dense)
        print()
        print("M1 shape:", M1.shape, "  nnz:", M1.nnz)
        print("M1 (n×n) dense:\\n", M1_dense)
        print()
        print("total mass (M1 all-entry sum):", M1_dense.sum())
        print("expected  (density × volume):", soft.density * tet_data.volume)
        """
    ),
    md(
        """
        ## 9. Solver-ready meshes

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
        ## 10. Mesher wrappers

        The wrappers accept `TriMeshData` and return volume `MeshData`. `tetwild` is optional; `has_tetwild()` reflects the build configuration. This section uses the real `box-with-sphere.obj` asset as input.
        """
    ),
    code(
        """
        mesher_surface = pgo.mesh.read_obj(str(ASSET_DIR / "box-with-sphere.obj"))
        cubic_from_surface = cubic_mesher(mesher_surface, resolution=4)
        tet_from_surface = tet_mesher(mesher_surface, backend="tetgen", config={"command": "pq1.414a0.1"})

        cubic_surface = VolumeMesh.create_from_single_material(cubic_from_surface, soft).extract_surface_mesh()
        tet_surface = VolumeMesh.create_from_single_material(tet_from_surface, soft).extract_surface_mesh()

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
    md(
        """
        ## 11. Surface remeshing tools

        `pypgo.tools.mesh` exposes CGAL and Geogram surface remeshing. Availability is build-dependent — check with `has_cgal_remesher()` / `has_geogram_remesher()` before calling. All functions accept and return `TriMeshData`.

        | Function | Backend | Key parameter |
        |---|---|---|
        | `cgal_smooth` | CGAL | `num_iter`, `sharp_angle` |
        | `cgal_isotropic_remesh` | CGAL | `target_edge_length` (absolute) |
        | `cgal_simplify` | CGAL | `target_ratio` (fraction of edges to keep) |
        | `cgal_repair_self_intersections` | CGAL | `method` |
        | `geogram_remesh` | Geogram | `target_num_vertices` |
        | `remove_isolated_vertices` | pure mesh | — |
        """
    ),
    code(
        """
        print("CGAL remesher available:", has_cgal_remesher())
        print("Geogram remesher available:", has_geogram_remesher())

        # Use bunny as the demo surface throughout this section
        demo_mesh = pgo.mesh.read_obj(str(ASSET_DIR / "bunny.obj"))
        print(f"bunny: {demo_mesh.num_vertices} vertices, {demo_mesh.num_elements} triangles")
        """
    ),
    md(
        """
        ### 11a. CGAL smooth

        `cgal_smooth` applies angle-and-area smoothing. Edges whose dihedral angle exceeds `sharp_angle` degrees are treated as feature edges and held fixed.
        """
    ),
    code(
        """
        if has_cgal_remesher():
            smoothed = cgal_smooth(demo_mesh, num_iter=10, sharp_angle=60.0)
            print(f"smoothed: {smoothed.num_vertices} vertices, {smoothed.num_elements} triangles")
            plot_surface(
                [demo_mesh, smoothed],
                titles=["bunny original", "cgal_smooth (10 iter, sharp=60°)"],
                show_edges=False,
                colors=["lightgray", "cornflowerblue"],
            )
        else:
            print("Skipping: CGAL not available")
        """
    ),
    md(
        """
        ### 11b. CGAL isotropic remeshing

        `cgal_isotropic_remesh` splits/collapses edges to reach a uniform `target_edge_length`. Here we compute the average edge length of the input and remesh at 2× (coarser) and 0.5× (finer).
        """
    ),
    code(
        """
        if has_cgal_remesher():
            verts = demo_mesh.vertices
            tris  = demo_mesh.elements
            edge_vecs = np.concatenate([
                verts[tris[:, 1]] - verts[tris[:, 0]],
                verts[tris[:, 2]] - verts[tris[:, 1]],
                verts[tris[:, 0]] - verts[tris[:, 2]],
            ])
            avg_edge_len = float(np.linalg.norm(edge_vecs, axis=1).mean())
            print(f"average edge length: {avg_edge_len:.6f}")

            coarse = cgal_isotropic_remesh(demo_mesh, target_edge_length=avg_edge_len * 2.0, num_iter=5)
            fine   = cgal_isotropic_remesh(demo_mesh, target_edge_length=avg_edge_len * 0.5, num_iter=5)

            print(f"coarse (2× avg): {coarse.num_vertices} vertices, {coarse.num_elements} triangles")
            print(f"fine   (0.5× avg): {fine.num_vertices} vertices, {fine.num_elements} triangles")

            plot_surface(
                [demo_mesh, coarse, fine],
                titles=["original", "isotropic 2× (coarser)", "isotropic 0.5× (finer)"],
                show_edges=True,
                colors=["lightgray", "lightsalmon", "mediumseagreen"],
                window_size=(1200, 360),
            )
        else:
            print("Skipping: CGAL not available")
        """
    ),
    md(
        """
        ### 11c. CGAL simplification

        `cgal_simplify` collapses edges until the mesh has approximately `target_ratio` of its original edge count.
        """
    ),
    code(
        """
        if has_cgal_remesher():
            simplified_50 = cgal_simplify(demo_mesh, target_ratio=0.5)
            simplified_10 = cgal_simplify(demo_mesh, target_ratio=0.1)

            print(f"original:   {demo_mesh.num_vertices} vertices, {demo_mesh.num_elements} triangles")
            print(f"ratio=0.50: {simplified_50.num_vertices} vertices, {simplified_50.num_elements} triangles")
            print(f"ratio=0.10: {simplified_10.num_vertices} vertices, {simplified_10.num_elements} triangles")

            plot_surface(
                [demo_mesh, simplified_50, simplified_10],
                titles=["original", "simplify ratio=0.50", "simplify ratio=0.10"],
                show_edges=True,
                colors=["lightgray", "plum", "lightsalmon"],
                window_size=(1200, 360),
            )
        else:
            print("Skipping: CGAL not available")
        """
    ),
    md(
        """
        ### 11d. Geogram remeshing

        `geogram_remesh` redistributes triangles to reach a target vertex count while preserving shape features.
        """
    ),
    code(
        """
        if has_geogram_remesher():
            geo_500  = geogram_remesh(demo_mesh, target_num_vertices=500)
            geo_2000 = geogram_remesh(demo_mesh, target_num_vertices=2000)

            print(f"original:     {demo_mesh.num_vertices} vertices, {demo_mesh.num_elements} triangles")
            print(f"target  500:  {geo_500.num_vertices} vertices, {geo_500.num_elements} triangles")
            print(f"target 2000:  {geo_2000.num_vertices} vertices, {geo_2000.num_elements} triangles")

            plot_surface(
                [demo_mesh, geo_500, geo_2000],
                titles=["original", "geogram target=500", "geogram target=2000"],
                show_edges=True,
                colors=["lightgray", "cornflowerblue", "mediumseagreen"],
                window_size=(1200, 360),
            )
        else:
            print("Skipping: Geogram not available")
        """
    ),
    md(
        """
        ## 12. Connected component operations

        The functions below operate on edge-connected or vertex-connected components of a `TriMeshData` surface mesh. The demo uses `box-with-sphere.obj`, which contains two disjoint closed shells.

        | Function | Returns |
        |---|---|
        | `triangle_component_ids` | `(component_ids, component_sizes)` per triangle |
        | `connected_components_by_edge` | list of triangle-index arrays, one per component |
        | `connected_components_by_vertex` | same, but vertex-connectivity |
        | `filter_small_components` | mesh with small components removed |
        | `get_outer_component` | single outermost shell |
        """
    ),
    code(
        """
        multi = pgo.mesh.read_obj(str(ASSET_DIR / "box-with-sphere.obj"))
        comp_ids, comp_sizes = triangle_component_ids(multi)

        print(f"mesh: {multi.num_vertices} vertices, {multi.num_elements} triangles")
        print(f"number of edge-connected components: {len(comp_sizes)}")
        print(f"component sizes (triangles): {sorted(comp_sizes.tolist(), reverse=True)}")
        """
    ),
    md(
        """
        ### 12a. Split and visualize each component

        `connected_components_by_edge` returns a list of triangle-index arrays. `take_elements` extracts a submesh for each component, which can then be visualized individually.
        """
    ),
    code(
        """
        edge_comps = connected_components_by_edge(multi)
        print(f"edge-connected components: {len(edge_comps)}")
        for i, tri_ids in enumerate(edge_comps):
            sub = multi.take_elements(tri_ids)
            print(f"  component {i}: {sub.num_vertices} vertices, {sub.num_elements} triangles, bbox {sub.bbox}")

        comp_colors = ["lightsteelblue", "lightsalmon", "mediumseagreen", "plum"]
        comp_meshes = [multi.take_elements(tri_ids) for tri_ids in edge_comps]
        plot_surface(
            comp_meshes,
            titles=[f"component {i} ({len(tri_ids)} tris)" for i, tri_ids in enumerate(edge_comps)],
            colors=comp_colors[: len(edge_comps)],
            show_edges=False,
            window_size=(900 if len(edge_comps) <= 2 else 1200, 360),
        )
        """
    ),
    md(
        """
        ### 12b. Filter small components

        `filter_small_components` removes components below a triangle-count threshold. Setting `min_triangles` just above the smallest component size strips it; `keep_largest=1` retains only the single biggest component regardless of threshold.
        """
    ),
    code(
        """
        smallest_size = int(sorted(comp_sizes)[0])
        filtered = filter_small_components(multi, min_triangles=smallest_size + 1)
        keep_one  = filter_small_components(multi, min_triangles=1, keep_largest=1)

        print(f"original:               {multi.num_vertices} vertices, {multi.num_elements} triangles")
        print(f"filter min={smallest_size+1}: {filtered.num_vertices} vertices, {filtered.num_elements} triangles")
        print(f"keep_largest=1:         {keep_one.num_vertices} vertices, {keep_one.num_elements} triangles")

        plot_surface(
            [multi, filtered, keep_one],
            titles=["original", f"filter min_triangles={smallest_size+1}", "keep_largest=1"],
            colors=["lightgray", "cornflowerblue", "mediumseagreen"],
            show_edges=False,
            window_size=(1200, 360),
        )
        """
    ),
    md(
        """
        ### 12c. Extract outermost component

        `get_outer_component` locates the topmost triangle, performs a vertex-connected BFS from it, and returns that shell — useful for isolating the outer surface of a nested mesh before volumetric meshing.
        """
    ),
    code(
        """
        outer = get_outer_component(multi)
        print(f"outer component: {outer.num_vertices} vertices, {outer.num_elements} triangles")

        plot_surface(
            [multi, outer],
            titles=["original (all components)", "get_outer_component"],
            colors=["lightgray", "mediumseagreen"],
            show_edges=False,
        )
        """
    ),
    md(
        """
        ### 12d. Edge vs. vertex connectivity

        `connected_components_by_vertex` uses vertex sharing rather than edge sharing. On a well-formed mesh the two are equivalent; they differ when components touch at a single vertex (pinch point) — edge connectivity splits them, vertex connectivity merges them.
        """
    ),
    code(
        """
        edge_comps   = connected_components_by_edge(multi)
        vertex_comps = connected_components_by_vertex(multi)

        print(f"edge-connected   components: {len(edge_comps)},   sizes: {sorted([len(c) for c in edge_comps], reverse=True)}")
        print(f"vertex-connected components: {len(vertex_comps)}, sizes: {sorted([len(c) for c in vertex_comps], reverse=True)}")
        """
    ),
    md(
        """
        ## 13. `split_components` and `minimum_bounding_sphere`

        These two functions were migrated from `scripts/dump_obj_components.py` and
        `scripts/generate_bounding_sphere.py` respectively.

        `split_components` is a convenience wrapper around `connected_components_by_edge`
        that returns ready-to-use `TriMeshData` objects instead of triangle-index arrays.

        `minimum_bounding_sphere` computes the smallest enclosing sphere for all vertices
        using Welzl's randomised algorithm (seed=0 for reproducibility). It accepts any
        mesh data type: `TriMeshData`, `TetMeshData`, or `CubicMeshData`.
        """
    ),
    code(
        """
        # split_components: each component returned as a standalone TriMeshData
        parts = split_components(multi)
        print(f"split_components → {len(parts)} submeshes")
        for i, part in enumerate(parts):
            print(f"  component {i}: {part.num_vertices} vertices, {part.num_elements} triangles,  bbox {part.bbox}")

        plot_surface(
            parts,
            titles=[f"component {i} ({p.num_elements} tris)" for i, p in enumerate(parts)],
            colors=["lightsteelblue", "lightsalmon", "mediumseagreen", "plum"][: len(parts)],
            show_edges=False,
            window_size=(900 if len(parts) <= 2 else 1200, 360),
        )
        """
    ),
    code(
        """
        # minimum_bounding_sphere on the bunny surface mesh
        center, radius = minimum_bounding_sphere(bunny)
        print(f"bunny bounding sphere:")
        print(f"  center : {center}")
        print(f"  radius : {radius:.6f}")
        print(f"  bbox diagonal : {np.linalg.norm(np.array(bunny.bbox[1]) - np.array(bunny.bbox[0])):.6f}")

        # also works on volume meshes
        center_tet, radius_tet = minimum_bounding_sphere(tet_data)
        print(f"\\ntet_data bounding sphere:")
        print(f"  center : {center_tet}")
        print(f"  radius : {radius_tet:.6f}")
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "mesh_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
