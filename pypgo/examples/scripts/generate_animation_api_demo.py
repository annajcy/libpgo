#!/usr/bin/env python3
"""Generate pypgo/examples/animation_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_animation_api_demo.py
"""

from __future__ import annotations

from notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # pypgo Animation API Demo

        This notebook shows how to assemble an animation sequence from per-frame
        displacements and export it to **Alembic (.abc)** using `pypgo.animation`.

        The pipeline mirrors the C++ `convertAnimation` CLI tool:

        1. Load a driving mesh (`.obj` surface or `.veg` tet mesh)
        2. Load per-frame displacements (`"objmesh"` frame files, or `"u"` Eigen binary files)
        3. Optionally embed a separate high-resolution display mesh via barycentric interpolation
        4. Write one `.abc` file per named sequence

        A synthetic box-oscillation example is used so the notebook runs without
        any pre-existing simulation output.
        """
    ),
    code(
        """
        import os
        import tempfile
        from pathlib import Path

        import numpy as np
        import pypgo as pgo
        import pypgo._core as _core

        from pypgo.mesh import TriMeshData
        from pypgo.animation import (
            AnimationSequence,
            process_sequence,
            read_u_file,
            write_u_file,
        )
        from pypgo.vis import plot_surface

        def _find_repo_root() -> Path:
            cwd = Path.cwd().resolve()
            for candidate in (cwd, *cwd.parents):
                if (candidate / "setup.py").exists() and (candidate / "pypgo").exists():
                    return candidate
            raise RuntimeError("Could not find repository root")

        REPO_ROOT = _find_repo_root()
        ASSET_DIR = REPO_ROOT / "pypgo" / "examples" / "assets" / "obj"

        print("Animation IO available:", _core.has_animation_io())
        print("box.obj:", ASSET_DIR / "box.obj")
        """
    ),
    md(
        """
        ## 1. Mesh and synthetic displacements

        Load `box.obj` as the display surface. Generate `N_FRAMES` of smooth
        sinusoidal displacement in the x-direction — this produces a simple
        oscillating-box animation without needing any simulation output.
        """
    ),
    code(
        """
        N_FRAMES = 48

        box = pgo.mesh.read_obj(str(ASSET_DIR / "box.obj"))
        print(f"box: {box.num_vertices} vertices, {box.num_elements} triangles")

        # Smooth sinusoidal translation along x, one full cycle over N_FRAMES
        t = np.linspace(0, 2 * np.pi, N_FRAMES, endpoint=False)
        amplitude = float(box.bbox[1][0] - box.bbox[0][0]) * 0.5  # half the bbox width

        # displacements: (N_FRAMES, n_verts * 3), flat per-frame vectors
        displacements = np.zeros((N_FRAMES, box.num_vertices * 3))
        displacements[:, 0::3] = amplitude * np.sin(t)[:, None]   # x component
        displacements[:, 1::3] = amplitude * 0.3 * np.sin(2 * t)[:, None]  # y wobble

        print(f"displacement amplitude: {amplitude:.4f}")
        print(f"frames: {N_FRAMES}, disp array: {displacements.shape}")
        """
    ),
    md(
        """
        ## 2. Visualize keyframes

        Show the displaced mesh at four keyframes to confirm the motion looks correct
        before writing any files.
        """
    ),
    code(
        """
        rest_verts = box.vertices.copy()
        keyframes = [0, N_FRAMES // 4, N_FRAMES // 2, 3 * N_FRAMES // 4]
        key_meshes = [
            TriMeshData(rest_verts + displacements[f].reshape(-1, 3), box.elements)
            for f in keyframes
        ]

        plot_surface(
            key_meshes,
            titles=[f"frame {f} / {N_FRAMES}" for f in keyframes],
            show_edges=True,
            colors=["lightgray", "cornflowerblue", "mediumseagreen", "lightsalmon"],
            window_size=(1200, 360),
        )
        """
    ),
    md(
        """
        ## 3. Low-level export: `_core.dump_abc`

        Call `dump_abc` directly to write a single mesh animation to an Alembic file.
        This is the thin C++ wrapper; the high-level API built on top of it is shown
        in the next section.

        Arguments:
        - `rest_positions` — flat `list[float]` of length `3 × n_verts` (rest-pose)
        - `displacements`  — `list[list[float]]`, one inner list per frame (delta from rest)
        - `triangles`      — `list[list[int]]`, one inner list of three indices per face
        """
    ),
    code(
        """
        tmpdir = Path(tempfile.mkdtemp())
        abc_low = tmpdir / "box_lowlevel.abc"

        _core.dump_abc(
            str(abc_low),
            "box",
            box.vertices.astype(np.float32).ravel().tolist(),
            [displacements[f].astype(np.float32).tolist() for f in range(N_FRAMES)],
            box.elements.tolist(),
        )

        print(f"Written: {abc_low}")
        print(f"  {box.num_vertices} verts × {N_FRAMES} frames × {box.num_elements} tris")
        print(f"  file size: {abc_low.stat().st_size / 1024:.1f} KB")
        """
    ),
    md(
        """
        ## 4. High-level export: `AnimationSequence` + `process_sequence`

        The high-level API reads displacements from disk, matching the C++ tool's
        conventions. Here we write synthetic displacements as Eigen binary `.u` files
        and pass them through `process_sequence`.

        **`.u` file format:** 3 `int32` header values `(nrows, ncols, entry_size=8)`,
        followed by column-major `float64` data. Use `write_u_file` / `read_u_file`
        to read and write from Python.
        """
    ),
    code(
        """
        u_dir = tmpdir / "u_files"
        u_dir.mkdir()

        for f in range(N_FRAMES):
            write_u_file(u_dir / f"frame_{f:04d}.u", displacements[f])

        # Verify round-trip
        loaded = read_u_file(u_dir / "frame_0000.u")
        print("round-trip check:", np.allclose(loaded[:, 0], displacements[0]))

        seq = AnimationSequence(
            name="box_oscillation",
            driving_mesh=str(ASSET_DIR / "box.obj"),
            sequence=str(u_dir / "frame_{:04d}.u"),
            sequence_type="u",
            sequence_range=(0, N_FRAMES),
        )
        print(seq)
        """
    ),
    code(
        """
        process_sequence(seq, tmpdir)

        abc_hl = tmpdir / "box_oscillation.abc"
        print(f"Written: {abc_hl}")
        print(f"  file size: {abc_hl.stat().st_size / 1024:.1f} KB")
        """
    ),
    md(
        """
        ## 5. Multi-sequence export: `convert_animation`

        `convert_animation` accepts either a JSON config dict (same format as the C++
        `convertAnimation` tool) or a path to a config file on disk. Each entry in
        `"meshes"` becomes one `.abc` output file.
        """
    ),
    code(
        """
        from pypgo.animation import convert_animation

        config = {
            "meshes": [
                {
                    "name": "box_sin",
                    "driving-mesh": str(ASSET_DIR / "box.obj"),
                    "sequence": str(u_dir / "frame_{:04d}.u"),
                    "sequence-type": "u",
                    "sequence-range": [0, N_FRAMES],
                    "scale": "1,1,1",
                },
                {
                    "name": "box_sin_half",
                    "driving-mesh": str(ASSET_DIR / "box.obj"),
                    "sequence": str(u_dir / "frame_{:04d}.u"),
                    "sequence-type": "u",
                    "sequence-range": [0, N_FRAMES],
                    "gap": 2,
                    "scale": "1,1,0.5",
                },
            ]
        }

        out_dir = tmpdir / "multi"
        convert_animation(config, output_folder=out_dir)

        for abc in sorted(out_dir.glob("*.abc")):
            print(f"  {abc.name}  ({abc.stat().st_size / 1024:.1f} KB)")
        """
    ),
    md(
        """
        ## 6. `"objmesh"` sequence type

        When per-frame geometry is stored as individual `.obj` files (common for
        output from cloth or rigid-body simulators), set `sequence_type="objmesh"`.
        The driving displacement for each frame is computed as
        `frame_vertices − rest_vertices`.

        Here we write the displaced box meshes to `.obj` files and re-export.
        """
    ),
    code(
        """
        obj_dir = tmpdir / "obj_frames"
        obj_dir.mkdir()

        for f in range(N_FRAMES):
            frame_verts = rest_verts + displacements[f].reshape(-1, 3)
            frame_mesh = TriMeshData(frame_verts, box.elements)
            pgo.mesh.write_obj(str(obj_dir / f"frame_{f:04d}.obj"), frame_mesh)

        seq_obj = AnimationSequence(
            name="box_objmesh",
            driving_mesh=str(ASSET_DIR / "box.obj"),
            sequence=str(obj_dir / "frame_{:04d}.obj"),
            sequence_type="objmesh",
            sequence_range=(0, N_FRAMES),
        )

        process_sequence(seq_obj, tmpdir)

        abc_obj = tmpdir / "box_objmesh.abc"
        print(f"Written: {abc_obj}  ({abc_obj.stat().st_size / 1024:.1f} KB)")
        """
    ),
    md(
        """
        ## 7. Cleanup

        Remove the temporary directory created for this demo.
        """
    ),
    code(
        """
        import shutil
        shutil.rmtree(tmpdir)
        print("Temporary files removed.")
        """
    ),
]


def main() -> None:
    root = repo_root()
    write_notebook(root / "pypgo" / "examples" / "animation_api_demo.ipynb", CELLS)


if __name__ == "__main__":
    main()
