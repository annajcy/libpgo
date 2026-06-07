#!/usr/bin/env python3
"""Generate pypgo/examples/animation_api_demo.ipynb.

Run from the repository root:

    conda run -n libpgo python pypgo/examples/scripts/generate_animation_api_demo.py
"""

from __future__ import annotations

from examples.scripts.notebook_builder import code, md, repo_root, write_notebook


CELLS = [
    md(
        """
        # pypgo Animation & Stress Field API Demo

        This notebook covers two related pipelines:

        **Animation export (Alembic)**
        | API | Description |
        |---|---|
        | `AbcWriter` | Incremental Alembic writer |
        | `dump_animation` | JSON-config-driven export (one-shot) |

        **Stress field**
        | API | Module | Requires |
        |---|---|---|
        | `compute_stress_field_stats` | `pypgo.stress` | NumPy only |
        | `dump_stress_vdb` | `pypgo.animation` | OpenVDB build |

        Both pipelines consume the standard simulation output layout:

        ```
        sim_output/
          states/deform{frame:04d}.u       ← per-frame displacement (3n × 1 Eigen binary)
          stress/von_mises{frame:04d}.json  ← per-element von Mises values
        ```

        Synthetic data is generated throughout so the notebook runs without any
        real simulation output.
        """
    ),
    code(
        """
        import json
        import shutil
        import tempfile
        from pathlib import Path

        import numpy as np
        import pypgo as pgo

        from pypgo.animation import (
            AbcWriter,
            dump_animation,
            dump_stress_vdb,
            has_animation_io,
            has_stress_vdb_export,
            read_u_file,
            write_u_file,
        )
        from pypgo.mesh import TriMeshData
        from pypgo.mesh.volume import read_veg
        from pypgo.animation import compute_stress_field_stats
        from pypgo.mesh.visualize import plot_surface

        def _find_repo_root() -> Path:
            cwd = Path.cwd().resolve()
            for candidate in (cwd, *cwd.parents):
                if (candidate / "setup.py").exists() and (candidate / "pypgo").exists():
                    return candidate
            raise RuntimeError("Could not find repository root")

        REPO_ROOT = _find_repo_root()
        ASSET_DIR = REPO_ROOT / "pypgo" / "examples" / "assets" / "obj"
        VEG_DIR   = REPO_ROOT / "pypgo" / "examples" / "assets" / "veg" / "tet"

        tmpdir = Path(tempfile.mkdtemp())

        print("Animation IO available :", has_animation_io())
        print("Stress VDB available   :", has_stress_vdb_export())
        """
    ),
    md(
        """
        ## 1. Mesh and synthetic displacements

        Load `box.obj` as the surface mesh. Generate `N_FRAMES` of smooth
        sinusoidal displacement — a translation along x with a y wobble.
        """
    ),
    code(
        """
        N_FRAMES = 48

        box = pgo.mesh.read_obj(str(ASSET_DIR / "box.obj"))
        print(f"box: {box.num_vertices} vertices, {box.num_elements} triangles")

        t = np.linspace(0, 2 * np.pi, N_FRAMES, endpoint=False)
        amplitude = float(box.bbox[1][0] - box.bbox[0][0]) * 0.5

        displacements = np.zeros((N_FRAMES, box.num_vertices * 3))
        displacements[:, 0::3] = amplitude * np.sin(t)[:, None]
        displacements[:, 1::3] = amplitude * 0.3 * np.sin(2 * t)[:, None]

        print(f"amplitude: {amplitude:.4f},  shape: {displacements.shape}")
        """
    ),
    md(
        """
        ## 2. Visualize keyframes
        """
    ),
    code(
        """
        rest_verts = box.vertices.copy()
        keyframes  = [0, N_FRAMES // 4, N_FRAMES // 2, 3 * N_FRAMES // 4]
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
        ## 3. Incremental export: `AbcWriter`

        ``AbcWriter`` accumulates displacement frames one at a time and writes
        the ``.abc`` file in one shot. Use it as a context manager for automatic
        finalisation, or call ``add_frame`` / ``write`` manually.
        """
    ),
    code(
        """
        abc_low = tmpdir / "box_lowlevel.abc"

        with AbcWriter(abc_low, "box",
                       rest_positions=box.vertices.ravel(),
                       triangles=box.elements.ravel()) as w:
            for f in range(N_FRAMES):
                w.add_frame(displacements[f])

        print(f"Written: {abc_low}  ({abc_low.stat().st_size / 1024:.1f} KB)")
        """
    ),
    md(
        """
        ## 4. Config-driven export: `dump_animation`

        ``dump_animation`` is the one-shot convenience: it loads a JSON config
        file and exports all meshes to the target folder in one call.

        Here we use ``"sequence-type": "objmesh"`` — each frame is an
        individual ``.obj`` file containing the deformed vertex positions.
        """
    ),
    code(
        """
        frame_dir = tmpdir / "obj_frames"
        frame_dir.mkdir()

        for f in range(N_FRAMES):
            frame_verts = rest_verts + displacements[f].reshape(-1, 3)
            pgo.mesh.write_obj(str(frame_dir / f"frame_{f:04d}.obj"),
                               TriMeshData(frame_verts, box.elements))

        config_single = {
            "meshes": [
                {
                    "name": "box_oscillation",
                    "driving-mesh": str(ASSET_DIR / "box.obj"),
                    "sequence": str(frame_dir / "frame_{:04d}.obj"),
                    "sequence-type": "objmesh",
                    "sequence-range": [0, N_FRAMES],
                }
            ]
        }
        json_path = tmpdir / "config_single.json"
        json_path.write_text(json.dumps(config_single, indent=2))

        dump_animation(json_path, tmpdir)

        abc_hl = tmpdir / "box_oscillation.abc"
        print(f"Written: {abc_hl}  ({abc_hl.stat().st_size / 1024:.1f} KB)")
        """
    ),
    md(
        """
        ## 5. Multi-sequence export with `dump_animation`

        Multiple meshes can be exported by listing them in the same JSON config.
        Each ``"meshes"`` entry becomes one ``.abc`` file.
        """
    ),
    code(
        """
        config_multi = {
            "meshes": [
                {
                    "name": "box_sin",
                    "driving-mesh": str(ASSET_DIR / "box.obj"),
                    "sequence": str(frame_dir / "frame_{:04d}.obj"),
                    "sequence-type": "objmesh",
                    "sequence-range": [0, N_FRAMES],
                    "scale": "1,1,1",
                },
                {
                    "name": "box_sin_half",
                    "driving-mesh": str(ASSET_DIR / "box.obj"),
                    "sequence": str(frame_dir / "frame_{:04d}.obj"),
                    "sequence-type": "objmesh",
                    "sequence-range": [0, N_FRAMES],
                    "gap": 2,
                    "scale": "1,1,0.5",
                },
            ]
        }

        out_dir = tmpdir / "multi"
        json_multi = tmpdir / "config_multi.json"
        json_multi.write_text(json.dumps(config_multi, indent=2))

        dump_animation(json_multi, out_dir)

        for abc in sorted(out_dir.glob("*.abc")):
            print(f"  {abc.name}  ({abc.stat().st_size / 1024:.1f} KB)")
        """
    ),
    md(
        """
        ## 6. Synthetic stress simulation data

        The stress pipeline expects the standard simulation output layout. Here we generate
        synthetic data from `bunny.veg`:

        - **Displacements** — a "breathing" animation: vertices oscillate radially
          toward and away from the mesh centroid.
        - **Stress** — per-element von Mises values drawn from a Gaussian whose
          mean varies sinusoidally over time, mimicking a stress wave.
        """
    ),
    code(
        """
        N_STRESS_FRAMES = 24
        rng = np.random.default_rng(42)

        veg     = read_veg(str(VEG_DIR / "bunny.veg"))
        md_mesh = veg.mesh_data
        n_verts = md_mesh.num_vertices
        n_elems = md_mesh.num_elements
        print(f"bunny.veg: {n_verts} vertices, {n_elems} tet elements")

        centroid  = md_mesh.vertices.mean(axis=0)
        radial    = md_mesh.vertices - centroid
        radial   /= np.linalg.norm(radial, axis=1, keepdims=True).clip(1e-8)
        bbox_diag = float(np.linalg.norm(
            md_mesh.vertices.max(axis=0) - md_mesh.vertices.min(axis=0)))
        breath_amp = bbox_diag * 0.04

        ts         = np.linspace(0, 2 * np.pi, N_STRESS_FRAMES, endpoint=False)
        sim_dir    = tmpdir / "stress_sim"
        states_dir = sim_dir / "states"
        stress_dir = sim_dir / "stress"
        states_dir.mkdir(parents=True)
        stress_dir.mkdir()

        for f in range(N_STRESS_FRAMES):
            disp = (radial * (breath_amp * np.sin(ts[f]))).ravel()[:, None]  # (3n, 1)
            write_u_file(states_dir / f"deform{f:04d}.u", disp)

            stress_mean = 1500.0 + 800.0 * np.sin(ts[f])
            values = np.abs(rng.normal(stress_mean, stress_mean * 0.25, n_elems))
            with open(stress_dir / f"von_mises{f:04d}.json", "w") as fp:
                json.dump({
                    "frame": f, "time": f / 24.0,
                    "stress_type": "von_mises", "location": "element",
                    "values": values.tolist(),
                }, fp)

        print(f"Wrote {N_STRESS_FRAMES} frames to {sim_dir}")
        """
    ),
    md(
        """
        ## 7. `compute_stress_field_stats`

        Aggregates per-frame statistics (min, mean, stddev, median, p99, max)
        and saves a single summary JSON with stress statistics
        tool format exactly. Works for **any mesh type** — tet or cubic — because
        it only reads the JSON files and is mesh-agnostic.
        """
    ),
    code(
        """
        stats = compute_stress_field_stats(
            stress_dir,
            prefix="von_mises",
            frame_start=0,
            frame_end=N_STRESS_FRAMES,
        )

        print(f"stress_type : {stats.stress_type!r}")
        print(f"frames      : {stats.num_frames}")
        print()
        print(f"{'frame':>5}  {'time':>6}  {'mean':>8}  {'p99':>8}  {'max':>8}")
        print("-" * 44)
        for s in stats.frames:
            print(f"{s.frame:>5}  {s.time:>6.3f}  {s.mean:>8.1f}  {s.p99:>8.1f}  {s.max:>8.1f}")
        """
    ),
    code(
        """
        summary_path = sim_dir / "stress_stats.json"
        stats.save(summary_path)
        print(f"Saved: {summary_path}  ({summary_path.stat().st_size} bytes)")

        with open(summary_path) as fp:
            doc = json.load(fp)
        print("top-level keys:", list(doc.keys()))
        print("frame[0]:", doc["frames"][0])
        """
    ),
    code(
        """
        try:
            import matplotlib.pyplot as plt

            frames_idx = [s.frame for s in stats.frames]
            means  = [s.mean  for s in stats.frames]
            p99s   = [s.p99   for s in stats.frames]
            maxs   = [s.max   for s in stats.frames]

            fig, ax = plt.subplots(figsize=(8, 3))
            ax.plot(frames_idx, means, label="mean",  linewidth=2)
            ax.plot(frames_idx, p99s,  label="p99",   linewidth=2, linestyle="--")
            ax.plot(frames_idx, maxs,  label="max",   linewidth=1, linestyle=":")
            ax.fill_between(frames_idx,
                            [s.mean - s.stddev for s in stats.frames],
                            [s.mean + s.stddev for s in stats.frames],
                            alpha=0.2, label="±1σ")
            ax.set_xlabel("frame")
            ax.set_ylabel("von Mises stress (Pa)")
            ax.set_title("Synthetic stress field — per-frame statistics")
            ax.legend()
            plt.tight_layout()
            plt.show()
        except ImportError:
            print("matplotlib not installed — skipping plot (mamba install matplotlib).")
        """
    ),
    md(
        """
        ## 8. `dump_stress_vdb` — OpenVDB export

        `dump_stress_vdb` reads the `states/` and `stress/` layout and writes one
        `.vdb` file per frame. Each file contains a `FloatGrid` named `"von_mises"`
        splatted at the deformed tet element positions.

        > **Tet-only limitation.** `dump_stress_vdb` uses `StressFieldVDBExporter`
        > internally, which only supports **tetrahedral** meshes. Cubic hex meshes
        > are not supported for VDB export. Use `compute_stress_field_stats` (section 7)
        > for mesh-agnostic per-frame statistics on hex outputs.

        `voxel_size=0.0` auto-derives the voxel size from ~½ the rest mesh's average
        tet edge length.
        """
    ),
    code(
        """
        if has_stress_vdb_export():
            vdb_dir = sim_dir / "vdb"
            n_written = dump_stress_vdb(
                veg_path=VEG_DIR / "bunny.veg",
                sim_output=sim_dir,
                output_dir=vdb_dir,
                prefix="vonMises",
                voxel_size=0.0,
                frame_start=0,
                frame_end=N_STRESS_FRAMES,
            )

            vdb_files = sorted(vdb_dir.glob("*.vdb"))
            print(f"Wrote {n_written} VDB frames to {vdb_dir}")
            print()
            print(f"{'file':<26}  {'size (KB)':>10}")
            print("-" * 40)
            for vf in vdb_files:
                print(f"{vf.name:<26}  {vf.stat().st_size / 1024:>10.1f}")
        else:
            print("Skipping: OpenVDB not available in this build.")
            print("Reconfigure with PGO_ENABLE_OPENVDB=ON to enable VDB export.")
        """
    ),
    md(
        """
        ## 9. Cleanup
        """
    ),
    code(
        """
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
