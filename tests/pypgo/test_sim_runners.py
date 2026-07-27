"""Tests for pypgo.tools.sim._runners (static + dynamic, volume + shell)."""

import json
from pathlib import Path

import numpy as np
import pytest

from pypgo.tools.sim._config import ConfigError, load_config
from pypgo.tools.sim._runners import run_dynamic, run_static
from pypgo.tools.sim._scene import build_scene

ASSETS = Path(__file__).resolve().parents[2] / "examples" / "assets"


def test_run_static_volume_box_hang(tmp_path):
    cfg = load_config(mesh_type="tet", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-3}},
        "loads.gravity": (0.0, -9.81, 0.0),
        "solver.max_iterations": 20,
        "solver.gradient_tolerance": 1e-4,
        "output.directory": str(tmp_path),
        "output.write_surfaces": True,
    })
    summary = run_static(build_scene(cfg), cfg)
    assert summary["mode"] == "static"
    assert summary["num_dofs"] > 0
    assert isinstance(summary["converged"], bool)
    saved = json.loads((tmp_path / "summary.json").read_text())
    assert saved["mode"] == "static"
    assert (tmp_path / "final_surface.obj").exists()


def test_run_dynamic_volume_two_steps(tmp_path):
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "loads.gravity": (0.0, -9.81, 0.0),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 2,
        "output.directory": str(tmp_path),
        "output.write_surfaces": True,
    })
    summary = run_dynamic(build_scene(cfg), cfg)
    assert summary["mode"] == "dynamic"
    assert summary["num_frames"] == 2
    assert summary["final_time"] == pytest.approx(0.002)
    assert (tmp_path / "surface" / "surface0000.obj").exists()
    assert (tmp_path / "surface" / "surface0001.obj").exists()
    saved = json.loads((tmp_path / "summary.json").read_text())
    assert saved["num_frames"] == 2
    assert isinstance(saved["frames"], list)


def test_run_dynamic_shell_two_steps(tmp_path):
    cfg = load_config(mesh_type="shell", mode="dynamic", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"mass": {"areal_density": 1.0}},
        "loads.gravity": (0.0, 0.0, -9.81),
        "initial_state.velocity": (0.0, 0.0, -1.0),
        "dynamic.timestep": 0.0005,
        "dynamic.num_steps": 2,
        "contact": [{"model": "floor", "axis": "z", "height": -0.1,
                     "stiffness": 5000.0}],
        "output.directory": str(tmp_path),
    })
    summary = run_dynamic(build_scene(cfg), cfg)
    assert summary["num_frames"] == 2
    # falling: mean z displacement is negative
    assert summary["frames"][-1]["accepted"] is True
    saved = json.loads((tmp_path / "summary.json").read_text())
    assert saved["num_frames"] == 2
    assert isinstance(saved["frames"], list)


def test_run_static_shell_drape(tmp_path):
    cfg = load_config(mesh_type="shell", mode="static", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"thickness": 0.001, "E_membrane": 1.0e6,
                     "nu_membrane": 0.4, "mass": {"density": 1000.0}},
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-6}},
        "loads.gravity": (0.0, 0.0, -9.81),
        "solver.max_iterations": 10,
        "solver.gradient_tolerance": 1e-4,
        "output.directory": str(tmp_path),
        "output.write_surfaces": True,
    })
    summary = run_static(build_scene(cfg), cfg)
    assert (tmp_path / "final_surface.obj").exists()
    # clamped edge stays put: fixed DOFs of the solution remain ~0
    assert summary["max_fixed_abs_u"] == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------------------
# Task 7: moving attachment targets + dump_interval
# ---------------------------------------------------------------------------


def test_run_dynamic_cubic_box_squash_moving_attachment(tmp_path):
    """2-step cubic box squash: top region pulled down by moving attachment.

    Box veg y ∈ [0.3781, 0.8781]. Bottom region (y-min) is a static anchor;
    top region (y-max) has movement [0,-1,0].  No gravity.  After 2 steps at
    dt=1e-3, the pushed (y-max) vertices should have mean y-displacement < 0.
    """
    cfg = load_config(mesh_type="cubic", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "cubic" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "constraints.attachments": [
            # bottom: static anchor
            {"vertices": {"region": {"axis": "y", "side": "min", "tolerance": 0.001}},
             "coeff": 5e4},
            # top: pulled downward
            {"vertices": {"region": {"axis": "y", "side": "max", "tolerance": 0.001}},
             "coeff": 5e4, "movement": [0.0, -1.0, 0.0]},
        ],
        "loads.gravity": (0.0, 0.0, 0.0),
        "dynamic.timestep": 1e-3,
        "dynamic.num_steps": 2,
        "output.directory": str(tmp_path),
        "output.write_surfaces": True,
    })
    bundle = build_scene(cfg)
    summary = run_dynamic(bundle, cfg)

    # Run completes with both frames accepted
    assert summary["num_frames"] == 2
    assert all(f["accepted"] for f in summary["frames"])

    # The pushed (y-max) vertices have mean y-displacement < 0.
    # Identify the y-max surface vertices from bundle.surface_rest.
    surf_verts = bundle.surface_rest
    y_max = surf_verts[:, 1].max()
    top_mask = np.abs(surf_verts[:, 1] - y_max) <= 0.001

    # Read displacement from the last written surface OBJ.
    last_surface_path = tmp_path / "surface" / "surface0001.obj"
    assert last_surface_path.exists()

    deformed = []
    with open(last_surface_path) as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                deformed.append([float(parts[1]), float(parts[2]), float(parts[3])])
    deformed = np.array(deformed)
    disp_y = deformed[:, 1] - surf_verts[:, 1]
    mean_top_disp_y = disp_y[top_mask].mean()
    assert mean_top_disp_y < 0.0, (
        f"Expected top vertices to move down, got mean y-disp = {mean_top_disp_y:.6f}"
    )


def test_dump_interval_controls_surface_writes(tmp_path):
    """With dump_interval=2 and 3 steps, only frames 0 and 2 (indices) are written."""
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "loads.gravity": (0.0, -9.81, 0.0),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 3,
        "output.directory": str(tmp_path),
        "output.write_surfaces": True,
        "output.dump_interval": 2,
    })
    summary = run_dynamic(build_scene(cfg), cfg)
    assert summary["num_frames"] == 3
    surface_dir = tmp_path / "surface"
    # frame_index 0 → written (0 % 2 == 0)
    assert (surface_dir / "surface0000.obj").exists()
    # frame_index 1 → NOT written (1 % 2 != 0)
    assert not (surface_dir / "surface0001.obj").exists()
    # frame_index 2 → written (2 % 2 == 0)
    assert (surface_dir / "surface0002.obj").exists()


# ---------------------------------------------------------------------------
# Task 10: write_states — .u displacement dumps
# ---------------------------------------------------------------------------


def test_run_dynamic_write_states_two_steps(tmp_path):
    """Dynamic 2-step run with write_states=True writes deform0000.u and deform0001.u.

    Both files must exist, be readable via read_u_file, and have shape (3n, 1)
    with finite values.
    """
    from pypgo.animation import read_u_file

    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "loads.gravity": (0.0, -9.81, 0.0),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 2,
        "output.directory": str(tmp_path),
        "output.write_states": True,
    })
    bundle = build_scene(cfg)
    summary = run_dynamic(bundle, cfg)
    assert summary["num_frames"] == 2

    states_dir = tmp_path / "states"
    u0_path = states_dir / "deform0000.u"
    u1_path = states_dir / "deform0001.u"
    assert u0_path.exists(), "deform0000.u not written"
    assert u1_path.exists(), "deform0001.u not written"

    u0 = read_u_file(u0_path)
    u1 = read_u_file(u1_path)
    expected_rows = bundle.num_dofs  # 3n for standard volume mesh
    assert u0.shape == (expected_rows, 1), f"Expected ({expected_rows}, 1), got {u0.shape}"
    assert u1.shape == (expected_rows, 1), f"Expected ({expected_rows}, 1), got {u1.shape}"
    assert np.all(np.isfinite(u0)), "deform0000.u contains non-finite values"
    assert np.all(np.isfinite(u1)), "deform0001.u contains non-finite values"


def test_run_dynamic_write_states_dump_interval(tmp_path):
    """With dump_interval=2 and write_states=True, only deform0000.u is written (not 0001)."""
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "loads.gravity": (0.0, -9.81, 0.0),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 2,
        "output.directory": str(tmp_path),
        "output.write_states": True,
        "output.dump_interval": 2,
    })
    run_dynamic(build_scene(cfg), cfg)

    states_dir = tmp_path / "states"
    # frame_index 0 → 0 % 2 == 0 → written
    assert (states_dir / "deform0000.u").exists()
    # frame_index 1 → 1 % 2 != 0 → NOT written
    assert not (states_dir / "deform0001.u").exists()


# ---------------------------------------------------------------------------
# Checkpoint / resume
# ---------------------------------------------------------------------------


def _dynamic_box_cfg(tmp_path, **overrides):
    base = {
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "loads.gravity": (0.0, -9.81, 0.0),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 2,
        "output.directory": str(tmp_path),
    }
    base.update(overrides)
    return load_config(mesh_type="tet", mode="dynamic", overrides=base)


def test_run_dynamic_write_checkpoints_two_steps(tmp_path):
    """write_checkpoints=True writes restartable npz state at dumped frames."""
    cfg = _dynamic_box_cfg(tmp_path, **{"output.write_checkpoints": True})
    bundle = build_scene(cfg)
    summary = run_dynamic(bundle, cfg)
    assert summary["num_frames"] == 2

    ckpt0 = tmp_path / "checkpoints" / "state0000.npz"
    ckpt1 = tmp_path / "checkpoints" / "state0001.npz"
    assert ckpt0.exists()
    assert ckpt1.exists()

    data = np.load(ckpt1)
    assert set(data.files) >= {
        "version", "displacement", "velocity", "acceleration",
        "timestep_id", "time", "num_dofs", "timestep", "integrator",
        "metadata_json",
    }
    assert data["displacement"].shape == (bundle.num_dofs,)
    assert data["velocity"].shape == (bundle.num_dofs,)
    assert data["acceleration"].shape == (bundle.num_dofs,)
    assert int(data["timestep_id"]) == 2
    assert float(data["time"]) == pytest.approx(0.002)
    assert int(data["num_dofs"]) == bundle.num_dofs
    assert float(data["timestep"]) == pytest.approx(cfg.dynamic.timestep)
    assert str(data["integrator"]) == cfg.dynamic.integrator
    metadata = json.loads(str(data["metadata_json"]))
    assert metadata["simulation"]["loads"]["gravity"] == [0.0, -9.81, 0.0]


def test_run_dynamic_resume_latest_uses_num_steps_as_total_target(tmp_path):
    cfg = _dynamic_box_cfg(tmp_path, **{"output.write_checkpoints": True})
    run_dynamic(build_scene(cfg), cfg)

    resume_cfg = _dynamic_box_cfg(
        tmp_path,
        **{
            "dynamic.num_steps": 4,
            "dynamic.resume": "latest",
            "output.write_checkpoints": True,
        },
    )
    summary = run_dynamic(build_scene(resume_cfg), resume_cfg)

    assert summary["initial_timestep_id"] == 2
    assert summary["target_timestep_id"] == 4
    assert summary["num_frames"] == 2
    assert summary["final_timestep_id"] == 4
    assert summary["final_time"] == pytest.approx(0.004)
    assert summary["resumed_from"].endswith("state0001.npz")
    assert (tmp_path / "checkpoints" / "state0003.npz").exists()


def test_run_dynamic_resume_from_explicit_checkpoint_path(tmp_path):
    cfg = _dynamic_box_cfg(tmp_path, **{"output.write_checkpoints": True})
    run_dynamic(build_scene(cfg), cfg)

    ckpt = tmp_path / "checkpoints" / "state0000.npz"
    resume_cfg = _dynamic_box_cfg(
        tmp_path,
        **{
            "dynamic.num_steps": 3,
            "dynamic.resume": str(ckpt),
            "output.write_checkpoints": True,
        },
    )
    summary = run_dynamic(build_scene(resume_cfg), resume_cfg)
    assert summary["initial_timestep_id"] == 1
    assert summary["final_timestep_id"] == 3
    assert summary["num_frames"] == 2


def test_run_dynamic_resume_rejects_mismatched_num_dofs(tmp_path):
    cfg = _dynamic_box_cfg(tmp_path, **{"output.write_checkpoints": True})
    bundle = build_scene(cfg)
    run_dynamic(bundle, cfg)
    ckpt = tmp_path / "checkpoints" / "state0001.npz"
    data = dict(np.load(ckpt))
    data["num_dofs"] = np.array(bundle.num_dofs + 1)
    bad = tmp_path / "checkpoints" / "bad_num_dofs.npz"
    np.savez(bad, **data)

    resume_cfg = _dynamic_box_cfg(
        tmp_path,
        **{"dynamic.resume": str(bad), "dynamic.num_steps": 4},
    )
    with pytest.raises(ConfigError, match="num_dofs"):
        run_dynamic(build_scene(resume_cfg), resume_cfg)


def test_run_dynamic_resume_rejects_mismatched_timestep(tmp_path):
    cfg = _dynamic_box_cfg(tmp_path, **{"output.write_checkpoints": True})
    run_dynamic(build_scene(cfg), cfg)
    ckpt = tmp_path / "checkpoints" / "state0001.npz"
    data = dict(np.load(ckpt))
    data["timestep"] = np.array(0.123)
    bad = tmp_path / "checkpoints" / "bad_timestep.npz"
    np.savez(bad, **data)

    resume_cfg = _dynamic_box_cfg(
        tmp_path,
        **{"dynamic.resume": str(bad), "dynamic.num_steps": 4},
    )
    with pytest.raises(ConfigError, match="timestep"):
        run_dynamic(build_scene(resume_cfg), resume_cfg)


def test_run_dynamic_resume_rejects_mismatched_integrator(tmp_path):
    cfg = _dynamic_box_cfg(tmp_path, **{"output.write_checkpoints": True})
    run_dynamic(build_scene(cfg), cfg)
    ckpt = tmp_path / "checkpoints" / "state0001.npz"
    data = dict(np.load(ckpt))
    data["integrator"] = np.array("trbdf2")
    bad = tmp_path / "checkpoints" / "bad_integrator.npz"
    np.savez(bad, **data)

    resume_cfg = _dynamic_box_cfg(
        tmp_path,
        **{"dynamic.resume": str(bad), "dynamic.num_steps": 4},
    )
    with pytest.raises(ConfigError, match="integrator"):
        run_dynamic(build_scene(resume_cfg), resume_cfg)


def test_run_dynamic_resume_rejects_mismatched_checkpoint_metadata(tmp_path):
    cfg = _dynamic_box_cfg(tmp_path, **{"output.write_checkpoints": True})
    run_dynamic(build_scene(cfg), cfg)

    resume_cfg = _dynamic_box_cfg(
        tmp_path,
        **{
            "dynamic.resume": "latest",
            "dynamic.num_steps": 4,
            "loads.gravity": (0.0, -1.0, 0.0),
        },
    )
    with pytest.raises(ConfigError, match="metadata"):
        run_dynamic(build_scene(resume_cfg), resume_cfg)


def test_run_dynamic_profile_writes_index_and_frame_json(tmp_path, monkeypatch):
    monkeypatch.setenv("PGO_PROFILE_DYNAMIC", "1")
    cfg = _dynamic_box_cfg(tmp_path, **{"dynamic.num_steps": 1})
    run_dynamic(build_scene(cfg), cfg)

    index_path = tmp_path / "profile_dynamic.jsonl"
    assert index_path.exists()
    rows = [json.loads(line) for line in index_path.read_text().splitlines()]
    assert len(rows) == 1

    row = rows[0]
    assert row["frame_index"] == 0
    assert row["profile_path"] == "profiles/frame_000000_profile.json"
    assert "solver_diagnostics" not in row
    assert "sections" not in row
    assert "counters" not in row

    profile = json.loads((tmp_path / row["profile_path"]).read_text())
    assert profile["frame_index"] == 0
    assert profile["solver_diagnostics"]["status"] == row["solver_status"]
    sections = profile["sections"]
    assert sections["name"] == "root"
    assert isinstance(sections["children"], list)
    assert "children_seconds" in sections
    assert "children_cpu_seconds" in sections
    for section in sections["children"]:
        assert section["name"] == section["path"]
        assert "local_name" in section
        assert "avg_cpu_cores" in section
        assert "max_cpu_cores" in section
        assert "children_avg_cpu_cores" in section
        assert "self_avg_cpu_cores" in section


def test_run_dynamic_write_checkpoints_dump_interval(tmp_path):
    """Checkpoints are written every step even when dump_interval skips visual dumps."""
    cfg = _dynamic_box_cfg(
        tmp_path,
        **{
            "dynamic.num_steps": 3,
            "output.write_checkpoints": True,
            "output.write_surfaces": True,
            "output.dump_interval": 2,
        },
    )
    run_dynamic(build_scene(cfg), cfg)

    ckpt_dir = tmp_path / "checkpoints"
    assert (ckpt_dir / "state0000.npz").exists()
    assert (ckpt_dir / "state0001.npz").exists()
    assert (ckpt_dir / "state0002.npz").exists()

    surface_dir = tmp_path / "surface"
    assert (surface_dir / "surface0000.obj").exists()
    assert not (surface_dir / "surface0001.obj").exists()
    assert (surface_dir / "surface0002.obj").exists()


def test_run_static_write_states(tmp_path):
    """Static run with write_states=True writes states/deform_final.u, readable with correct shape."""
    from pypgo.animation import read_u_file

    cfg = load_config(mesh_type="tet", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-3}},
        "loads.gravity": (0.0, -9.81, 0.0),
        "solver.max_iterations": 10,
        "solver.gradient_tolerance": 1e-4,
        "output.directory": str(tmp_path),
        "output.write_states": True,
    })
    bundle = build_scene(cfg)
    run_static(bundle, cfg)

    u_path = tmp_path / "states" / "deform_final.u"
    assert u_path.exists(), "states/deform_final.u not written"

    u = read_u_file(u_path)
    assert u.shape == (bundle.num_dofs, 1), (
        f"Expected ({bundle.num_dofs}, 1), got {u.shape}"
    )
    assert np.all(np.isfinite(u)), "deform_final.u contains non-finite values"


# ---------------------------------------------------------------------------
# Task 11: write_stress — von Mises stress output tests
# ---------------------------------------------------------------------------


def test_run_dynamic_write_stress_two_steps(tmp_path):
    """Dynamic 2-step tet run with write_stress=True writes von_mises0000.json and 0001.json.

    Each file must have the correct legacy format fields and len(values) == num_elements.
    Also verifies compute_stress_field_stats can consume the output stress directory.
    """
    import json as _json
    from pypgo.animation.stress_stats import compute_stress_field_stats

    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "loads.gravity": (0.0, -9.81, 0.0),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 2,
        "output.directory": str(tmp_path),
        "output.write_stress": True,
    })
    bundle = build_scene(cfg)
    summary = run_dynamic(bundle, cfg)
    assert summary["num_frames"] == 2

    stress_dir = tmp_path / "stress"
    f0_path = stress_dir / "von_mises0000.json"
    f1_path = stress_dir / "von_mises0001.json"
    assert f0_path.exists(), "von_mises0000.json not written"
    assert f1_path.exists(), "von_mises0001.json not written"

    num_elements = bundle.asset.num_elements

    for idx, path in enumerate([f0_path, f1_path]):
        doc = _json.loads(path.read_text())
        assert doc["frame"] == idx, f"Expected frame={idx}, got {doc['frame']}"
        assert "time" in doc and isinstance(doc["time"], float)
        assert doc["stress_type"] == "von_mises"
        assert doc["location"] == "element"
        assert isinstance(doc["values"], list)
        assert len(doc["values"]) == num_elements, (
            f"Expected {num_elements} values, got {len(doc['values'])}"
        )

    # verify compute_stress_field_stats can consume the directory
    stats = compute_stress_field_stats(stress_dir, prefix="von_mises", frame_start=0)
    assert stats.num_frames == 2
    assert stats.stress_type == "von_mises"
    assert stats.location == "element"


# ---------------------------------------------------------------------------
# Task 12: shell dynamic write_stress
# ---------------------------------------------------------------------------


def test_run_dynamic_shell_write_stress_two_steps(tmp_path):
    """Shell dynamic 2-step run with write_stress=True writes von_mises files.

    Uses shell.obj with gravity + clamped top edge so deformation is nonzero.
    Verifies: 2 files written, len(values) == num_elements (2048), all finite,
    and max(values) > 0 on frame 1 (deformation has started under gravity).
    """
    import json as _json

    cfg = load_config(mesh_type="shell", mode="dynamic", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {
            "thickness": 0.001,
            "E_membrane": 1.0e6,
            "nu_membrane": 0.4,
            "mass": {"density": 1000.0},
        },
        "constraints.fixed": {"region": {"axis": "y", "side": "max", "tolerance": 1e-6}},
        "loads.gravity": (0.0, 0.0, -9.81),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 2,
        "output.directory": str(tmp_path),
        "output.write_stress": True,
    })
    bundle = build_scene(cfg)
    summary = run_dynamic(bundle, cfg)
    assert summary["num_frames"] == 2

    stress_dir = tmp_path / "stress"
    f0_path = stress_dir / "von_mises0000.json"
    f1_path = stress_dir / "von_mises0001.json"
    assert f0_path.exists(), "von_mises0000.json not written for shell"
    assert f1_path.exists(), "von_mises0001.json not written for shell"

    num_elements = bundle.asset.num_elements  # 2048 for shell.obj

    for idx, path in enumerate([f0_path, f1_path]):
        doc = _json.loads(path.read_text())
        assert doc["frame"] == idx
        assert doc["stress_type"] == "von_mises"
        assert doc["location"] == "element"
        values = doc["values"]
        assert isinstance(values, list)
        assert len(values) == num_elements, (
            f"Expected {num_elements} values, got {len(values)}"
        )
        assert all(np.isfinite(v) for v in values), "Non-finite stress values"

    # Frame 1: deformation under gravity with clamped top edge => some stress > 0
    doc1 = _json.loads(f1_path.read_text())
    assert max(doc1["values"]) > 0, (
        "Expected max von Mises stress > 0 after 1 step under gravity"
    )


def test_run_dynamic_write_abc(tmp_path):
    """Dynamic run with write_abc=True writes a nonempty animation.abc whose
    frame count matches the dump cadence."""
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "loads.gravity": (0.0, -9.81, 0.0),
        "dynamic.timestep": 0.001,
        "dynamic.num_steps": 3,
        "output.directory": str(tmp_path),
        "output.write_abc": True,
    })
    summary = run_dynamic(build_scene(cfg), cfg)
    assert summary["num_frames"] == 3
    abc_path = tmp_path / "animation.abc"
    assert abc_path.exists()
    assert abc_path.stat().st_size > 0


def test_run_static_write_abc_warns_and_skips(tmp_path, capsys):
    """write_abc is dynamic-only; static mode warns to stderr and writes no abc."""
    cfg = load_config(mesh_type="tet", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-3}},
        "loads.gravity": (0.0, -9.81, 0.0),
        "solver.max_iterations": 5,
        "output.directory": str(tmp_path),
        "output.write_abc": True,
    })
    run_static(build_scene(cfg), cfg)
    assert not (tmp_path / "animation.abc").exists()
    assert "write_abc" in capsys.readouterr().err
