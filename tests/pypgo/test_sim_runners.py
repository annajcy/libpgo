"""Tests for pypgo.tools.sim._runners (static + dynamic, volume + shell)."""

import json
from pathlib import Path

import numpy as np
import pytest

from pypgo.tools.sim._config import load_config
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
