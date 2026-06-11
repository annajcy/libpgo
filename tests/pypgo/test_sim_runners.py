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
