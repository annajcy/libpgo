"""Tests for pypgo.tools.sim._config."""

import json

import pytest

from pypgo.tools.sim._config import (
    ConfigError,
    RegionSelector,
    SimConfig,
    VertexSelector,
    load_config,
)


def _write(tmp_path, payload, name="scene.json"):
    p = tmp_path / name
    p.write_text(json.dumps(payload))
    return p


def test_minimal_volume_dynamic_config(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "model.veg", "surface": "model.obj"},
        "dynamic": {"timestep": 0.001, "num_steps": 5},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.mesh_type == "tet"
    assert cfg.mode == "dynamic"
    # JSON-relative path resolution
    assert cfg.mesh.volume == tmp_path / "model.veg"
    assert cfg.mesh.surface == tmp_path / "model.obj"
    assert cfg.output.directory == tmp_path / "out"
    assert cfg.dynamic.timestep == 0.001
    assert cfg.dynamic.num_steps == 5
    # defaults
    assert cfg.mesh.formulation == "auto"
    assert cfg.material.model == "stable_neo"
    assert cfg.dynamic.integrator == "implicit_euler"
    assert cfg.solver.max_iterations == 50
    assert cfg.loads.gravity == (0.0, 0.0, 0.0)
    assert cfg.contact == ()


def test_cli_overrides_beat_json(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "model.veg", "surface": "model.obj"},
        "dynamic": {"timestep": 0.001, "num_steps": 100},
        "output": {"directory": "out"},
    })
    cfg = load_config(
        mesh_type="tet", mode="dynamic", json_path=cfg_path,
        overrides={"dynamic.num_steps": 2, "loads.gravity": (0.0, -9.81, 0.0)},
    )
    assert cfg.dynamic.num_steps == 2
    assert cfg.dynamic.timestep == 0.001  # untouched JSON value survives
    assert cfg.loads.gravity == (0.0, -9.81, 0.0)


def test_pure_cli_no_json():
    cfg = load_config(
        mesh_type="cubic", mode="dynamic",
        overrides={
            "mesh.volume": "/abs/model.veg",
            "mesh.surface": "/abs/model.obj",
            "dynamic.timestep": 0.01,
            "output.directory": "/abs/out",
        },
    )
    assert str(cfg.mesh.volume) == "/abs/model.veg"
    assert cfg.dynamic.timestep == 0.01


def test_type_field_mismatch_rejected(tmp_path):
    cfg_path = _write(tmp_path, {
        "type": "shell",
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.01},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="type"):
        load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)


def test_dynamic_requires_timestep(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="timestep"):
        load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)


def test_static_ignores_dynamic_section(tmp_path, capsys):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.01},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="static", json_path=cfg_path)
    assert cfg.mode == "static"  # no error; warning printed to stderr
    assert "dynamic" in capsys.readouterr().err


def test_volume_mesh_required_for_tet():
    with pytest.raises(ConfigError, match="volume"):
        load_config(mesh_type="tet", mode="static",
                    overrides={"mesh.surface": "/abs/m.obj",
                               "output.directory": "/abs/out"})


def test_shell_config(tmp_path):
    cfg_path = _write(tmp_path, {
        "type": "shell",
        "mesh": {"surface": "shell.obj"},
        "material": {"thickness": 0.002, "E_membrane": 2.0e4,
                     "nu_membrane": 0.35, "mass": {"density": 500.0}},
        "constraints": {"fixed": {"region": {"axis": "y", "side": "max"}}},
        "loads": {"gravity": [0.0, 0.0, -9.81]},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="shell", mode="static", json_path=cfg_path)
    assert cfg.material.thickness == 0.002
    assert cfg.material.mass.density == 500.0
    assert cfg.material.mass.areal_density is None
    assert cfg.constraints.fixed.region == RegionSelector(axis="y", side="max")


def test_shell_rejects_volume_mesh(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "shell.obj"},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="volume"):
        load_config(mesh_type="shell", mode="static", json_path=cfg_path)


def test_shell_mass_requires_exactly_one_source(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"surface": "shell.obj"},
        "material": {"mass": {"density": 500.0, "areal_density": 1.0}},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="mass"):
        load_config(mesh_type="shell", mode="static", json_path=cfg_path)


def test_vertex_selector_exactly_one_source():
    with pytest.raises(ConfigError):
        VertexSelector(indices=(1, 2), region=RegionSelector(axis="x", side="min"))
    with pytest.raises(ConfigError):
        VertexSelector()


def test_contact_config(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "contact": [
            {"model": "ipc", "dhat": 0.002, "dhat_external": 0.005,
             "kappa": 3000.0,
             "obstacles": [{"mesh": "bottom.obj"},
                           {"mesh": "mover.obj", "velocity": [0.0, 0.1, 0.0]}]},
            {"model": "floor", "axis": "y", "height": 0.013, "stiffness": 5000.0},
        ],
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    ipc, floor = cfg.contact
    assert ipc.model == "ipc" and ipc.kappa == 3000.0
    assert ipc.obstacles[0].mesh == tmp_path / "bottom.obj"
    assert ipc.obstacles[0].velocity is None
    assert ipc.obstacles[1].velocity == (0.0, 0.1, 0.0)
    assert floor.model == "floor" and floor.height == 0.013


def test_unknown_contact_model_rejected(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "contact": [{"model": "magic"}],
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="contact"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_obstacles_only_for_ipc(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "contact": [{"model": "floor", "obstacles": [{"mesh": "b.obj"}]}],
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="obstacle"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_output_directory_required():
    with pytest.raises(ConfigError, match="output"):
        load_config(mesh_type="tet", mode="static",
                    overrides={"mesh.volume": "/m.veg", "mesh.surface": "/m.obj"})


def test_hermite_formulation_only_for_cubic(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj",
                 "formulation": "cubic-tricubic-hermite"},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="formulation"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


# ---------------------------------------------------------------------------
# New tests covering review-finding fixes
# ---------------------------------------------------------------------------


def test_region_selector_missing_side_raises_config_error(tmp_path):
    """Fix 2a: missing 'side' in region must raise ConfigError, not KeyError."""
    cfg_path = _write(tmp_path, {
        "mesh": {"surface": "shell.obj"},
        "constraints": {"fixed": {"region": {"axis": "y"}}},  # 'side' absent
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="side"):
        load_config(mesh_type="shell", mode="static", json_path=cfg_path)


def test_attachment_missing_vertices_raises_config_error(tmp_path):
    """Fix 2b: attachment entry without 'vertices' must raise ConfigError, not KeyError."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "constraints": {"attachments": [{"coeff": 1e4}]},  # 'vertices' absent
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="vertices"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_floor_contact_invalid_axis_raises_config_error(tmp_path):
    """Fix 3: floor contact with an invalid axis must raise ConfigError at load time."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "contact": [{"model": "floor", "axis": "w"}],
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="axis"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_damping_length_check_applies_in_static_mode(tmp_path):
    """Fix 4: bad damping length must be caught even in static mode."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"damping": [1, 2, 3]},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="damping"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_frictional_contact_rejected_in_static_mode(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "contact": [{"model": "frictional_sampled_penalty"}],
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="frictional"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)
