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
    assert cfg.dynamic.resume is None
    assert cfg.solver.max_iterations == 50
    assert cfg.loads.gravity == (0.0, 0.0, 0.0)
    assert cfg.contact == ()
    assert cfg.output.write_checkpoints is False


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


# ---------------------------------------------------------------------------
# Task 7: movement, dump_interval, enable_material_max_step
# ---------------------------------------------------------------------------


def test_attachment_movement_parsed(tmp_path):
    """movement field is parsed into a 3-tuple on AttachmentConfig."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "constraints": {"attachments": [
            {"vertices": {"indices": [0]}, "coeff": 5e4,
             "movement": [0.0, -1.0, 0.0]},
        ]},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    att = cfg.constraints.attachments[0]
    assert att.movement == (0.0, -1.0, 0.0)


def test_attachment_movement_none_by_default(tmp_path):
    """Omitting movement yields None (plain static attachment)."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "constraints": {"attachments": [
            {"vertices": {"indices": [0]}, "coeff": 1e5},
        ]},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.constraints.attachments[0].movement is None


def test_static_mode_with_nonzero_movement_rejected(tmp_path):
    """Nonzero movement on a static-mode config must raise ConfigError."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "constraints": {"attachments": [
            {"vertices": {"indices": [0]}, "movement": [0.0, -1.0, 0.0]},
        ]},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="movement"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_static_mode_zero_movement_allowed(tmp_path):
    """Zero movement vector in static mode is fine (treated as plain attachment)."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "constraints": {"attachments": [
            {"vertices": {"indices": [0]}, "movement": [0.0, 0.0, 0.0]},
        ]},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="static", json_path=cfg_path)
    assert cfg.constraints.attachments[0].movement == (0.0, 0.0, 0.0)


def test_dump_interval_default_is_1(tmp_path):
    """OutputConfig.dump_interval defaults to 1."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.output.dump_interval == 1


def test_dump_interval_parsed(tmp_path):
    """dump_interval round-trips from JSON."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out", "dump_interval": 10},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.output.dump_interval == 10


def test_dump_interval_zero_rejected(tmp_path):
    """dump_interval < 1 must raise ConfigError."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out", "dump_interval": 0},
    })
    with pytest.raises(ConfigError, match="dump_interval"):
        load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)


def test_enable_material_max_step_defaults_true_volume(tmp_path):
    """VolumeMaterialConfig.enable_material_max_step defaults to True."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.material.enable_material_max_step is True


def test_enable_material_max_step_false_volume(tmp_path):
    """enable_material_max_step=false round-trips for VolumeMaterialConfig."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "material": {"model": "stable_neo", "enable_material_max_step": False},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.material.enable_material_max_step is False


def test_enable_material_max_step_defaults_true_shell(tmp_path):
    """ShellMaterialConfig.enable_material_max_step defaults to True."""
    cfg_path = _write(tmp_path, {
        "mesh": {"surface": "shell.obj"},
        "material": {"mass": {"density": 1000.0}},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="shell", mode="static", json_path=cfg_path)
    assert cfg.material.enable_material_max_step is True


# ---------------------------------------------------------------------------
# Task 11: write_stress config tests
# ---------------------------------------------------------------------------


def test_write_stress_defaults_false(tmp_path):
    """OutputConfig.write_stress defaults to False."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.output.write_stress is False


def test_write_stress_volume_accepted(tmp_path):
    """write_stress=True is accepted for volume (tet/cubic) mesh types."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out", "write_stress": True},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.output.write_stress is True


def test_write_stress_shell_accepted(tmp_path):
    """write_stress=True is now accepted for shell mesh types (Task 12)."""
    cfg_path = _write(tmp_path, {
        "mesh": {"surface": "shell.obj"},
        "material": {"mass": {"density": 1000.0}},
        "output": {"directory": "out", "write_stress": True},
    })
    cfg = load_config(mesh_type="shell", mode="static", json_path=cfg_path)
    assert cfg.output.write_stress is True


def test_enable_material_max_step_false_shell(tmp_path):
    """enable_material_max_step=false round-trips for ShellMaterialConfig."""
    cfg_path = _write(tmp_path, {
        "mesh": {"surface": "shell.obj"},
        "material": {"mass": {"density": 1000.0}, "enable_material_max_step": False},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="shell", mode="static", json_path=cfg_path)
    assert cfg.material.enable_material_max_step is False


# ---------------------------------------------------------------------------
# Task 10: write_states
# ---------------------------------------------------------------------------


def test_write_states_default_false(tmp_path):
    """OutputConfig.write_states defaults to False."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.output.write_states is False


def test_write_states_parsed_true(tmp_path):
    """write_states=true round-trips from JSON."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out", "write_states": True},
    })
    cfg = load_config(mesh_type="tet", mode="dynamic", json_path=cfg_path)
    assert cfg.output.write_states is True


def test_write_checkpoints_default_false_and_parsed(tmp_path):
    """OutputConfig.write_checkpoints defaults to False and round-trips from JSON."""
    base = {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    }
    cfg = load_config(mesh_type="tet", mode="dynamic",
                      json_path=_write(tmp_path, base, "a.json"))
    assert cfg.output.write_checkpoints is False

    base["output"]["write_checkpoints"] = True
    cfg = load_config(mesh_type="tet", mode="dynamic",
                      json_path=_write(tmp_path, base, "b.json"))
    assert cfg.output.write_checkpoints is True


def test_dynamic_resume_default_latest_and_path(tmp_path):
    """dynamic.resume accepts None, 'latest', and explicit checkpoint paths."""
    base = {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001, "resume": "latest"},
        "output": {"directory": "out"},
    }
    cfg = load_config(mesh_type="tet", mode="dynamic",
                      json_path=_write(tmp_path, base, "latest.json"))
    assert cfg.dynamic.resume == "latest"

    ckpt = tmp_path / "state0002.npz"
    cfg = load_config(
        mesh_type="tet", mode="dynamic",
        json_path=_write(tmp_path, {
            "mesh": {"volume": "m.veg", "surface": "m.obj"},
            "dynamic": {"timestep": 0.001, "resume": str(ckpt)},
            "output": {"directory": "out"},
        }, "path.json"),
    )
    assert cfg.dynamic.resume == ckpt


def test_dynamic_resume_rejected_in_static_mode(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"resume": "latest"},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="resume"):
        load_config(mesh_type="tet", mode="static", json_path=cfg_path)


def test_dynamic_checkpoint_resume_cli_overrides(tmp_path):
    from pypgo.tools.sim._cli import _overrides_from_args, build_parser

    parser = build_parser(
        prog="pypgo-sim-tet-dynamic",
        mesh_type="tet",
        mode="dynamic",
    )
    args = parser.parse_args([
        "--volume", str(tmp_path / "m.veg"),
        "--surface", str(tmp_path / "m.obj"),
        "--output-dir", str(tmp_path / "out"),
        "--timestep", "0.001",
        "--write-checkpoints",
        "--resume", "latest",
    ])
    overrides = _overrides_from_args(args, mesh_type="tet", mode="dynamic")
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides=overrides)
    assert cfg.output.write_checkpoints is True
    assert cfg.dynamic.resume == "latest"


# ---------------------------------------------------------------------------
# write_abc
# ---------------------------------------------------------------------------


def test_write_abc_default_false_and_parsed(tmp_path):
    """OutputConfig.write_abc defaults to False and round-trips from JSON."""
    base = {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "dynamic": {"timestep": 0.001},
        "output": {"directory": "out"},
    }
    cfg = load_config(mesh_type="tet", mode="dynamic",
                      json_path=_write(tmp_path, base, "a.json"))
    assert cfg.output.write_abc is False

    base["output"]["write_abc"] = True
    cfg = load_config(mesh_type="tet", mode="dynamic",
                      json_path=_write(tmp_path, base, "b.json"))
    assert cfg.output.write_abc is True


def test_surface_attachments_parsed(tmp_path):
    """constraints.surface_attachments parse with selector + coeff."""
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "constraints": {
            "surface_attachments": [
                {"vertices": {"indices": [0, 5]}, "coeff": 2e4},
            ],
        },
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="cubic", mode="static", json_path=cfg_path)
    assert len(cfg.constraints.surface_attachments) == 1
    sa = cfg.constraints.surface_attachments[0]
    assert sa.coeff == 2e4
    assert sa.vertices.indices == (0, 5)


def test_surface_attachment_missing_vertices_rejected(tmp_path):
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "constraints": {"surface_attachments": [{"coeff": 1.0}]},
        "output": {"directory": "out"},
    })
    with pytest.raises(ConfigError, match="vertices"):
        load_config(mesh_type="cubic", mode="static", json_path=cfg_path)


def test_surface_attachment_file_path_resolved(tmp_path):
    """surface_attachments[].vertices.file resolves relative to the JSON dir."""
    (tmp_path / "patch.txt").write_text("0 1 2\n")
    cfg_path = _write(tmp_path, {
        "mesh": {"volume": "m.veg", "surface": "m.obj"},
        "constraints": {
            "surface_attachments": [
                {"vertices": {"file": "patch.txt"}, "coeff": 1.0},
            ],
        },
        "output": {"directory": "out"},
    })
    cfg = load_config(mesh_type="tet", mode="static", json_path=cfg_path)
    assert cfg.constraints.surface_attachments[0].vertices.file == tmp_path / "patch.txt"
