"""Tests for pypgo.tools.sim._scene."""

from pathlib import Path

import numpy as np
import pytest

from pypgo.mesh import read_obj
from pypgo.tools.sim._config import (
    ConfigError, RegionSelector, VertexSelector, load_config,
)
from pypgo.tools.sim._scene import build_scene, resolve_vertex_selector

ASSETS = Path(__file__).resolve().parents[2] / "examples" / "assets"


def test_missing_mesh_raises_config_error():
    cfg = load_config(mesh_type="tet", mode="static", overrides={
        "mesh.volume": "/nonexistent/missing.veg",
        "mesh.surface": "/nonexistent/missing.obj",
        "output.directory": "/tmp/unused",
    })
    with pytest.raises(ConfigError, match="cannot load"):
        build_scene(cfg)


def test_resolve_selector_indices():
    verts = np.zeros((5, 3))
    sel = VertexSelector(indices=(3, 1, 3))
    assert resolve_vertex_selector(sel, verts).tolist() == [1, 3]


def test_resolve_selector_region_max_y():
    verts = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]])
    sel = VertexSelector(region=RegionSelector(axis="y", side="max", tolerance=1e-9))
    assert resolve_vertex_selector(sel, verts).tolist() == [1, 2]


def test_resolve_selector_file(tmp_path):
    f = tmp_path / "fixed.txt"
    f.write_text("2 0\n1\n")
    sel = VertexSelector(file=f)
    assert resolve_vertex_selector(sel, np.zeros((4, 3))).tolist() == [0, 1, 2]


def test_resolve_selector_out_of_range():
    sel = VertexSelector(indices=(10,))
    with pytest.raises(ConfigError, match="range"):
        resolve_vertex_selector(sel, np.zeros((3, 3)))


def test_resolve_selector_empty_indices():
    sel = VertexSelector(indices=())
    with pytest.raises(ConfigError, match="no vertices"):
        resolve_vertex_selector(sel, np.zeros((3, 3)))


def test_resolve_selector_file_non_integer(tmp_path):
    f = tmp_path / "bad.txt"
    f.write_text("2 abc")
    sel = VertexSelector(file=f)
    with pytest.raises(ConfigError, match="non-integer"):
        resolve_vertex_selector(sel, np.zeros((5, 3)))


def test_hermite_fixed_dofs_pins_all_24():
    cfg = load_config(mesh_type="cubic", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "cubic" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "mesh.formulation": "cubic-tricubic-hermite",
        "constraints.fixed": {"region": {"axis": "y", "side": "min", "tolerance": 1e-3}},
        "output.directory": "/tmp/unused",
    })
    bundle = build_scene(cfg)
    assert bundle.fixed_dofs is not None
    assert bundle.fixed_dofs.size % 24 == 0


@pytest.fixture(scope="module")
def tet_box_cfg_payload():
    return {
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "output.directory": "/tmp/unused",
    }


def test_build_volume_scene_tet_static(tet_box_cfg_payload):
    cfg = load_config(mesh_type="tet", mode="static", overrides={
        **tet_box_cfg_payload,
        "loads.gravity": (0.0, -9.81, 0.0),
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-3}},
    })
    bundle = build_scene(cfg)
    n = bundle.num_dofs
    assert bundle.dofs_per_vertex == 3
    assert bundle.mass is None
    assert bundle.gravity_force.shape == (n,)
    assert float(np.linalg.norm(bundle.gravity_force)) > 0.0
    assert bundle.fixed_dofs is not None and bundle.fixed_dofs.size % 3 == 0
    # surface mapping: zero displacement keeps the rest surface
    rest = bundle.surface_positions(np.zeros(n))
    np.testing.assert_allclose(rest, bundle.surface_rest)
    # initial vector tiling
    v = bundle.initial_vector((0.0, -3.0, 0.0))
    assert v.shape == (n,)
    assert v[1] == -3.0 and v[0] == 0.0


def test_build_volume_scene_neo_hookean(tet_box_cfg_payload):
    cfg = load_config(mesh_type="tet", mode="static", overrides={
        **tet_box_cfg_payload,
        "material.model": "neo_hookean",
    })
    bundle = build_scene(cfg)
    assert bundle.deformation.num_elastic_params == 0
    assert not hasattr(bundle.deformation, "elastic_definition")
    hessian_values = bundle.deformation.hessian(
        bundle.deformation.zero_state()).to_coo()[2]
    assert np.all(np.isfinite(hessian_values))


def test_build_volume_scene_with_contact_and_attachment(tet_box_cfg_payload):
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        **tet_box_cfg_payload,
        "dynamic.timestep": 0.001,
        "contact": [
            {"model": "floor", "axis": "y", "height": 0.0, "stiffness": 100.0},
            {"model": "ipc", "dhat": 0.002, "kappa": 100.0,
             "obstacles": [{"mesh": str(ASSETS / "obj" / "bottom.obj")}]},
        ],
        "constraints.attachments": [
            {"vertices": {"region": {"axis": "y", "side": "max",
                                     "tolerance": 1e-3}}, "coeff": 1e4},
        ],
    })
    bundle = build_scene(cfg)
    assert len(bundle.contact_energies) == 2
    assert len(bundle.attachment_energies) == 1
    # the IPC energy is stateful and tracks obstacles
    assert len(bundle.stateful_contacts) == 1
    assert len(bundle.stateful_contacts[0].obstacles) == 1


def test_build_volume_scene_hermite(tet_box_cfg_payload):
    cfg = load_config(mesh_type="cubic", mode="dynamic", overrides={
        "mesh.volume": str(ASSETS / "veg" / "cubic" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "mesh.formulation": "cubic-tricubic-hermite",
        "dynamic.timestep": 0.001,
        "output.directory": "/tmp/unused",
    })
    bundle = build_scene(cfg)
    assert bundle.dofs_per_vertex == 24
    vec = bundle.initial_vector((0.0, -3.0, 0.0))
    assert vec[1] == -3.0           # first vertex position-y DOF
    assert vec[3] == 0.0            # derivative DOFs stay zero
    assert vec[25] == -3.0          # second vertex position-y DOF


def test_hermite_rejects_attachments():
    cfg = load_config(mesh_type="cubic", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "cubic" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "mesh.formulation": "cubic-tricubic-hermite",
        "constraints.attachments": [
            {"vertices": {"indices": [0]}, "coeff": 1.0},
        ],
        "output.directory": "/tmp/unused",
    })
    with pytest.raises(ConfigError, match="attachment"):
        build_scene(cfg)


def test_formulation_auto_detects_element_width(tet_box_cfg_payload):
    cfg = load_config(mesh_type="tet", mode="static", overrides=tet_box_cfg_payload)
    bundle = build_scene(cfg)
    assert type(bundle.formulation).__name__ == "TetLinear"


def test_formulation_mismatch_rejected():
    # cubic command pointed at a tet veg with formulation auto -> clear error
    cfg = load_config(mesh_type="cubic", mode="static", overrides={
        "mesh.volume": str(ASSETS / "veg" / "tet" / "box.veg"),
        "mesh.surface": str(ASSETS / "obj" / "box.obj"),
        "output.directory": "/tmp/unused",
    })
    with pytest.raises(ConfigError, match="element"):
        build_scene(cfg)


def _shell_overrides():
    return {
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"thickness": 0.001, "E_membrane": 1.0e6,
                     "nu_membrane": 0.4, "mass": {"density": 1000.0}},
        "output.directory": "/tmp/unused",
    }


def test_build_shell_scene_static():
    cfg = load_config(mesh_type="shell", mode="static", overrides={
        **_shell_overrides(),
        "loads.gravity": (0.0, 0.0, -9.81),
        "constraints.fixed": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-6}},
    })
    bundle = build_scene(cfg)
    assert bundle.dofs_per_vertex == 3
    assert bundle.surface_map is None
    n = bundle.num_dofs
    assert bundle.mass is None
    assert float(np.linalg.norm(bundle.gravity_force)) > 0.0
    verts = np.asarray(read_obj(str(ASSETS / "obj" / "shell.obj")).vertices, dtype=np.float64)
    y_max_count = int((np.abs(verts[:, 1] - verts[:, 1].max()) <= 1e-6).sum())
    assert y_max_count > 0
    assert bundle.fixed_dofs.size == y_max_count * 3
    # identity surface mapping
    u = np.zeros(n)
    u[2] = -0.5
    pos = bundle.surface_positions(u)
    assert pos[0, 2] == pytest.approx(bundle.surface_rest[0, 2] - 0.5)


def test_build_shell_scene_areal_density_and_contact():
    cfg = load_config(mesh_type="shell", mode="dynamic", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"mass": {"areal_density": 1.0}},
        "dynamic.timestep": 0.0005,
        "contact": [
            {"model": "sampled_penalty", "stiffness": 10.0,
             "friction_coeff": 0.3, "velocity_eps": 1e-4},
            {"model": "floor", "axis": "z", "height": -0.1, "stiffness": 5000.0},
        ],
        "output.directory": "/tmp/unused",
    })
    bundle = build_scene(cfg)
    assert len(bundle.contact_energies) == 2
    assert len(bundle.stateful_contacts) == 1  # frictional penalty only
    assert bundle.mass.shape == (bundle.num_dofs, bundle.num_dofs)
    ones = np.ones(bundle.num_dofs, dtype=np.float64)
    assert float(ones @ (bundle.mass @ ones)) > 0.0


def test_build_shell_scene_sampled_penalty_without_friction_is_not_step_dependent():
    cfg = load_config(mesh_type="shell", mode="dynamic", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"mass": {"areal_density": 1.0}},
        "dynamic.timestep": 0.0005,
        "contact": [{"model": "sampled_penalty", "stiffness": 10.0}],
        "output.directory": "/tmp/unused",
    })
    bundle = build_scene(cfg)
    assert len(bundle.contact_energies) == 1
    assert bundle.contact_energies[0].is_step_dependent is False
    assert len(bundle.stateful_contacts) == 0


def test_build_shell_scene_sampled_penalty_static_obstacle():
    cfg = load_config(mesh_type="shell", mode="dynamic", overrides={
        "mesh.surface": str(ASSETS / "obj" / "shell.obj"),
        "material": {"mass": {"areal_density": 1.0}},
        "dynamic.timestep": 0.0005,
        "contact": [{"model": "sampled_penalty", "stiffness": 10.0,
                     "obstacles": [{"mesh": str(ASSETS / "obj" / "bottom.obj")}]}],
        "output.directory": "/tmp/unused",
    })
    bundle = build_scene(cfg)
    assert len(bundle.contact_energies) == 1
    assert len(bundle.contact_energies[0].obstacles) == 1


# ---------------------------------------------------------------------------
# Task 7: MovingAttachment on SceneBundle
# ---------------------------------------------------------------------------


def test_volume_scene_moving_attachment_detected(tet_box_cfg_payload):
    """Attachment with nonzero movement → appears in moving_attachments, not just attachments."""
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        **tet_box_cfg_payload,
        "dynamic.timestep": 0.001,
        "constraints.attachments": [
            # plain (static) attachment at y-max
            {"vertices": {"region": {"axis": "y", "side": "max", "tolerance": 1e-3}},
             "coeff": 5e4},
            # moving attachment at y-min
            {"vertices": {"region": {"axis": "y", "side": "min", "tolerance": 1e-3}},
             "coeff": 5e4, "movement": [0.0, -1.0, 0.0]},
        ],
    })
    bundle = build_scene(cfg)
    # Both appear in attachment_energies
    assert len(bundle.attachment_energies) == 2
    # Only the moving one appears in moving_attachments
    assert len(bundle.moving_attachments) == 1
    ma = bundle.moving_attachments[0]
    np.testing.assert_array_equal(ma.velocity, [0.0, -1.0, 0.0])
    assert ma.num_vertices > 0


def test_volume_scene_plain_attachment_not_in_moving(tet_box_cfg_payload):
    """Plain attachment (no movement) does NOT appear in moving_attachments."""
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        **tet_box_cfg_payload,
        "dynamic.timestep": 0.001,
        "constraints.attachments": [
            {"vertices": {"region": {"axis": "y", "side": "max", "tolerance": 1e-3}},
             "coeff": 5e4},
        ],
    })
    bundle = build_scene(cfg)
    assert len(bundle.attachment_energies) == 1
    assert len(bundle.moving_attachments) == 0


def test_volume_scene_zero_movement_not_in_moving(tet_box_cfg_payload):
    """Attachment with all-zero movement vector does NOT appear in moving_attachments."""
    cfg = load_config(mesh_type="tet", mode="dynamic", overrides={
        **tet_box_cfg_payload,
        "dynamic.timestep": 0.001,
        "constraints.attachments": [
            {"vertices": {"region": {"axis": "y", "side": "max", "tolerance": 1e-3}},
             "coeff": 5e4, "movement": [0.0, 0.0, 0.0]},
        ],
    })
    bundle = build_scene(cfg)
    assert len(bundle.attachment_energies) == 1
    assert len(bundle.moving_attachments) == 0


def test_shell_scene_moving_attachment_detected():
    """Shell builder populates moving_attachments just like the volume builder."""
    cfg = load_config(mesh_type="shell", mode="dynamic", overrides={
        **_shell_overrides(),
        "dynamic.timestep": 0.0005,
        "constraints.attachments": [
            {"vertices": {"region": {"axis": "y", "side": "max", "tolerance": 1e-6}},
             "coeff": 1e4, "movement": [0.0, 0.0, -0.5]},
        ],
    })
    bundle = build_scene(cfg)
    assert len(bundle.attachment_energies) == 1
    assert len(bundle.moving_attachments) == 1
    np.testing.assert_array_equal(bundle.moving_attachments[0].velocity, [0.0, 0.0, -0.5])


def test_surface_attachment_volume_and_hermite():
    """surface_attachments build a QuadraticEnergy through the embedding and
    work for ALL formulations including tricubic Hermite (no DOF clamping)."""
    for formulation in ("auto", "cubic-tricubic-hermite"):
        cfg = load_config(mesh_type="cubic", mode="static", overrides={
            "mesh.volume": str(ASSETS / "veg" / "cubic" / "box.veg"),
            "mesh.surface": str(ASSETS / "obj" / "box.obj"),
            "mesh.formulation": formulation,
            "constraints.surface_attachments": [
                {"vertices": {"region": {"axis": "y", "side": "max",
                                         "tolerance": 1e-3}}, "coeff": 1e4},
            ],
            "output.directory": "/tmp/unused",
        })
        bundle = build_scene(cfg)
        assert len(bundle.attachment_energies) == 1
        e = bundle.attachment_energies[0]
        assert e.num_dofs == bundle.num_dofs
        # holding at rest: zero displacement has zero energy, nonzero has positive
        n = bundle.num_dofs
        assert e.value(np.zeros(n)) == pytest.approx(0.0, abs=1e-12)
        u = bundle.initial_vector((0.0, -0.1, 0.0))
        assert e.value(u) > 0.0


def test_surface_attachment_shell_identity():
    """For shells the embedding is identity; surface attachment still works."""
    cfg = load_config(mesh_type="shell", mode="static", overrides={
        **_shell_overrides(),
        "constraints.surface_attachments": [
            {"vertices": {"region": {"axis": "y", "side": "max",
                                     "tolerance": 1e-6}}, "coeff": 1e4},
        ],
    })
    bundle = build_scene(cfg)
    assert len(bundle.attachment_energies) == 1
    assert bundle.attachment_energies[0].num_dofs == bundle.num_dofs
