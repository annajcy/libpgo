"""Tests for pypgo.tools.sim._scene."""

from pathlib import Path

import numpy as np
import pytest

from pypgo.tools.sim._config import (
    ConfigError, RegionSelector, VertexSelector, load_config,
)
from pypgo.tools.sim._scene import build_scene, resolve_vertex_selector

ASSETS = Path(__file__).resolve().parents[2] / "examples" / "assets"


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


def test_resolve_selector_empty_region():
    sel = VertexSelector(indices=())
    with pytest.raises(ConfigError, match="no vertices"):
        resolve_vertex_selector(sel, np.zeros((3, 3)))


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
    assert bundle.mass.shape == (n, n)
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
    assert len(bundle.ipc_contacts) == 1
    assert len(bundle.ipc_contacts[0].obstacles) == 1


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
