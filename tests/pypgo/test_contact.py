"""Tests for pypgo.contact construction facades."""

import numpy as np
import pytest

import pypgo
import pypgo.contact as contact
import pypgo.energy as energy


def _triangle_surface():
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    triangles = np.array([[0, 1, 2]], dtype=np.int64)
    return vertices, triangles


def test_contact_module_is_lazy_loaded():
    assert pypgo.contact is contact


def test_floor_energy_is_potential_energy_and_evaluates_penalty():
    vertices, _ = _triangle_surface()
    surface = contact.ContactSurface.identity(vertices)
    floor = contact.FloorEnergy(
        surface,
        axis="z",
        side="keep_above",
        height=0.0,
        stiffness=10.0,
    )

    assert isinstance(floor, energy.PotentialEnergy)
    assert floor.state_kind == "displacement"
    assert floor.num_dofs == 9
    assert floor.axis == "z"
    assert floor.side == "keep_above"
    assert floor.height == pytest.approx(0.0)
    assert floor.stiffness == pytest.approx(10.0)

    x = np.zeros(9, dtype=np.float64)
    x[2] = -0.2
    assert floor.value(x) == pytest.approx(0.5 * 10.0 * 0.2 * 0.2)
    grad = floor.gradient(x)
    assert grad.shape == (9,)
    assert grad[2] == pytest.approx(-2.0)


def test_floor_parameters_and_set_height_update_energy():
    vertices, _ = _triangle_surface()
    surface = contact.ContactSurface.identity(vertices)
    params = contact.FloorParameters(axis="z", side="keep_above", height=-0.1, stiffness=10.0)
    floor = contact.FloorEnergy(surface, parameters=params)

    x = np.zeros(9, dtype=np.float64)
    assert floor.height == pytest.approx(-0.1)
    assert floor.parameters == params
    assert floor.value(x) == pytest.approx(0.0)

    floor.set_height(0.2)
    assert floor.height == pytest.approx(0.2)
    assert floor.parameters.height == pytest.approx(0.2)
    assert floor.value(x) == pytest.approx(3 * 0.5 * 10.0 * 0.2 * 0.2)

    with pytest.raises(ValueError, match="parameters"):
        contact.FloorEnergy(surface, parameters=params, height=0.0)


def test_contact_vertex_embedding_validates_shape_and_arity():
    embedding = contact.ContactVertexEmbedding(
        indices=np.array([[0, 1, 2]], dtype=np.int64),
        weights=np.array([[0.2, 0.3, 0.5]], dtype=np.float64),
    )
    assert embedding.embedding_arity == 3
    assert embedding.indices.shape == (1, 3)
    assert embedding.weights.shape == (1, 3)

    with pytest.raises(ValueError, match="same shape"):
        contact.ContactVertexEmbedding(
            indices=np.array([[0, 1]], dtype=np.int64),
            weights=np.array([[0.5, 0.25, 0.25]], dtype=np.float64),
        )
    with pytest.raises(ValueError, match="sum"):
        contact.ContactVertexEmbedding(
            indices=np.array([[0, 1, 2]], dtype=np.int64),
            weights=np.array([[0.5, 0.5, 0.5]], dtype=np.float64),
        )


def test_embedded_contact_surface_uses_simulation_dof_count():
    vertices, _ = _triangle_surface()
    rows = list(range(9))
    cols = list(range(9))
    values = [1.0] * 9
    surface_map = pypgo.sparse.SparseMatrix.from_coo((9, 12), rows, cols, values)

    surface = contact.ContactSurface.embedded(vertices, surface_map)
    assert surface.num_surface_dofs == 9
    assert surface.num_simulation_dofs == 12

    floor = contact.FloorEnergy(surface, axis="z", side="keep_above", height=0.0, stiffness=10.0)
    x = np.zeros(12, dtype=np.float64)
    x[2] = -0.2
    assert floor.num_dofs == 12
    assert floor.value(x) == pytest.approx(0.5 * 10.0 * 0.2 * 0.2)

    fake_embedding = type(
        "FakeSurfaceEmbedding",
        (),
        {"rest_surface": type("RestSurface", (), {"vertices": vertices})(), "interpolation_matrix": surface_map},
    )()
    from_embedding = contact.ContactSurface.from_surface_embedding(fake_embedding)
    assert from_embedding.num_simulation_dofs == 12


def test_sampled_penalty_energy_lifecycle_and_metadata():
    vertices, triangles = _triangle_surface()
    surface = contact.ContactSurface.identity(vertices)
    params = contact.SampledPenaltyParameters(stiffness=3.0, samples=1)
    penalty = contact.SampledPenaltyEnergy(surface, triangles, params=params)

    assert isinstance(penalty, energy.PotentialEnergy)
    assert penalty.state_kind == "displacement"
    assert penalty.params == params
    assert penalty.is_step_dependent is False
    assert not hasattr(penalty, "refresh_active_set")
    assert not hasattr(penalty, "clear_active_set")

    x = np.zeros(9, dtype=np.float64)
    assert penalty.value(x) == pytest.approx(0.0)
    assert penalty.gradient(x).shape == (9,)
    assert penalty.hessian(x).shape == (9, 9)


def test_sampled_penalty_accepts_embedded_surface_map():
    vertices, triangles = _triangle_surface()

    rows = list(range(9)) + [0]
    cols = list(range(9)) + [9]
    values = [1.0] * 9 + [0.25]
    surface_map = pypgo.sparse.SparseMatrix.from_coo((9, 10), rows, cols, values)

    surface = contact.ContactSurface.embedded(vertices, surface_map)
    params = contact.SampledPenaltyParameters(stiffness=3.0, samples=1)
    penalty = contact.SampledPenaltyEnergy(surface, triangles, params=params)

    assert penalty.num_dofs == 10
    assert penalty.value(np.zeros(10, dtype=np.float64)) == pytest.approx(0.0)


def test_ipc_energy_lifecycle_and_metadata():
    vertices, triangles = _triangle_surface()
    surface = contact.ContactSurface.identity(vertices)
    params = contact.IPCParameters(dhat=1e-3, kappa=100.0)
    ipc = contact.IPCEnergy(surface, triangles, params=params)

    assert isinstance(ipc, energy.PotentialEnergy)
    assert ipc.state_kind == "displacement"
    assert ipc.params == params
    assert ipc.is_step_dependent is False
    assert not hasattr(ipc, "refresh_active_set")
    assert not hasattr(ipc, "clear_active_set")

    x = np.zeros(9, dtype=np.float64)
    ipc.begin_step(time=0.0, timestep=0.1, previous_x=x)
    assert ipc.value(x) == pytest.approx(0.0)
    assert ipc.gradient(x).shape == (9,)
    assert ipc.hessian(x).shape == (9, 9)
    assert not hasattr(ipc, "set_obstacle_time")


def test_ipc_energy_accepts_obstacle_specs_and_moving_time_update():
    vertices, triangles = _triangle_surface()
    vertices = vertices.copy()
    vertices[:, 2] += 0.02
    surface = contact.ContactSurface.identity(vertices)

    static_obstacle_vertices, obstacle_triangles = _triangle_surface()
    moving_obstacle_vertices = static_obstacle_vertices.copy()
    moving_obstacle_vertices[:, 2] -= 0.2

    obstacles = [
        contact.ObstacleSpec.static(static_obstacle_vertices, obstacle_triangles),
        contact.ObstacleSpec.linear_velocity(
            moving_obstacle_vertices,
            obstacle_triangles,
            velocity=np.array([0.0, 0.0, 0.1], dtype=np.float64),
            reference_time=0.0,
        ),
    ]
    ipc = contact.IPCEnergy(
        surface,
        triangles,
        params=contact.IPCParameters(dhat=0.1, dhat_external=0.1, kappa=1.0),
        obstacles=obstacles,
    )

    assert ipc.obstacles == tuple(obstacles)
    x = np.zeros(9, dtype=np.float64)
    ipc.begin_step(time=0.0, timestep=0.1, previous_x=x)
    assert np.isfinite(ipc.value(x))
    ipc.set_moving_obstacle_time(0.25)
    assert np.isfinite(ipc.value(x))


def test_contact_public_surface_matches_plan():
    expected = {
        "ContactSurface",
        "ContactVertexEmbedding",
        "FloorEnergy",
        "FloorParameters",
        "IPCEnergy",
        "IPCParameters",
        "SampledPenaltyEnergy",
        "SampledPenaltyParameters",
        "FrictionalSampledPenaltyEnergy",
        "FrictionParameters",
        "ObstacleSpec",
    }
    public = {name for name in contact.__all__ if not name.startswith("_")}
    assert public == expected

    forbidden = {
        "MappedSurfacePotentialEnergy",
        "FloorContactEnergy",
        "EmbeddedSurfaceIPCPotentialEnergy",
        "EmbeddedDofMap",
        "ContactSurfaceAdapter",
        "IPCContactEnergy",
        "SampledPenaltyContactEnergy",
        "FrictionalSampledPenaltyContactEnergy",
        "StatefulContactEnergy",
        "StepDependentEnergy",
        "FloorPenaltyParameters",
        "SurfaceIPCCore",
        "ObstacleSurfaceView",
        "markObstacleStatic",
        "setObstacleTime",
        "set_obstacle_time",
    }
    assert forbidden.isdisjoint(set(dir(contact)))

def test_frictional_sampled_penalty_requires_previous_state():
    vertices, triangles = _triangle_surface()
    surface = contact.ContactSurface.identity(vertices)
    frictional = contact.FrictionalSampledPenaltyEnergy(
        surface,
        triangles,
        params=contact.SampledPenaltyParameters(stiffness=3.0, samples=1),
        friction=contact.FrictionParameters(friction_coeff=0.5, velocity_eps=1e-4),
    )

    assert frictional.is_step_dependent is True
    with pytest.raises(ValueError, match="previous_x"):
        frictional.begin_step(time=0.0, timestep=0.1)
    with pytest.raises(ValueError, match="positive"):
        frictional.begin_step(time=0.0, timestep=0.0, previous_x=np.zeros(9))

    frictional.begin_step(time=0.0, timestep=0.1, previous_x=np.zeros(9))


def test_contact_surface_and_energy_handles_are_concrete_peers():
    import pypgo._core as _core

    surface = contact.ContactSurface.identity(
        np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float64)
    )
    triangles = np.array([[0, 1, 2]], dtype=np.int64)
    e = contact.SampledPenaltyEnergy(surface, triangles)

    assert isinstance(surface._handle, _core.PyContactSurface)
    assert isinstance(e._handle, _core.PySampledPenaltyContactEnergy)
    assert isinstance(e._handle, _core.PyStatefulContactEnergy)
    assert isinstance(e._handle, _core.PyPotentialEnergy)
    # The stateful peer is the single source of truth: no dual _contact_core.
    assert not hasattr(e, "_contact_core")

    ipc = contact.IPCEnergy(surface, triangles)
    assert isinstance(ipc._handle, _core.PyIPCContactEnergy)
    assert not hasattr(ipc, "_contact_core")

    frictional = contact.FrictionalSampledPenaltyEnergy(surface, triangles)
    assert isinstance(frictional._handle, _core.PyFrictionalSampledPenaltyContactEnergy)
    assert not hasattr(frictional, "_contact_core")
