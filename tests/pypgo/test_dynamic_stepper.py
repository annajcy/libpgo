import numpy as np
import pytest

import pypgo as pgo
from pypgo.sim import DynamicSimulation, DynamicState, DynamicFrame


def _rest_state(n):
    return DynamicState(
        displacement=np.zeros(n),
        velocity=np.zeros(n),
        acceleration=np.zeros(n),
    )


def _spring(n, stiffness=10.0):
    # ½ k x'x  ->  QuadraticEnergy with A = k I, b = 0
    A = stiffness * np.eye(n, dtype=np.float64)
    return pgo.energy.QuadraticEnergy(A)


def test_import_dynamic_simulation():
    assert DynamicSimulation is not None
    assert DynamicState is not None
    assert DynamicFrame is not None


def test_one_step_implicit_euler_quadratic():
    n = 3
    sim = DynamicSimulation(
        mass=np.eye(n),
        state=_rest_state(n),
        timestep=0.05,
        energy=_spring(n),
        integrator="implicit_euler",
    )
    frame = sim.step(external_force=np.array([1.0, -2.0, 0.5]))
    assert frame.accepted
    assert frame.displacement.shape == (n,)
    assert len(frame.stage_results) == 1
    assert frame.frame_index == 0
    # state advanced
    assert sim.state.timestep_id == 1


def test_one_step_trbdf2_has_two_stages():
    n = 2
    sim = DynamicSimulation(
        mass=np.eye(n),
        state=_rest_state(n),
        timestep=0.05,
        energy=_spring(n),
        integrator="trbdf2",
    )
    frame = sim.step(external_force=np.array([1.0, -1.0]))
    assert frame.accepted
    assert len(frame.stage_results) == 2


def test_unknown_integrator_raises():
    with pytest.raises(ValueError):
        DynamicSimulation(
            mass=np.eye(2),
            state=_rest_state(2),
            timestep=0.05,
            integrator="rk4",
        )


def test_fixed_dofs_remain_fixed():
    n = 3
    state = DynamicState(
        displacement=np.array([0.0, 0.7, 0.0]),
        velocity=np.zeros(n),
        acceleration=np.zeros(n),
    )
    sim = DynamicSimulation(
        mass=np.eye(n),
        state=state,
        timestep=0.05,
        energy=_spring(n),
        integrator="implicit_euler",
        fixed_dofs=[1],
    )
    frame = sim.step(external_force=np.ones(n))
    assert frame.accepted
    assert frame.displacement[1] == pytest.approx(0.7, abs=1e-9)


def test_run_zero_returns_empty_and_does_not_mutate():
    n = 2
    sim = DynamicSimulation(
        mass=np.eye(n),
        state=_rest_state(n),
        timestep=0.05,
        energy=_spring(n),
    )
    frames = sim.run(0, external_force=np.ones(n))
    assert frames == []
    assert sim.state.timestep_id == 0
    np.testing.assert_array_equal(sim.state.displacement, np.zeros(n))


def test_run_negative_raises():
    sim = DynamicSimulation(mass=np.eye(1), state=_rest_state(1), timestep=0.05)
    with pytest.raises(ValueError):
        sim.run(-1)


def test_returned_arrays_do_not_alias_input():
    n = 2
    force = np.array([1.0, 2.0])
    sim = DynamicSimulation(
        mass=np.eye(n),
        state=_rest_state(n),
        timestep=0.05,
        energy=_spring(n),
    )
    frame = sim.step(external_force=force)
    # mutating the returned array must not affect the simulation's state
    before = sim.state.displacement.copy()
    frame.displacement[:] = 999.0
    np.testing.assert_array_equal(sim.state.displacement, before)


def test_free_fall_matches_implicit_euler_recurrence():
    n = 1
    m, h, f = 2.0, 0.05, -19.6
    sim = DynamicSimulation(
        mass=np.array([[m]]),
        state=_rest_state(n),
        timestep=h,
        integrator="implicit_euler",
    )
    u = v = 0.0
    for _ in range(10):
        frame = sim.step(external_force=np.array([f]))
        assert frame.accepted
        v += h * f / m
        u += h * v
        assert frame.velocity[0] == pytest.approx(v, abs=1e-3)
        assert frame.displacement[0] == pytest.approx(u, abs=1e-3)
