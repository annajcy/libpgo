"""Dynamic simulation engine — implicit time-stepping with energy, mass, and damping."""

from __future__ import annotations

from typing import Sequence

import numpy as np

import pypgo._core as _core
from pypgo import solver as _solver
from pypgo._utils import sized_vector
from pypgo.energy import PotentialEnergy
from pypgo.sparse import sparse_to_coo_lists
from pypgo.sim.state import DynamicFrame, DynamicState
from pypgo.sim.stepper import BackwardEulerDynamicStepper, DynamicStepper, TRBDF2DynamicStepper


_INTEGRATORS = ("implicit_euler", "trbdf2")


def _solver_result(data: dict, x: np.ndarray) -> _solver.SolverResult:
    return _solver.SolverResult(
        x=x,
        status=_solver.SolveStatus(int(data["status"])),
        converged=bool(data["converged"]),
        iterations=int(data["iterations"]),
        raw_status_code=int(data["raw_status_code"]),
        final_objective=None,
        final_gradient_norm=data["final_gradient_norm"],
        final_gradient_max_norm=data["final_gradient_max_norm"],
        diagnostics=_solver.SolveDiagnostics.from_dict(data["diagnostics"]),
    )


def _normalize_dynamic_stepper(integrator) -> DynamicStepper:
    if integrator is None:
        return BackwardEulerDynamicStepper()
    if isinstance(integrator, DynamicStepper):
        return integrator
    if isinstance(integrator, str):
        if integrator == "implicit_euler":
            return BackwardEulerDynamicStepper()
        if integrator == "trbdf2":
            return TRBDF2DynamicStepper()
        raise ValueError(f"integrator must be one of {_INTEGRATORS}, got {integrator!r}")
    raise TypeError("integrator must be a pypgo.sim.DynamicStepper")


class DynamicSimulation:
    """Drive an implicit dynamic simulation from in-memory mass / energy / state.

    The dynamic stepper, mass, persistent energy, damping, and fixed DOFs are
    fixed at construction. Solver settings live on the optimizer object passed
    to :meth:`step`.
    """

    def __init__(
        self,
        *,
        mass,
        state: DynamicState,
        timestep: float,
        energy=None,
        integrator: DynamicStepper | str | None = None,
        damping: tuple[float, float] = (0.0, 0.0),
        fixed_dofs: Sequence[int] | None = None,
    ) -> None:
        if not timestep > 0.0:
            raise ValueError("timestep must be positive")

        stepper = _normalize_dynamic_stepper(integrator)
        n, mass_rows, mass_cols, mass_vals = sparse_to_coo_lists(mass)

        handle = None
        if energy is not None:
            if not isinstance(energy, PotentialEnergy):
                raise TypeError("energy must be a pypgo.energy.PotentialEnergy")
            handle = energy._handle

        mass_damping, stiffness_damping = (float(damping[0]), float(damping[1]))

        self._n = n
        self._frame_index = int(state.timestep_id)
        self._handle = _core.PyDynamicSimulation(
            num_dofs=n,
            mass_rows=mass_rows,
            mass_cols=mass_cols,
            mass_vals=mass_vals,
            energy=handle,
            mass_damping=mass_damping,
            stiffness_damping=stiffness_damping,
            displacement=sized_vector("displacement", state.displacement, n),
            velocity=sized_vector("velocity", state.velocity, n),
            acceleration=sized_vector("acceleration", state.acceleration, n),
            timestep_id=int(state.timestep_id),
            time=float(state.time),
            timestep=float(timestep),
            integrator=stepper._handle,
            fixed_dofs=[int(d) for d in (fixed_dofs or [])],
        )

    @property
    def num_dofs(self) -> int:
        return self._n

    @property
    def state(self) -> DynamicState:
        return DynamicState(
            displacement=np.asarray(self._handle.displacement, dtype=np.float64),
            velocity=np.asarray(self._handle.velocity, dtype=np.float64),
            acceleration=np.asarray(self._handle.acceleration, dtype=np.float64),
            timestep_id=int(self._handle.timestep_id),
            time=float(self._handle.time),
        )

    def step(
        self,
        *,
        external_force: np.ndarray | Sequence[float] | None = None,
        fixed_values: np.ndarray | Sequence[float] | None = None,
        optimizer: _solver.Optimizer | None = None,
    ) -> DynamicFrame:
        optimizer = (
            optimizer if optimizer is not None
            else _solver.NewtonOptimizer(
                termination=_solver.AbsoluteTermination(abs_tolerance=1e-6))
        )
        if not isinstance(optimizer, _solver.Optimizer):
            raise TypeError("optimizer must be a pypgo.solver.Optimizer")

        force = (
            np.zeros(self._n, dtype=np.float64)
            if external_force is None
            else sized_vector("external_force", external_force, self._n)
        )
        has_fixed = fixed_values is not None
        fixed_arr = (
            np.empty(0, dtype=np.float64)
            if fixed_values is None
            else np.ascontiguousarray(np.asarray(fixed_values, dtype=np.float64).ravel())
        )

        data = self._handle.step(
            force,
            fixed_arr,
            has_fixed,
            optimizer._handle,
        )

        displacement = np.asarray(data["displacement"], dtype=np.float64)
        velocity = np.asarray(data["velocity"], dtype=np.float64)
        acceleration = np.asarray(data["acceleration"], dtype=np.float64)

        empty = np.empty(0, dtype=np.float64)
        solver_result = _solver_result(data["solver"], displacement)
        stage_results = [_solver_result(s, empty) for s in data["stage_results"]]

        frame = DynamicFrame(
            frame_index=self._frame_index,
            displacement=displacement,
            velocity=velocity,
            acceleration=acceleration,
            solver_result=solver_result,
            stage_results=stage_results,
            accepted=bool(data["accepted"]),
        )
        self._frame_index += 1
        return frame

    def run(self, num_steps: int, **step_kwargs) -> list[DynamicFrame]:
        if num_steps < 0:
            raise ValueError("num_steps must be non-negative")
        return [self.step(**step_kwargs) for _ in range(num_steps)]
