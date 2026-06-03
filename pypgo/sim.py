"""Solver-facing simulation mesh, shell material, and dynamic-stepping helpers."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

import pypgo._core as _core
from pypgo import solver as _solver
from pypgo.mesh import TriMeshData, read_obj, write_obj


@dataclass(frozen=True)
class KoiterStVKShellMaterial:
    name: str = "shell"
    thickness: float = 0.001
    E_membrane: float = 1e6
    nu_membrane: float = 0.4


ShellMaterialLike = KoiterStVKShellMaterial


class SimulationMesh:
    """Solver-ready simulation mesh created by explicit factory methods."""

    def __init__(self, core_obj):
        if not isinstance(core_obj, _core.PySimulationMesh):
            raise TypeError(f"core_obj must be PySimulationMesh, got {type(core_obj).__name__}")
        self._core_obj = core_obj

    @classmethod
    def create_volumetric(cls, volume_mesh) -> "SimulationMesh":
        from pypgo.mesh.veg import VolumeMesh

        if not isinstance(volume_mesh, VolumeMesh):
            raise TypeError(f"volume_mesh must be a VolumeMesh, got {type(volume_mesh).__name__}")
        return cls(_core.create_simulation_mesh_from_volume(volume_mesh._core_obj))

    @classmethod
    def create_shell(cls, surface: TriMeshData, material: ShellMaterialLike) -> "SimulationMesh":
        if not isinstance(surface, TriMeshData):
            raise TypeError(f"surface must be a TriMeshData, got {type(surface).__name__}")
        if not isinstance(material, KoiterStVKShellMaterial):
            raise TypeError(f"material must be a KoiterStVKShellMaterial, got {type(material).__name__}")
        return cls(_core.create_simulation_mesh_from_shell(
            surface._core_obj,
            float(material.thickness),
            float(material.E_membrane),
            float(material.nu_membrane),
        ))

    @property
    def mesh_type(self) -> str:
        return self._core_obj.mesh_type()

    @property
    def num_vertices(self) -> int:
        return self._core_obj.num_vertices()

    @property
    def num_elements(self) -> int:
        return self._core_obj.num_elements()

    @property
    def num_element_vertices(self) -> int:
        return self._core_obj.num_element_vertices()


def write_shell(path, surface: TriMeshData, material: ShellMaterialLike) -> None:
    if not isinstance(surface, TriMeshData):
        raise TypeError(f"surface must be a TriMeshData, got {type(surface).__name__}")
    if not isinstance(material, KoiterStVKShellMaterial):
        raise TypeError(f"material must be a KoiterStVKShellMaterial, got {type(material).__name__}")

    shell_path = Path(path)
    obj_path = shell_path.with_suffix("").with_suffix(".obj")
    write_obj(str(obj_path), surface)
    payload = {
        "mesh_obj": obj_path.name,
        "material": {
            "kind": "KoiterStVKShellMaterial",
            **asdict(material),
        },
    }
    shell_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def read_shell(path):
    shell_path = Path(path)
    payload = json.loads(shell_path.read_text())
    surface = read_obj(str(shell_path.parent / payload["mesh_obj"]))
    material_payload = dict(payload["material"])
    kind = material_payload.pop("kind")
    if kind != "KoiterStVKShellMaterial":
        raise ValueError(f"unsupported shell material kind: {kind}")
    return surface, KoiterStVKShellMaterial(**material_payload)


# ─────────────────────────────────────────────────────────────────────────
# Dynamic time-stepping API
# ─────────────────────────────────────────────────────────────────────────

_INTEGRATORS = ("implicit_euler", "trbdf2")


@dataclass(frozen=True)
class DynamicState:
    """Per-frame kinematic state carried across dynamic time steps."""

    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    timestep_id: int = 0
    time: float = 0.0


@dataclass(frozen=True)
class DynamicFrame:
    """Result of a single ``DynamicSimulation.step``.

    ``solver_result`` is the final stage's solve; ``stage_results`` holds one
    entry for implicit Euler and two for TRBDF2 (``gamma`` < 1). Per-stage
    solution vectors are not captured, so stage results carry an empty ``x``.
    """

    frame_index: int
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    solver_result: _solver.SolverResult
    stage_results: Sequence[_solver.SolverResult]
    accepted: bool


def _mass_to_coo(mass):
    """Normalize a mass matrix to (num_dofs, rows, cols, values) COO arrays.

    Accepts a ``pypgo.sparse.SparseMatrix``, a SciPy sparse matrix, or a dense
    array-like.
    """
    if hasattr(mass, "to_coo"):  # pypgo.sparse.SparseMatrix
        rows, cols, values = mass.to_coo()
        n = mass.shape[0]
    elif hasattr(mass, "tocoo"):  # scipy.sparse
        coo = mass.tocoo()
        rows, cols, values = coo.row, coo.col, coo.data
        n = mass.shape[0]
    else:
        dense = np.asarray(mass, dtype=np.float64)
        if dense.ndim != 2 or dense.shape[0] != dense.shape[1]:
            raise ValueError(f"mass must be a square matrix, got shape {dense.shape}")
        rows, cols = np.nonzero(dense)
        values = dense[rows, cols]
        n = dense.shape[0]
    return (
        int(n),
        [int(r) for r in np.asarray(rows).ravel()],
        [int(c) for c in np.asarray(cols).ravel()],
        [float(v) for v in np.asarray(values).ravel()],
    )


def _vec(name, arr, n):
    out = np.ascontiguousarray(np.asarray(arr, dtype=np.float64).ravel())
    if out.size != n:
        raise ValueError(f"{name} must have length {n}, got {out.size}")
    return out


def _solver_result(data: dict, x: np.ndarray) -> _solver.SolverResult:
    diagnostics = _solver.SolveDiagnostics(**data["diagnostics"])
    return _solver.SolverResult(
        x=x,
        status=_solver.SolveStatus(int(data["status"])),
        converged=bool(data["converged"]),
        iterations=int(data["iterations"]),
        raw_status_code=int(data["raw_status_code"]),
        final_objective=None,
        final_gradient_norm=data["final_gradient_norm"],
        final_gradient_max_norm=data["final_gradient_max_norm"],
        diagnostics=diagnostics,
    )


class DynamicSimulation:
    """Drive an implicit dynamic simulation from in-memory mass / energy / state.

    The integrator (``"implicit_euler"`` or ``"trbdf2"``), mass, persistent
    energy, damping, fixed DOFs, and solver settings are fixed at construction
    (the fixed-DOF set is immutable; rebuild the object to change it). Each
    :meth:`step` advances one time step under an external force.
    """

    def __init__(
        self,
        *,
        mass,
        state: DynamicState,
        timestep: float,
        energy=None,
        integrator: str = "implicit_euler",
        damping: tuple[float, float] = (0.0, 0.0),
        solver: _solver.NewtonOptions | None = None,
        fixed_dofs: Sequence[int] | None = None,
        trbdf2_gamma: float = 0.5,
    ) -> None:
        if integrator not in _INTEGRATORS:
            raise ValueError(f"integrator must be one of {_INTEGRATORS}, got {integrator!r}")
        if not timestep > 0.0:
            raise ValueError("timestep must be positive")

        n, mass_rows, mass_cols, mass_vals = _mass_to_coo(mass)

        handle = None
        if energy is not None:
            if not hasattr(energy, "_handle"):
                raise TypeError("energy must be a pypgo.energy PotentialEnergy-compatible object")
            handle = energy._handle

        mass_damping, stiffness_damping = (float(damping[0]), float(damping[1]))
        opts = solver if solver is not None else _solver.NewtonOptions()

        self._n = n
        self._frame_index = 0
        self._sim = _core.PyDynamicSimulation(
            num_dofs=n,
            mass_rows=mass_rows,
            mass_cols=mass_cols,
            mass_vals=mass_vals,
            energy=handle,
            mass_damping=mass_damping,
            stiffness_damping=stiffness_damping,
            displacement=_vec("displacement", state.displacement, n),
            velocity=_vec("velocity", state.velocity, n),
            acceleration=_vec("acceleration", state.acceleration, n),
            timestep=float(timestep),
            integrator=integrator,
            fixed_dofs=[int(d) for d in (fixed_dofs or [])],
            max_iter=int(opts.max_iter),
            tol=float(opts.tol),
            verbose=int(opts.verbose),
            gamma=float(trbdf2_gamma),
        )

    @property
    def num_dofs(self) -> int:
        return self._n

    @property
    def state(self) -> DynamicState:
        return DynamicState(
            displacement=np.asarray(self._sim.displacement, dtype=np.float64),
            velocity=np.asarray(self._sim.velocity, dtype=np.float64),
            acceleration=np.asarray(self._sim.acceleration, dtype=np.float64),
            timestep_id=int(self._sim.timestep_id),
            time=float(self._sim.time),
        )

    def step(
        self,
        *,
        external_force: np.ndarray | Sequence[float] | None = None,
        fixed_values: np.ndarray | Sequence[float] | None = None,
    ) -> DynamicFrame:
        force = (
            np.zeros(self._n, dtype=np.float64)
            if external_force is None
            else _vec("external_force", external_force, self._n)
        )
        has_fixed = fixed_values is not None
        fixed_arr = (
            np.empty(0, dtype=np.float64)
            if fixed_values is None
            else np.ascontiguousarray(np.asarray(fixed_values, dtype=np.float64).ravel())
        )

        data = self._sim.step(force, fixed_arr, has_fixed)

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
