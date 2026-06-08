"""Deformation energy factory — energy construction from state + formulation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector
from pypgo.energy import PotentialEnergy
from pypgo.fem.state import DeformationModelState
from pypgo.sparse import SparseMatrix


# ---------------------------------------------------------------------------
# DeformationEnergy — FEM deformation energy
# ---------------------------------------------------------------------------


class DeformationEnergy(PotentialEnergy):
    """Deformation energy for FEM simulations (tet, cubic, shell).

    Created by ``pypgo.fem.deformation_energy()``, not directly by users.
    This is a **displacement**-kind energy: ``state_kind == "displacement"``.

    Parameters
    ----------
    core : PyDeformationEnergy
        C++ deformation energy wrapper (from ``_core._create_deformation_energy``).

    Properties
    ----------
    rest_position : ndarray (num_vertices, 3) float64
        Rest (undeformed) positions.
    """

    def __init__(self, core):
        if not isinstance(core, _core.PyDeformationEnergy):
            raise TypeError(
                f"core must be a PyDeformationEnergy, got {type(core).__name__}"
            )
        object.__setattr__(self, "_handle", core)
        super().__init__(core)

    @property
    def rest_position(self) -> np.ndarray:
        return np.asarray(self._handle.rest_position(), dtype=np.float64)

    @property
    def num_vertices(self) -> int:
        return self._handle.num_vertices

    @property
    def num_elastic_params(self) -> int:
        """Per-element elastic parameter count."""
        return self._handle.num_elastic_params

    @property
    def num_plastic_params(self) -> int:
        """Per-element plastic parameter count."""
        return self._handle.num_plastic_params

    @property
    def num_elastic_dofs(self) -> int:
        """Total elastic parameter DOFs across all elements."""
        return self._handle.num_elastic_dofs

    @property
    def num_plastic_dofs(self) -> int:
        """Total plastic parameter DOFs across all elements."""
        return self._handle.num_plastic_dofs

    def plastic_gradient(self, displacement: np.ndarray) -> np.ndarray:
        u = float_vector("displacement", displacement)
        return np.asarray(self._handle.plastic_gradient(u), dtype=np.float64)

    def plastic_hessian(self, displacement: np.ndarray):
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.plastic_hessian(u))

    def plastic_jacobian(self, displacement: np.ndarray):
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.plastic_jacobian(u))

    def __repr__(self) -> str:
        return f"DeformationEnergy({self.num_dofs} DOFs, state_kind='{self.state_kind}')"


class PlasticMaterialEnergy(PotentialEnergy):
    """Material energy with the plastic field as the optimization variable.

    Created by ``pypgo.fem.plastic_material_energy()``. The displacement is fixed;
    the input state vector is the plastic field's global DOF vector.
    """

    def __init__(self, handle, *, state, deformation_energy, fixed_displacement):
        object.__setattr__(self, "_state", state)
        object.__setattr__(self, "_deformation_energy", deformation_energy)
        object.__setattr__(
            self,
            "_fixed_displacement",
            np.asarray(fixed_displacement, dtype=np.float64).copy(),
        )
        super().__init__(handle)

    @property
    def state(self):
        return self._state

    @property
    def deformation_energy(self):
        return self._deformation_energy

    @property
    def fixed_displacement(self) -> np.ndarray:
        return self._fixed_displacement.copy()

    def __repr__(self) -> str:
        return f"PlasticMaterialEnergy({self.num_dofs} DOFs, state_kind='{self.state_kind}')"


# ---------------------------------------------------------------------------
# DeformationOptions
# ---------------------------------------------------------------------------


@dataclass
class DeformationOptions:
    enforce_spd: bool = True
    enable_material_max_step: bool = True


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


def _resolve_formulation(state: DeformationModelState, formulation):
    from pypgo.fem.formulations import Formulation

    if formulation is None:
        raise ValueError(
            "formulation is required. Pass TetP1(), LinearCubic(), TricubicHermite(), or KoiterShell()."
        )
    if not isinstance(formulation, Formulation):
        raise TypeError(
            f"formulation must be a Formulation, got {type(formulation).__name__}"
        )
    return formulation


# ---------------------------------------------------------------------------
# Energy factories
# ---------------------------------------------------------------------------


def deformation_energy(
    state,
    formulation=None,
    *,
    options=None,
) -> DeformationEnergy:
    if not isinstance(state, DeformationModelState):
        raise TypeError(
            f"state must be a DeformationModelState, got {type(state).__name__}"
        )
    formulation = _resolve_formulation(state, formulation)

    if options is None:
        options = DeformationOptions()
    if not isinstance(options, DeformationOptions):
        raise TypeError(f"options must be DeformationOptions, got {type(options).__name__}")

    core = _core._create_deformation_energy(
        state._handle,
        formulation.name,
        bool(options.enforce_spd),
        bool(options.enable_material_max_step),
    )
    return DeformationEnergy(core)


def plastic_material_energy(
    state,
    deformation_energy,
    *,
    fixed_displacement,
) -> PlasticMaterialEnergy:
    """Create a material energy whose optimization variable is the plastic field."""
    if not isinstance(state, DeformationModelState):
        raise TypeError(
            f"state must be a DeformationModelState, got {type(state).__name__}"
        )
    if not isinstance(deformation_energy, DeformationEnergy):
        raise TypeError(
            f"deformation_energy must be a DeformationEnergy, got {type(deformation_energy).__name__}"
        )

    u = np.asarray(fixed_displacement, dtype=np.float64, order="C")
    if u.ndim != 1:
        raise ValueError(f"fixed_displacement must be 1-D, got shape {u.shape}")
    if u.size != deformation_energy.num_dofs:
        raise ValueError(
            f"fixed_displacement size must be {deformation_energy.num_dofs}, got {u.size}"
        )

    handle = _core._create_plastic_material_energy(
        state._handle,
        deformation_energy._handle,
        u,
    )
    return PlasticMaterialEnergy(
        handle,
        state=state,
        deformation_energy=deformation_energy,
        fixed_displacement=u,
    )
