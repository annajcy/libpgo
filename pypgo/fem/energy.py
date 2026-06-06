"""Deformation energy factory — energy construction from state + formulation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo.energy import DeformationEnergy, PlasticMaterialEnergy
from pypgo.fem.state import DeformationModelState


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
        state._core,
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
        state._core,
        deformation_energy._core,
        u,
    )
    return PlasticMaterialEnergy(
        handle,
        state=state,
        deformation_energy=deformation_energy,
        fixed_displacement=u,
    )
