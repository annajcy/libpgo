"""FEM deformation and material energies."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector
from pypgo.energy import PotentialEnergy
from pypgo.fem.fields import MaterialAssignment, OptimizableParameters
from pypgo.sparse import SparseMatrix


# ---------------------------------------------------------------------------
# DeformationEnergy — FEM deformation energy
# ---------------------------------------------------------------------------


class DeformationEnergy(PotentialEnergy):
    """Deformation energy for FEM simulations (tet, cubic, shell).

    This is a **displacement**-kind energy: ``state_kind == "displacement"``.

    Parameters
    ----------
    assignment : MaterialAssignment
        Complete material binding for the simulation mesh.
    formulation : Formulation
        Element formulation used to assemble the deformation energy.
    options : DeformationOptions, optional
        Assembly and material-step options.

    Properties
    ----------
    rest_state : ndarray (num_dofs,) float64
        Full generalized rest state, including formulation-specific DOFs.
    vertex_rest_positions : ndarray (num_vertices, 3) float64
        Rest positions of the actual mesh vertices.

    Notes
    -----
    Material derivatives use ``u`` for displacement, ``p`` for plastic-field
    DOFs, and ``e`` for elastic-field DOFs.
    """

    def __init__(self, assignment, *, formulation, options=None):
        if not isinstance(assignment, MaterialAssignment):
            raise TypeError(
                "assignment must be a MaterialAssignment, "
                f"got {type(assignment).__name__}"
            )
        formulation = _resolve_formulation(formulation)
        if options is None:
            options = DeformationOptions()
        if not isinstance(options, DeformationOptions):
            raise TypeError(
                f"options must be DeformationOptions, got {type(options).__name__}"
            )

        element_weights = options.element_weights
        if element_weights is not None:
            element_weights = np.asarray(
                element_weights, dtype=np.float64, order="C"
            )
            if element_weights.ndim != 1:
                raise ValueError(
                    "options.element_weights must be 1-D, "
                    f"got shape {element_weights.shape}"
                )
            if element_weights.size != assignment.mesh.num_elements:
                raise ValueError(
                    "options.element_weights size must be "
                    f"{assignment.mesh.num_elements}, got {element_weights.size}"
                )
            element_weights = np.ascontiguousarray(
                element_weights, dtype=np.float64
            )

        core = _core._create_deformation_energy(
            assignment._handle,
            formulation._handle,
            element_weights,
            bool(options.project_hessian_psd),
            bool(options.enable_material_max_step),
        )
        object.__setattr__(self, "_handle", core)
        object.__setattr__(
            self, "_elastic_definition", assignment.parameterization.elastic.definition)
        object.__setattr__(
            self, "_plastic_definition", assignment.parameterization.plastic.definition)
        object.__setattr__(
            self, "_optimizable_parameters", assignment.optimizable_parameters
        )
        object.__setattr__(self, "_assignment", assignment)
        super().__init__(core)

    @property
    def rest_state(self) -> np.ndarray:
        """Full generalized rest state, with shape ``(num_dofs,)``."""
        return np.asarray(self._handle.rest_state(), dtype=np.float64)

    @property
    def vertex_rest_positions(self) -> np.ndarray:
        """Rest positions of the mesh vertices, with shape ``(num_vertices, 3)``."""
        return np.asarray(self._handle.vertex_rest_positions(), dtype=np.float64)

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
        """Number of unique/global DOFs in the elastic parameter field."""
        return self._handle.num_elastic_dofs

    @property
    def num_plastic_dofs(self) -> int:
        """Number of unique/global DOFs in the plastic parameter field."""
        return self._handle.num_plastic_dofs

    @property
    def elastic_definition(self):
        return self._elastic_definition

    @property
    def plastic_definition(self):
        return self._plastic_definition

    @property
    def optimizable_parameters(self) -> OptimizableParameters:
        return self._optimizable_parameters

    @property
    def assignment(self) -> MaterialAssignment:
        """Complete mesh-bound material assignment used to build this energy."""
        return self._assignment

    def dE_de(self, displacement: np.ndarray) -> np.ndarray:
        """Return ``∂E/∂e`` with shape ``(num_elastic_dofs,)``."""
        u = float_vector("displacement", displacement)
        return np.asarray(self._handle.dE_de(u), dtype=np.float64)

    def element_von_mises(self, displacement: np.ndarray) -> np.ndarray:
        """Compute per-element von Mises stress.

        Parameters
        ----------
        displacement : ndarray, shape (num_dofs,)
            Current displacement vector relative to the rest position.

        Returns
        -------
        ndarray, shape (num_elements,)
            Per-element von Mises stress values.

        Raises
        ------
        NotImplementedError
            If the selected deformation or elastic model does not implement
            von Mises stress recovery.
        """
        u = float_vector("displacement", displacement)
        return np.asarray(self._handle.element_von_mises_stresses(u), dtype=np.float64)

    def d2E_de2(self, displacement: np.ndarray) -> SparseMatrix:
        """Return ``∂²E/∂e²`` with shape ``(num_elastic_dofs, num_elastic_dofs)``."""
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.d2E_de2(u))

    def d2E_dpde(self, displacement: np.ndarray) -> SparseMatrix:
        """Return ``∂²E/∂p∂e`` with shape ``(num_plastic_dofs, num_elastic_dofs)``."""
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.d2E_dpde(u))

    def dE_dp(self, displacement: np.ndarray) -> np.ndarray:
        """Return ``∂E/∂p`` with shape ``(num_plastic_dofs,)``."""
        u = float_vector("displacement", displacement)
        return np.asarray(self._handle.dE_dp(u), dtype=np.float64)

    def d2E_dp2(self, displacement: np.ndarray) -> SparseMatrix:
        """Return ``∂²E/∂p²`` with shape ``(num_plastic_dofs, num_plastic_dofs)``."""
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.d2E_dp2(u))

    def d2E_dude(self, displacement: np.ndarray) -> SparseMatrix:
        """Return ``∂²E/∂u∂e`` with shape ``(num_dofs, num_elastic_dofs)``."""
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.d2E_dude(u))

    def d2E_dudp(self, displacement: np.ndarray) -> SparseMatrix:
        """Return ``∂²E/∂u∂p`` with shape ``(num_dofs, num_plastic_dofs)``."""
        u = float_vector("displacement", displacement)
        return SparseMatrix(self._handle.d2E_dudp(u))

    def __repr__(self) -> str:
        return f"DeformationEnergy({self.num_dofs} DOFs, state_kind='{self.state_kind}')"


# ---------------------------------------------------------------------------
# DeformationOptions
# ---------------------------------------------------------------------------


@dataclass
class DeformationOptions:
    """Options controlling deformation-energy assembly.

    ``element_weights`` optionally supplies one scalar assembler weight per
    simulation-mesh element; ``None`` uses unit weights.
    """

    project_hessian_psd: bool = True
    enable_material_max_step: bool = True
    element_weights: np.ndarray | None = None


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


def _resolve_formulation(formulation):
    from pypgo.fem.formulations import Formulation

    if formulation is None:
        raise ValueError(
            "formulation is required. Pass TetLinear(), CubicLinear(), CubicTricubicHermite(), or KoiterShell()."
        )
    if not isinstance(formulation, Formulation):
        raise TypeError(
            f"formulation must be a Formulation, got {type(formulation).__name__}"
        )
    return formulation
