"""FEM deformation and material energies."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector
from pypgo.energy import PotentialEnergy
from pypgo.fem.fields import MaterialBinding, MaterialState
from pypgo.fem.mesh import SimulationMesh
from pypgo.sparse import SparseMatrix


@dataclass(frozen=True)
class MaterialVJP:
    """Result of ``(d²E / du dm).T @ adjoint`` for both material domains."""

    elastic: np.ndarray
    plastic: np.ndarray


# ---------------------------------------------------------------------------
# DeformationEnergyOperator — FEM deformation energy
# ---------------------------------------------------------------------------


class DeformationEnergyOperator:
    """Explicit differentiable operator ``(u, material_state) -> E``.

    Parameters
    ----------
    mesh : SimulationMesh
        Simulation mesh used by the operator.
    material_binding : MaterialBinding
        Immutable material models, elementwise fixed values, and frames.
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

    def __init__(self, mesh, material_binding, *, formulation, options=None):
        if not isinstance(mesh, SimulationMesh):
            raise TypeError("mesh must be a SimulationMesh")
        if not isinstance(material_binding, MaterialBinding):
            raise TypeError("material_binding must be a MaterialBinding")
        if material_binding.num_elements != mesh.num_elements:
            raise ValueError("material binding element count must match mesh")
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
            if element_weights.size != mesh.num_elements:
                raise ValueError(
                    "options.element_weights size must be "
                    f"{mesh.num_elements}, got {element_weights.size}"
                )
            element_weights = np.ascontiguousarray(
                element_weights, dtype=np.float64
            )

        core = _core._create_deformation_energy_operator(
            mesh._handle,
            material_binding._handle,
            formulation._handle,
            element_weights,
            bool(options.project_hessian_psd),
            bool(options.enable_material_max_step),
        )
        object.__setattr__(self, "_handle", core)

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
    def num_elements(self) -> int:
        return self._handle.num_elements

    @property
    def num_elastic_params(self) -> int:
        """Per-element elastic parameter count."""
        return self._handle.num_elastic_params

    @property
    def num_plastic_params(self) -> int:
        """Per-element plastic parameter count."""
        return self._handle.num_plastic_params

    @property
    def num_elastic_values(self) -> int:
        """Number of element-major elastic material values."""
        return self._handle.num_elastic_values

    @property
    def num_plastic_values(self) -> int:
        """Number of element-major plastic material values."""
        return self._handle.num_plastic_values

    @property
    def num_dofs(self) -> int:
        return self._handle.num_dofs

    def value(self, displacement: np.ndarray, material_state: MaterialState) -> float:
        u, state = self._inputs(displacement, material_state)
        return float(self._handle.value(u, state._handle))

    def gradient(self, displacement: np.ndarray, material_state: MaterialState) -> np.ndarray:
        u, state = self._inputs(displacement, material_state)
        return np.asarray(self._handle.gradient(u, state._handle), dtype=np.float64)

    def hessian(self, displacement: np.ndarray, material_state: MaterialState) -> SparseMatrix:
        u, state = self._inputs(displacement, material_state)
        return SparseMatrix(self._handle.hessian(u, state._handle))

    def zero_state(self) -> np.ndarray:
        return np.asarray(self._handle.zero_state(), dtype=np.float64)

    def dE_de(self, displacement: np.ndarray, material_state: MaterialState) -> np.ndarray:
        """Return ``∂E/∂e`` with shape ``(num_elastic_values,)``."""
        u, state = self._inputs(displacement, material_state)
        return np.asarray(self._handle.dE_de(u, state._handle), dtype=np.float64)

    def element_von_mises(self, displacement: np.ndarray, material_state: MaterialState) -> np.ndarray:
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
        u, state = self._inputs(displacement, material_state)
        return np.asarray(
            self._handle.element_von_mises_stresses(u, state._handle),
            dtype=np.float64)

    def dE_dp(self, displacement: np.ndarray, material_state: MaterialState) -> np.ndarray:
        """Return ``∂E/∂p`` with shape ``(num_plastic_values,)``."""
        u, state = self._inputs(displacement, material_state)
        return np.asarray(self._handle.dE_dp(u, state._handle), dtype=np.float64)

    def elastic_material_vjp(
        self, displacement: np.ndarray, material_state: MaterialState,
        adjoint: np.ndarray,
    ) -> np.ndarray:
        """Return ``(∂²E/∂u∂e)ᵀ adjoint`` without assembling the mixed Hessian."""
        u, state = self._inputs(displacement, material_state)
        adjoint = float_vector("adjoint", adjoint)
        if adjoint.size != self.num_dofs:
            raise ValueError("adjoint size must match operator.num_dofs")
        return np.asarray(
            self._handle.elastic_material_vjp(u, state._handle, adjoint),
            dtype=np.float64,
        )

    def plastic_material_vjp(
        self, displacement: np.ndarray, material_state: MaterialState,
        adjoint: np.ndarray,
    ) -> np.ndarray:
        """Return ``(∂²E/∂u∂p)ᵀ adjoint`` without assembling the mixed Hessian."""
        u, state = self._inputs(displacement, material_state)
        adjoint = float_vector("adjoint", adjoint)
        if adjoint.size != self.num_dofs:
            raise ValueError("adjoint size must match operator.num_dofs")
        return np.asarray(
            self._handle.plastic_material_vjp(u, state._handle, adjoint),
            dtype=np.float64,
        )

    def material_vjp(
        self, displacement: np.ndarray, material_state: MaterialState,
        adjoint: np.ndarray,
    ) -> MaterialVJP:
        """Direct material VJP; this is the recommended adjoint interface."""
        return MaterialVJP(
            elastic=self.elastic_material_vjp(
                displacement, material_state, adjoint),
            plastic=self.plastic_material_vjp(
                displacement, material_state, adjoint),
        )

    def _inputs(self, displacement, material_state):
        if not isinstance(material_state, MaterialState):
            raise TypeError("material_state must be a MaterialState")
        elastic_size = self.num_elastic_values
        plastic_size = self.num_plastic_values
        if material_state.elastic_values.size != elastic_size:
            raise ValueError(
                f"elastic material state must contain {elastic_size} values")
        if material_state.plastic_values.size != plastic_size:
            raise ValueError(
                f"plastic material state must contain {plastic_size} values")
        u = float_vector("displacement", displacement)
        if u.size != self.num_dofs:
            raise ValueError("displacement size must match operator.num_dofs")
        return u, material_state

    def __repr__(self) -> str:
        return f"DeformationEnergyOperator({self.num_dofs} DOFs)"


class DeformationPotentialEnergy(PotentialEnergy):
    """Displacement potential with one immutable material state fixed."""

    def __init__(self, energy_operator, material_state):
        if not isinstance(energy_operator, DeformationEnergyOperator):
            raise TypeError("energy_operator must be a DeformationEnergyOperator")
        if not isinstance(material_state, MaterialState):
            raise TypeError("material_state must be a MaterialState")
        handle = _core._create_deformation_potential_energy(
            energy_operator._handle, material_state._handle)
        object.__setattr__(self, "energy_operator", energy_operator)
        object.__setattr__(self, "material_state", material_state)
        super().__init__(handle)

    @property
    def rest_state(self):
        return self.energy_operator.rest_state

    @property
    def vertex_rest_positions(self):
        return self.energy_operator.vertex_rest_positions

    @property
    def num_vertices(self):
        return self.energy_operator.num_vertices

    @property
    def num_elements(self):
        return self.energy_operator.num_elements

    @property
    def num_elastic_params(self):
        return self.energy_operator.num_elastic_params

    @property
    def num_plastic_params(self):
        return self.energy_operator.num_plastic_params

    @property
    def num_elastic_values(self):
        return self.energy_operator.num_elastic_values

    @property
    def num_plastic_values(self):
        return self.energy_operator.num_plastic_values

    def dE_de(self, displacement):
        return self.energy_operator.dE_de(displacement, self.material_state)

    def dE_dp(self, displacement):
        return self.energy_operator.dE_dp(displacement, self.material_state)

    def material_vjp(self, displacement, adjoint):
        return self.energy_operator.material_vjp(
            displacement, self.material_state, adjoint)

    def elastic_material_vjp(self, displacement, adjoint):
        return self.energy_operator.elastic_material_vjp(
            displacement, self.material_state, adjoint)

    def plastic_material_vjp(self, displacement, adjoint):
        return self.energy_operator.plastic_material_vjp(
            displacement, self.material_state, adjoint)

    def element_von_mises(self, displacement):
        return self.energy_operator.element_von_mises(
            displacement, self.material_state)

    def __repr__(self):
        return f"DeformationPotentialEnergy({self.num_dofs} DOFs)"


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
