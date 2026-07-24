"""Deformation energy factory — direct construction from mesh, materials, and fields."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import pypgo._core as _core
from pypgo._utils import float_vector
from pypgo.energy import PotentialEnergy
from pypgo.fem.fields import (
    ConstantDofLayout,
    ElementwiseDofLayout,
    IdentityParameterMapping,
    MaterialParameters,
    ParameterDofLayout,
    ParameterMapping,
)
from pypgo.sparse import SparseMatrix


def _parameter_values_array(name, values, num_rows, num_local_dofs):
    arr = np.asarray(values, dtype=np.float64, order="C")
    if arr.ndim == 1:
        expected = num_rows * num_local_dofs
        if arr.size != expected:
            raise ValueError(f"{name} flat size must be {expected}, got {arr.size}")
        arr = arr.reshape((num_rows, num_local_dofs))
    if arr.ndim != 2:
        raise ValueError(f"{name} must be 1-D or 2-D, got shape {arr.shape}")
    if arr.shape != (num_rows, num_local_dofs):
        raise ValueError(f"{name} shape must be {(num_rows, num_local_dofs)}, got {arr.shape}")
    return np.ascontiguousarray(arr, dtype=np.float64)


def _layout_rows(name, layout, num_elements):
    if isinstance(layout, ElementwiseDofLayout):
        return num_elements
    if isinstance(layout, ConstantDofLayout):
        return 1
    raise TypeError(
        f"{name} must be ElementwiseDofLayout or ConstantDofLayout, got {type(layout).__name__}"
    )


def _parameter_init_values(name, values, layout, num_elements, num_local_dofs):
    if values is None:
        return None
    rows = _layout_rows(f"{name.removesuffix('_values')}_layout", layout, num_elements)
    return _parameter_values_array(name, values, rows, num_local_dofs).ravel()


def _require_sim_mesh(sim_mesh):
    from pypgo.fem.mesh import SimulationMesh as _SimulationMesh

    if not isinstance(sim_mesh, _SimulationMesh):
        raise TypeError(
            f"sim_mesh must be a SimulationMesh, got {type(sim_mesh).__name__}"
        )
    return sim_mesh


def _elastic_value_channels(sim_mesh, elastic):
    from pypgo.fem.elastic import ElasticModelConfig

    if isinstance(elastic, ElasticModelConfig):
        return elastic._handle.num_channels(sim_mesh._handle)
    raise TypeError(f"elastic must be an ElasticModelConfig, got {type(elastic).__name__}")


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
    rest_state : ndarray (num_dofs,) float64
        Full generalized rest state, including formulation-specific DOFs.
    vertex_rest_positions : ndarray (num_vertices, 3) float64
        Rest positions of the actual mesh vertices.

    Notes
    -----
    Material derivatives use ``u`` for displacement, ``p`` for plastic-field
    DOFs, and ``e`` for elastic-field DOFs.
    """

    def __init__(self, core, *, elastic_model=None, plastic_model=None):
        if not isinstance(core, _core.PyDeformationEnergy):
            raise TypeError(
                f"core must be a PyDeformationEnergy, got {type(core).__name__}"
            )
        object.__setattr__(self, "_handle", core)
        object.__setattr__(self, "_elastic_model", elastic_model)
        object.__setattr__(self, "_plastic_model", plastic_model)
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
    def elastic_model(self):
        return self._elastic_model

    @property
    def plastic_model(self):
        return self._plastic_model

    @property
    def parameters(self) -> MaterialParameters:
        return MaterialParameters(self._handle.parameters)

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


class PlasticMaterialEnergy(PotentialEnergy):
    """Material energy with the plastic field as the optimization variable.

    Created by ``pypgo.fem.plastic_material_energy()``. The displacement is fixed;
    the input state vector is the plastic field's global DOF vector.
    """

    def __init__(self, handle, *, deformation_energy, fixed_displacement):
        object.__setattr__(self, "_deformation_energy", deformation_energy)
        object.__setattr__(
            self,
            "_fixed_displacement",
            np.asarray(fixed_displacement, dtype=np.float64).copy(),
        )
        super().__init__(handle)

    @property
    def deformation_energy(self):
        return self._deformation_energy

    @property
    def fixed_displacement(self) -> np.ndarray:
        return self._fixed_displacement.copy()

    def __repr__(self) -> str:
        return f"PlasticMaterialEnergy({self.num_dofs} DOFs, state_kind='{self.state_kind}')"


class ElasticMaterialEnergy(PotentialEnergy):
    """Material energy with the elastic field as the optimization variable.

    Created by ``pypgo.fem.elastic_material_energy()``. The displacement is fixed;
    the input state vector is the elastic field's global DOF vector.
    """

    def __init__(self, handle, *, deformation_energy, fixed_displacement):
        object.__setattr__(self, "_deformation_energy", deformation_energy)
        object.__setattr__(
            self,
            "_fixed_displacement",
            np.asarray(fixed_displacement, dtype=np.float64).copy(),
        )
        super().__init__(handle)

    @property
    def deformation_energy(self):
        return self._deformation_energy

    @property
    def fixed_displacement(self) -> np.ndarray:
        return self._fixed_displacement.copy()

    def __repr__(self) -> str:
        return f"ElasticMaterialEnergy({self.num_dofs} DOFs, state_kind='{self.state_kind}')"


# ---------------------------------------------------------------------------
# DeformationOptions
# ---------------------------------------------------------------------------


@dataclass
class DeformationOptions:
    """Options controlling deformation-energy assembly.

    ``element_weights`` optionally supplies one scalar assembler weight per
    simulation-mesh element; ``None`` uses unit weights.
    """

    enforce_spd: bool = True
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


# ---------------------------------------------------------------------------
# Energy factories
# ---------------------------------------------------------------------------


def deformation_energy(
    sim_mesh,
    *,
    elastic,
    elastic_layout=None,
    elastic_mapping=None,
    elastic_values=None,
    plastic,
    plastic_layout=None,
    plastic_mapping=None,
    plastic_values=None,
    formulation=None,
    options=None,
) -> DeformationEnergy:
    sim_mesh = _require_sim_mesh(sim_mesh)
    formulation = _resolve_formulation(formulation)

    from pypgo.fem.elastic import ElasticModelConfig
    from pypgo.fem.plastic import PlasticModelConfig

    if not isinstance(elastic, ElasticModelConfig):
        raise TypeError(f"elastic must be an ElasticModelConfig, got {type(elastic).__name__}")
    if not isinstance(plastic, PlasticModelConfig):
        raise TypeError(f"plastic must be a PlasticModelConfig, got {type(plastic).__name__}")

    if elastic_layout is None:
        elastic_layout = ElementwiseDofLayout()
    if plastic_layout is None:
        plastic_layout = ElementwiseDofLayout()
    if elastic_mapping is None:
        elastic_mapping = IdentityParameterMapping()
    if plastic_mapping is None:
        plastic_mapping = IdentityParameterMapping()
    if not isinstance(elastic_layout, ParameterDofLayout):
        raise TypeError(
            f"elastic_layout must be a ParameterDofLayout, got {type(elastic_layout).__name__}"
        )
    if not isinstance(plastic_layout, ParameterDofLayout):
        raise TypeError(
            f"plastic_layout must be a ParameterDofLayout, got {type(plastic_layout).__name__}"
        )
    if not isinstance(elastic_mapping, ParameterMapping):
        raise TypeError(
            f"elastic_mapping must be a ParameterMapping, got {type(elastic_mapping).__name__}"
        )
    if not isinstance(plastic_mapping, ParameterMapping):
        raise TypeError(
            f"plastic_mapping must be a ParameterMapping, got {type(plastic_mapping).__name__}"
        )

    num_elastic_channels = _elastic_value_channels(sim_mesh, elastic)
    num_plastic_channels = plastic.dofs
    elastic_values = _parameter_init_values(
        "elastic_values",
        elastic_values,
        elastic_layout,
        sim_mesh.num_elements,
        num_elastic_channels,
    )
    plastic_values = _parameter_init_values(
        "plastic_values",
        plastic_values,
        plastic_layout,
        sim_mesh.num_elements,
        num_plastic_channels,
    )

    if options is None:
        options = DeformationOptions()
    if not isinstance(options, DeformationOptions):
        raise TypeError(f"options must be DeformationOptions, got {type(options).__name__}")

    element_weights = options.element_weights
    if element_weights is not None:
        element_weights = np.asarray(element_weights, dtype=np.float64, order="C")
        if element_weights.ndim != 1:
            raise ValueError(
                f"options.element_weights must be 1-D, got shape {element_weights.shape}"
            )
        if element_weights.size != sim_mesh.num_elements:
            raise ValueError(
                "options.element_weights size must be "
                f"{sim_mesh.num_elements}, got {element_weights.size}"
            )
        element_weights = np.ascontiguousarray(element_weights, dtype=np.float64)

    core = _core._create_deformation_energy(
        sim_mesh._handle,
        elastic._handle,
        elastic_values,
        plastic._handle,
        plastic_values,
        elastic_layout._handle,
        elastic_mapping._handle,
        plastic_layout._handle,
        plastic_mapping._handle,
        formulation._handle,
        element_weights,
        bool(options.enforce_spd),
        bool(options.enable_material_max_step),
    )
    return DeformationEnergy(core, elastic_model=elastic, plastic_model=plastic)


def plastic_material_energy(
    deformation_energy,
    *,
    fixed_displacement,
) -> PlasticMaterialEnergy:
    """Create a material energy whose optimization variable is the plastic field."""
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
        deformation_energy._handle,
        u,
    )
    return PlasticMaterialEnergy(
        handle,
        deformation_energy=deformation_energy,
        fixed_displacement=u,
    )


def elastic_material_energy(
    deformation_energy,
    *,
    fixed_displacement,
) -> ElasticMaterialEnergy:
    """Create a material energy whose optimization variable is the elastic field."""
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

    handle = _core._create_elastic_material_energy(
        deformation_energy._handle,
        u,
    )
    return ElasticMaterialEnergy(
        handle,
        deformation_energy=deformation_energy,
        fixed_displacement=u,
    )
