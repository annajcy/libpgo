"""PyTorch interoperability for differentiable pypgo equilibrium solves."""

from __future__ import annotations

from typing import Sequence

import numpy as np
import torch as _torch

from pypgo import solver
from pypgo._utils import float_vector, int_vector, vertex_array
from pypgo.energy import PotentialEnergy


class _StaticEquilibriumFunction(_torch.autograd.Function):
    @staticmethod
    def forward(ctx, parameter_values, layer):
        if parameter_values.device.type != "cpu":
            raise ValueError(f"{layer._layer_name} currently supports CPU tensors only")
        if parameter_values.dtype != _torch.float64:
            raise TypeError(f"{layer._layer_name} currently requires torch.float64 parameter tensors")
        if parameter_values.ndim != 1:
            raise ValueError(f"parameter_values must be 1-D, got shape {tuple(parameter_values.shape)}")

        parameter_np = parameter_values.detach().cpu().numpy().copy()
        if parameter_np.size != layer.num_parameter_dofs:
            raise ValueError(
                f"{layer._parameter_name}_values size must be {layer.num_parameter_dofs}, got {parameter_np.size}"
            )

        layer._set_parameter_values(parameter_np)
        problem = solver.OptimizationProblem(objective=layer.objective_energy)
        problem.fix_variables(
            layer.fixed_dofs.tolist(),
            layer.fixed_values,
            num_dofs=layer.energy.num_dofs,
        )
        inner = layer.inner_optimizer.solve(problem, layer._warm_start)
        layer._warm_start = inner.x.copy()

        surface_vertices = layer.surface_vertices + inner.x.reshape((-1, layer._dof_stride))[layer.surface_vertex_ids, :3]
        layer._last_equilibrium_displacement = inner.x.copy()
        layer._last_surface_vertices = surface_vertices.copy()
        layer._last_inner_result = inner

        ctx.layer = layer
        ctx.parameter_values = parameter_np
        ctx.displacement = inner.x.copy()
        return _torch.as_tensor(surface_vertices, dtype=parameter_values.dtype)

    @staticmethod
    def backward(ctx, grad_surface):
        layer = ctx.layer
        if grad_surface is None:
            return None, None
        if grad_surface.device.type != "cpu":
            raise ValueError(f"{layer._layer_name} currently supports CPU tensors only")

        layer._set_parameter_values(ctx.parameter_values)
        grad_surface_np = np.asarray(grad_surface.detach().cpu().numpy(), dtype=np.float64)
        if grad_surface_np.shape != layer.surface_vertices.shape:
            raise ValueError(
                f"grad_surface shape must be {layer.surface_vertices.shape}, got {grad_surface_np.shape}"
            )

        grad_u = np.zeros(layer.energy.num_dofs, dtype=np.float64)
        grad_u_reshaped = grad_u.reshape((-1, layer._dof_stride))
        padded = np.zeros((len(layer.surface_vertex_ids), layer._dof_stride), dtype=np.float64)
        padded[:, :3] = grad_surface_np
        np.add.at(grad_u_reshaped, layer.surface_vertex_ids, padded)

        grad_parameter = np.zeros(layer.num_parameter_dofs, dtype=np.float64)
        if layer.free_dofs.size:
            hessian = layer.objective_energy.hessian(ctx.displacement).to_dense()
            parameter_jacobian = layer._parameter_jacobian(ctx.displacement)
            adjoint = np.zeros(layer.energy.num_dofs, dtype=np.float64)
            adjoint[layer.free_dofs] = np.linalg.solve(
                hessian[np.ix_(layer.free_dofs, layer.free_dofs)],
                grad_u[layer.free_dofs],
            )
            grad_parameter = -(parameter_jacobian.T @ adjoint)

        return _torch.as_tensor(grad_parameter, dtype=grad_surface.dtype), None


class _BaseStaticEquilibriumLayer(_torch.nn.Module):
    """Shared implementation for implicitly differentiable equilibrium layers."""

    _layer_name = "BaseStaticEquilibriumLayer"
    _parameter_name = "parameter"

    def __init__(
        self,
        *,
        energy,
        fixed_dofs: Sequence[int],
        fixed_values,
        surface_vertices,
        surface_vertex_ids: Sequence[int],
        inner_optimizer: solver.Optimizer | None = None,
        objective_energy=None,
    ) -> None:
        super().__init__()
        if not isinstance(energy, PotentialEnergy):
            raise TypeError("energy must be a pypgo.energy.PotentialEnergy")
        if objective_energy is None:
            objective_energy = energy
        if not isinstance(objective_energy, PotentialEnergy):
            raise TypeError("objective_energy must be a pypgo.energy.PotentialEnergy")
        if objective_energy.num_dofs != energy.num_dofs:
            raise ValueError("objective_energy num_dofs must match energy.num_dofs")

        self.energy = energy
        self.objective_energy = objective_energy
        self.fixed_dofs = int_vector("fixed_dofs", fixed_dofs)
        self.fixed_values = float_vector("fixed_values", fixed_values)
        if self.fixed_values.size != self.fixed_dofs.size:
            raise ValueError("fixed_values size must match fixed_dofs size")
        if len(set(self.fixed_dofs.tolist())) != self.fixed_dofs.size:
            raise ValueError("fixed_dofs must be unique")

        self.surface_vertices = vertex_array("surface_vertices", surface_vertices)
        self.surface_vertex_ids = int_vector("surface_vertex_ids", surface_vertex_ids)
        if self.surface_vertex_ids.size != self.surface_vertices.shape[0]:
            raise ValueError("surface_vertex_ids size must match surface_vertices rows")
        if np.any(self.surface_vertex_ids < 0) or np.any(self.surface_vertex_ids >= energy.num_vertices):
            raise ValueError("surface_vertex_ids contain out-of-range vertex ids")

        self.inner_optimizer = inner_optimizer or solver.NewtonOptimizer()
        if not isinstance(self.inner_optimizer, solver.Optimizer):
            raise TypeError("inner_optimizer must be a pypgo.solver.Optimizer")

        self.plastic_shape = tuple(energy.plastic_field.values.shape)
        self.num_plastic_dofs = int(np.prod(self.plastic_shape))
        if self.num_plastic_dofs != energy.num_plastic_dofs:
            raise ValueError("plastic field size does not match energy.num_plastic_dofs")
        self.elastic_shape = tuple(energy.elastic_field.values.shape)
        self.num_elastic_dofs = int(np.prod(self.elastic_shape))
        if self.num_elastic_dofs != energy.num_elastic_dofs:
            raise ValueError("elastic field size does not match energy.num_elastic_dofs")

        self._dof_stride = energy.num_dofs // energy.num_vertices

        mask = np.ones(energy.num_dofs, dtype=bool)
        if np.any(self.fixed_dofs < 0) or np.any(self.fixed_dofs >= energy.num_dofs):
            raise ValueError("fixed_dofs contain out-of-range DOFs")
        mask[self.fixed_dofs] = False
        self.free_dofs = np.nonzero(mask)[0].astype(np.int64)

        self._warm_start = energy.zero_state()
        self._last_equilibrium_displacement: np.ndarray | None = None
        self._last_surface_vertices: np.ndarray | None = None
        self._last_inner_result = None

    def reset_warm_start(self, displacement=None) -> None:
        """Reset the forward solve warm start."""

        if displacement is None:
            self._warm_start = self.energy.zero_state()
            return
        self._warm_start = float_vector("displacement", displacement).copy()
        if self._warm_start.size != self.energy.num_dofs:
            raise ValueError("displacement size must match energy.num_dofs")

    @property
    def last_equilibrium_displacement(self) -> np.ndarray:
        if self._last_equilibrium_displacement is None:
            raise RuntimeError(f"{self._layer_name} has not run a forward pass yet")
        return self._last_equilibrium_displacement.copy()

    @property
    def last_surface_vertices(self) -> np.ndarray:
        if self._last_surface_vertices is None:
            raise RuntimeError(f"{self._layer_name} has not run a forward pass yet")
        return self._last_surface_vertices.copy()

    @property
    def last_inner_result(self):
        if self._last_inner_result is None:
            raise RuntimeError(f"{self._layer_name} has not run a forward pass yet")
        return self._last_inner_result

    def forward(self, parameter_values):
        return _StaticEquilibriumFunction.apply(parameter_values, self)


class PlasticStaticEquilibriumLayer(_BaseStaticEquilibriumLayer):
    """Implicitly differentiable equilibrium layer with plastic field input.

    The forward pass solves ``argmin_u E(u, a)`` for the given plastic field
    ``a`` and returns observed surface vertices. The backward pass uses
    ``energy.plastic_jacobian(u)`` in the adjoint contraction.
    """

    _layer_name = "PlasticStaticEquilibriumLayer"
    _parameter_name = "plastic"

    @property
    def parameter_shape(self):
        return self.plastic_shape

    @property
    def num_parameter_dofs(self) -> int:
        return self.num_plastic_dofs

    def _set_parameter_values(self, values) -> None:
        self.energy.set_plastic_values(values.reshape(self.plastic_shape))

    def _parameter_jacobian(self, displacement) -> np.ndarray:
        return self.energy.plastic_jacobian(displacement).to_dense()


class ElasticStaticEquilibriumLayer(_BaseStaticEquilibriumLayer):
    """Implicitly differentiable equilibrium layer with elastic field input.

    The forward pass solves ``argmin_u E(u, b)`` for the given elastic field
    ``b`` and returns observed surface vertices. The backward pass uses
    ``energy.elastic_jacobian(u)`` in the adjoint contraction.
    """

    _layer_name = "ElasticStaticEquilibriumLayer"
    _parameter_name = "elastic"

    @property
    def parameter_shape(self):
        return self.elastic_shape

    @property
    def num_parameter_dofs(self) -> int:
        return self.num_elastic_dofs

    def _set_parameter_values(self, values) -> None:
        self.energy.set_elastic_values(values.reshape(self.elastic_shape))

    def _parameter_jacobian(self, displacement) -> np.ndarray:
        return self.energy.elastic_jacobian(displacement).to_dense()


__all__ = [
    "PlasticStaticEquilibriumLayer",
    "ElasticStaticEquilibriumLayer",
]
