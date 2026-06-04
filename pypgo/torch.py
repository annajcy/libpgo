"""PyTorch interoperability for differentiable pypgo equilibrium solves."""

from __future__ import annotations

from typing import Sequence

import numpy as np
import torch as _torch

from pypgo import solver


def _float_vector(name: str, values) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64, order="C")
    if arr.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {arr.shape}")
    return arr


def _float_matrix3(name: str, values) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64, order="C")
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"{name} must have shape (n, 3), got {arr.shape}")
    return arr


def _int_vector(name: str, values) -> np.ndarray:
    arr = np.asarray(values, dtype=np.int64, order="C")
    if arr.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {arr.shape}")
    return arr


class _StaticEquilibriumFunction(_torch.autograd.Function):
    @staticmethod
    def forward(ctx, plastic_values, layer):
        if plastic_values.device.type != "cpu":
            raise ValueError("StaticEquilibriumLayer currently supports CPU tensors only")
        if plastic_values.dtype != _torch.float64:
            raise TypeError("StaticEquilibriumLayer currently requires torch.float64 plastic tensors")
        if plastic_values.ndim != 1:
            raise ValueError(f"plastic_values must be 1-D, got shape {tuple(plastic_values.shape)}")

        plastic_np = plastic_values.detach().cpu().numpy().copy()
        if plastic_np.size != layer.num_plastic_dofs:
            raise ValueError(
                f"plastic_values size must be {layer.num_plastic_dofs}, got {plastic_np.size}"
            )

        layer.state.set_plastic_values(plastic_np.reshape(layer.plastic_shape))
        problem = solver.OptimizationProblem(objective=layer.energy)
        problem.fix_variables(
            layer.fixed_dofs.tolist(),
            layer.fixed_values,
            num_dofs=layer.energy.num_dofs,
        )
        inner = layer.inner_optimizer.solve(problem, layer._warm_start)
        layer._warm_start = inner.x.copy()

        surface_vertices = layer.surface_vertices + inner.x.reshape((-1, 3))[layer.surface_vertex_ids]
        layer._last_equilibrium_displacement = inner.x.copy()
        layer._last_surface_vertices = surface_vertices.copy()
        layer._last_inner_result = inner

        ctx.layer = layer
        ctx.plastic_values = plastic_np
        ctx.displacement = inner.x.copy()
        return _torch.as_tensor(surface_vertices, dtype=plastic_values.dtype)

    @staticmethod
    def backward(ctx, grad_surface):
        layer = ctx.layer
        if grad_surface is None:
            return None, None
        if grad_surface.device.type != "cpu":
            raise ValueError("StaticEquilibriumLayer currently supports CPU tensors only")

        layer.state.set_plastic_values(ctx.plastic_values.reshape(layer.plastic_shape))
        grad_surface_np = np.asarray(grad_surface.detach().cpu().numpy(), dtype=np.float64)
        if grad_surface_np.shape != layer.surface_vertices.shape:
            raise ValueError(
                f"grad_surface shape must be {layer.surface_vertices.shape}, got {grad_surface_np.shape}"
            )

        grad_u = np.zeros(layer.energy.num_dofs, dtype=np.float64)
        np.add.at(grad_u.reshape((-1, 3)), layer.surface_vertex_ids, grad_surface_np)

        grad_plastic = np.zeros(layer.num_plastic_dofs, dtype=np.float64)
        if layer.free_dofs.size:
            hessian = layer.energy.hessian(ctx.displacement).to_dense()
            plastic_jacobian = layer.energy.plastic_jacobian(ctx.displacement).to_dense()
            adjoint = np.zeros(layer.energy.num_dofs, dtype=np.float64)
            adjoint[layer.free_dofs] = np.linalg.solve(
                hessian[np.ix_(layer.free_dofs, layer.free_dofs)],
                grad_u[layer.free_dofs],
            )
            grad_plastic = -(plastic_jacobian.T @ adjoint)

        return _torch.as_tensor(grad_plastic, dtype=grad_surface.dtype), None


class StaticEquilibriumLayer(_torch.nn.Module):
    """Implicitly differentiable static equilibrium layer.

    The forward pass solves ``argmin_u E(u, a)`` for the given plastic field
    ``a`` and returns observed surface vertices. The backward pass uses implicit
    differentiation of the equilibrium equation, solving the adjoint system
    ``H_ff lambda = dL/du_f`` and returning ``-J.T @ lambda``.
    """

    def __init__(
        self,
        *,
        state,
        energy,
        fixed_dofs: Sequence[int],
        fixed_values,
        surface_vertices,
        surface_vertex_ids: Sequence[int],
        inner_optimizer: solver.NewtonOptimizer | None = None,
    ) -> None:
        super().__init__()
        if not hasattr(energy, "_handle"):
            raise TypeError("energy must be a pypgo.energy PotentialEnergy-compatible object")

        self.state = state
        self.energy = energy
        self.fixed_dofs = _int_vector("fixed_dofs", fixed_dofs)
        self.fixed_values = _float_vector("fixed_values", fixed_values)
        if self.fixed_values.size != self.fixed_dofs.size:
            raise ValueError("fixed_values size must match fixed_dofs size")
        if len(set(self.fixed_dofs.tolist())) != self.fixed_dofs.size:
            raise ValueError("fixed_dofs must be unique")

        self.surface_vertices = _float_matrix3("surface_vertices", surface_vertices)
        self.surface_vertex_ids = _int_vector("surface_vertex_ids", surface_vertex_ids)
        if self.surface_vertex_ids.size != self.surface_vertices.shape[0]:
            raise ValueError("surface_vertex_ids size must match surface_vertices rows")
        if np.any(self.surface_vertex_ids < 0) or np.any(self.surface_vertex_ids >= energy.num_vertices):
            raise ValueError("surface_vertex_ids contain out-of-range vertex ids")

        self.inner_optimizer = inner_optimizer or solver.NewtonOptimizer()
        if not isinstance(self.inner_optimizer, solver.NewtonOptimizer):
            raise TypeError("inner_optimizer must be a pypgo.solver.NewtonOptimizer")

        self.plastic_shape = tuple(state.plastic_field.values.shape)
        self.num_plastic_dofs = int(np.prod(self.plastic_shape))
        if self.num_plastic_dofs != energy.num_plastic_dofs:
            raise ValueError("state plastic field size does not match energy.num_plastic_dofs")

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
        self._warm_start = _float_vector("displacement", displacement).copy()
        if self._warm_start.size != self.energy.num_dofs:
            raise ValueError("displacement size must match energy.num_dofs")

    @property
    def last_equilibrium_displacement(self) -> np.ndarray:
        if self._last_equilibrium_displacement is None:
            raise RuntimeError("StaticEquilibriumLayer has not run a forward pass yet")
        return self._last_equilibrium_displacement.copy()

    @property
    def last_surface_vertices(self) -> np.ndarray:
        if self._last_surface_vertices is None:
            raise RuntimeError("StaticEquilibriumLayer has not run a forward pass yet")
        return self._last_surface_vertices.copy()

    @property
    def last_inner_result(self):
        if self._last_inner_result is None:
            raise RuntimeError("StaticEquilibriumLayer has not run a forward pass yet")
        return self._last_inner_result

    def forward(self, plastic_values):
        return _StaticEquilibriumFunction.apply(plastic_values, self)


__all__ = [
    "StaticEquilibriumLayer",
]
