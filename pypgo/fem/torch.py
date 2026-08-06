"""PyTorch interoperability for differentiable pypgo equilibrium solves.

This module is imported lazily: ``import pypgo.fem.torch`` pulls in torch and
is intended for users who install the optional ``pypgo[torch]`` extra.  It
wraps the existing C++ equilibrium solver and material VJP in a thin
``torch.autograd.Function``, so the finite-element math stays in the C++
backend while torch owns the autograd graph and the outer optimization loop.

The layer implements the implicit-function-theorem backward for the static
equilibrium ``u*(m) = argmin_u E(u; m)``:

* displacement output:  ``grad_m = -material_vjp(u*, lambda)`` with
  ``K_ff^T lambda = grad_u_f``;
* reaction output:  ``grad_m = material_vjp(u*, b) - material_vjp(u*, lambda)``
  with ``b = S^T grad_y`` and ``K_ff^T lambda = (K b)_f``.

Only CPU float64 tensors are supported, matching the C++ backend's double
precision.  The solve itself is treated as a black box: backward uses the
Hessian at the converged displacement, never unrolled Newton iterations.
First-order gradients only: double backward (second derivatives through the
equilibrium solve) is not implemented in v1.
"""

from __future__ import annotations

import numpy as np
import torch

import pypgo._core as _core
from pypgo import solver as _solver
from pypgo._utils import float_vector, int_vector
from pypgo.fem.energy import DeformationEnergyOperator, DeformationPotentialEnergy
from pypgo.fem.fields import MaterialState
from pypgo.sparse import SparseMatrix

__all__ = ["StaticEnergyLayer", "StaticForceLayer", "StaticEquilibriumLayer"]


def _check_tensor_input(name: str, value) -> None:
    if not isinstance(value, torch.Tensor):
        raise TypeError(f"{name} must be a torch.Tensor")
    if value.device.type != "cpu":
        raise ValueError(
            f"StaticEquilibriumLayer currently supports CPU tensors only; "
            f"{name} is on {value.device}")
    if value.dtype != torch.float64:
        raise TypeError(
            f"StaticEquilibriumLayer requires torch.float64 tensors; "
            f"{name} has dtype {value.dtype}")


def _solve_free_system(
    hessian: SparseMatrix,
    rhs: np.ndarray,
    fixed_dofs: np.ndarray,
    fixed_row_mask: np.ndarray,
    backend,
) -> np.ndarray:
    """Solve ``K_ff lambda = rhs_free`` with ``lambda_fixed = 0``.

    The free block is enforced by zeroing every fixed row *and* column of the
    full Hessian (keeping it symmetric for the LDLT backends) and overlaying an
    identity on the fixed diagonal.  Because the sparse COO builder merges
    duplicate entries, the identity overlay simply sums with the zeroed
    diagonal.
    """
    rows, cols, values = hessian.to_coo()
    values = values.copy()
    values[fixed_row_mask[rows] | fixed_row_mask[cols]] = 0.0
    masked = SparseMatrix.from_coo(
        hessian.shape,
        np.concatenate([rows, fixed_dofs]),
        np.concatenate([cols, fixed_dofs]),
        np.concatenate([values, np.ones(fixed_dofs.size)]),
    )
    rhs = np.asarray(rhs, dtype=np.float64).copy()
    rhs[fixed_dofs] = 0.0
    return masked.solve(rhs, backend=backend)


class _StaticEquilibriumFunction(torch.autograd.Function):
    @staticmethod
    def forward(ctx, elastic_values, plastic_values, layer):
        _check_tensor_input("elastic_values", elastic_values)
        _check_tensor_input("plastic_values", plastic_values)
        if elastic_values.ndim != 2 or plastic_values.ndim != 2:
            raise ValueError(
                "elastic_values and plastic_values must be 2-D "
                "(num_elements, num_channels); broadcast global parameters "
                "with tensor.expand(num_elements, -1)")
        operator = layer.energy_operator
        if elastic_values.shape != (
                operator.num_elements, operator.num_elastic_params):
            raise ValueError(
                "elastic_values shape must be "
                f"({operator.num_elements}, {operator.num_elastic_params}), "
                f"got {tuple(elastic_values.shape)}")
        if plastic_values.shape != (
                operator.num_elements, operator.num_plastic_params):
            raise ValueError(
                "plastic_values shape must be "
                f"({operator.num_elements}, {operator.num_plastic_params}), "
                f"got {tuple(plastic_values.shape)}")

        elastic_np = elastic_values.detach().cpu().contiguous().numpy()
        plastic_np = plastic_values.detach().cpu().contiguous().numpy()

        state = MaterialState(elastic_np.reshape(-1), plastic_np.reshape(-1))
        energy = DeformationPotentialEnergy(operator, state)
        problem = _solver.OptimizationProblem(objective=energy)
        problem.fix_variables(
            layer.fixed_dofs.tolist(), layer.fixed_values,
            num_dofs=energy.num_dofs)
        x0 = layer.initial_displacement.copy()
        x0[layer.fixed_dofs] = layer.fixed_values
        result = layer.inner_optimizer.solve(problem, x0)
        x = result.x.copy()
        layer._last_displacement = x
        layer._last_result = result

        if layer.residual_tol is not None and layer.free_dofs.size:
            residual = np.max(np.abs(energy.gradient(x)[layer.free_dofs]))
            if residual > layer.residual_tol:
                raise RuntimeError(
                    "static equilibrium solve failed: max free residual "
                    f"{residual:.3e} exceeds tolerance {layer.residual_tol:.3e} "
                    f"(status={result.status.name}, iterations={result.iterations})")

        ctx.layer = layer
        ctx.energy = energy
        ctx.displacement = x
        ctx.elastic_shape = elastic_np.shape
        ctx.plastic_shape = plastic_np.shape

        if layer.reaction_selectors is None:
            return torch.from_numpy(x)
        reaction = layer.reaction_selectors @ energy.gradient(x)
        if layer._single_reaction:
            return torch.from_numpy(np.asarray(reaction[0]))
        return torch.from_numpy(reaction)

    @staticmethod
    def backward(ctx, grad_output):
        if grad_output is None:
            return None, None, None
        layer = ctx.layer
        operator = layer.energy_operator
        grad_np = np.asarray(grad_output.detach().cpu().numpy(), dtype=np.float64)

        if layer.reaction_selectors is None:
            if grad_np.shape != (operator.num_dofs,):
                raise ValueError(
                    "grad_output for displacement output must have shape "
                    f"({operator.num_dofs},), got {grad_np.shape}")
            direct_adjoint = None
            rhs = grad_np
        else:
            if layer._single_reaction:
                grad_np = grad_np.reshape(1)
            if grad_np.shape[0] != layer.reaction_selectors.shape[0]:
                raise ValueError(
                    "grad_output for reaction output must have shape "
                    f"({layer.reaction_selectors.shape[0]},), got {grad_np.shape}")
            direct_adjoint = layer.reaction_selectors.T @ grad_np
            rhs = None  # computed once the Hessian is available

        hessian = ctx.energy.hessian(ctx.displacement)
        if layer.reaction_selectors is not None:
            rhs = hessian @ direct_adjoint
        lambda_free = _solve_free_system(
            hessian, rhs, layer.fixed_dofs,
            layer._fixed_row_mask, layer.sparse_backend)

        if layer.reaction_selectors is None:
            grad_e = -ctx.energy.elastic_material_vjp(
                ctx.displacement, lambda_free)
            grad_p = -ctx.energy.plastic_material_vjp(
                ctx.displacement, lambda_free)
        else:
            grad_e = (
                ctx.energy.elastic_material_vjp(ctx.displacement, direct_adjoint)
                - ctx.energy.elastic_material_vjp(ctx.displacement, lambda_free))
            grad_p = (
                ctx.energy.plastic_material_vjp(ctx.displacement, direct_adjoint)
                - ctx.energy.plastic_material_vjp(ctx.displacement, lambda_free))
        return (
            torch.from_numpy(grad_e.reshape(ctx.elastic_shape)),
            torch.from_numpy(grad_p.reshape(ctx.plastic_shape)),
            None,
        )


def _build_energy_evaluation(
    displacement,
    elastic_values,
    plastic_values,
    layer,
):
    """Validate inputs and bind the material state to the operator.

    Shared by the energy and force autograd functions; returns the NumPy
    inputs plus the bound potential energy ready for C++ evaluation.
    """
    _check_tensor_input("displacement", displacement)
    _check_tensor_input("elastic_values", elastic_values)
    _check_tensor_input("plastic_values", plastic_values)
    operator = layer.energy_operator
    if displacement.ndim != 1:
        raise ValueError(
            f"displacement must be 1-D with shape ({operator.num_dofs},), "
            f"got {tuple(displacement.shape)}")
    if displacement.shape[0] != operator.num_dofs:
        raise ValueError(
            f"displacement size must be {operator.num_dofs}, "
            f"got {displacement.shape[0]}")
    if elastic_values.ndim != 2 or plastic_values.ndim != 2:
        raise ValueError(
            "elastic_values and plastic_values must be 2-D "
            "(num_elements, num_channels); broadcast global parameters "
            "with tensor.expand(num_elements, -1)")
    if elastic_values.shape != (
            operator.num_elements, operator.num_elastic_params):
        raise ValueError(
            "elastic_values shape must be "
            f"({operator.num_elements}, {operator.num_elastic_params}), "
            f"got {tuple(elastic_values.shape)}")
    if plastic_values.shape != (
            operator.num_elements, operator.num_plastic_params):
        raise ValueError(
            "plastic_values shape must be "
            f"({operator.num_elements}, {operator.num_plastic_params}), "
            f"got {tuple(plastic_values.shape)}")

    displacement_np = displacement.detach().cpu().contiguous().numpy()
    elastic_np = elastic_values.detach().cpu().contiguous().numpy()
    plastic_np = plastic_values.detach().cpu().contiguous().numpy()
    state = MaterialState(elastic_np.reshape(-1), plastic_np.reshape(-1))
    energy = DeformationPotentialEnergy(operator, state)
    return displacement_np, elastic_np, plastic_np, energy


class _EnergyFunction(torch.autograd.Function):
    """Differentiable scalar energy evaluation ``E(u; m)``.

    Forward returns ``E``; backward returns the exact C++ first derivatives
    ``dE/du`` (force gradient), ``dE/de`` and ``dE/dp``.
    """

    @staticmethod
    def forward(ctx, displacement, elastic_values, plastic_values, layer):
        displacement_np, elastic_np, plastic_np, energy = (
            _build_energy_evaluation(
                displacement, elastic_values, plastic_values, layer))
        ctx.energy = energy
        ctx.displacement = displacement_np
        ctx.elastic_shape = elastic_np.shape
        ctx.plastic_shape = plastic_np.shape
        ctx.grad_u = energy.gradient(displacement_np)
        ctx.grad_e = energy.dE_de(displacement_np)
        ctx.grad_p = energy.dE_dp(displacement_np)
        return torch.from_numpy(np.asarray(energy.value(displacement_np)))

    @staticmethod
    def backward(ctx, grad_output):
        if grad_output is None:
            return None, None, None, None
        scale = float(np.asarray(
            grad_output.detach().cpu().numpy(), dtype=np.float64))
        grad_u = torch.from_numpy(scale * ctx.grad_u)
        grad_e = torch.from_numpy(
            (scale * ctx.grad_e).reshape(ctx.elastic_shape))
        grad_p = torch.from_numpy(
            (scale * ctx.grad_p).reshape(ctx.plastic_shape))
        return grad_u, grad_e, grad_p, None


class _ForceFunction(torch.autograd.Function):
    """Differentiable force evaluation ``R(u; m) = grad_u E(u; m)``.

    Forward returns the full force vector; backward uses the displacement
    Hessian (sparse matvec) and the material VJP -- the mixed second
    derivatives -- so the force output stays connected to the material
    parameters in the autograd graph.
    """

    @staticmethod
    def forward(ctx, displacement, elastic_values, plastic_values, layer):
        displacement_np, elastic_np, plastic_np, energy = (
            _build_energy_evaluation(
                displacement, elastic_values, plastic_values, layer))
        ctx.energy = energy
        ctx.displacement = displacement_np
        ctx.elastic_shape = elastic_np.shape
        ctx.plastic_shape = plastic_np.shape
        gradient = energy.gradient(displacement_np)
        ctx.gradient_shape = gradient.shape
        return torch.from_numpy(gradient)

    @staticmethod
    def backward(ctx, grad_output):
        if grad_output is None:
            return None, None, None, None
        adjoint = np.asarray(
            grad_output.detach().cpu().numpy(), dtype=np.float64)
        if adjoint.shape != ctx.gradient_shape:
            raise ValueError(
                "grad_output for force output must have shape "
                f"{ctx.gradient_shape}, got {adjoint.shape}")
        hessian = ctx.energy.hessian(ctx.displacement)
        grad_u = hessian @ adjoint
        grad_e = ctx.energy.elastic_material_vjp(
            ctx.displacement, adjoint)
        grad_p = ctx.energy.plastic_material_vjp(
            ctx.displacement, adjoint)
        return (
            torch.from_numpy(grad_u),
            torch.from_numpy(grad_e.reshape(ctx.elastic_shape)),
            torch.from_numpy(grad_p.reshape(ctx.plastic_shape)),
            None,
        )


class _StaticEnergyModule(torch.nn.Module):
    """Shared base for the energy/force layers (internal)."""

    def __init__(self, *, energy_operator: DeformationEnergyOperator) -> None:
        super().__init__()
        if not isinstance(energy_operator, DeformationEnergyOperator):
            raise TypeError(
                "energy_operator must be a pypgo.fem.DeformationEnergyOperator")
        self.energy_operator = energy_operator

    @property
    def num_dofs(self) -> int:
        return self.energy_operator.num_dofs


class StaticEnergyLayer(_StaticEnergyModule):
    """Differentiable energy evaluation ``E(u; m)``.

    Unlike :class:`StaticEquilibriumLayer`, this layer does not solve any
    equilibrium: it evaluates the energy of the operator at a *given*
    displacement and exposes exact first derivatives through autograd:
    ``dE/du`` is the C++ force gradient, ``dE/de`` and ``dE/dp`` are the
    material-channel derivatives.  Use :class:`StaticForceLayer` when you need
    the force vector itself to stay differentiable with respect to the
    material parameters (e.g. force matching in direct constitutive
    calibration).

    Boundary conditions are not part of this layer: ``displacement`` is the
    full state vector and the caller decides which entries are fixed.

    Only CPU float64 tensors are supported, and only first-order gradients:
    double backward is not implemented.
    """

    def __init__(
        self,
        *,
        energy_operator: DeformationEnergyOperator,
    ) -> None:
        super().__init__(energy_operator=energy_operator)

    def forward(self, displacement, elastic_values, plastic_values):
        """Evaluate the deformation energy at a prescribed displacement.

        Parameters
        ----------
        displacement : torch.Tensor float64 cpu, shape (num_dofs,)
            Full displacement state (fixed DOFs are the caller's concern).
        elastic_values : torch.Tensor float64 cpu, shape (num_elements, C_e)
            Element-major optimizable elastic channels.
        plastic_values : torch.Tensor float64 cpu, shape (num_elements, C_p)
            Element-major optimizable plastic channels; use shape
            (num_elements, 0) when the plastic model has no channels.

        Returns
        -------
        torch.Tensor (0-d)
            The total deformation energy.
        """
        return _EnergyFunction.apply(
            displacement, elastic_values, plastic_values, self)


class StaticForceLayer(_StaticEnergyModule):
    """Differentiable force evaluation ``R(u; m) = grad_u E(u; m)``.

    Unlike :class:`StaticEnergyLayer`, the output is the full force vector
    (the gradient of the energy with respect to the displacement).  Its
    backward pass uses the displacement Hessian (sparse matvec) and the
    material VJP -- i.e. the mixed second derivatives -- so the force output
    remains connected to the material parameters in the autograd graph.  This
    is what makes force matching in direct constitutive calibration work.

    Boundary conditions are not part of this layer: ``displacement`` is the
    full state vector and the caller decides which entries are fixed.

    Only CPU float64 tensors are supported, and only first-order gradients:
    double backward is not implemented.
    """

    def __init__(
        self,
        *,
        energy_operator: DeformationEnergyOperator,
    ) -> None:
        super().__init__(energy_operator=energy_operator)

    def forward(self, displacement, elastic_values, plastic_values):
        """Evaluate the internal force vector at a prescribed displacement.

        Parameters
        ----------
        displacement : torch.Tensor float64 cpu, shape (num_dofs,)
            Full displacement state (fixed DOFs are the caller's concern).
        elastic_values : torch.Tensor float64 cpu, shape (num_elements, C_e)
            Element-major optimizable elastic channels.
        plastic_values : torch.Tensor float64 cpu, shape (num_elements, C_p)
            Element-major optimizable plastic channels; use shape
            (num_elements, 0) when the plastic model has no channels.

        Returns
        -------
        torch.Tensor, shape (num_dofs,)
            The full force vector ``R = grad E(u; m)``.
        """
        return _ForceFunction.apply(
            displacement, elastic_values, plastic_values, self)


class StaticEquilibriumLayer(torch.nn.Module):
    """Differentiable static equilibrium solve for one load case.

    The layer owns the immutable physics (operator, boundary conditions,
    inner optimizer, sparse backend) and the autograd function implements the
    implicit-function-theorem backward.  It has no trainable parameters; the
    caller owns the material values and passes them to ``forward``.

    Parameters
    ----------
    energy_operator : DeformationEnergyOperator
        The immutable FEM operator.  Build it once and share it across all
        load cases; each forward binds a fresh ``MaterialState`` to it.
    fixed_dofs, fixed_values : array-like
        Prescribed displacement DOFs and values for this load case.
    reaction_selectors : array-like or None, optional
        If None, ``forward`` returns the equilibrium displacement ``u*``.
        Otherwise it must be a vector (num_dofs,) or matrix (k, num_dofs) and
        ``forward`` returns the total reaction ``S @ grad E(u*)`` (scalar for a
        vector selector, (k,) for a matrix).
    inner_optimizer : pypgo.solver.Optimizer, optional
        Newton optimizer used for the forward equilibrium solve.  Defaults to
        ``NewtonOptimizer(max_iterations=200, AbsoluteTermination(1e-10))``.
    sparse_backend : pypgo.solver SparseSolver handle, optional
        Backend for the backward adjoint solve.  Defaults to ``Auto``.
    initial_displacement : array-like, optional
        Fixed analytic initial guess for the Newton solve (no warm start).
        Defaults to the operator rest state.
    residual_tol : float or None, optional
        Maximum allowed free residual after the solve; a larger residual
        raises ``RuntimeError``.  ``None`` disables the check.
    """

    def __init__(
        self,
        *,
        energy_operator: DeformationEnergyOperator,
        fixed_dofs,
        fixed_values,
        reaction_selectors=None,
        inner_optimizer: _solver.Optimizer | None = None,
        sparse_backend=None,
        initial_displacement=None,
        residual_tol: float | None = 1.0e-6,
    ) -> None:
        super().__init__()
        if not isinstance(energy_operator, DeformationEnergyOperator):
            raise TypeError(
                "energy_operator must be a pypgo.fem.DeformationEnergyOperator")
        self.energy_operator = energy_operator

        fixed_dofs = int_vector("fixed_dofs", fixed_dofs)
        fixed_values = float_vector("fixed_values", fixed_values)
        if fixed_dofs.size != fixed_values.size:
            raise ValueError("fixed_values size must match fixed_dofs size")
        if len(set(fixed_dofs.tolist())) != fixed_dofs.size:
            raise ValueError("fixed_dofs must be unique")
        if np.any(fixed_dofs < 0) or np.any(fixed_dofs >= energy_operator.num_dofs):
            raise ValueError(
                f"fixed_dofs contain out-of-range indices for {energy_operator.num_dofs} DOFs")
        self.fixed_dofs = fixed_dofs
        self.fixed_values = fixed_values
        self.free_dofs = np.setdiff1d(
            np.arange(energy_operator.num_dofs, dtype=np.int64), fixed_dofs)
        self._fixed_row_mask = np.zeros(energy_operator.num_dofs, dtype=bool)
        self._fixed_row_mask[fixed_dofs] = True

        self._single_reaction = False
        if reaction_selectors is None:
            self.reaction_selectors = None
        else:
            selectors = np.asarray(reaction_selectors, dtype=np.float64)
            if selectors.ndim == 1:
                self._single_reaction = True
                selectors = selectors.reshape(1, -1)
            if selectors.ndim != 2 or selectors.shape[1] != energy_operator.num_dofs:
                raise ValueError(
                    "reaction_selectors must be a vector or matrix with "
                    f"{energy_operator.num_dofs} columns")
            self.reaction_selectors = selectors

        if inner_optimizer is None:
            inner_optimizer = _solver.NewtonOptimizer(
                max_iterations=200,
                termination=_solver.AbsoluteTermination(abs_tolerance=1.0e-10))
        if not isinstance(inner_optimizer, _solver.Optimizer):
            raise TypeError(
                "inner_optimizer must be a pypgo.solver.Optimizer")
        self.inner_optimizer = inner_optimizer

        if sparse_backend is None:
            sparse_backend = _solver.Auto()
        if not isinstance(sparse_backend, _core.PySparseSolver):
            raise TypeError(
                "sparse_backend must be a pypgo.solver SparseSolver handle "
                "(e.g. pypgo.solver.EigenLDLT())")
        self.sparse_backend = sparse_backend

        if initial_displacement is None:
            initial_displacement = energy_operator.rest_state
        initial_displacement = float_vector(
            "initial_displacement", initial_displacement)
        if initial_displacement.size != energy_operator.num_dofs:
            raise ValueError(
                "initial_displacement size must match operator num_dofs "
                f"({energy_operator.num_dofs})")
        self.initial_displacement = initial_displacement

        if residual_tol is not None and residual_tol < 0.0:
            raise ValueError("residual_tol must be non-negative or None")
        self.residual_tol = residual_tol

        self._last_displacement: np.ndarray | None = None
        self._last_result = None

    @property
    def num_dofs(self) -> int:
        return self.energy_operator.num_dofs

    def forward(self, elastic_values, plastic_values):
        """Run the differentiable equilibrium solve.

        Parameters
        ----------
        elastic_values : torch.Tensor float64 cpu, shape (num_elements, C_e)
            Element-major optimizable elastic channels.
        plastic_values : torch.Tensor float64 cpu, shape (num_elements, C_p)
            Element-major optimizable plastic channels; use shape
            (num_elements, 0) when the plastic model has no channels.  When
            channels exist (e.g. ``VolumetricPlasticityDefinition(dofs=3/6)``),
            initialize them at the model's rest state (identity strain
            components: all-ones diagonal, zero off-diagonal) or the forward
            solve may fail on nonphysical configurations.

        Returns
        -------
        torch.Tensor
            Equilibrium displacement ``u*``, or the reaction readout when
            ``reaction_selectors`` was provided.
        """
        return _StaticEquilibriumFunction.apply(
            elastic_values, plastic_values, self)
