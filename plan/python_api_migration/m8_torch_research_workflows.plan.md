# M8 PyTorch Research Workflows and Neural Plastic Field Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make libpgo energies usable inside PyTorch research loops, then extend the same boundary to neural `F_p` plastic field optimization.

**Architecture:** M8 keeps NumPy and C++ as the numerical base and adds a lazy optional `pypgo.torch` adapter on top. The required baseline exposes energy-level autograd with respect to solver state `u`; neural plastic field optimization is a follow-on research path that requires explicit plastic-field inputs and `dE/dplastic` gradients from the deformation energy core.

**Tech Stack:** C++ deformation energy core, nanobind, NumPy CPU arrays, PyTorch `torch.autograd.Function`, pytest, generated notebooks under `pypgo/examples`.

---

## Motivation

M8 should demonstrate that libpgo is useful in ML and research workflows without turning the full simulator into a differentiable solver. The most important first capability is:

```python
loss = data_loss + weight * pgo.torch.energy_value(energy, u)
loss.backward()
```

This lets a PyTorch model predict a deformation state while libpgo supplies a physically meaningful energy term. The C++ energy computes:

```text
E(u)
dE/du
```

That is enough for displacement-field regularization, learned initial guesses, deformation priors, and observation fitting where the learned output is `u`.

Neural plastic field optimization is the next layer:

```python
plastic = fp_net(element_features)
loss = pgo.torch.energy_value(energy, u, plastic=plastic)
loss.backward()
```

This requires the C++ energy to compute both:

```text
dE/du
dE/dplastic
```

Without `dE/dplastic`, PyTorch can backpropagate to `u` but cannot update the neural network that produced `F_p`.

## Research Outcome: What Neural `F_p` Enables

Optimizing an `F_p` field means optimizing a material's permanent deformation memory, not merely the current vertex displacement. A concrete demo should be:

```text
Rest beam:
[===========]

Target released shape:
   (========)
```

The network represents a smooth per-element plastic field:

```python
fp_net(X_element) -> F_p(element)
```

The optimizer adjusts `fp_net` so that, after the material stores this plastic field, the object naturally prefers a target curved or sculpted shape. This is useful for inverse plastic design, plastic sculpting, learned manufacturing prestrain, and soft-robot body design.

The first neural `F_p` example should use a small tetrahedral or cubic beam instead of a complex character mesh. The target is easier to inspect, runs quickly, and clearly shows the difference between elastic displacement and permanent plastic memory.

## Scope Boundaries

### In M8 Baseline

- `pypgo.torch.energy_value(energy, u)` for CPU `torch.float64` tensors.
- PyTorch backward calls C++ `energy.gradient(u)` and returns `dE/du`.
- `pypgo.torch.EnergyModule` convenience wrapper.
- `SparseMatrix.to_torch_sparse_coo()`.
- Example: a neural network predicts displacement `u`, and libpgo energy acts as a physical regularizer.
- Lazy optional import: `import pypgo` must not import PyTorch.

### M8.5 Research Extension

- `energy.value(u, plastic=plastic)` accepts explicit plastic state.
- `energy.value_and_gradients(u, plastic=plastic)` returns value, `dE/du`, and `dE/dplastic`.
- `pypgo.torch.energy_value(energy, u, plastic=plastic)` backpropagates to both `u` and `plastic`.
- Example: a neural network predicts `F_p` or a compact plastic parameter field that makes a beam settle into a target shape.

### Out Of Scope

- Differentiating through Newton iterations or full dynamic simulation.
- CUDA tensor support at the `_core` boundary.
- Element-wise Python callbacks inside C++ inner loops.
- PyTorch replacing NumPy as the base public data protocol.
- Automatically differentiating arbitrary material parameters unless C++ explicitly returns their gradients.

## Target Public API

### Baseline Energy Adapter

```python
import torch
import pypgo as pgo
import pypgo.torch as pgo_torch

energy = pgo.energy.deformation_energy(
    sim_mesh,
    formulation=pgo.fem.TetP1(),
    elastic=pgo.energy.StableNeo(),
    plastic="volumetric_dof6",
)

u = torch.zeros(energy.num_dofs, dtype=torch.float64, requires_grad=True)
value = pgo_torch.energy_value(energy, u)
value.backward()

assert u.grad.shape == u.shape
```

### Convenience Module

```python
regularizer = pgo_torch.EnergyModule(energy, weight=1e-3)

u = displacement_net(rest_positions).reshape(-1)
loss = observation_loss(u) + regularizer(u)
loss.backward()
```

### Neural Plastic Field Extension

```python
element_features = torch.from_numpy(energy.element_centers).double()
plastic = fp_net(element_features)  # compact field or flattened per-element F_p
u = torch.zeros(energy.num_dofs, dtype=torch.float64, requires_grad=True)

loss = pgo_torch.energy_value(energy, u, plastic=plastic)
loss.backward()

for param in fp_net.parameters():
    assert param.grad is not None
```

The public plastic tensor shape must be model-specific but explicit. For a full deformation-gradient field, use:

```text
plastic.shape == (num_elements, 3, 3)
```

For existing volumetric 6-DOF plastic models, prefer:

```text
plastic.shape == (num_elements, 6)
```

Do not silently infer the shape from tensor rank. The Python wrapper should read `energy.plastic_layout` and validate the input against it.

## File Structure

### M8 Baseline Files

- Create: `pypgo/torch.py`
  - Lazy imports PyTorch.
  - Defines `energy_value`, `EnergyModule`, and the private autograd function.
- Modify: `pypgo/__init__.py`
  - Adds `"torch"` to `__all__` while preserving lazy import semantics.
- Modify: `pypgo/sparse.py`
  - Adds `SparseMatrix.to_torch_sparse_coo(device=None)`.
- Test: `tests/pypgo/test_torch_energy.py`
  - Covers lazy import, forward value, backward gradient, dtype/device errors.
- Test: `tests/pypgo/test_sparse_torch.py`
  - Covers sparse COO conversion and lazy missing-PyTorch errors.
- Create: `pypgo/examples/scripts/generate_torch_energy_regularizer.py`
  - Generates the displacement regularizer notebook.
- Create: `pypgo/examples/torch_energy_regularizer.ipynb`
  - Shows PyTorch network output `u` regularized by libpgo energy.

### M8.5 Neural Plastic Field Files

- Modify: `src/core/solidDeformationModel/deformationModelEnergy.h`
  - Adds a C++ service boundary for explicit plastic inputs and plastic gradients.
- Modify: `src/core/solidDeformationModel/deformationModelEnergy.cpp`
  - Implements value and gradient evaluation with external plastic state.
- Modify: `src/python/pypgo/bindings/energy_bindings.cpp`
  - Exposes `value_with_plastic`, `gradient_with_plastic`, and `value_and_gradients`.
- Modify: `pypgo/energy.py`
  - Adds Python validation for `plastic` shape, dtype, and layout.
- Modify: `pypgo/torch.py`
  - Extends `energy_value` to accept `plastic=None`.
- Test: `tests/pypgo/test_plastic_field_energy.py`
  - Covers NumPy-level `value_and_gradients`.
- Test: `tests/pypgo/test_torch_plastic_field.py`
  - Covers PyTorch backward into neural plastic parameters.
- Create: `pypgo/examples/scripts/generate_neural_plastic_field.py`
  - Generates the neural plastic field notebook.
- Create: `pypgo/examples/neural_plastic_field.ipynb`
  - Demonstrates the beam-to-target-shape inverse plastic design example.

## Implementation Tasks

### Task 1: Add The Baseline Torch Adapter

**Files:**
- Create: `pypgo/torch.py`
- Modify: `pypgo/__init__.py`
- Test: `tests/pypgo/test_torch_energy.py`

- [ ] **Step 1: Write the failing lazy-import test**

```python
def test_import_pypgo_does_not_import_torch(monkeypatch):
    import importlib
    import sys

    sys.modules.pop("pypgo", None)
    sys.modules.pop("torch", None)

    module = importlib.import_module("pypgo")

    assert module is not None
    assert "torch" not in sys.modules
```

- [ ] **Step 2: Write the failing backward test using a tiny fake energy**

```python
class QuadraticEnergy:
    num_dofs = 3

    def value(self, u):
        import numpy as np
        arr = np.asarray(u, dtype=np.float64)
        return float(0.5 * arr.dot(arr))

    def gradient(self, u):
        import numpy as np
        return np.asarray(u, dtype=np.float64).copy()


def test_energy_value_backward_matches_energy_gradient():
    import torch
    import pypgo.torch as pgo_torch

    energy = QuadraticEnergy()
    u = torch.tensor([1.0, -2.0, 3.0], dtype=torch.float64, requires_grad=True)

    value = pgo_torch.energy_value(energy, u)
    value.backward()

    assert value.dtype == torch.float64
    torch.testing.assert_close(u.grad, torch.tensor([1.0, -2.0, 3.0], dtype=torch.float64))
```

- [ ] **Step 3: Run the failing tests**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_torch_energy.py
```

Expected:

```text
FAIL tests/pypgo/test_torch_energy.py::test_energy_value_backward_matches_energy_gradient
```

The failure should be caused by missing `pypgo.torch` or missing `energy_value`.

- [ ] **Step 4: Implement `pypgo/torch.py`**

```python
"""Optional PyTorch adapters for pypgo."""

from __future__ import annotations

import numpy as np


def _torch():
    try:
        import torch
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "pypgo.torch requires PyTorch. Install torch to use pypgo's PyTorch adapters."
        ) from exc
    return torch


def _cpu_float64_numpy(name: str, tensor):
    torch = _torch()
    if not isinstance(tensor, torch.Tensor):
        raise TypeError(f"{name} must be a torch.Tensor, got {type(tensor).__name__}")
    if tensor.device.type != "cpu":
        raise ValueError(f"{name} must be a CPU tensor; got device {tensor.device}")
    if tensor.dtype != torch.float64:
        raise TypeError(f"{name} must have dtype torch.float64, got {tensor.dtype}")
    return np.ascontiguousarray(tensor.detach().numpy(), dtype=np.float64)


class _EnergyValueFunction:
    @staticmethod
    def apply(energy, u):
        torch = _torch()

        class _Fn(torch.autograd.Function):
            @staticmethod
            def forward(ctx, u_tensor):
                u_np = _cpu_float64_numpy("u", u_tensor)
                value = float(energy.value(u_np))
                grad = np.ascontiguousarray(energy.gradient(u_np), dtype=np.float64)
                if grad.shape != u_np.shape:
                    raise ValueError(f"energy.gradient(u) returned shape {grad.shape}, expected {u_np.shape}")
                ctx.save_for_backward(torch.from_numpy(grad).to(device=u_tensor.device))
                return u_tensor.new_tensor(value)

            @staticmethod
            def backward(ctx, grad_output):
                (grad_u,) = ctx.saved_tensors
                return grad_output * grad_u

        return _Fn.apply(u)


def energy_value(energy, u):
    """Return a differentiable scalar PyTorch tensor for ``energy.value(u)``."""
    return _EnergyValueFunction.apply(energy, u)


def EnergyModule(energy, weight: float = 1.0):
    """Return a small ``torch.nn.Module`` wrapper around ``energy_value``."""
    torch = _torch()

    class _EnergyModule(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.energy = energy
            self.weight = float(weight)

        def forward(self, u):
            return self.weight * energy_value(self.energy, u)

    return _EnergyModule()
```

- [ ] **Step 5: Add lazy module export**

Modify `pypgo/__init__.py`:

```python
__all__ = ["mesh", "sim", "sparse", "tools", "torch"]
```

Keep the existing `__getattr__` lazy import path unchanged.

- [ ] **Step 6: Run the tests**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_torch_energy.py
```

Expected:

```text
2 passed
```

- [ ] **Step 7: Commit the baseline adapter**

```bash
git add pypgo/__init__.py pypgo/torch.py tests/pypgo/test_torch_energy.py
git commit -m "feat: add torch energy adapter"
```

### Task 2: Add Torch Sparse COO Conversion

**Files:**
- Modify: `pypgo/sparse.py`
- Test: `tests/pypgo/test_sparse_torch.py`

- [ ] **Step 1: Write the failing sparse conversion test**

```python
def test_sparse_matrix_to_torch_sparse_coo():
    import torch
    import pypgo as pgo

    core = pgo._core.create_sparse_matrix(
        3,
        4,
        [0, 2],
        [1, 3],
        [1.5, -2.0],
    )
    sparse = pgo.sparse.SparseMatrix(core)

    tensor = sparse.to_torch_sparse_coo()

    assert tensor.shape == (3, 4)
    assert tensor.dtype == torch.float64
    assert tensor.layout == torch.sparse_coo
    torch.testing.assert_close(tensor.coalesce().values(), torch.tensor([1.5, -2.0], dtype=torch.float64))
```

- [ ] **Step 2: Run the failing test**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_sparse_torch.py
```

Expected:

```text
FAIL tests/pypgo/test_sparse_torch.py::test_sparse_matrix_to_torch_sparse_coo
```

The failure should mention missing `to_torch_sparse_coo`.

- [ ] **Step 3: Implement `to_torch_sparse_coo`**

Add this method to `pypgo/sparse.py`:

```python
    def to_torch_sparse_coo(self, device=None):
        try:
            import torch
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(
                "SparseMatrix.to_torch_sparse_coo requires PyTorch. Install torch to use this adapter."
            ) from exc

        rows, cols, values = self.to_coo()
        indices_np = np.vstack([rows, cols]).astype(np.int64, copy=False)
        indices = torch.as_tensor(indices_np, dtype=torch.int64, device=device)
        tensor_values = torch.as_tensor(values, dtype=torch.float64, device=device)
        return torch.sparse_coo_tensor(indices, tensor_values, size=self.shape, device=device).coalesce()
```

- [ ] **Step 4: Run the sparse test**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_sparse_torch.py
```

Expected:

```text
1 passed
```

- [ ] **Step 5: Commit sparse conversion**

```bash
git add pypgo/sparse.py tests/pypgo/test_sparse_torch.py
git commit -m "feat: convert sparse matrices to torch COO"
```

### Task 3: Add The Displacement-Regularized Research Example

**Files:**
- Create: `pypgo/examples/scripts/generate_torch_energy_regularizer.py`
- Create: `pypgo/examples/torch_energy_regularizer.ipynb`
- Modify: `tests/pypgo/test_example_notebooks.py`

- [ ] **Step 1: Write a notebook generator smoke test**

Add to `tests/pypgo/test_example_notebooks.py`:

```python
def load_torch_energy_generator():
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        spec = importlib.util.spec_from_file_location(
            "generate_torch_energy_regularizer_test",
            SCRIPT_DIR / "generate_torch_energy_regularizer.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(SCRIPT_DIR))
    return module


def test_torch_energy_regularizer_notebook_shows_energy_inside_loss():
    module = load_torch_energy_generator()
    source = "\n".join(cell.source for cell in module.CELLS)

    assert "pgo_torch.energy_value" in source
    assert "loss = observation_loss + physics_weight * elastic_loss" in source
    assert "loss.backward()" in source
    assert "CPU tensors" in source
    assert "solver-level differentiable simulation" in source
```

- [ ] **Step 2: Run the failing test**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_example_notebooks.py::test_torch_energy_regularizer_notebook_shows_energy_inside_loss
```

Expected:

```text
FAIL tests/pypgo/test_example_notebooks.py::test_torch_energy_regularizer_notebook_shows_energy_inside_loss
```

The failure should be caused by the missing generator script.

- [ ] **Step 3: Create the generator**

The generator should follow `pypgo/examples/scripts/generate_numpy_interoperate.py` and define `CELLS`. Include this core training cell:

```python
import torch
import pypgo as pgo
import pypgo.torch as pgo_torch

rest = torch.from_numpy(energy.rest_position.reshape(-1, 3)).double()

model = torch.nn.Sequential(
    torch.nn.Linear(3, 64),
    torch.nn.Tanh(),
    torch.nn.Linear(64, 3),
).double()

optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
physics_weight = 1e-3

for step in range(100):
    optimizer.zero_grad()
    u_vertices = model(rest)
    u = u_vertices.reshape(-1)

    observation_loss = torch.nn.functional.mse_loss(u_vertices[handle_ids], target_displacements)
    elastic_loss = pgo_torch.energy_value(energy, u)
    loss = observation_loss + physics_weight * elastic_loss

    loss.backward()
    optimizer.step()
```

The notebook text must state that this example uses CPU tensors and energy-level autograd, not solver-level differentiable simulation.

- [ ] **Step 4: Generate the notebook**

Run:

```bash
conda run -n libpgo python pypgo/examples/scripts/generate_torch_energy_regularizer.py
```

Expected:

```text
pypgo/examples/torch_energy_regularizer.ipynb
```

The file should be created or overwritten with deterministic cell ids.

- [ ] **Step 5: Run the notebook generator test**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_example_notebooks.py::test_torch_energy_regularizer_notebook_shows_energy_inside_loss
```

Expected:

```text
1 passed
```

- [ ] **Step 6: Commit the example**

```bash
git add pypgo/examples/scripts/generate_torch_energy_regularizer.py pypgo/examples/torch_energy_regularizer.ipynb tests/pypgo/test_example_notebooks.py
git commit -m "docs: add torch energy regularizer example"
```

### Task 4: Add Explicit Plastic Field Layout To The Python Energy API

**Files:**
- Modify: `pypgo/energy.py`
- Test: `tests/pypgo/test_plastic_field_energy.py`

- [ ] **Step 1: Write the failing layout validation test**

```python
def test_plastic_layout_rejects_wrong_shape(tiny_deformation_energy):
    import numpy as np
    import pytest

    energy = tiny_deformation_energy
    plastic = np.zeros((energy.num_elements, 5), dtype=np.float64)

    with pytest.raises(ValueError, match="plastic"):
        energy.value(energy.zero_state(), plastic=plastic)
```

- [ ] **Step 2: Define the public layout object**

Add this dataclass to `pypgo/energy.py`:

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class PlasticLayout:
    name: str
    shape_per_element: tuple[int, ...]

    def full_shape(self, num_elements: int) -> tuple[int, ...]:
        return (int(num_elements), *self.shape_per_element)
```

- [ ] **Step 3: Add explicit normalization**

Add this helper to `pypgo/energy.py`:

```python
def _normalize_plastic(name: str, plastic, layout: PlasticLayout, num_elements: int):
    import numpy as np

    arr = np.ascontiguousarray(plastic, dtype=np.float64)
    expected = layout.full_shape(num_elements)
    if arr.shape != expected:
        raise ValueError(f"{name} must have shape {expected} for plastic layout {layout.name!r}, got {arr.shape}")
    return arr
```

- [ ] **Step 4: Wire validation into `DeformationEnergy.value`**

The wrapper should call:

```python
if plastic is None:
    return self._core_obj.value(u_arr)

plastic_arr = _normalize_plastic("plastic", plastic, self.plastic_layout, self.num_elements)
return self._core_obj.value_with_plastic(u_arr, plastic_arr.reshape(-1))
```

- [ ] **Step 5: Run the validation test**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_plastic_field_energy.py
```

Expected:

```text
1 passed
```

- [ ] **Step 6: Commit the plastic layout API**

```bash
git add pypgo/energy.py tests/pypgo/test_plastic_field_energy.py
git commit -m "feat: validate explicit plastic field inputs"
```

### Task 5: Expose C++ Gradients With Respect To Plastic State

**Files:**
- Modify: `src/core/solidDeformationModel/deformationModelEnergy.h`
- Modify: `src/core/solidDeformationModel/deformationModelEnergy.cpp`
- Modify: `src/python/pypgo/bindings/energy_bindings.cpp`
- Test: `tests/pypgo/test_plastic_field_energy.py`

- [ ] **Step 1: Add a failing NumPy gradient test**

```python
def test_value_and_gradients_returns_state_and_plastic_grads(tiny_deformation_energy):
    import numpy as np

    energy = tiny_deformation_energy
    u = energy.zero_state()
    plastic = np.zeros(energy.plastic_layout.full_shape(energy.num_elements), dtype=np.float64)

    value, grad_u, grad_plastic = energy.value_and_gradients(u, plastic=plastic)

    assert isinstance(value, float)
    assert grad_u.shape == u.shape
    assert grad_plastic.shape == plastic.shape
    assert np.all(np.isfinite(grad_u))
    assert np.all(np.isfinite(grad_plastic))
```

- [ ] **Step 2: Add the C++ service signatures**

Add methods equivalent to:

```cpp
double valueWithPlastic(EigenSupport::ConstRefVecXd u, EigenSupport::ConstRefVecXd plastic) const;
Eigen::VectorXd gradientWithPlastic(EigenSupport::ConstRefVecXd u, EigenSupport::ConstRefVecXd plastic) const;
Eigen::VectorXd plasticGradient(EigenSupport::ConstRefVecXd u, EigenSupport::ConstRefVecXd plastic) const;
```

The implementation must evaluate the same energy model as `value(u)` while using the caller-provided plastic state for the current evaluation.

- [ ] **Step 3: Bind the methods**

Expose nanobind methods named:

```python
value_with_plastic(u, plastic_flat)
gradient_with_plastic(u, plastic_flat)
plastic_gradient(u, plastic_flat)
```

Each binding should accept contiguous `float64` NumPy arrays and release the GIL around C++ computation.

- [ ] **Step 4: Add Python wrapper method**

Add to `DeformationEnergy`:

```python
def value_and_gradients(self, u, *, plastic):
    u_arr = self._normalize_state(u)
    plastic_arr = _normalize_plastic("plastic", plastic, self.plastic_layout, self.num_elements)
    plastic_flat = plastic_arr.reshape(-1)
    value = self._core_obj.value_with_plastic(u_arr, plastic_flat)
    grad_u = self._core_obj.gradient_with_plastic(u_arr, plastic_flat)
    grad_plastic = self._core_obj.plastic_gradient(u_arr, plastic_flat).reshape(plastic_arr.shape)
    return float(value), grad_u, grad_plastic
```

- [ ] **Step 5: Run the plastic gradient tests**

Run:

```bash
conda run -n libpgo cmake --build --preset python-build -j 8
conda run -n libpgo python -m pytest -q tests/pypgo/test_plastic_field_energy.py
```

Expected:

```text
tests/pypgo/test_plastic_field_energy.py passed
```

- [ ] **Step 6: Commit plastic gradients**

```bash
git add src/core/solidDeformationModel/deformationModelEnergy.h src/core/solidDeformationModel/deformationModelEnergy.cpp src/python/pypgo/bindings/energy_bindings.cpp pypgo/energy.py tests/pypgo/test_plastic_field_energy.py
git commit -m "feat: expose deformation energy plastic gradients"
```

### Task 6: Extend Torch Autograd To Plastic Inputs

**Files:**
- Modify: `pypgo/torch.py`
- Test: `tests/pypgo/test_torch_plastic_field.py`

- [ ] **Step 1: Write the failing torch plastic backward test**

```python
def test_energy_value_backward_updates_plastic_network(tiny_deformation_energy):
    import torch
    import pypgo.torch as pgo_torch

    energy = tiny_deformation_energy
    features = torch.randn(energy.num_elements, 3, dtype=torch.float64)
    net = torch.nn.Linear(3, energy.plastic_layout.shape_per_element[0]).double()
    u = torch.zeros(energy.num_dofs, dtype=torch.float64, requires_grad=True)

    plastic = net(features)
    value = pgo_torch.energy_value(energy, u, plastic=plastic)
    value.backward()

    assert u.grad is not None
    assert net.weight.grad is not None
    assert torch.all(torch.isfinite(net.weight.grad))
```

- [ ] **Step 2: Update `energy_value` signature**

Change the public function to:

```python
def energy_value(energy, u, *, plastic=None):
    if plastic is None:
        return _StateEnergyValueFunction.apply(energy, u)
    return _StatePlasticEnergyValueFunction.apply(energy, u, plastic)
```

- [ ] **Step 3: Implement plastic-aware autograd**

The private function should:

```python
value, grad_u, grad_plastic = energy.value_and_gradients(u_np, plastic=plastic_np)
ctx.save_for_backward(
    torch.from_numpy(grad_u).to(device=u_tensor.device),
    torch.from_numpy(grad_plastic).to(device=plastic_tensor.device),
)
```

The backward method should return:

```python
return None, grad_output * grad_u, grad_output * grad_plastic
```

The wrapper must reject CUDA tensors with a clear CPU-only error.

- [ ] **Step 4: Run the torch plastic test**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_torch_plastic_field.py
```

Expected:

```text
1 passed
```

- [ ] **Step 5: Commit torch plastic autograd**

```bash
git add pypgo/torch.py tests/pypgo/test_torch_plastic_field.py
git commit -m "feat: backpropagate torch energy through plastic fields"
```

### Task 7: Add The Neural `F_p` Field Example

**Files:**
- Create: `pypgo/examples/scripts/generate_neural_plastic_field.py`
- Create: `pypgo/examples/neural_plastic_field.ipynb`
- Modify: `tests/pypgo/test_example_notebooks.py`

- [ ] **Step 1: Add a generator content test**

```python
def test_neural_plastic_field_notebook_explains_inverse_plastic_design():
    module = load_neural_plastic_field_generator()
    source = "\n".join(cell.source for cell in module.CELLS)

    assert "fp_net(element_features)" in source
    assert "pgo_torch.energy_value(energy, u, plastic=plastic)" in source
    assert "target released shape" in source
    assert "dE/dplastic" in source
    assert "not differentiating through Newton iterations" in source
```

- [ ] **Step 2: Create the generator**

Add this helper next to the other notebook loader helpers in `tests/pypgo/test_example_notebooks.py`:

```python
def load_neural_plastic_field_generator():
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        spec = importlib.util.spec_from_file_location(
            "generate_neural_plastic_field_test",
            SCRIPT_DIR / "generate_neural_plastic_field.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(SCRIPT_DIR))
    return module
```

The notebook should use a beam or box mesh and include this conceptual training loop:

```python
element_features = torch.from_numpy(energy.element_centers).double()
fp_net = torch.nn.Sequential(
    torch.nn.Linear(3, 64),
    torch.nn.Tanh(),
    torch.nn.Linear(64, energy.plastic_layout.shape_per_element[0]),
).double()

u = torch.zeros(energy.num_dofs, dtype=torch.float64, requires_grad=True)
optimizer = torch.optim.Adam(fp_net.parameters(), lr=1e-3)

for step in range(200):
    optimizer.zero_grad()
    plastic = fp_net(element_features)

    physics_loss = pgo_torch.energy_value(energy, u, plastic=plastic)
    shape_loss = target_shape_loss(u)
    smooth_loss = plastic_smoothness_loss(plastic, element_adjacency)
    det_loss = plastic_det_barrier(plastic)

    loss = shape_loss + 1e-3 * physics_loss + 1e-4 * smooth_loss + 1e-4 * det_loss
    loss.backward()
    optimizer.step()
```

The notebook should explain that a practical released-shape solve uses an outer optimization loop and a static solve, while this first example verifies the differentiable `F_p` loss path.

- [ ] **Step 3: Generate the notebook**

Run:

```bash
conda run -n libpgo python pypgo/examples/scripts/generate_neural_plastic_field.py
```

Expected:

```text
pypgo/examples/neural_plastic_field.ipynb
```

- [ ] **Step 4: Run the notebook test**

Run:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_example_notebooks.py::test_neural_plastic_field_notebook_explains_inverse_plastic_design
```

Expected:

```text
1 passed
```

- [ ] **Step 5: Commit the neural plastic field example**

```bash
git add pypgo/examples/scripts/generate_neural_plastic_field.py pypgo/examples/neural_plastic_field.ipynb tests/pypgo/test_example_notebooks.py
git commit -m "docs: add neural plastic field research example"
```

## Validation Commands

Run focused PyTorch tests:

```bash
conda run -n libpgo python -m pytest -q \
  tests/pypgo/test_torch_energy.py \
  tests/pypgo/test_sparse_torch.py \
  tests/pypgo/test_torch_plastic_field.py
```

Run plastic field API tests:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_plastic_field_energy.py
```

Run notebook generator checks:

```bash
conda run -n libpgo python -m pytest -q tests/pypgo/test_example_notebooks.py
```

Run the broad Python smoke suite after binding changes:

```bash
conda run -n libpgo cmake --build --preset python-build -j 8
conda run -n libpgo python -m pytest -q tests/pypgo
```

## Risk Assessment

| Risk | Impact | Mitigation |
|---|---:|---|
| Users think M8 differentiates through the solver | High | Notebook and doc text must say energy-level autograd only. |
| `F_p` optimization is attempted without `dE/dplastic` | High | Keep neural plastic field under M8.5 and require `value_and_gradients`. |
| CUDA tensors accidentally copy silently to CPU | Medium | First version rejects non-CPU tensors with a clear error. |
| Plastic layout is ambiguous | High | Expose `energy.plastic_layout` and validate exact shape. |
| PyTorch becomes a hard dependency | High | Import PyTorch only inside `pypgo.torch` and adapter methods. |
| Neural `F_p` outputs invalid deformation gradients | Medium | Examples include smoothness and determinant/volume regularization terms. |
| Example is too complex to run in CI | Medium | CI tests inspect notebook source; runtime examples use tiny meshes and short loops. |

## Done Criteria

- `import pypgo` does not import PyTorch.
- `pypgo.torch.energy_value(energy, u)` returns a scalar tensor and backpropagates `dE/du`.
- `SparseMatrix.to_torch_sparse_coo()` works when PyTorch is installed and raises a helpful lazy error otherwise.
- The displacement regularizer notebook shows a neural network using libpgo energy inside a PyTorch loss.
- Neural plastic field support is not marked complete until C++ returns `dE/dplastic`.
- The neural `F_p` notebook clearly frames the effect as inverse plastic design: optimizing a permanent plastic memory field so an object prefers a target shape.
