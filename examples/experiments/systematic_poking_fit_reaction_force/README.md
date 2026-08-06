# Systematic Poking Calibration

Fitting the Systematic Poking principal-stretch material to synthetic
Neo-Hookean data, end to end in PyTorch.

This folder contains two reference notebooks. Both calibrate the same
`f''(knot)` spline values plus volumetric coefficient against a Neo-Hookean
target, but they use different kinds of data and different torch layers:

| Notebook | Data | Layer | Differentiability |
| --- | --- | --- | --- |
| [`static_solve_fitting.ipynb`](static_solve_fitting.ipynb) | equilibrium reaction forces (displacement is solved) | `StaticEquilibriumLayer` | implicit-function-theorem adjoint through the Newton solve |
| [`direct_fitting.ipynb`](direct_fitting.ipynb) | prescribed deformation states and their internal forces (displacement is data) | `StaticForceLayer` | exact `grad E` forward and Hessian/mixed-derivative backward, no Newton |

The notebooks are self-contained: all mesh, load-case, and fitting code lives
in the notebooks themselves. There are no experiment scripts or generated
artifacts in this folder.

## What each notebook shows

**`static_solve_fitting.ipynb`**

1. Builds a 2×2×2 hexahedral mesh and one shared `DeformationEnergyOperator`;
2. Wraps each of 36 load cases (free/confined uniaxial + simple shear) in a
   `StaticEquilibriumLayer`;
3. Generates target reactions with the Neo-Hookean material;
4. Defines a scalar loss (normalized reaction mismatch plus a smoothness
   penalty) and calls `loss.backward()` -- the layer's backward implements
   the implicit-function-theorem adjoint `K_ff^T λ = (K^T c)_f` internally;
5. Fits the 18 parameters in log space with `torch.optim.Adam`;
6. Visualizes the solved configurations with `pgo.visualize` and plots the
   reaction curves.

**`direct_fitting.ipynb`**

1. Prescribes homogeneous deformation states (uniaxial at every non-rest
   knot, plus four shears) -- all displacement DOFs are data;
2. Evaluates full force vectors `R = grad E(u; m)` with `StaticForceLayer`;
3. Fits in log space with `torch.optim.Adam`; no equilibrium solve anywhere
   in the loop;
4. Runs a predictive check: the directly calibrated material is plugged into
   `StaticEquilibriumLayer` and its equilibrium reaction curves are compared
   with the target.

## The torch layers

All three layers live in `pypgo.fem.torch` (lazy-imported; requires the
optional `pypgo[torch]` extra):

- `StaticEnergyLayer` -- `E(u; m)` with exact first derivatives;
- `StaticForceLayer` -- `grad E(u; m)` with Hessian-matvec and material-VJP
  backward, so forces stay connected to the material parameters;
- `StaticEquilibriumLayer` -- the static equilibrium solve `u*(m)` with an
  implicit-function-theorem backward.

They accept CPU float64 tensors only: material values are element-major
`(num_elements, channels)` tensors (broadcast global parameters with
`torch.exp(theta).expand(num_elements, -1)`), and the plastic argument is
always required (`(num_elements, 0)` when the model has no plastic channels).

## Running the notebooks

```bash
pip install -e ".[torch,viz,dev]"   # viz only needed for pgo.visualize
jupyter notebook static_solve_fitting.ipynb
```

Both notebooks fall back gracefully when matplotlib or pyvista is not
installed (plots are skipped with a message).

## Regression coverage

The key invariants are pinned by the self-contained test
`tests/pypgo/test_systematic_poking_reaction_force_fit_demo.py`: load-case
constraints, the implicit reaction Jacobian against central finite
differences (rtol 5e-7), and Gauss-Newton convergence.
