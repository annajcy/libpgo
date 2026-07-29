# Fit Systematic Poking to Neo-Hookean stress data

This demo performs a real material-parameter optimization through the public
`pypgo.fem` API. It generates synthetic first Piola stress observations from a
classic compressible Neo-Hookean material and fits the 18 positive parameters
of a 17-knot Systematic Poking material:

- 17 samples of the stretch curvature \(f''(s_k)\);
- one volumetric coefficient \(\lambda\), whose fixed spline shape samples
  \((\log J)^2/2\).

The target material uses \(E=2\times10^5\) and \(\nu=0.35\). Optimization starts
from the substantially different values \(E=1.2\times10^5\) and \(\nu=0.25\).

## Stress residual and analytic Jacobian

For a unit cube under an affine deformation \(F\), the nodal displacement is

\[
u_a=(F-I)X_a.
\]

Let \(B=\partial u/\partial\operatorname{vec}(F)\). Because the reference
volume is one, the FEM energy gradient gives

\[
\operatorname{vec}(P)=B^T\frac{\partial E}{\partial u}.
\]

The material parameter Jacobian comes from the mixed derivative already
provided by `DeformationEnergy`:

\[
\frac{\partial\operatorname{vec}(P)}{\partial e}
=B^T\frac{\partial^2E}{\partial u\,\partial e}.
\]

The optimizer uses \(e_i=E_{\mathrm{ref}}\exp(\theta_i)\), so all curvature
samples and \(\lambda\) remain strictly positive. Residuals are normalized by
\(E_{\mathrm{ref}}\), and the chain rule gives

\[
\frac{\partial(P/E_{\mathrm{ref}})}{\partial\theta_i}
=\frac{\partial P}{\partial e_i}\frac{e_i}{E_{\mathrm{ref}}}.
\]

A damped Gauss-Newton method with backtracking solves the resulting nonlinear
problem in log parameters. The physical stress model is linear in \(e\), so
the demo also computes an unconstrained linear least-squares solution as an
independent oracle.

## Experiment design

The training set includes uniaxial, biaxial, volumetric, isochoric, simple
shear, and randomly rotated distinct-stretch states. It also includes a
uniaxial state at every non-rest stretch knot from 0.5 to 2.0.

Those knot-aligned states are important. If the observed stretch range stays
strictly inside the spline domain, the two outer curvature samples do not
affect any observation. The design matrix then loses rank and those parameters
cannot be identified, regardless of optimizer quality.

The holdout set uses different deformation magnitudes and a different random
seed. It is never used by Gauss-Newton.

## Run

From the repository root:

```bash
PYTHONPATH=build/base/lib:$PYTHONPATH \
python examples/demo/optimization/systematic_poking_fit_neo_hookean/main.py
```

Useful options:

```bash
python examples/demo/optimization/systematic_poking_fit_neo_hookean/main.py \
  --knot-count 17 \
  --max-iterations 60 \
  --output-dir /tmp/systematic-poking-fit \
  --no-plots
```

The default deterministic experiment converges in about six iterations. In the
reference run, normalized component RMSE changed as follows:

| split | initial | fitted |
|---|---:|---:|
| training | \(1.8353\times10^{-1}\) | \(3.0649\times10^{-4}\) |
| holdout | \(1.4709\times10^{-1}\) | \(2.4385\times10^{-4}\) |

The residual floor is expected: the fitted material represents the
Neo-Hookean one-dimensional terms with finite-resolution integrated-curvature
splines.

As a resolution check, the same deterministic experiment gives:

| knots | parameters | design condition | fitted holdout RMSE |
|---:|---:|---:|---:|
| 5 | 6 | \(7.01\times10^1\) | \(4.67\times10^{-3}\) |
| 9 | 10 | \(2.11\times10^2\) | \(1.00\times10^{-3}\) |
| 17 | 18 | \(8.00\times10^2\) | \(2.44\times10^{-4}\) |
| 33 | 34 | \(3.80\times10^3\) | \(7.54\times10^{-5}\) |

All four systems are full rank and reach the positive linear oracle. More
knots reduce representation error but increase conditioning, so real noisy
data will eventually require smoothness regularization or fewer parameters.

## Outputs

Results are written under this demo's `output/` directory by default:

- `summary.json`: fit errors, design rank and condition number;
- `optimization_history.csv`: objective, gradient, damping, and step history;
- `fitted_parameters.csv`: initial, fitted, linear-oracle, and analytic
  Neo-Hookean reference samples;
- `stress_predictions.npz`: target and predicted stresses for both splits;
- `fit_diagnostics.png`: curvature, convergence, and holdout stress plots,
  when Matplotlib is installed.

This is a constitutive calibration example, not yet a full poking inverse
problem. Replacing synthetic affine stress observations with reaction-force or
displacement data requires differentiating through FEM equilibrium and contact.
