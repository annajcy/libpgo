# Fit Systematic Poking through static reaction forces

This demo is the bridge between direct constitutive calibration and a full
contact-based poking inverse problem. It fits a Systematic Poking material to
synthetic Neo-Hookean force-displacement curves while differentiating through
an actual static equilibrium solve.

The default configuration uses:

- a \(2\times2\times2\) cubic-linear FEM mesh;
- 17 stretch-curvature parameters \(q_k=f''(s_k)\);
- one volumetric coefficient \(\lambda\);
- positive log parameters \(e_i=E_{\mathrm{ref}}\exp(\theta_i)\);
- free-uniaxial, confined-uniaxial, and simple-shear reaction curves;
- curvature smoothness weight \(3\times10^{-3}\).

## Forward problem

For each prescribed boundary displacement, the free displacement degrees of
freedom satisfy

\[
R_f(u_f^\star,u_c,e)
=
\frac{\partial E_h}{\partial u_f}
=0.
\]

The observed scalar reaction is

\[
y(e)=c^T R(u^\star,e),
\]

where \(c\) sums the relevant top-face reaction component.

Unlike the direct \(P(F)\) fitting demo, the deformation state is not supplied
to the constitutive model. It is found by Newton's method. The backend solver
is followed by a small dense Newton polish so every retained state satisfies

\[
\lVert R_f\rVert_\infty\leq10^{-6}.
\]

For the material scale \(E_{\mathrm{ref}}=2\times10^5\), this is a relative
equilibrium residual below \(5\times10^{-12}\).

## Implicit reaction sensitivity

Differentiate free equilibrium with respect to physical material parameters:

\[
K_{ff}\frac{du_f^\star}{de}
+E_{ue,f}=0,
\]

so

\[
\frac{du_f^\star}{de}
=-K_{ff}^{-1}E_{ue,f}.
\]

The total reaction derivative is

\[
\boxed{
\frac{dy}{de}
=
c^T
\left(
E_{ue}
-K_{:f}K_{ff}^{-1}E_{ue,f}
\right)
}.
\]

With \(e_i=E_{\mathrm{ref}}\exp(\theta_i)\), the normalized residual Jacobian
is

\[
\frac{\partial(y/E_{\mathrm{ref}})}{\partial\theta_i}
=
\frac{1}{E_{\mathrm{ref}}}
\frac{\partial y}{\partial e_i}e_i.
\]

This complete derivative, including a fresh static solve for every finite
difference perturbation, matches central finite differences with relative
Frobenius error about \(5.2\times10^{-10}\).

## Data splits

Training uses every non-rest stretch knot in both uniaxial protocols, plus
four shear levels. Validation uses geometric midpoints between adjacent
knots and distinct shear levels. It is used to select the smoothness weight.

After that selection, a separate holdout is evaluated at the one-quarter and
three-quarter log-space points of every knot interval, plus eight previously
unused shear levels. The default 17-knot counts are:

| split | cases | role |
|---|---:|---|
| training | 36 | fit material parameters |
| validation | 36 | select regularization |
| holdout | 72 | final untouched evaluation |

## Objective and regularization

The data residual for reaction observation \(j\) is

\[
r_j(\theta)
=
\frac{y_j(\theta)-y_j^\star}{E_{\mathrm{ref}}}.
\]

For uniformly spaced log stretch knots, smoothness is imposed on the log
curvatures:

\[
L(\theta)
=
\frac12\sum_jr_j^2
+
\frac{\alpha}{2}
\left\|D_2\theta_q\right\|_2^2.
\]

The log-space penalty discourages relative curvature oscillation without
favoring the large-magnitude compression-side parameters.

An exploratory validation sweep gave:

| \(\alpha\) | train RMSE | validation RMSE |
|---:|---:|---:|
| \(0\) | \(2.10\times10^{-4}\) | \(3.88\times10^{-4}\) |
| \(10^{-4}\) | \(2.45\times10^{-4}\) | \(2.90\times10^{-4}\) |
| \(10^{-3}\) | \(2.89\times10^{-4}\) | \(2.56\times10^{-4}\) |
| \(3\times10^{-3}\) | \(3.08\times10^{-4}\) | \(2.55\times10^{-4}\) |
| \(10^{-2}\) | \(3.34\times10^{-4}\) | \(2.65\times10^{-4}\) |

The selected value is \(3\times10^{-3}\). The subsequently evaluated sealed
holdout RMSE is \(2.50\times10^{-4}\).
The unaggregated sweep values are retained in `regularization_sweep.csv`.

A separate no-regularization resolution check is retained in
`resolution_sweep.csv`. The 5-, 9-, and 17-knot designs are all full rank;
their conditions are approximately 47, 139, and 334, respectively. Because
the number of knot-aligned training observations grows with model resolution,
this is a joint capacity-and-excitation check, not a controlled
fixed-data model-capacity comparison.

## Run

From the repository root:

```bash
PYTHONPATH=build/base/lib:$PYTHONPATH \
python examples/demo/optimization/systematic_poking_fit_reaction_force/main.py
```

Useful controls:

```bash
python examples/demo/optimization/systematic_poking_fit_reaction_force/main.py \
  --knot-count 17 \
  --grid-size 2 \
  --smoothness-weight 3e-3 \
  --max-iterations 40 \
  --output-dir /tmp/systematic-poking-reaction-fit \
  --no-plots
```

## Default result

The deterministic default run converges in seven outer Gauss-Newton
iterations. Its data Jacobian has rank \(18/18\) and condition number about
332.

| split | initial RMSE | fitted RMSE |
|---|---:|---:|
| training | \(3.52\times10^{-1}\) | \(3.08\times10^{-4}\) |
| validation | \(3.14\times10^{-1}\) | \(2.55\times10^{-4}\) |
| holdout | \(3.15\times10^{-1}\) | \(2.50\times10^{-4}\) |

Outputs under `output/` include:

- `summary.json`;
- `optimization_history.csv`;
- `fitted_parameters.csv`;
- `reaction_predictions.npz`;
- `fit_diagnostics.png`, when Matplotlib is installed.

The next extension should replace prescribed top-face loading with a
cylindrical contact indenter while retaining the same equilibrium sensitivity
and identifiability checks.
