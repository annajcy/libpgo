# Experimental Design: One-Element SP Fits of Heterogeneous Neo-Hookean Cubes

Status: concise design contract; implementation has not started.

## 1. Research question

The experiment asks one question:

> Can a homogeneous one-element Systematic Poking (SP) material fit the
> relaxed macroscopic response of a heterogeneous \(3\times3\times3\)
> Neo-Hookean (NH) cube better than a fitted homogeneous one-element NH
> material?

For every heterogeneous fine target, the experiment is

\[
\boxed{
\text{fine heterogeneous NH}
\xrightarrow{\text{KUBC relaxation}}
P^\star(F)
\xrightarrow{\text{same train/validation/test data}}
\begin{cases}
\text{fitted one-element NH},\\
\text{fitted one-element SP}.
\end{cases}
}
\]

The coarse models fit only the effective map \(F\mapsto P^\star(F)\). They do
not reconstruct fine displacements, element stresses, or the 27 local moduli.

Two fine-target families are used:

1. independently sampled random Young's-modulus fields;
2. deterministic center-soft / outward-hardening four-shell fields.

Each random realization is fitted separately. Responses are never averaged
across realizations before fitting. Statistics across seeds are computed only
after every realization has produced its own fitted NH and SP models.

The result is a finite-specimen effective response under affine Dirichlet
boundary conditions, not a converged infinite-medium homogenized law.

---

## 2. Common setup

### 2.1 Fine and coarse meshes

The reference domain is the unit cube

\[
\Omega_0=[0,1]^3,\qquad V_0=1.
\]

The fine mesh contains:

- \(3\times3\times3=27\) cubic-linear hexahedral elements;
- \(4^3=64\) vertices;
- 56 boundary vertices;
- 8 interior vertices, giving 24 free displacement degrees of freedom under
  KUBC.

The coarse mesh contains one cubic-linear hexahedral element and has no
internal degrees of freedom under the same macroscopic deformation.

### 2.2 Fine local material

Fine element \(e\) uses the existing compressible NH material with

\[
E_e>0,\qquad \nu_e=\nu_0,
\]

where

\[
E_0=2\times10^5,\qquad \nu_0=0.35.
\]

Only \(E_e\) varies. Every target field is normalized so that

\[
\frac1{27}\sum_{e=1}^{27}E_e=E_0.
\]

Fixing \(\nu_e=\nu_0\) isolates stiffness-scale heterogeneity and makes the
affine-lock check in Section 5 exact.

### 2.3 Relaxed fine target

For each prescribed macroscopic deformation gradient \(F\), impose

\[
u_b(X)=(F-I)X
\]

on the 56 boundary vertices. Leave the 8 interior vertices free and solve

\[
E^\star(F)
=
\min_{u_i}
E_h(u_i,u_b(F);E_1,\ldots,E_{27},\nu_0).
\]

The target response is

\[
\boxed{P^\star(F)=\bar P_{\mathrm{KUBC,relaxed}}(F)}.
\]

At the converged equilibrium, extract the macroscopic first Piola stress from
the boundary reactions:

\[
\bar P_{\alpha\beta}(F)
=
\frac1{V_0}
\sum_{a\in\partial\Omega_0}
r_{a,\alpha}X_{a,\beta}.
\]

A target sample is valid only if the interior equilibrium residual is below
the chosen tolerance, all values are finite, and every element has positive
determinant. Store the residual, iteration count, minimum element \(J\), and
solve status with every target.

### 2.4 Coarse models

Fit both coarse models to exactly the same target rows and weights.

**Fitted one-element NH baseline**

\[
P_{\mathrm{NH}}(F)
=
\mu(F-F^{-T})
+
\lambda_{\mathrm{NH}}\log J\,F^{-T},
\]

with positive fitted \(\mu\) and \(\lambda_{\mathrm{NH}}\).

**Fitted one-element SP**

Use the existing paper-style SP model with 17 stretch knots

\[
s_k
=
\exp\!\left(
\operatorname{linspace}(\log0.5,\log2,17)
\right)
\]

and 17 volume knots on

\[
J_k
=
\exp\!\left(
\operatorname{linspace}(-1,1,17)
\right).
\]

Fit the 18 positive physical parameters

\[
(q_0,\ldots,q_{16},\lambda),
\qquad q_k=f''(s_k)>0,\quad \lambda>0.
\]

Use positive log parameters and select SP smoothness regularization on the
validation split only.

### 2.5 Deformation data and split

Retained deformation states must satisfy

\[
\det F>0,\qquad
0.5\le s_i(F)\le2,\qquad
e^{-1}\le\det F\le e.
\]

Include:

- uniaxial deformation;
- equal biaxial deformation;
- volumetric deformation;
- isochoric diagonal deformation;
- simple shear;
- seeded random spectral deformation;
- combined volume, isochoric, shear, and rotation states.

Use:

- training: axis-aligned paths plus a small random-spectral set;
- validation: interleaved amplitudes and independent random-spectral states;
- final test: off-grid amplitudes, independent rotations, and combined
  deformations.

The split is by complete deformation state, never by stress component. Save
every \(F\) and its split once. Do not regenerate or inspect final-test cases
while choosing regularization, initialization, or stopping.

Fit with a family-balanced stress objective:

\[
L_{\mathrm{data}}
=
\frac12
\sum_{g\in\mathcal G}
\frac1{N_g}
\sum_{j\in g}
\left\|
\frac{P_{\mathrm{model}}(F_j)-P^\star(F_j)}{E_0}
\right\|_F^2.
\]

---

## 3. Experiment 1: random heterogeneous realizations

### 3.1 Field construction

For element centroids \(x_e\), sample

\[
\eta^{(m)}\sim\mathcal N(0,C),
\qquad
C_{ef}
=
\sigma^2
\exp\!\left(
-\frac{\|x_e-x_f\|^2}{2\ell^2}
\right).
\]

Construct

\[
a_e=\exp(\eta_e),
\qquad
E_e
=
E_0
\frac{a_e}{\frac1{27}\sum_fa_f}.
\]

This guarantees positive moduli and exact arithmetic mean \(E_0\). Handle
\(\sigma=0\) directly as the homogeneous field.

Use

\[
\sigma\in\{0,0.20,0.40,0.60\},
\qquad
\ell\in\left\{\frac16,\frac13,\frac23\right\}.
\]

Use 16 independent seeds per nonzero \((\sigma,\ell)\) setting in the full
run. The smoke run uses two seeds at
\((\sigma,\ell)=(0.40,1/3)\).

### 3.2 Unit of analysis

For each realization \(m\):

1. generate and save its 27 values \(E_e^{(m)}\);
2. generate its relaxed target \(P_m^\star(F)\);
3. fit a separate one-element NH model to \(P_m^\star\);
4. fit a separate one-element SP model to \(P_m^\star\);
5. evaluate both models on the held-out deformation states.

Do not form or fit

\[
\frac1M\sum_mP_m^\star(F).
\]

The random-field distribution is invariant under cubic-grid rotations, but a
single realization is generally anisotropic and has no cubic symmetry. Both
coarse models are isotropic, so realization-level directional residuals are
expected and must be reported rather than hidden.

After all individual fits are complete, summarize errors and SP-over-NH gains
across seeds using the mean, median, spread, confidence interval, and fraction
of realizations on which SP wins.

### 3.3 Hypothesis

For a substantial fraction of random heterogeneous specimens, non-affine
strain redistribution creates a held-out effective response that is not well
described by a refitted two-parameter NH model, and one-element SP reduces
that error.

---

## 4. Experiment 2: center-soft four-shell field

### 4.1 Field construction

For element index \(e=(i,j,k)\), define

\[
d_e^2=(i-1)^2+(j-1)^2+(k-1)^2\in\{0,1,2,3\}.
\]

The four shells contain:

| \(d_e^2\) | count | role |
|---:|---:|---|
| 0 | 1 | center |
| 1 | 6 | face centers |
| 2 | 12 | edge centers |
| 3 | 8 | corners |

Let

\[
\xi_e=\sqrt{\frac{d_e^2}{3}},
\qquad
\rho=\frac{E_{\mathrm{corner}}}{E_{\mathrm{center}}}\ge1,
\qquad
b_e=\rho^{\xi_e}.
\]

Normalize by

\[
\bar b
=
\frac{
1
+6\rho^{1/\sqrt3}
+12\rho^{\sqrt{2/3}}
+8\rho
}{27},
\qquad
E_e=E_0\frac{b_e}{\bar b}.
\]

Use

\[
\rho\in\{1,2,5,10,20\}.
\]

The \(\rho=1\) case is the homogeneous check. For every nontrivial \(\rho\),
generate one deterministic target and fit its own one-element NH and SP
models.

The field has cubic symmetry, not continuous rotational isotropy. Coordinate
axis and coordinate shear-plane permutations should agree, while arbitrary
rotations may differ.

### 4.2 Hypothesis

As \(\rho\) increases, strain concentrates toward the softer inner shells and
the relaxed effective response departs from a refitted homogeneous NH law.
One-element SP should reduce held-out error over a nontrivial range of
\(\rho\).

---

## 5. Required checks

These checks validate the target and fitting pipeline. They are not additional
scientific experiments.

### 5.1 Nominal NH reference and homogeneous fine check

Evaluate the nominal one-element response

\[
P_{\mathrm{nominal}}(F)=P_{\mathrm{NH}}(F;E_0,\nu_0).
\]

Set all 27 fine elements to \(E_0,\nu_0\) and run the relaxed KUBC solve. The
fine interior state must be affine, and

\[
P_{\mathrm{fine,homogeneous}}(F)
\approx
P_{\mathrm{nominal}}(F).
\]

Failure indicates a problem with the mesh, element orientation, KUBC,
equilibrium solve, reaction extraction, or stress aggregation.

### 5.2 Heterogeneous affine-lock check

For a mean-normalized heterogeneous field, prescribe

\[
u_a=(F-I)X_a
\]

at all 64 vertices. Every element then experiences the same \(F\).

At fixed \(\nu_0\), both NH Lamé parameters are linear in \(E_e\), so

\[
P_{\mathrm{NH}}(F;E_e,\nu_0)
=
E_eA(F,\nu_0).
\]

Therefore

\[
\bar P_{\mathrm{affine-lock}}(F)
=
\frac1{27}\sum_eP_{\mathrm{NH}}(F;E_e,\nu_0)
=
P_{\mathrm{nominal}}(F).
\]

Because the 8 interior vertices are constrained, they generally carry
nonzero constraint reactions. Compute affine-lock stress either by element
volume averaging or by including reactions from all 64 vertices:

\[
\bar P_{\mathrm{affine-lock}}
=
\frac1{V_0}\sum_eV_eP_e
=
\frac1{V_0}\sum_{a=1}^{64}r_aX_a^T.
\]

Do not use the boundary-only reaction sum for affine-lock.

The relaxation signal is

\[
\Delta P_{\mathrm{relax}}(F)
=
P^\star(F)-P_{\mathrm{affine-lock}}(F).
\]

If affine-lock does not match nominal NH, check mean normalization,
elementwise material assignment, element volumes, affine constraints, and
stress extraction. If \(\Delta P_{\mathrm{relax}}\) is negligible, the chosen
heterogeneity and loading do not create a meaningful closure signal.

### 5.3 Macroscopic stress and equilibrium checks

For representative admissible \(F\), verify

\[
\bar P_{\alpha\beta}
\approx
\frac{
E^\star(F+\varepsilon e_\alpha e_\beta^T)
-
E^\star(F-\varepsilon e_\alpha e_\beta^T)
}{
2\varepsilon V_0
}
\]

for all nine components. Use a finite-difference step sweep and require a
stable agreement range.

Also verify:

- the KUBC system has exactly 24 free displacement degrees of freedom;
- the equilibrium residual satisfies the chosen tolerance;
- warm and cold starts agree on representative states;
- every accepted element remains orientation-preserving.

### 5.4 Fitting checks

- Fit NH both with positive log parameters and with its physical linear
  least-squares oracle; positive solutions must agree.
- Verify the analytic SP parameter Jacobian against central differences.
- Report the SP design-matrix rank and condition number.
- Run several SP initialization scales on one representative target.
- Verify that complete \(F\) states are disjoint across train, validation,
  and final test.

Do not interpret scientific results until all checks in this section pass.

---

## 6. Metrics and decision rules

For model \(m\) and split \(S\), report

\[
\operatorname{RelErr}_S(m)
=
\frac{
\sqrt{\sum_{j\in S}
\|P_j^{(m)}-P_j^\star\|_F^2}
}{
\sqrt{\sum_{j\in S}
\|P_j^\star\|_F^2}
}.
\]

Define

\[
\operatorname{Gain}
=
1-
\frac{
\operatorname{RelErr}_{\mathrm{test}}(\mathrm{SP})
}{
\operatorname{RelErr}_{\mathrm{test}}(\mathrm{fitted\ NH})
}.
\]

Also report family-balanced error, per-family error, and maximum-case error.

For Experiment 1, compute these metrics for every realization separately,
then summarize them across seeds. Do not evaluate an ensemble-averaged target.
For Experiment 2, report them separately for every \(\rho\).

A setting supports the SP closure hypothesis only when:

- SP improves held-out test error over fitted NH, not only training error;
- improvement occurs across most deformation families;
- maximum-case error does not become worse;
- the result is stable to reasonable regularization and initialization
  changes;
- the relaxed/affine-lock gap is clearly above the numerical check error.

The hypothesis is weakened when fitted NH and SP have comparable held-out
errors, or when SP improves training but not validation/test.

The result is inconclusive when target equilibrium, stress extraction,
affine-lock, rank, split-integrity, or deformation-domain checks fail. A large
directional residual shared by both isotropic coarse models should be reported
as realization or cubic anisotropy, not silently attributed to SP capacity.

---

## 7. Minimum execution plan

### Step A: pass checks

1. Build the \(3^3\) fine mesh and one-element coarse mesh.
2. Pass the homogeneous fine-NH check.
3. Pass the all-vertex affine-lock check.
4. Pass minimized-energy finite differences for \(\bar P\).
5. Pass NH and SP fitting checks.

Stop if any of these checks fails.

### Step B: smoke experiment

Run:

- two random realizations at \((\sigma,\ell)=(0.40,1/3)\);
- the four-shell field at \(\rho=5\);
- reduced train, validation, and held-out deformation sets.

For each target, fit separate NH and SP models. Inspect equilibrium,
relaxed/affine-lock separation, training-versus-test behavior, and SP gain.
Use the smoke run to fix implementation and data coverage, not to make a
scientific claim.

### Step C: full experiments

Freeze the deformation manifest, fitting procedure, and decision rules.
Then:

1. run every random seed and fit each realization separately;
2. aggregate realization-level metrics only after all fits finish;
3. run every \(\rho\) and fit it separately;
4. evaluate the sealed final test once.

Save enough information to reproduce every metric without rerunning target
generation: configuration, deformation manifest, material fields, target
responses, solve diagnostics, fitted parameters, predictions, and metrics.
