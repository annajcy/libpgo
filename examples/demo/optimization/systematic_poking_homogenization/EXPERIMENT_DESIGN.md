# Experimental Design: One-Element Systematic Poking Surrogates for Heterogeneous Neo-Hookean Cubes

Status: design contract for handoff; implementation has not started.

This document specifies two experiments in which a homogeneous, one-element
Systematic Poking (SP) model is fitted to the effective response of a
heterogeneous \(3\times3\times3\) Neo-Hookean (NH) finite-element cube.

The two fine-scale targets are:

1. a statistically isotropic random Young's-modulus field;
2. a deterministic four-shell, center-soft / outward-hardening
   Young's-modulus field.

The \(2\times2\times2\) version is intentionally skipped. Every fine target in
this design uses \(3\times3\times3=27\) cubic-linear hexahedral elements.

---

## 1. Research question

The main question is not merely whether an 18-parameter SP model can reduce a
training loss. It is:

> Can a homogeneous one-element SP material act as an effective constitutive
> closure for the relaxed macroscopic response of a heterogeneous
> \(3\times3\times3\) NH specimen, and does it generalize better than a
> homogeneous one-element NH material?

Three effects must be separated:

1. **local constitutive behavior:** every fine element is NH;
2. **heterogeneity-induced relaxation:** fine interior vertices move
   non-affinely because the element parameters differ;
3. **kinematic coarsening:** the one-element model has no internal degrees of
   freedom and is necessarily affine under an imposed macroscopic \(F\).

The experiments therefore include both constitutive baselines and kinematic
controls.

Because the specimen contains only three cells per direction, the result
should initially be described as a **finite-specimen effective response under
affine Dirichlet boundary conditions**, not as a converged infinite-medium
homogenized law.

---

## 2. Models and notation

### 2.1 Domain and meshes

The reference domain is the unit cube

\[
\Omega_0=[0,1]^3,\qquad V_0=1.
\]

The fine mesh has:

- \(3\times3\times3=27\) cubic-linear hexahedral elements;
- \(4^3=64\) vertices;
- \(192\) displacement degrees of freedom;
- \(56\) boundary vertices and \(8\) interior vertices.

Under an affine displacement prescribed on the entire boundary, only the
\(8\times3=24\) interior displacement degrees of freedom require a static
solve.

The coarse mesh has:

- one cubic-linear hexahedral element;
- eight vertices;
- no interior vertices.

The coarse displacement is therefore exactly affine under the same boundary
protocol.

### 2.2 Fine local material

Fine element \(e\) uses the existing classical compressible
`NeoHookeanDefinition` with parameters

\[
E_e>0,\qquad \nu_e=\nu_0.
\]

The default nominal parameters are

\[
E_0=2\times10^5,\qquad \nu_0=0.35.
\]

The first version varies only \(E_e\). Holding \(\nu_e\) fixed makes the
experiment easier to interpret: heterogeneity changes the local stiffness
scale without independently changing the shear-to-volume ratio.

An extension may instead construct random fields for shear and bulk moduli,

\[
\mu_e>0,\qquad \kappa_e>0,
\]

but this must not be mixed into the first evidence run.

### 2.3 Coarse Systematic Poking model

Use the current paper-style `SystematicPokingDefinition` with 17 stretch
knots:

\[
s_k=\exp\!\left(
  \operatorname{linspace}(\log 0.5,\log 2,17)
\right),
\]

and 17 volume knots:

\[
J_k=\exp\!\left(
  \operatorname{linspace}(-1,1,17)
\right).
\]

Its 18 fitted physical parameters are

\[
p=
\left(q_0,\ldots,q_{16},\lambda\right),
\qquad
q_k=f''(s_k)>0,\quad \lambda>0.
\]

Use positive log parameters

\[
p_i=E_0\exp(\theta_i).
\]

The stretch and volume rest-knot indices are both 8, corresponding to
\(s=1\) and \(J=1\).

### 2.4 Coarse Neo-Hookean baseline

The primary baseline is a homogeneous one-element NH model fitted to the same
training data. Optimize

\[
\mu>0,\qquad \lambda_{\mathrm{NH}}>0
\]

and convert to \(E,\nu\) for FEM validation.

For the current classical compressible NH law,

\[
P_{\mathrm{NH}}(F)
=
\mu\left(F-F^{-T}\right)
+
\lambda_{\mathrm{NH}}\log J\,F^{-T}.
\]

This expression is linear in the two physical parameters. It gives both a
cheap least-squares baseline and a linear-oracle check on any log-parameter
optimizer.

---

## 3. Fine-scale effective response

### 3.1 Primary boundary condition: KUBC

For each prescribed macroscopic deformation gradient \(F\), impose

\[
u_b(X)=(F-I)X
\]

on every boundary vertex. Interior degrees of freedom remain free.

The relaxed fine energy is

\[
E^\star(F)
=
\min_{u_i}
E_h\!\left(u_i,u_b(F);E_1,\ldots,E_{27},\nu_0\right).
\]

The equilibrium condition is

\[
\frac{\partial E_h}{\partial u_i}(u_i^\star,u_b)=0.
\]

This protocol is deliberately different from prescribing the same affine
motion at every fine vertex. The free interior vertices are what allow the
heterogeneous material to redistribute strain.

### 3.2 Extracting the macroscopic first Piola stress

Let

\[
r_a=\frac{\partial E_h}{\partial u_a}
\]

be the energy gradient at boundary vertex \(a\). With the codebase sign
convention, this is the constrained reaction used by the existing reaction
fitting demo.

The effective first Piola stress is

\[
\boxed{
\bar P_{\alpha\beta}(F)
=
\frac{1}{V_0}
\sum_{a\in\partial\Omega_0}
r_{a,\alpha}X_{a,\beta}
}.
\]

Equivalently,

\[
\delta E^\star
=
V_0\,\bar P:\delta F,
\qquad
\bar P=\frac{1}{V_0}\frac{\partial E^\star}{\partial F}.
\]

This identity must be verified by finite differences of the minimized energy:

\[
\bar P_{\alpha\beta}
\approx
\frac{
E^\star(F+\varepsilon e_\alpha e_\beta^T)
-
E^\star(F-\varepsilon e_\alpha e_\beta^T)
}{
2\varepsilon V_0
}.
\]

### 3.3 Equilibrium acceptance

A target sample is valid only if:

\[
\left\|
\frac{\partial E_h}{\partial u_i}
\right\|_\infty
\le 10^{-6},
\]

the deformation is finite, and every element remains orientation-preserving.
Record the actual residual, Newton iteration count, minimum element \(J\), and
solve status for every sample.

### 3.4 Affine-lock diagnostic

For the same fine material field, also evaluate a diagnostic state in which
the affine displacement is prescribed at **all** 64 vertices.

The difference

\[
\Delta P_{\mathrm{relax}}
=
\bar P_{\mathrm{KUBC}}-\bar P_{\mathrm{affine-lock}}
\]

measures the contribution of non-affine internal relaxation. If this
difference is numerically negligible, the experiment is not strongly testing
heterogeneity-induced closure.

This diagnostic is especially clean in the first version. With fixed
\(\nu_0\), both NH Lamé parameters scale linearly with \(E_e\). If every
equal-volume element experiences the same affine \(F\), then

\[
\bar P_{\mathrm{affine-lock}}(F)
=
\frac1{27}\sum_e P_{\mathrm{NH}}(F;E_e,\nu_0)
=
P_{\mathrm{NH}}(F;E_0,\nu_0)
\]

because \(\frac1{27}\sum_eE_e=E_0\). Therefore any departure of the relaxed
target from nominal homogeneous NH is caused by heterogeneous internal
relaxation, not by a trivial change in mean modulus.

---

## 4. Experiment 1: statistically isotropic random field

### 4.1 Purpose

This experiment asks whether an isotropic coarse SP law can represent the
ensemble-effective response created by spatially random but statistically
isotropic stiffness variation.

The clean primary target is the **ensemble-mean response**. A secondary
analysis fits each realization separately to quantify realization-level
anisotropy and variance.

### 4.2 Element centroids

For element indices \(i,j,k\in\{0,1,2\}\), use centroid

\[
x_{ijk}
=
\left(
\frac{i+1/2}{3},
\frac{j+1/2}{3},
\frac{k+1/2}{3}
\right).
\]

### 4.3 Gaussian log-field

For a realization seed \(m\), sample

\[
\eta^{(m)}\sim\mathcal N(0,C),
\]

with radial covariance

\[
C_{ef}
=
\sigma^2
\exp\!\left(
-\frac{\|x_e-x_f\|^2}{2\ell^2}
\right).
\]

Because the covariance depends only on Euclidean distance, the underlying
continuous random-field law has no preferred direction.

Use a deterministic eigendecomposition or Cholesky factorization with a
documented diagonal jitter, for example

\[
C_\epsilon=C+10^{-12}\sigma^2I.
\]

Special-case \(\sigma=0\) by returning the homogeneous field directly; do not
attempt to factor the zero covariance matrix.

Construct positive stiffness multipliers

\[
a_e=\exp\!\left(\eta_e-\frac{\sigma^2}{2}\right),
\]

then normalize each realization:

\[
\tilde a_e
=
\frac{a_e}{\frac1{27}\sum_{f=1}^{27}a_f},
\qquad
E_e=E_0\tilde a_e.
\]

Therefore every realization satisfies the exact arithmetic-mean constraint

\[
\frac1{27}\sum_{e=1}^{27}E_e=E_0.
\]

Here \(E_0,\nu_0\) are the specified nominal ground-truth parameters. The
actual elementwise \(E_e\) values are random ground-truth fields derived from
them; they are not all equal to \(E_0\).

### 4.4 Default field sweep

Use:

\[
\sigma\in\{0,\ 0.20,\ 0.40,\ 0.60\},
\qquad
\ell\in\left\{\frac16,\frac13,\frac23\right\}.
\]

Interpretation:

- \(\sigma=0\): homogeneous positive control;
- \(\ell=1/6\): rapidly varying field;
- \(\ell=1/3\): correlation near one element width;
- \(\ell=2/3\): smoothly varying field across the specimen.

Run 16 independent seeds for the full experiment. The smoke test uses two
seeds at \((\sigma,\ell)=(0.40,1/3)\), plus the homogeneous control.

Store the generated \(E_e\), seed, covariance parameters, minimum, maximum,
mean, standard deviation, and contrast \(E_{\max}/E_{\min}\).

### 4.5 Isotropy caveat and checks

A single random realization is generally anisotropic. Statistical isotropy is
a property of the distribution and is approached by ensemble averaging.

Consequently:

- the primary isotropic closure fit uses
  \[
  \bar P_{\mathrm{ens}}(F)
  =
  \frac1M\sum_{m=1}^M\bar P^{(m)}(F);
  \]
- realization-level fits are reported separately;
- confidence intervals across seeds must accompany mean errors;
- x/y/z and xy/xz/yz directional spreads must be reported.

Do not claim isotropy from one field visualization.

### 4.6 Primary hypothesis

At moderate heterogeneity, a fitted one-element SP model will reduce
held-out ensemble-mean stress error relative to the fitted one-element NH
baseline because heterogeneous strain redistribution creates an effective
law that is not exactly two-parameter NH.

### 4.7 Strongest competing explanations

1. The effective response remains nearly NH over the tested deformation
   range, so SP adds no meaningful test improvement.
2. SP improves only because it has 18 parameters and overfits the training
   deformation families.
3. Directional differences from finite realizations dominate the error, so
   an isotropic SP law cannot represent individual targets.
4. The apparent gap comes from one-element kinematic coarsening rather than
   constitutive closure.
5. The KUBC response of a three-cell specimen is boundary-condition-specific
   and does not transfer to free-surface structural tests.

---

## 5. Experiment 2: four-shell center-soft / outward-hardening field

### 5.1 Purpose

This experiment replaces random heterogeneity with a controlled mechanism.
The 27 elements are divided into the four natural cubic shells:

| shell \(\ell\) | index condition | element count | geometric role |
|---:|---|---:|---|
| 0 | \(d_e^2=0\) | 1 | cube center |
| 1 | \(d_e^2=1\) | 6 | face centers |
| 2 | \(d_e^2=2\) | 12 | edge centers |
| 3 | \(d_e^2=3\) | 8 | corners |

The modulus increases monotonically from the center through face centers and
edge centers to the corners, while the volume-mean modulus remains fixed. It
asks whether SP can encode the macroscopic nonlinearity induced by predictable
strain localization toward the softer inner shells.

### 5.2 Field definition

For element index \(e=(i,j,k)\), \(i,j,k\in\{0,1,2\}\), define

\[
d_e^2=(i-1)^2+(j-1)^2+(k-1)^2
\in\{0,1,2,3\}.
\]

The physical centroid radius from the cube center is

\[
r_e=\frac{\sqrt{d_e^2}}{3},
\]

and its normalized value is

\[
\xi_e=\frac{r_e}{r_{\max}}
=
\sqrt{\frac{d_e^2}{3}}
\in
\left\{
0,\frac1{\sqrt3},\sqrt{\frac23},1
\right\}.
\]

Let

\[
\rho=\frac{E_{\mathrm{corner}}}{E_{\mathrm{center}}}\ge1
\]

be the corner-to-center contrast ratio.

Use a log-linear radial profile

\[
b_e=\rho^{\xi_e}.
\]

Thus the four unnormalized shell moduli are proportional to

\[
1,\qquad
\rho^{1/\sqrt3},\qquad
\rho^{\sqrt{2/3}},\qquad
\rho.
\]

Normalize them to keep the arithmetic volume mean equal to \(E_0\):

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

This construction gives exactly four modulus values, preserves

\[
\frac1{27}\sum_e E_e=E_0,
\]

and satisfies

\[
\frac{E_{\mathrm{corner}}}{E_{\mathrm{center}}}=\rho.
\]

Keep

\[
\nu_e=\nu_0
\]

for every element.

### 5.3 Default contrast sweep

Use

\[
\rho\in\{1,\ 2,\ 5,\ 10,\ 20\}.
\]

The \(\rho=1\) case is the homogeneous positive control. The first
implementation should stop at \(\rho=20\); larger contrasts are allowed only
after equilibrium robustness and mesh sensitivity have been checked.

### 5.4 Symmetry statement

The field is invariant under the proper rotations of the cube. It has cubic
symmetry, not exact continuous \(SO(3)\) isotropy.

Therefore:

- x-, y-, and z-axis versions of the same load should agree;
- xy-, xz-, and yz-plane versions of the same shear should agree;
- arbitrarily rotated loads may differ.

This distinction is a result to measure, not an implementation defect to hide.

### 5.5 Optional second stage

The first version keeps the radial profile fixed and varies only \(\rho\).
Only after that study is complete, add a profile-shape exponent
\(\gamma>0\):

\[
b_e=\rho^{\xi_e^\gamma}.
\]

For example,

\[
\gamma\in\{0.5,1,2\}.
\]

This separates total center-to-corner contrast from how quickly the stiffness
rises across the face-center and edge-center shells. Do not optimize four
independent shell moduli in the first version; that would make the target
family harder to interpret without answering a more diagnostic question.

### 5.6 Primary hypothesis

As \(\rho\) increases, the relaxed fine response departs from homogeneous NH
because strain progressively concentrates toward the center and inner shells.
SP should outperform fitted coarse NH on held-out deformation paths until
kinematic coarsening becomes the dominant error.

### 5.7 Strongest competing explanations

1. The four-shell contrast may still be too weak to change the macroscopic
   response appreciably.
2. The effective response is still well represented by a softer homogeneous
   NH law.
3. At high contrast, one-element kinematics dominate and neither coarse
   constitutive law generalizes.
4. Improvements appear only on the training axes and fail under rotated or
   combined loading.

---

## 6. Deformation dataset

### 6.1 Valid deformation domain

Every retained \(F\) must satisfy:

\[
\det F>0,
\qquad
0.5\le s_i(F)\le2,
\qquad
e^{-1}\le\det F\le e.
\]

These bounds keep the experiment inside the current 17-knot SP domain.
Extrapolation should be reported separately and must not be mixed into the
primary score.

### 6.2 Families

Use the following base families, including axis or plane permutations:

1. Uniaxial deformation:
   \[
   F_{\mathrm{uni}}(t)
   =
   \operatorname{diag}(e^t,1,1).
   \]

2. Equal biaxial deformation:
   \[
   F_{\mathrm{bi}}(t)
   =
   \operatorname{diag}(e^t,e^t,1).
   \]

3. Volumetric deformation:
   \[
   F_{\mathrm{vol}}(t)=e^tI.
   \]

4. Isochoric diagonal deformation:
   \[
   F_{\mathrm{iso}}(t)
   =
   \operatorname{diag}(e^t,e^{-t},1).
   \]

5. Simple shear:
   \[
   F_{\mathrm{sh}}(\gamma)
   =
   I+\gamma\,e_i e_j^T,
   \qquad i\ne j.
   \]

6. Random spectral deformation:
   \[
   F=Q_L
   \operatorname{diag}(e^{t_1},e^{t_2},e^{t_3})
   Q_R^T,
   \]
   with independent deterministic-seed rotations and rejection against the
   valid-domain checks.

7. Combined held-out deformation:
   \[
   F=e^v
   Q_L
   \operatorname{diag}(e^t,e^{-t},1)
   Q_R^T
   \left(I+\gamma e_i e_j^T\right).
   \]

### 6.3 Split policy

The split is by deformation state, never by individual stress component.
All nine components of one \(P(F)\) belong to the same split.

Use:

- **training:** axis-aligned uniaxial, biaxial, volumetric, isochoric, shear,
  and a small random-spectral set;
- **validation:** interleaved amplitudes and independent random-spectral
  states; used only to select regularization and stopping;
- **final test:** untouched off-grid amplitudes, independent random rotations,
  and combined volume/shear/isochoric states.

Use the amplitude grids from
`systematic_poking_fit_neo_hookean/main.py` as the initial source, but generate
the split manifest once and save every \(F\) explicitly. Do not regenerate
test cases during optimizer tuning.

The default self-contained manifest is:

| split | family | amplitudes | orientations / count |
|---|---|---|---:|
| train | uniaxial | \(t=\pm\{0.10,0.25,0.40,0.55\}\) | canonical x-axis / 8 |
| train | biaxial | \(t=\pm\{0.15,0.25,0.35\}\) | canonical xy-plane / 6 |
| train | volumetric | \(t=\pm\{0.08,0.15,0.22\}\) | 6 |
| train | isochoric | \(t=\pm\{0.15,0.30,0.45\}\) | canonical xy pair / 6 |
| train | shear | \(\gamma=\pm\{0.25,0.50,0.80\}\) | canonical xy shear / 6 |
| train | random spectral | seeded admissible samples | 12 |
| train | knot-aligned uniaxial | all non-rest \(s_k\) | canonical x-axis / 16 |
| validation | uniaxial | \(t=\pm\{0.12,0.28,0.48\}\) | canonical x-axis / 6 |
| validation | biaxial | \(t=\pm\{0.10,0.20,0.30\}\) | canonical xy-plane / 6 |
| validation | volumetric | \(t=\pm\{0.04,0.11,0.19\}\) | 6 |
| validation | isochoric | \(t=\pm\{0.075,0.225,0.375\}\) | canonical xy pair / 6 |
| validation | shear | \(\gamma=\pm\{0.125,0.375,0.65\}\) | canonical xy shear / 6 |
| validation | random spectral | independent seeded samples | 12 |
| final test | uniaxial | \(t=\pm\{0.175,0.325,0.475\}\) | x, y, z / 18 |
| final test | biaxial | \(t=\pm\{0.075,0.175,0.275\}\) | xy, xz, yz / 18 |
| final test | volumetric | \(t=\pm\{0.065,0.13,0.205\}\) | 6 |
| final test | isochoric | \(t=\pm\{0.10,0.25,0.40\}\) | three axis pairs / 18 |
| final test | shear | \(\gamma=\pm\{0.20,0.45,0.70\}\) | all six ordered planes / 36 |
| final test | random spectral | independent seeded samples | 24 |
| final test | combined | independent seeded samples | 24 |

This gives 60 training, 42 validation, and 144 final-test states before any
sample is rejected by the valid-domain checks. A rejected deterministic state
is an error in the manifest. A rejected random state is resampled
deterministically from the same random-number stream.

The canonical training orientations deliberately leave most axis and
shear-plane permutations for final testing. The random-spectral training
states still provide general three-dimensional excitation, so this is an
orientation-transfer test rather than a deliberately blind fit.

Recommended deterministic seeds:

- training deformation seed: `20260801`;
- validation deformation seed: `20260802`;
- final-test deformation seed: `20260803`;
- random-field master seed: `20260810`.

### 6.4 Family balancing

Use a family-balanced stress objective:

\[
L_{\mathrm{data}}
=
\frac12
\sum_{g\in\mathcal G}
\frac{1}{N_g}
\sum_{j\in g}
\left\|
\frac{P_{\mathrm{model}}(F_j)-P^\star_j}{E_0}
\right\|_F^2.
\]

This prevents a family with more sampled amplitudes from dominating solely by
sample count.

For Experiment 1, \(P^\star\) is the ensemble-mean fine stress in the primary
fit. For Experiment 2, it is the deterministic fine stress.

---

## 7. Fitting protocol

### 7.1 Systematic Poking fit

Use

\[
p_i=E_0\exp(\theta_i)
\]

and minimize

\[
L(\theta)
=
L_{\mathrm{data}}(\theta)
+
\frac{\alpha}{2}
\left\|D_2\theta_q\right\|_2^2,
\]

where \(D_2\) is the second-difference operator on the 17 log-curvature
parameters. The volume coefficient is not included in this smoothness term.

Sweep

\[
\alpha\in
\{0,10^{-5},3\!\times\!10^{-5},10^{-4},
3\!\times\!10^{-4},10^{-3},3\!\times\!10^{-3},10^{-2}\}
\]

and select it by validation error. Evaluate the final test exactly once after
selection.

Reuse the analytic SP parameter Jacobian already exercised by the direct
\(P(F)\) fitting demo. The coarse one-element SP prediction is affine, so no
coarse static solve is needed for this primary stress objective.

### 7.2 Neo-Hookean fit

Fit the two-parameter homogeneous NH baseline on the identical training
states and family weights.

Run both:

1. positive log-parameter optimization;
2. the physical-parameter linear least-squares oracle.

If their training predictions disagree materially while the oracle parameters
are positive, treat that as an optimizer bug.

### 7.3 Optimizer-independent SP oracle

For a one-element affine SP model, stress is linear in the physical curvature
samples and volume scale. Construct

\[
\operatorname{vec}P(F_j)=B_jp
\]

and solve the stacked physical-parameter least-squares problem.

This oracle is not the final model if it contains non-positive parameters.
Its roles are:

- verify the analytic design matrix and optimizer;
- report rank and condition number;
- distinguish parameterization/optimization failure from model-capacity
  failure.

### 7.4 Initialization

Initialize SP from the nominal NH samples:

\[
q_k^{(0)}
=
\mu_0\left(1+\frac1{s_k^2}\right),
\]

with the nominal NH volume coefficient.

Initialize fitted coarse NH at \((E_0,\nu_0)\).

Also run at least three SP initialization scales,

\[
0.5p^{(0)},\quad p^{(0)},\quad 2p^{(0)},
\]

for one representative setting. Convergence to materially different test
solutions indicates an optimization or identifiability issue.

---

## 8. Baselines and controls

Every comparison must include:

1. **Nominal coarse NH:** one element with \((E_0,\nu_0)\).
2. **Fitted coarse NH:** one element with the best two NH parameters.
3. **Fitted coarse SP:** one element with 18 fitted SP parameters.
4. **Homogeneous fine positive control:** fine \(3^3\) mesh with
   \(E_e=E_0,\nu_e=\nu_0\).
5. **Affine-lock diagnostic:** heterogeneous fine mesh with all vertices
   prescribed affinely.

Add this diagnostic when the primary three-model comparison is working:

6. **Fine shared-SP control:** \(3^3\) mesh with one homogeneous SP parameter
   vector shared by all 27 elements.

Interpretation of the fine shared-SP control:

- fine shared SP succeeds, coarse SP fails: kinematic coarsening is the main
  bottleneck;
- both SP models fail: isotropic SP constitutive capacity or data coverage is
  insufficient;
- coarse SP succeeds: a one-element constitutive closure is plausible.

---

## 9. Evaluation metrics

### 9.1 Primary stress error

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

Also report family-balanced error, per-family error, normalized RMSE, and
maximum case error.

### 9.2 Improvement over fitted NH

Define

\[
\operatorname{Gain}
=
1-
\frac{\operatorname{RelErr}_{\mathrm{test}}(\mathrm{SP})}
{\operatorname{RelErr}_{\mathrm{test}}(\mathrm{fitted\ NH})}.
\]

Positive gain means SP is better; a gain of \(0.5\) means the SP test error is
half the fitted-NH test error.

### 9.3 Additional metrics

Report:

- error for every deformation family;
- stress tangent error on selected held-out paths,
  \(dP/dt\), using central differences;
- effective-energy error when \(E^\star(F)\) is stored;
- equilibrium residual and solve-failure count;
- minimum element determinant in fine solves;
- SP design-matrix rank and condition number;
- parameter positivity and sensitivity to initialization;
- difference between relaxed and affine-lock fine responses;
- directional spread across axis and shear-plane permutations.

For Experiment 1, report mean, median, standard deviation, and bootstrap 95%
confidence interval across realizations for realization-level metrics.

### 9.4 Structural-transfer validation

After the \(P(F)\) fit is frozen, run the existing static protocols on both
fine and coarse meshes:

- free uniaxial reaction;
- confined uniaxial reaction;
- simple-shear tangential reaction.

Generate cases independently from physical coordinates on each mesh; do not
reuse fine-mesh vertex IDs on the coarse mesh.

This stage answers whether the effective constitutive fit transfers from KUBC
stress data to free-surface specimen behavior. It is a final validation, not
part of parameter fitting.

---

## 10. Pre-registered decision rules

### 10.1 Evidence supporting the SP closure hypothesis

For a nontrivial heterogeneity setting, call the result supportive only if:

1. final-test
   \[
   \operatorname{Gain}\ge0.5;
   \]
2. SP improves most deformation families, not only one family;
3. SP reduces maximum-case error;
4. no new equilibrium failures or nonphysical test states appear;
5. structural-transfer error is also lower than fitted coarse NH;
6. the result is stable to reasonable initialization and regularization
   changes.

For Experiment 1 realization-level analysis, additionally require SP to beat
fitted NH for at least 75% of realizations and for the median gain to be
positive.

### 10.2 Evidence against the hypothesis

The hypothesis is not supported if:

- fitted coarse NH and SP have comparable held-out error;
- SP improves training but not validation/final test;
- gains disappear under rotated or combined loads;
- fine shared SP succeeds but coarse SP does not, showing that one-element
  kinematics, rather than the local law, is the main limitation;
- the relaxed/affine-lock difference is negligible, so the supposed
  heterogeneity mechanism was not excited.

### 10.3 Inconclusive outcomes

Treat the result as inconclusive if:

- target equilibrium tolerance is not met;
- the SP design matrix is rank deficient under the chosen training set;
- the result changes strongly with seed but too few realizations were run;
- the target frequently leaves the spline domain;
- high contrast produces mesh-dependent localization or element inversion;
- the test set was inspected before model/regularization choices were frozen.

---

## 11. Required implementation changes

### 11.1 Do not modify the old demos in place

Create a new directory:

```text
examples/demo/optimization/systematic_poking_homogenization/
```

Suggested files:

```text
main.py                  # CLI and experiment orchestration
mesh.py                  # n x n x n cubic grid and boundary queries
material_fields.py       # random and four-shell radial field generators
deformation_cases.py     # deterministic train/validation/test manifests
effective_response.py    # KUBC solve and macroscopic P extraction
fit_models.py            # coarse NH/SP fitting and linear oracles
metrics.py               # aggregation and decision statistics
dump_states.py           # OBJ export for representative fine states
README.md                # run instructions and obtained results
EXPERIMENT_DESIGN.md     # this contract
```

These may initially be fewer modules, but field generation, equilibrium
solving, and metric computation should remain independently testable.

### 11.2 Mesh helper

The existing reaction demo's `make_cubic_grid` currently rejects \(n=1\).
The new helper must accept every integer \(n\ge1\), because both \(n=3\) and
\(n=1\) are required.

Verify:

- element ordering and positive rest Jacobians;
- exactly 27 fine elements and one coarse element;
- 56 fine boundary vertices and eight fine interior vertices;
- boundary detection from coordinates, not hard-coded IDs.

### 11.3 Fine elementwise parameters

Construct the fine NH fixed-value array as

```python
fixed_values = np.column_stack([youngs_moduli, poisson_ratios])
```

with shape `(27, 2)`, using:

- `FixedParameterField`;
- `ElementwiseParameterLayout(27, 2)`;
- `IdentityMaterialChannelMapping(2)`.

Do not project a homogeneous imported material catalog for the heterogeneous
target.

### 11.4 Decouple fine and coarse energy construction

The current fitting demos build target and fitted materials on the same mesh.
The new code must own separate fine and coarse:

- imports;
- energy objects;
- rest-position arrays;
- boundary/fixed/free DOF lists;
- affine bases.

The same physical \(F\) is shared; node indices are not.

### 11.5 Dataset cache

Target generation is deterministic and should run once. Store:

```text
dataset_manifest.json
deformation_cases.npz
material_fields.npz
target_effective_response.npz
```

At minimum, each target row stores:

- experiment, field setting, realization seed;
- split, family, case label;
- \(F\), \(\bar P\), minimized energy;
- equilibrium residual, iteration count, minimum \(J\);
- affine-lock \(\bar P\), when enabled.

The fitting stage reads this cache and must not rerun target solves unless
explicitly requested.

---

## 12. Verification tests required before interpreting results

Add:

```text
tests/pypgo/test_systematic_poking_homogenization_demo.py
```

The test suite must cover:

1. **Mesh topology**
   - counts for \(n=1\) and \(n=3\);
   - positive element orientation;
   - boundary/interior partition.

2. **Random-field determinism**
   - identical seed gives identical \(E_e\);
   - different seed changes the field;
   - exact arithmetic mean equals \(E_0\);
   - all moduli are positive;
   - empirical covariance sanity over many cheap field draws.

3. **Four-shell radial field**
   - shell counts are exactly \(1,6,12,8\);
   - center, face-center, edge-center, and corner indices are classified
     correctly;
   - exactly four monotonically increasing modulus values exist for
     \(\rho>1\);
   - corner-to-center contrast is exactly \(\rho\);
   - exact mean \(E_0\);
   - equality under cubic index permutations.

4. **KUBC construction**
   - all and only boundary vertices are constrained;
   - prescribed values equal \((F-I)X\);
   - exactly 24 fine interior DOFs remain free.

5. **Equilibrium**
   - free residual below tolerance;
   - warm and cold starts agree;
   - homogeneous target produces the affine interior state.

6. **Macroscopic stress extraction**
   - boundary-reaction \(\bar P\) matches finite differences of minimized
     energy for random admissible \(F\);
   - all nine \(F\) components are checked.

7. **Homogeneous positive control**
   - fine \(3^3\) homogeneous NH and coarse one-element NH agree within tight
     tolerance for the full smoke deformation set.

8. **Constitutive Jacobians**
   - coarse SP analytic \(dP/d\theta\) matches central finite differences;
   - coarse NH design matrix matches finite differences;
   - full parameter columns are tested.

9. **Split integrity**
   - labels and \(F\) matrices are disjoint across train, validation, and
     final test;
   - saved manifest reloads exactly.

10. **Symmetry checks**
    - homogeneous control is rotation/isotropy consistent;
    - four-shell field matches x/y/z and xy/xz/yz counterparts;
    - random ensemble directional discrepancy decreases as realizations are
      aggregated, within statistical tolerance.

11. **End-to-end smoke test**
    - two random realizations and one four-shell contrast;
    - reduced case counts and optimizer iterations;
    - finite metrics and expected output files;
    - SP training loss decreases.

No scientific conclusion should be drawn until tests 1--8 pass.

---

## 13. Output artifacts and plots

Each run directory should contain:

```text
config.json
dataset_manifest.json
material_fields.npz
target_effective_response.npz
fit_summary.json
fitted_sp_parameters.csv
fitted_nh_parameters.csv
optimization_history.csv
predictions.npz
```

Generate:

1. training and validation loss curves;
2. train/validation/test error by model;
3. per-family test error;
4. fine target vs coarse NH vs coarse SP stress paths;
5. SP curvature samples \(q_k\) and fitted volume coefficient;
6. random-field slice/voxel plots;
7. four-shell radial-field visualization;
8. gain versus \(\sigma\), \(\ell\), or \(\rho\);
9. relaxed versus affine-lock response;
10. directional anisotropy plots;
11. representative rest/deformed OBJ files with a JSON solve manifest.

Every plot must be reproducible from saved numerical artifacts without
rerunning optimization.

---

## 14. Execution stages

### Stage A: target generator

1. Implement \(n=1\) and \(n=3\) meshes.
2. Implement elementwise NH fixed parameters.
3. Implement KUBC static solve.
4. Extract \(\bar P\) from boundary reactions.
5. Pass energy finite-difference and homogeneous-control tests.

Stop here if macroscopic stress extraction is not verified.

### Stage B: coarse baselines

1. Fit the two-parameter coarse NH baseline.
2. Fit the 18-parameter coarse SP model.
3. Compare both optimizers to their physical linear oracles.
4. Verify analytic SP parameter Jacobians.

Stop here if the homogeneous positive control does not recover NH.

### Stage C: minimum diagnostic experiments

Run:

- random field: \((\sigma,\ell)=(0.40,1/3)\), two seeds;
- four-shell outward-hardening field: \(\rho=5\);
- homogeneous controls;
- reduced deformation manifests.

Inspect:

- relaxed/affine-lock gap;
- train/validation separation;
- SP versus fitted NH final-test error;
- equilibrium robustness.

Use this stage to correct implementation and deformation coverage, not to
claim evidence.

The smoke run contains at most

\[
(1+2+1)(60+42+144)=984
\]

fine target states: one homogeneous control, two random realizations, and one
four-shell field. Benchmark this stage before launching the full sweep.

### Stage D: full Experiment 1

1. Freeze deformation manifests and regularization sweep.
2. Generate all field realizations.
3. Fit ensemble-mean models for each \((\sigma,\ell)\).
4. Run realization-level secondary fits.
5. Report paired statistics and directional spread.

Treat \(\sigma=0\) as one shared homogeneous setting rather than repeating it
for every \(\ell\). The default sweep then contains

\[
1+3\times3\times16=145
\]

fine fields and

\[
145(60+42+144)=35{,}670
\]

cached KUBC states. Parallelization, if added, should be over independent
field/settings, and each worker should write a separate temporary artifact
that is merged only after successful completion.

### Stage E: full Experiment 2

1. Run the full \(\rho\) sweep.
2. Measure center strain localization and affine-relaxation gap.
3. Add the fine shared-SP control at representative contrasts.
4. Add the radial profile-shape \(\gamma\) sweep only if needed.

The initial five-contrast sweep contains

\[
5(60+42+144)=1{,}230
\]

KUBC states. These counts exclude structural-transfer solves, which should be
scheduled only after the primary fits are frozen.

### Stage F: structural transfer

Freeze parameters, then run free uniaxial, confined uniaxial, and shear
reaction curves. This stage determines whether the KUBC effective law
transfers to different specimen boundary conditions.

---

## 15. Interpretation table

| Observation | Most likely interpretation |
|---|---|
| Coarse SP clearly beats fitted coarse NH on held-out \(P(F)\) and reaction curves | SP is a useful effective constitutive closure |
| SP wins only on training data | extra parameters are overfitting |
| SP wins on \(P(F)\) but not structural reaction curves | KUBC effective law is boundary-condition-specific or coarse kinematics dominate |
| Fine shared SP wins but coarse SP fails | one-element kinematic coarsening is the bottleneck |
| Coarse NH and SP are similar | effective response is NH-like over this regime |
| Relaxed and affine-lock targets are similar | heterogeneity-induced internal relaxation is weak |
| Random ensemble fit works but per-realization fits have directional residuals | ensemble is isotropic, individual specimens are not |
| Center-soft x/y/z agree but arbitrary rotations differ | expected cubic, not continuous, symmetry |
| Errors rise sharply with \(\rho\) and minimum \(J\) falls | high-contrast localization or mesh resolution is limiting |

---

## 16. Handoff checklist

The next session should begin by:

1. reading this document;
2. reading:
   - `examples/demo/optimization/systematic_poking_fit_neo_hookean/main.py`;
   - `examples/demo/optimization/systematic_poking_fit_reaction_force/main.py`;
   - `pypgo/fem/elastic.py`;
   - `pypgo/fem/fields.py`;
3. confirming the current build and Python test command;
4. implementing Stage A only;
5. reporting the homogeneous-control and \(\bar P\) finite-difference results
   before starting either optimizer.

Implementation decisions already fixed by this design:

- no \(2\times2\times2\) experiment;
- fine mesh is \(3\times3\times3\);
- coarse model is one element;
- first random-field version varies \(E\) only and fixes \(\nu\);
- each random realization is mean-normalized to \(E_0\);
- Experiment 1 primary target is the ensemble-mean response;
- Experiment 2 uses the fixed \(1+6+12+8\) radial shell partition and varies
  only the corner-to-center contrast \(\rho\) initially;
- primary fitting uses relaxed KUBC macroscopic stress;
- free/confined/shear reactions are untouched structural-transfer tests;
- comparisons use fitted coarse NH, not only nominal NH;
- final-test data remain sealed until model and regularization choices are
  frozen.

The first implementation milestone is successful when a homogeneous
\(3\times3\times3\) NH target produces:

1. an affine interior equilibrium state;
2. a reaction-extracted \(\bar P\) matching minimized-energy finite
   differences;
3. the same \(\bar P(F)\) as the homogeneous one-element NH model;
4. a fitted one-element SP result consistent with the existing direct
   Neo-Hookean fitting demo.
