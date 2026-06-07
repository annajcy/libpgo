# `pypgo.fem.elastic` — Constitutive Models

## Purpose

`elastic.py` chooses the **strain energy density** $\Psi(\mathbf F)$ — the function that
turns deformation into stored energy. Each Python class is a thin handle over a C++
`ElasticModel`; the model knows how to compute three things at a deformation gradient
$\mathbf F$:

$$\Psi(\mathbf F)\ \ \text{(energy)}, \qquad \mathbf P=\frac{\partial\Psi}{\partial\mathbf F}\ \ \text{(stress)}, \qquad \frac{\partial^2\Psi}{\partial\mathbf F^2}\ \ \text{(tangent)}.$$

These are the only material-specific quantities in the whole pipeline; everything in
[`formulations.md`](formulations.md) and [`energy.md`](energy.md) is model-agnostic and
just chains them through $\partial\mathbf F/\partial\mathbf u$ and the element integral.

## The shared interface

Every 3D model derives from `ElasticModel3DDeformationGradient`
(`src/core/solidDeformationModel/elastic/elasticModel3DDeformationGradient.h:14`) and
implements

```cpp
double compute_psi (const double *param, const double F[9], const double U[9], const double V[9], const double S[3]) const;
void   compute_P   (... , double P[9])  const;   // 1st Piola–Kirchhoff stress  ∂Ψ/∂F
void   compute_dPdF(... , double dPdF[81]) const; // material tangent           ∂²Ψ/∂F²
```

Three details matter for reading the math:

- **Energy is evaluated on the *elastic* part.** The element first removes plasticity,
  $\mathbf F_e=\mathbf F\,\mathbf F_p^{-1}$, takes its SVD
  $\mathbf F_e=\mathbf U\boldsymbol\Sigma\mathbf V^\top$, and passes
  $(\mathbf F_e,\mathbf U,\mathbf V,\boldsymbol\Sigma)$ to the model
  (`formulations/elements/volumetricDeformationModel.h:54,68`). Singular values make the
  energy **rotation-invariant** and let stable models clamp eigenvalues analytically.
- **`param` carries the optimizable elastic channels** (from the field in
  [`fields.md`](fields.md)). For the standard volumetric models below it is **ignored** —
  their Lamé coefficients are baked in at construction from the *mesh* material
  ($E,\nu\Rightarrow\lambda,\mu$, see [`../mesh/volume/material.md`](../mesh/volume/material.md)),
  so `getNumParameters() == 0`. Only the shell / activation models expose channels.
- **SPD projection.** When `enforce_spd` is on (see [`energy.md`](energy.md)), `compute_dPdF`
  returns a positive-semidefinite projection of the tangent so Newton stays descent — for
  Stable Neo-Hookean this is the analytic eigenvalue clamp in
  `elastic/elasticModelStableNeoHookeanMaterial.cpp:126-180`.

### Interface ↔ function

The six quantities every model can supply, and the C++ method that returns each
(`elastic/elasticModel3DDeformationGradient.h:20-45`). $\theta_i$ denotes the $i$-th
optimizable channel; the parameter derivatives default to $0$ for models with no channels.

| Quantity | Formula | C++ method |
|---|---|---|
| energy density | $\Psi(\mathbf F)$ | `compute_psi(param, F, U, V, S)` |
| 1st PK stress | $\mathbf P=\dfrac{\partial\Psi}{\partial\mathbf F}$ | `compute_P(…, P[9])` |
| material tangent | $\dfrac{\partial\mathbf P}{\partial\mathbf F}=\dfrac{\partial^2\Psi}{\partial\mathbf F^2}$ | `compute_dPdF(…, dPdF[81])` |
| parameter gradient | $\dfrac{\partial\Psi}{\partial\theta_i}$ | `compute_dpsi_dparam(param, i, …)` |
| parameter Hessian | $\dfrac{\partial^2\Psi}{\partial\theta_i\,\partial\theta_j}$ | `compute_d2psi_dparam2(param, i, j, …)` |
| stress–parameter | $\dfrac{\partial\mathbf P}{\partial\theta_i}$ | `compute_dP_dparam(param, i, …)` |

## Models

### `StVK` — Saint-Venant–Kirchhoff

The simplest large-strain model: linear elasticity written in the Green strain
$\mathbf E=\tfrac12(\mathbf F^\top\mathbf F-\mathbf I)$.

$$\Psi_{\text{StVK}} = \mu\,\lVert\mathbf E\rVert_F^2 + \tfrac{\lambda}{2}\,\operatorname{tr}(\mathbf E)^2.$$

Cheap and rotation-correct, but it *softens* (and collapses) under strong compression. The
implementation evaluates an algebraically equivalent **invariant form** in the singular
values $\mathbf S$ of $\mathbf F$ ($I_1=\operatorname{tr}\mathbf S$,
$I_2=\operatorname{tr}\mathbf S^2$, $I_3=\det\mathbf S$):
`elastic/elasticModel3DSTVKMaterial.cpp:85`.

### `StableNeo` — Stable Neo-Hookean

Neo-Hookean energy rewritten by Smith et al. (2018) to be well-defined and stable through
inversion ($\det\mathbf F\le 0$). With $\hat\lambda=\lambda+\mu$ and $r=\mu/\hat\lambda$:

$$\Psi_{\text{SNH}} = \tfrac12\Big[\mu\,(I_C-3) + \hat\lambda\,(J-1-r)^2\Big] - \tfrac12\hat\lambda r^2, \qquad I_C=\lVert\mathbf F\rVert_F^2,\ \ J=\det\mathbf F,$$

$$\mathbf P = \mu\,\mathbf F + \hat\lambda\,(J-1-r)\,\frac{\partial J}{\partial\mathbf F}.$$

The $(J-1-r)$ rest-stable volume term and the constant offset (so $\Psi(\mathbf I)=0$) are
the "stable" modifications. Recommended default for volumetric soft bodies.
`elastic/elasticModelStableNeoHookeanMaterial.cpp:33-51,65-76`.

### `LinearElastic` — linearized elasticity

Small-strain energy in $\boldsymbol\varepsilon=\tfrac12(\mathbf F+\mathbf F^\top)-\mathbf I$:

$$\Psi_{\text{lin}} = \mu\,\lVert\boldsymbol\varepsilon\rVert_F^2 + \tfrac{\lambda}{2}\operatorname{tr}(\boldsymbol\varepsilon)^2, \qquad \mathbf P = 2\mu\,\boldsymbol\varepsilon + \lambda\operatorname{tr}(\boldsymbol\varepsilon)\,\mathbf I.$$

Constant tangent (fast), but **not** rotation-invariant — only valid for tiny rotations.
`elastic/elasticModelLinearMaterial.cpp:13-30`.

### `MooneyRivlin` — invariant hyperelasticity

Rubber-like model in the isochoric invariants $\bar I_1,\bar I_2$ of
$\mathbf C=\mathbf F^\top\mathbf F$ plus a volume term in $J$. Full energy and the
$E,\nu\to$ coefficient mapping are documented at
[`../mesh/volume/material.md`](../mesh/volume/material.md) (built from a
`MooneyRivlinMaterial`); core implementation `elastic/elasticModel3DMooneyRivlin.cpp:114-155`.

### `StVKVolume` — StVK with a volume penalty

A composite (`ElasticModelCombinedMaterial<2>`,
`elastic/elasticModelFactory.cpp:56-60`) of an invariant-based StVK deviatoric energy and a
quadratic volume penalty

$$\Psi_{\text{vol}} = \tfrac12\,s\,(\det\mathbf F - 1)^2$$

(`elastic/elasticModelVolumeMaterial.cpp:13-18`, $s$ from the material's compression ratio),
with stress $\mathbf P_{\text{vol}} = s\,(\det\mathbf F-1)\,\partial(\det\mathbf F)/\partial\mathbf F$.
Use it to stiffen volume preservation on top of StVK.

### `KoiterStVK` — thin-shell StVK

A **shell** model (pair with [`KoiterShell`](formulations.md)). Instead of a 3D
$\mathbf F$, it acts on the surface's **first** and **second fundamental forms** — the
metric $\mathbf a$ (membrane stretch) and curvature $\mathbf b$ (bending) — measured against
their rest values $\bar{\mathbf a},\bar{\mathbf b}$:

$$\Psi_{\text{Koiter}} = \underbrace{\Psi_a\!\big(\mathbf a-\bar{\mathbf a}\big)}_{\text{membrane (StVK)}} + \underbrace{\Psi_b\!\big(\mathbf b-\bar{\mathbf b}\big)}_{\text{bending (StVK)}}.$$

Unlike the volumetric models it **exposes 5 optimizable channels** — `E_membrane`,
`nu_membrane`, `E_bending`, `nu_bending`, `thickness` — sourced from an `ENuh` material
(`elastic/elasticModel2DFundamentalFormsSTVK.h:36`,
`elastic/elasticModelFactory.cpp:164-165`).

## Reference

| Python | $\Psi$ kind | C++ class | Elastic channels | Coefficients from |
|---|---|---|---|---|
| `StVK` | Green-strain | `ElasticModel3DSTVKMaterial` | 0 | mesh material ($\lambda,\mu$) |
| `StableNeo` | stable neo-Hookean | `ElasticModelStableNeoHookeanMaterial` | 0 | mesh material ($\lambda,\mu$) |
| `LinearElastic` | small-strain | `ElasticModelLinearMaterial` | 0 | mesh material ($\lambda,\mu$) |
| `MooneyRivlin` | invariant | `ElasticModel3DMooneyRivlin` | 0 | `MooneyRivlinMaterial` |
| `StVKVolume` | StVK + $\tfrac12 s(J-1)^2$ | `ElasticModelCombinedMaterial<2>` | 0 | mesh material |
| `KoiterStVK` | shell (fundamental forms) | `ElasticModel2DFundamentalFormsSTVK` | 5 | `ENuh` material |

All inherit `ElasticModel`; `.name` returns the C++ model id (`"stvk"`, `"stable_neo"`,
`"linear"`, `"mooney_rivlin"`, `"stvk_vol"`, `"koiter_stvk"`;
`elastic/elasticModelFactory.cpp:106-136`).

## Quick reference

```python
from pypgo.fem import StableNeo, StVK, KoiterStVK

mat = StableNeo()        # volumetric default
mat.name                 # "stable_neo"
# Channels are model-defined; query against a mesh:
StVK()._core_obj.num_channels(sim_mesh._core_obj)   # -> 0 (Lamé come from the mesh material)
KoiterStVK()._core_obj.num_channels(sim_mesh._core_obj)  # -> 5
```

## Further reading

- [`formulations.md`](formulations.md) — how $\mathbf F$ reaches these models.
- [`fields.md`](fields.md) — what the `param` channels hold and how they are optimized.
- [`plastic.md`](plastic.md) — why the energy sees $\mathbf F_e$, not $\mathbf F$.
- [`../mesh/volume/material.md`](../mesh/volume/material.md) — $E,\nu\to\lambda,\mu$ and the
  Mooney-Rivlin energy in full.
