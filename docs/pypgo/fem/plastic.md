# `pypgo.fem.plastic` — Plasticity

## Purpose

Plasticity models **permanent** deformation: the part of a body's shape change that does not
spring back. The standard device is a **multiplicative split** of the deformation gradient
into an elastic and a plastic factor,

$$\mathbf F = \mathbf F_e\,\mathbf F_p, \qquad\Longrightarrow\qquad \mathbf F_e = \mathbf F\,\mathbf F_p^{-1}.$$

Only the elastic factor $\mathbf F_e$ stores energy, so every elastic model in
[`elastic.md`](elastic.md) is evaluated at $\mathbf F_e$, never at the raw $\mathbf F$. The
plastic factor $\mathbf F_p$ is a small per-element state that the plastic model
parametrizes; `plastic.py` chooses that parametrization and its size.

## What a plastic model provides

A C++ `PlasticModel3DDeformationGradient` exposes the plastic factor and its inverse from a
parameter vector (`src/core/solidDeformationModel/plastic/plasticModel3DDeformationGradient.h:22-31`):

```cpp
void computeA   (const double *param, double A[9])    const;  // A   = F_p
void computeAInv(const double *param, double AInv[9]) const;  // AInv = F_p^{-1}
```

plus derivatives ($\partial\mathbf F_p^{-1}/\partial a$, $\partial\det\mathbf F_p/\partial a$,
…) so the assembler can differentiate the energy with respect to the plastic parameters $a$
(see [`energy.md`](energy.md)). The rest state is $\mathbf F_p=\mathbf I$
(`defaultParams` → identity, `…/plasticModel3DDeformationGradient.h:44-48`); a model with
zero parameters leaves $\mathbf F_e=\mathbf F$ and the simulation is purely elastic.

### Plastic operators ↔ function

Because $\mathbf F_e=\mathbf F\,\mathbf F_p^{-1}(a)$, every energy derivative with respect to
the plastic parameters $a$ flows through the plastic factor by the chain rule
(`deformation/volume/volumetricDeformationModel.cpp:636-688`):

$$\frac{\partial\mathbf F_e}{\partial a_i}=\mathbf F\,\frac{\partial\mathbf F_p^{-1}}{\partial a_i}, \qquad \frac{\partial\Psi}{\partial a_i}=\mathbf P:\frac{\partial\mathbf F_e}{\partial a_i}, \qquad \frac{\partial\mathbf F_e}{\partial\mathbf x}=\frac{\partial\mathbf F}{\partial\mathbf x}\,\mathbf F_p^{-1}.$$

The model supplies the plastic-factor pieces these formulas need
(`plastic/plasticModel3DDeformationGradient.h:22-48`):

| Quantity | Symbol | C++ method |
|---|---|---|
| plastic factor | $\mathbf F_p=A(a)$ | `computeA(param, A)` |
| inverse | $\mathbf F_p^{-1}=A^{-1}(a)$ | `computeAInv(param, AInv)` |
| volume change | $\det\mathbf F_p$ | `compute_detA(param)` |
| inverse sensitivity | $\partial\mathbf F_p^{-1}/\partial a_i$ | `compute_dAInv_da(param, i, ret)` |
| 2nd inverse sensitivity | $\partial^2\mathbf F_p^{-1}/\partial a_i\partial a_j$ | `compute_d2AInv_da2(param, i, j, ret)` |
| volume sensitivity | $\partial\det\mathbf F_p/\partial a_i$ | `compute_ddetA_da(param, …)` |
| 2nd volume sensitivity | $\partial^2\det\mathbf F_p/\partial a_i\partial a_j$ | `compute_d2detA_da2(param, …)` |
| parametrization | $a\leftrightarrow\mathbf F_p$, rest $=\mathbf I$ | `toParam` / `defaultFp` / `defaultParams` |

The element assembly that consumes these into $\partial E/\partial a$, $\partial^2E/\partial a^2$,
$\partial^2E/\partial\mathbf x\,\partial a$ is detailed in [`energy.md`](energy.md).

## Volumetric plasticity

`VolumetricPlasticity(dofs)` with `dofs ∈ {0, 3, 6}` — richer DOFs = more general $\mathbf F_p$:

| `dofs` | $\mathbf F_p$ parametrization | C++ class |
|---|---|---|
| `0` | none ($\mathbf F_p\equiv\mathbf I$) — purely elastic | `PlasticModel3DConstant` |
| `3` | 3-parameter plastic factor | `PlasticModel3D3DOF` |
| `6` | symmetric plastic factor — the 6 upper-triangular entries $(F_{p,00},F_{p,01},F_{p,02},F_{p,11},F_{p,12},F_{p,22})$ | `PlasticModel3D6DOF` |

The 6-DOF case (`plastic/plasticModel3D6DOF.h:21,50-60`) is the general anisotropic plastic
stretch; the 3-DOF case is a restricted (cheaper) subspace. Default is `dofs=6`.

## Shell plasticity

`ShellPlasticity(dofs)` with `dofs ∈ {0, 1}` acts on the shell's fundamental forms rather
than a 3D gradient:

| `dofs` | meaning | C++ class |
|---|---|---|
| `0` | none | (constant) |
| `1` | scalar **uniform stretch** of the membrane | `PlasticModel2DFundamentalFormsUniformStretch` |

Default is `dofs=1`. Pair shell plasticity with the [`KoiterShell`](formulations.md)
formulation and [`KoiterStVK`](elastic.md) material.

## Optimizing the plastic field

The per-element plastic parameters live in a parameter field ([`fields.md`](fields.md)) and
are themselves an **optimization variable**. Holding the displacement $\mathbf u$ fixed and
minimizing the deformation energy over the plastic field is exactly the energy-based plastic
update — exposed as [`plastic_material_energy`](energy.md). This is how the engine performs
return-mapping-style plasticity as a variational problem.

## Quick reference

```python
from pypgo.fem import VolumetricPlasticity, ShellPlasticity

plastic = VolumetricPlasticity()        # dofs=6 (anisotropic)
plastic.name                            # C++ model id
plastic.dofs                            # 6  → plastic field has 6 channels per element

VolumetricPlasticity(dofs=0)            # disable plasticity (F_e = F)
ShellPlasticity(dofs=1)                 # membrane uniform-stretch plasticity
```

`VolumetricPlasticity` rejects any `dofs` outside `{0,3,6}`; `ShellPlasticity` outside
`{0,1}` (`pypgo/fem/plastic.py`).

## Further reading

- [`elastic.md`](elastic.md) — the energy evaluated at $\mathbf F_e$.
- [`fields.md`](fields.md) — how the per-element $\mathbf F_p$ parameters are stored.
- [`energy.md`](energy.md) — `plastic_material_energy` and the plastic derivatives.
