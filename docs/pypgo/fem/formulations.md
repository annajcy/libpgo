# `pypgo.fem.formulations` — Discretization

## Purpose

A **formulation** is the finite-element discretization: it decides how a continuous
displacement field $\mathbf u(\mathbf X)$ is represented by a finite set of nodal DOFs, how
the deformation gradient $\mathbf F$ is recovered inside each element, and how the element
energy integral is approximated. It is the bridge between the continuum law
$\Psi(\mathbf F)$ ([`elastic.md`](elastic.md)) and the discrete energy $E(\mathbf u)$
([`energy.md`](energy.md)).

In the C++ engine a formulation is a **composition of four reusable subsystems**, each
answering one question of the discrete theory:

| Subsystem | Question it answers | C++ |
|---|---|---|
| **shape function** | how do nodal values interpolate inside an element? | `formulations/shapeFunction/` |
| **quadrature** | how is $\int_e\Psi\,dV$ approximated? | `formulations/quadrature/` |
| **element mapping** | how does $\mathbf F$ come from local element DOFs? | `deformation/volume/`, `deformation/shell/` |
| **DOF layout** | how do element DOFs map into the global vector? | `formulations/dof/` |

## ElementMapping inside an element

Within an element, world position interpolates the nodal positions $\mathbf x_a$ through the
**shape functions** $N_a(\boldsymbol\xi)$ defined on a reference element $\boldsymbol\xi$:

$$\mathbf x(\boldsymbol\xi)=\sum_a N_a(\boldsymbol\xi)\,\mathbf x_a, \qquad \mathbf F=\sum_a \mathbf x_a\otimes\nabla_{\mathbf X}N_a, \qquad \nabla_{\mathbf X}N_a=\mathbf D_m^{-\top}\,\frac{\partial N_a}{\partial\boldsymbol\xi},$$

where $\mathbf D_m=\partial\mathbf X/\partial\boldsymbol\xi$ is the **rest Jacobian**. The
`VolumetricElementMapping` precomputes the rest-only quantities once and then evaluates $\mathbf F$
per quadrature point as $\mathbf F=\mathbf x\,(\partial N/\partial\boldsymbol\xi)^\top\mathbf D_m^{-1}$
(`src/core/solidDeformationModel/deformation/volume/volumetricElementMapping.h:31,49`). The cached
pieces are `restDmInv` ($\mathbf D_m^{-1}$), `dN_dX` ($\nabla_{\mathbf X}N_a$),
`rest_dFdx` ($\partial\mathbf F/\partial\mathbf x$, constant per element), and
`weightDetJ` $=|\det\mathbf D_m|\,w_q$ — the rest volume carried by quadrature point $q$
(`…/volumetricElementMapping.h:22-28`). The element energy is then simply

$$E_e=\sum_q \underbrace{|\det\mathbf D_m^{q}|\,w_q}_{\texttt{weightDetJ}}\;\Psi\big(\mathbf F_e(\boldsymbol\xi_q)\big).$$

## Element operators: energy, force, stiffness

Differentiating $E_e$ through the chain $\mathbf u\to\mathbf F\to\Psi$ gives the element's
internal force and stiffness. Writing $J_q=|\det\mathbf D_m^q|\,w_q$ and taking the purely
elastic case ($\mathbf F_e=\mathbf F$, see [`plastic.md`](plastic.md) for the general split):

$$E_e=\sum_q J_q\,\Psi(\mathbf F^q), \qquad \frac{\partial E_e}{\partial\mathbf x}=\sum_q J_q\Big(\frac{\partial\mathbf F}{\partial\mathbf x}\Big)^{\!\top}\!\mathbf P^q, \qquad \frac{\partial^2 E_e}{\partial\mathbf x^2}=\sum_q J_q\Big(\frac{\partial\mathbf F}{\partial\mathbf x}\Big)^{\!\top}\frac{\partial^2\Psi}{\partial\mathbf F^2}\Big|_{\mathbf F^q}\frac{\partial\mathbf F}{\partial\mathbf x},$$

with $\mathbf P=\partial\Psi/\partial\mathbf F$ from [`elastic.md`](elastic.md) and
$\partial\mathbf F/\partial\mathbf x$ the constant (per element) `rest_dFdx`. The
implementation folds $J_q$ and the rest shape functions into $\mathbf B_m^q=J_q\,\partial N/\partial\mathbf X$
so the force is the compact product $\mathbf P^q\mathbf B_m^q$
(`deformation/volume/volumetricDeformationModel.cpp:122-177`).

> With plasticity the same formulas hold after $\mathbf F\to\mathbf F_e$,
> $J_q\to J_q\det\mathbf F_p$, $\partial\mathbf F/\partial\mathbf x\to\partial\mathbf F_e/\partial\mathbf x$,
> and $\mathbf B_m^q\to\det\mathbf F_p\,\mathbf F_p^{-\top}\mathbf B_m^q$
> (`…/volumetricDeformationModel.cpp:110-114`).

### ElementMapping ↔ function

| Quantity | Formula | C++ |
|---|---|---|
| rest Jacobian | $\mathbf D_m=\partial\mathbf X/\partial\boldsymbol\xi$ | `VolumetricElementMapping` (`restDmInv` $=\mathbf D_m^{-1}$) |
| shape gradients | $\nabla_{\mathbf X}N_a=\mathbf D_m^{-\top}\partial N_a/\partial\boldsymbol\xi$ | `VolumetricElementMapping` (`dN_dX`) |
| deformation gradient | $\mathbf F=\mathbf x\,(\partial N/\partial\boldsymbol\xi)^{\!\top}\mathbf D_m^{-1}$ | `computeFref` / `computeF` |
| $\mathbf F$ sensitivity | $\partial\mathbf F/\partial\mathbf x$ (const.) | `computedFrefdx` / `rest_dFdx` |
| quadrature weight | $J_q=|\det\mathbf D_m^q|\,w_q$ | `weightDetJ(q)` |
| element energy | $E_e=\sum_q J_q\,\Psi$ | `computeEnergy` |
| element force | $\partial E_e/\partial\mathbf x=\sum_q\mathbf P^q\mathbf B_m^q$ | `compute_dE_dx` / `computeForceFromP` |
| element stiffness | $\partial^2E_e/\partial\mathbf x^2$ | `compute_d2E_dx2` |
| mass matrix | $M_{ab}=\int\rho N_aN_b\,dV$ | `buildMassMatrix` / `mass_matrix` |
| body force | $\mathbf f=\mathbf M\mathbf a$ | `buildBodyForce` / `body_force` |

## The four subsystems

### ShapeFunction — `formulations/shapeFunction/`

Shape functions $N_a$ and their reference gradients $\partial N_a/\partial\boldsymbol\xi$
(`ShapeFunction`, `formulations/shapeFunction/shapeFunction.h`). The three concrete bases differ in *order* and
*continuity*:

- **`TetLinearShapeFunction`** — 4 nodes, linear. $\mathbf F$ is **constant** over the tet (one material
  point suffices). The classic, robust workhorse.
- **`CubicLinearShapeFunction`** — 8 nodes, trilinear tensor product on $[0,1]^3$. $\mathbf F$
  varies across the hex.
- **`CubicTricubicHermiteShapeFunction`** — 64 scalar functions = 8 corners × 8 **Hermite modes**
  (value; the three first derivatives $\partial_\xi,\partial_\eta,\partial_\zeta$; the three
  mixed seconds; the full mixed third). This gives a $C^1$ (smooth-derivative) field — the
  shape-function basis for high-order/smooth simulation.

### Quadrature — `formulations/quadrature/`

Points $\boldsymbol\xi_q$ and weights $w_q$ for $\int_e\Psi\,dV\approx\sum_q w_q\Psi(\boldsymbol\xi_q)$
(`Quadrature`, `formulations/quadrature/quadrature.h`). Accuracy must match the integrand's
polynomial degree:

- **`TetLinearDefaultQuadrature`** — 1 point at the centroid ($w=\tfrac16$); exact because the
  P1 integrand is constant.
- **`GaussLegendreHexQuadrature2`** — $2^3=8$ points; enough for trilinear hexes.
- **`GaussLegendreHexQuadrature4`** — $4^3=64$ points; the tricubic Hermite deformation
  gradient is high-order, so $2^3$ would **under-integrate** (rank-deficient stiffness).

### ElementMapping — `deformation/volume/`, `deformation/shell/`

The `VolumetricElementMapping` above: rest-geometry precomputation and the $\mathbf F$,
$\partial\mathbf F/\partial\mathbf x$ mapping shared by all volumetric formulations.
Shells use `ShellElementMapping` implementations such as `KoiterShellElementMapping` for
fundamental-form mapping.
Small tet-linear helper functions that do not need a full `VolumetricElementMapping` object live
next to `TetLinearShapeFunction`.

### DOF layout — `formulations/dof/`

`DofLayout` (`formulations/dof/dofLayout.h`) is the *assembly bookkeeping*: it maps an
element's local DOFs to global indices, **gathers** $\mathbf x_{\text{loc}}=\mathbf P\,\mathbf x$,
**scatters** gradients $\mathbf P^\top$, and stamps each element's Hessian block into the
global sparsity pattern. This is precisely the step that stitches the per-element $E_e$ into
$E(\mathbf u)=\sum_e E_e$.

- **`Vertex3DofLayout`** — the default: one mesh vertex carries 3 DOFs,
  $\text{global}=v\cdot3+c$. Used by tet P1, trilinear hex, and the Koiter shell.
- **`CubicTricubicHermiteDofLayout`** — one vertex carries **24** DOFs (8 Hermite modes × 3
  coords), $\text{global}=v\cdot24+\text{mode}\cdot3+c$; 192 local DOFs per hex. A
  formulation overrides `createDofLayout` / `buildGlobalRestDofs` to opt into this
  (`formulations/formulation/volumetricFormulation/cubicTricubicHermiteFormulation.*`).

## The Python wrappers

`Formulation` exposes `.name` (the C++ id). `VolumetricFormulation` adds **dynamics
operators** used for time integration (these are independent of the elastic energy):

| Method | Math | C++ |
|---|---|---|
| `mass_matrix(volume)` | $M_{ab}=\int_\Omega \rho\,N_a N_b\,dV$ | `VolumetricFormulation::buildMassMatrix` |
| `body_force(volume, acceleration)` | $\mathbf f=\mathbf M\,\mathbf a$ (e.g. gravity) | `…::buildBodyForce` |
| `surface_embedding_matrix(volume, surface_vertices)` | barycentric map: surface DOFs $\to$ volume DOFs | `…::buildSurfaceEmbeddingMatrix` |

`CubicTricubicHermiteFormulation` overrides all three with Hermite-aware versions
(`formulations/formulation/volumetricFormulation/cubicTricubicHermiteFormulation.*`).

### Concrete formulations

| Python | ShapeFunction | Quadrature | DOF layout | Local DOFs | `.name` |
|---|---|---|---|---|---|
| `TetLinear` | linear tet (4) | 1-pt centroid | `Vertex3` | 12 | `tet_linear` |
| `CubicLinear` | trilinear hex (8) | $2^3$ Gauss | `Vertex3` | 24 | `cubic_linear` |
| `CubicTricubicHermite` | tricubic Hermite (64) | $4^3$ Gauss | `CubicTricubicHermite` (24/vtx) | 192 | `cubic_tricubic_hermite` |
| `KoiterShell` | Koiter shell | — | `Vertex3` | 18 | `shell_koiter` |

(`formulations/formulation/volumetricFormulation/`, `formulations/formulation/shellFormulation/`.)
`TetLinear`/`CubicLinear`/`CubicTricubicHermite`
are volumetric; `KoiterShell` is a shell — pair it with [`KoiterStVK`](elastic.md).

## Quick reference

```python
from pypgo.fem import TetLinear, CubicTricubicHermite

formulation = TetLinear()
formulation.name                       # "tet_linear"

# Dynamics operators (need a VolumeMesh, not the energy):
M = formulation.mass_matrix(volume)                       # SparseMatrix
f = formulation.body_force(volume, [0.0, -9.8, 0.0])      # gravity load
```

## Further reading

- [`elastic.md`](elastic.md) — the $\Psi(\mathbf F)$ that each quadrature point evaluates.
- [`energy.md`](energy.md) — how `DofLayout` assembly yields $E(\mathbf u)$, $\mathbf g$, $\mathbf K$.
- [`overview.md`](overview.md) — where formulations sit in the full pipeline.
