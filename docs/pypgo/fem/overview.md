# `pypgo.fem` — FEM Deformation Energy

## Purpose

`pypgo.fem` is the **construction surface for elastic / plastic deformation energy**. It
lets you assemble, from a simulation mesh, the discrete potential energy

$$E(\mathbf u) \;=\; \int_\Omega \Psi\big(\mathbf F(\mathbf u)\big)\,dV$$

whose minimizer is the deformed shape, together with its gradient (internal force) and
Hessian (tangent stiffness). The Python layer is a **thin, typed wrapper** over the C++
engine in `src/core/solidDeformationModel/`; it owns no math itself — it picks a
*constitutive model*, a *discretization (formulation)*, a *plasticity model*, and the
*material parameter fields*, binds them to one mesh, and hands the result to the pypgo
optimizer through the standard `value / gradient / hessian / max_step` interface.

This document is the map. Each section names a piece of the continuum-mechanics pipeline,
the Python module that exposes it, and the C++ subsystem that realizes it.

## The pipeline: from continuum to discrete energy

FEM elasticity is a chain of four modeling decisions and one assembly step. Read it
top-to-bottom; each row is a later document.

### 1. Kinematics — how shape becomes strain

A body has a *rest* (material) configuration $\mathbf X\in\Omega$ and a *deformed* (world)
configuration $\mathbf x = \mathbf X + \mathbf u$, where $\mathbf u$ is the **displacement**
field — the simulation unknown. The local stretching/rotation is the **deformation
gradient**

$$\mathbf F \;=\; \frac{\partial \mathbf x}{\partial \mathbf X} \;=\; \mathbf I + \nabla_{\mathbf X}\mathbf u \;\in\; \mathbb R^{3\times3}.$$

Strain is the part of $\mathbf F$ that is not a rigid rotation. Two common measures:

$$\underbrace{\mathbf E = \tfrac12(\mathbf F^\top\mathbf F - \mathbf I)}_{\text{Green–Lagrange (large strain)}}, \qquad \underbrace{\boldsymbol\varepsilon = \tfrac12(\nabla\mathbf u + \nabla\mathbf u^\top)}_{\text{infinitesimal (small strain)}}.$$

→ realized per element in [`formulations.md`](formulations.md).

### 2. Constitutive law — how strain becomes energy

A **strain energy density** $\Psi(\mathbf F)$ assigns an elastic energy (per unit rest
volume) to a deformation. Its derivatives are the physics:

$$\mathbf P = \frac{\partial \Psi}{\partial \mathbf F}\ \text{(1st Piola–Kirchhoff stress)}, \qquad \frac{\partial^2\Psi}{\partial \mathbf F^2}\ \text{(material tangent)}.$$

The choice of $\Psi$ (StVK, Stable Neo-Hookean, Mooney-Rivlin, …) is the *material model*.
→ [`elastic.md`](elastic.md).

### 3. Plasticity — permanent deformation

For materials that flow, $\mathbf F$ splits multiplicatively into elastic and plastic parts

$$\mathbf F = \mathbf F_e\,\mathbf F_p, \qquad \mathbf F_e = \mathbf F\,\mathbf F_p^{-1},$$

and the energy is stored only in the **elastic** part: $\Psi(\mathbf F_e)$. The per-element
$\mathbf F_p$ is a small parameter field. → [`plastic.md`](plastic.md).

### 4. Material parameter fields — where the coefficients live

The coefficients inside $\Psi$ (Lamé $\lambda,\mu$; shell thickness; activation; the plastic
$\mathbf F_p$) are stored as **fields over the mesh** — one shared set (constant) or one set
per element (elementwise). → [`fields.md`](fields.md).

### 5. Assembly — the discrete energy and its derivatives

Summing the element integrals gives the global energy, force, and stiffness consumed by the
solver. → state container in [`state.md`](state.md), energy in [`energy.md`](energy.md).

$$E(\mathbf u)=\sum_e \sum_q w_q\,|\det \mathbf D_m^{q}|\;\Psi\big(\mathbf F_e(\mathbf u)\big), \qquad \mathbf g=\nabla_{\mathbf u}E,\qquad \mathbf K=\nabla^2_{\mathbf u}E.$$

### Module ↔ math map

| Python module | Math object | Pipeline stage | C++ home |
|---|---|---|---|
| [`elastic.py`](elastic.md) | $\Psi(\mathbf F)$, $\mathbf P=\partial\Psi/\partial\mathbf F$ | constitutive law | `elastic/` |
| [`formulations.py`](formulations.md) | $\mathbf F(\mathbf u)$, $N_a$, $\mathbf M$, $\mathbf f$ | discretization | `formulations/` |
| [`fields.py`](fields.md) | material coefficients + sensitivities | parameters | `formulations/parameters/` |
| [`plastic.py`](plastic.md) | $\mathbf F_p$ (plastic field) | plasticity | `plastic/` |
| [`state.py`](state.md) | $(\text{mesh},\Psi,\text{p-model},\text{fields})$ bundle | state | `deformation/deformationModelState.*` |
| [`energy.py`](energy.md) | $E(\mathbf u)$, $\mathbf g$, $\mathbf K$, param derivatives | assembly | `deformation/`, `energy/` |

## The C++ engine architecture

A `Formulation` is a **composition** of four reusable subsystems. It builds one
`DeformationModel` per element; the manager owns the per-element models plus the parameter
fields; the assembler gathers element contributions into the global operators; and the
energy class exposes them through the optimizer's `PotentialEnergy` interface.

```
Formulation  =  basis  +  quadrature  +  dof-layout  +  kernel/geometry      (formulations/)
    └─ createElement() ──▶ DeformationModel per element   (E_e, ∂E/∂x, ∂²E/∂x², parameter derivs)
          └─ DeformationModelManager   (owns element models + OptimizableField parameters)
                └─ DeformationModelAssembler   (gather/scatter ▶ global E, g, K, parameter Jacobians)
                      └─ DeformationModelEnergy / PlasticMaterialEnergy   (PotentialEnergy: value/grad/hessian)
```

Each subsystem answers exactly one question of the discrete theory:

| C++ subsystem | Mathematical role | Key types |
|---|---|---|
| `formulations/basis/` | shape functions $N_a(\boldsymbol\xi)$ and $\partial N_a/\partial\boldsymbol\xi$ — *interpolation* | `Basis`; `TetP1Basis` (4 nodes), `HexTrilinearBasis` (8), `HexTricubicHermiteBasis` (64 = 8 corners × 8 Hermite modes) |
| `formulations/quadrature/` | the integral $\int_e\!\Psi\,dV\approx\sum_q w_q\,\Psi(\boldsymbol\xi_q)$ — *numerical integration* | `Quadrature`; `TetP1DefaultQuadrature` (1 pt), `GaussLegendreHexQuadrature2`/`…4` (2³ / 4³) |
| `formulations/geometry/` + `kernels/` | rest Jacobian $\mathbf D_m=\partial\mathbf X/\partial\boldsymbol\xi$, $\mathbf F$ and $\partial\mathbf F/\partial\mathbf x$ — *kinematics* | `VolumetricKernel` |
| `formulations/dof/` | local↔global DOF map, gather/scatter, Hessian sparsity — *assembly bookkeeping* | `DofLayout`; `Vertex3DofLayout` (3/vertex), `HexTricubicHermiteDofLayout` (24/vertex) |
| `formulations/elements/` | per-element $E_e,\nabla E_e,\nabla^2E_e$ and parameter derivatives — *the element kernel* | `VolumetricDeformationModel`, `ShellDeformationModel` |
| `formulations/parameters/` | coefficient storage and $\partial(\text{value})/\partial(\text{params})$ — *material fields* | `ParameterField`, `OptimizableField` |

The four concrete formulations are just different (basis, quadrature, DOF-layout) triples
— see `src/core/solidDeformationModel/formulations/formulation.cpp:96` (`tet_p1`), `:111`
(`hex_trilinear`), `:370` (`hex_tricubic_hermite`), `:440` (`shell_koiter`).

## Two variational problems

The same assembled energy supports two optimizations, distinguished by *what is variable*:

- **Forward (simulation)** — variable is the displacement $\mathbf u$; minimize
  $E(\mathbf u)$. This is [`DeformationEnergy`](energy.md) (`state_kind == "displacement"`).
- **Inverse (material parameter optimization)** — variable is a *material parameter field*
  with $\mathbf u$ held fixed. This is [`PlasticMaterialEnergy`](energy.md)
  (`state_kind == "generic"`). The assembler provides the needed sensitivities — gradients
  $\partial E/\partial a$ (plastic), $\partial E/\partial b$ (elastic), their Hessians, and
  the displacement–parameter cross-blocks $\partial^2E/\partial\mathbf x\,\partial a$
  (`compute_df_da`, `src/core/solidDeformationModel/deformation/deformationModelAssembler.h:42`).
  The Python surface currently exposes the **plastic** path.

## Public API

Everything is importable directly from `pypgo.fem` (see `pypgo/fem/__init__.py`):

```python
from pypgo.fem import (
    # Formulations (discretization)
    TetP1, LinearCubic, TricubicHermite, KoiterShell,
    # Elastic constitutive models
    StableNeo, StVK, StVKVolume, LinearElastic, MooneyRivlin, KoiterStVK,
    # Plasticity
    VolumetricPlasticity, ShellPlasticity,
    # Parameter field descriptors
    ConstantField, ElementwiseField,
    # State + energy
    deformation_model_state, deformation_energy, plastic_material_energy,
    DeformationOptions,
)
```

### End-to-end example

```python
import numpy as np
from pypgo.fem import (
    deformation_model_state, deformation_energy,
    StableNeo, ElementwiseField, VolumetricPlasticity, TetP1,
)

# 1. Bind material + parameter fields to a SimulationMesh (built elsewhere).
state = deformation_model_state(
    sim_mesh,
    elastic=StableNeo(),       # Ψ(F): Stable Neo-Hookean
    elastic_field=ElementwiseField(),   # defaults seeded from the mesh material
    plastic=VolumetricPlasticity(dofs=0),  # purely elastic
    plastic_field=ElementwiseField(),
)

# 2. Assemble E(u) for a chosen discretization.
energy = deformation_energy(state, TetP1())

# 3. Evaluate at a displacement u (force = -gradient, stiffness K = hessian).
u = np.zeros(energy.num_dofs)
print(energy.value(u))          # scalar elastic energy
g = energy.gradient(u)          # internal force vector
K = energy.hessian(u)           # tangent stiffness (SparseMatrix)
```

## Formula ↔ function index

A master map from each mathematical object to the Python entry point and the C++ kernel that
computes it. Notation: $J_q = w_q\,|\det\mathbf D_m^q|$ is the rest quadrature weight,
$\mathbf F_e=\mathbf F\,\mathbf F_p^{-1}$ the elastic gradient, $a$/$b$ the plastic/elastic
parameters. Each row is detailed in the linked document.

| Math object | Formula | Python | C++ kernel | Doc |
|---|---|---|---|---|
| strain energy density | $\Psi(\mathbf F_e)$ | model class | `compute_psi` | [elastic](elastic.md) |
| 1st PK stress | $\mathbf P=\partial\Psi/\partial\mathbf F$ | — | `compute_P` | [elastic](elastic.md) |
| material tangent | $\partial^2\Psi/\partial\mathbf F^2$ | — | `compute_dPdF` | [elastic](elastic.md) |
| deformation gradient | $\mathbf F=\mathbf x\,(\partial N/\partial\boldsymbol\xi)^{\!\top}\mathbf D_m^{-1}$ | — | `VolumetricKernel::computeFref` | [formulations](formulations.md) |
| plastic split | $\mathbf F_p=A(a)$, $\mathbf F_e=\mathbf F\,\mathbf F_p^{-1}$ | — | `computeA` / `computeAInv` | [plastic](plastic.md) |
| element energy | $E_e=\sum_q J_q\det\mathbf F_p\;\Psi(\mathbf F_e^q)$ | — | `computeEnergy` | [formulations](formulations.md) |
| total energy | $E(\mathbf u)=\sum_e E_e$ | `energy.value(u)` | assembler `computeEnergy` | [energy](energy.md) |
| internal force | $\mathbf g=\partial E/\partial\mathbf u$ | `energy.gradient(u)` | `compute_dE_dx` | [energy](energy.md) |
| tangent stiffness | $\mathbf K=\partial^2E/\partial\mathbf u^2$ | `energy.hessian(u)` | `compute_d2E_dx2` | [energy](energy.md) |
| mass matrix | $M_{ab}=\int\rho N_aN_b\,dV$ | `mass_matrix(vol)` | `buildMassMatrix` | [formulations](formulations.md) |
| body force | $\mathbf f=\mathbf M\mathbf a$ | `body_force(vol, a)` | `buildBodyForce` | [formulations](formulations.md) |
| plastic gradient | $\partial E/\partial a$ | `energy.plastic_gradient(u)` | `compute_dE_da` | [energy](energy.md) |
| plastic Hessian | $\partial^2E/\partial a^2$ | `energy.plastic_hessian(u)` | `compute_d2E_da2` | [energy](energy.md) |
| displ.–plastic coupling | $\partial^2E/\partial\mathbf u\,\partial a$ | `energy.plastic_jacobian(u)` | `compute_d2E_dxda` / `compute_df_da` | [energy](energy.md) |
| elastic-param gradient | $\partial E/\partial b$ | (assembler) | `compute_dE_db` | [energy](energy.md) |
| admissible step | $\alpha^\star:\ \det(\mathbf F+\alpha\,\Delta\mathbf F)>0$ | `energy.max_step(u, du)` | `computeLocalMaxStepSize` | [energy](energy.md) |
| von Mises stress | $\sigma_{vM}(\boldsymbol\sigma),\ \boldsymbol\sigma=\mathbf P\mathbf F_e^{\!\top}/\det\mathbf F_e$ | — | `vonMisesStress` | [energy](energy.md) |

## Further reading

- [`elastic.md`](elastic.md) — strain energy densities $\Psi(\mathbf F)$ and stress.
- [`formulations.md`](formulations.md) — basis · quadrature · DOF layout · kinematics.
- [`fields.md`](fields.md) — material parameter fields and their sensitivities.
- [`plastic.md`](plastic.md) — the elastic/plastic split $\mathbf F=\mathbf F_e\mathbf F_p$.
- [`state.md`](state.md) — `DeformationModelState`, the mesh+material bundle.
- [`energy.md`](energy.md) — assembly, solver interface, and parameter optimization.
- [`../mesh/volume/material.md`](../mesh/volume/material.md) — mesh-level material specs
  ($E,\nu$ → Lamé $\lambda,\mu$; Mooney-Rivlin) consumed by the elastic models here.
