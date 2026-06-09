# `pypgo.fem.energy` — Assembly & Optimization

## Purpose

`energy.py` is where the modeling decisions become a **callable energy**. It assembles the
per-element contributions ([`formulations.md`](formulations.md)) directly from a
`SimulationMesh`, material models, and parameter-field descriptors, then exposes the global
potential and its derivatives through the optimizer's standard interface. It builds two kinds
of energy, one for each of the two variational problems:

- `deformation_energy(...)` → **`DeformationEnergy`** — the forward problem, variable
  $\mathbf u$.
- `plastic_material_energy(...)` → **`PlasticMaterialEnergy`** — the inverse problem,
  variable = the plastic parameter field, with $\mathbf u$ fixed.

## Forward energy: $E(\mathbf u)$

$$E(\mathbf u)=\sum_e\sum_q |\det\mathbf D_m^{q}|\,w_q\;\Psi\big(\mathbf F_e(\mathbf u)\big), \qquad \mathbf g=\frac{\partial E}{\partial \mathbf u}\ \text{(internal force)}, \qquad \mathbf K=\frac{\partial^2 E}{\partial \mathbf u^2}\ \text{(tangent stiffness)}.$$

The `DeformationModelAssembler` loops elements, evaluating each `VolumetricDeformationModel`
(`compute_dE_dx`, `compute_d2E_dx2`) and scattering through the `DofLayout` into the global
gradient and the Hessian sparsity template `KTemplate`
(`src/core/solidDeformationModel/deformation/deformationModelAssembler.h:57`). The result is
a `DeformationEnergy`, whose `state_kind` is `"displacement"`
(`energy/deformationModelEnergy.h:37`).

### Building it

```python
from pypgo.fem import (
    deformation_energy, DeformationOptions, TetP1,
    StableNeo, VolumetricPlasticity, ElementwiseField,
)

energy = deformation_energy(
    sim_mesh,
    elastic=StableNeo(),
    elastic_field=ElementwiseField(),
    plastic=VolumetricPlasticity(dofs=6),
    plastic_field=ElementwiseField(),
    formulation=TetP1(),
    options=DeformationOptions(enforce_spd=True, enable_material_max_step=True),
)
```

`DeformationOptions` controls two solver-critical behaviors:

| Option | Effect | Why |
|---|---|---|
| `enforce_spd` | each element returns an SPD projection of its tangent | guarantees $\mathbf K\succeq 0$ so a Newton step is a descent direction even far from equilibrium (analytic eigenvalue clamp for Stable Neo-Hookean, `elastic/elasticModelStableNeoHookeanMaterial.cpp:126-180`) |
| `enable_material_max_step` | bounds the line-search step so the configuration stays admissible | keeps $\det\mathbf F>0$ (no element inversion); polynomial root-finding in `deformation/materialMaxStepPolynomialUtils.*`, surfaced as `max_step` |

### Evaluating it — the `PotentialEnergy` interface

`DeformationEnergy` is defined in `pypgo/fem/energy.py` (and re-exported as
`pypgo.fem.DeformationEnergy`) and inherits the same `PotentialEnergy` interface
as every pypgo energy (`pypgo/energy.py`), so it drops straight into the optimizer:

```python
import numpy as np
u  = np.zeros(energy.num_dofs)
E  = energy.value(u)          # scalar
g  = energy.gradient(u)       # internal force vector
K  = energy.hessian(u)        # tangent stiffness (SparseMatrix)
a  = energy.max_step(u, du)   # admissible step along du (∞ if unbounded)
x0 = energy.zero_state()      # rest displacement

energy.rest_position          # (num_vertices, 3) undeformed positions
```

## Inverse problem: material parameter optimization

The same assembled energy can be differentiated with respect to **material parameters**, not
just displacement. Internally each element computes (notation: $a$ = plastic params, $b$ =
elastic params; `deformation/volume/volumetricDeformationModel.h:37-44`):

$$\frac{\partial E}{\partial a},\ \frac{\partial^2 E}{\partial a^2},\qquad \frac{\partial E}{\partial b},\ \frac{\partial^2 E}{\partial b^2},\qquad \underbrace{\frac{\partial^2 E}{\partial \mathbf x\,\partial a},\ \frac{\partial^2 E}{\partial \mathbf x\,\partial b}}_{\text{displacement–parameter coupling}}.$$

Explicitly, with $V_q=J_q\det\mathbf F_p$ the deformed-rest volume weight, the **plastic**
gradient threads the chain rule $\partial\mathbf F_e/\partial a_i=\mathbf F\,\partial\mathbf F_p^{-1}/\partial a_i$
([`plastic.md`](plastic.md)) — note the two terms, one from the volume change and one from the
strain ($\mathbf F_p$ rescales both the measure and the elastic strain):

$$\frac{\partial E}{\partial a_i}=\sum_q\Big[\,J_q\,\frac{\partial\det\mathbf F_p}{\partial a_i}\,\Psi(\mathbf F_e^q) \;+\; V_q\,\underbrace{\mathbf P^q:\Big(\mathbf F\,\frac{\partial\mathbf F_p^{-1}}{\partial a_i}\Big)}_{\partial\Psi/\partial a_i}\Big].$$

The **elastic** gradient is simpler — the parameters enter $\Psi$ directly, so it uses the
material's own $\partial\Psi/\partial\theta_i$ ([`elastic.md`](elastic.md)):

$$\frac{\partial E}{\partial b_i}=\sum_q V_q\,\frac{\partial\Psi}{\partial\theta_i}\Big|_{\mathbf F_e^q}.$$

The displacement–plastic coupling combines stress sensitivity and second-order kinematics:

$$\frac{\partial^2 E}{\partial\mathbf x\,\partial a_i}=\sum_q\Big[\,J_q\frac{\partial\det\mathbf F_p}{\partial a_i}\Big(\frac{\partial\mathbf F_e}{\partial\mathbf x}\Big)^{\!\top}\!\mathbf P^q + V_q\Big(\big(\tfrac{\partial\mathbf F_e}{\partial\mathbf x}\big)^{\!\top}\tfrac{\partial\mathbf P}{\partial a_i} + \big(\tfrac{\partial^2\mathbf F_e}{\partial\mathbf x\,\partial a_i}\big)^{\!\top}\!\mathbf P^q\Big)\Big],$$

with $\partial\mathbf P/\partial a_i=\frac{\partial^2\Psi}{\partial\mathbf F^2}\,\frac{\partial\mathbf F_e}{\partial a_i}$
(`deformation/volume/volumetricDeformationModel.cpp:354-477`). All three are mapped from
local parameters to the global vector by the field Jacobian
$\mathbf D=\partial(\text{local})/\partial(\text{global})$ as $\mathbf D^\top(\cdot)$ /
$\mathbf D^\top(\cdot)\mathbf D$.

The assembler folds these into global operators via the field's `computeDerivative`
([`fields.md`](fields.md)) and dedicated sparsity templates — `dfdaTemplate`,
`dfdbTemplate`, `d2Eda2Template` — with entry points `computePlasticGradient`,
`computePlasticHessian`, `compute_df_da`, `compute_df_db`
(`deformation/deformationModelAssembler.h:42-47,57-62`).

### The plastic path (exposed in Python)

`DeformationEnergy` surfaces the plastic sensitivities directly:

```python
energy.num_plastic_dofs
energy.plastic_gradient(u)    # ∂E/∂a          at displacement u
energy.plastic_hessian(u)     # ∂²E/∂a²        (SparseMatrix)
energy.plastic_jacobian(u)    # ∂²E/∂x∂a       (= compute_df_da, the displacement–plastic coupling)
```

To optimize the plastic field as a variational problem, wrap it as a `PlasticMaterialEnergy`,
whose **optimization variable is the plastic field** and whose displacement is frozen
(`state_kind == "generic"`, `energy/plasticMaterialEnergy.h:36`):

```python
from pypgo.fem import plastic_material_energy

pe = plastic_material_energy(config, energy, fixed_displacement=u)
p0 = pe.zero_state()
pe.value(p)      # E(u_fixed, p)
pe.gradient(p)   # ∂E/∂p
pe.hessian(p)    # ∂²E/∂p²
```

Minimizing `pe` over `p` is the energy-based plastic update of [`plastic.md`](plastic.md).

> The C++ assembler also computes the **elastic**-parameter sensitivities
> (`compute_dE_db`, `compute_df_db`, `getNumElasticParams`), so inverse design over elastic
> coefficients is supported by the engine; the current Python convenience surface wires up
> the **plastic** field. Elastic-parameter optimization can be driven through the same
> assembler entry points.

## Formula ↔ function reference

Every assembled quantity, its Python entry point, and the C++ element/assembler operators.
$J_q=|\det\mathbf D_m^q|\,w_q$, $V_q=J_q\det\mathbf F_p$.

| Quantity | Formula | Python | C++ (element → assembler) |
|---|---|---|---|
| energy | $E=\sum_e\sum_q V_q\,\Psi(\mathbf F_e^q)$ | `value(u)` | `computeEnergy` |
| internal force | $\mathbf g=\partial E/\partial\mathbf u$ | `gradient(u)` | `compute_dE_dx` → scatter |
| tangent stiffness | $\mathbf K=\partial^2E/\partial\mathbf u^2$ | `hessian(u)` | `compute_d2E_dx2` → `KTemplate` |
| admissible step | $\alpha^\star:\ \det(\mathbf F+\alpha\Delta\mathbf F)>0$ | `max_step(u, du)` | `computeLocalMaxStepSize` → `computeMaxStepLimit` |
| plastic gradient | $\partial E/\partial a$ | `plastic_gradient(u)` | `compute_dE_da` → `computePlasticGradient` |
| plastic Hessian | $\partial^2E/\partial a^2$ | `plastic_hessian(u)` | `compute_d2E_da2` → `computePlasticHessian` (`d2Eda2Template`) |
| displ.–plastic coupling | $\partial^2E/\partial\mathbf u\,\partial a$ | `plastic_jacobian(u)` | `compute_d2E_dxda` → `compute_df_da` (`dfdaTemplate`) |
| elastic gradient | $\partial E/\partial b$ | — | `compute_dE_db` |
| elastic Hessian | $\partial^2E/\partial b^2$ | — | `compute_d2E_db2` |
| displ.–elastic coupling | $\partial^2E/\partial\mathbf u\,\partial b$ | — | `compute_d2E_dxdb` → `compute_df_db` (`dfdbTemplate`) |
| plastic–elastic cross | $\partial^2E/\partial a\,\partial b$ | — | `compute_d2E_dadb` |
| von Mises stress | $\sigma_{vM}(\boldsymbol\sigma)$, $\boldsymbol\sigma=\mathbf P\mathbf F_e^{\!\top}/\det\mathbf F_e$ | — | `vonMisesStress` |
| max principal strain | $\lambda_{\max}\!\big(\tfrac12(\mathbf F_e^{\!\top}\mathbf F_e-\mathbf I)\big)$ | — | `maxStrain` |

Python methods are on `DeformationEnergy` (`pypgo/fem/energy.py`); C++ element operators on
`VolumetricDeformationModel` and assembler methods on `DeformationModelAssembler`.

## Further reading

- [`formulations.md`](formulations.md) — the element integrals and `DofLayout` assembly.
- [`fields.md`](fields.md) — `computeDerivative`, the bridge to parameter gradients.
- [`plastic.md`](plastic.md) — what the plastic optimization variable means.
- [`overview.md`](overview.md) — the two-variational-problems picture.
