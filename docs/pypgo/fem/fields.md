# `pypgo.fem.fields` — Material Parameter Fields

## Purpose

The constitutive models in [`elastic.md`](elastic.md) and [`plastic.md`](plastic.md) carry
**coefficients** — Lamé $\lambda,\mu$, shell thickness, muscle activation, the plastic
$\mathbf F_p$. A *parameter field* decides how those coefficients vary over the mesh and,
crucially, makes them addressable as an optimization vector. `fields.py` provides two
**descriptors** (what you ask for) and one **runtime view** (what C++ gives back).

A field maps every *material location* to a vector of `num_channels` numbers:

$$\text{field}: (\text{element } e,\ \text{quadrature point } q)\ \longmapsto\ \boldsymbol\theta_e\in\mathbb R^{C}.$$

`num_channels` $C$ is **model-defined** — it is the model's `getNumParameters()`. For the
volumetric elastic models it is `0` (their stiffness comes from the mesh material, not the
field); for `KoiterStVK` it is `5`; for `VolumetricPlasticity(dofs=6)` it is `6`.

## Two layouts

How many independent copies of $\boldsymbol\theta$ exist is the whole distinction
(`src/core/solidDeformationModel/formulations/parameters/parameterField.h:13-49`):

| Descriptor | Meaning | Stored shape | Global DOFs | `globalDof(e,i)` |
|---|---|---|---|---|
| `ConstantField` | one set shared by **every** element | `(1, C)` | $C$ | $i$ |
| `ElementwiseField` | one set **per element** | `(num_elements, C)` | $C\cdot N_{\text{ele}}$ | $e\cdot C+i$ |

`ConstantField` is the right choice for a homogeneous material or a global design variable;
`ElementwiseField` is a spatially-varying field (e.g. per-element stiffness to be optimized).
Both accept `values=None` to let C++ seed model-appropriate defaults from the mesh/material,
or an explicit array of the shape above.

```python
from pypgo.fem import ElementwiseField, ConstantField

ElementwiseField()                       # values=None → C++ defaults
ConstantField(values=[1e6, 0.4])         # one shared (num_channels,) row
ElementwiseField(values=per_element_arr) # (num_elements, num_channels)
```

## The runtime view: `ParameterField`

After a state is built ([`state.md`](state.md)), the live C++-owned field is exposed read-only:

```python
pf = state.elastic_field          # a ParameterField
pf.domain                          # "elastic" or "plastic"
pf.model                           # e.g. "stvk", "koiter_stvk"
pf.num_elements
pf.num_value_rows                  # 1 (constant) or num_elements (elementwise)
pf.num_channels                    # C
pf.values                          # (num_value_rows, num_channels) float64 copy (property)
```

## The optimization layer: `OptimizableField`

Material-parameter optimization needs more than storage — it needs to express how the
*value at a material point* depends on the *global parameter vector*. That is the C++
`OptimizableField` interface (`…/parameterField.h:51-68`):

- `computeValue(e, q, out)` — assemble $\boldsymbol\theta$ at $(e,q)$ from the global vector
  (the forward map used while evaluating $\Psi$).
- `computeDerivative(e, q, derivOut)` — $\partial\boldsymbol\theta/\partial(\text{global params})$,
  the sparse Jacobian that lets the assembler convert per-element parameter gradients into
  global ones.
- `dofLayout()` — a `ParameterDofLayout` with `numGlobalDofs()`, `globalDof(e, i)`, and
  `gather(e, global, local)`; this is what makes `ConstantField` accumulate into shared
  columns while `ElementwiseField` writes to disjoint blocks.

### Field kernels ↔ function

Writing $\mathbf g$ for the global parameter vector and $\boldsymbol\theta_{e,q}$ for the
$C$-vector seen at material point $(e,q)$ (`…/parameterField.h:46-68`):

| Quantity | Formula | C++ method |
|---|---|---|
| parameter value | $\boldsymbol\theta_{e,q}=\operatorname{gather}_e(\mathbf g)$ | `computeValue(e, q, out)` |
| value Jacobian | $\partial\boldsymbol\theta_{e,q}/\partial\mathbf g$ | `computeDerivative(e, q, derivOut)` |
| global index | $\operatorname{globalDof}(e,i)=\begin{cases}e\,C+i & \text{elementwise}\\ i & \text{constant}\end{cases}$ | `dofLayout()->globalDof(e, i)` |
| global vector | $\mathbf g$ | `globalData()` / `setGlobalData(data)` |

This `computeDerivative` Jacobian is the hinge of the **inverse problem**: combined with the element derivatives
$\partial E/\partial a$ (plastic) and $\partial E/\partial b$ (elastic) in
[`energy.md`](energy.md), it produces the gradient of an objective with respect to the design
field. The field kinds enum (`ParameterFieldKind`) also reserves `QUADRATURE_POINT`,
`NODAL_INTERPOLATED`, and `EXTERNAL_PROCEDURAL` for future spatially-richer layouts
(`…/parameterField.h:13-20`).

## Updating values

Fields are mutated in place through the state (see [`state.md`](state.md)):

```python
state.set_elastic_values(new_values)   # validates shape/dtype, writes into C++ storage
state.set_plastic_values(new_values)
```

## Further reading

- [`elastic.md`](elastic.md) — what the elastic channels mean per model.
- [`plastic.md`](plastic.md) — the plastic field as $\mathbf F_p$ parameters.
- [`state.md`](state.md) — how descriptors become live fields bound to a mesh.
- [`energy.md`](energy.md) — the parameter derivatives that consume `computeDerivative`.
