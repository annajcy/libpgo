# `pypgo.fem.state` — Deformation Model State

## Purpose

`DeformationModelState` is the **binding object**: it ties one `SimulationMesh` to a full
material choice — an elastic model with its parameter field, and a plastic model with its
parameter field — so the four modeling decisions of the pipeline become one concrete,
mesh-specific bundle:

$$\text{state} \;=\; \big(\underbrace{\text{mesh}}_{\text{geometry}},\ \underbrace{\Psi,\ \text{elastic field}}_{\text{elastic}},\ \underbrace{\text{p-model},\ \text{plastic field}}_{\text{plastic}}\big).$$

It owns no solver math; it is the input that [`deformation_energy`](energy.md) and
[`plastic_material_energy`](energy.md) consume. In C++ the state is the lifetime owner of the
mesh and the two `OptimizableField`s; managers and assemblers *borrow* them
(`src/core/solidDeformationModel/deformation/deformationModelState.h:46-58`).

## Building a state

```python
from pypgo.fem import (
    deformation_model_state, StVK, VolumetricPlasticity,
    ElementwiseField, ConstantField,
)

state = deformation_model_state(
    sim_mesh,
    elastic=StVK(),                    # Ψ(F)
    elastic_field=ConstantField(),     # one shared coefficient set (defaults from mesh)
    plastic=VolumetricPlasticity(6),   # F_p parametrization
    plastic_field=ElementwiseField(),  # per-element plastic state
)
```

The factory (`pypgo/fem/state.py`) does three things before crossing into C++:

1. **Resolve channel counts** — queries `elastic.num_channels(sim_mesh)` (and the plastic
   model's DOFs) so it knows the field shapes.
2. **Resolve field values** — a descriptor with `values=None` defers to C++ defaults
   (seeded from the mesh material); an explicit array is validated and reshaped to
   `(1, C)` (constant) or `(num_elements, C)` (elementwise).
3. **Create the C++ state** — `DeformationModelState::create(mesh, elastic, ElasticFieldInit,
   plastic, PlasticFieldInit)`, where each `*FieldInit` carries the field *type*
   (`ELEMENTWISE` / `CONSTANT`) and optional initial values
   (`…/deformationModelState.h:34-58`).

## Reading and updating

```python
state.elastic_model     # "stvk"
state.plastic_model     # plastic model id
state.num_elements

state.elastic_field     # ParameterField view (see fields.md)
state.plastic_field

state.set_elastic_values(values)   # in-place update of the elastic field
state.set_plastic_values(values)   # in-place update of the plastic field
```

`set_*` validate shape/dtype and write straight into the C++-owned storage, so an existing
energy built from this state sees the new coefficients on its next evaluation — the mechanism
behind material-parameter sweeps and optimization.

## Snapshots for optimization

For linearizing an objective around the *current* parameters, C++ exposes
`elasticParameterSnapshot()` and `plasticParameterSnapshot()`
(`…/deformationModelState.h:73-74`) — flat copies of the global parameter vectors in the
exact ordering used by the assembler's parameter Jacobians (see [`energy.md`](energy.md)).

## Further reading

- [`fields.md`](fields.md) — `ConstantField` / `ElementwiseField` descriptors and the live
  `ParameterField` view returned by `state.elastic_field` / `state.plastic_field`.
- [`elastic.md`](elastic.md), [`plastic.md`](plastic.md) — the models a state binds.
- [`energy.md`](energy.md) — turning a state into an evaluable energy.
