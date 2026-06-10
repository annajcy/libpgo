# MassField + Formulation Dynamics Interfaces + Self-Weight Demo

Date: 2026-06-10
Status: approved design, pending implementation plan

## Background

The dynamics-related operators (`buildMassMatrix`, `buildBodyForce`,
`buildSurfaceEmbeddingMatrix`) exist only on `VolumetricFormulation` and take
the legacy `VolumetricMeshes::VolumetricMesh`, with density implicitly read
from the .veg material. `ShellFormulation` has no mass/body-force path at all,
so shell static-with-gravity and shell dynamics cannot be assembled, and the
shell material optimization demo (`elastic_material_optimization_demo.ipynb`)
hand-rolls a fixed areal load that it calls "gravity": thickness is optimized
as an elastic parameter but the load never changes with thickness, and the
adjoint gradient omits the load-vs-parameter term.

This spec introduces an explicit MassField abstraction, migrates the
formulation dynamics interfaces to `SimulationMesh` + MassField, and upgrades
the demo to a physically correct self-weight mode with the corrected adjoint
gradient.

Out of scope (separate specs): the sim CLI redesign (shell/cubic/tet ×
static/dynamic entry points), shell dynamics end-to-end (DynamicSimulation
wiring), and migration of `buildSurfaceEmbeddingMatrix` (see §3).

## 1. MassField hierarchy (C++)

New top-level directory `src/core/solidDeformationModel/mass/`, sibling of
`material/` and `formulations/`. Dependency direction is one-way:
`mass/` → `material/fields/parameterField.h`; `formulations/` → `mass/`.

```
MassField                       // virtual dtor, compatibleMeshType(), validate(const SimulationMesh&)
├── VolumeMassField             // double volumeDensity(int ele) const = 0   [kg/m^3]
│   ├── ConstantVolumeDensity
│   └── ElementwiseVolumeDensity     // preserves .veg multi-region density
└── ShellMassField              // double arealDensity(int ele) const = 0    [kg/m^2]
    ├── ConstantShellArealDensity
    ├── ShellDensityThickness        // rho × (constant | elementwise) h convenience
    └── ShellDensityElasticThickness // rho × h read live from an elastic OptimizableField channel
```

Design decisions already settled in discussion:

- Mass/load operators do NOT go on the `Formulation` base class. Volume and
  shell expect different MassField types; the base class keeps only the
  deformation core (`createElement`, `createDofLayout`, `buildGlobalRestDofs`,
  `compatibleMeshType`).
- No `elementDensity()` on the `MassField` base: the units differ between
  volume (kg/m^3) and shell (kg/m^2), so the semantically named accessors live
  on the domain subclasses.
- `ShellDensityElasticThickness` holds `const OptimizableField &` plus a
  channel index and evaluates `arealDensity(ele) = rho * computeValue(ele)[ch]`
  live. No duplicated thickness storage, no manual sync after the optimizer
  calls `setGlobalData`. This replaces the earlier ThicknessField
  mini-hierarchy idea.
- Parameter dependence is expressed by a capability interface, not a bool:

```cpp
class ElasticParameterDependentMassField {
public:
  virtual ~ElasticParameterDependentMassField() = default;
  // d(arealDensity(ele)) / d(local elastic dofs); dof layout reuses the
  // referenced OptimizableField's ParameterDofLayout.
  virtual void arealDensityParameterDerivative(int ele, double *out) const = 0;
  virtual const OptimizableField &parameterField() const = 0;
};
```

`ShellDensityElasticThickness` multiply-inherits `ShellMassField` and
`ElasticParameterDependentMassField`. Callers probe with `dynamic_cast`.
Responsibility split: the mass field answers "what is the density and its
parameter sensitivity"; geometric assembly (areas, vertex distribution)
belongs to the formulation.

Rejected alternatives:
- Folding mass into `ParameterField` via a `ParameterDomain::MASS` — loses the
  volume/shell type safety (kg/m^3 vs kg/m^2 indistinguishable at compile
  time) and the rho×h composition is awkward in a flat channel model.
- A standalone `MassAssembler` class instead of formulation methods — assembly
  needs the formulation's shape function/quadrature/DofLayout internals and
  there is no second consumer; extracting it relocates code without decoupling
  anything.

## 2. Formulation interface migration

New signatures, unified on `SimulationMesh`; the old
`VolumetricMeshes::VolumetricMesh`-based `buildMassMatrix`/`buildBodyForce`
overloads are deleted outright (including the
`CubicTricubicHermiteFormulation` overrides of them), and callers are
migrated in the same change.

```cpp
// VolumetricFormulation
EigenSupport::SpMatD buildMassMatrix(const SimulationMesh &, const VolumeMassField &) const;
EigenSupport::VXd    buildBodyForce (const SimulationMesh &, const EigenSupport::V3d &accel,
                                     const VolumeMassField &) const;

// ShellFormulation
EigenSupport::SpMatD buildMassMatrix(const SimulationMesh &, const ShellMassField &) const;
EigenSupport::VXd    buildBodyForce (const SimulationMesh &, const EigenSupport::V3d &accel,
                                     const ShellMassField &) const;
// d f_g / d b; dynamic_casts the mass field for the capability interface,
// throws with a clear message if the field is not parameter-dependent.
EigenSupport::SpMatD buildBodyForceParameterJacobian(
  const SimulationMesh &, const EigenSupport::V3d &accel, const ShellMassField &) const;
```

Implementation:

- **Volume: one generic DofLayout-driven assembly loop** in
  `VolumetricFormulation`, not per-subclass overrides. Uses
  `createDofLayout()` + `shapeFunction()`/`quadrature()` to assemble
  `∫ rho N^T N dV` (mass) and `∫ rho N^T g dV` (body force), scattering via
  `getGlobalDofIndices` (skipping -1 sentinel slots). Tet/cubic linear and
  tricubic Hermite all go through this one loop; Hermite no longer needs
  hand-written overrides.
- **Consistent volume mass via a `massQuadrature()` hook.** (Corrected during
  planning — an earlier revision said "lumped only", which contradicts the
  legacy behavior.) The legacy paths are consistent, not lumped: the vega
  `GenerateMassMatrix::computeMassMatrix(&mesh, M, true)` call computes the
  consistent FEM mass (the `true` flag is `inflate3Dim`, not lumping), and
  the existing Hermite tests (`tests/pypgo/test_hermite_dynamic_helpers.py`)
  assert consistent-mass properties: exact kinetic energy under a constant
  velocity field carried by value DOFs, and nonzero generalized body-force
  entries on derivative DOFs. Row-sum lumping is also ill-defined for the
  Hermite basis (the full 64-function basis is not a partition of unity).
  So the generic loop assembles `∫ rho N^T N dV` consistently using
  `massQuadrature()`, a new virtual on `VolumetricFormulation` defaulting to
  the elastic `quadrature()`. Quadrature-order audit: Hermite uses Gauss 4³
  (exact for tricubic N^TN) and cubic linear uses Gauss 2³ (exact for
  trilinear N^TN on regular hexes), but tet linear's 1-point rule
  under-integrates the quadratic N^TN — `TetLinearFormulation` overrides
  `massQuadrature()` with a standard 4-point degree-2 tet rule, reproducing
  the analytic `rho·V/20·(1+δ_ij)` consistent tet mass.
- **Shell: direct lumped loop** (no shell quadrature infrastructure exists):
  per triangle, `arealDensity(e) * area_e / 3` to each of the three corner
  vertices. The Koiter 6-vertex stencil affects only bending energy;
  displacement DOFs are 3 per vertex and mass/gravity on corner vertices is
  standard.
- `buildBodyForceParameterJacobian`: per element, chain
  `∂f_g/∂(rho_a(e))` (geometry, from the formulation) with
  `arealDensityParameterDerivative` and the OptimizableField's dof layout to
  scatter into a sparse (numDofs × numParameterDofs) matrix.
- **Deliberately deferred: `buildSurfaceEmbeddingMatrix` migration.** It
  depends on `InterpolationCoordinates::BarycentricCoordinates` (point
  location + spatial search bound to `VolumetricMesh`), the demo does not use
  it, and its real consumer is IPC contact — it moves in the CLI spec. It is
  the only legacy signature that survives this change.

Error handling: `validate(mesh)` checks element-count consistency; passing a
mass field of the wrong domain (e.g. `VolumeMassField` to a shell formulation)
is a compile-time type error in C++ and a `TypeError` naming the expected
units in Python.

## 3. Python layer

- New module `pypgo/fem/mass.py`: `VolumeDensity` (constant or elementwise),
  `ShellArealDensity`, `ShellDensityThickness`,
  `ShellDensityElasticThickness(density, parameter_field, channel)`, plus
  convenience `volume_density_from_veg(volume)` that reads per-region .veg
  densities into an elementwise field (keeps the .veg workflow ergonomic
  after the old API is deleted).
- `pypgo/fem/formulations.py`:
  - `VolumetricFormulation.mass_matrix(sim_mesh, mass_field)` and
    `.body_force(sim_mesh, acceleration, mass_field)` replace the
    volume-object-based signatures.
  - `ShellFormulation` gains `mass_matrix`, `body_force`, and
    `body_force_parameter_jacobian` with `ShellMassField` arguments.
- Migrate existing callers: `pypgo/tools/sim/volume_ipc.py`,
  `examples/static_solve_dragon_gravity_demo.ipynb`.

## 4. Demo physical correction (self-weight closed loop)

`ElasticStaticEquilibriumLayer` (pypgo/fem/torch.py) gains an optional
`external_load` provider with protocol:

```python
class ExternalLoadProvider(Protocol):
    def force(self) -> np.ndarray: ...               # f(b) at current parameters
    def parameter_jacobian(self) -> sparse matrix: ...  # d f / d b at current parameters
```

- **forward**: after `_set_parameter_values(b)`, re-evaluate `force()` and
  refresh the gravity `LinearEnergy(-f_g)` before the inner solve.
- **backward**: `_parameter_jacobian(u)` becomes
  `energy.elastic_jacobian(u) - external_load.parameter_jacobian()`,
  supplying the previously missing ∂f_g/∂b term in the adjoint contraction.

A `SelfWeightGravity` helper wraps (shell formulation, sim_mesh,
`ShellDensityElasticThickness`, acceleration) and implements the protocol via
`body_force` / `body_force_parameter_jacobian`.

`elastic_material_optimization_demo.ipynb` keeps two modes:
- `fixed_area_load`: current behavior, renamed to match what it actually does;
- `self_weight`: the new path — when the target material changes thickness,
  the weight changes with it, and the outer gradient includes the load term.

## 5. Testing

- Shell body force sums to total weight `rho·h·A_total·g`; shell mass matrix
  row sums equal lumped vertex masses. Volume: constant-velocity kinetic
  energy equals `½·rho·V·v²` (consistent-mass invariant).
- Volume regression: new generic assembly matches the legacy vega
  `GenerateMassMatrix` consistent output (`VolumeMesh.mass_matrix()`, which
  stays as a mesh-level API) on a tet mesh and a cubic mesh.
- `buildBodyForceParameterJacobian` verified by finite differences (existing
  conventions: enforceSPD=0, magnitude-scaled steps).
- End-to-end demo gradient check: perturb `b`, compare numerical outer-loss
  gradient against the adjoint gradient with the new load term.
- Python type errors: wrong-domain mass field raises `TypeError` mentioning
  expected units.

## Implementation phasing (within one plan)

1. `mass/` hierarchy + fixed-density fields; volume generic assembly +
   migration of old signatures and callers (regression-tested against legacy).
2. Shell `mass_matrix`/`body_force` with fixed mass fields.
3. `ShellDensityElasticThickness` + capability interface +
   `buildBodyForceParameterJacobian` (FD-tested).
4. Torch layer `external_load` + `SelfWeightGravity` + demo upgrade +
   end-to-end gradient check.
