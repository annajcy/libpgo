# MassField + Formulation Dynamics Interfaces + Self-Weight Demo — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Introduce an explicit MassField hierarchy, migrate `buildMassMatrix`/`buildBodyForce` from legacy `VolumetricMesh` signatures to `SimulationMesh` + MassField (covering shell for the first time), and upgrade the elastic material optimization demo to a physically correct self-weight mode with the corrected adjoint gradient.

**Architecture:** New `src/core/solidDeformationModel/mass/` hierarchy (`MassField` → `VolumeMassField`/`ShellMassField`); one generic DofLayout-driven consistent-mass assembly loop on `VolumetricFormulation` (replacing per-subclass legacy overrides); lumped triangle assembly on `ShellFormulation`; thickness-coupled shell mass reads the elastic `OptimizableField` live and exposes its parameter derivative through a capability interface; the torch equilibrium layer gains an `external_load` provider that closes the ∂f_g/∂b adjoint gap.

**Tech Stack:** C++17 core (`solidDeformationModel`), nanobind bindings (`src/python/pypgo`), Python package (`pypgo/`), pytest (`tests/pypgo`).

**Spec:** `docs/superpowers/specs/2026-06-10-massfield-shell-dynamics-design.md`

**Build & test commands (used throughout):**

```bash
cmake --build --preset base --target pypgo_core      # rebuild C++ core + python extension
python -m pytest tests/pypgo -x -q                   # full python suite
```

If the `base` build preset is not configured on this machine, use the user's local preset (`cmake --build --preset local-base --target pypgo_core` — see `CMakeUserPresets.json`).

**Verified codebase facts this plan relies on:**

- `DofLayout` (`src/core/solidDeformationModel/formulations/dof/dofLayout.h`) provides `numGlobalDofs()`, `numLocalDofs(ele)`, `getGlobalDofIndices(ele, indices)` (−1 for sentinel slots), `gather`. Local DOF ordering is node-major: local dof = `node*3 + coord` (see `vertex3DofLayout.h:13`, and Hermite rest-dof packing in `cubicTricubicHermiteFormulation.cpp` `buildGlobalRestDofs`).
- `VolumetricElementMapping` (`deformation/volume/volumetricElementMapping.h`) has ctor `(const double *restPositions, const ShapeFunction &, const Quadrature &)` and exposes `weightDetJ(q)` = |det Dm| · quadrature weight.
- Legacy mass is **consistent**, not lumped: `GenerateMassMatrix::computeMassMatrix(&mesh, M, true)`'s `true` is `inflate3Dim`. Existing Hermite tests assert consistent-mass invariants.
- Quadratures: tet linear = 1-point (under-integrates N^TN, needs a degree-2 mass rule); cubic linear = Gauss 2³ (exact for trilinear N^TN on regular hexes); Hermite = Gauss 4³ (exact for tricubic N^TN).
- `ElementwiseParameterField : OptimizableField` global dof = `ele*numChannels + channel`; `computeDerivative(ele, q, out)` writes a column-major `numChannels × numLocalDofs` matrix.
- Shell `SimulationMesh` elements have 6 vertex indices (corners = j 0..2, edge-neighbors = j 3..5, −1 sentinels possible on j 3..5); displacement DOFs are 3 per vertex.
- Energy bindings expose the elastic field as `PyParameterField` (`src/python/pypgo/energy/core.h:26`) wrapping `shared_ptr<OptimizableField>`; the Python `ParameterField` wrapper stores it as `self._handle`.
- The demo notebook is **generated** from `examples/scripts/generate_elastic_material_optimization_demo.py`; edit the generator, then regenerate. The dragon notebook (`examples/static_solve_dragon_gravity_demo.ipynb`) is hand-written; edit the .ipynb directly.
- `python -c "import json,..."` style cell inspection or NotebookEdit can be used for .ipynb edits.

---

## Phase 1 — mass/ hierarchy + volume migration

### Task 1: C++ MassField base + volume mass fields

**Files:**
- Create: `src/core/solidDeformationModel/mass/massField.h`
- Create: `src/core/solidDeformationModel/mass/massField.cpp`
- Create: `src/core/solidDeformationModel/mass/volumeMassField.h`
- Create: `src/core/solidDeformationModel/mass/volumeMassField.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1.1: Write `mass/massField.h`**

```cpp
#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

class SimulationMesh;
enum class SimulationMeshType;

// Mass-property field consumed by formulation mass / body-force assembly.
// Separate from the constitutive material (material/) and the optimizable
// parameter fields (material/fields/): it answers "how much mass per
// integration region". Unit-correct accessors live on the domain subclasses:
// VolumeMassField::volumeDensity [kg/m^3], ShellMassField::arealDensity [kg/m^2].
class MassField
{
public:
  virtual ~MassField() = default;

  virtual bool compatibleWith(SimulationMeshType meshType) const = 0;

  // Throws std::invalid_argument on mismatch (mesh type, element counts).
  virtual void validate(const SimulationMesh &mesh) const;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
```

Note: the spec sketched `compatibleMeshType()` returning a single enum; that cannot express "TET or CUBIC" for volume fields, so the implemented interface is the predicate `compatibleWith(SimulationMeshType)`.

- [ ] **Step 1.2: Write `mass/massField.cpp`**

```cpp
#include "massField.h"

#include "simulation/simulationMesh.h"

#include <stdexcept>
#include <string>

namespace pgo
{
namespace SolidDeformationModel
{

void MassField::validate(const SimulationMesh &mesh) const
{
  if (!compatibleWith(mesh.getElementType())) {
    throw std::invalid_argument(
      std::string("mass field is incompatible with mesh type ") +
      meshTypeName(mesh.getElementType()));
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 1.3: Write `mass/volumeMassField.h`**

```cpp
#pragma once

#include "massField.h"

#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Volumetric mass distribution; density in kg/m^3.
class VolumeMassField : public MassField
{
public:
  virtual double volumeDensity(int ele) const = 0;

  bool compatibleWith(SimulationMeshType meshType) const override;
};

class ConstantVolumeDensity : public VolumeMassField
{
public:
  explicit ConstantVolumeDensity(double density);

  double volumeDensity(int /*ele*/) const override { return density_; }

private:
  double density_ = 0.0;
};

class ElementwiseVolumeDensity : public VolumeMassField
{
public:
  explicit ElementwiseVolumeDensity(EigenSupport::VXd densities);

  void validate(const SimulationMesh &mesh) const override;
  double volumeDensity(int ele) const override { return densities_[ele]; }

private:
  EigenSupport::VXd densities_;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 1.4: Write `mass/volumeMassField.cpp`**

```cpp
#include "volumeMassField.h"

#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

bool VolumeMassField::compatibleWith(SimulationMeshType meshType) const
{
  return meshType == SimulationMeshType::TET || meshType == SimulationMeshType::CUBIC;
}

ConstantVolumeDensity::ConstantVolumeDensity(double density)
  : density_(density)
{
  if (!(density > 0.0)) {
    throw std::invalid_argument("ConstantVolumeDensity requires density > 0");
  }
}

ElementwiseVolumeDensity::ElementwiseVolumeDensity(EigenSupport::VXd densities)
  : densities_(std::move(densities))
{
  if (densities_.size() == 0 || (densities_.array() <= 0.0).any()) {
    throw std::invalid_argument("ElementwiseVolumeDensity requires positive per-element densities");
  }
}

void ElementwiseVolumeDensity::validate(const SimulationMesh &mesh) const
{
  VolumeMassField::validate(mesh);
  if (static_cast<int>(densities_.size()) != mesh.getNumElements()) {
    throw std::invalid_argument("ElementwiseVolumeDensity size does not match mesh element count");
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 1.5: Register in `src/core/solidDeformationModel/CMakeLists.txt`**

Add to the `SOLID_DEFORMATION_MODEL_HEADERS` list (keep alphabetic grouping with a new `mass/` block near `material/`):

```cmake
  mass/massField.h
  mass/volumeMassField.h
```

Add to `SOLID_DEFORMATION_MODEL_SOURCES`:

```cmake
  mass/massField.cpp
  mass/volumeMassField.cpp
```

- [ ] **Step 1.6: Build**

Run: `cmake --build --preset base --target pypgo_core`
Expected: compiles cleanly.

- [ ] **Step 1.7: Commit**

```bash
git add src/core/solidDeformationModel/mass src/core/solidDeformationModel/CMakeLists.txt
git commit -m "feat(mass): add MassField base and volume mass fields"
```

### Task 2: Degree-2 tet mass quadrature + `massQuadrature()` hook

**Files:**
- Create: `src/core/solidDeformationModel/formulations/quadrature/tetDegree2Quadrature.h`
- Create: `src/core/solidDeformationModel/formulations/quadrature/tetDegree2Quadrature.cpp`
- Modify: `src/core/solidDeformationModel/formulations/formulation/volumetricFormulation/volumetricFormulation.h`
- Modify: `src/core/solidDeformationModel/formulations/formulation/volumetricFormulation/tetLinearFormulation.h`
- Modify: `src/core/solidDeformationModel/formulations/formulation/volumetricFormulation/tetLinearFormulation.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 2.1: Write `tetDegree2Quadrature.h`**

```cpp
#pragma once

#include "quadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{

// 4-point degree-2 tetrahedron rule (reference volume 1/6, weight 1/24 each).
// Exact for the quadratic N^T N mass integrand of the linear tet; used as
// TetLinearFormulation::massQuadrature() because the elastic 1-point rule
// under-integrates it.
class TetDegree2Quadrature : public Quadrature
{
public:
  static constexpr int kNumPoints = 4;

  int numPoints() const override { return kNumPoints; }
  std::unique_ptr<Quadrature> clone() const override { return std::make_unique<TetDegree2Quadrature>(*this); }
  void point(int i, double xi[3]) const override;
  double weight(int i) const override;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 2.2: Write `tetDegree2Quadrature.cpp`**

```cpp
#include "tetDegree2Quadrature.h"

namespace pgo
{
namespace SolidDeformationModel
{
namespace
{
// Barycentric (a, b, b, b) permutations with a + 3b = 1.
constexpr double kA = 0.5854101966249685;
constexpr double kB = 0.1381966011250105;
}  // namespace

void TetDegree2Quadrature::point(int i, double xi[3]) const
{
  xi[0] = kB;
  xi[1] = kB;
  xi[2] = kB;
  if (i < 3) {
    xi[i] = kA;
  }
  // i == 3: (b, b, b); the implicit 4th barycentric coordinate is a.
}

double TetDegree2Quadrature::weight(int /*i*/) const
{
  return 1.0 / 24.0;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 2.3: Add the hook to `volumetricFormulation.h`**

Inside `class VolumetricFormulation`, after the `quadrature()` accessor:

```cpp
  // Quadrature used for mass / body-force assembly. Defaults to the elastic
  // quadrature; formulations whose elastic rule under-integrates rho*N^T*N
  // (tet linear) override this.
  virtual const Quadrature &massQuadrature() const { return *quadrature_; }
```

- [ ] **Step 2.4: Override in `tetLinearFormulation.h` / `.cpp`**

Header — add inside the class declaration:

```cpp
  const Quadrature &massQuadrature() const override;
```

Source — add (include `"formulations/quadrature/tetDegree2Quadrature.h"`):

```cpp
const Quadrature &TetLinearFormulation::massQuadrature() const
{
  static const TetDegree2Quadrature quad;
  return quad;
}
```

- [ ] **Step 2.5: Register in CMakeLists**

Add `formulations/quadrature/tetDegree2Quadrature.h` to headers and `formulations/quadrature/tetDegree2Quadrature.cpp` to sources next to the existing quadrature entries.

- [ ] **Step 2.6: Build and commit**

Run: `cmake --build --preset base --target pypgo_core` → compiles cleanly.

```bash
git add src/core/solidDeformationModel/formulations src/core/solidDeformationModel/CMakeLists.txt
git commit -m "feat(mass): add degree-2 tet quadrature and massQuadrature hook"
```

### Task 3: Generic volume mass/body-force assembly, delete legacy signatures

**Files:**
- Modify: `src/core/solidDeformationModel/formulations/formulation/volumetricFormulation/volumetricFormulation.h`
- Modify: `src/core/solidDeformationModel/formulations/formulation/volumetricFormulation/volumetricFormulation.cpp`
- Modify: `src/core/solidDeformationModel/formulations/formulation/volumetricFormulation/cubicTricubicHermiteFormulation.h`
- Modify: `src/core/solidDeformationModel/formulations/formulation/volumetricFormulation/cubicTricubicHermiteFormulation.cpp`

- [ ] **Step 3.1: Replace the legacy declarations in `volumetricFormulation.h`**

Delete:

```cpp
  virtual EigenSupport::SpMatD buildMassMatrix(
    const VolumetricMeshes::VolumetricMesh &mesh) const;
  virtual EigenSupport::VXd buildBodyForce(
    const VolumetricMeshes::VolumetricMesh &mesh,
    const EigenSupport::V3d &acceleration) const;
```

Add (keep `buildSurfaceEmbeddingMatrix(const VolumetricMeshes::VolumetricMesh&, ...)` untouched — it is deliberately deferred to the CLI spec):

```cpp
  // Consistent mass matrix / generalized body force, assembled with the
  // formulation's shape function over massQuadrature(), scattered through
  // the formulation's DofLayout. Density comes from the mass field.
  EigenSupport::SpMatD buildMassMatrix(
    const SimulationMesh &mesh, const VolumeMassField &massField) const;
  EigenSupport::VXd buildBodyForce(
    const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
    const VolumeMassField &massField) const;
```

Add a forward declaration `class VolumeMassField;` next to the existing forward declarations in the header.

- [ ] **Step 3.2: Implement the generic loop in `volumetricFormulation.cpp`**

Add includes: `"mass/volumeMassField.h"`, `"formulations/dof/dofLayout.h"`. Replace the two legacy method bodies with:

```cpp
EigenSupport::SpMatD VolumetricFormulation::buildMassMatrix(
  const SimulationMesh &mesh, const VolumeMassField &massField) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }
  massField.validate(mesh);

  const std::unique_ptr<DofLayout> dofLayout = createDofLayout(mesh);
  const ES::VXd restDofs = buildGlobalRestDofs(mesh);
  const ShapeFunction &sf = shapeFunction();
  const Quadrature &quad = massQuadrature();
  const int numNodes = sf.numNodes();

  std::vector<double> N(numNodes);
  std::vector<double> localRest;
  std::vector<int> globalIdx;
  std::vector<ES::TripletD> entries;

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    localRest.resize(dofLayout->numLocalDofs(ele));
    dofLayout->gather(ele, restDofs.data(), localRest.data());
    const VolumetricElementMapping mapping(localRest.data(), sf, quad);
    dofLayout->getGlobalDofIndices(ele, globalIdx);
    const double rho = massField.volumeDensity(ele);

    for (int q = 0; q < quad.numPoints(); q++) {
      double xi[3];
      quad.point(q, xi);
      sf.N(xi[0], xi[1], xi[2], N.data());
      const double w = rho * mapping.weightDetJ(q);

      for (int a = 0; a < numNodes; a++) {
        const double wa = w * N[a];
        if (wa == 0.0)
          continue;
        for (int b = 0; b < numNodes; b++) {
          const double m = wa * N[b];
          if (m == 0.0)
            continue;
          for (int d = 0; d < 3; d++) {
            const int ga = globalIdx[a * 3 + d];
            const int gb = globalIdx[b * 3 + d];
            if (ga < 0 || gb < 0)
              continue;
            entries.emplace_back(ga, gb, m);
          }
        }
      }
    }
  }

  ES::SpMatD M(dofLayout->numGlobalDofs(), dofLayout->numGlobalDofs());
  M.setFromTriplets(entries.begin(), entries.end());
  return M;
}

EigenSupport::VXd VolumetricFormulation::buildBodyForce(
  const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
  const VolumeMassField &massField) const
{
  if (mesh.getElementType() != compatibleMeshType()) {
    throw std::invalid_argument("mesh type is incompatible with this formulation");
  }
  massField.validate(mesh);

  const std::unique_ptr<DofLayout> dofLayout = createDofLayout(mesh);
  const ES::VXd restDofs = buildGlobalRestDofs(mesh);
  const ShapeFunction &sf = shapeFunction();
  const Quadrature &quad = massQuadrature();
  const int numNodes = sf.numNodes();

  std::vector<double> N(numNodes);
  std::vector<double> localRest;
  std::vector<int> globalIdx;
  ES::VXd f = ES::VXd::Zero(dofLayout->numGlobalDofs());

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    localRest.resize(dofLayout->numLocalDofs(ele));
    dofLayout->gather(ele, restDofs.data(), localRest.data());
    const VolumetricElementMapping mapping(localRest.data(), sf, quad);
    dofLayout->getGlobalDofIndices(ele, globalIdx);
    const double rho = massField.volumeDensity(ele);

    for (int q = 0; q < quad.numPoints(); q++) {
      double xi[3];
      quad.point(q, xi);
      sf.N(xi[0], xi[1], xi[2], N.data());
      const double w = rho * mapping.weightDetJ(q);

      for (int a = 0; a < numNodes; a++) {
        const double fa = w * N[a];
        if (fa == 0.0)
          continue;
        for (int d = 0; d < 3; d++) {
          const int ga = globalIdx[a * 3 + d];
          if (ga < 0)
            continue;
          f[ga] += fa * acceleration[d];
        }
      }
    }
  }

  return f;
}
```

Also delete the now-unused `#include "generateMassMatrix.h"` and the `#include "volumetricMesh.h"` only if `buildSurfaceEmbeddingMatrix` no longer needs it (it does need it — keep both includes that the embedding path still uses; delete only `generateMassMatrix.h`).

- [ ] **Step 3.3: Delete the Hermite legacy overrides**

In `cubicTricubicHermiteFormulation.h`, delete the `buildMassMatrix` and `buildBodyForce` override declarations (keep `buildSurfaceEmbeddingMatrix`).

In `cubicTricubicHermiteFormulation.cpp`, delete the two override definitions and the anonymous-namespace helpers `buildHermiteMassMatrix` / `buildHermiteBodyForce` they call (keep `buildHermiteSurfaceEmbeddingMatrix` and anything it uses).

- [ ] **Step 3.4: Build**

Run: `cmake --build --preset base --target pypgo_core`
Expected: **fails** in `src/python/pypgo/fem/formulation/core.cpp` — the bindings still call the deleted signatures. That's the cue for Task 4 (the C++ core library itself must compile; only the binding TU errors).

- [ ] **Step 3.5: Commit** (core-only; bindings land with Task 4)

```bash
git add src/core/solidDeformationModel
git commit -m "feat(mass): generic SimulationMesh+MassField volume mass/body-force assembly, drop legacy VolumetricMesh signatures"
```

### Task 4: Volume Python bindings, `pypgo.fem.mass`, caller migration, tests

**Files:**
- Create: `src/python/pypgo/fem/mass/core.h`
- Create: `src/python/pypgo/fem/mass/core.cpp`
- Create: `src/python/pypgo/fem/mass/bindings.cpp`
- Modify: `src/python/pypgo/fem/formulation/core.h`
- Modify: `src/python/pypgo/fem/formulation/core.cpp`
- Modify: `src/python/pypgo/fem/formulation/bindings.cpp`
- Modify: `src/python/pypgo/module.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Create: `pypgo/fem/mass.py`
- Modify: `pypgo/fem/formulations.py`
- Modify: `pypgo/fem/__init__.py`
- Modify: `pypgo/tools/sim/volume_ipc.py`
- Modify: `tests/pypgo/test_hermite_dynamic_helpers.py`
- Create: `tests/pypgo/test_mass_fields.py`

- [ ] **Step 4.1: Write the failing tests first — `tests/pypgo/test_mass_fields.py`**

```python
import numpy as np
import pytest

import pypgo as pgo
import pypgo.fem as pf


def _unit_tet_volume(*, density=2.0):
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2, 3]], dtype=np.int64)
    mesh = pgo.mesh.TetMeshData(vertices, elements)
    material = pgo.mesh.volume.ENuMaterial(density=density, E=1e6, nu=0.45)
    return pgo.mesh.volume.VolumeMesh.create_from_single_material(mesh, material)


def _single_cube_volume(*, density=2.0):
    vertices = np.array(
        [
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64)
    mesh = pgo.mesh.CubicMeshData(vertices, elements)
    material = pgo.mesh.volume.ENuMaterial(density=density, E=1e6, nu=0.45)
    return pgo.mesh.volume.VolumeMesh.create_from_single_material(mesh, material)


def test_tet_mass_matrix_matches_legacy_vega_consistent_mass():
    volume = _unit_tet_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    M_new = pf.TetLinear().mass_matrix(sim_mesh, pf.VolumeDensity(2.0)).to_dense()
    M_legacy = volume.mass_matrix().to_dense()
    np.testing.assert_allclose(M_new, M_legacy, rtol=1e-12, atol=1e-14)


def test_cubic_mass_matrix_matches_legacy_vega_consistent_mass():
    volume = _single_cube_volume(density=3.0)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    M_new = pf.CubicLinear().mass_matrix(sim_mesh, pf.VolumeDensity(3.0)).to_dense()
    M_legacy = volume.mass_matrix().to_dense()
    np.testing.assert_allclose(M_new, M_legacy, rtol=1e-12, atol=1e-14)


def test_tet_body_force_distributes_total_weight():
    volume = _unit_tet_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    g = np.array([0.0, -9.8, 0.0])
    f = pf.TetLinear().body_force(sim_mesh, g, pf.VolumeDensity(2.0))
    tet_volume = 1.0 / 6.0
    total = f.reshape(-1, 3).sum(axis=0)
    np.testing.assert_allclose(total, 2.0 * tet_volume * g, rtol=1e-12)
    # Linear tet: each vertex carries rho*V/4.
    np.testing.assert_allclose(f.reshape(-1, 3), np.tile(2.0 * tet_volume / 4.0 * g, (4, 1)), rtol=1e-12)


def test_volume_constant_velocity_kinetic_energy_is_exact():
    volume = _single_cube_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    M = pf.CubicLinear().mass_matrix(sim_mesh, pf.VolumeDensity(2.0)).to_dense()
    v = np.array([0.4, -0.2, 0.7])
    qdot = np.tile(v, 8)
    kinetic = 0.5 * qdot @ (M @ qdot)
    assert kinetic == pytest.approx(0.5 * 2.0 * 1.0 * float(v @ v), rel=1e-12)


def test_volume_density_from_veg_reads_region_density():
    volume = _unit_tet_volume(density=7.5)
    field = pf.volume_density_from_veg(volume)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    f = pf.TetLinear().body_force(sim_mesh, [0.0, -1.0, 0.0], field)
    np.testing.assert_allclose(f.reshape(-1, 3).sum(axis=0), [0.0, -7.5 / 6.0, 0.0], rtol=1e-12)


def test_volume_mass_field_type_errors():
    volume = _unit_tet_volume()
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    with pytest.raises(TypeError):
        pf.TetLinear().mass_matrix(sim_mesh, "not a mass field")
    with pytest.raises(TypeError):
        # Old call style: VolumeMesh in place of SimulationMesh.
        pf.TetLinear().mass_matrix(volume, pf.VolumeDensity(1.0))
    with pytest.raises(ValueError):
        pf.VolumeDensity(-1.0)
    with pytest.raises(ValueError):
        # Elementwise size mismatch surfaces as ValueError (C++ invalid_argument).
        pf.TetLinear().mass_matrix(sim_mesh, pf.VolumeDensity(np.array([1.0, 2.0])))
```

Note: if `pgo.mesh.TetMeshData` is named differently, mirror whatever `tests/pypgo/test_mesh_utilities.py` uses for tet meshes.

- [ ] **Step 4.2: Run the new tests to verify they fail**

Run: `python -m pytest tests/pypgo/test_mass_fields.py -q`
Expected: FAIL/ERROR (`AttributeError: ... VolumeDensity` and binding mismatch — extension not rebuilt yet either).

- [ ] **Step 4.3: Write `src/python/pypgo/fem/mass/core.h`**

```cpp
#pragma once

#include "mass/volumeMassField.h"

#include <memory>
#include <vector>

namespace pgo
{

class PyVolumeMassField
{
public:
  explicit PyVolumeMassField(std::shared_ptr<SolidDeformationModel::VolumeMassField> field)
    : field_(std::move(field)) {}

  const SolidDeformationModel::VolumeMassField &get() const { return *field_; }

private:
  std::shared_ptr<SolidDeformationModel::VolumeMassField> field_;
};

std::shared_ptr<PyVolumeMassField> make_constant_volume_density(double density);
std::shared_ptr<PyVolumeMassField> make_elementwise_volume_density(const std::vector<double> &densities);

}  // namespace pgo
```

- [ ] **Step 4.4: Write `src/python/pypgo/fem/mass/core.cpp`**

```cpp
#include "core.h"

namespace pgo
{

std::shared_ptr<PyVolumeMassField> make_constant_volume_density(double density)
{
  return std::make_shared<PyVolumeMassField>(
    std::make_shared<SolidDeformationModel::ConstantVolumeDensity>(density));
}

std::shared_ptr<PyVolumeMassField> make_elementwise_volume_density(const std::vector<double> &densities)
{
  EigenSupport::VXd values(static_cast<Eigen::Index>(densities.size()));
  for (size_t i = 0; i < densities.size(); i++)
    values[static_cast<Eigen::Index>(i)] = densities[i];
  return std::make_shared<PyVolumeMassField>(
    std::make_shared<SolidDeformationModel::ElementwiseVolumeDensity>(std::move(values)));
}

}  // namespace pgo
```

- [ ] **Step 4.5: Write `src/python/pypgo/fem/mass/bindings.cpp`**

```cpp
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "core.h"

namespace nb = nanobind;
using namespace pgo;

void init_mass_bindings(nb::module_ &m)
{
    nb::class_<PyVolumeMassField>(m, "PyVolumeMassField");

    m.def("make_constant_volume_density", &make_constant_volume_density,
        nb::arg("density"));
    m.def("make_elementwise_volume_density", &make_elementwise_volume_density,
        nb::arg("densities"));
}
```

- [ ] **Step 4.6: Migrate `src/python/pypgo/fem/formulation/core.h` / `.cpp` / `bindings.cpp`**

In `core.h`: add includes `"../mass/core.h"` and `"../../simulation/core.h"`; replace the two compute declarations:

```cpp
PySparseMatrix compute_formulation_mass_matrix(
  const PySimulationMesh &simMesh,
  const PyVolumetricFormulation &formulation,
  const PyVolumeMassField &massField);

std::vector<double> compute_formulation_body_force(
  const PySimulationMesh &simMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyVolumeMassField &massField);
```

(`compute_formulation_surface_embedding_matrix` keeps its `PyVolumeMesh` signature.)

In `core.cpp` replace the two definitions:

```cpp
PySparseMatrix compute_formulation_mass_matrix(
  const PySimulationMesh &simMesh,
  const PyVolumetricFormulation &formulation,
  const PyVolumeMassField &massField)
{
  pgo::EigenSupport::SpMatD M;
  {
    nanobind::gil_scoped_release release;
    M = formulation.volumetric().buildMassMatrix(simMesh.mesh(), massField.get());
  }
  return PySparseMatrix(std::move(M));
}

std::vector<double> compute_formulation_body_force(
  const PySimulationMesh &simMesh,
  const PyVolumetricFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyVolumeMassField &massField)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  pgo::EigenSupport::VXd f;
  {
    nanobind::gil_scoped_release release;
    f = formulation.volumetric().buildBodyForce(simMesh.mesh(), a, massField.get());
  }
  return std::vector<double>(f.data(), f.data() + f.size());
}
```

In `bindings.cpp` update the two registrations:

```cpp
    m.def("compute_formulation_mass_matrix", &compute_formulation_mass_matrix,
        nb::arg("sim_mesh"), nb::arg("formulation"), nb::arg("mass_field"));
    m.def("compute_formulation_body_force", &compute_formulation_body_force,
        nb::arg("sim_mesh"), nb::arg("formulation"), nb::arg("acceleration"), nb::arg("mass_field"));
```

- [ ] **Step 4.7: Register the new binding TU**

`src/python/pypgo/module.cpp`: add `void init_mass_bindings(nb::module_ &m);` to the declaration block and `init_mass_bindings(m);` to the call block.

`src/python/pypgo/CMakeLists.txt`: add `fem/mass/core.cpp` and `fem/mass/bindings.cpp` next to the `fem/formulation/*.cpp` entries.

- [ ] **Step 4.8: Write `pypgo/fem/mass.py`**

```python
"""Mass-property fields for formulation mass / body-force assembly.

A mass field answers "how much mass per integration region" — separate from
the constitutive material. Volume fields carry kg/m^3, shell fields kg/m^2.
"""

from __future__ import annotations

import numpy as np

import pypgo._core as _core


class VolumeMassField:
    """Base for volumetric (kg/m^3) mass fields. Holds a C++ handle."""

    def __init__(self, handle) -> None:
        self._handle = handle

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class VolumeDensity(VolumeMassField):
    """Volumetric density: scalar (constant) or 1-D per-element array."""

    def __init__(self, density) -> None:
        arr = np.asarray(density, dtype=np.float64)
        if arr.ndim == 0:
            super().__init__(_core.make_constant_volume_density(float(arr)))
        elif arr.ndim == 1:
            super().__init__(_core.make_elementwise_volume_density(arr.tolist()))
        else:
            raise ValueError(f"density must be a scalar or 1-D array, got shape {arr.shape}")


def volume_density_from_veg(volume) -> VolumeDensity:
    """Per-element densities from a VolumeMesh's .veg material regions."""
    densities = np.zeros(volume.num_elements, dtype=np.float64)
    for _name, material, elements in volume.to_veg_file().to_volume_regions():
        densities[np.asarray(elements, dtype=np.int64)] = float(material.density)
    return VolumeDensity(densities)
```

- [ ] **Step 4.9: Migrate `pypgo/fem/formulations.py`**

Replace `_require_volume_mesh` usage in `mass_matrix` / `body_force` (keep it for `surface_embedding_matrix`) and rewrite the two methods:

```python
def _require_sim_mesh(sim_mesh):
    from pypgo.fem.mesh import SimulationMesh

    if not isinstance(sim_mesh, SimulationMesh):
        raise TypeError(f"sim_mesh must be a SimulationMesh, got {type(sim_mesh).__name__}")


class VolumetricFormulation(Formulation):
    """Volumetric formulation with dynamics operators."""

    def mass_matrix(self, sim_mesh, mass_field):
        """Consistent mass matrix; density from a VolumeDensity (kg/m^3) field."""
        from pypgo.sparse import SparseMatrix
        from pypgo.fem.mass import VolumeMassField

        _require_sim_mesh(sim_mesh)
        if not isinstance(mass_field, VolumeMassField):
            raise TypeError(
                f"volumetric mass_matrix expects a VolumeMassField (kg/m^3), got {type(mass_field).__name__}")
        return SparseMatrix(
            _core.compute_formulation_mass_matrix(sim_mesh._handle, self._handle, mass_field._handle))

    def body_force(self, sim_mesh, acceleration, mass_field) -> np.ndarray:
        """Generalized body force for a constant 3-vector acceleration."""
        from pypgo.fem.mass import VolumeMassField

        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_sim_mesh(sim_mesh)
        if not isinstance(mass_field, VolumeMassField):
            raise TypeError(
                f"volumetric body_force expects a VolumeMassField (kg/m^3), got {type(mass_field).__name__}")
        return np.asarray(
            _core.compute_formulation_body_force(
                sim_mesh._handle, self._handle, accel.tolist(), mass_field._handle),
            dtype=np.float64,
        )
```

(`surface_embedding_matrix` stays exactly as it is.)

- [ ] **Step 4.10: Export from `pypgo/fem/__init__.py`**

Add (matching however `formulations` symbols are re-exported there):

```python
from pypgo.fem.mass import VolumeMassField, VolumeDensity, volume_density_from_veg
```

- [ ] **Step 4.11: Migrate `pypgo/tools/sim/volume_ipc.py`**

Replace lines 101–114 (`# 4.` and `# 7.` blocks):

```python
    # 4. Mass, surface embedding, contact surface
    mass_field = _fem.volume_density_from_veg(volume)
    mass = fm.mass_matrix(sim_mesh, mass_field)
    surface_map = fm.surface_embedding_matrix(volume, surface.vertices)
    contact_surface = _contact.ContactSurface.embedded(surface.vertices, surface_map)
```

```python
    # 7. Body force (gravity)
    external_force = fm.body_force(sim_mesh, np.array(gravity, dtype=np.float64), mass_field)
```

- [ ] **Step 4.12: Migrate `tests/pypgo/test_hermite_dynamic_helpers.py`**

In each test that calls `mass_matrix(volume)` / `body_force(volume, g)`, build the inputs once:

```python
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    mass_field = pf.VolumeDensity(<the density used by that test>)
```

and change calls to `pf.CubicTricubicHermite().mass_matrix(sim_mesh, mass_field)` / `.body_force(sim_mesh, g, mass_field)`. `surface_embedding_matrix(volume, ...)` calls stay unchanged. All existing assertions (consistent-mass kinetic energy, derivative-DOF body-force entries, totals) must still pass — they encode the consistent-mass contract the generic loop must reproduce.

- [ ] **Step 4.13: Build and run the tests**

```bash
cmake --build --preset base --target pypgo_core
python -m pytest tests/pypgo/test_mass_fields.py tests/pypgo/test_hermite_dynamic_helpers.py -q
```

Expected: PASS. If the tet/cubic parity tests disagree with vega beyond tolerance, debug the quadrature/detJ path before proceeding — do not loosen tolerances.

- [ ] **Step 4.14: Run the full suite and commit**

```bash
python -m pytest tests/pypgo -x -q
git add src/python/pypgo pypgo/fem pypgo/tools/sim/volume_ipc.py tests/pypgo/test_mass_fields.py tests/pypgo/test_hermite_dynamic_helpers.py
git commit -m "feat(mass): SimulationMesh+MassField python API, migrate volume callers and tests"
```

### Task 5: Migrate the dragon gravity notebook

**Files:**
- Modify: `examples/static_solve_dragon_gravity_demo.ipynb` (hand-written notebook, NOT generated)

- [ ] **Step 5.1: Locate the body-force cell**

Run: `grep -n "body_force" examples/static_solve_dragon_gravity_demo.ipynb`
The notebook already creates a `SimulationMesh` for its deformation energy (search `create_volumetric` for the variable name, typically `sim_mesh`).

- [ ] **Step 5.2: Update the gravity cell with NotebookEdit**

Replace the `formulation.body_force(volume, gravity_accel)` call with:

```python
mass_field = pf.volume_density_from_veg(volume)
gravity_force = formulation.body_force(sim_mesh, gravity_accel, mass_field)
```

(adapt `pf`/variable names to the notebook's own imports; the physics is unchanged — the .veg density now flows in explicitly through the mass field).

- [ ] **Step 5.3: Verify and commit**

The user already has uncommitted edits staged for this notebook — commit only the cells this task changed alongside them after a quick `git diff --staged` review; if the staged content looks unrelated or surprising, stop and ask the user instead of committing over it.

```bash
python -m pytest tests/pypgo/test_notebook.py -q
git add examples/static_solve_dragon_gravity_demo.ipynb
git commit -m "refactor: dragon gravity demo uses SimulationMesh + volume_density_from_veg"
```

## Phase 2 — shell mass / body force (fixed fields)

### Task 6: C++ shell mass fields + ShellFormulation assembly

**Files:**
- Create: `src/core/solidDeformationModel/mass/shellMassField.h`
- Create: `src/core/solidDeformationModel/mass/shellMassField.cpp`
- Modify: `src/core/solidDeformationModel/formulations/formulation/shellFormulation/shellFormulation.h`
- Modify: `src/core/solidDeformationModel/formulations/formulation/shellFormulation/shellFormulation.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 6.1: Write `mass/shellMassField.h`**

```cpp
#pragma once

#include "massField.h"

#include "EigenSupport.h"

namespace pgo
{
namespace SolidDeformationModel
{

// Shell mass distribution; areal density rho*h in kg/m^2.
class ShellMassField : public MassField
{
public:
  virtual double arealDensity(int ele) const = 0;

  bool compatibleWith(SimulationMeshType meshType) const override;
};

class ConstantShellArealDensity : public ShellMassField
{
public:
  explicit ConstantShellArealDensity(double arealDensity);

  double arealDensity(int /*ele*/) const override { return arealDensity_; }

private:
  double arealDensity_ = 0.0;
};

// rho * h(e) with a fixed thickness (constant or per element).
class ShellDensityThickness : public ShellMassField
{
public:
  ShellDensityThickness(double density, double thickness);
  ShellDensityThickness(double density, EigenSupport::VXd thickness);

  void validate(const SimulationMesh &mesh) const override;
  double arealDensity(int ele) const override;

private:
  double density_ = 0.0;
  double constantThickness_ = 0.0;
  EigenSupport::VXd elementThickness_;  // empty when constant
};

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 6.2: Write `mass/shellMassField.cpp`**

```cpp
#include "shellMassField.h"

#include "simulation/simulationMesh.h"

#include <stdexcept>

namespace pgo
{
namespace SolidDeformationModel
{

bool ShellMassField::compatibleWith(SimulationMeshType meshType) const
{
  return meshType == SimulationMeshType::SHELL;
}

ConstantShellArealDensity::ConstantShellArealDensity(double arealDensity)
  : arealDensity_(arealDensity)
{
  if (!(arealDensity > 0.0)) {
    throw std::invalid_argument("ConstantShellArealDensity requires arealDensity > 0");
  }
}

ShellDensityThickness::ShellDensityThickness(double density, double thickness)
  : density_(density), constantThickness_(thickness)
{
  if (!(density > 0.0) || !(thickness > 0.0)) {
    throw std::invalid_argument("ShellDensityThickness requires positive density and thickness");
  }
}

ShellDensityThickness::ShellDensityThickness(double density, EigenSupport::VXd thickness)
  : density_(density), elementThickness_(std::move(thickness))
{
  if (!(density > 0.0) || elementThickness_.size() == 0 ||
      (elementThickness_.array() <= 0.0).any()) {
    throw std::invalid_argument("ShellDensityThickness requires positive density and thicknesses");
  }
}

void ShellDensityThickness::validate(const SimulationMesh &mesh) const
{
  ShellMassField::validate(mesh);
  if (elementThickness_.size() != 0 &&
      static_cast<int>(elementThickness_.size()) != mesh.getNumElements()) {
    throw std::invalid_argument("ShellDensityThickness thickness size does not match mesh element count");
  }
}

double ShellDensityThickness::arealDensity(int ele) const
{
  const double h = elementThickness_.size() != 0 ? elementThickness_[ele] : constantThickness_;
  return density_ * h;
}

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 6.3: Declare the shell assembly in `shellFormulation.h`**

Add forward declarations and methods inside `class ShellFormulation`:

```cpp
class ShellMassField;
// ... inside the class:
  // Lumped shell mass / body force: per triangle, arealDensity(e)*area_e/3 to
  // each of the three corner vertices. The Koiter 6-vertex stencil only
  // affects bending energy; displacement DOFs are 3 per vertex.
  EigenSupport::SpMatD buildMassMatrix(
    const SimulationMesh &mesh, const ShellMassField &massField) const;
  EigenSupport::VXd buildBodyForce(
    const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
    const ShellMassField &massField) const;
```

(also `#include "EigenDef.h"` if the header does not already see the Eigen typedefs through `formulation.h` — it does, `formulation.h` includes `EigenDef.h`.)

- [ ] **Step 6.4: Implement in `shellFormulation.cpp`**

Add includes `"mass/shellMassField.h"` and the anonymous-namespace helper + methods:

```cpp
namespace
{
namespace ES = EigenSupport;

double triangleRestArea(const SimulationMesh &mesh, int ele)
{
  double p[3][3];
  for (int j = 0; j < 3; j++)
    mesh.getVertex(ele, j, p[j]);
  const ES::V3d a(p[0]), b(p[1]), c(p[2]);
  return 0.5 * ((b - a).cross(c - a)).norm();
}
}  // namespace

EigenSupport::SpMatD ShellFormulation::buildMassMatrix(
  const SimulationMesh &mesh, const ShellMassField &massField) const
{
  massField.validate(mesh);

  std::vector<ES::TripletD> entries;
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double m = massField.arealDensity(ele) * triangleRestArea(mesh, ele) / 3.0;
    for (int j = 0; j < 3; j++) {
      const int v = mesh.getVertexIndex(ele, j);
      for (int d = 0; d < 3; d++)
        entries.emplace_back(v * 3 + d, v * 3 + d, m);
    }
  }

  const int numDofs = mesh.getNumVertices() * 3;
  ES::SpMatD M(numDofs, numDofs);
  M.setFromTriplets(entries.begin(), entries.end());
  return M;
}

EigenSupport::VXd ShellFormulation::buildBodyForce(
  const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
  const ShellMassField &massField) const
{
  massField.validate(mesh);

  ES::VXd f = ES::VXd::Zero(mesh.getNumVertices() * 3);
  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    const double m = massField.arealDensity(ele) * triangleRestArea(mesh, ele) / 3.0;
    for (int j = 0; j < 3; j++) {
      const int v = mesh.getVertexIndex(ele, j);
      f.segment<3>(v * 3) += m * acceleration;
    }
  }
  return f;
}
```

(Note the existing anonymous namespace in this file already exists — merge the helper into it.)

- [ ] **Step 6.5: Register, build, commit**

CMakeLists: add `mass/shellMassField.h` / `mass/shellMassField.cpp`.

Run: `cmake --build --preset base --target pypgo_core` → compiles cleanly.

```bash
git add src/core/solidDeformationModel
git commit -m "feat(mass): shell mass fields and lumped ShellFormulation mass/body-force assembly"
```

### Task 7: Shell Python bindings + wrappers + tests

**Files:**
- Modify: `src/python/pypgo/fem/mass/core.h`, `core.cpp`, `bindings.cpp`
- Modify: `src/python/pypgo/fem/formulation/core.h`, `core.cpp`, `bindings.cpp`
- Modify: `pypgo/fem/mass.py`
- Modify: `pypgo/fem/formulations.py`
- Modify: `pypgo/fem/__init__.py`
- Modify: `tests/pypgo/test_mass_fields.py`

- [ ] **Step 7.1: Write the failing tests — append to `tests/pypgo/test_mass_fields.py`**

```python
def _shell_grid(nx=2, ny=2):
    def vid(i, j):
        return i * (ny + 1) + j

    vertices = np.array(
        [[i / nx, j / ny, 0.0] for i in range(nx + 1) for j in range(ny + 1)],
        dtype=np.float64,
    )
    triangles = []
    for i in range(nx):
        for j in range(ny):
            triangles.append([vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)])
            triangles.append([vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)])
    triangles = np.asarray(triangles, dtype=np.int64)
    surface = pgo.mesh.TriMeshData(vertices, triangles)
    material = pf.KoiterStVKShellMaterial(thickness=1e-3, E_membrane=2e4, nu_membrane=0.35)
    return surface, vertices, triangles, pf.SimulationMesh.create_shell(surface, material)


def test_shell_body_force_matches_manual_lumped_formula():
    _surface, vertices, triangles, sim = _shell_grid()
    g = np.array([0.0, 0.0, -9.81])
    rho_h = 1.0
    f = pf.KoiterShell().body_force(sim, g, pf.ShellArealDensity(rho_h))

    vertex_area = np.zeros(vertices.shape[0])
    for tri in triangles:
        a, b, c = vertices[tri]
        vertex_area[tri] += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 3.0
    manual = (rho_h * vertex_area[:, None] * g).ravel()
    np.testing.assert_allclose(f, manual, rtol=1e-12, atol=1e-15)


def test_shell_body_force_total_weight():
    _surface, _vertices, _triangles, sim = _shell_grid()
    g = np.array([0.0, 0.0, -9.81])
    f = pf.KoiterShell().body_force(sim, g, pf.ShellDensityThickness(density=1000.0, thickness=1e-3))
    # Unit square shell: total area 1, rho*h = 1.
    np.testing.assert_allclose(f.reshape(-1, 3).sum(axis=0), 1.0 * g, rtol=1e-12)


def test_shell_mass_matrix_row_sums_are_lumped_vertex_masses():
    _surface, vertices, triangles, sim = _shell_grid()
    M = pf.KoiterShell().mass_matrix(sim, pf.ShellArealDensity(2.0)).to_dense()
    vertex_area = np.zeros(vertices.shape[0])
    for tri in triangles:
        a, b, c = vertices[tri]
        vertex_area[tri] += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 3.0
    np.testing.assert_allclose(np.diag(M).reshape(-1, 3), 2.0 * vertex_area[:, None] * np.ones(3), rtol=1e-12)
    np.testing.assert_allclose(M, np.diag(np.diag(M)), atol=1e-15)


def test_shell_volume_mass_field_cross_domain_type_errors():
    _surface, _vertices, _triangles, sim = _shell_grid()
    with pytest.raises(TypeError):
        pf.KoiterShell().mass_matrix(sim, pf.VolumeDensity(1000.0))
    volume = _unit_tet_volume()
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    with pytest.raises(TypeError):
        pf.TetLinear().mass_matrix(sim_mesh, pf.ShellArealDensity(1.0))
```

Run: `python -m pytest tests/pypgo/test_mass_fields.py -q` → new tests FAIL (`AttributeError: ShellArealDensity`).

- [ ] **Step 7.2: Extend `src/python/pypgo/fem/mass/core.h` / `.cpp` / `bindings.cpp`**

core.h — add:

```cpp
#include "mass/shellMassField.h"

class PyShellMassField
{
public:
  explicit PyShellMassField(std::shared_ptr<SolidDeformationModel::ShellMassField> field)
    : field_(std::move(field)) {}

  const SolidDeformationModel::ShellMassField &get() const { return *field_; }

protected:
  std::shared_ptr<SolidDeformationModel::ShellMassField> field_;
};

std::shared_ptr<PyShellMassField> make_constant_shell_areal_density(double arealDensity);
std::shared_ptr<PyShellMassField> make_shell_density_thickness_constant(double density, double thickness);
std::shared_ptr<PyShellMassField> make_shell_density_thickness_elementwise(
  double density, const std::vector<double> &thickness);
```

core.cpp — add:

```cpp
std::shared_ptr<PyShellMassField> make_constant_shell_areal_density(double arealDensity)
{
  return std::make_shared<PyShellMassField>(
    std::make_shared<SolidDeformationModel::ConstantShellArealDensity>(arealDensity));
}

std::shared_ptr<PyShellMassField> make_shell_density_thickness_constant(double density, double thickness)
{
  return std::make_shared<PyShellMassField>(
    std::make_shared<SolidDeformationModel::ShellDensityThickness>(density, thickness));
}

std::shared_ptr<PyShellMassField> make_shell_density_thickness_elementwise(
  double density, const std::vector<double> &thickness)
{
  EigenSupport::VXd values(static_cast<Eigen::Index>(thickness.size()));
  for (size_t i = 0; i < thickness.size(); i++)
    values[static_cast<Eigen::Index>(i)] = thickness[i];
  return std::make_shared<PyShellMassField>(
    std::make_shared<SolidDeformationModel::ShellDensityThickness>(density, std::move(values)));
}
```

bindings.cpp — add:

```cpp
    nb::class_<PyShellMassField>(m, "PyShellMassField");
    m.def("make_constant_shell_areal_density", &make_constant_shell_areal_density,
        nb::arg("areal_density"));
    m.def("make_shell_density_thickness_constant", &make_shell_density_thickness_constant,
        nb::arg("density"), nb::arg("thickness"));
    m.def("make_shell_density_thickness_elementwise", &make_shell_density_thickness_elementwise,
        nb::arg("density"), nb::arg("thickness"));
```

- [ ] **Step 7.3: Shell compute functions in `fem/formulation/core.h` / `.cpp` / `bindings.cpp`**

core.h — give `PyShellFormulation` a typed accessor and declare:

```cpp
  const SolidDeformationModel::ShellFormulation &shell() const
  {
    return static_cast<const SolidDeformationModel::ShellFormulation &>(get());
  }
```

```cpp
PySparseMatrix compute_shell_formulation_mass_matrix(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const PyShellMassField &massField);

std::vector<double> compute_shell_formulation_body_force(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyShellMassField &massField);
```

core.cpp — implement both:

```cpp
PySparseMatrix compute_shell_formulation_mass_matrix(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const PyShellMassField &massField)
{
  pgo::EigenSupport::SpMatD M;
  {
    nanobind::gil_scoped_release release;
    M = formulation.shell().buildMassMatrix(simMesh.mesh(), massField.get());
  }
  return PySparseMatrix(std::move(M));
}

std::vector<double> compute_shell_formulation_body_force(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyShellMassField &massField)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  pgo::EigenSupport::VXd f;
  {
    nanobind::gil_scoped_release release;
    f = formulation.shell().buildBodyForce(simMesh.mesh(), a, massField.get());
  }
  return std::vector<double>(f.data(), f.data() + f.size());
}
```

(`fem/formulation/core.cpp` must include `"formulations/formulation/shellFormulation/shellFormulation.h"` if `formulations.h` does not already pull it in.)

bindings.cpp — register both with `nb::arg("sim_mesh"), nb::arg("formulation"), [nb::arg("acceleration"),] nb::arg("mass_field")`.

- [ ] **Step 7.4: Python wrappers — append to `pypgo/fem/mass.py`**

```python
class ShellMassField:
    """Base for shell (kg/m^2) mass fields. Holds a C++ handle."""

    def __init__(self, handle) -> None:
        self._handle = handle

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class ShellArealDensity(ShellMassField):
    """Constant areal density rho*h in kg/m^2."""

    def __init__(self, areal_density: float) -> None:
        super().__init__(_core.make_constant_shell_areal_density(float(areal_density)))


class ShellDensityThickness(ShellMassField):
    """rho * h with fixed thickness (scalar or per-element array)."""

    def __init__(self, *, density: float, thickness) -> None:
        arr = np.asarray(thickness, dtype=np.float64)
        if arr.ndim == 0:
            super().__init__(_core.make_shell_density_thickness_constant(float(density), float(arr)))
        elif arr.ndim == 1:
            super().__init__(_core.make_shell_density_thickness_elementwise(float(density), arr.tolist()))
        else:
            raise ValueError(f"thickness must be a scalar or 1-D array, got shape {arr.shape}")
```

- [ ] **Step 7.5: `ShellFormulation` methods in `pypgo/fem/formulations.py`**

```python
class ShellFormulation(Formulation):
    """Shell formulation with lumped mass / body-force operators."""

    def _require_shell_mass_field(self, mass_field):
        from pypgo.fem.mass import ShellMassField

        if not isinstance(mass_field, ShellMassField):
            raise TypeError(
                f"shell formulation expects a ShellMassField (kg/m^2), got {type(mass_field).__name__}")

    def mass_matrix(self, sim_mesh, mass_field):
        """Lumped shell mass matrix."""
        from pypgo.sparse import SparseMatrix

        _require_sim_mesh(sim_mesh)
        self._require_shell_mass_field(mass_field)
        return SparseMatrix(
            _core.compute_shell_formulation_mass_matrix(sim_mesh._handle, self._handle, mass_field._handle))

    def body_force(self, sim_mesh, acceleration, mass_field) -> np.ndarray:
        """Lumped shell body force for a constant 3-vector acceleration."""
        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_sim_mesh(sim_mesh)
        self._require_shell_mass_field(mass_field)
        return np.asarray(
            _core.compute_shell_formulation_body_force(
                sim_mesh._handle, self._handle, accel.tolist(), mass_field._handle),
            dtype=np.float64,
        )
```

- [ ] **Step 7.6: Export, build, test, commit**

`pypgo/fem/__init__.py`: extend the mass import with `ShellMassField, ShellArealDensity, ShellDensityThickness`.

```bash
cmake --build --preset base --target pypgo_core
python -m pytest tests/pypgo/test_mass_fields.py -q          # PASS
python -m pytest tests/pypgo -x -q                            # PASS
git add src/python/pypgo pypgo/fem tests/pypgo/test_mass_fields.py
git commit -m "feat(mass): shell mass/body-force python API"
```

## Phase 3 — elastic-parameter-coupled shell mass

### Task 8: C++ `ShellDensityElasticThickness` + body-force parameter Jacobian

**Files:**
- Create: `src/core/solidDeformationModel/mass/elasticParameterDependentMassField.h`
- Create: `src/core/solidDeformationModel/mass/shellDensityElasticThickness.h`
- Create: `src/core/solidDeformationModel/mass/shellDensityElasticThickness.cpp`
- Modify: `src/core/solidDeformationModel/formulations/formulation/shellFormulation/shellFormulation.h`
- Modify: `src/core/solidDeformationModel/formulations/formulation/shellFormulation/shellFormulation.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 8.1: Write `mass/elasticParameterDependentMassField.h`**

```cpp
#pragma once

namespace pgo
{
namespace SolidDeformationModel
{

class OptimizableField;

// Capability interface for mass fields whose density depends on optimizable
// elastic parameters. Implementations multiply-inherit their domain MassField
// and this interface; callers probe with dynamic_cast.
class ElasticParameterDependentMassField
{
public:
  virtual ~ElasticParameterDependentMassField() = default;

  virtual const OptimizableField &parameterField() const = 0;

  // d(arealDensity(ele)) / d(local parameter dofs); out has length
  // parameterField().dofLayout()->numLocalDofs().
  virtual void arealDensityParameterDerivative(int ele, double *out) const = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 8.2: Write `mass/shellDensityElasticThickness.h`**

```cpp
#pragma once

#include "shellMassField.h"
#include "elasticParameterDependentMassField.h"

#include <memory>

namespace pgo
{
namespace SolidDeformationModel
{

class OptimizableField;

// rho * h with h read live from a channel of an elastic OptimizableField
// (e.g. the Koiter elastic field's thickness channel 4). No duplicated
// thickness storage: after the optimizer calls setGlobalData on the field,
// this mass field sees the new values.
class ShellDensityElasticThickness :
  public ShellMassField,
  public ElasticParameterDependentMassField
{
public:
  ShellDensityElasticThickness(double density,
    std::shared_ptr<const OptimizableField> field, int thicknessChannel);

  void validate(const SimulationMesh &mesh) const override;
  double arealDensity(int ele) const override;

  const OptimizableField &parameterField() const override { return *field_; }
  void arealDensityParameterDerivative(int ele, double *out) const override;

private:
  double density_ = 0.0;
  std::shared_ptr<const OptimizableField> field_;
  int channel_ = 0;
};

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 8.3: Write `mass/shellDensityElasticThickness.cpp`**

```cpp
#include "shellDensityElasticThickness.h"

#include "material/fields/parameterField.h"
#include "simulation/simulationMesh.h"

#include <stdexcept>
#include <vector>

namespace pgo
{
namespace SolidDeformationModel
{

ShellDensityElasticThickness::ShellDensityElasticThickness(double density,
  std::shared_ptr<const OptimizableField> field, int thicknessChannel)
  : density_(density), field_(std::move(field)), channel_(thicknessChannel)
{
  if (!(density > 0.0)) {
    throw std::invalid_argument("ShellDensityElasticThickness requires density > 0");
  }
  if (!field_) {
    throw std::invalid_argument("ShellDensityElasticThickness requires a parameter field");
  }
  if (channel_ < 0 || channel_ >= field_->numChannels()) {
    throw std::invalid_argument("ShellDensityElasticThickness thickness channel out of range");
  }
}

void ShellDensityElasticThickness::validate(const SimulationMesh &mesh) const
{
  ShellMassField::validate(mesh);
  if (!field_->dofLayout()->matchesParameterShape(field_->numChannels(), mesh.getNumElements())) {
    throw std::invalid_argument(
      "ShellDensityElasticThickness parameter field shape does not match mesh element count");
  }
}

double ShellDensityElasticThickness::arealDensity(int ele) const
{
  std::vector<double> value(field_->numChannels());
  field_->computeValue(ele, 0, value.data());
  return density_ * value[channel_];
}

void ShellDensityElasticThickness::arealDensityParameterDerivative(int ele, double *out) const
{
  const int numChannels = field_->numChannels();
  const int numLocal = field_->dofLayout()->numLocalDofs();
  std::vector<double> deriv(static_cast<size_t>(numChannels) * numLocal);
  field_->computeDerivative(ele, 0, deriv.data());
  // deriv is column-major numChannels x numLocal; take the thickness row.
  for (int k = 0; k < numLocal; k++)
    out[k] = density_ * deriv[static_cast<size_t>(k) * numChannels + channel_];
}

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 8.4: `buildBodyForceParameterJacobian` on `ShellFormulation`**

Header — add:

```cpp
  // d f_g / d b for a parameter-dependent shell mass field; throws
  // std::invalid_argument if the field does not implement
  // ElasticParameterDependentMassField. Shape: (numVertices*3) x numParameterDofs.
  EigenSupport::SpMatD buildBodyForceParameterJacobian(
    const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
    const ShellMassField &massField) const;
```

Source — add includes `"mass/elasticParameterDependentMassField.h"`, `"material/fields/parameterField.h"` and:

```cpp
EigenSupport::SpMatD ShellFormulation::buildBodyForceParameterJacobian(
  const SimulationMesh &mesh, const EigenSupport::V3d &acceleration,
  const ShellMassField &massField) const
{
  massField.validate(mesh);
  const auto *dependent = dynamic_cast<const ElasticParameterDependentMassField *>(&massField);
  if (dependent == nullptr) {
    throw std::invalid_argument(
      "buildBodyForceParameterJacobian requires a mass field that depends on elastic parameters");
  }

  const OptimizableField &field = dependent->parameterField();
  const auto *layout = field.dofLayout();
  const int numLocal = layout->numLocalDofs();
  std::vector<double> dRho(numLocal);
  std::vector<ES::TripletD> entries;

  for (int ele = 0; ele < mesh.getNumElements(); ele++) {
    dependent->arealDensityParameterDerivative(ele, dRho.data());
    const double areaThird = triangleRestArea(mesh, ele) / 3.0;
    for (int k = 0; k < numLocal; k++) {
      if (dRho[k] == 0.0)
        continue;
      const int col = layout->globalDof(ele, k);
      const double s = dRho[k] * areaThird;
      for (int j = 0; j < 3; j++) {
        const int v = mesh.getVertexIndex(ele, j);
        for (int d = 0; d < 3; d++)
          entries.emplace_back(v * 3 + d, col, s * acceleration[d]);
      }
    }
  }

  ES::SpMatD J(mesh.getNumVertices() * 3, layout->numGlobalDofs());
  J.setFromTriplets(entries.begin(), entries.end());
  return J;
}
```

- [ ] **Step 8.5: Register, build, commit**

CMakeLists: add the two new headers and `mass/shellDensityElasticThickness.cpp`.

Run: `cmake --build --preset base --target pypgo_core` → compiles cleanly.

```bash
git add src/core/solidDeformationModel
git commit -m "feat(mass): elastic-thickness-coupled shell mass field and body-force parameter jacobian"
```

### Task 9: Bindings + Python + FD test for the parameter Jacobian

**Files:**
- Modify: `src/python/pypgo/fem/mass/core.h`, `core.cpp`, `bindings.cpp`
- Modify: `src/python/pypgo/fem/formulation/core.h`, `core.cpp`, `bindings.cpp`
- Modify: `pypgo/fem/mass.py`
- Modify: `pypgo/fem/formulations.py`
- Modify: `pypgo/fem/__init__.py`
- Modify: `tests/pypgo/test_mass_fields.py`

- [ ] **Step 9.1: Write the failing test — append to `tests/pypgo/test_mass_fields.py`**

```python
def _shell_energy(sim, triangles):
    base_row = np.array([2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3], dtype=np.float64)
    elastic = np.tile(base_row, (triangles.shape[0], 1))
    plastic = np.ones((triangles.shape[0], 1), dtype=np.float64)
    return pf.deformation_energy(
        sim,
        elastic=pf.KoiterStVK(),
        elastic_field=pf.ElementwiseField(values=elastic),
        plastic=pf.ShellPlasticity(dofs=1),
        plastic_field=pf.ElementwiseField(values=plastic),
        formulation=pf.KoiterShell(),
        options=pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
    )


def test_shell_elastic_thickness_mass_field_reads_live_values():
    _surface, _vertices, triangles, sim = _shell_grid()
    energy = _shell_energy(sim, triangles)
    field = pf.ShellDensityElasticThickness(density=1000.0, parameter_field=energy.elastic_field, channel=4)
    g = np.array([0.0, 0.0, -9.81])
    ks = pf.KoiterShell()

    f0 = ks.body_force(sim, g, field)
    values = energy.elastic_field.values.copy()
    values[:, 4] *= 2.0  # double the thickness
    energy.set_elastic_values(values)
    f1 = ks.body_force(sim, g, field)
    np.testing.assert_allclose(f1, 2.0 * f0, rtol=1e-12)


def test_shell_body_force_parameter_jacobian_matches_differences():
    _surface, _vertices, triangles, sim = _shell_grid()
    energy = _shell_energy(sim, triangles)
    field = pf.ShellDensityElasticThickness(density=1000.0, parameter_field=energy.elastic_field, channel=4)
    g = np.array([0.0, 0.0, -9.81])
    ks = pf.KoiterShell()

    b0 = energy.elastic_field.values.copy()
    f0 = ks.body_force(sim, g, field)
    J = ks.body_force_parameter_jacobian(sim, g, field).to_dense()
    assert J.shape == (sim.num_vertices * 3, b0.size)

    rng = np.random.default_rng(0)
    db = np.zeros_like(b0)
    db[:, 4] = rng.uniform(-0.5, 0.5, size=b0.shape[0]) * b0[:, 4]
    energy.set_elastic_values(b0 + db)
    f1 = ks.body_force(sim, g, field)
    # f_g is linear in h, so the Jacobian is exact even for finite steps.
    np.testing.assert_allclose(f1 - f0, J @ db.ravel(), rtol=1e-10, atol=1e-14)
    energy.set_elastic_values(b0)


def test_body_force_parameter_jacobian_rejects_fixed_mass_fields():
    _surface, _vertices, _triangles, sim = _shell_grid()
    with pytest.raises(ValueError):
        pf.KoiterShell().body_force_parameter_jacobian(
            sim, [0.0, 0.0, -9.81], pf.ShellArealDensity(1.0))
```

Run: `python -m pytest tests/pypgo/test_mass_fields.py -q` → new tests FAIL.

- [ ] **Step 9.2: Bindings**

`fem/mass/core.h`: include `"mass/shellDensityElasticThickness.h"` and `"../../energy/core.h"`; declare:

```cpp
std::shared_ptr<PyShellMassField> make_shell_density_elastic_thickness(
  double density, const PyParameterField &field, int thicknessChannel);
```

`fem/mass/core.cpp`:

```cpp
std::shared_ptr<PyShellMassField> make_shell_density_elastic_thickness(
  double density, const PyParameterField &field, int thicknessChannel)
{
  return std::make_shared<PyShellMassField>(
    std::make_shared<SolidDeformationModel::ShellDensityElasticThickness>(
      density, field.field(), thicknessChannel));
}
```

`fem/mass/bindings.cpp`: register with `nb::arg("density"), nb::arg("parameter_field"), nb::arg("channel")`.

`fem/formulation/core.h` — declare:

```cpp
PySparseMatrix compute_shell_formulation_body_force_parameter_jacobian(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyShellMassField &massField);
```

`fem/formulation/core.cpp` — implement:

```cpp
PySparseMatrix compute_shell_formulation_body_force_parameter_jacobian(
  const PySimulationMesh &simMesh,
  const PyShellFormulation &formulation,
  const std::vector<double> &acceleration,
  const PyShellMassField &massField)
{
  if (acceleration.size() != 3) {
    throw std::invalid_argument("acceleration must contain exactly 3 values");
  }

  pgo::EigenSupport::V3d a(acceleration[0], acceleration[1], acceleration[2]);
  pgo::EigenSupport::SpMatD J;
  {
    nanobind::gil_scoped_release release;
    J = formulation.shell().buildBodyForceParameterJacobian(simMesh.mesh(), a, massField.get());
  }
  return PySparseMatrix(std::move(J));
}
```

`fem/formulation/bindings.cpp` — register:

```cpp
    m.def("compute_shell_formulation_body_force_parameter_jacobian",
        &compute_shell_formulation_body_force_parameter_jacobian,
        nb::arg("sim_mesh"), nb::arg("formulation"), nb::arg("acceleration"), nb::arg("mass_field"));
```

- [ ] **Step 9.3: Python**

`pypgo/fem/mass.py` — append:

```python
class ShellDensityElasticThickness(ShellMassField):
    """rho * h with h read live from an elastic ParameterField channel.

    Shares storage with the energy's elastic field: set_elastic_values()
    updates the thickness seen here, no manual sync.
    """

    def __init__(self, *, density: float, parameter_field, channel: int = 4) -> None:
        from pypgo.fem.fields import ParameterField

        if not isinstance(parameter_field, ParameterField):
            raise TypeError(
                f"parameter_field must be a ParameterField, got {type(parameter_field).__name__}")
        super().__init__(_core.make_shell_density_elastic_thickness(
            float(density), parameter_field._handle, int(channel)))
```

`pypgo/fem/formulations.py` — append to `ShellFormulation`:

```python
    def body_force_parameter_jacobian(self, sim_mesh, acceleration, mass_field):
        """d(body force)/d(elastic parameters) for a parameter-dependent mass field."""
        from pypgo.sparse import SparseMatrix

        accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
        if accel.size != 3:
            raise ValueError(f"acceleration must be a 3-vector, got length {accel.size}")
        _require_sim_mesh(sim_mesh)
        self._require_shell_mass_field(mass_field)
        return SparseMatrix(
            _core.compute_shell_formulation_body_force_parameter_jacobian(
                sim_mesh._handle, self._handle, accel.tolist(), mass_field._handle))
```

`pypgo/fem/__init__.py`: add `ShellDensityElasticThickness` to the mass import.

- [ ] **Step 9.4: Build, test, commit**

```bash
cmake --build --preset base --target pypgo_core
python -m pytest tests/pypgo/test_mass_fields.py -q          # PASS
git add src/python/pypgo pypgo/fem tests/pypgo/test_mass_fields.py
git commit -m "feat(mass): python API for elastic-thickness shell mass and parameter jacobian"
```

## Phase 4 — torch external load + demo

### Task 10: `external_load` on the equilibrium layer + `SelfWeightGravity`

**Files:**
- Modify: `pypgo/fem/torch.py`
- Modify: `pypgo/fem/mass.py`
- Modify: `pypgo/fem/__init__.py`
- Create: `tests/pypgo/test_torch_external_load.py`

- [ ] **Step 10.1: Write the failing test — `tests/pypgo/test_torch_external_load.py`**

```python
import numpy as np
import pytest

import pypgo as pgo
import pypgo.fem as pf

torch = pytest.importorskip("torch")


def _setup(nx=2, ny=2):
    def vid(i, j):
        return i * (ny + 1) + j

    vertices = np.array(
        [[i / nx, j / ny, 0.0] for i in range(nx + 1) for j in range(ny + 1)],
        dtype=np.float64,
    )
    triangles = []
    for i in range(nx):
        for j in range(ny):
            triangles.append([vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)])
            triangles.append([vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)])
    triangles = np.asarray(triangles, dtype=np.int64)
    surface = pgo.mesh.TriMeshData(vertices, triangles)
    material = pf.KoiterStVKShellMaterial(thickness=1e-3, E_membrane=2e4, nu_membrane=0.35)
    sim = pf.SimulationMesh.create_shell(surface, material)

    base_row = np.array([2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3], dtype=np.float64)
    elastic = np.tile(base_row, (triangles.shape[0], 1))
    plastic = np.ones((triangles.shape[0], 1), dtype=np.float64)
    energy = pf.deformation_energy(
        sim,
        elastic=pf.KoiterStVK(),
        elastic_field=pf.ElementwiseField(values=elastic),
        plastic=pf.ShellPlasticity(dofs=1),
        plastic_field=pf.ElementwiseField(values=plastic),
        formulation=pf.KoiterShell(),
        options=pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
    )

    mass_field = pf.ShellDensityElasticThickness(
        density=1000.0, parameter_field=energy.elastic_field, channel=4)
    load = pf.SelfWeightGravity(
        formulation=pf.KoiterShell(), sim_mesh=sim, mass_field=mass_field,
        acceleration=[0.0, 0.0, -20.0])

    fixed_vertices = np.flatnonzero(np.isclose(vertices[:, 1], 1.0)).astype(np.int64)
    fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3, dtype=np.int64)).ravel()
    layer = pgo.fem.ElasticStaticEquilibriumLayer(
        energy=energy,
        fixed_dofs=fixed_dofs,
        fixed_values=np.zeros(fixed_dofs.size),
        surface_vertices=vertices,
        surface_vertex_ids=np.arange(vertices.shape[0], dtype=np.int64),
        inner_optimizer=pgo.solver.NewtonOptimizer(max_iterations=200, gradient_tolerance=1e-11),
        external_load=load,
    )
    return layer, elastic, vertices


def test_self_weight_forward_responds_to_thickness():
    layer, elastic, _vertices = _setup()
    b = torch.tensor(elastic.ravel(), dtype=torch.float64)
    out_thin = layer(b).detach().numpy().copy()

    thick = elastic.copy()
    thick[:, 4] *= 4.0
    layer.reset_warm_start()
    out_thick = layer(torch.tensor(thick.ravel(), dtype=torch.float64)).detach().numpy().copy()
    # 4x thickness: weight x4, bending stiffness ~x64 -> sag must change.
    assert not np.allclose(out_thin, out_thick, atol=1e-10)


def test_self_weight_gradient_matches_finite_differences():
    layer, elastic, vertices = _setup()
    rng = np.random.default_rng(1)
    R = rng.standard_normal(vertices.shape)

    def loss_np(b_flat):
        layer.reset_warm_start()
        out = layer(torch.tensor(b_flat, dtype=torch.float64))
        return float((out * torch.tensor(R)).sum())

    b0 = elastic.ravel().copy()
    b = torch.tensor(b0, dtype=torch.float64, requires_grad=True)
    layer.reset_warm_start()
    loss = (layer(b) * torch.tensor(R)).sum()
    loss.backward()
    grad = b.grad.numpy()

    # Probe a thickness dof and a membrane-E dof on an interior element
    # (magnitude-scaled FD steps per repo convention).
    num_channels = 5
    for dof in (0 * num_channels + 4, 1 * num_channels + 4, 0 * num_channels + 0):
        h = 1e-6 * max(abs(b0[dof]), 1e-8)
        bp = b0.copy(); bp[dof] += h
        bm = b0.copy(); bm[dof] -= h
        fd = (loss_np(bp) - loss_np(bm)) / (2 * h)
        assert grad[dof] == pytest.approx(fd, rel=2e-3, abs=1e-8), f"dof {dof}"


def test_external_load_rejected_on_plastic_layer():
    layer, elastic, _vertices = _setup()
    with pytest.raises(ValueError):
        pgo.fem.PlasticStaticEquilibriumLayer(
            energy=layer.energy,
            fixed_dofs=layer.fixed_dofs,
            fixed_values=layer.fixed_values,
            surface_vertices=layer.surface_vertices,
            surface_vertex_ids=layer.surface_vertex_ids,
            external_load=layer.external_load,
        )
```

Run: `python -m pytest tests/pypgo/test_torch_external_load.py -q` → FAIL (`SelfWeightGravity` missing / unexpected `external_load` kwarg).

- [ ] **Step 10.2: `SelfWeightGravity` — append to `pypgo/fem/mass.py`**

```python
class SelfWeightGravity:
    """External-load provider: shell self-weight from a parameter-coupled mass field.

    Implements the ``ElasticStaticEquilibriumLayer`` external_load protocol:
    ``force()`` and ``parameter_jacobian()`` evaluated at the parameter
    field's current values.
    """

    def __init__(self, *, formulation, sim_mesh, mass_field, acceleration) -> None:
        from pypgo.fem.formulations import ShellFormulation

        if not isinstance(formulation, ShellFormulation):
            raise TypeError(
                f"formulation must be a ShellFormulation, got {type(formulation).__name__}")
        if not isinstance(mass_field, ShellMassField):
            raise TypeError(
                f"mass_field must be a ShellMassField, got {type(mass_field).__name__}")
        self._formulation = formulation
        self._sim_mesh = sim_mesh
        self._mass_field = mass_field
        self._acceleration = np.asarray(acceleration, dtype=np.float64).reshape(3)

    def force(self) -> np.ndarray:
        return self._formulation.body_force(self._sim_mesh, self._acceleration, self._mass_field)

    def parameter_jacobian(self):
        return self._formulation.body_force_parameter_jacobian(
            self._sim_mesh, self._acceleration, self._mass_field)
```

Export `SelfWeightGravity` from `pypgo/fem/__init__.py`.

- [ ] **Step 10.3: `external_load` in `pypgo/fem/torch.py`**

(a) Add the import near the existing `solver` import: `from pypgo import energy as _energy_mod` (match the module's actual import style — it currently imports `solver`; mirror that pattern).

(b) `_BaseStaticEquilibriumLayer.__init__`: add keyword `external_load=None`; after the `inner_optimizer` validation block insert:

```python
        if external_load is not None:
            if self._parameter_name != "elastic":
                raise ValueError("external_load is only supported on ElasticStaticEquilibriumLayer")
            if not callable(getattr(external_load, "force", None)) or not callable(
                getattr(external_load, "parameter_jacobian", None)
            ):
                raise TypeError("external_load must provide force() and parameter_jacobian()")
        self.external_load = external_load
```

(c) Add a method on the base layer:

```python
    def _build_objective(self):
        """Objective for the inner solve; re-adds the parameter-dependent load at current b."""
        if self.external_load is None:
            return self.objective_energy
        load = np.asarray(self.external_load.force(), dtype=np.float64)
        if load.shape != (self.energy.num_dofs,):
            raise ValueError(
                f"external_load.force() must return shape ({self.energy.num_dofs},), got {load.shape}")
        return _energy_mod.EnergySet([
            (self.objective_energy, 1.0),
            (_energy_mod.LinearEnergy(-load), 1.0),
        ])
```

(d) `_StaticEquilibriumFunction.forward`: replace

```python
        problem = solver.OptimizationProblem(objective=layer.objective_energy)
```

with

```python
        objective = layer._build_objective()
        problem = solver.OptimizationProblem(objective=objective)
```

(the backward Hessian can keep using `layer.objective_energy` — the added `LinearEnergy` has zero Hessian).

(e) `ElasticStaticEquilibriumLayer._parameter_jacobian`: replace the body with

```python
    def _parameter_jacobian(self, displacement) -> np.ndarray:
        jac = self.energy.elastic_jacobian(displacement).to_dense()
        if self.external_load is not None:
            # Inner gradient is ∇E(u,b) - f_g(b); its mixed b-derivative
            # therefore subtracts d f_g / d b.
            jac = jac - self.external_load.parameter_jacobian().to_dense()
        return jac
```

Note `backward` calls `layer._set_parameter_values(ctx.parameter_values)` before this — the mass field already sees the right b.

- [ ] **Step 10.4: Run tests, full suite, commit**

```bash
python -m pytest tests/pypgo/test_torch_external_load.py -q   # PASS
python -m pytest tests/pypgo -x -q                             # PASS
git add pypgo/fem tests/pypgo/test_torch_external_load.py
git commit -m "feat(fem): external_load provider on equilibrium layer, SelfWeightGravity, adjoint load term"
```

### Task 11: Demo generator upgrade + regeneration

**Files:**
- Modify: `examples/scripts/generate_elastic_material_optimization_demo.py`
- Regenerate: `examples/elastic_material_optimization_demo.ipynb`

- [ ] **Step 11.1: Rework the gravity cell (generator lines ~230–256)**

Replace the manual lumped-gravity block with a two-mode cell:

```python
        GRAVITY_MODE = "self_weight"  # "self_weight" | "fixed_area_load"
        sag_strength = 20.0
        gravity_accel = np.array([0.0, 0.0, -sag_strength], dtype=np.float64)

        if GRAVITY_MODE == "fixed_area_load":
            # Fixed downward areal load (the old behavior, now honestly named):
            # the load does NOT change when thickness is optimized.
            vertex_area = np.zeros(vertices.shape[0], dtype=np.float64)
            for tri in triangles:
                a, b, c = vertices[tri]
                vertex_area[tri] += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 3.0
            gravity_force = np.zeros(energy.num_dofs, dtype=np.float64)
            gravity_force.reshape((-1, 3))[:] = 1.0 * vertex_area[:, None] * gravity_accel
            external_load = None
            objective = pe.EnergySet([(energy, 1.0), (pe.LinearEnergy(-gravity_force), 1.0)])
        else:
            # Physically correct self-weight: f_g = rho * h * area/3 per corner
            # vertex, with h read live from elastic channel 4. rho chosen so
            # rho*h0 = 1 kg/m^2 matches the old load magnitude at b0.
            mass_field = pf.ShellDensityElasticThickness(
                density=1000.0, parameter_field=energy.elastic_field, channel=4)
            external_load = pf.SelfWeightGravity(
                formulation=pf.KoiterShell(), sim_mesh=sim,
                mass_field=mass_field, acceleration=gravity_accel)
            gravity_force = external_load.force()
            objective = energy  # the layer / target solve add the load at current b

        def _objective_at_current_b():
            if external_load is None:
                return objective
            return pe.EnergySet([(energy, 1.0), (pe.LinearEnergy(-external_load.force()), 1.0)])
```

and inside `solve_surface_for_elastic`, after `energy.set_elastic_values(elastic_values)`, replace `objective=objective` with `objective=_objective_at_current_b()` so the synthetic-target solve also carries thickness-dependent weight.

- [ ] **Step 11.2: Layer construction (generator lines ~276–283)**

Pass the provider through:

```python
        equilibrium_layer = pgo.fem.ElasticStaticEquilibriumLayer(
            energy=energy,
            objective_energy=objective,
            external_load=external_load,
            ...,  # keep the existing fixed_dofs/fixed_values/surface args unchanged
        )
```

- [ ] **Step 11.3: Fix stale references**

- The `print("gravity force norm", ...)` line and the `np.savez(..., gravity_force=gravity_force, ...)` call keep working since `gravity_force` is defined in both modes (in self-weight mode it is the load at b0).
- Update the markdown cells that describe the gravity model (sections "Add gravity..." and the backward-pass explanation) to describe the two modes and the new `- ∂f_g/∂b` adjoint term in self-weight mode.

- [ ] **Step 11.4: Regenerate the notebook**

```bash
python examples/scripts/generate_elastic_material_optimization_demo.py
python -m pytest tests/pypgo/test_notebook.py -q
```

Expected: notebook regenerated under `examples/`, tier-1 notebook tests PASS. If the environment supports it, also smoke-run the notebook: `RUN_NOTEBOOKS=1 python -m pytest tests/pypgo/test_notebook.py -k elastic_material -q`.

- [ ] **Step 11.5: Full suite + commit**

```bash
python -m pytest tests/pypgo -q
git add examples/scripts/generate_elastic_material_optimization_demo.py examples/elastic_material_optimization_demo.ipynb
git commit -m "feat(demo): self-weight mode with thickness-coupled gravity and corrected adjoint"
```

---

## Spec coverage checklist

- §1 MassField hierarchy → Tasks 1, 6, 8 (interface change: `compatibleWith(meshType)` predicate instead of single-enum `compatibleMeshType()`, see Task 1 note).
- §2 formulation migration: volume generic loop + `massQuadrature()` → Tasks 2–3; shell lumped loop → Task 6; parameter Jacobian → Task 8; legacy signatures deleted → Task 3; surface embedding deliberately untouched.
- §3 Python layer → Tasks 4, 7, 9 (`pypgo/fem/mass.py`, formulations methods, `volume_density_from_veg`, caller migration: `volume_ipc.py` Task 4, dragon notebook Task 5, Hermite tests Task 4).
- §4 demo correction → Tasks 10 (layer + provider) and 11 (generator + notebook).
- §5 testing → `test_mass_fields.py` (Tasks 4/7/9), `test_torch_external_load.py` (Task 10), migrated Hermite tests (Task 4), notebook tier-1 (Task 11).
