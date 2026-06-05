# Hermite Dynamic Contact Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make tricubic Hermite usable in the C++ dynamic/contact chain by adding formulation-aware mass, body-force, surface-embedding, and boundary-DOF operators in C++, with Python reduced to bindings and thin convenience wrappers.

**Architecture:** The deformation stack already owns formulation DOFs (`nvtx*3` for trilinear, `nvtx*24` for Hermite), and the contact stack already accepts `x_surf = X_surf_rest + W * u_sim` through mapped contact. This plan adds the missing C++ formulation dynamics operators so mass, gravity/body force, contact embedding, fixed DOFs, `EnergySet`, and `DynamicSimulation` all agree on one simulation DOF space. Trilinear/tet remain the old `VolumeMesh.mass_matrix()` / barycentric `W` special cases; Hermite builds a consistent `nvtx*24` mass and `W_hermite`.

**Tech Stack:** C++17, Eigen sparse matrices, Vega `VolumetricMesh`, `BarycentricCoordinates`, `solidDeformationModel`, nanobind, Python/NumPy wrappers, GTest, pytest, CMake/Ninja.

---

## Confirmed Design Decisions

- Core implementation belongs in C++. Python only binds and wraps it.
- Cubic mesh scope is the existing Vega `CubicMesh`: uniform, axis-aligned grid. Hermite derivative transform is `T = I`; rotated/flipped local-axis chain-rule transforms are deferred.
- Contact core is already Hermite-ready. IPC/floor/sampled-penalty energies should not know Hermite modes; they consume `ContactSurfaceSpec + W`.
- Dynamic stepper is already `numDofs()` generic. It needs only dimension-consistent mass, state, external force, fixed DOFs, and energies.
- Hermite mass is consistent, not lumped:
  `M_ij = integral rho * H_i * H_j dV`, expanded over x/y/z.
- Body force is assembled directly in DOF space:
  `f_i = integral rho * H_i * a dV`; derivative modes generally receive nonzero generalized force.
- Boundary condition semantics are explicit:
  `value` fixes mode 0 only, `first` fixes modes 0..3, and `all` fixes modes 0..7. The first robust drop demo may use `all`.
- Legacy `TimeIntegrator` vertex API has been deleted by the user and must not be reintroduced.

## File Structure

- Create: `src/core/solidDeformationModel/formulations/formulationDynamics.h`
  - Declares C++ formulation-aware dynamic operators and Hermite boundary helpers.
- Create: `src/core/solidDeformationModel/formulations/formulationDynamics.cpp`
  - Implements trilinear/tet fallback operators and Hermite mass/body-force/surface-embedding assembly.
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
  - Registers the new C++ files.
- Create: `tests/src/core/solidDeformationModel/formulations/formulationDynamics_gtest.cpp`
  - C++ tests for mass, body force, Hermite W, and boundary helpers.
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`
  - Registers the new GTest target.
- Modify: `src/python/pypgo/bindings/mesh_bindings.cpp`
  - Binds C++ formulation dynamics operators and Hermite boundary helpers.
- Modify: `pypgo/fem.py`
  - Replaces Python assembly logic with calls into `_core`.
- Modify: `tests/pypgo/test_hermite_dynamic_helpers.py`
  - Keeps Python-level behavior tests as binding/integration tests.
- Modify: `pypgo/examples/scripts/generate_tricubic_hermite_box_drop_ipc_demo.py`
  - Keeps the notebook script, now exercising C++-backed helpers.
- Modify: `pypgo/examples/tricubic_hermite_box_drop_ipc_demo.ipynb`
  - Regenerate after wrapper changes.

---

## Task 1: C++ Formulation Dynamics RED Tests

**Files:**
- Create: `tests/src/core/solidDeformationModel/formulations/formulationDynamics_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add the C++ test target**

Add this target near the existing Hermite formulation tests:

```cmake
add_executable(formulationDynamics_gtest formulations/formulationDynamics_gtest.cpp)
target_link_libraries(formulationDynamics_gtest PRIVATE GTest::gtest_main solidDeformationModel volumetricMesh mesh)
set_property(TARGET formulationDynamics_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(formulationDynamics_gtest)
```

- [ ] **Step 2: Write failing tests**

Create `tests/src/core/solidDeformationModel/formulations/formulationDynamics_gtest.cpp` with tests that include the future header:

```cpp
#include "gtest/gtest.h"
#include "formulations/formulationDynamics.h"
#include "formulations/formulation.h"
#include "cubicMesh.h"
#include "generateMassMatrix.h"

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <vector>

using namespace pgo;
using namespace pgo::SolidDeformationModel;

namespace
{
std::unique_ptr<VolumetricMeshes::CubicMesh> makeSingleCube(double density)
{
  double vertices[24] = {
    0,0,0, 1,0,0, 1,1,0, 0,1,0,
    0,0,1, 1,0,1, 1,1,1, 0,1,1
  };
  int elements[8] = { 0,1,2,3,4,5,6,7 };
  return std::make_unique<VolumetricMeshes::CubicMesh>(8, vertices, 1, elements, 1e6, 0.45, density);
}

double denseAt(const EigenSupport::SpMatD &M, int r, int c)
{
  return M.coeff(r, c);
}
}  // namespace

TEST(FormulationDynamicsGTest, HermiteMassHasCorrectShapeSymmetryAndConstantVelocityEnergy)
{
  auto mesh = makeSingleCube(2.0);
  EigenSupport::SpMatD M = buildFormulationMassMatrix(*mesh, TricubicHermiteFormulation{});
  ASSERT_EQ(M.rows(), 8 * 24);
  ASSERT_EQ(M.cols(), 8 * 24);

  EigenSupport::VXd qdot = EigenSupport::VXd::Zero(8 * 24);
  EigenSupport::V3d v(0.4, -0.2, 0.7);
  for (int vertex = 0; vertex < 8; vertex++)
    qdot.segment<3>(vertex * 24) = v;

  const double kinetic = 0.5 * qdot.dot(M * qdot);
  const double expected = 0.5 * 2.0 * v.squaredNorm();
  EXPECT_NEAR(kinetic, expected, 1e-10);

  for (int k = 0; k < M.outerSize(); ++k)
    for (EigenSupport::SpMatD::InnerIterator it(M, k); it; ++it)
      EXPECT_NEAR(it.value(), denseAt(M, it.col(), it.row()), 1e-12);
}

TEST(FormulationDynamicsGTest, HermiteBodyForceHasCorrectTotalAndDerivativeEntries)
{
  auto mesh = makeSingleCube(3.0);
  EigenSupport::V3d a(0.0, -9.8, 0.0);
  EigenSupport::VXd f = buildFormulationBodyForce(*mesh, TricubicHermiteFormulation{}, a);
  ASSERT_EQ(f.size(), 8 * 24);

  EigenSupport::V3d valueForce = EigenSupport::V3d::Zero();
  double derivativeNorm = 0.0;
  for (int vertex = 0; vertex < 8; vertex++) {
    valueForce += f.segment<3>(vertex * 24);
    derivativeNorm += f.segment(vertex * 24 + 3, 21).norm();
  }

  EXPECT_TRUE(valueForce.isApprox(3.0 * a, 1e-10));
  EXPECT_GT(derivativeNorm, 0.0);
}

TEST(FormulationDynamicsGTest, HermiteSurfaceEmbeddingReproducesAffineDisplacement)
{
  auto mesh = makeSingleCube(1.0);
  EigenSupport::MXd points(3, 3);
  points << 0.25, 0.50, 0.75,
            1.00, 0.00, 0.50,
            0.00, 1.00, 0.00;

  EigenSupport::SpMatD W = buildFormulationSurfaceEmbeddingMatrix(*mesh, TricubicHermiteFormulation{}, points);
  ASSERT_EQ(W.rows(), points.rows() * 3);
  ASSERT_EQ(W.cols(), 8 * 24);

  EigenSupport::VXd q = EigenSupport::VXd::Zero(8 * 24);
  EigenSupport::M3d A;
  A << 0.1, 0.2, 0.0,
       0.0, -0.1, 0.3,
       0.05, 0.0, 0.2;
  EigenSupport::V3d b(0.3, -0.4, 0.2);
  for (int vertex = 0; vertex < 8; vertex++) {
    const auto &X = mesh->getVertex(vertex);
    const int base = vertex * 24;
    q.segment<3>(base) = A * EigenSupport::V3d(X[0], X[1], X[2]) + b;
    q.segment<3>(base + 3) = A.col(0);
    q.segment<3>(base + 6) = A.col(1);
    q.segment<3>(base + 9) = A.col(2);
  }

  EigenSupport::VXd mapped = W * q;
  for (int i = 0; i < points.rows(); i++) {
    EigenSupport::V3d p = points.row(i).transpose();
    EXPECT_TRUE(mapped.segment<3>(i * 3).isApprox(A * p + b, 1e-12));
  }
}

TEST(FormulationDynamicsGTest, TrilinearMassAndEmbeddingMatchLegacyOperators)
{
  auto mesh = makeSingleCube(2.0);
  EigenSupport::SpMatD legacyMass;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(mesh.get(), legacyMass, true);
  EigenSupport::SpMatD mass = buildFormulationMassMatrix(*mesh, LinearCubicFormulation{});
  EXPECT_TRUE(mass.isApprox(legacyMass, 1e-12));
}

TEST(FormulationDynamicsGTest, HermiteBoundaryHelpersExposePolicies)
{
  std::vector<int> value = hermiteVertexDofs({ 2 }, HermiteBoundaryPolicy::Value);
  std::vector<int> first = hermiteVertexDofs({ 2 }, HermiteBoundaryPolicy::First);
  std::vector<int> all = hermiteVertexDofs({ 2 }, HermiteBoundaryPolicy::All);

  ASSERT_EQ(value.size(), 3);
  EXPECT_EQ(value.front(), 48);
  ASSERT_EQ(first.size(), 12);
  EXPECT_EQ(first.front(), 48);
  EXPECT_EQ(first.back(), 59);
  ASSERT_EQ(all.size(), 24);
  EXPECT_EQ(all.front(), 48);
  EXPECT_EQ(all.back(), 71);
}
```

- [ ] **Step 3: Run RED**

Run:

```bash
cmake --build build/base --target formulationDynamics_gtest
```

Expected: fail because `formulations/formulationDynamics.h` does not exist.

---

## Task 2: Implement C++ Formulation Dynamics Operators

**Files:**
- Create: `src/core/solidDeformationModel/formulations/formulationDynamics.h`
- Create: `src/core/solidDeformationModel/formulations/formulationDynamics.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add the public C++ API**

Create `formulationDynamics.h`:

```cpp
#pragma once

#include "EigenDef.h"

#include <vector>

namespace pgo
{
namespace VolumetricMeshes { class VolumetricMesh; }
namespace SolidDeformationModel
{

class Formulation;

enum class HermiteBoundaryPolicy
{
  Value,
  First,
  All,
};

EigenSupport::SpMatD buildFormulationMassMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation);

EigenSupport::VXd buildFormulationBodyForce(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation,
  const EigenSupport::V3d &acceleration);

EigenSupport::SpMatD buildFormulationSurfaceEmbeddingMatrix(
  const VolumetricMeshes::VolumetricMesh &mesh,
  const Formulation &formulation,
  const EigenSupport::MXd &surfaceVertices);

std::vector<int> hermiteVertexDofs(
  const std::vector<int> &vertexIds,
  HermiteBoundaryPolicy policy);

std::vector<int> hermiteFaceDofs(
  const VolumetricMeshes::VolumetricMesh &mesh,
  int axis,
  bool maxSide,
  HermiteBoundaryPolicy policy);

}  // namespace SolidDeformationModel
}  // namespace pgo
```

- [ ] **Step 2: Register C++ files**

In `src/core/solidDeformationModel/CMakeLists.txt`, add:

```cmake
formulations/formulationDynamics.h
```

to headers and:

```cmake
formulations/formulationDynamics.cpp
```

to sources.

- [ ] **Step 3: Implement trilinear/tet fallback**

In `formulationDynamics.cpp`, implement non-Hermite mass using:

```cpp
VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(&mesh, M, true);
```

and non-Hermite surface embedding using:

```cpp
InterpolationCoordinates::BarycentricCoordinates bc(numPoints, surfaceVertices.data(), &mesh);
return bc.generateInterpolationMatrix();
```

Reject `shell_koiter` from these volume helpers with `std::invalid_argument`.

- [ ] **Step 4: Implement Hermite mass**

For `formulation.getName() == "hex_tricubic_hermite"`:

```text
for each element:
  volumeWeight = mesh.getElementVolume(ele) * mesh.getElementDensity(ele)
  local64 = sum_q weight_q * outer(H_q, H_q) * volumeWeight
  scatter local64 to vector DOFs:
    global = vertexId * 24 + mode * 3 + coord
```

Use `HexTricubicHermiteBasis` and `GaussLegendreHexQuadrature4` so C++ has one Hermite basis convention.

- [ ] **Step 5: Implement Hermite body force**

For `TricubicHermiteFormulation`:

```text
for each element:
  local64 = sum_q weight_q * H_q * rho * volume
  scatter local64[node] * acceleration[coord]
```

- [ ] **Step 6: Implement Hermite surface embedding**

Use `BarycentricCoordinates` to locate each surface point in a cubic element, then recover unit-cube coordinates from the trilinear weights:

```cpp
xi   = w[1] + w[2] + w[5] + w[6];
eta  = w[2] + w[3] + w[6] + w[7];
zeta = w[4] + w[5] + w[6] + w[7];
```

Evaluate the 64 Hermite basis values and scatter:

```cpp
row = surfacePoint * 3 + coord;
col = vertexId * 24 + mode * 3 + coord;
value = H[corner * 8 + mode];
```

- [ ] **Step 7: Implement Hermite boundary helpers**

Policies map to modes:

```cpp
Value -> {0}
First -> {0,1,2,3}
All   -> {0,1,2,3,4,5,6,7}
```

`hermiteFaceDofs` finds vertices whose coordinate on `axis` equals min or max within a small absolute tolerance.

- [ ] **Step 8: Run GREEN**

Run:

```bash
cmake --build build/base --target formulationDynamics_gtest
build/base/tests/src/core/solidDeformationModel/formulationDynamics_gtest
```

Expected: all formulation dynamics tests pass.

---

## Task 3: Bind C++ Operators And Thin Python Wrappers

**Files:**
- Modify: `src/python/pypgo/bindings/mesh_bindings.cpp`
- Modify: `pypgo/fem.py`
- Test: `tests/pypgo/test_hermite_dynamic_helpers.py`

- [ ] **Step 1: Add binding functions**

In `mesh_bindings.cpp`, add helpers that dispatch formulation strings:

```cpp
std::unique_ptr<SolidDeformationModel::Formulation> makeVolumeFormulation(const std::string &name);
PySparseMatrix compute_formulation_mass_matrix(const PyVolumeMesh &volumeMesh, const std::string &formulationName);
std::vector<double> compute_formulation_body_force(const PyVolumeMesh &volumeMesh, const std::string &formulationName, const std::vector<double> &accel);
PySparseMatrix compute_formulation_surface_embedding_matrix(const PyVolumeMesh &volumeMesh, const std::string &formulationName, const std::vector<double> &surfaceVerticesFlat);
std::vector<int> hermite_vertex_dofs(const std::vector<int> &vertexIds, const std::string &policy);
std::vector<int> hermite_face_dofs(const PyVolumeMesh &volumeMesh, int axis, bool maxSide, const std::string &policy);
```

Bind them as:

```cpp
m.def("compute_formulation_mass_matrix", &compute_formulation_mass_matrix);
m.def("compute_formulation_body_force", &compute_formulation_body_force);
m.def("compute_formulation_surface_embedding_matrix", &compute_formulation_surface_embedding_matrix);
m.def("hermite_vertex_dofs", &hermite_vertex_dofs);
m.def("hermite_face_dofs", &hermite_face_dofs);
```

- [ ] **Step 2: Replace Python assembly with thin wrappers**

In `pypgo/fem.py`, keep public names but make them call `_core`:

```python
def formulation_mass_matrix(volume, formulation):
    return SparseMatrix(_core.compute_formulation_mass_matrix(volume._core_obj, _formulation_name(formulation)))

def body_force(volume, formulation, acceleration):
    accel = np.asarray(acceleration, dtype=np.float64).reshape(-1)
    if accel.size != 3:
        raise ValueError(...)
    return np.asarray(_core.compute_formulation_body_force(volume._core_obj, _formulation_name(formulation), accel.tolist()), dtype=np.float64)

def surface_embedding_matrix(volume, surface_vertices, formulation):
    points = np.asarray(surface_vertices, dtype=np.float64)
    return SparseMatrix(_core.compute_formulation_surface_embedding_matrix(volume._core_obj, _formulation_name(formulation), points.reshape(-1).tolist()))
```

Remove Python-only Hermite quadrature/mass/body-force assembly helpers.

- [ ] **Step 3: Run Python binding tests**

Run:

```bash
cmake --build build/base --target pypgo_core
conda run -n libpgo python -m pytest tests/pypgo/test_hermite_dynamic_helpers.py -q
```

Expected: all Hermite dynamic helper tests pass, now backed by C++.

---

## Task 4: Notebook And Regression

**Files:**
- Modify: `pypgo/examples/scripts/generate_tricubic_hermite_box_drop_ipc_demo.py`
- Modify: `pypgo/examples/tricubic_hermite_box_drop_ipc_demo.ipynb`
- Modify: `pypgo/examples/scripts/README.md`
- Modify: `tests/pypgo/test_notebook.py`

- [ ] **Step 1: Ensure notebook uses only public Python wrappers**

The notebook script should call:

```python
mass = pf.formulation_mass_matrix(volume, pf.TricubicHermite())
force = pf.body_force(volume, pf.TricubicHermite(), gravity)
W = pf.surface_embedding_matrix(volume, surface_vertices, pf.TricubicHermite())
fixed = pf.hermite_face_dofs(volume, axis="y", side="max", policy="all")
```

No notebook cell should assemble Hermite basis values manually.

- [ ] **Step 2: Regenerate and execute notebook**

Run:

```bash
conda run -n libpgo python pypgo/examples/scripts/generate_tricubic_hermite_box_drop_ipc_demo.py
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute pypgo/examples/tricubic_hermite_box_drop_ipc_demo.ipynb --output /tmp/pypgo_tricubic_hermite_box_drop_ipc_demo_executed.ipynb --ExecutePreprocessor.timeout=400
```

Expected: notebook executes successfully.

- [ ] **Step 3: Run focused Python regressions**

Run:

```bash
conda run -n libpgo python -m pytest tests/pypgo/test_hermite_dynamic_helpers.py tests/pypgo/test_contact.py tests/pypgo/test_dynamic_stepper.py tests/pypgo/test_notebook.py -q
```

Expected: all pass, skips unchanged.

- [ ] **Step 4: Run C++ regressions**

Run:

```bash
ctest --test-dir build/base -R "formulationDynamics|tricubicHermite|hexTricubicHermite|gaussLegendreHexQuadrature4|DeformationModel|Formulation|contact|runIPC|runSim" --output-on-failure
```

Expected: all selected tests pass.

---

## Self-Review

**Spec coverage**

- Formal implementation plan: covered by this file.
- Self review: this section.
- Execute: tasks are directly executable, with RED/GREEN test gates.
- Legacy `TimeIntegrator` vertex API deletion: no task reintroduces it; dynamic remains state-vector/mass/force based.
- C++ chain support: core mass/body force/W/boundary helpers are in `solidDeformationModel`, not Python.
- Python only binding: `pypgo/fem.py` remains public API but delegates to `_core`.
- Contact design decisions: no contact core changes; mapped contact consumes `W`.
- Boundary semantics: policies are explicit and available in C++ and Python.

**Placeholder scan**

- No "TBD" or "implement later" placeholders.
- Deferred general rotated/flipped-cell transforms are explicitly out of scope, not a missing task.
- Each task lists exact files, commands, expected results, and API signatures.

**Type consistency**

- `buildFormulationMassMatrix`, `buildFormulationBodyForce`, `buildFormulationSurfaceEmbeddingMatrix`, `hermiteVertexDofs`, and `hermiteFaceDofs` are declared once and referenced consistently.
- Python wrappers call `_core.compute_formulation_*`, matching the binding names.
- Hermite global indexing is consistent everywhere: `vertex * 24 + mode * 3 + coord`.
