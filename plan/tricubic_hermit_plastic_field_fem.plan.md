# Tricubic Hermite Plastic Field FEM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Do not stage or commit unless the user explicitly asks for git integration.

**Goal:** Add a true tricubic Hermite FEM path whose geometry uses Hermite nodal DOFs, whose plastic prestrain `Fp` is a design variable, and whose first two plastic design spaces are constant field followed by sparse control points with kernel interpolation.

**Architecture:** Keep the existing tet/cubic vertex-only path intact. Add a Hermite-specific geometry DOF layout, a tricubic Hermite evaluator/deformation model, and a plastic design field abstraction that evaluates design variables at quadrature points. The first implementation uses one global constant `Fp` parameter block; the second implementation reuses the same interface with sparse control points and precomputed kernel weights.

**Tech Stack:** C++20, Eigen, existing `solidDeformationModel` elastic/plastic material APIs, GoogleTest, CMake, optional references from `/Users/jinceyang/Documents/tricubic_hermit/output/jupyter-notebook`.

---

## Design Decisions

1. **True Hermite geometry DOFs are separate from plastic design DOFs.**

   ```text
   x_dofs: Hermite geometry field
   z_dofs: plastic design field parameters
   ```

   The geometry unknown vector is not `3 * numVertices`. For a cubic/Hermite node it is:

   ```text
   8 Hermite types * 3 coordinates = 24 DOFs per node
   ```

2. **Quadrature points do not own geometry DOFs.**

   Quadrature points evaluate `JX`, `Jx`, `F`, `Fp`, `Fe`, energy, gradients, and Hessians. In this plan, quadrature points sample the plastic design field but do not become independent plastic variables.

3. **Phase 1 plastic field is globally constant.**

   The first design vector is:

   ```text
   z_constant.size() == 6
   a_q = z_constant for every element and quadrature point
   Fp_q = PlasticModel3D6DOF::computeA(a_q)
   ```

   This is intentionally lower-dimensional than per-element or per-quadrature plastic variables. It proves the chain from design variable to `Fp_q` to energy derivatives before spatial variation is added.

4. **Phase 2 plastic field uses sparse control points + kernel interpolation.**

   Each control point stores a 6D plastic parameter. Each quadrature point precomputes interpolation weights from rest-space location `X_q`.

   ```text
   a_q = sum_k w_qk * z_k
   z.size() == 6 * numControlPoints
   d a_q / d z_k = w_qk * I_6
   ```

5. **Do not interpolate raw `Fp` matrices.**

   Interpolate the 6D parameter `a`, then call `PlasticModel3D6DOF` to construct `Fp`. This matches existing derivative code in `PlasticModel3D6DOF` and keeps the first implementation small.

6. **First scope is structured/axis-aligned cubic meshes.**

   The first true Hermite path assumes a `CubicMesh`-style element order and a shared global parametric chart. General unstructured hex Hermite continuity and cross-face frame transforms are out of scope for this plan.

7. **All Hermite vector DOF arrays use row-major scalar storage.**

   Any `double Q64x3[192]` buffer is laid out as:

   ```text
   Q64x3[i * 3 + 0] = x component of scalar Hermite basis i
   Q64x3[i * 3 + 1] = y component of scalar Hermite basis i
   Q64x3[i * 3 + 2] = z component of scalar Hermite basis i
   ```

   Do not pass `Eigen::Matrix<double, 64, 3>::data()` into this API unless the matrix is explicitly `Eigen::RowMajor`. Eigen's default column-major storage is incompatible with this evaluator contract.

8. **Element corner ordering is a hard API contract.**

   `TricubicHermiteBasis::flatIndex(a, b, c, type)` uses `vertex = a + 2 * b + 4 * c`. Existing `CubicMesh` element order is different for the top face of each z-layer: Vega order is `{000, 100, 110, 010, 001, 101, 111, 011}` while Hermite basis order is `{000, 100, 010, 110, 001, 101, 011, 111}`. Any `CubicMesh` integration must explicitly reorder local element nodes before constructing or gathering Hermite local DOFs.

## Non-Negotiable Finite Element Verification Gate

Do not connect this work to `runSim`, IPC, time integration, or any simulation demo until the finite-element checks in this plan pass. The first milestone is a mathematically verified element and assembler, not a visually running simulation.

The implementation must preserve this formula contract:

```text
X = X(xi, eta, zeta)
x = x(xi, eta, zeta)
JX = [X_,xi X_,eta X_,zeta]
Jx = [x_,xi x_,eta x_,zeta]
F = Jx * inv(JX)
Fp = Fp(a)
Fe = F * inv(Fp)
E_q = w_q * det(JX_q) * det(Fp_q) * psi(Fe_q)
```

Every derivative implementation must be checked against independent finite differences before it is trusted:

```text
dE/dx       vs finite difference of element energy
dE/da       vs finite difference of element energy, where a is the element-local per-quadrature plastic parameter block
d2E/dx2*v   vs finite difference of dE/dx
d2E/dxda*v  vs finite difference of dE/dx with respect to the element-local a block
assembler dE/dx vs finite difference of assembled energy
assembler dE/dz vs finite difference of assembled energy, where z is the global plastic design vector
```

Use central differences with `eps = 1e-6` for gradient checks unless a test documents a tighter problem-specific reason. For Hessian-vector checks, compare against central differences of gradients with `eps = 1e-6` and report both max absolute error and max relative error.

Required finite-element sanity checks before any simulation integration:

- rest state has `F = I`, `Fe = I`, and zero energy for identity `Fp`;
- rigid translation produces zero energy and zero geometry gradient;
- affine deformation on an affine Hermite box matches the analytic single-point formula `volume * det(Fp) * psi(F * inv(Fp))` up to quadrature tolerance;
- increasing quadrature order from `4x4x4` to `5x5x5` changes affine tests by less than `1e-10`;
- all local Hessian blocks used by Newton-style solvers are symmetric within `1e-8`;
- all finite-difference derivative checks pass on at least one rest state, one perturbed geometry state, and one non-identity SPD `Fp` state.

## File Map

Create:

- `src/core/solidDeformationModel/tricubicHermiteBasis.h`
- `src/core/solidDeformationModel/tricubicHermiteBasis.cpp`
- `src/core/solidDeformationModel/tricubicHermiteDofLayout.h`
- `src/core/solidDeformationModel/tricubicHermiteDofLayout.cpp`
- `src/core/solidDeformationModel/plasticDesignField.h`
- `src/core/solidDeformationModel/plasticDesignField.cpp`
- `src/core/solidDeformationModel/tricubicHermiteDeformationModel.h`
- `src/core/solidDeformationModel/tricubicHermiteDeformationModel.cpp`
- `src/core/solidDeformationModel/tricubicHermiteAssembler.h`
- `src/core/solidDeformationModel/tricubicHermiteAssembler.cpp`
- `tests/src/core/solidDeformationModel/tricubicHermiteBasis_gtest.cpp`
- `tests/src/core/solidDeformationModel/plasticDesignField_gtest.cpp`
- `tests/src/core/solidDeformationModel/tricubicHermiteDeformationModel_gtest.cpp`
- `tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest.cpp`

Modify:

- `src/core/solidDeformationModel/CMakeLists.txt`
- `tests/src/core/solidDeformationModel/CMakeLists.txt`

Do not modify in this plan:

- `src/tools/runSim/runIPCSim.cpp`
- existing `DeformationModelAssembler`
- existing tet/cubic deformation models
- IPC/contact code

---

## Task 1: Add Tricubic Hermite Basis And Evaluator

**Files:**
- Create: `src/core/solidDeformationModel/tricubicHermiteBasis.h`
- Create: `src/core/solidDeformationModel/tricubicHermiteBasis.cpp`
- Create: `tests/src/core/solidDeformationModel/tricubicHermiteBasis_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add failing basis/evaluator tests**

  Create `tests/src/core/solidDeformationModel/tricubicHermiteBasis_gtest.cpp` with tests for 1D endpoint duality, 64 basis indexing, partition of unity for value DOFs, constant vector-field reproduction, and affine box Jacobian. Do not add a test asserting that derivative basis families sum to zero; that is not a valid Hermite identity on `[-1, 1]`.

  ```cpp
  #include <gtest/gtest.h>

  #include "tricubicHermiteBasis.h"
  #include "EigenSupport.h"

  #include <algorithm>
  #include <array>
  #include <cmath>

  namespace
  {
  namespace ES = pgo::EigenSupport;
  using pgo::SolidDeformationModel::TricubicHermiteBasis;

  TEST(TricubicHermiteBasisGTest, OneDimensionalEndpointDuality)
  {
    double value[4];
    double deriv[4];
    TricubicHermiteBasis::evaluate1D(-1.0, value, deriv);
    EXPECT_NEAR(value[0], 1.0, 1e-12);
    EXPECT_NEAR(deriv[1], 1.0, 1e-12);
    EXPECT_NEAR(value[2], 0.0, 1e-12);
    EXPECT_NEAR(deriv[3], 0.0, 1e-12);

    TricubicHermiteBasis::evaluate1D(1.0, value, deriv);
    EXPECT_NEAR(value[0], 0.0, 1e-12);
    EXPECT_NEAR(deriv[1], 0.0, 1e-12);
    EXPECT_NEAR(value[2], 1.0, 1e-12);
    EXPECT_NEAR(deriv[3], 1.0, 1e-12);
  }

  TEST(TricubicHermiteBasisGTest, FlatIndexRoundTrip)
  {
    for (int c = 0; c < 2; c++) {
      for (int b = 0; b < 2; b++) {
        for (int a = 0; a < 2; a++) {
          for (int type = 0; type < 8; type++) {
            const int idx = TricubicHermiteBasis::flatIndex(a, b, c, type);
            int aa = -1, bb = -1, cc = -1, tt = -1;
            TricubicHermiteBasis::unflattenIndex(idx, aa, bb, cc, tt);
            EXPECT_EQ(aa, a);
            EXPECT_EQ(bb, b);
            EXPECT_EQ(cc, c);
            EXPECT_EQ(tt, type);
          }
        }
      }
    }
  }

  TEST(TricubicHermiteBasisGTest, ValueBasisPartitionsUnity)
  {
    double B[64];
    double G[3 * 64];
    TricubicHermiteBasis::evaluate(0.125, -0.3, 0.75, B, G);

    double sumValueBasis = 0.0;
    for (int i = 0; i < 64; i += 8) {
      sumValueBasis += B[i];
    }
    EXPECT_NEAR(sumValueBasis, 1.0, 1e-12);
  }

  TEST(TricubicHermiteBasisGTest, ConstantVectorFieldHasConstantValueAndZeroJacobian)
  {
    double Q[64 * 3] = {};
    for (int c = 0; c < 2; c++) {
      for (int b = 0; b < 2; b++) {
        for (int a = 0; a < 2; a++) {
          const int valueIndex = TricubicHermiteBasis::flatIndex(a, b, c, 0);
          Q[valueIndex * 3 + 0] = 3.0;
          Q[valueIndex * 3 + 1] = -2.0;
          Q[valueIndex * 3 + 2] = 5.0;
        }
      }
    }

    ES::V3d value;
    ES::M3d J;
    TricubicHermiteBasis::evaluateVectorValue(0.33, -0.25, 0.61, Q, value.data());
    TricubicHermiteBasis::evaluateVectorJacobian(0.33, -0.25, 0.61, Q, J.data());
    EXPECT_NEAR((value - ES::V3d(3.0, -2.0, 5.0)).norm(), 0.0, 1e-12);
    EXPECT_NEAR(J.norm(), 0.0, 1e-12);
  }

  TEST(TricubicHermiteBasisGTest, AffineBoxJacobianMatchesHalfExtents)
  {
    double Q[64 * 3] = {};
    const double hx = 4.0;
    const double hy = 2.0;
    const double hz = 6.0;
    auto setQ = [&](int index, const ES::V3d &value) {
      Q[index * 3 + 0] = value[0];
      Q[index * 3 + 1] = value[1];
      Q[index * 3 + 2] = value[2];
    };

    for (int c = 0; c < 2; c++) {
      for (int b = 0; b < 2; b++) {
        for (int a = 0; a < 2; a++) {
          setQ(TricubicHermiteBasis::flatIndex(a, b, c, 0), ES::V3d(a * hx, b * hy, c * hz));
          setQ(TricubicHermiteBasis::flatIndex(a, b, c, 1), ES::V3d(hx * 0.5, 0.0, 0.0));
          setQ(TricubicHermiteBasis::flatIndex(a, b, c, 2), ES::V3d(0.0, hy * 0.5, 0.0));
          setQ(TricubicHermiteBasis::flatIndex(a, b, c, 3), ES::V3d(0.0, 0.0, hz * 0.5));
        }
      }
    }

    ES::M3d J;
    TricubicHermiteBasis::evaluateVectorJacobian(0.2, -0.4, 0.7, Q, J.data());
    EXPECT_NEAR((J - ES::V3d(hx * 0.5, hy * 0.5, hz * 0.5).asDiagonal()).norm(), 0.0, 1e-12);
  }
  }  // namespace
  ```

- [ ] **Step 2: Run the new test target and confirm it fails to compile**

  Run:

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteBasis_gtest -j 8
  ```

  Expected: compile failure because `tricubicHermiteBasis.h` does not exist.

- [ ] **Step 3: Add `TricubicHermiteBasis` API**

  Create `src/core/solidDeformationModel/tricubicHermiteBasis.h`:

  ```cpp
  #pragma once

  namespace pgo
  {
  namespace SolidDeformationModel
  {

  class TricubicHermiteBasis
  {
  public:
    static constexpr int kNumCorners = 8;
    static constexpr int kNumTypes = 8;
    static constexpr int kNumScalarBasis = 64;
    static constexpr int kNumVectorDofs = 192;

    static int flatIndex(int a, int b, int c, int type);
    static void unflattenIndex(int index, int &a, int &b, int &c, int &type);
    static void typeBits(int type, int &alpha, int &beta, int &gamma);

    static void evaluate1D(double s, double value[4], double deriv[4]);
    static void evaluate(double xi, double eta, double zeta, double basis[64], double gradRef[3 * 64]);
    static void evaluateVectorValue(double xi, double eta, double zeta, const double Q64x3[64 * 3], double out[3]);
    static void evaluateVectorJacobian(double xi, double eta, double zeta, const double Q64x3[64 * 3], double J[9]);
  };

  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 4: Implement the evaluator**

  Create `src/core/solidDeformationModel/tricubicHermiteBasis.cpp`:

  ```cpp
  #include "tricubicHermiteBasis.h"

  #include "EigenSupport.h"

  #include <stdexcept>

  namespace ES = pgo::EigenSupport;

  namespace pgo
  {
  namespace SolidDeformationModel
  {
  namespace
  {
  constexpr int kTypeBits[8][3] = {
    { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 },
    { 1, 1, 0 }, { 1, 0, 1 }, { 0, 1, 1 }, { 1, 1, 1 }
  };

  int basisSlot(int vertexBit, int derivativeBit)
  {
    return 2 * vertexBit + derivativeBit;
  }
  }  // namespace

  int TricubicHermiteBasis::flatIndex(int a, int b, int c, int type)
  {
    return ((a + 2 * b + 4 * c) * kNumTypes) + type;
  }

  void TricubicHermiteBasis::unflattenIndex(int index, int &a, int &b, int &c, int &type)
  {
    if (index < 0 || index >= kNumScalarBasis) {
      throw std::out_of_range("TricubicHermiteBasis index is outside [0, 64).");
    }
    const int vertex = index / kNumTypes;
    type = index % kNumTypes;
    a = vertex % 2;
    b = (vertex / 2) % 2;
    c = (vertex / 4) % 2;
  }

  void TricubicHermiteBasis::typeBits(int type, int &alpha, int &beta, int &gamma)
  {
    if (type < 0 || type >= kNumTypes) {
      throw std::out_of_range("TricubicHermiteBasis type is outside [0, 8).");
    }
    alpha = kTypeBits[type][0];
    beta = kTypeBits[type][1];
    gamma = kTypeBits[type][2];
  }

  void TricubicHermiteBasis::evaluate1D(double s, double value[4], double deriv[4])
  {
    value[0] = (s * s * s - 3.0 * s + 2.0) * 0.25;
    value[1] = (s * s * s - s * s - s + 1.0) * 0.25;
    value[2] = (-s * s * s + 3.0 * s + 2.0) * 0.25;
    value[3] = (s * s * s + s * s - s - 1.0) * 0.25;

    deriv[0] = (3.0 * s * s - 3.0) * 0.25;
    deriv[1] = (3.0 * s * s - 2.0 * s - 1.0) * 0.25;
    deriv[2] = (-3.0 * s * s + 3.0) * 0.25;
    deriv[3] = (3.0 * s * s + 2.0 * s - 1.0) * 0.25;
  }

  void TricubicHermiteBasis::evaluate(double xi, double eta, double zeta, double basis[64], double gradRef[3 * 64])
  {
    double px[4], py[4], pz[4];
    double dpx[4], dpy[4], dpz[4];
    evaluate1D(xi, px, dpx);
    evaluate1D(eta, py, dpy);
    evaluate1D(zeta, pz, dpz);

    for (int i = 0; i < kNumScalarBasis; i++) {
      int a, b, c, type;
      unflattenIndex(i, a, b, c, type);
      int alpha, beta, gamma;
      typeBits(type, alpha, beta, gamma);
      const int ix = basisSlot(a, alpha);
      const int iy = basisSlot(b, beta);
      const int iz = basisSlot(c, gamma);

      basis[i] = px[ix] * py[iy] * pz[iz];
      gradRef[i] = dpx[ix] * py[iy] * pz[iz];
      gradRef[64 + i] = px[ix] * dpy[iy] * pz[iz];
      gradRef[128 + i] = px[ix] * py[iy] * dpz[iz];
    }
  }

  void TricubicHermiteBasis::evaluateVectorValue(double xi, double eta, double zeta,
    const double Q64x3[64 * 3], double out[3])
  {
    double basis[64];
    double gradRef[3 * 64];
    evaluate(xi, eta, zeta, basis, gradRef);

    Eigen::Map<ES::V3d> outMap(out);
    outMap.setZero();
    for (int i = 0; i < kNumScalarBasis; i++) {
      outMap += basis[i] * Eigen::Map<const ES::V3d>(Q64x3 + i * 3);
    }
  }

  void TricubicHermiteBasis::evaluateVectorJacobian(double xi, double eta, double zeta,
    const double Q64x3[64 * 3], double J[9])
  {
    double basis[64];
    double gradRef[3 * 64];
    evaluate(xi, eta, zeta, basis, gradRef);

    Eigen::Map<ES::M3d> JMap(J);
    JMap.setZero();
    for (int i = 0; i < kNumScalarBasis; i++) {
      const ES::V3d q = Eigen::Map<const ES::V3d>(Q64x3 + i * 3);
      JMap.col(0) += q * gradRef[i];
      JMap.col(1) += q * gradRef[64 + i];
      JMap.col(2) += q * gradRef[128 + i];
    }
  }

  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 5: Register source and test targets**

  In `src/core/solidDeformationModel/CMakeLists.txt`, add:

  ```cmake
  tricubicHermiteBasis.h
  ```

  to `SOLID_DEFORMATION_MODEL_HEADERS`, and:

  ```cmake
  tricubicHermiteBasis.cpp
  ```

  to `SOLID_DEFORMATION_MODEL_SOURCES`.

  In `tests/src/core/solidDeformationModel/CMakeLists.txt`, add:

  ```cmake
  add_executable(tricubicHermiteBasis_gtest tricubicHermiteBasis_gtest.cpp)
  target_link_libraries(tricubicHermiteBasis_gtest PRIVATE GTest::gtest_main solidDeformationModel)
  set_property(TARGET tricubicHermiteBasis_gtest PROPERTY FOLDER "tests/gtest")
  gtest_discover_tests(tricubicHermiteBasis_gtest)
  ```

- [ ] **Step 6: Run basis tests**

  Run:

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteBasis_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteBasis_gtest
  ```

  Expected: all tests pass.

---

## Task 2: Add Hermite Geometry DOF Layout

**Files:**
- Create: `src/core/solidDeformationModel/tricubicHermiteDofLayout.h`
- Create: `src/core/solidDeformationModel/tricubicHermiteDofLayout.cpp`
- Create: `tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add layout tests**

  In `tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest.cpp`, add a first test that constructs one cubic element and verifies:

  ```text
  global geometry DOFs = numVertices * 24
  local geometry DOFs = 192
  local value DOFs map to the element's 8 vertex indices
  derivative DOFs are scaled by h/2 per active derivative direction
  mixed derivative DOFs multiply all active direction scales
  ```

  Test body:

  ```cpp
  #include <gtest/gtest.h>

  #include "tricubicHermiteDofLayout.h"
  #include "EigenSupport.h"

  #include <array>

  namespace
  {
  using pgo::SolidDeformationModel::TricubicHermiteDofLayout;

  TEST(TricubicHermiteDofLayoutGTest, SingleElementGatherScalesReferenceDerivatives)
  {
    const std::array<int, 8> elementVertices = { 0, 1, 2, 3, 4, 5, 6, 7 };
    const double hx = 4.0;
    const double hy = 2.0;
    const double hz = 6.0;
    TricubicHermiteDofLayout layout(8, 1, elementVertices.data(), hx, hy, hz);

    EXPECT_EQ(layout.getNumGlobalGeometryDofs(), 8 * 24);
    EXPECT_EQ(layout.getNumElementGeometryDofs(), 192);

    Eigen::VectorXd global = Eigen::VectorXd::Zero(layout.getNumGlobalGeometryDofs());
    const int node = 0;
    global[node * 24 + 0] = 1.0;
    global[node * 24 + 3] = 2.0;
    global[node * 24 + 6] = 3.0;
    global[node * 24 + 12] = 4.0;

    Eigen::VectorXd local = Eigen::VectorXd::Zero(layout.getNumElementGeometryDofs());
    layout.gatherElementGeometry(0, global.data(), local.data());
    EXPECT_NEAR(local[0], 1.0, 1e-12);
    EXPECT_NEAR(local[3], 2.0 * hx * 0.5, 1e-12);
    EXPECT_NEAR(local[6], 3.0 * hy * 0.5, 1e-12);
    EXPECT_NEAR(local[12], 4.0 * hx * hy * 0.25, 1e-12);
  }
  }  // namespace
  ```

- [ ] **Step 2: Add layout API**

  Create `src/core/solidDeformationModel/tricubicHermiteDofLayout.h`:

  ```cpp
  #pragma once

  #include <vector>

  namespace pgo
  {
  namespace SolidDeformationModel
  {

  class TricubicHermiteDofLayout
  {
  public:
    static constexpr int kTypesPerNode = 8;
    static constexpr int kCoords = 3;
    static constexpr int kDofsPerNode = 24;
    static constexpr int kNodesPerElement = 8;
    static constexpr int kElementGeometryDofs = 192;

    // Preconditions:
    // - all elements share the same axis-aligned physical sizes hx/hy/hz;
    // - elementNodeIndices are in Hermite basis order:
    //   {000, 100, 010, 110, 001, 101, 011, 111};
    // - global derivative DOFs are physical derivatives, and gather converts
    //   them to reference-coordinate derivatives by multiplying by h/2 in each active direction.
    TricubicHermiteDofLayout(int numNodes, int numElements, const int *elementNodeIndices,
      double hx, double hy, double hz);

    int getNumNodes() const;
    int getNumElements() const;
    int getNumGlobalGeometryDofs() const;
    int getNumElementGeometryDofs() const;
    const int *getElementNodeIndices(int ele) const;

    void gatherElementGeometry(int ele, const double *globalGeometryDofs, double *localGeometryDofs) const;
    void scatterAddElementGeometryGradient(int ele, const double *localGradient, double *globalGradient) const;

    double referenceDerivativeScale(int type) const;

  private:
    int numNodes_ = 0;
    int numElements_ = 0;
    std::vector<int> elementNodeIndices_;
    double hx_ = 1.0;
    double hy_ = 1.0;
    double hz_ = 1.0;
  };

  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement layout gather/scatter**

  Create `src/core/solidDeformationModel/tricubicHermiteDofLayout.cpp`:

  ```cpp
  #include "tricubicHermiteDofLayout.h"

  #include "tricubicHermiteBasis.h"

  #include <atomic>
  #include <cstring>
  #include <stdexcept>

  namespace pgo
  {
  namespace SolidDeformationModel
  {

  TricubicHermiteDofLayout::TricubicHermiteDofLayout(int numNodes, int numElements,
    const int *elementNodeIndices, double hx, double hy, double hz):
    numNodes_(numNodes), numElements_(numElements), hx_(hx), hy_(hy), hz_(hz)
  {
    elementNodeIndices_.assign(elementNodeIndices, elementNodeIndices + numElements * kNodesPerElement);
  }

  int TricubicHermiteDofLayout::getNumNodes() const { return numNodes_; }
  int TricubicHermiteDofLayout::getNumElements() const { return numElements_; }
  int TricubicHermiteDofLayout::getNumGlobalGeometryDofs() const { return numNodes_ * kDofsPerNode; }
  int TricubicHermiteDofLayout::getNumElementGeometryDofs() const { return kElementGeometryDofs; }

  const int *TricubicHermiteDofLayout::getElementNodeIndices(int ele) const
  {
    return elementNodeIndices_.data() + ele * kNodesPerElement;
  }

  double TricubicHermiteDofLayout::referenceDerivativeScale(int type) const
  {
    int alpha, beta, gamma;
    TricubicHermiteBasis::typeBits(type, alpha, beta, gamma);
    double scale = 1.0;
    if (alpha) scale *= hx_ * 0.5;
    if (beta) scale *= hy_ * 0.5;
    if (gamma) scale *= hz_ * 0.5;
    return scale;
  }

  void TricubicHermiteDofLayout::gatherElementGeometry(int ele, const double *globalGeometryDofs,
    double *localGeometryDofs) const
  {
    const int *nodes = getElementNodeIndices(ele);
    for (int corner = 0; corner < kNodesPerElement; corner++) {
      const int node = nodes[corner];
      for (int type = 0; type < kTypesPerNode; type++) {
        const double scale = referenceDerivativeScale(type);
        for (int coord = 0; coord < kCoords; coord++) {
          const int globalIndex = node * kDofsPerNode + type * kCoords + coord;
          const int localIndex = (corner * kTypesPerNode + type) * kCoords + coord;
          localGeometryDofs[localIndex] = scale * globalGeometryDofs[globalIndex];
        }
      }
    }
  }

  void TricubicHermiteDofLayout::scatterAddElementGeometryGradient(int ele, const double *localGradient,
    double *globalGradient) const
  {
    const int *nodes = getElementNodeIndices(ele);
    for (int corner = 0; corner < kNodesPerElement; corner++) {
      const int node = nodes[corner];
      for (int type = 0; type < kTypesPerNode; type++) {
        const double scale = referenceDerivativeScale(type);
        for (int coord = 0; coord < kCoords; coord++) {
          const int globalIndex = node * kDofsPerNode + type * kCoords + coord;
          const int localIndex = (corner * kTypesPerNode + type) * kCoords + coord;
          std::atomic_ref<double> ref(globalGradient[globalIndex]);
          ref.fetch_add(scale * localGradient[localIndex]);
        }
      }
    }
  }

  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 4: Register layout files and test target**

  Add `tricubicHermiteDofLayout.h/.cpp` to `src/core/solidDeformationModel/CMakeLists.txt`.

  Add to `tests/src/core/solidDeformationModel/CMakeLists.txt`:

  ```cmake
  add_executable(tricubicHermiteAssembler_gtest tricubicHermiteAssembler_gtest.cpp)
  target_link_libraries(tricubicHermiteAssembler_gtest PRIVATE GTest::gtest_main solidDeformationModel)
  set_property(TARGET tricubicHermiteAssembler_gtest PROPERTY FOLDER "tests/gtest")
  gtest_discover_tests(tricubicHermiteAssembler_gtest)
  ```

- [ ] **Step 5: Run layout tests**

  Run:

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteAssembler_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest
  ```

  Expected: layout tests pass.

---

## Task 3: Add Plastic Design Field Interface And Constant Field

**Files:**
- Create: `src/core/solidDeformationModel/plasticDesignField.h`
- Create: `src/core/solidDeformationModel/plasticDesignField.cpp`
- Create: `tests/src/core/solidDeformationModel/plasticDesignField_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add tests for constant field**

  Create `tests/src/core/solidDeformationModel/plasticDesignField_gtest.cpp`:

  ```cpp
  #include <gtest/gtest.h>

  #include "plasticDesignField.h"

  namespace
  {
  using pgo::SolidDeformationModel::ConstantPlasticDesignField;

  TEST(PlasticDesignFieldGTest, ConstantFieldCopiesOneDesignBlockEverywhere)
  {
    ConstantPlasticDesignField field(2, 8);
    EXPECT_EQ(field.getNumDesignParameters(), 6);

    const double z[6] = { 1.0, 0.1, 0.2, 1.1, 0.3, 0.9 };
    double a[6] = {};
    field.evaluateParameters(1, 7, z, a);
    for (int i = 0; i < 6; i++) {
      EXPECT_DOUBLE_EQ(a[i], z[i]);
    }
  }

  TEST(PlasticDesignFieldGTest, ConstantFieldMapsGradientBySummation)
  {
    ConstantPlasticDesignField field(2, 3);
    double globalGrad[6] = {};
    const double localGradA[6] = { 1, 2, 3, 4, 5, 6 };
    const double localGradB[6] = { 10, 20, 30, 40, 50, 60 };
    field.scatterAddParameterGradient(0, 0, localGradA, globalGrad);
    field.scatterAddParameterGradient(1, 2, localGradB, globalGrad);

    EXPECT_DOUBLE_EQ(globalGrad[0], 11);
    EXPECT_DOUBLE_EQ(globalGrad[1], 22);
    EXPECT_DOUBLE_EQ(globalGrad[2], 33);
    EXPECT_DOUBLE_EQ(globalGrad[3], 44);
    EXPECT_DOUBLE_EQ(globalGrad[4], 55);
    EXPECT_DOUBLE_EQ(globalGrad[5], 66);
  }
  }  // namespace
  ```

- [ ] **Step 2: Add `PlasticDesignField` API**

  Create `src/core/solidDeformationModel/plasticDesignField.h`:

  ```cpp
  #pragma once

  #include <vector>

  namespace pgo
  {
  namespace SolidDeformationModel
  {

  class PlasticDesignField
  {
  public:
    static constexpr int kParamsPerFp = 6;

    virtual ~PlasticDesignField() {}
    virtual int getNumElements() const = 0;
    virtual int getNumQuadraturePointsPerElement() const = 0;
    virtual int getNumDesignParameters() const = 0;

    virtual void evaluateParameters(int ele, int qp, const double *designParameters, double a[6]) const = 0;
    virtual void scatterAddParameterGradient(int ele, int qp, const double dE_da[6], double *dE_dz) const = 0;
  };

  class ConstantPlasticDesignField : public PlasticDesignField
  {
  public:
    ConstantPlasticDesignField(int numElements, int numQuadraturePointsPerElement);

    int getNumElements() const override;
    int getNumQuadraturePointsPerElement() const override;
    int getNumDesignParameters() const override;

    void evaluateParameters(int ele, int qp, const double *designParameters, double a[6]) const override;
    void scatterAddParameterGradient(int ele, int qp, const double dE_da[6], double *dE_dz) const override;

  private:
    int numElements_ = 0;
    int numQuadraturePointsPerElement_ = 0;
  };

  class SparseKernelPlasticDesignField : public PlasticDesignField
  {
  public:
    struct Influence
    {
      int controlPoint = -1;
      double weight = 0.0;
    };

    SparseKernelPlasticDesignField(int numElements, int numQuadraturePointsPerElement,
      int numControlPoints, std::vector<std::vector<Influence>> influences);

    // Contract for influences:
    // influences is a flat table with one row per element/quadrature-point pair.
    // influences.size() == numElements * numQuadraturePointsPerElement.
    // row index = ele * numQuadraturePointsPerElement + qp.

    int getNumElements() const override;
    int getNumQuadraturePointsPerElement() const override;
    int getNumDesignParameters() const override;

    void evaluateParameters(int ele, int qp, const double *designParameters, double a[6]) const override;
    void scatterAddParameterGradient(int ele, int qp, const double dE_da[6], double *dE_dz) const override;

  private:
    int flatQuadratureIndex(int ele, int qp) const;

    int numElements_ = 0;
    int numQuadraturePointsPerElement_ = 0;
    int numControlPoints_ = 0;
    std::vector<std::vector<Influence>> influences_;
  };

  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement constant field and sparse field core mapping**

  Create `src/core/solidDeformationModel/plasticDesignField.cpp`:

  ```cpp
  #include "plasticDesignField.h"

  #include <atomic>
  #include <cstring>
  #include <stdexcept>

  namespace pgo
  {
  namespace SolidDeformationModel
  {

  ConstantPlasticDesignField::ConstantPlasticDesignField(int numElements, int numQuadraturePointsPerElement):
    numElements_(numElements), numQuadraturePointsPerElement_(numQuadraturePointsPerElement)
  {
  }

  int ConstantPlasticDesignField::getNumElements() const { return numElements_; }
  int ConstantPlasticDesignField::getNumQuadraturePointsPerElement() const { return numQuadraturePointsPerElement_; }
  int ConstantPlasticDesignField::getNumDesignParameters() const { return kParamsPerFp; }

  void ConstantPlasticDesignField::evaluateParameters(int, int, const double *designParameters, double a[6]) const
  {
    std::memcpy(a, designParameters, sizeof(double) * kParamsPerFp);
  }

  void ConstantPlasticDesignField::scatterAddParameterGradient(int, int, const double dE_da[6], double *dE_dz) const
  {
    for (int i = 0; i < kParamsPerFp; i++) {
      std::atomic_ref<double> ref(dE_dz[i]);
      ref.fetch_add(dE_da[i]);
    }
  }

  SparseKernelPlasticDesignField::SparseKernelPlasticDesignField(int numElements, int numQuadraturePointsPerElement,
    int numControlPoints, std::vector<std::vector<Influence>> influences):
    numElements_(numElements),
    numQuadraturePointsPerElement_(numQuadraturePointsPerElement),
    numControlPoints_(numControlPoints),
    influences_(std::move(influences))
  {
    if ((int)influences_.size() != numElements_ * numQuadraturePointsPerElement_) {
      throw std::invalid_argument("SparseKernelPlasticDesignField influence table has wrong size.");
    }
  }

  int SparseKernelPlasticDesignField::getNumElements() const { return numElements_; }
  int SparseKernelPlasticDesignField::getNumQuadraturePointsPerElement() const { return numQuadraturePointsPerElement_; }
  int SparseKernelPlasticDesignField::getNumDesignParameters() const { return numControlPoints_ * kParamsPerFp; }

  int SparseKernelPlasticDesignField::flatQuadratureIndex(int ele, int qp) const
  {
    return ele * numQuadraturePointsPerElement_ + qp;
  }

  void SparseKernelPlasticDesignField::evaluateParameters(int ele, int qp, const double *designParameters, double a[6]) const
  {
    std::memset(a, 0, sizeof(double) * kParamsPerFp);
    for (const Influence &influence : influences_[flatQuadratureIndex(ele, qp)]) {
      const double *zk = designParameters + influence.controlPoint * kParamsPerFp;
      for (int i = 0; i < kParamsPerFp; i++) {
        a[i] += influence.weight * zk[i];
      }
    }
  }

  void SparseKernelPlasticDesignField::scatterAddParameterGradient(int ele, int qp, const double dE_da[6], double *dE_dz) const
  {
    for (const Influence &influence : influences_[flatQuadratureIndex(ele, qp)]) {
      double *gk = dE_dz + influence.controlPoint * kParamsPerFp;
      for (int i = 0; i < kParamsPerFp; i++) {
        std::atomic_ref<double> ref(gk[i]);
        ref.fetch_add(influence.weight * dE_da[i]);
      }
    }
  }

  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 4: Register plastic field files and tests**

  Add `plasticDesignField.h/.cpp` to `src/core/solidDeformationModel/CMakeLists.txt`.

  Add to `tests/src/core/solidDeformationModel/CMakeLists.txt`:

  ```cmake
  add_executable(plasticDesignField_gtest plasticDesignField_gtest.cpp)
  target_link_libraries(plasticDesignField_gtest PRIVATE GTest::gtest_main solidDeformationModel)
  set_property(TARGET plasticDesignField_gtest PROPERTY FOLDER "tests/gtest")
  gtest_discover_tests(plasticDesignField_gtest)
  ```

- [ ] **Step 5: Run plastic design field tests**

  Run:

  ```bash
  cmake --build build/base_no_mkl --target plasticDesignField_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/plasticDesignField_gtest
  ```

  Expected: all tests pass.

---

## Task 4: Add Tricubic Hermite Deformation Model With Constant `Fp` Sampling

**Files:**
- Create: `src/core/solidDeformationModel/tricubicHermiteDeformationModel.h`
- Create: `src/core/solidDeformationModel/tricubicHermiteDeformationModel.cpp`
- Create: `tests/src/core/solidDeformationModel/tricubicHermiteDeformationModel_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`
- Modify: `tests/src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add energy and finite-difference tests**

  Create `tests/src/core/solidDeformationModel/tricubicHermiteDeformationModel_gtest.cpp` with tests for:

  ```text
  rest energy is zero when x == X and Fp == I
  rigid translation energy is zero
  affine deformation energy matches volume * det(Fp) * psi(F * inv(Fp))
  quadrature order 4 and 5 agree on affine deformation
  dE/dx matches finite difference
  dE/dLocalPlasticParams matches finite difference
  d2E/dx2 is symmetric and Hessian-vector products match finite difference
  d2E/dxda matches finite difference of dE/dx with respect to local plastic parameters
  ```

  Use `ElasticModelStableNeoHookeanMaterial` and `PlasticModel3D6DOF`.

  The test fixture must include these three states:

  ```text
  State A: rest Hermite box, identity Fp
  State B: perturbed Hermite geometry, identity Fp
  State C: perturbed Hermite geometry, non-identity SPD Fp parameter [1.05, 0.02, -0.01, 0.96, 0.015, 1.02]
  ```

  These tests are the primary formula check. Do not relax them to make simulation run.

  The test file must include this concrete fixture. It fixes material parameters, rest/current Hermite DOF construction, the `prepareData` call sequence, and finite-difference step size:

  ```cpp
  #include <gtest/gtest.h>

  #include "tricubicHermiteBasis.h"
  #include "tricubicHermiteDeformationModel.h"

  #include "elasticModelStableNeoHookeanMaterial.h"
  #include "plasticModel3D6DOF.h"
  #include "EigenSupport.h"

  #include <algorithm>
  #include <array>
  #include <cmath>
  #include <functional>
  #include <memory>
  #include <vector>

  namespace
  {
  namespace ES = pgo::EigenSupport;
  using pgo::SolidDeformationModel::ElasticModelStableNeoHookeanMaterial;
  using pgo::SolidDeformationModel::PlasticModel3D6DOF;
  using pgo::SolidDeformationModel::TricubicHermiteBasis;
  using pgo::SolidDeformationModel::TricubicHermiteDeformationModel;

  constexpr int kDofs = 192;
  constexpr int kParamsPerFp = 6;
  constexpr double kMu = 1.0e4;
  constexpr double kLambda = 1.0e4;
  constexpr double kFdEps = 1.0e-6;

  void setVectorDof(std::array<double, kDofs> &q, int scalarIndex, const ES::V3d &value)
  {
    q[scalarIndex * 3 + 0] = value[0];
    q[scalarIndex * 3 + 1] = value[1];
    q[scalarIndex * 3 + 2] = value[2];
  }

  std::array<double, kDofs> makeAffineBoxHermiteDofs(double hx, double hy, double hz)
  {
    std::array<double, kDofs> q = {};
    for (int c = 0; c < 2; c++) {
      for (int b = 0; b < 2; b++) {
        for (int a = 0; a < 2; a++) {
          setVectorDof(q, TricubicHermiteBasis::flatIndex(a, b, c, 0), ES::V3d(a * hx, b * hy, c * hz));
          setVectorDof(q, TricubicHermiteBasis::flatIndex(a, b, c, 1), ES::V3d(0.5 * hx, 0.0, 0.0));
          setVectorDof(q, TricubicHermiteBasis::flatIndex(a, b, c, 2), ES::V3d(0.0, 0.5 * hy, 0.0));
          setVectorDof(q, TricubicHermiteBasis::flatIndex(a, b, c, 3), ES::V3d(0.0, 0.0, 0.5 * hz));
        }
      }
    }
    return q;
  }

  std::array<double, kDofs> transformHermiteDofs(const std::array<double, kDofs> &rest,
    const ES::M3d &A, const ES::V3d &translation)
  {
    std::array<double, kDofs> q = {};
    for (int i = 0; i < 64; i++) {
      int a = 0, b = 0, c = 0, type = 0;
      TricubicHermiteBasis::unflattenIndex(i, a, b, c, type);
      const ES::V3d source(rest[i * 3 + 0], rest[i * 3 + 1], rest[i * 3 + 2]);
      const ES::V3d value = A * source + (type == 0 ? translation : ES::V3d::Zero());
      setVectorDof(q, i, value);
    }
    return q;
  }

  std::array<double, kDofs> makePerturbedHermiteDofs(const std::array<double, kDofs> &rest)
  {
    std::array<double, kDofs> q = rest;
    for (int i = 0; i < kDofs; i++) {
      q[i] += 1.0e-3 * static_cast<double>((i % 11) - 5);
    }
    return q;
  }

  std::vector<double> makeRepeatedPlasticParams(const TricubicHermiteDeformationModel &model,
    const double param6[kParamsPerFp])
  {
    std::vector<double> params(model.getNumLocalPlasticParams(), 0.0);
    for (int q = 0; q < model.getNumQuadraturePoints(); q++) {
      std::copy(param6, param6 + kParamsPerFp, params.data() + q * kParamsPerFp);
    }
    return params;
  }

  void computeSignedSVD(const ES::M3d &F, ES::M3d &U, ES::M3d &V, ES::V3d &S)
  {
    Eigen::JacobiSVD<ES::M3d, Eigen::NoQRPreconditioner> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();
    if (U.determinant() < 0.0) {
      U.col(2) *= -1.0;
      S(2) *= -1.0;
    }
    if (V.determinant() < 0.0) {
      V.col(2) *= -1.0;
      S(2) *= -1.0;
    }
  }

  double computeEnergy(TricubicHermiteDeformationModel &model, const double *x, const double *localPlasticParams)
  {
    std::unique_ptr<pgo::SolidDeformationModel::DeformationModel::CacheData,
      std::function<void(pgo::SolidDeformationModel::DeformationModel::CacheData *)>>
      cache(model.allocateCacheData(), [&](pgo::SolidDeformationModel::DeformationModel::CacheData *ptr) { model.freeCacheData(ptr); });
    model.prepareData(x, localPlasticParams, nullptr, cache.get());
    return model.computeEnergy(cache.get());
  }

  ES::VXd computeGradient(TricubicHermiteDeformationModel &model, const double *x, const double *localPlasticParams)
  {
    std::unique_ptr<pgo::SolidDeformationModel::DeformationModel::CacheData,
      std::function<void(pgo::SolidDeformationModel::DeformationModel::CacheData *)>>
      cache(model.allocateCacheData(), [&](pgo::SolidDeformationModel::DeformationModel::CacheData *ptr) { model.freeCacheData(ptr); });
    model.prepareData(x, localPlasticParams, nullptr, cache.get());
    ES::VXd grad = ES::VXd::Zero(kDofs);
    model.compute_dE_dx(cache.get(), grad.data());
    return grad;
  }

  ES::VXd computeLocalPlasticGradient(TricubicHermiteDeformationModel &model,
    const double *x, const double *localPlasticParams)
  {
    std::unique_ptr<pgo::SolidDeformationModel::DeformationModel::CacheData,
      std::function<void(pgo::SolidDeformationModel::DeformationModel::CacheData *)>>
      cache(model.allocateCacheData(), [&](pgo::SolidDeformationModel::DeformationModel::CacheData *ptr) { model.freeCacheData(ptr); });
    model.prepareData(x, localPlasticParams, nullptr, cache.get());
    ES::VXd grad = ES::VXd::Zero(model.getNumLocalPlasticParams());
    model.compute_dE_dLocalPlasticParams(cache.get(), grad.data());
    return grad;
  }

  ES::MXd computeHessian(TricubicHermiteDeformationModel &model, const double *x, const double *localPlasticParams)
  {
    std::unique_ptr<pgo::SolidDeformationModel::DeformationModel::CacheData,
      std::function<void(pgo::SolidDeformationModel::DeformationModel::CacheData *)>>
      cache(model.allocateCacheData(), [&](pgo::SolidDeformationModel::DeformationModel::CacheData *ptr) { model.freeCacheData(ptr); });
    model.prepareData(x, localPlasticParams, nullptr, cache.get());
    ES::MXd hess = ES::MXd::Zero(kDofs, kDofs);
    model.compute_d2E_dx2(cache.get(), hess.data());
    return hess;
  }

  ES::MXd computeMixedDxDa(TricubicHermiteDeformationModel &model, const double *x, const double *localPlasticParams)
  {
    std::unique_ptr<pgo::SolidDeformationModel::DeformationModel::CacheData,
      std::function<void(pgo::SolidDeformationModel::DeformationModel::CacheData *)>>
      cache(model.allocateCacheData(), [&](pgo::SolidDeformationModel::DeformationModel::CacheData *ptr) { model.freeCacheData(ptr); });
    model.prepareData(x, localPlasticParams, nullptr, cache.get());
    ES::MXd mixed = ES::MXd::Zero(kDofs, model.getNumLocalPlasticParams());
    model.compute_d2E_dxda(cache.get(), mixed.data());
    return mixed;
  }

  double analyticAffineEnergy(const ES::M3d &A, const double plasticParam[6], double volume)
  {
    ElasticModelStableNeoHookeanMaterial elastic(kMu, kLambda);
    PlasticModel3D6DOF plastic;
    ES::M3d FpInv;
    plastic.computeAInv(plasticParam, FpInv.data());
    const double detFp = plastic.compute_detA(plasticParam);
    const ES::M3d Fe = A * FpInv;
    ES::M3d U, V;
    ES::V3d S;
    computeSignedSVD(Fe, U, V, S);
    return volume * detFp * elastic.compute_psi(nullptr, Fe.data(), U.data(), V.data(), S.data());
  }

  double maxRelativeError(const ES::VXd &a, const ES::VXd &b)
  {
    double err = 0.0;
    for (int i = 0; i < a.size(); i++) {
      err = std::max(err, std::abs(a[i] - b[i]) / std::max(1.0e-9, std::abs(b[i])));
    }
    return err;
  }
  }  // namespace

  TEST(TricubicHermiteDeformationModelGTest, RestAndTranslationHaveZeroEnergy)
  {
    const auto rest = makeAffineBoxHermiteDofs(4.0, 2.0, 6.0);
    ElasticModelStableNeoHookeanMaterial elastic(kMu, kLambda);
    PlasticModel3D6DOF plastic;
    TricubicHermiteDeformationModel model(rest.data(), &elastic, &plastic, 4);
    const double identityPlastic[6] = { 1, 0, 0, 1, 0, 1 };
    const std::vector<double> localPlastic = makeRepeatedPlasticParams(model, identityPlastic);

    EXPECT_NEAR(computeEnergy(model, rest.data(), localPlastic.data()), 0.0, 1.0e-9);
    const std::array<double, kDofs> translated =
      transformHermiteDofs(rest, ES::M3d::Identity(), ES::V3d(0.7, -0.2, 1.1));
    EXPECT_NEAR(computeEnergy(model, translated.data(), localPlastic.data()), 0.0, 1.0e-9);
    EXPECT_NEAR(computeGradient(model, translated.data(), localPlastic.data()).norm(), 0.0, 1.0e-7);
  }

  TEST(TricubicHermiteDeformationModelGTest, AffineEnergyMatchesAnalyticFormula)
  {
    const double hx = 4.0, hy = 2.0, hz = 6.0;
    const auto rest = makeAffineBoxHermiteDofs(hx, hy, hz);
    ES::M3d A;
    A << 1.08, 0.03, -0.02,
      0.01, 0.94, 0.04,
      -0.02, 0.01, 1.05;
    const auto current = transformHermiteDofs(rest, A, ES::V3d::Zero());
    const double plasticParam[6] = { 1.05, 0.02, -0.01, 0.96, 0.015, 1.02 };

    ElasticModelStableNeoHookeanMaterial elastic(kMu, kLambda);
    PlasticModel3D6DOF plastic;
    TricubicHermiteDeformationModel model4(rest.data(), &elastic, &plastic, 4);
    TricubicHermiteDeformationModel model5(rest.data(), &elastic, &plastic, 5);
    const std::vector<double> localPlastic4 = makeRepeatedPlasticParams(model4, plasticParam);
    const std::vector<double> localPlastic5 = makeRepeatedPlasticParams(model5, plasticParam);

    const double expected = analyticAffineEnergy(A, plasticParam, hx * hy * hz);
    const double energy4 = computeEnergy(model4, current.data(), localPlastic4.data());
    const double energy5 = computeEnergy(model5, current.data(), localPlastic5.data());
    EXPECT_NEAR(energy4, expected, 1.0e-8 * std::max(1.0, std::abs(expected)));
    EXPECT_NEAR(energy5, expected, 1.0e-8 * std::max(1.0, std::abs(expected)));
    EXPECT_NEAR(energy4 - energy5, 0.0, 1.0e-10);
  }

  TEST(TricubicHermiteDeformationModelGTest, PositionAndLocalPlasticGradientsMatchFiniteDifferences)
  {
    const auto rest = makeAffineBoxHermiteDofs(4.0, 2.0, 6.0);
    std::array<double, kDofs> x = makePerturbedHermiteDofs(rest);
    ElasticModelStableNeoHookeanMaterial elastic(kMu, kLambda);
    PlasticModel3D6DOF plastic;
    TricubicHermiteDeformationModel model(rest.data(), &elastic, &plastic, 4);
    const double plasticParam[6] = { 1.05, 0.02, -0.01, 0.96, 0.015, 1.02 };
    std::vector<double> localPlastic = makeRepeatedPlasticParams(model, plasticParam);

    const ES::VXd analyticX = computeGradient(model, x.data(), localPlastic.data());
    ES::VXd fdX = ES::VXd::Zero(kDofs);
    for (int i = 0; i < kDofs; i++) {
      std::array<double, kDofs> xp = x;
      std::array<double, kDofs> xm = x;
      xp[i] += kFdEps;
      xm[i] -= kFdEps;
      fdX[i] = (computeEnergy(model, xp.data(), localPlastic.data()) -
        computeEnergy(model, xm.data(), localPlastic.data())) / (2.0 * kFdEps);
    }
    EXPECT_LT(maxRelativeError(analyticX, fdX), 1.0e-5);

    const ES::VXd analyticA = computeLocalPlasticGradient(model, x.data(), localPlastic.data());
    ES::VXd fdA = ES::VXd::Zero(model.getNumLocalPlasticParams());
    for (int i = 0; i < model.getNumLocalPlasticParams(); i++) {
      std::vector<double> ap = localPlastic;
      std::vector<double> am = localPlastic;
      ap[i] += kFdEps;
      am[i] -= kFdEps;
      fdA[i] = (computeEnergy(model, x.data(), ap.data()) -
        computeEnergy(model, x.data(), am.data())) / (2.0 * kFdEps);
    }
    EXPECT_LT(maxRelativeError(analyticA, fdA), 1.0e-5);
  }

  TEST(TricubicHermiteDeformationModelGTest, HessianAndMixedBlocksMatchFiniteDifferences)
  {
    const auto rest = makeAffineBoxHermiteDofs(4.0, 2.0, 6.0);
    std::array<double, kDofs> x = makePerturbedHermiteDofs(rest);
    ElasticModelStableNeoHookeanMaterial elastic(kMu, kLambda);
    PlasticModel3D6DOF plastic;
    TricubicHermiteDeformationModel model(rest.data(), &elastic, &plastic, 4);
    const double plasticParam[6] = { 1.05, 0.02, -0.01, 0.96, 0.015, 1.02 };
    std::vector<double> localPlastic = makeRepeatedPlasticParams(model, plasticParam);

    const ES::MXd H = computeHessian(model, x.data(), localPlastic.data());
    EXPECT_NEAR((H - H.transpose()).norm(), 0.0, 1.0e-8 * std::max(1.0, H.norm()));

    ES::VXd direction = ES::VXd::Zero(kDofs);
    for (int i = 0; i < kDofs; i++) {
      direction[i] = 0.01 * static_cast<double>((i % 7) - 3);
    }
    std::array<double, kDofs> xp = x;
    std::array<double, kDofs> xm = x;
    for (int i = 0; i < kDofs; i++) {
      xp[i] += kFdEps * direction[i];
      xm[i] -= kFdEps * direction[i];
    }
    const ES::VXd fdHv = (computeGradient(model, xp.data(), localPlastic.data()) -
      computeGradient(model, xm.data(), localPlastic.data())) / (2.0 * kFdEps);
    EXPECT_LT(maxRelativeError(H * direction, fdHv), 1.0e-4);

    const ES::MXd mixed = computeMixedDxDa(model, x.data(), localPlastic.data());
    const int checkedColumns[] = { 0, 5, 6, model.getNumLocalPlasticParams() - 1 };
    for (int col : checkedColumns) {
      std::vector<double> ap = localPlastic;
      std::vector<double> am = localPlastic;
      ap[col] += kFdEps;
      am[col] -= kFdEps;
      const ES::VXd fdCol = (computeGradient(model, x.data(), ap.data()) -
        computeGradient(model, x.data(), am.data())) / (2.0 * kFdEps);
      EXPECT_LT(maxRelativeError(mixed.col(col), fdCol), 1.0e-4);
    }
  }
  ```

- [ ] **Step 2: Add model API**

  Create `src/core/solidDeformationModel/tricubicHermiteDeformationModel.h`:

  ```cpp
  #pragma once

  #include "deformationModel.h"

  namespace pgo
  {
  namespace SolidDeformationModel
  {

  class TricubicHermiteDeformationModelInternal;

  class TricubicHermiteDeformationModel : public DeformationModel
  {
  public:
    TricubicHermiteDeformationModel(const double restHermiteDofs[192],
      ElasticModel *elasticModel, PlasticModel *plasticModel, int quadratureOrder = 4);
    ~TricubicHermiteDeformationModel() override;

    CacheData *allocateCacheData() const override;
    void freeCacheData(CacheData *data) const override;
    void prepareData(const double *x, const double *param, const double *materialParam, CacheData *cacheData) const override;

    void enableSPD(int enable) override;
    double computeEnergy(const CacheData *cacheData) const override;
    void compute_dE_dx(const CacheData *cacheData, double *grad) const override;
    void compute_d2E_dx2(const CacheData *cacheData, double *hess) const override;
    void compute_d2E_dxda(const CacheData *cacheData, double *hess) const override;
    void compute_d2E_dxdb(const CacheData *cacheData, double *hess) const override;

    void compute_dE_dLocalPlasticParams(const CacheData *cacheData, double *grad) const;

    int getNumVertices() const override { return 8; }
    int getNumDOFs() const override { return 192; }
    int getNumMaterialLocations() const override;
    int getNumQuadraturePoints() const;
    int getNumLocalPlasticParams() const;

  private:
    TricubicHermiteDeformationModelInternal *data_;
  };

  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement geometry cache and energy**

  Implement `tricubicHermiteDeformationModel.cpp` by following the structure of `cubicMeshDeformationModel.cpp`, with these differences:

  ```text
  local geometry matrix is 64 x 3
  dFdx is 9 x 192 per quadrature point
  quadrature order starts at 4x4x4
  param passed to prepareData is a local plastic parameter block of size numQuadraturePoints * 6
  local plastic block layout is [a_q0(6), a_q1(6), ..., a_qN(6)]
  ```

  This is a deliberate deviation from `CubicMeshDeformationModel`, where `prepareData` receives one 6D plastic parameter block shared by all quadrature points. `TricubicHermiteAssembler` must allocate the local plastic buffer with `model.getNumLocalPlasticParams()`, not with `plasticModel->getNumParameters()`.

  The constructor must downcast and validate both model families:

  ```cpp
  ind->elasticModel = dynamic_cast<const ElasticModel3DDeformationGradient *>(elasticModel);
  ind->plasticModel = dynamic_cast<const PlasticModel3DDeformationGradient *>(plasticModel);
  if (ind->elasticModel == nullptr) {
    throw std::invalid_argument("TricubicHermiteDeformationModel requires ElasticModel3DDeformationGradient.");
  }
  if (ind->plasticModel == nullptr) {
    throw std::invalid_argument("TricubicHermiteDeformationModel requires PlasticModel3DDeformationGradient.");
  }
  ```

  Required internal data members:

  ```cpp
  struct QuadraturePointData
  {
    double xi = 0.0;
    double eta = 0.0;
    double zeta = 0.0;
    double weight = 0.0;
    double basis[64] = {};
    double gradRef[3 * 64] = {};
    pgo::EigenSupport::M3d JX = pgo::EigenSupport::M3d::Identity();
    pgo::EigenSupport::M3d JXInv = pgo::EigenSupport::M3d::Identity();
    double detJX = 1.0;
    Eigen::Matrix<double, 9, 192> dFdx;
  };
  ```

  `QuadraturePointData` is rest-geometry/internal data only. Per-evaluation quantities belong in the cache returned by `allocateCacheData`, not in the internal struct. Add a cache class with arrays sized by `getNumQuadraturePoints()`:

  ```cpp
  class TricubicHermiteDeformationModelCacheData : public DeformationModelCacheData
  {
  public:
    Eigen::Matrix<double, 64, 3, Eigen::RowMajor> x64x3;
    std::vector<pgo::EigenSupport::M3d> Jx;
    std::vector<pgo::EigenSupport::M3d> Fref;
    std::vector<pgo::EigenSupport::M3d> Fp;
    std::vector<pgo::EigenSupport::M3d> FpInv;
    std::vector<double> detFp;
    std::vector<pgo::EigenSupport::VXd> ddetFp_da;
    std::vector<std::array<pgo::EigenSupport::M3d, 6>> dFpInv_dai;
    std::vector<pgo::EigenSupport::M3d> Fe;
    std::vector<pgo::EigenSupport::M3d> U;
    std::vector<pgo::EigenSupport::M3d> V;
    std::vector<pgo::EigenSupport::V3d> S;
    std::vector<Eigen::Matrix<double, 9, 192>> dFe_dx;
  };
  ```

  Do not add `d2detFp_da2` or `d2FpInv_daidaj` in this plan. Those arrays are only needed for a future `d2E/da2` implementation and should not be allocated until that API is reintroduced.

  `allocateCacheData()` must resize every per-quadrature cache vector explicitly:

  ```cpp
  DeformationModel::CacheData *TricubicHermiteDeformationModel::allocateCacheData() const
  {
    auto *cache = new TricubicHermiteDeformationModelCacheData;
    const int numQP = getNumQuadraturePoints();

    cache->x64x3.setZero();
    cache->Jx.assign(numQP, ES::M3d::Zero());
    cache->Fref.assign(numQP, ES::M3d::Zero());
    cache->Fp.assign(numQP, ES::M3d::Identity());
    cache->FpInv.assign(numQP, ES::M3d::Identity());
    cache->detFp.assign(numQP, 1.0);
    cache->ddetFp_da.assign(numQP, ES::VXd::Zero(6));
    cache->dFpInv_dai.assign(numQP, std::array<ES::M3d, 6>{});
    cache->Fe.assign(numQP, ES::M3d::Identity());
    cache->U.assign(numQP, ES::M3d::Identity());
    cache->V.assign(numQP, ES::M3d::Identity());
    cache->S.assign(numQP, ES::V3d::Ones());
    cache->dFe_dx.assign(numQP, Eigen::Matrix<double, 9, 192>::Zero());

    for (auto &perQpDerivatives : cache->dFpInv_dai) {
      for (ES::M3d &derivative : perQpDerivatives) {
        derivative.setZero();
      }
    }

    return cache;
  }
  ```

  `freeCacheData()` must delete the exact cache type:

  ```cpp
  void TricubicHermiteDeformationModel::freeCacheData(DeformationModel::CacheData *data) const
  {
    delete static_cast<TricubicHermiteDeformationModelCacheData *>(data);
  }
  ```

  `compute_dE_dLocalPlasticParams(cache, grad)` writes `getNumLocalPlasticParams()` values. The layout is:

  ```text
  grad[q * 6 + i] = dE / d a_q_i
  ```

  `compute_d2E_dxda(cache, hess)` writes a `192 x getNumLocalPlasticParams()` column-major matrix where column `q * 6 + i` is `d/d a_q_i (dE/dx)`.

  Required formula at quadrature point `q`:

  ```text
  Jx = x64x3^T * gradRef^T
  Fref = Jx * JXInv
  Fp = plasticModel.computeA(a_q)
  FpInv = plasticModel.computeAInv(a_q)
  Fe = Fref * FpInv
  energy += weight * detJX * detFp * psi(Fe)
  ```

  For geometry gradient:

  ```text
  P_ref = P_e * FpInv^T
  dE_dx += (weight * detJX * detFp) * dFref_dx^T * vec(P_ref)
  ```

  For plastic gradient, port the same terms used by `CubicMeshDeformationModel::compute_dE_da`:

  ```text
  dE/da_i = weight * detJX * ddetFp/da_i * psi
          + weight * detJX * detFp * P_e : dFe/da_i
  dFe/da_i = Fref * dFpInv/da_i
  ```

- [ ] **Step 4: Implement geometry Hessian and mixed geometry/plastic Hessian**

  Port the matrix-chain style from `CubicMeshDeformationModel`:

  ```text
  dFe/dx = dFref/dx * FpInv
  Hxx += vol * dFe_dx^T * dP_dFe * dFe_dx
  Hxz_{q,i} += dV/da_{q,i} * dpsi_dx
             + vol * dFe_dx^T * dP_dFe * vec(dFe/da_{q,i})
             + vol * (d2Fe/dx da_{q,i})^T * vec(P)
  ```

  Here:

  ```text
  vol = weight_q * detJX_q * detFp_q
  dV/da_{q,i} = weight_q * detJX_q * ddetFp/da_i
  dFe/da_{q,i} = Fref_q * dFpInv/da_i
  d2Fe/dx_j da_{q,i} = dFref/dx_j * dFpInv/da_i
  ```

  This is the same mixed derivative structure as `CubicMeshDeformationModelInternal::compute_d2Fe_dx_dai`, but with one independent 6D plastic block per quadrature point.

  The code should keep helper functions local to the `.cpp` file:

  ```cpp
  static void compute_dFref_dx(const double gradRef[3 * 64],
    const pgo::EigenSupport::M3d &JXInv, Eigen::Matrix<double, 9, 192> &dFdx);

  static void computeCurrent_dFdx(const Eigen::Matrix<double, 9, 192> &rest_dFdx,
    const pgo::EigenSupport::M3d &FpInv, Eigen::Matrix<double, 9, 192> &dFdx);

  static void compute_d2Fe_dx_dai(const pgo::EigenSupport::M3d &dFpInvDai,
    const Eigen::Matrix<double, 9, 192> &dFrefDx, Eigen::Matrix<double, 9, 192> &d2FeDxDai);
  ```

  `compute_d2E_dxdb` is inherited from `DeformationModel`; in this Hermite plan, `b` means elastic material parameters. The initial tests use elastic models with zero elastic parameters, so this method may return immediately when `elasticModel->getNumParameters() == 0`. Do not add untested nonzero elastic-parameter support in this plan.

- [ ] **Step 5: Register model and tests**

  Add `tricubicHermiteDeformationModel.h/.cpp` to `src/core/solidDeformationModel/CMakeLists.txt`.

  Add to `tests/src/core/solidDeformationModel/CMakeLists.txt`:

  ```cmake
  add_executable(tricubicHermiteDeformationModel_gtest tricubicHermiteDeformationModel_gtest.cpp)
  target_link_libraries(tricubicHermiteDeformationModel_gtest PRIVATE GTest::gtest_main solidDeformationModel)
  set_property(TARGET tricubicHermiteDeformationModel_gtest PROPERTY FOLDER "tests/gtest")
  gtest_discover_tests(tricubicHermiteDeformationModel_gtest)
  ```

- [ ] **Step 6: Run deformation model tests**

  Run:

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteDeformationModel_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteDeformationModel_gtest
  ```

  Expected: energy invariance tests pass, affine analytic energy matches, quadrature order 4/5 affine comparison passes, Hessian symmetry is below `1e-8`, and finite-difference derivative errors are within `1e-5` relative tolerance for gradients and `1e-4` relative tolerance for Hessian-vector comparisons.

---

## Task 5: Add Hermite Assembler For Geometry And Plastic Design Variables

**Files:**
- Create: `src/core/solidDeformationModel/tricubicHermiteAssembler.h`
- Create: `src/core/solidDeformationModel/tricubicHermiteAssembler.cpp`
- Modify: `tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Extend assembler tests**

  Add tests to `tricubicHermiteAssembler_gtest.cpp`:

  ```text
  one element assembler energy equals direct model energy
  two element assembler energy equals sum of direct model energies
  assembler geometry gradient equals finite difference
  assembler plastic design gradient equals finite difference for ConstantPlasticDesignField
  assembler plastic design gradient equals finite difference for SparseKernelPlasticDesignField
  ```

  These tests must run without `runSim`. They are finite-element assembly checks and must pass before any simulation-driver integration is attempted.
  Geometry Hessian assembly is out of scope for this FEM plan and is introduced in `plan/tricubic_hermite_plastic_field_simulation_integration.plan.md`.

- [ ] **Step 2: Add assembler API**

  Create `src/core/solidDeformationModel/tricubicHermiteAssembler.h`:

  ```cpp
  #pragma once

  #include <memory>
  #include <vector>

  namespace pgo
  {
  namespace SolidDeformationModel
  {

  class PlasticDesignField;
  class TricubicHermiteDeformationModel;
  class TricubicHermiteDofLayout;

  class TricubicHermiteAssembler
  {
  public:
    TricubicHermiteAssembler(std::shared_ptr<const TricubicHermiteDofLayout> layout,
      std::shared_ptr<const PlasticDesignField> plasticField,
      std::vector<std::shared_ptr<const TricubicHermiteDeformationModel>> elementModels);

    int getNumGeometryDofs() const;
    int getNumPlasticDesignDofs() const;

    double computeEnergy(const double *geometryDofs, const double *plasticDesignDofs,
      const double *elasticParams) const;

    void computeGeometryGradient(const double *geometryDofs, const double *plasticDesignDofs,
      const double *elasticParams, double *gradient) const;

    void computePlasticDesignGradient(const double *geometryDofs, const double *plasticDesignDofs,
      const double *elasticParams, double *gradient) const;

  private:
    std::shared_ptr<const TricubicHermiteDofLayout> layout_;
    std::shared_ptr<const PlasticDesignField> plasticField_;
    std::vector<std::shared_ptr<const TricubicHermiteDeformationModel>> elementModels_;
  };

  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement assembler energy and gradients**

  In `tricubicHermiteAssembler.cpp`, implement:

  ```text
  gather local x through TricubicHermiteDofLayout
  assert plasticField->getNumQuadraturePointsPerElement() == model.getNumQuadraturePoints()
  allocate localA with model.getNumLocalPlasticParams()
  evaluate plastic field parameters per quadrature point into localA[q * 6 ... q * 6 + 5]
  call model.prepareData(localX, localA, elasticParams, cache)
  scatter local dE/dx through layout
  call model.compute_dE_dLocalPlasticParams(cache, localPlasticGradient)
  scatter each localPlasticGradient[q * 6 ... q * 6 + 5] through PlasticDesignField
  ```

  For the constant field, the local plastic parameter block passed into `TricubicHermiteDeformationModel::prepareData` is:

  ```text
  [a_q0, a_q1, ..., a_qN]
  size = numQuadraturePoints * 6
  ```

  This makes the model independent of the global design representation.

  The assembler constructor must reject mismatched dimensions before any evaluation:

  ```cpp
  if (elementModels_.size() != static_cast<std::size_t>(layout_->getNumElements())) {
    throw std::invalid_argument("TricubicHermiteAssembler element model count does not match layout.");
  }
  if (plasticField_->getNumElements() != layout_->getNumElements()) {
    throw std::invalid_argument("TricubicHermiteAssembler plastic field element count does not match layout.");
  }
  for (const auto &model : elementModels_) {
    if (!model) {
      throw std::invalid_argument("TricubicHermiteAssembler element model must not be null.");
    }
    if (model->getNumQuadraturePoints() != plasticField_->getNumQuadraturePointsPerElement()) {
      throw std::invalid_argument("TricubicHermiteAssembler model quadrature count does not match plastic field.");
    }
  }
  ```

- [ ] **Step 4: Register assembler files**

  Add `tricubicHermiteAssembler.h/.cpp` to `src/core/solidDeformationModel/CMakeLists.txt`.

- [ ] **Step 5: Run assembler tests**

  Run:

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteAssembler_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest
  ```

  Expected: assembler and direct model energy match to `1e-10`; constant-field and sparse-field finite-difference gradients match within `1e-5` relative tolerance.

---

## Task 6: Add Sparse Control Points + Kernel Interpolation

**Files:**
- Modify: `src/core/solidDeformationModel/plasticDesignField.h`
- Modify: `src/core/solidDeformationModel/plasticDesignField.cpp`
- Modify: `tests/src/core/solidDeformationModel/plasticDesignField_gtest.cpp`
- Modify: `tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest.cpp`

- [ ] **Step 1: Add sparse field weight tests**

  Add tests covering:

  ```text
  weights at each quadrature point sum to 1
  evaluateParameters returns weighted control parameters
  scatterAddParameterGradient applies transposed weights
  ```

  Example test:

  ```cpp
  TEST(PlasticDesignFieldGTest, SparseKernelFieldAppliesWeightsAndTranspose)
  {
    using Field = pgo::SolidDeformationModel::SparseKernelPlasticDesignField;
    std::vector<std::vector<Field::Influence>> influences(1);
    influences[0] = { { 0, 0.25 }, { 1, 0.75 } };
    Field field(1, 1, 2, influences);

    const double z[12] = { 1, 2, 3, 4, 5, 6, 10, 20, 30, 40, 50, 60 };
    double a[6] = {};
    field.evaluateParameters(0, 0, z, a);
    EXPECT_NEAR(a[0], 7.75, 1e-12);
    EXPECT_NEAR(a[5], 46.5, 1e-12);

    double gradZ[12] = {};
    const double gradA[6] = { 2, 4, 6, 8, 10, 12 };
    field.scatterAddParameterGradient(0, 0, gradA, gradZ);
    EXPECT_NEAR(gradZ[0], 0.5, 1e-12);
    EXPECT_NEAR(gradZ[6], 1.5, 1e-12);
  }

  TEST(PlasticDesignFieldGTest, SparseKernelInfluenceRowsAreNormalized)
  {
    using pgo::SolidDeformationModel::PlasticControlPoint;
    using pgo::SolidDeformationModel::buildSparseKernelInfluences;

    const double quadraturePositions[6] = {
      0.25, 0.0, 0.0,
      0.50, 0.0, 0.0,
    };
    PlasticControlPoint controlPoints[3];
    controlPoints[0].position[0] = 0.0;
    controlPoints[1].position[0] = 0.5;
    controlPoints[2].position[0] = 1.0;

    const auto influences = buildSparseKernelInfluences(
      quadraturePositions, 2, controlPoints, 3, 1.0);

    ASSERT_EQ(influences.size(), 2u);
    for (const auto &row : influences) {
      ASSERT_FALSE(row.empty());
      double sum = 0.0;
      for (const auto &influence : row) {
        sum += influence.weight;
      }
      EXPECT_NEAR(sum, 1.0, 1e-12);
    }
  }

  TEST(PlasticDesignFieldGTest, SparseKernelBuilderFallsBackToNearestControlPoint)
  {
    using pgo::SolidDeformationModel::PlasticControlPoint;
    using pgo::SolidDeformationModel::buildSparseKernelInfluences;

    const double quadraturePositions[3] = { 10.0, 0.0, 0.0 };
    PlasticControlPoint controlPoints[2];
    controlPoints[0].position[0] = 0.0;
    controlPoints[1].position[0] = 8.0;

    const auto influences = buildSparseKernelInfluences(
      quadraturePositions, 1, controlPoints, 2, 0.25);

    ASSERT_EQ(influences.size(), 1u);
    ASSERT_EQ(influences[0].size(), 1u);
    EXPECT_EQ(influences[0][0].controlPoint, 1);
    EXPECT_NEAR(influences[0][0].weight, 1.0, 1e-12);
  }
  ```

- [ ] **Step 2: Add helper for building Wendland kernel weights**

  Extend `plasticDesignField.h` with:

  ```cpp
  struct PlasticControlPoint
  {
    double position[3] = {};
  };

  std::vector<std::vector<SparseKernelPlasticDesignField::Influence>> buildSparseKernelInfluences(
    const double *quadraturePositions, int numQuadraturePoints,
    const PlasticControlPoint *controlPoints, int numControlPoints, double radius);
  ```

  Contract:

  ```text
  quadraturePositions points to 3 * numQuadraturePoints doubles
  numQuadraturePoints is the total number of quadrature rows to build, not the per-element count
  return.size() == numQuadraturePoints
  return[row] contains normalized control-point influences for quadrature row `row`
  ```

  Implement Wendland C2 weight:

  ```text
  phi(r) = (1 - r)^4 * (4r + 1), 0 <= r < 1
  phi(r) = 0, r >= 1
  w_k = phi_k / sum_j phi_j
  ```

  If a quadrature point has no control point inside `radius`, assign weight `1.0` to the nearest control point.

- [ ] **Step 3: Add assembler finite-difference test for sparse field**

  Add a two-control-point single-element test:

  ```text
  z0 = identity-like parameter
  z1 = perturbed SPD-like parameter
  quadrature weights are nontrivial
  dE/dz from assembler matches finite difference
  ```

  Add this concrete test to `tricubicHermiteAssembler_gtest.cpp`:

  ```cpp
  #include "tricubicHermiteAssembler.h"
  #include "tricubicHermiteDeformationModel.h"
  #include "tricubicHermiteDofLayout.h"
  #include "plasticDesignField.h"
  #include "elasticModelStableNeoHookeanMaterial.h"
  #include "plasticModel3D6DOF.h"
  #include "EigenSupport.h"

  #include <algorithm>
  #include <array>
  #include <cmath>
  #include <memory>
  #include <vector>

  namespace
  {
  namespace ES = pgo::EigenSupport;

  constexpr double kSparseAssemblerFdEps = 1.0e-6;

  void setSparseAssemblerDof(std::vector<double> &q, int node, int type, const ES::V3d &value)
  {
    const int base = node * 24 + type * 3;
    q[base + 0] = value[0];
    q[base + 1] = value[1];
    q[base + 2] = value[2];
  }

  std::vector<double> makeSparseAssemblerRestGlobalDofs(double hx, double hy, double hz)
  {
    std::vector<double> q(8 * 24, 0.0);
    for (int c = 0; c < 2; c++) {
      for (int b = 0; b < 2; b++) {
        for (int a = 0; a < 2; a++) {
          const int node = a + 2 * b + 4 * c;
          setSparseAssemblerDof(q, node, 0, ES::V3d(a * hx, b * hy, c * hz));
          setSparseAssemblerDof(q, node, 1, ES::V3d(1.0, 0.0, 0.0));
          setSparseAssemblerDof(q, node, 2, ES::V3d(0.0, 1.0, 0.0));
          setSparseAssemblerDof(q, node, 3, ES::V3d(0.0, 0.0, 1.0));
        }
      }
    }
    return q;
  }

  std::vector<double> makeSparseAssemblerAffineCurrentDofs(const std::vector<double> &rest)
  {
    ES::M3d A;
    A << 1.06, 0.02, -0.01,
      0.01, 0.95, 0.03,
      -0.02, 0.01, 1.04;

    std::vector<double> q(rest.size(), 0.0);
    for (int node = 0; node < 8; node++) {
      for (int type = 0; type < 8; type++) {
        const int base = node * 24 + type * 3;
        const ES::V3d source(rest[base + 0], rest[base + 1], rest[base + 2]);
        const ES::V3d value = A * source;
        q[base + 0] = value[0];
        q[base + 1] = value[1];
        q[base + 2] = value[2];
      }
    }
    return q;
  }

  double sparseAssemblerMaxRelativeError(const ES::VXd &a, const ES::VXd &b)
  {
    double err = 0.0;
    for (int i = 0; i < a.size(); i++) {
      err = std::max(err, std::abs(a[i] - b[i]) / std::max(1.0e-9, std::abs(b[i])));
    }
    return err;
  }
  }  // namespace

  TEST(TricubicHermiteAssemblerGTest, SparseKernelPlasticDesignGradientMatchesFiniteDifference)
  {
    using pgo::SolidDeformationModel::ElasticModelStableNeoHookeanMaterial;
    using pgo::SolidDeformationModel::PlasticModel3D6DOF;
    using pgo::SolidDeformationModel::SparseKernelPlasticDesignField;
    using pgo::SolidDeformationModel::TricubicHermiteAssembler;
    using pgo::SolidDeformationModel::TricubicHermiteDeformationModel;
    using pgo::SolidDeformationModel::TricubicHermiteDofLayout;

    const double hx = 4.0, hy = 2.0, hz = 6.0;
    const std::array<int, 8> elementNodes = { 0, 1, 2, 3, 4, 5, 6, 7 };
    auto layout = std::make_shared<TricubicHermiteDofLayout>(8, 1, elementNodes.data(), hx, hy, hz);

    const std::vector<double> restGlobal = makeSparseAssemblerRestGlobalDofs(hx, hy, hz);
    const std::vector<double> currentGlobal = makeSparseAssemblerAffineCurrentDofs(restGlobal);
    std::array<double, 192> restLocal = {};
    layout->gatherElementGeometry(0, restGlobal.data(), restLocal.data());

    ElasticModelStableNeoHookeanMaterial elastic(1.0e4, 1.0e4);
    PlasticModel3D6DOF plastic;
    auto model = std::make_shared<TricubicHermiteDeformationModel>(
      restLocal.data(), &elastic, &plastic, 4);
    const int numQP = model->getNumQuadraturePoints();

    using Field = SparseKernelPlasticDesignField;
    std::vector<std::vector<Field::Influence>> influences(numQP);
    for (int qp = 0; qp < numQP; qp++) {
      influences[qp] = { { 0, 0.35 }, { 1, 0.65 } };
    }
    auto plasticField = std::make_shared<Field>(1, numQP, 2, influences);

    std::vector<std::shared_ptr<const TricubicHermiteDeformationModel>> models = { model };
    TricubicHermiteAssembler assembler(layout, plasticField, models);

    std::vector<double> z = {
      1.0, 0.0, 0.0, 1.0, 0.0, 1.0,
      1.04, 0.02, -0.01, 0.97, 0.015, 1.03
    };
    ASSERT_EQ(static_cast<int>(z.size()), assembler.getNumPlasticDesignDofs());

    ES::VXd analytic = ES::VXd::Zero(assembler.getNumPlasticDesignDofs());
    assembler.computePlasticDesignGradient(currentGlobal.data(), z.data(), nullptr, analytic.data());

    ES::VXd fd = ES::VXd::Zero(assembler.getNumPlasticDesignDofs());
    for (int i = 0; i < assembler.getNumPlasticDesignDofs(); i++) {
      std::vector<double> zp = z;
      std::vector<double> zm = z;
      zp[i] += kSparseAssemblerFdEps;
      zm[i] -= kSparseAssemblerFdEps;
      fd[i] = (assembler.computeEnergy(currentGlobal.data(), zp.data(), nullptr) -
        assembler.computeEnergy(currentGlobal.data(), zm.data(), nullptr)) / (2.0 * kSparseAssemblerFdEps);
    }

    EXPECT_LT(sparseAssemblerMaxRelativeError(analytic, fd), 1.0e-5);
  }
  ```

- [ ] **Step 4: Run plastic field and assembler tests**

  Run:

  ```bash
  cmake --build build/base_no_mkl --target plasticDesignField_gtest tricubicHermiteAssembler_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/plasticDesignField_gtest
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest
  ```

  Expected: all tests pass.

---

## Task 7: Add Regularization Hooks For Plastic Design Space

**Files:**
- Modify: `src/core/solidDeformationModel/plasticDesignField.h`
- Modify: `src/core/solidDeformationModel/plasticDesignField.cpp`
- Modify: `tests/src/core/solidDeformationModel/plasticDesignField_gtest.cpp`

- [ ] **Step 1: Add identity prior energy helper**

  Add functions:

  ```cpp
  double computePlasticIdentityPrior(const double *designParameters, int numDesignParameters, double weight);
  void computePlasticIdentityPriorGradient(const double *designParameters, int numDesignParameters,
    double weight, double *gradient);
  ```

  The identity-like 6D parameter is:

  ```text
  [1, 0, 0, 1, 0, 1]
  ```

  The prior is:

  ```text
  0.5 * weight * ||z - z_identity||^2
  ```

- [ ] **Step 2: Add sparse control smoothness helper**

  Add a helper that accepts weighted control-neighbor pairs:

  ```cpp
  struct PlasticControlNeighbor
  {
    int i = -1;
    int j = -1;
    double weight = 1.0;
  };

  double computePlasticControlSmoothness(const double *designParameters,
    const PlasticControlNeighbor *neighbors, int numNeighbors, double weight);

  void computePlasticControlSmoothnessGradient(const double *designParameters,
    const PlasticControlNeighbor *neighbors, int numNeighbors, double weight, double *gradient);
  ```

  Energy:

  ```text
  0.5 * globalWeight * pairWeight * ||z_i - z_j||^2
  ```

- [ ] **Step 3: Add tests for prior and smoothness gradients**

  Add finite-difference tests in `plasticDesignField_gtest.cpp` for both helpers. Use central difference with `eps = 1e-6` and assert max absolute error below `1e-7`.

- [ ] **Step 4: Run tests**

  Run:

  ```bash
  cmake --build build/base_no_mkl --target plasticDesignField_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/plasticDesignField_gtest
  ```

  Expected: regularization helper tests pass.

---

## Verification Matrix

This matrix is the gate before simulation work. If any command in this section fails, do not add `runSim` integration, do not run a visual simulation demo, and do not tune parameters around the failure.

Run after all tasks:

```bash
cmake --build build/base_no_mkl \
  --target tricubicHermiteBasis_gtest plasticDesignField_gtest tricubicHermiteDeformationModel_gtest tricubicHermiteAssembler_gtest \
  -j 8

build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteBasis_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/plasticDesignField_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteDeformationModel_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest
```

Expected final behavior:

- Hermite basis satisfies endpoint duality and affine Jacobian tests.
- Constant plastic design field maps one 6D `Fp` parameter block to every quadrature point.
- Sparse plastic design field evaluates weighted control parameters and scatters gradients through transposed weights.
- Tricubic Hermite deformation energy is zero at rest and under rigid translation.
- Tricubic Hermite affine deformation energy matches the analytic finite-element formula.
- Geometry gradient, plastic parameter gradient, mixed derivatives, and Hessian-vector checks match finite differences.
- Assembled geometry and plastic design gradients match finite differences for constant and sparse plastic fields.
- Existing tet/cubic tests still build because existing assembler and model APIs remain untouched.

## Scope After This Plan

Only after the verification matrix passes, the next plan can integrate this Hermite path into a user-facing driver or inverse-design objective. That later integration should decide:

- how Hermite geometry DOFs are initialized from a cubic mesh;
- whether the optimizer solves only `z`, only `x`, or a coupled `(x, z)` system;
- whether `Fp` positivity uses projection, barrier terms, or a log/Cholesky parameterization;
- whether neural network plastic fields are hosted in C++ or through an external Python optimization loop.
- how to add simulation tests after the finite-element unit and assembly tests have already proved the formulas.

## Self-Review

- Spec coverage: true Hermite geometry DOFs are covered by Tasks 1, 2, 4, and 5. Constant `Fp` design field is covered by Tasks 3, 4, and 5. Sparse control points + kernel interpolation are covered by Task 6. Regularization needed for design-space stability is covered by Task 7. The finite-element-first requirement is covered by the verification gate, Task 4 derivative checks, Task 5 assembly checks, and the simulation-blocking verification matrix. The external review fixes are incorporated: row-major DOF storage, Hermite/CubicMesh corner-order mismatch, per-quadrature local plastic parameter layout, model-local plastic parameter sizing, plastic model downcast, cache-data ownership, mixed derivative formula, C++20 standard, and fixed finite-difference epsilon.
- Placeholder scan: the plan avoids unresolved placeholder markers, open-ended implementation placeholders, and unspecified test commands. Later-scope items are explicitly separated from this implementation plan.
- Type consistency: the plan consistently uses `TricubicHermiteBasis`, `TricubicHermiteDofLayout`, `PlasticDesignField`, `ConstantPlasticDesignField`, `SparseKernelPlasticDesignField`, `TricubicHermiteDeformationModel`, and `TricubicHermiteAssembler`.
