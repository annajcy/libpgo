# Tricubic Hermite Plastic Field Simulation Integration Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Do not stage or commit unless the user explicitly asks for git integration.

**Goal:** Connect the verified tricubic Hermite FEM + plastic design field path to `runIPCSim` as a real dynamic simulation mode with IPC contact, mapped surface output, gravity, initial velocity, fixed constraints, optional surface pressure, restart, and stress diagnostics.

**Architecture:** Keep the existing tet/cubic/shell vertex-DOF simulation path intact. Add a separate Hermite volume path for `cubic-mesh` inputs where the simulation state is the full Hermite geometry displacement vector, not vertex-only displacement. Surface contact, output, pressure, and attachments use sparse maps from Hermite DOFs to surface vertices.

**Tech Stack:** C++20, Eigen sparse matrices, existing `PotentialEnergy` / `ImplicitBackwardEulerTimeIntegrator`, existing IPC `MappedSurfacePotentialEnergy`, `CubicMesh`, `BarycentricCoordinates`, GoogleTest, CMake.

---

## Preconditions

This plan starts only after `plan/tricubic_hermit_plastic_field_fem.plan.md` passes its full finite-element verification matrix:

```bash
cmake --build build/base_no_mkl \
  --target tricubicHermiteBasis_gtest plasticDesignField_gtest tricubicHermiteDeformationModel_gtest tricubicHermiteAssembler_gtest \
  -j 8

build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteBasis_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/plasticDesignField_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteDeformationModel_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest
```

Do not implement this simulation plan by weakening the finite-element tests. If a simulation smoke test fails because FE derivatives are wrong, fix the FE layer first.

---

## Design Decisions

1. **Hermite simulation uses all Hermite geometry DOFs.**

   ```text
   q.size() = numCubicMeshNodes * 8 Hermite types * 3 coordinates
   ```

   This is a true Hermite simulation state. It is not a vertex-only simulation that happens to evaluate Hermite elements internally.

2. **The existing `DeformationModelManager` and `DeformationModelAssembler` stay vertex-only.**

   The old classes assume:

   ```text
   localDOFs = numElementVertices * 3
   globalDOFs = numVertices * 3
   plastic params = numElements * plasticModel->getNumParameters()
   ```

   Those assumptions are incompatible with full Hermite geometry DOFs and quadrature-sampled plastic fields. The Hermite path uses `TricubicHermiteDofLayout`, `TricubicHermiteAssembler`, and a Hermite-specific `PotentialEnergy` wrapper.

3. **IPC contact already has the right abstraction.**

   `MappedSurfacePotentialEnergy` consumes:

   ```text
   surfaceRestPositions
   surfaceFromSimulationDispMap
   ```

   Therefore the Hermite path must build:

   ```text
   W_surface_from_hermite: R^(numHermiteDofs) -> R^(3 * numSurfaceVertices)
   ```

   IPC does not need to know whether the underlying simulation DOFs are vertices or Hermite derivatives.

4. **Body acceleration and initial velocity are fields, not repeated per DOF triple.**

   The old vertex path can set every vertex velocity to `init-vel` and every vertex acceleration to `g`. The Hermite path must encode a constant translation field:

   ```text
   value DOF velocity     = init-vel
   derivative DOF velocity = 0
   value DOF acceleration     = g
   derivative DOF acceleration = 0
   ```

   Applying `g` or `init-vel` to derivative DOFs is wrong and injects nonphysical derivative motion.

5. **Forward simulation keeps plastic design DOFs fixed.**

   `Fp` design variables are simulation parameters for this plan, not dynamic state variables. The dynamic unknown is only Hermite geometry displacement. Later inverse-design plans may optimize `z`, but that is not part of `runIPCSim`.

6. **First supported simulation mode is dynamic volume IPC on cubic meshes.**

   `simulation-discretization = "tricubic-hermite"` requires:

   ```text
   cubic-mesh present
   surface-mesh present
   sim-type == dynamic
   elastic-material in { stable-neo, stvk-vol }
   plastic-field.type in { constant, sparse-kernel }
   ```

   It rejects `tet-mesh` and shell-only configs.

7. **Fixed constraints are surface-mapped in Hermite mode.**

   Existing `MultipleVertexPulling` indexes direct simulation vertices. For Hermite DOFs, fixed vertex files in this plan refer to surface vertex indices and are pulled through the same Hermite surface map. This keeps attachments meaningful even when simulation DOFs are not mesh vertices.

---

## Config Contract

Add one top-level selector. Absence keeps existing behavior:

```json
"simulation-discretization": "vertex"
```

Hermite mode:

```json
"simulation-discretization": "tricubic-hermite",
"hermite": {
  "quadrature-order": 4,
  "mass-quadrature-order": 4,
  "fixed-vertices-space": "surface",
  "plastic-field": {
    "type": "constant",
    "params": [1, 0, 0, 1, 0, 1]
  }
}
```

Sparse control points:

```json
"hermite": {
  "quadrature-order": 4,
  "mass-quadrature-order": 4,
  "fixed-vertices-space": "surface",
  "plastic-field": {
    "type": "sparse-kernel",
    "radius": 0.08,
    "control-points": [
      { "position": [0.0, 0.0, 0.0], "params": [1, 0, 0, 1, 0, 1] },
      { "position": [0.1, 0.0, 0.0], "params": [1.02, 0, 0, 1, 0, 1] }
    ]
  }
}
```

---

## File Map

Create:

- `src/core/solidDeformationModel/tricubicHermiteCubicMeshAdapter.h`
- `src/core/solidDeformationModel/tricubicHermiteCubicMeshAdapter.cpp`
- `src/core/solidDeformationModel/tricubicHermiteEmbedding.h`
- `src/core/solidDeformationModel/tricubicHermiteEmbedding.cpp`
- `src/core/solidDeformationModel/tricubicHermiteMassMatrix.h`
- `src/core/solidDeformationModel/tricubicHermiteMassMatrix.cpp`
- `src/core/solidDeformationModel/tricubicHermiteDeformationEnergy.h`
- `src/core/solidDeformationModel/tricubicHermiteDeformationEnergy.cpp`
- `src/core/constraintPotentialEnergies/mappedPointPullingPotentialEnergy.h`
- `src/core/constraintPotentialEnergies/mappedPointPullingPotentialEnergy.cpp`
- `src/tools/runSim/runIPCSimHermiteSetup.h`
- `src/tools/runSim/runIPCSimHermiteSetup.cpp`
- `tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest.cpp`
- `tests/src/core/constraintPotentialEnergies/mappedPointPullingPotentialEnergy_gtest.cpp`
- `examples/ipc/hermite/box-drop/box-hermite-ipc.json`

Modify:

- `src/core/solidDeformationModel/tricubicHermiteAssembler.h`
- `src/core/solidDeformationModel/tricubicHermiteAssembler.cpp`
- `src/core/solidDeformationModel/tricubicHermiteDeformationModel.h`
- `src/core/solidDeformationModel/tricubicHermiteDeformationModel.cpp`
- `src/core/solidDeformationModel/CMakeLists.txt`
- `src/core/constraintPotentialEnergies/CMakeLists.txt`
- `src/tools/runSim/runIPCSimSetup.h`
- `src/tools/runSim/runIPCSimSetup.cpp`
- `src/tools/runSim/runIPCSim.cpp`
- `src/tools/runSim/CMakeLists.txt`
- `tests/src/core/solidDeformationModel/CMakeLists.txt`
- `tests/src/core/constraintPotentialEnergies/CMakeLists.txt`
- `tests/src/tools/runIPCSim_gtest.cpp`

Do not modify in this plan:

- tet simulation setup behavior
- cubic vertex-DOF simulation setup behavior
- shell setup behavior
- IPC core barrier formulas
- Newton/integrator solver internals

---

## Task 1: Add Hermite Simulation Config Parsing

**Files:**
- Modify: `src/tools/runSim/runIPCSimSetup.h`
- Modify: `src/tools/runSim/runIPCSimSetup.cpp`
- Create: `src/tools/runSim/runIPCSimHermiteSetup.h`
- Create: `src/tools/runSim/runIPCSimHermiteSetup.cpp`
- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: Add config parser tests**

  Add tests that verify default behavior is unchanged and Hermite mode rejects unsupported inputs.

  ```cpp
  pgo::ConfigFileJSON makeJson(const std::string &contents)
  {
    const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "run_ipc_sim_hermite_config_test.json";
    {
      std::ofstream out(path);
      out << contents;
    }

    pgo::ConfigFileJSON config;
    if (!config.open(path.string().c_str())) {
      throw std::runtime_error("Failed to open temporary Hermite config test JSON.");
    }
    return config;
  }

  TEST(RunIPCSimConfigGTest, MissingSimulationDiscretizationDefaultsToVertex)
  {
    pgo::RunIPCSim::HermiteSimulationConfig config;
    EXPECT_FALSE(pgo::RunIPCSim::tryParseHermiteSimulationConfig(makeJson("{}"), config));
  }

  TEST(RunIPCSimConfigGTest, ParsesHermiteConstantPlasticField)
  {
    const auto json = makeJson(R"({
      "simulation-discretization": "tricubic-hermite",
      "cubic-mesh": "box.veg",
      "surface-mesh": "box.obj",
      "sim-type": "dynamic",
      "elastic-material": "stable-neo",
      "hermite": {
        "quadrature-order": 4,
        "mass-quadrature-order": 4,
        "fixed-vertices-space": "surface",
        "plastic-field": {
          "type": "constant",
          "params": [1, 0, 0, 1, 0, 1]
        }
      }
    })");

    pgo::RunIPCSim::HermiteSimulationConfig config;
    ASSERT_TRUE(pgo::RunIPCSim::tryParseHermiteSimulationConfig(json, config));
    EXPECT_EQ(config.quadratureOrder, 4);
    EXPECT_EQ(config.massQuadratureOrder, 4);
    EXPECT_EQ(config.fixedVerticesSpace, pgo::RunIPCSim::HermiteFixedVerticesSpace::SURFACE);
    EXPECT_EQ(config.plasticField.type, pgo::RunIPCSim::HermitePlasticFieldType::CONSTANT);
    EXPECT_DOUBLE_EQ(config.plasticField.constantParams[0], 1.0);
    EXPECT_DOUBLE_EQ(config.plasticField.constantParams[5], 1.0);
  }
  ```

- [ ] **Step 2: Add parser API**

  Create `src/tools/runSim/runIPCSimHermiteSetup.h` with:

  ```cpp
  #pragma once

  #include "EigenSupport.h"

  #include <array>
  #include <string>
  #include <vector>

  namespace pgo
  {
  class ConfigFileJSON;

  namespace RunIPCSim
  {
  enum class HermiteFixedVerticesSpace
  {
    SURFACE,
    VALUE_NODE,
  };

  enum class HermitePlasticFieldType
  {
    CONSTANT,
    SPARSE_KERNEL,
  };

  struct HermitePlasticControlPointConfig
  {
    EigenSupport::V3d position = EigenSupport::V3d::Zero();
    std::array<double, 6> params = { 1, 0, 0, 1, 0, 1 };
  };

  struct HermitePlasticFieldConfig
  {
    HermitePlasticFieldType type = HermitePlasticFieldType::CONSTANT;
    std::array<double, 6> constantParams = { 1, 0, 0, 1, 0, 1 };
    double radius = 0.0;
    std::vector<HermitePlasticControlPointConfig> controlPoints;
  };

  struct HermiteSimulationConfig
  {
    int quadratureOrder = 4;
    int massQuadratureOrder = 4;
    HermiteFixedVerticesSpace fixedVerticesSpace = HermiteFixedVerticesSpace::SURFACE;
    HermitePlasticFieldConfig plasticField;
  };

  bool tryParseHermiteSimulationConfig(const ConfigFileJSON &jconfig, HermiteSimulationConfig &config);
  }  // namespace RunIPCSim
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement parser validation**

  Implement:

  ```cpp
  bool tryParseHermiteSimulationConfig(const pgo::ConfigFileJSON &jconfig, HermiteSimulationConfig &config)
  {
    if (!jconfig.exist("simulation-discretization"))
      return false;

    const std::string mode = jconfig.getString("simulation-discretization");
    if (mode == "vertex")
      return false;
    if (mode != "tricubic-hermite")
      throw std::invalid_argument("simulation-discretization must be `vertex` or `tricubic-hermite`.");

    if (!jconfig.exist("cubic-mesh"))
      throw std::invalid_argument("tricubic-hermite simulation requires `cubic-mesh`.");
    if (jconfig.exist("tet-mesh"))
      throw std::invalid_argument("tricubic-hermite simulation does not support `tet-mesh`.");
    if (jconfig.getString("sim-type") != "dynamic")
      throw std::invalid_argument("tricubic-hermite simulation only supports `sim-type = dynamic`.");

    const auto &hermite = jconfig.handle().at("hermite");
    config.quadratureOrder = hermite.value("quadrature-order", 4);
    config.massQuadratureOrder = hermite.value("mass-quadrature-order", config.quadratureOrder);
    if (config.quadratureOrder < 4)
      throw std::invalid_argument("hermite.quadrature-order must be at least 4.");
    if (config.massQuadratureOrder < 4)
      throw std::invalid_argument("hermite.mass-quadrature-order must be at least 4.");

    const std::string fixedSpace = hermite.value("fixed-vertices-space", "surface");
    if (fixedSpace == "surface")
      config.fixedVerticesSpace = HermiteFixedVerticesSpace::SURFACE;
    else if (fixedSpace == "value-node")
      config.fixedVerticesSpace = HermiteFixedVerticesSpace::VALUE_NODE;
    else
      throw std::invalid_argument("hermite.fixed-vertices-space must be `surface` or `value-node`.");

    const auto &plastic = hermite.at("plastic-field");
    const std::string plasticType = plastic.at("type").get<std::string>();
    if (plasticType == "constant") {
      config.plasticField.type = HermitePlasticFieldType::CONSTANT;
      config.plasticField.constantParams = plastic.at("params").get<std::array<double, 6>>();
    }
    else if (plasticType == "sparse-kernel") {
      config.plasticField.type = HermitePlasticFieldType::SPARSE_KERNEL;
      config.plasticField.radius = plastic.at("radius").get<double>();
      if (config.plasticField.radius <= 0.0)
        throw std::invalid_argument("hermite.plastic-field.radius must be positive.");
      config.plasticField.controlPoints.clear();
      for (const auto &cpJson : plastic.at("control-points")) {
        HermitePlasticControlPointConfig cp;
        const auto p = cpJson.at("position").get<std::array<double, 3>>();
        cp.position = EigenSupport::V3d(p[0], p[1], p[2]);
        cp.params = cpJson.at("params").get<std::array<double, 6>>();
        config.plasticField.controlPoints.push_back(cp);
      }
      if (config.plasticField.controlPoints.empty())
        throw std::invalid_argument("sparse-kernel plastic field requires at least one control point.");
    }
    else {
      throw std::invalid_argument("hermite.plastic-field.type must be `constant` or `sparse-kernel`.");
    }

    return true;
  }
  ```

- [ ] **Step 4: Run config tests**

  ```bash
  cmake --build build/base_no_mkl --target runIPCSim_gtest -j 8
  build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='*Hermite*'
  ```

  Expected: parser tests pass and existing `runIPCSim_gtest` tests still pass.

---

## Task 2: Add CubicMesh-To-Hermite Adapter

**Files:**
- Create: `src/core/solidDeformationModel/tricubicHermiteCubicMeshAdapter.h`
- Create: `src/core/solidDeformationModel/tricubicHermiteCubicMeshAdapter.cpp`
- Modify: `tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add adapter tests**

  Add tests that verify CubicMesh local order is converted to Hermite order and rest global DOFs represent an affine identity chart.

  ```cpp
  std::unique_ptr<pgo::VolumetricMeshes::CubicMesh> makeOneCubeCubicMesh(double h)
  {
    const std::array<double, 24> vertices = {
      0, 0, 0,
      h, 0, 0,
      h, h, 0,
      0, h, 0,
      0, 0, h,
      h, 0, h,
      h, h, h,
      0, h, h,
    };
    const std::array<int, 8> elements = { 0, 1, 2, 3, 4, 5, 6, 7 };
    return std::make_unique<pgo::VolumetricMeshes::CubicMesh>(
      8, vertices.data(), 1, elements.data(), 1.0e4, 0.4, 1.0);
  }

  TEST(TricubicHermiteSimulationGTest, CubicElementOrderIsConvertedToHermiteOrder)
  {
    const std::array<int, 8> cubicOrder = { 10, 11, 12, 13, 14, 15, 16, 17 };
    std::array<int, 8> hermiteOrder = {};
    pgo::SolidDeformationModel::convertCubicElementNodesToHermiteOrder(cubicOrder.data(), hermiteOrder.data());
    const std::array<int, 8> expected = { 10, 11, 13, 12, 14, 15, 17, 16 };
    EXPECT_EQ(hermiteOrder, expected);
  }

  TEST(TricubicHermiteSimulationGTest, RestDofsEncodeAffineIdentityDerivatives)
  {
    auto cubicMesh = makeOneCubeCubicMesh(2.0);
    const auto adapter = pgo::SolidDeformationModel::buildTricubicHermiteCubicMeshAdapter(*cubicMesh);
    ASSERT_EQ(adapter.restDofs.size(), adapter.layout->getNumGlobalGeometryDofs());

    const int node = 0;
    EXPECT_NEAR(adapter.restDofs[node * 24 + 3 + 0], 1.0, 1e-12);
    EXPECT_NEAR(adapter.restDofs[node * 24 + 6 + 1], 1.0, 1e-12);
    EXPECT_NEAR(adapter.restDofs[node * 24 + 9 + 2], 1.0, 1e-12);
    for (int type = 4; type < 8; ++type) {
      EXPECT_NEAR(adapter.restDofs[node * 24 + type * 3 + 0], 0.0, 1e-12);
      EXPECT_NEAR(adapter.restDofs[node * 24 + type * 3 + 1], 0.0, 1e-12);
      EXPECT_NEAR(adapter.restDofs[node * 24 + type * 3 + 2], 0.0, 1e-12);
    }
  }
  ```

- [ ] **Step 2: Add adapter API**

  ```cpp
  #pragma once

  #include "EigenDef.h"

  #include <memory>
  #include <vector>

  namespace pgo
  {
  namespace VolumetricMeshes
  {
  class CubicMesh;
  }

  namespace SolidDeformationModel
  {
  class TricubicHermiteDofLayout;

  struct TricubicHermiteCubicMeshAdapter
  {
    std::shared_ptr<TricubicHermiteDofLayout> layout;
    EigenSupport::VXd restDofs;
    double hx = 1.0;
    double hy = 1.0;
    double hz = 1.0;
    std::vector<int> hermiteElementNodes;
  };

  void convertCubicElementNodesToHermiteOrder(const int cubicNodes[8], int hermiteNodes[8]);
  TricubicHermiteCubicMeshAdapter buildTricubicHermiteCubicMeshAdapter(
    const VolumetricMeshes::CubicMesh &cubicMesh);
  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement adapter**

  Implementation rules:

  ```text
  cubic local order   = {000,100,110,010,001,101,111,011}
  Hermite local order = {000,100,010,110,001,101,011,111}
  conversion indices  = {0,1,3,2,4,5,7,6}
  ```

  Rest DOF convention:

  ```text
  type 0: vertex position
  type 1: physical dX/dx = (1,0,0)
  type 2: physical dX/dy = (0,1,0)
  type 3: physical dX/dz = (0,0,1)
  type 4..7: mixed derivatives = 0
  ```

  `TricubicHermiteDofLayout::gatherElementGeometry` multiplies these physical derivative DOFs by `hx/2`, `hy/2`, `hz/2`, so local element DOFs become reference-coordinate Hermite derivatives.

- [ ] **Step 4: Run adapter tests**

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteSimulation_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest --gtest_filter='*Cubic*Hermite*'
  ```

  Expected: adapter tests pass.

---

## Task 3: Build Surface-To-Hermite Embedding Matrix

**Files:**
- Create: `src/core/solidDeformationModel/tricubicHermiteEmbedding.h`
- Create: `src/core/solidDeformationModel/tricubicHermiteEmbedding.cpp`
- Modify: `tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add embedding tests**

	  ```cpp
	  TEST(TricubicHermiteSimulationGTest, ReferenceDerivativeScaleUsesHalfEdgeProducts)
	  {
	    auto cubicMesh = makeOneCubeCubicMesh(1.0);
	    const auto adapter = pgo::SolidDeformationModel::buildTricubicHermiteCubicMeshAdapter(*cubicMesh);

	    EXPECT_NEAR(adapter.layout->referenceDerivativeScale(0), 1.0, 1e-12);
	    EXPECT_NEAR(adapter.layout->referenceDerivativeScale(1), 0.5, 1e-12);
	    EXPECT_NEAR(adapter.layout->referenceDerivativeScale(2), 0.5, 1e-12);
	    EXPECT_NEAR(adapter.layout->referenceDerivativeScale(3), 0.5, 1e-12);
	    EXPECT_NEAR(adapter.layout->referenceDerivativeScale(4), 0.25, 1e-12);
	    EXPECT_NEAR(adapter.layout->referenceDerivativeScale(7), 0.125, 1e-12);
	  }

	  TEST(TricubicHermiteSimulationGTest, HermiteEmbeddingReproducesAffineRestSurfacePositions)
	  {
	    auto cubicMesh = makeOneCubeCubicMesh(2.0);
	    const auto adapter = pgo::SolidDeformationModel::buildTricubicHermiteCubicMeshAdapter(*cubicMesh);

	    const double surfacePoints[6] = {
	      0.25, 0.50, 1.25,
	      1.00, 0.25, 0.75,
	    };
	    pgo::InterpolationCoordinates::BarycentricCoordinates bc(2, surfacePoints, cubicMesh.get());

	    const EigenSupport::SpMatD W =
	      pgo::SolidDeformationModel::buildTricubicHermiteSurfaceEmbeddingMatrix(
	        *adapter.layout, bc);

	    ASSERT_EQ(W.rows(), 6);
	    ASSERT_EQ(W.cols(), adapter.restDofs.size());
	    const EigenSupport::VXd embedded = W * adapter.restDofs;
	    for (int i = 0; i < 6; ++i)
	      EXPECT_NEAR(embedded[i], surfacePoints[i], 1e-12);
	  }

	  TEST(TricubicHermiteSimulationGTest, HermiteEmbeddingMapsRigidTranslation)
	  {
	    auto cubicMesh = makeOneCubeCubicMesh(2.0);
	    const auto adapter = pgo::SolidDeformationModel::buildTricubicHermiteCubicMeshAdapter(*cubicMesh);
	    const double surfacePoint[3] = { 0.25, 0.50, 1.25 };
	    pgo::InterpolationCoordinates::BarycentricCoordinates bc(1, surfacePoint, cubicMesh.get());
	    const EigenSupport::SpMatD W =
	      pgo::SolidDeformationModel::buildTricubicHermiteSurfaceEmbeddingMatrix(*adapter.layout, bc);

	    EigenSupport::VXd q = EigenSupport::VXd::Zero(adapter.restDofs.size());
	    pgo::SolidDeformationModel::setTricubicHermiteConstantTranslation(
	      *adapter.layout, EigenSupport::V3d(3.0, -2.0, 5.0), q.data());

	    const EigenSupport::VXd surfaceDisp = W * q;
	    EXPECT_NEAR(surfaceDisp[0], 3.0, 1e-12);
	    EXPECT_NEAR(surfaceDisp[1], -2.0, 1e-12);
	    EXPECT_NEAR(surfaceDisp[2], 5.0, 1e-12);
	  }

	  TEST(TricubicHermiteSimulationGTest, CubicBarycentricWeightsRecoverReferenceCoordinates)
	  {
	    auto cubicMesh = makeOneCubeCubicMesh(2.0);
	    const double surfacePoint[3] = { 0.25, 0.50, 1.25 };
	    pgo::InterpolationCoordinates::BarycentricCoordinates bc(1, surfacePoint, cubicMesh.get());
	    ASSERT_EQ(bc.getEmbeddingElement(0), 0);

	    const double *w = bc.getEmbeddingWeights(0);
	    const double alpha = w[1] + w[2] + w[5] + w[6];
	    const double beta = w[2] + w[3] + w[6] + w[7];
	    const double gamma = w[4] + w[5] + w[6] + w[7];

	    EXPECT_NEAR(alpha, 0.125, 1e-12);
	    EXPECT_NEAR(beta, 0.25, 1e-12);
	    EXPECT_NEAR(gamma, 0.625, 1e-12);
	  }
	  ```

- [ ] **Step 2: Add embedding API**

  ```cpp
  #pragma once

  #include "EigenDef.h"

  namespace pgo
  {
  namespace InterpolationCoordinates
  {
  class BarycentricCoordinates;
  }

  namespace SolidDeformationModel
  {
  class TricubicHermiteDofLayout;

  EigenSupport::SpMatD buildTricubicHermiteSurfaceEmbeddingMatrix(
    const TricubicHermiteDofLayout &layout,
    const InterpolationCoordinates::BarycentricCoordinates &surfaceEmbedding);

  void setTricubicHermiteConstantTranslation(
    const TricubicHermiteDofLayout &layout,
    const EigenSupport::V3d &translation,
    double *globalDofs);

  void setTricubicHermiteConstantVectorFieldOnValueDofs(
    const TricubicHermiteDofLayout &layout,
    const EigenSupport::V3d &value,
    double *globalDofs);
  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement embedding**

	  For each surface vertex, use `BarycentricCoordinates` to get the containing cubic element and cubic trilinear weights. The formula below assumes the weights are in the existing `CubicMesh` local vertex order `{000,100,110,010,001,101,111,011}`. Do not change this order silently; `CubicBarycentricWeightsRecoverReferenceCoordinates` is the regression test that ties `BarycentricCoordinates::getEmbeddingWeights` to this convention. Recover cube coordinates:

  ```cpp
  const double alpha = w[1] + w[2] + w[5] + w[6];
  const double beta  = w[2] + w[3] + w[6] + w[7];
  const double gamma = w[4] + w[5] + w[6] + w[7];
  const double xi = 2.0 * alpha - 1.0;
  const double eta = 2.0 * beta - 1.0;
  const double zeta = 2.0 * gamma - 1.0;
	  ```

	  Evaluate 64 Hermite basis values at `(xi, eta, zeta)`. This step depends on the FEM plan's `TricubicHermiteDofLayout::referenceDerivativeScale(int type)` API; do not duplicate the scaling table in the embedding code. For each local scalar basis `corner,type`, write sparse entries:

  ```text
  W(surfaceVertex*3 + coord, globalNode*24 + type*3 + coord)
    += basis[corner*8 + type] * layout.referenceDerivativeScale(type)
  ```

  The multiplication by `referenceDerivativeScale(type)` is required because global derivative DOFs are physical derivatives, while local Hermite element DOFs are reference-coordinate derivatives.

- [ ] **Step 4: Run embedding tests**

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteSimulation_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest --gtest_filter='*Embedding*'
  ```

  Expected: affine rest positions and rigid translations are reproduced to `1e-12`.

---

## Task 4: Add Hermite Consistent Mass Matrix

**Files:**
- Create: `src/core/solidDeformationModel/tricubicHermiteMassMatrix.h`
- Create: `src/core/solidDeformationModel/tricubicHermiteMassMatrix.cpp`
- Modify: `tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add mass matrix tests**

  ```cpp
  TEST(TricubicHermiteSimulationGTest, HermiteMassMatrixIsSymmetricAndTranslationMassMatchesTotalMass)
  {
    auto cubicMesh = makeOneCubeCubicMesh(2.0);
    const auto adapter = pgo::SolidDeformationModel::buildTricubicHermiteCubicMeshAdapter(*cubicMesh);

    EigenSupport::SpMatD M;
    pgo::SolidDeformationModel::computeTricubicHermiteMassMatrix(
      *cubicMesh, *adapter.layout, 4, M);

    ASSERT_EQ(M.rows(), adapter.restDofs.size());
    ASSERT_EQ(M.cols(), adapter.restDofs.size());
    EXPECT_NEAR((M - EigenSupport::SpMatD(M.transpose())).norm(), 0.0, 1e-10);

    EigenSupport::VXd v = EigenSupport::VXd::Zero(M.rows());
    pgo::SolidDeformationModel::setTricubicHermiteConstantVectorFieldOnValueDofs(
      *adapter.layout, EigenSupport::V3d(2.0, -1.0, 0.5), v.data());

    const double totalMass = cubicMesh->getElementVolume(0) * cubicMesh->getElementDensity(0);
    const double kineticTwice = v.dot(M * v);
    EXPECT_NEAR(kineticTwice, totalMass * (2.0 * 2.0 + 1.0 + 0.25), 1e-8);
  }
  ```

- [ ] **Step 2: Add mass API**

  ```cpp
  #pragma once

  #include "EigenDef.h"

  namespace pgo
  {
  namespace VolumetricMeshes
  {
  class CubicMesh;
  }

  namespace SolidDeformationModel
  {
  class TricubicHermiteDofLayout;

  void computeTricubicHermiteMassMatrix(
    const VolumetricMeshes::CubicMesh &cubicMesh,
    const TricubicHermiteDofLayout &layout,
    int quadratureOrder,
    EigenSupport::SpMatD &massMatrix);
  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement mass assembly**

  For each element and quadrature point:

  ```text
  M_ij += rho * det(JX_q) * w_q * N_i(xi_q) * scale_i * N_j(xi_q) * scale_j
  ```

  Inflate each scalar entry to the three coordinate blocks:

  ```text
  M(global_i_coord, global_j_coord) += scalarMass
  ```

  Use the same Hermite local order and `referenceDerivativeScale(type)` convention as the embedding matrix.

- [ ] **Step 4: Run mass tests**

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteSimulation_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest --gtest_filter='*Mass*'
  ```

  Expected: matrix dimensions are correct, symmetry error is below `1e-10`, and rigid translation kinetic mass matches total physical mass.

---

## Task 5: Extend Hermite Assembler For Simulation Hessians, Max Step, And Stress

**Files:**
- Modify: `src/core/solidDeformationModel/tricubicHermiteAssembler.h`
- Modify: `src/core/solidDeformationModel/tricubicHermiteAssembler.cpp`
- Modify: `src/core/solidDeformationModel/tricubicHermiteDeformationModel.h`
- Modify: `src/core/solidDeformationModel/tricubicHermiteDeformationModel.cpp`
- Modify: `tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest.cpp`

- [ ] **Step 1: Add assembler simulation tests**

  Add tests:

  ```text
  getGeometryHessianTemplate has rows/cols = layout.getNumGlobalGeometryDofs()
  computeGeometryHessian matches finite differences of computeGeometryGradient
  computeMaterialMaxStepObservation returns alpha < 1 for a direction that inverts Jx
  computeVonMisesStresses returns one nonnegative scalar per element
  ```

- [ ] **Step 2: Extend assembler API**

  Add:

  ```cpp
  struct HermiteMaterialMaxStepObservation
  {
    double alpha = 1.0;
    bool hasIllegalInitialState = false;
    int limitingElementId = -1;
    int limitingQuadraturePointId = -1;
  };

  const EigenSupport::SpMatD &getGeometryHessianTemplate() const;

  void computeGeometryHessian(const double *geometryDofs, const double *plasticDesignDofs,
    const double *elasticParams, EigenSupport::SpMatD &hessian) const;

  HermiteMaterialMaxStepObservation computeMaterialMaxStepObservation(
    const double *geometryDofs, const double *geometryDofDirection) const;

  void computeVonMisesStresses(const double *geometryDofs, const double *plasticDesignDofs,
    const double *elasticParams, double *elementStresses) const;
  ```

- [ ] **Step 3: Implement Hessian template and scatter**

  Build a global Hessian template from element node pairs and all Hermite types:

  ```text
  for each element
    for corner_i,type_i,coord_i
      for corner_j,type_j,coord_j
        row = node_i * 24 + type_i * 3 + coord_i
        col = node_j * 24 + type_j * 3 + coord_j
  ```

  When scattering local Hessians, multiply by the same derivative scales already used by `TricubicHermiteDofLayout::scatterAddElementGeometryGradient`.

- [ ] **Step 4: Implement material max step**

  For each quadrature point, check positivity of current `det(Jx)` along direction `dJx`:

  ```text
  Jx(alpha) = Jx0 + alpha * dJx
  phi(alpha) = det(Jx(alpha))
  ```

  Reuse the cubic/tet material max-step polynomial utilities if possible. If not, add a Hermite-local helper that computes the smallest positive root where `phi(alpha) <= eps`, clamps to `[0, 1]`, and returns `kMaterialMaxStepMinClamp` for illegal initial states. This protects the true Hermite geometry Jacobian, which is the relevant simulation state.

- [ ] **Step 5: Implement von Mises stress**

  Add `TricubicHermiteDeformationModel::vonMisesStress` using the same formula as `CubicMeshDeformationModel::vonMisesStress`:

  ```text
  P = d psi / d Fe
  cauchy = P * Fe^T / det(Fe)
  vonMises = sqrt(0.5 * ((s00-s11)^2 + (s11-s22)^2 + (s22-s00)^2
              + 6 * (s12^2 + s20^2 + s01^2)))
  ```

  The assembler reports the maximum quadrature-point value per element.

- [ ] **Step 6: Run assembler simulation tests**

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteAssembler_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest --gtest_filter='*Hessian*:*MaxStep*:*VonMises*'
  ```

  Expected: finite-difference Hessian checks pass, max-step catches inversion directions, and stress outputs are finite.

---

## Task 6: Add Hermite PotentialEnergy Wrapper

**Files:**
- Create: `src/core/solidDeformationModel/tricubicHermiteDeformationEnergy.h`
- Create: `src/core/solidDeformationModel/tricubicHermiteDeformationEnergy.cpp`
- Modify: `tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest.cpp`
- Modify: `src/core/solidDeformationModel/CMakeLists.txt`

- [ ] **Step 1: Add energy wrapper tests**

  ```cpp
  struct OneCubeHermiteSimulationFixture
  {
    std::shared_ptr<pgo::SolidDeformationModel::TricubicHermiteAssembler> assembler;
    std::shared_ptr<pgo::SolidDeformationModel::ElasticModelStableNeoHookeanMaterial> elastic;
    std::shared_ptr<pgo::SolidDeformationModel::PlasticModel3D6DOF> plastic;
    EigenSupport::VXd restDofs;
    EigenSupport::VXd plasticDesignDofs;
  };

  OneCubeHermiteSimulationFixture makeOneCubeHermiteSimulationFixture()
  {
    auto cubicMesh = makeOneCubeCubicMesh(2.0);
    auto adapter = pgo::SolidDeformationModel::buildTricubicHermiteCubicMeshAdapter(*cubicMesh);

	    const int numQpPerElement = 64;  // 4x4x4 quadrature points per element, not global QP count.
	    auto plasticField =
	      std::make_shared<pgo::SolidDeformationModel::ConstantPlasticDesignField>(1, numQpPerElement);
	    if (plasticField->getNumQuadraturePointsPerElement() != numQpPerElement)
	      throw std::runtime_error("ConstantPlasticDesignField constructor must take per-element quadrature count.");

    double localRest[192] = {};
    adapter.layout->gatherElementGeometry(0, adapter.restDofs.data(), localRest);

    OneCubeHermiteSimulationFixture fixture;
    fixture.elastic = std::make_shared<pgo::SolidDeformationModel::ElasticModelStableNeoHookeanMaterial>(1.0e4, 1.0e4);
    fixture.plastic = std::make_shared<pgo::SolidDeformationModel::PlasticModel3D6DOF>();

    std::vector<std::shared_ptr<const pgo::SolidDeformationModel::TricubicHermiteDeformationModel>> models;
    models.push_back(std::make_shared<pgo::SolidDeformationModel::TricubicHermiteDeformationModel>(
      localRest, fixture.elastic.get(), fixture.plastic.get(), 4));

    fixture.assembler = std::make_shared<pgo::SolidDeformationModel::TricubicHermiteAssembler>(
      adapter.layout, plasticField, models);
    fixture.restDofs = adapter.restDofs;
    fixture.plasticDesignDofs = EigenSupport::VXd::Zero(6);
    fixture.plasticDesignDofs << 1, 0, 0, 1, 0, 1;
    return fixture;
  }

  TEST(TricubicHermiteSimulationGTest, HermitePotentialEnergyUsesDisplacementState)
  {
    auto fixture = makeOneCubeHermiteSimulationFixture();
    pgo::SolidDeformationModel::TricubicHermiteDeformationEnergy energy(
      fixture.assembler, fixture.restDofs, fixture.plasticDesignDofs, EigenSupport::VXd());

    EigenSupport::VXd q = EigenSupport::VXd::Zero(fixture.restDofs.size());
    EXPECT_NEAR(energy.func(q), 0.0, 1e-9);

    EigenSupport::VXd grad = EigenSupport::VXd::Zero(q.size());
    energy.gradient(q, grad);
    EXPECT_NEAR(grad.norm(), 0.0, 1e-7);
  }
  ```

- [ ] **Step 2: Add wrapper API**

  ```cpp
  #pragma once

  #include "potentialEnergy.h"

  #include <memory>
  #include <vector>

  namespace pgo
  {
  namespace SolidDeformationModel
  {
  class TricubicHermiteAssembler;

  class TricubicHermiteDeformationEnergy : public NonlinearOptimization::PotentialEnergy
  {
  public:
    TricubicHermiteDeformationEnergy(std::shared_ptr<const TricubicHermiteAssembler> assembler,
      EigenSupport::VXd restDofs, EigenSupport::VXd plasticDesignDofs, EigenSupport::VXd elasticParams);

    double func(EigenSupport::ConstRefVecXd q) const override;
    void gradient(EigenSupport::ConstRefVecXd q, EigenSupport::RefVecXd grad) const override;
    void hessian(EigenSupport::ConstRefVecXd q, EigenSupport::SpMatD &hess) const override;
    void createHessian(EigenSupport::SpMatD &hess) const override;
    void getDOFs(std::vector<int> &dofs) const override;
    int getNumDOFs() const override;
    NonlinearOptimization::MaxStepResult computeMaxStepLimit(
      EigenSupport::ConstRefVecXd q, EigenSupport::ConstRefVecXd dq) const override;

    const EigenSupport::VXd &restDofs() const { return restDofs_; }
    const EigenSupport::VXd &plasticDesignDofs() const { return plasticDesignDofs_; }

  private:
    EigenSupport::VXd absoluteDofs(EigenSupport::ConstRefVecXd q) const;

    std::shared_ptr<const TricubicHermiteAssembler> assembler_;
    EigenSupport::VXd restDofs_;
    EigenSupport::VXd plasticDesignDofs_;
    EigenSupport::VXd elasticParams_;
    std::vector<int> dofs_;
    bool enableMaterialMaxStep_ = true;
  };
  }  // namespace SolidDeformationModel
  }  // namespace pgo
  ```

- [ ] **Step 3: Implement wrapper**

  The wrapper must add rest DOFs before calling the assembler:

  ```cpp
  EigenSupport::VXd TricubicHermiteDeformationEnergy::absoluteDofs(EigenSupport::ConstRefVecXd q) const
  {
    if (q.size() != restDofs_.size())
      throw std::invalid_argument("Hermite displacement vector size does not match rest DOF size.");
    return restDofs_ + q;
  }
  ```

  `gradient(q, grad)` writes a full vector of size `q.size()`. `hessian(q, hess)` calls `assembler_->computeGeometryHessian`. `computeMaxStepLimit(q, dq)` calls `assembler_->computeMaterialMaxStepObservation(restDofs_ + q, dq)`.

- [ ] **Step 4: Run wrapper tests**

  ```bash
  cmake --build build/base_no_mkl --target tricubicHermiteSimulation_gtest -j 8
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest --gtest_filter='*PotentialEnergy*'
  ```

  Expected: rest energy is zero, displacement gradients match finite differences, and Hessian topology is fixed.

---

## Task 7: Add Mapped Surface Point Pulling For Hermite Attachments

**Files:**
- Create: `src/core/constraintPotentialEnergies/mappedPointPullingPotentialEnergy.h`
- Create: `src/core/constraintPotentialEnergies/mappedPointPullingPotentialEnergy.cpp`
- Create: `tests/src/core/constraintPotentialEnergies/mappedPointPullingPotentialEnergy_gtest.cpp`
- Modify: `src/core/constraintPotentialEnergies/CMakeLists.txt`
- Modify: `tests/src/core/constraintPotentialEnergies/CMakeLists.txt`

- [ ] **Step 1: Add mapped pulling tests**

  ```cpp
  TEST(MappedPointPullingPotentialEnergyGTest, GradientMatchesFiniteDifference)
  {
    EigenSupport::SpMatD W(6, 12);
    std::vector<EigenSupport::TripletD> triplets = {
      { 0, 0, 0.25 }, { 0, 6, 0.75 },
      { 1, 1, 0.25 }, { 1, 7, 0.75 },
      { 2, 2, 0.25 }, { 2, 8, 0.75 },
      { 3, 3, 1.0 }, { 4, 4, 1.0 }, { 5, 5, 1.0 },
    };
    W.setFromTriplets(triplets.begin(), triplets.end());

    EigenSupport::VXd surfaceRest(6);
    surfaceRest << 0, 0, 0, 1, 0, 0;
    const int selected[1] = { 0 };
    EigenSupport::VXd target(3);
    target << 0.1, -0.2, 0.3;

    pgo::ConstraintPotentialEnergies::MappedPointPullingPotentialEnergy energy(
      surfaceRest, W, 1, selected, target.data(), 10.0);

    EigenSupport::VXd q = EigenSupport::VXd::Random(12) * 0.01;
    EigenSupport::VXd grad = EigenSupport::VXd::Zero(12);
    energy.gradient(q, grad);
    for (int i = 0; i < q.size(); ++i) {
      EigenSupport::VXd qp = q;
      EigenSupport::VXd qm = q;
      qp[i] += 1e-6;
      qm[i] -= 1e-6;
      const double fd = (energy.func(qp) - energy.func(qm)) / (2e-6);
      EXPECT_NEAR(grad[i], fd, 1e-6);
    }
  }
  ```

- [ ] **Step 2: Add mapped pulling API**

  ```cpp
  class MappedPointPullingPotentialEnergy : public NonlinearOptimization::PotentialEnergy
  {
  public:
    MappedPointPullingPotentialEnergy(const EigenSupport::VXd &surfaceRestPositions,
      const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
      int numPts, const int *surfaceVertexIndices, const double *targets,
      double coeff);

    double func(EigenSupport::ConstRefVecXd q) const override;
    void gradient(EigenSupport::ConstRefVecXd q, EigenSupport::RefVecXd grad) const override;
    void hessian(EigenSupport::ConstRefVecXd q, EigenSupport::SpMatD &hess) const override;
    void createHessian(EigenSupport::SpMatD &hess) const override;
    void getDOFs(std::vector<int> &dofs) const override;
    int getNumDOFs() const override;
    NonlinearOptimization::MaxStepResult computeMaxStepLimit(
      EigenSupport::ConstRefVecXd, EigenSupport::ConstRefVecXd) const override;

    void setTargetPos(const double *targets);
    void setCoeff(double coeff);
  };
  ```

- [ ] **Step 3: Implement mapped pulling**

  Let `S` select the requested surface vertex rows from `W`. The energy is:

  ```text
  p(q) = S * (surfaceRestPositions + W * q)
  E(q) = 0.5 * coeff * ||p(q) - target||^2
  grad = coeff * W_selected^T * (p(q) - target)
  H = coeff * W_selected^T * W_selected
  ```

  `createHessian` builds the fixed sparse topology from `W_selected^T * W_selected`.

- [ ] **Step 4: Run mapped pulling tests**

  ```bash
  cmake --build build/base_no_mkl --target mappedPointPullingPotentialEnergy_gtest -j 8
  build/base_no_mkl/tests/src/core/constraintPotentialEnergies/mappedPointPullingPotentialEnergy_gtest
  ```

  Expected: finite-difference gradient passes and Hessian is symmetric.

---

## Task 8: Build Hermite Volume IPC Simulation Context

**Files:**
- Create: `src/tools/runSim/runIPCSimHermiteSetup.cpp`
- Modify: `src/tools/runSim/runIPCSimHermiteSetup.h`
- Modify: `src/tools/runSim/runIPCSimSetup.h`
- Modify: `src/tools/runSim/runIPCSimSetup.cpp`
- Modify: `src/tools/runSim/CMakeLists.txt`
- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: Add context-level tests**

  Add a tiny one-cube config test:

	  ```text
	  buildHermiteVolumeIpcSimulation returns:
	    simulationRestPosition.size == restDofs.size
	    M.rows == restDofs.size
	    surfaceFromSimulationDispMap.cols == restDofs.size
	    elasticEnergy->getNumDOFs() == restDofs.size
	    initialVelocity has init-vel only on value DOFs
	    bodyAcceleration has g only on value DOFs
	  ```

- [ ] **Step 2: Generalize `IpcSimulationContext` energy and initial fields**

	  Add a forward declaration for the mapped pulling class in `runIPCSimSetup.h`:

	  ```cpp
	  namespace ConstraintPotentialEnergies
	  {
	  class MultipleVertexPulling;
	  class MappedPointPullingPotentialEnergy;
	  }
	  ```

	  Change:

	  ```cpp
	  std::shared_ptr<SolidDeformationModel::DeformationModelEnergy> elasticEnergy;
	  ```

	  to:

	  ```cpp
	  std::shared_ptr<NonlinearOptimization::PotentialEnergy> elasticEnergy;
	  EigenSupport::VXd initialVelocity;
	  EigenSupport::VXd bodyAcceleration;
	  std::shared_ptr<RunIPCSim::VonMisesStressEvaluator> vonMisesStressEvaluator;
	  std::vector<std::shared_ptr<ConstraintPotentialEnergies::MappedPointPullingPotentialEnergy>> mappedPullingEnergies;
	  std::vector<EigenSupport::VXd> mappedPullingTargets;
	  std::vector<EigenSupport::VXd> mappedPullingTargetRests;
	  ```

	  Keep the existing `pullingEnergies`, `pullingTargets`, and `pullingTargetRests` fields for vertex/shell simulations. Do not store `MappedPointPullingPotentialEnergy` in `pullingEnergies`; that vector is still `std::vector<std::shared_ptr<MultipleVertexPulling>>` and the main loop calls `MultipleVertexPulling::setTargetPos` on it.

	  Add:

  ```cpp
  class VonMisesStressEvaluator
  {
  public:
    virtual ~VonMisesStressEvaluator() = default;
    virtual int getNumElements() const = 0;
    virtual const char *getLocationName() const = 0;
    virtual void compute(EigenSupport::ConstRefVecXd displacement, double *elementStresses) const = 0;
  };
  ```

	  Existing vertex path can either leave `vonMisesStressEvaluator` null and keep the old assembler branch, or add a small adapter around `DeformationModelAssembler`.

	  Also update the local helper in `src/tools/runSim/runIPCSim.cpp` from:

	  ```cpp
	  void logRunIPCSimMaxStepSummary(
	    const std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> &elasticEnergy,
	    const std::shared_ptr<pgo::Contact::CIPC::EmbeddedSurfaceIPCPotentialEnergy> &collisionHandler,
	    const std::shared_ptr<pgo::Simulation::ImplicitBackwardEulerTimeIntegrator> &integrator)
	  ```

	  to:

	  ```cpp
	  void logRunIPCSimMaxStepSummary(
	    const std::shared_ptr<const pgo::NonlinearOptimization::PotentialEnergy> &elasticEnergy,
	    const std::shared_ptr<pgo::Contact::CIPC::EmbeddedSurfaceIPCPotentialEnergy> &collisionHandler,
	    const std::shared_ptr<pgo::Simulation::ImplicitBackwardEulerTimeIntegrator> &integrator)
	  ```

	  The current body only marks `elasticEnergy` unused, but the signature must accept `context.elasticEnergy` after the context field becomes `std::shared_ptr<PotentialEnergy>`.

- [ ] **Step 3: Implement Hermite setup builder**

  Add:

  ```cpp
  IpcSimulationContext buildHermiteVolumeIpcSimulation(
    const pgo::ConfigFileJSON &jconfig,
    const HermiteSimulationConfig &hermiteConfig);
  ```

  Builder steps:

	  ```text
	  load cubic mesh with scale
	  load surface mesh with scale
	  compute BarycentricCoordinates from surface rest positions to cubic mesh
	  build TricubicHermiteCubicMeshAdapter
	  set context.simulationRestPosition = adapter.restDofs
	  build W_surface_from_hermite
	  build Hermite mass matrix
	  build plastic design field from hermite.plastic-field
	  build one TricubicHermiteDeformationModel per cubic element
	  build TricubicHermiteAssembler
	  build TricubicHermiteDeformationEnergy
	  build mapped fixed-vertex pulling energies into context.mappedPullingEnergies
	  build IPC and floor energies with W_surface_from_hermite
	  build surface pressure force by W_surface_from_hermite^T * surfaceForce
	  fill initialVelocity and bodyAcceleration using value DOFs only
	  ```

	  The Hermite setup must not call `RunSim::initializeVolumetricSimulation`; that helper creates vertex-only `SimulationMesh`, vertex-only mass, and vertex-only `DeformationModelAssembler`.

	  `context.simulationRestPosition` is mandatory, not diagnostic-only. `runIPCSim.cpp` uses `context.simulationRestPosition.size()` as the simulation state dimension for restart matrices, state output, and integrator vectors. The Hermite builder must assign:

	  ```cpp
	  context.simulationRestPosition = adapter.restDofs;
	  ```

	  For Hermite fixed constraints, push target animation data into `context.mappedPullingTargets` and `context.mappedPullingTargetRests`. Do not push mapped energies into `context.extraGeneralImplicitForceModels`, because animated targets must be updated each frame before the energy is added to the integrator.

- [ ] **Step 4: Build plastic design field from config**

  Constant:

	  ```text
	  z.size = 6
	  z = hermite.plastic-field.params
	  numQuadraturePointsPerElement = quadratureOrder^3
	  plasticField = ConstantPlasticDesignField(numElements, numQuadraturePointsPerElement)
	  ```

	  `ConstantPlasticDesignField` takes the per-element quadrature-point count as its second constructor argument, not the global count. Add a setup test assertion:

	  ```cpp
	  EXPECT_EQ(plasticField->getNumQuadraturePointsPerElement(), numQuadraturePointsPerElement);
	  EXPECT_EQ(plasticField->getNumElements(), numElements);
	  ```

	  Sparse kernel:

	  ```text
	  z.size = 6 * numControlPoints
	  control positions are config rest-space positions
	  numPlasticSamples = numElements * numQuadraturePointsPerElement
	  allQuadraturePositions stores every element/qp rest position in row-major order:
	    allQuadraturePositions[(ele * numQuadraturePointsPerElement + qp) * 3 + coord]
	  influences = buildSparseKernelInfluences(allQuadraturePositions.data(), numPlasticSamples, controlPoints, numControlPoints, radius)
	  assert influences.size() == numPlasticSamples
	  plasticField = SparseKernelPlasticDesignField(numElements, numQuadraturePointsPerElement, numControlPoints, influences)
	  ```

	  This must match the FEM plan contract exactly: `buildSparseKernelInfluences` takes the total number of quadrature samples across all elements, while `SparseKernelPlasticDesignField` maps the returned flat row table back with `ele * numQuadraturePointsPerElement + qp`.

- [ ] **Step 5: Implement Hermite initial velocity and body acceleration**

  Add helper:

  ```cpp
  EigenSupport::VXd makeHermiteConstantTranslationField(
    const TricubicHermiteDofLayout &layout,
    const EigenSupport::V3d &value)
  {
    EigenSupport::VXd field = EigenSupport::VXd::Zero(layout.getNumGlobalGeometryDofs());
    for (int node = 0; node < layout.getNumNodes(); ++node) {
      field.segment<3>(node * 24) = value;
    }
    return field;
  }
  ```

  This helper intentionally writes only type-0 value DOFs. Derivative DOF entries remain zero.

- [ ] **Step 6: Route `buildVolumeIpcSimulation`**

  In `buildVolumeIpcSimulation`:

  ```cpp
  HermiteSimulationConfig hermiteConfig;
  if (tryParseHermiteSimulationConfig(jconfig, hermiteConfig)) {
    return buildHermiteVolumeIpcSimulation(jconfig, hermiteConfig);
  }
  ```

	  Keep the existing vertex path below this branch unchanged.

- [ ] **Step 7: Populate old vertex and shell context fields**

	  Because Task 9 removes vertex-count initialization from `runIPCSim.cpp`, both old setup paths must populate the new context fields before returning. Add a helper in `runIPCSimSetup.cpp`:

	  ```cpp
	  static EigenSupport::VXd makeRepeatedTripletField(int n3, const EigenSupport::V3d &value)
	  {
	    if (n3 % 3 != 0)
	      throw std::invalid_argument("Vertex simulation state size must be divisible by 3.");
	    EigenSupport::VXd field(n3);
	    for (int i = 0; i < n3 / 3; ++i)
	      field.segment<3>(i * 3) = value;
	    return field;
	  }
	  ```

	  At the end of both `buildVolumeIpcSimulation`'s old vertex branch and `buildShellIpcSimulation`, after `context.simulationRestPosition` is assigned, add:

	  ```cpp
	  const EigenSupport::V3d extAcc =
	    EigenSupport::Mp<EigenSupport::V3d>(jconfig.getValue<std::array<double, 3>>("g", 1).data());
	  const EigenSupport::V3d initialVel =
	    EigenSupport::Mp<EigenSupport::V3d>(jconfig.getValue<std::array<double, 3>>("init-vel", 1).data());
	  const int n3 = static_cast<int>(context.simulationRestPosition.size());
	  context.bodyAcceleration = makeRepeatedTripletField(n3, extAcc);
	  context.initialVelocity = makeRepeatedTripletField(n3, initialVel);
	  ```

	  This preserves old tet/cubic/shell behavior while letting Hermite setup provide value-DOF-only fields.

- [ ] **Step 8: Run setup tests**

  ```bash
  cmake --build build/base_no_mkl --target runIPCSim_gtest tricubicHermiteSimulation_gtest -j 8
  build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='*Hermite*'
  build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest
  ```

  Expected: Hermite setup context dimensions are consistent and old volume setup tests still pass.

---

## Task 9: Update `runIPCSim.cpp` For Non-Vertex Simulation State

**Files:**
- Modify: `src/tools/runSim/runIPCSim.cpp`
- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: Add state initialization regression test**

  Add a test that fails if `runIPCSim` repeats `init-vel` or `g` over every Hermite DOF triple.

  ```text
  Hermite context with 2 nodes:
    initialVelocity[type0] = init-vel
    initialVelocity[type1..7] = 0
    bodyAcceleration[type0] = g
    bodyAcceleration[type1..7] = 0
  ```

- [ ] **Step 2: Replace vertex-count initialization**

	  Remove the main-loop parsing of `extAcc` and `initialVel` after setup owns these fields, and remove `const int n = n3 / 3;`. After this change, `n3` is a generic simulation-state dimension and must not be interpreted as a vertex count.

	  Replace:

	  ```cpp
	  ES::VXd g(n3);
	  for (int vi = 0; vi < n; ++vi)
	    g.segment<3>(vi * 3) = extAcc;

  ES::VXd gravityForce(n3);
  ES::mv(context.M, g, gravityForce);
  ES::VXd fext = gravityForce;

  ES::VXd u = ES::VXd::Zero(n3);
	  ES::VXd uvel = ES::VXd::Zero(n3);
	  for (int i = 0; i < n; ++i)
	    uvel.segment<3>(i * 3) = initialVel;
	  ```

  with:

  ```cpp
  if (context.bodyAcceleration.size() != n3)
    throw std::runtime_error("runIPCSim context bodyAcceleration size does not match simulation state size.");
  if (context.initialVelocity.size() != n3)
    throw std::runtime_error("runIPCSim context initialVelocity size does not match simulation state size.");

  ES::VXd gravityForce(n3);
  ES::mv(context.M, context.bodyAcceleration, gravityForce);
	  ES::VXd fext = gravityForce;

	  ES::VXd u = ES::VXd::Zero(n3);
	  ES::VXd uvel = context.initialVelocity;
	  ES::VXd uacc = ES::VXd::Zero(n3);
	  ```

	  The old vertex and shell paths must fill `context.bodyAcceleration` and `context.initialVelocity` with repeated vertex vectors during setup, as specified in Task 8 Step 7.

- [ ] **Step 3: Add mapped pulling dispatch**

	  Keep the existing setup-time registration for `context.pullingEnergies`:

	  ```cpp
	  for (auto &pullingEnergy : context.pullingEnergies)
	    intg->addImplicitForceModel(pullingEnergy, 0, 0);
	  ```

	  Do not register `context.mappedPullingEnergies` there. They are general force models and are cleared by `intg->clearGeneralImplicitForceModel()` each frame. After the existing per-frame `context.pullingEnergies[pi]->setTargetPos(...)` loop and before adding collision/floor/general force models, add:

	  ```cpp
	  for (std::size_t pi = 0; pi < context.mappedPullingEnergies.size(); ++pi) {
	    const ES::VXd curTgt =
	      context.mappedPullingTargetRests[pi] * (1.0 - ratio) + context.mappedPullingTargets[pi] * ratio;
	    context.mappedPullingEnergies[pi]->setTargetPos(curTgt.data());
	    intg->addGeneralImplicitForceModel(context.mappedPullingEnergies[pi], 0, 0);
	    std::cout << "Frame " << framei << ", mapped attachment " << pi
	              << " target: " << curTgt.transpose().head(3) << std::endl;
	  }
	  ```

	  Add size checks before the time loop:

	  ```cpp
	  if (context.pullingTargets.size() != context.pullingEnergies.size() ||
	      context.pullingTargetRests.size() != context.pullingEnergies.size())
	    throw std::runtime_error("Vertex pulling target arrays do not match pulling energy count.");
	  if (context.mappedPullingTargets.size() != context.mappedPullingEnergies.size() ||
	      context.mappedPullingTargetRests.size() != context.mappedPullingEnergies.size())
	    throw std::runtime_error("Mapped pulling target arrays do not match mapped pulling energy count.");
	  ```

- [ ] **Step 4: Update stress output branch**

  In `writeVonMisesStressJson`, first use `context.vonMisesStressEvaluator` if present:

  ```cpp
  if (context.vonMisesStressEvaluator) {
    std::vector<double> elementStresses(context.vonMisesStressEvaluator->getNumElements(), 0.0);
    context.vonMisesStressEvaluator->compute(displacement, elementStresses.data());
    stressJson["frame"] = frame;
    stressJson["time"] = static_cast<double>(frame) * timestep;
    stressJson["stress_type"] = "von_mises";
    stressJson["location"] = context.vonMisesStressEvaluator->getLocationName();
    stressJson["values"] = elementStresses;
    const std::filesystem::path outputPath = framePath(outputDirs.stress, "von_mises", frame, ".json");
    std::ofstream out(outputPath);
    if (!out.is_open())
      throw std::runtime_error("Failed to write von Mises stress JSON: " + outputPath.string());
    out << stressJson.dump(2) << '\n';
    return;
  }
  ```

  Keep the existing `DeformationModelAssembler` branch for vertex tet/cubic simulations.

- [ ] **Step 5: Verify restart and surface output still use generic dimensions**

  Confirm these existing operations remain valid:

  ```text
  deformXXXX.u stores [u, uvel, uacc] with rows = context.simulationRestPosition.size()
  surface output computes usurf = W_surface_from_simulation * u
  pressure force uses W_surface_from_simulation^T * surfaceForce
  ```

- [ ] **Step 6: Run runIPCSim tests**

  ```bash
  cmake --build build/base_no_mkl --target runIPCSim_gtest -j 8
  build/base_no_mkl/tests/src/tools/runIPCSim_gtest
  ```

  Expected: existing tests pass and Hermite initialization tests catch value-vs-derivative DOF mistakes.

---

## Task 10: Add Hermite IPC Example And Smoke Test

**Files:**
- Create: `examples/ipc/hermite/box-drop/box-hermite-ipc.json`
- Copy the known-good cubic box assets from `examples/ipc/cubic/box/` into `examples/ipc/hermite/box-drop/`
- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: Add example config**

  Example:

  ```json
  {
    "simulation-discretization": "tricubic-hermite",
    "cubic-mesh": "box.veg",
    "surface-mesh": "box.obj",
    "fixed-vertices": [],
    "external-objects": [
      {
        "filename": "bottom.obj",
        "movement": [0, 0, 0]
      }
    ],
    "g": [0, -9.81, 0],
    "init-vel": [0, 0, 0],
    "init-disp": [0, 0, 0],
    "scale": 1.0,
    "timestep": 0.001,
    "num-timestep": 5,
    "damping-params": [0, 0],
    "sim-type": "dynamic",
    "solver-eps": 1e-4,
    "solver-max-iter": 50,
    "elastic-material": "stable-neo",
    "dump-interval": 1,
    "output": "ret-box-hermite-ipc",
    "ipc-dhat": 0.002,
    "ipc-kappa": 3000.0,
    "ipc-dhat-external": 0.005,
    "enable-material-max-step": true,
    "output-von-mises": true,
    "hermite": {
      "quadrature-order": 4,
      "mass-quadrature-order": 4,
      "fixed-vertices-space": "surface",
      "plastic-field": {
        "type": "constant",
        "params": [1, 0, 0, 1, 0, 1]
      }
    }
  }
	  ```

- [ ] **Step 2: Copy smoke-test mesh assets**

	  Use the existing tiny cubic IPC box assets. Do not invent new mesh formats for this smoke test.

	  ```bash
	  mkdir -p examples/ipc/hermite/box-drop
	  cp examples/ipc/cubic/box/box.veg examples/ipc/hermite/box-drop/box.veg
	  cp examples/ipc/cubic/box/box.obj examples/ipc/hermite/box-drop/box.obj
	  cp examples/ipc/cubic/box/bottom.obj examples/ipc/hermite/box-drop/bottom.obj
	  ```

	  These files provide a cubic volume mesh, its matching surface mesh, and the static floor obstacle used by the existing cubic IPC smoke path.

- [ ] **Step 3: Add smoke test**

  The smoke test should run a tiny config for 1 to 5 timesteps and assert:

  ```text
  runIPCSim exits with code 0
  states/deform0000.u exists
  surface/ret0000.obj exists
  stress/von_mises0000.json exists when output-von-mises=true
  log contains `tricubic-hermite`
  no NaN/Inf appears in output state
  ```

- [ ] **Step 4: Run smoke test manually**

  ```bash
  cmake --build build/base_no_mkl --target runIPCSim -j 8
  build/base_no_mkl/src/tools/runSim/runIPCSim examples/ipc/hermite/box-drop/box-hermite-ipc.json --log
  ```

  Expected: command returns 0, produces surface OBJ frames, writes Hermite-sized `.u` state files, and reports finite solver diagnostics.

---

## Task 11: Add Sparse-Kernel Plastic Field Simulation Smoke Test

**Files:**
- Create: `examples/ipc/hermite/box-drop/box-hermite-sparse-plastic-ipc.json`
- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: Add sparse plastic example config**

  Use the same mesh assets as Task 10, but change the Hermite block:

  ```json
  "hermite": {
    "quadrature-order": 4,
    "mass-quadrature-order": 4,
    "fixed-vertices-space": "surface",
    "plastic-field": {
      "type": "sparse-kernel",
      "radius": 2.0,
      "control-points": [
        { "position": [0.0, 0.0, 0.0], "params": [1, 0, 0, 1, 0, 1] },
        { "position": [1.0, 0.0, 0.0], "params": [1.02, 0, 0, 1, 0, 1] }
      ]
    }
  }
  ```

- [ ] **Step 2: Add sparse smoke assertions**

  The test should assert:

  ```text
  parsed plastic field type is sparse-kernel
  design DOF count = 6 * numControlPoints
  each quadrature row has normalized kernel weights
  runIPCSim exits with code 0 for 1 to 5 timesteps
  ```

- [ ] **Step 3: Run sparse smoke test**

  ```bash
  cmake --build build/base_no_mkl --target runIPCSim_gtest -j 8
  build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='*Hermite*Sparse*'
  ```

  Expected: sparse kernel simulation path uses the same verified assembler and runs without changing IPC code.

---

## Task 12: Full Verification Matrix

Run this after all tasks:

```bash
cmake --build build/base_no_mkl \
  --target tricubicHermiteBasis_gtest plasticDesignField_gtest tricubicHermiteDeformationModel_gtest tricubicHermiteAssembler_gtest tricubicHermiteSimulation_gtest mappedPointPullingPotentialEnergy_gtest runIPCSim_gtest runIPCSim \
  -j 8

build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteBasis_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/plasticDesignField_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteDeformationModel_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteAssembler_gtest
build/base_no_mkl/tests/src/core/solidDeformationModel/tricubicHermiteSimulation_gtest
build/base_no_mkl/tests/src/core/constraintPotentialEnergies/mappedPointPullingPotentialEnergy_gtest
build/base_no_mkl/tests/src/tools/runIPCSim_gtest

build/base_no_mkl/src/tools/runSim/runIPCSim examples/ipc/hermite/box-drop/box-hermite-ipc.json --log
build/base_no_mkl/src/tools/runSim/runIPCSim examples/ipc/hermite/box-drop/box-hermite-sparse-plastic-ipc.json --log
```

Expected final behavior:

- Existing tet/cubic/shell `runIPCSim` configs still take the vertex path and remain bitwise-compatible except for extra context fields.
- Hermite mode uses `numNodes * 24` simulation DOFs.
- `surfaceFromSimulationDispMap.cols()` equals Hermite DOF count.
- Gravity and initial velocity affect only value DOFs for constant translation fields.
- IPC contact, floor contact, and surface pressure operate through the Hermite surface map.
- Fixed constraints in Hermite mode operate on mapped surface vertices.
- State restart reads and writes Hermite-sized `.u` matrices.
- Surface OBJ output remains surface-vertex sized.
- Constant `Fp` and sparse-kernel `Fp` fields both run through the same Hermite assembler.
- `output-von-mises=true` works through the Hermite stress evaluator.

---

## Self-Review

- Spec coverage: this plan covers full `runIPCSim` integration after finite-element verification: config parsing, cubic-to-Hermite rest DOFs, surface embedding, mass matrix, Hessian/max-step/stress, `PotentialEnergy` wrapping, mapped attachments, IPC context creation, main-loop state initialization, restart/output, constant plastic field, and sparse-kernel plastic field.
- Critical correctness gates: the plan explicitly prevents the two main simulation bugs for Hermite DOFs: treating every 3-vector as a vertex for gravity/initial velocity, and using vertex-only `DeformationModelAssembler` assumptions for Hermite geometry.
- Placeholder scan: no task relies on "do something similar" or unspecified validation. Each new API has concrete contracts, test expectations, and commands.
- Scope control: inverse design over `Fp`, neural plastic fields, tet Hermite, unstructured hex Hermite frames, and changing IPC barrier formulas are intentionally out of scope.
