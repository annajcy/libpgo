# Contact Internal Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 contact 模块重构为直接组合 `SurfaceDofMap` 的 simulation-space contact energy，保持 sampled penalty 行为贴近旧版，并把 IPC 拆成 pair generation、assembly、energy lifecycle 三层。

**Architecture:** 第一阶段先迁移公共 DOF mapping 与 concrete energies，避免在旧 mapped wrapper 还存在时引入新的 `StatefulContactEnergy + StepAwareEnergy` 继承造成菱形或重复 base。第二阶段收敛 sampled penalty 的 builder/bundle/evaluator 与 Python API。第三阶段将 IPC 的 pair generation、assembly、cache lifecycle 从 `SurfaceIPCCore` 中拆出，并让 `IPCContactEnergy` 自己组合 `SurfaceDofMap`。

**Tech Stack:** C++17, Eigen sparse matrices, CMake/CTest, GoogleTest, nanobind, Python pytest.

---

## 执行规则

这份计划适合 subagent-driven 串行执行。不要并行派多个 implementer：多个任务会修改 `src/core/contact/CMakeLists.txt`、factory、Python binding 和 contact tests，并行执行会产生冲突。

每个 task 的 implementer 需要：

- 只修改本 task 列出的文件。
- 先写或更新测试，再改实现。
- 跑本 task 的 verification 命令。
- 不自动提交 git commit；controller review 通过后再由用户决定是否提交。

每个 task 完成后需要两个 review：

- spec compliance review：确认 task 的验收点都满足，且没有引入 `ContactTable` 或向后兼容 facade。
- code quality review：确认继承关系清晰、没有重复 `PotentialEnergy`/`StepAwareEnergy` base、没有跨 task 重构。

## 文件结构目标

### 公共层

- `src/core/contact/surfaceDofMap.h`
- `src/core/contact/surfaceDofMap.cpp`

  从当前 `EmbeddedDofMap` 升级而来，负责 surface displacement/position mapping 和 gradient/Hessian pullback。

- `src/core/contact/statefulContactEnergy.h`

  最终只保留 contact 的公共 long-lived energy 边界：`StatefulContactEnergy : PotentialEnergy, StepAwareEnergy`，默认 `beginStep()` no-op，提供 `isStepDependent()`。

### floor

- `src/core/contact/floor/floorContactEnergy.h`
- `src/core/contact/floor/floorContactEnergy.cpp`

  直接继承 `StatefulContactEnergy`，组合 `SurfaceDofMap`，不再继承 `MappedSurfacePotentialEnergy`。

### sampled penalty

- `src/core/contact/sampled_penalty/sampledPenaltyEvaluationBundle.h`
- `src/core/contact/sampled_penalty/sampledPenaltyEvaluationBundle.cpp`

  当前 `SampledPenaltyActiveSet` 的语义重命名；保存 external/self child energies 和 buffers。

- `src/core/contact/sampled_penalty/sampledPenaltyContactBuilder.h`
- `src/core/contact/sampled_penalty/sampledPenaltyContactBuilder.cpp`

  当前 `SampledPenaltyContactDetector` 的语义收窄；输入 absolute surface positions，执行 handler detection 并构造 bundle。

- `src/core/contact/sampled_penalty/sampledPenaltyContactEvaluator.h`
- `src/core/contact/sampled_penalty/sampledPenaltyContactEvaluator.cpp`

  消费 bundle，计算 surface-space value/gradient/Hessian，不执行 detection。

- `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`
- `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp`

  public simulation-space `SampledPenaltyContactEnergy`，组合 `SurfaceDofMap`、builder、evaluator 和 optional friction state。

删除或停用：

- `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.h`
- `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.cpp`
- `SampledPenaltySurfaceContactEnergy`
- `FrictionalSampledPenaltySurfaceContactEnergy`

### IPC

- `src/core/contact/ipc/ipcPairGenerator.h`
- `src/core/contact/ipc/ipcPairGenerator.cpp`

  拥有 IPC topology、obstacle surfaces、moving-obstacle state、active pair generation、line-search superset 和 CCD max-step。

- `src/core/contact/ipc/ipcContactAssembler.h`
- `src/core/contact/ipc/ipcContactAssembler.cpp`

  对固定 `SurfaceIPCActiveSet` 计算 normal barrier value/gradient/Hessian。

- `src/core/contact/ipc/ipcContactEnergy.h`
- `src/core/contact/ipc/ipcContactEnergy.cpp`

  public simulation-space `IPCContactEnergy`，组合 `SurfaceDofMap`、`IPCPairGenerator`、`IPCContactAssembler`、`IPCActiveSetCache`，只额外继承 `LineSearchAwareEnergy`。

删除或停用：

- `src/core/contact/ipc/core/surfaceIPCCore.h`
- `src/core/contact/ipc/core/surfaceIPCCore.cpp`

### Python/API

- `src/python/pypgo/contact/core.h`
- `src/python/pypgo/contact/core.cpp`
- `src/python/pypgo/contact/bindings.cpp`
- `pypgo/contact/__init__.py`
- `pypgo/contact/energies.py`
- `pypgo/tools/sim/_config.py`
- `pypgo/tools/sim/_scene.py`

统一 sampled penalty Python API 为 `SampledPenaltyEnergy(surface, surface_triangles, params=params, friction=None)` 或 `SampledPenaltyEnergy(surface, surface_triangles, params=params, friction=FrictionParameters(friction_coeff=0.4, velocity_eps=1e-4))`。

## Task 1: 引入 `SurfaceDofMap`

**Files:**

- Rename: `src/core/contact/embeddedDofMap.h` -> `src/core/contact/surfaceDofMap.h`
- Rename: `src/core/contact/embeddedDofMap.cpp` -> `src/core/contact/surfaceDofMap.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `src/core/contact/contactEnergyFactory.h`
- Modify: `tests/src/core/contact/CMakeLists.txt`
- Create: `tests/src/core/contact/surfaceDofMap_gtest.cpp`
- Modify: `tests/src/core/contact/contactEnergyFactory_gtest.cpp`

- [ ] **Step 1: Rename the files**

Run:

```bash
git mv src/core/contact/embeddedDofMap.h src/core/contact/surfaceDofMap.h
git mv src/core/contact/embeddedDofMap.cpp src/core/contact/surfaceDofMap.cpp
```

- [ ] **Step 2: Write `SurfaceDofMap` tests**

Create `tests/src/core/contact/surfaceDofMap_gtest.cpp` with these test cases:

```cpp
#include <gtest/gtest.h>

#include "surfaceDofMap.h"

#include <stdexcept>
#include <vector>

namespace
{
namespace ES = pgo::EigenSupport;
namespace Contact = pgo::Contact;

ES::MXd makeRestVertices()
{
  ES::MXd V(2, 3);
  V << 1.0, 2.0, 3.0,
       4.0, 5.0, 6.0;
  return V;
}

ES::SpMatD makeSurfaceMap()
{
  std::vector<ES::TripletD> entries;
  entries.emplace_back(0, 0, 1.0);
  entries.emplace_back(1, 1, 1.0);
  entries.emplace_back(2, 2, 1.0);
  entries.emplace_back(3, 3, 2.0);
  entries.emplace_back(4, 4, 3.0);
  entries.emplace_back(5, 5, 4.0);
  entries.emplace_back(0, 6, 0.5);
  ES::SpMatD W(6, 7);
  W.setFromTriplets(entries.begin(), entries.end());
  return W;
}
}  // namespace

TEST(SurfaceDofMapGTest, MapsDisplacementsAndPositions)
{
  Contact::SurfaceDofMap map(makeRestVertices(), makeSurfaceMap());
  ES::VXd u = ES::VXd::Zero(7);
  u << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 2.0;

  const ES::VXd du = map.surfaceDisplacements(u);
  const ES::VXd x = map.surfacePositions(u);

  EXPECT_EQ(map.numSimulationDofs(), 7);
  EXPECT_EQ(map.numSurfaceDofs(), 6);
  EXPECT_DOUBLE_EQ(du[0], 1.1);
  EXPECT_DOUBLE_EQ(du[3], 0.8);
  EXPECT_DOUBLE_EQ(du[4], 1.5);
  EXPECT_DOUBLE_EQ(du[5], 2.4);
  EXPECT_DOUBLE_EQ(x[0], 2.1);
  EXPECT_DOUBLE_EQ(x[5], 8.4);
}

TEST(SurfaceDofMapGTest, PullsBackGradientAndHessian)
{
  const ES::SpMatD W = makeSurfaceMap();
  Contact::SurfaceDofMap map(makeRestVertices(), W);

  ES::VXd gs(6);
  gs << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  const ES::VXd expectedGradient = W.transpose() * gs;
  const ES::VXd actualGradient = map.pullbackGradient(gs);

  EXPECT_TRUE(actualGradient.isApprox(expectedGradient));

  std::vector<ES::TripletD> hEntries;
  for (int i = 0; i < 6; ++i)
    hEntries.emplace_back(i, i, static_cast<double>(i + 1));
  ES::SpMatD Hs(6, 6);
  Hs.setFromTriplets(hEntries.begin(), hEntries.end());

  ES::SpMatD expectedHessian = W.transpose() * Hs * W;
  ES::SpMatD actualHessian;
  map.pullbackHessian(Hs, actualHessian);
  EXPECT_TRUE(actualHessian.isApprox(expectedHessian));
}

TEST(SurfaceDofMapGTest, RejectsInvalidShapes)
{
  ES::MXd badVertices(2, 2);
  badVertices.setZero();
  EXPECT_THROW(Contact::SurfaceDofMap(badVertices, makeSurfaceMap()), std::invalid_argument);

  ES::SpMatD badRows(5, 7);
  EXPECT_THROW(Contact::SurfaceDofMap(makeRestVertices(), badRows), std::invalid_argument);

  Contact::SurfaceDofMap map(makeRestVertices(), makeSurfaceMap());
  ES::VXd badU = ES::VXd::Zero(6);
  EXPECT_THROW((void)map.surfacePositions(badU), std::invalid_argument);
}
```

- [ ] **Step 3: Implement `SurfaceDofMap`**

In `src/core/contact/surfaceDofMap.h`, define:

```cpp
class SurfaceDofMap
{
public:
  SurfaceDofMap(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap);

  int numSimulationDofs() const { return static_cast<int>(simulationDofs_.size()); }
  int numSurfaceDofs() const { return static_cast<int>(surfaceRestPositions_.size()); }
  const std::vector<int> &simulationDofs() const { return simulationDofs_; }

  EigenSupport::VXd surfaceDisplacements(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  EigenSupport::VXd surfacePositions(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  EigenSupport::VXd pullbackGradient(EigenSupport::ConstRefVecXd surfaceGradient) const;
  void pullbackHessian(const EigenSupport::SpMatD &surfaceHessian, EigenSupport::SpMatD &simulationHessian) const;

private:
  void validateSimulationDisplacementSize(EigenSupport::ConstRefVecXd simulationDisplacements) const;
  void validateSurfaceVectorSize(EigenSupport::ConstRefVecXd surfaceVector) const;

  EigenSupport::VXd surfaceRestPositions_;
  EigenSupport::SpMatD surfaceFromSimulationDispMap_;
  std::vector<int> simulationDofs_;
};
```

In `src/core/contact/surfaceDofMap.cpp`, keep the current constructor and mapping logic, rename error messages from `EmbeddedDofMap` to `SurfaceDofMap`, and add:

```cpp
void SurfaceDofMap::validateSurfaceVectorSize(EigenSupport::ConstRefVecXd surfaceVector) const
{
  if (surfaceVector.size() != surfaceFromSimulationDispMap_.rows())
    throw std::invalid_argument("SurfaceDofMap surface vector size does not match surface map row count.");
}

EigenSupport::VXd SurfaceDofMap::pullbackGradient(EigenSupport::ConstRefVecXd surfaceGradient) const
{
  validateSurfaceVectorSize(surfaceGradient);
  return surfaceFromSimulationDispMap_.transpose() * surfaceGradient;
}

void SurfaceDofMap::pullbackHessian(const EigenSupport::SpMatD &surfaceHessian, EigenSupport::SpMatD &simulationHessian) const
{
  if (surfaceHessian.rows() != surfaceFromSimulationDispMap_.rows() ||
      surfaceHessian.cols() != surfaceFromSimulationDispMap_.rows())
    throw std::invalid_argument("SurfaceDofMap surface Hessian shape does not match surface map row count.");
  simulationHessian = surfaceFromSimulationDispMap_.transpose() * surfaceHessian * surfaceFromSimulationDispMap_;
}
```

- [ ] **Step 4: Update build files and includes**

In `src/core/contact/CMakeLists.txt`, replace `embeddedDofMap.h/.cpp` with `surfaceDofMap.h/.cpp`.

In `src/core/contact/contactEnergyFactory.h`, replace:

```cpp
#include "embeddedDofMap.h"
```

with:

```cpp
#include "surfaceDofMap.h"
```

In `tests/src/core/contact/CMakeLists.txt`, add:

```cmake
add_executable(surfaceDofMap_gtest surfaceDofMap_gtest.cpp)
target_link_libraries(surfaceDofMap_gtest PRIVATE GTest::gtest_main contact)
set_property(TARGET surfaceDofMap_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(surfaceDofMap_gtest)
```

In `tests/src/core/contact/contactEnergyFactory_gtest.cpp`, update the old `EmbeddedDofMapMapsSimulationDisplacementsToSurfacePositions` test to include `surfaceDofMap.h` and instantiate `Contact::SurfaceDofMap`.

- [ ] **Step 5: Verify**

Run:

```bash
cmake --build build --target surfaceDofMap_gtest contactEnergyFactory_gtest -j8
ctest --test-dir build --output-on-failure -R 'SurfaceDofMapGTest|ContactEnergyFactoryGTest'
```

Expected: all selected tests pass.

## Task 2: 将 floor 改成组合 `SurfaceDofMap`

**Files:**

- Modify: `src/core/contact/floor/floorContactEnergy.h`
- Modify: `src/core/contact/floor/floorContactEnergy.cpp`
- Modify: `tests/src/core/contact/floorContactEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/contactEnergyFactory_gtest.cpp`

- [ ] **Step 1: Add failing inheritance-boundary tests**

In `tests/src/core/contact/floorContactEnergy_gtest.cpp`, add a test that asserts floor remains a `StatefulContactEnergy` but is not line-search/evaluation aware:

```cpp
TEST(FloorContactEnergyGTest, UsesOnlyStatefulContactBoundary)
{
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd rest = flattenPositions(V);
  Contact::Floor::FloorPenaltyParameters params;
  params.floorAxis = Contact::Floor::FloorAxis::Z;
  params.floorSide = Contact::Floor::FloorSide::KEEP_ABOVE;
  params.floorHeight = 0.0;
  params.floorKappa = 1.0;

  FloorContactEnergy energy(V, makeIdentityEmbedding(rest.size()), params);

  EXPECT_NE(dynamic_cast<Contact::StatefulContactEnergy *>(&energy), nullptr);
  EXPECT_EQ(dynamic_cast<NO::LineSearchAwareEnergy *>(&energy), nullptr);
  EXPECT_EQ(dynamic_cast<NO::EvaluationStateAwareEnergy *>(&energy), nullptr);
}
```

Add includes for `statefulContactEnergy.h`, `lineSearchAwareEnergy.h`, and `evaluationStateAwareEnergy.h` if the file does not already include them. In the anonymous namespace, add:

```cpp
namespace Contact = pgo::Contact;
namespace NO = pgo::NonlinearOptimization;
```

- [ ] **Step 2: Change class inheritance and members**

In `src/core/contact/floor/floorContactEnergy.h`, replace the mapped base include with:

```cpp
#include "statefulContactEnergy.h"
#include "surfaceDofMap.h"
```

Change the class declaration to:

```cpp
class FloorContactEnergy : public StatefulContactEnergy
{
public:
  FloorContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const FloorPenaltyParameters &params);

  void setFloorHeight(double h);
  double floorHeight() const;
  ContactModelKind contactModelKind() const override { return ContactModelKind::Floor; }

  double func(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  void gradient(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  void hessian(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianInPlace(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianAlloc(EigenSupport::SpMatD &simulationHessian) const override;
  double func_grad(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  double func_grad_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void gradient_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override;
  int isHessianTopologyFixed() const override { return 0; }

private:
  double computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const;
  void computeSurfaceGradient(EigenSupport::ConstRefVecXd surfacePositions, EigenSupport::RefVecXd surfaceGradient) const;
  void computeSurfaceHessian(EigenSupport::ConstRefVecXd surfacePositions, EigenSupport::SpMatD &surfaceHessian) const;
  void computeSurfaceAll(
    EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;

  SurfaceDofMap dofMap_;
  FloorPenaltyParameters params_;
};
```

- [ ] **Step 3: Move mapping logic into floor**

In `floorContactEnergy.cpp`, construct `dofMap_` instead of `IPC::MappedSurfacePotentialEnergy`:

```cpp
FloorContactEnergy::FloorContactEnergy(
  const EigenSupport::MXd &surfaceRestVertices,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
  const FloorPenaltyParameters &params):
  dofMap_(surfaceRestVertices, surfaceFromSimulationDispMap),
  params_(params)
{
  (void)floorAxisToIndex(params_.floorAxis);
  (void)floorSideToSign(params_.floorSide);
  if (!std::isfinite(params_.floorHeight))
    throw std::invalid_argument("FloorPenaltyParameters.floorHeight must be finite.");
  if (!std::isfinite(params_.floorKappa))
    throw std::invalid_argument("FloorPenaltyParameters.floorKappa must be finite.");
}
```

Add the public evaluation methods with this flow:

```cpp
double FloorContactEnergy::func(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  return computeSurfaceEnergy(surfacePositions);
}

void FloorContactEnergy::gradient(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
  computeSurfaceGradient(surfacePositions, surfaceGradient);
  simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
}

void FloorContactEnergy::hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  EigenSupport::SpMatD surfaceHessian;
  computeSurfaceHessian(surfacePositions, surfaceHessian);
  dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
}
```

Implement fused methods by building surface positions once per public call and pulling back once. Use `computeSurfaceAll()` for `func_grad_hessian()`.

Implement DOF methods:

```cpp
void FloorContactEnergy::getDOFs(std::vector<int> &dofs) const
{
  dofs = dofMap_.simulationDofs();
}

int FloorContactEnergy::getNumDOFs() const
{
  return dofMap_.numSimulationDofs();
}
```

- [ ] **Step 4: Verify**

Run:

```bash
cmake --build build --target floorContactEnergy_gtest contactEnergyFactory_gtest -j8
ctest --test-dir build --output-on-failure -R 'FloorContactEnergyGTest|ContactEnergyFactoryGTest'
```

Expected: floor tests still pass and the new inheritance-boundary test passes.

## Task 3: 将 sampled penalty 拆成 bundle、builder、evaluator

**Files:**

- Rename: `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.h` -> `src/core/contact/sampled_penalty/sampledPenaltyEvaluationBundle.h`
- Rename: `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.cpp` -> `src/core/contact/sampled_penalty/sampledPenaltyEvaluationBundle.cpp`
- Rename: `src/core/contact/sampled_penalty/sampledPenaltyContactDetector.h` -> `src/core/contact/sampled_penalty/sampledPenaltyContactBuilder.h`
- Rename: `src/core/contact/sampled_penalty/sampledPenaltyContactDetector.cpp` -> `src/core/contact/sampled_penalty/sampledPenaltyContactBuilder.cpp`
- Create: `src/core/contact/sampled_penalty/sampledPenaltyContactEvaluator.h`
- Create: `src/core/contact/sampled_penalty/sampledPenaltyContactEvaluator.cpp`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `tests/src/core/contact/sampledPenaltyInternals_gtest.cpp`
- Modify: `tests/src/core/contact/sampledPenaltyContactEnergy_gtest.cpp`

- [ ] **Step 1: Rename active set to evaluation bundle**

Run:

```bash
git mv src/core/contact/sampled_penalty/sampledPenaltyActiveSet.h src/core/contact/sampled_penalty/sampledPenaltyEvaluationBundle.h
git mv src/core/contact/sampled_penalty/sampledPenaltyActiveSet.cpp src/core/contact/sampled_penalty/sampledPenaltyEvaluationBundle.cpp
git mv src/core/contact/sampled_penalty/sampledPenaltyContactDetector.h src/core/contact/sampled_penalty/sampledPenaltyContactBuilder.h
git mv src/core/contact/sampled_penalty/sampledPenaltyContactDetector.cpp src/core/contact/sampled_penalty/sampledPenaltyContactBuilder.cpp
```

Rename types:

```text
SampledPenaltyActiveSet -> SampledPenaltyEvaluationBundle
SampledPenaltyActiveEnergyConfigurator -> SampledPenaltyEnergyConfigurator
SampledPenaltyContactDetector -> SampledPenaltyContactBuilder
buildActiveSet(EigenSupport::ConstRefVecXd x, const SampledPenaltyEnergyConfigurator &configurator)
  -> buildFromPositions(EigenSupport::ConstRefVecXd surfacePositions, const SampledPenaltyEnergyConfigurator &configurator)
```

- [ ] **Step 2: Add evaluator API**

Create `sampledPenaltyContactEvaluator.h`:

```cpp
#pragma once

#include "EigenDef.h"

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

struct SampledPenaltyEvaluationBundle;

class SampledPenaltyContactEvaluator
{
public:
  double func(const SampledPenaltyEvaluationBundle &bundle, EigenSupport::ConstRefVecXd surfacePositions) const;
  void gradient(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient) const;
  void hessian(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::SpMatD &surfaceHessian) const;
  double func_grad(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient) const;
  double func_grad_hessian(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;
  void gradient_hessian(
    const SampledPenaltyEvaluationBundle &bundle,
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
```

Create `sampledPenaltyContactEvaluator.cpp` by moving the current child-energy accumulation code from `SampledPenaltySurfaceContactEnergy::func`, `gradient`, and `hessianInPlace`. The evaluator must not call builder or handler methods.

- [ ] **Step 3: Keep surface energy compiling during this task**

Update current `SampledPenaltySurfaceContactEnergy` to use `SampledPenaltyContactBuilder` and `SampledPenaltyContactEvaluator`, but keep its public surface-position API unchanged for this task. This gives a smaller review diff before the next task changes it into simulation-space energy.

The old `evaluationActiveSet(EigenSupport::ConstRefVecXd surfacePositions)` method should become:

```cpp
const SampledPenaltyEvaluationBundle &SampledPenaltySurfaceContactEnergy::evaluationBundle(
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  validateSurfacePositionVector(surfacePositions);
  return activeSetCache_.forEvaluation(
    surfacePositions,
    [this](EigenSupport::ConstRefVecXd state) { return buildEvaluationBundle(state); });
}
```

The old `buildActiveSet(EigenSupport::ConstRefVecXd surfacePositions)` method should become:

```cpp
std::unique_ptr<SampledPenaltyEvaluationBundle> SampledPenaltySurfaceContactEnergy::buildEvaluationBundle(
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  SampledPenaltyEnergyConfigurator configurator;
  configurator.configureExternal = [this](PointPenetrationEnergy &energy) {
    configureExternalActiveEnergy(energy);
  };
  configurator.configureSelf = [this](PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd state) {
    configureSelfActiveEnergy(energy, state);
  };
  return builder_.buildFromPositions(surfacePositions, configurator);
}
```

- [ ] **Step 4: Update CMake**

In `src/core/contact/CMakeLists.txt`, replace old sampled penalty files with:

```cmake
sampled_penalty/sampledPenaltyEvaluationBundle.h
sampled_penalty/sampledPenaltyContactBuilder.h
sampled_penalty/sampledPenaltyContactEvaluator.h
```

and:

```cmake
sampled_penalty/sampledPenaltyEvaluationBundle.cpp
sampled_penalty/sampledPenaltyContactBuilder.cpp
sampled_penalty/sampledPenaltyContactEvaluator.cpp
```

- [ ] **Step 5: Verify**

Run:

```bash
cmake --build build --target sampledPenaltyInternals_gtest sampledPenaltyContactEnergy_gtest -j8
ctest --test-dir build --output-on-failure -R 'SampledPenaltyInternalsGTest|SampledPenaltyContactEnergyGTest'
```

Expected: sampled penalty tests pass with only type/file name updates.

## Task 4: 让 sampled penalty 成为 simulation-space energy，并移除 sampled cache/line-search

**Files:**

- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyFrictionState.h`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyFrictionState.cpp`
- Delete: `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.h`
- Delete: `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.cpp`
- Modify: `src/core/contact/contactEnergyFactory.h`
- Modify: `src/core/contact/contactEnergyFactory.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `tests/src/core/contact/sampledPenaltyContactEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/sampledPenaltyInternals_gtest.cpp`
- Modify: `tests/src/core/contact/contactEnergyFactory_gtest.cpp`

- [ ] **Step 1: Add failing tests for final sampled penalty boundaries**

Update `tests/src/core/contact/contactEnergyFactory_gtest.cpp` so `CreateSampledPenaltyEnergyBuildsNormalAndFrictionalModels` asserts:

```cpp
EXPECT_EQ(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(normal.get()), nullptr);
EXPECT_EQ(dynamic_cast<const NO::LineSearchAwareEnergy *>(normal.get()), nullptr);
EXPECT_NE(dynamic_cast<NO::StepAwareEnergy *>(normal.get()), nullptr);
EXPECT_FALSE(normal->isStepDependent());

EXPECT_EQ(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(frictional.get()), nullptr);
EXPECT_EQ(dynamic_cast<const NO::LineSearchAwareEnergy *>(frictional.get()), nullptr);
EXPECT_NE(dynamic_cast<NO::StepAwareEnergy *>(frictional.get()), nullptr);
EXPECT_TRUE(frictional->isStepDependent());
```

Add a test named `SampledPenaltyFactoryUsesSimulationSpacePreviousState`:

```cpp
TEST(ContactEnergyFactoryGTest, SampledPenaltyFactoryUsesSimulationSpacePreviousState)
{
  Contact::ContactSurfaceSpec surface;
  surface.restVertices = makeTriangle();
  surface.surfaceFromSimulationDispMap = makeIdentityMap(9);
  surface.surfaceFromSimulationDispMap.conservativeResize(9, 12);
  surface.surfaceFromSimulationDispMap.coeffRef(0, 9) = 0.25;
  surface.surfaceFromSimulationDispMap.makeCompressed();

  Contact::SampledPenaltyContactSpec params;
  params.stiffness = 10.0;
  params.samples = 1;

  Contact::FrictionContactSpec friction;
  friction.frictionCoeff = 0.5;
  friction.velocityEps = 1e-5;

  auto energy = Contact::SampledPenalty::createSampledPenaltyEnergy(
    surface, makeTriangleFaces(), params, friction);
  ASSERT_NE(energy, nullptr);
  EXPECT_EQ(energy->getNumDOFs(), 12);
  EXPECT_TRUE(energy->isStepDependent());

  ES::VXd previous = ES::VXd::Zero(12);
  NO::StepState state;
  state.previousX = &previous;
  state.timestep = 0.1;
  EXPECT_NO_THROW(energy->beginStep(state));
}
```

- [ ] **Step 2: Replace the two sampled penalty classes with one class**

In `sampledPenaltyContactEnergy.h`, define:

```cpp
struct SampledPenaltyContactEnergyOptions
{
  ParametersSpec params;
  std::optional<FrictionParametersSpec> friction;
};

class SampledPenaltyContactEnergy final : public StatefulContactEnergy
{
public:
  SampledPenaltyContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::MXi &surfaceTriangles,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const SampledPenaltyContactEnergyOptions &options,
    std::vector<Mesh::TriMeshGeo> externalSurfaces = {});
  ~SampledPenaltyContactEnergy() override;

  ContactModelKind contactModelKind() const override { return ContactModelKind::SampledPenalty; }
  bool isStepDependent() const override { return frictionState_.has_value(); }
  void beginStep(const NonlinearOptimization::StepState &state) override;

  double func(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  void gradient(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  void hessian(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianInPlace(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianAlloc(EigenSupport::SpMatD &simulationHessian) const override;
  double func_grad(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  double func_grad_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void gradient_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override;
  int isHessianTopologyFixed() const override { return 0; }

  void updateExternalSurface(int index, const Mesh::TriMeshGeo &surface);

private:
  SampledPenaltyEnergyConfigurator makeConfigurator() const;
  std::unique_ptr<SampledPenaltyEvaluationBundle> buildBundle(EigenSupport::ConstRefVecXd surfacePositions) const;
  void configureExternalEnergy(PointPenetrationEnergy &energy) const;
  void configureSelfEnergy(
    PointTrianglePairCouplingEnergyWithCollision &energy,
    EigenSupport::ConstRefVecXd surfacePositions) const;

  SurfaceDofMap dofMap_;
  SampledPenaltyContactEnergyOptions options_;
  SampledPenaltyContactBuilder builder_;
  SampledPenaltyContactEvaluator evaluator_;
  std::optional<SampledPenaltyFrictionState> frictionState_;
};
```

- [ ] **Step 3: Implement no-cache evaluation**

Each public evaluation method must build a bundle exactly once per public call:

```cpp
double SampledPenaltyContactEnergy::func(EigenSupport::ConstRefVecXd simulationDisplacements) const
{
  const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
  const std::unique_ptr<SampledPenaltyEvaluationBundle> bundle = buildBundle(surfacePositions);
  return evaluator_.func(*bundle, surfacePositions);
}
```

For `func_grad_hessian`, use:

```cpp
const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
const std::unique_ptr<SampledPenaltyEvaluationBundle> bundle = buildBundle(surfacePositions);
EigenSupport::VXd surfaceGradient = EigenSupport::VXd::Zero(dofMap_.numSurfaceDofs());
EigenSupport::SpMatD surfaceHessian;
const double value = evaluator_.func_grad_hessian(*bundle, surfacePositions, surfaceGradient, surfaceHessian);
simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
return value;
```

`hessianAlloc()` must resize to simulation DOFs:

```cpp
void SampledPenaltyContactEnergy::hessianAlloc(EigenSupport::SpMatD &simulationHessian) const
{
  simulationHessian.resize(getNumDOFs(), getNumDOFs());
  simulationHessian.setZero();
}
```

- [ ] **Step 4: Map previous simulation displacement for friction**

Keep `SampledPenaltyFrictionState` storing surface-space previous positions. In `SampledPenaltyContactEnergy::beginStep()`:

```cpp
void SampledPenaltyContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  if (!frictionState_)
    return;
  if (state.previousX == nullptr)
    throw std::invalid_argument("SampledPenaltyContactEnergy::beginStep requires previousX when friction is enabled.");
  if (state.timestep <= 0.0)
    throw std::invalid_argument("SampledPenaltyContactEnergy::beginStep requires a positive timestep when friction is enabled.");

  EigenSupport::VXd previousSurfacePositions = dofMap_.surfacePositions(*state.previousX);
  NonlinearOptimization::StepState mapped = state;
  mapped.previousX = &previousSurfacePositions;
  frictionState_->beginStep(mapped, dofMap_.numSurfaceDofs());
}
```

- [ ] **Step 5: Update factory**

In `contactEnergyFactory.h`, replace the two sampled penalty factory functions with:

```cpp
std::shared_ptr<StatefulContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const SampledPenaltyContactSpec &params,
  std::optional<FrictionContactSpec> friction = std::nullopt,
  std::vector<Mesh::TriMeshGeo> externalSurfaces = {});
```

In `contactEnergyFactory.cpp`, construct `SampledPenaltyContactEnergy` directly:

```cpp
SampledPenalty::SampledPenaltyContactEnergyOptions options;
options.params = toSampledPenaltyParameters(params);
if (friction)
  options.friction = toFrictionParameters(*friction);

return std::make_shared<SampledPenalty::SampledPenaltyContactEnergy>(
  surface.restVertices,
  surfaceTriangles,
  surface.surfaceFromSimulationDispMap,
  options,
  std::move(externalSurfaces));
```

Remove `#include "mappedContactEnergy.h"` from this file.

- [ ] **Step 6: Delete sampled active-set cache**

Remove the cache files from `src/core/contact/CMakeLists.txt` and delete them:

```bash
git rm src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.h
git rm src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.cpp
```

Update `tests/src/core/contact/sampledPenaltyInternals_gtest.cpp` so it tests `SampledPenaltyEvaluationBundle` RAII and `SampledPenaltyContactEvaluator`, not cache semantics.

- [ ] **Step 7: Verify**

Run:

```bash
cmake --build build --target sampledPenaltyContactEnergy_gtest sampledPenaltyInternals_gtest contactEnergyFactory_gtest -j8
ctest --test-dir build --output-on-failure -R 'SampledPenaltyContactEnergyGTest|SampledPenaltyInternalsGTest|ContactEnergyFactoryGTest'
```

Expected: sampled penalty is no longer `EvaluationStateAwareEnergy` or `LineSearchAwareEnergy`; normal sampled penalty has `isStepDependent() == false`; friction sampled penalty has `isStepDependent() == true`.

## Task 5: 收敛 sampled penalty Python API

**Files:**

- Modify: `src/python/pypgo/contact/core.h`
- Modify: `src/python/pypgo/contact/core.cpp`
- Modify: `src/python/pypgo/contact/bindings.cpp`
- Modify: `pypgo/contact/__init__.py`
- Modify: `pypgo/contact/energies.py`
- Modify: `pypgo/tools/sim/_config.py`
- Modify: `pypgo/tools/sim/_scene.py`
- Modify: `tests/pypgo/test_contact.py`
- Modify: `tests/pypgo/test_sim_config.py`
- Modify: `tests/pypgo/test_sim_scene.py`

- [ ] **Step 1: Update Python tests first**

In `tests/pypgo/test_contact.py`, remove `FrictionalSampledPenaltyEnergy` from `expected` and add a friction case through `SampledPenaltyEnergy`:

```python
def test_sampled_penalty_optional_friction_requires_previous_state():
    vertices, triangles = _triangle_surface()
    surface = contact.ContactSurface.identity(vertices)
    penalty = contact.SampledPenaltyEnergy(
        surface,
        triangles,
        params=contact.SampledPenaltyParameters(stiffness=3.0, samples=1),
        friction=contact.FrictionParameters(friction_coeff=0.5, velocity_eps=1e-4),
    )

    assert penalty.is_step_dependent is True
    with pytest.raises(ValueError, match="previous_x"):
        penalty.begin_step(time=0.0, timestep=0.1)
    with pytest.raises(ValueError, match="positive"):
        penalty.begin_step(time=0.0, timestep=0.0, previous_x=np.zeros(9))

    penalty.begin_step(time=0.0, timestep=0.1, previous_x=np.zeros(9))
```

Change handle assertions:

```python
frictional = contact.SampledPenaltyEnergy(
    surface,
    triangles,
    friction=contact.FrictionParameters(),
)
assert isinstance(frictional._handle, _core.PySampledPenaltyContactEnergy)
assert not hasattr(_core, "PyFrictionalSampledPenaltyContactEnergy")
```

- [ ] **Step 2: Update Python facade**

In `pypgo/contact/energies.py`, change `SampledPenaltyEnergy.__init__` signature to:

```python
def __init__(
    self,
    surface: ContactSurface,
    surface_triangles,
    *,
    params: SampledPenaltyParameters | None = None,
    friction: FrictionParameters | None = None,
) -> None:
```

Validate friction:

```python
if friction is not None and not isinstance(friction, FrictionParameters):
    raise TypeError("friction must be a FrictionParameters or None")
```

Call `_core._create_sampled_penalty_contact_energy` with two extra arguments after `enable_external_contact`:

```python
friction.friction_coeff if friction is not None else None,
friction.velocity_eps if friction is not None else None,
```

Store:

```python
object.__setattr__(self, "friction", friction)
```

Override `begin_step` only when friction is enabled:

```python
def begin_step(self, *, time: float, timestep: float, previous_x=None) -> None:
    if self.friction is not None:
        if previous_x is None:
            raise ValueError("previous_x is required when SampledPenaltyEnergy friction is enabled")
        if float(timestep) <= 0.0:
            raise ValueError("timestep must be positive")
    super().begin_step(time=time, timestep=timestep, previous_x=previous_x)
```

Delete the `FrictionalSampledPenaltyEnergy` class.

In `pypgo/contact/__init__.py`, remove the import and `__all__` entry for `FrictionalSampledPenaltyEnergy`.

- [ ] **Step 3: Update nanobind layer**

In `src/python/pypgo/contact/core.h`, delete `PyFrictionalSampledPenaltyContactEnergy` and `createFrictionalSampledPenaltyEnergy`.

Change `createSampledPenaltyEnergy` signature to include:

```cpp
nb::object frictionCoeff,
nb::object velocityEps
```

In `core.cpp`, change `PyStatefulContactEnergy::isStepDependent()` to:

```cpp
return energy_->isStepDependent();
```

Remove `#include "stepDependentEnergy.h"`.

Build optional friction:

```cpp
std::optional<CT::FrictionContactSpec> friction;
if (!frictionCoeff.is_none() || !velocityEps.is_none()) {
  if (frictionCoeff.is_none() || velocityEps.is_none())
    throw nb::value_error("friction_coeff and velocity_eps must be provided together");
  CT::FrictionContactSpec spec;
  spec.frictionCoeff = nb::cast<double>(frictionCoeff);
  spec.velocityEps = nb::cast<double>(velocityEps);
  friction = spec;
}
auto energy = CT::SampledPenalty::createSampledPenaltyEnergy(
  surface.spec(), triangles,
  makeSampledPenaltyParams(stiffness, samples, enableSelfContact, enableExternalContact),
  friction);
```

In `bindings.cpp`, delete:

```cpp
nb::class_<PyFrictionalSampledPenaltyContactEnergy, PyStatefulContactEnergy>(m, "PyFrictionalSampledPenaltyContactEnergy");
m.def("_create_frictional_sampled_penalty_contact_energy", &createFrictionalSampledPenaltyEnergy,
  nb::arg("surface"),
  nb::arg("surface_triangles"),
  nb::arg("stiffness"),
  nb::arg("samples"),
  nb::arg("enable_self_contact"),
  nb::arg("enable_external_contact"),
  nb::arg("friction_coeff"),
  nb::arg("velocity_eps"));
```

Update `_create_sampled_penalty_contact_energy` binding to accept:

```cpp
nb::arg("friction_coeff") = nb::none(),
nb::arg("velocity_eps") = nb::none()
```

- [ ] **Step 4: Update sim config**

In `pypgo/tools/sim/_config.py`, change:

```python
CONTACT_MODELS = ("ipc", "floor", "sampled_penalty")
```

For dynamic friction, keep using `ContactConfig.friction_coeff` and `velocity_eps`; reject friction fields on non-sampled models:

```python
if model != "sampled_penalty" and ("friction_coeff" in payload or "velocity_eps" in payload):
    raise ConfigError(f"{label}: friction fields are only supported for sampled_penalty contact")
```

In static mode, reject sampled penalty contact only when friction is enabled:

```python
if cfg.mode != "dynamic" and any(c.model == "sampled_penalty" and c.friction_coeff > 0.0 for c in contact):
    raise ConfigError("sampled_penalty contact with friction requires dynamic mode")
```

In `pypgo/tools/sim/_scene.py`, replace the old `frictional_sampled_penalty` branch with:

```python
friction = None
if cfg.friction_coeff > 0.0:
    friction = _contact.FrictionParameters(
        friction_coeff=cfg.friction_coeff,
        velocity_eps=cfg.velocity_eps,
    )
e = _contact.SampledPenaltyEnergy(
    contact_surface,
    surface_triangles,
    params=_contact.SampledPenaltyParameters(
        stiffness=cfg.stiffness,
        samples=cfg.samples,
        enable_self_contact=cfg.enable_self_contact,
        enable_external_contact=cfg.enable_external_contact,
    ),
    friction=friction,
)
if e.is_step_dependent:
    stateful.append(e)
```

Keep IPC in `stateful` because moving obstacle time still uses `begin_step`.

- [ ] **Step 5: Verify**

Run:

```bash
cmake --build build --target pypgo -j8
python -m pytest tests/pypgo/test_contact.py tests/pypgo/test_sim_config.py tests/pypgo/test_sim_scene.py -q
```

Expected: `FrictionalSampledPenaltyEnergy` is no longer public, optional friction works through `SampledPenaltyEnergy`, and sampled penalty `is_step_dependent` comes from C++ `StatefulContactEnergy::isStepDependent()`.

## Task 6: 引入 `IPCPairGenerator`

**Files:**

- Create: `src/core/contact/ipc/ipcPairGenerator.h`
- Create: `src/core/contact/ipc/ipcPairGenerator.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Create: `tests/src/core/contact/ipcPairGenerator_gtest.cpp`
- Modify: `tests/src/core/contact/CMakeLists.txt`

- [ ] **Step 1: Add generator tests against current IPC behavior**

Create `tests/src/core/contact/ipcPairGenerator_gtest.cpp` with tests that compare `IPCPairGenerator` to current helper functions:

```cpp
#include <gtest/gtest.h>

#include "ipc/ipcPairGenerator.h"
#include "ipc/core/surfaceIPCMaxStep.h"
#include "testCIPCHelpers.h"

namespace
{
namespace ES = pgo::EigenSupport;
namespace IPC = pgo::Contact::IPC;
namespace NO = pgo::NonlinearOptimization;
}

TEST(IPCPairGeneratorGTest, BuildsSelfActiveSet)
{
  const auto [V, F] = pgo::Contact::CIPCTest::makeTwoTriangleMesh();
  IPC::IPCPairGenerator::Parameters params;
  params.dhat = 0.5;
  params.dhatExternal = 0.5;

  IPC::IPCPairGenerator generator(params);
  generator.setMesh(V, F);
  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(V);
  const IPC::SurfaceIPCActiveSet activeSet = generator.buildActiveSet(x);

  EXPECT_EQ(activeSet.positions.size(), x.size());
  EXPECT_TRUE(activeSet.selfPairs.size() > 0 || activeSet.externalPairs.size() == 0);
}

TEST(IPCPairGeneratorGTest, ComputesMaxStepConstraint)
{
  const auto [V, F] = pgo::Contact::CIPCTest::makeTwoTriangleMesh();
  IPC::IPCPairGenerator::Parameters params;
  params.dhat = 0.5;
  params.dhatExternal = 0.5;
  params.slackness = 1.0;

  IPC::IPCPairGenerator generator(params);
  generator.setMesh(V, F);
  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(V);
  ES::VXd dx = ES::VXd::Zero(x.size());
  dx[2] = -0.01;

  const NO::StepConstraint constraint = generator.computeMaxStepLimit(x, dx, nullptr);
  EXPECT_EQ(constraint.source, NO::StepSource::Contact);
  EXPECT_GT(constraint.alpha, 0.0);
  EXPECT_LE(constraint.alpha, 1.0);
}
```

These tests use `pgo::Contact::CIPCTest::makeTwoTriangleMesh()` and `pgo::Contact::CIPCTest::flattenPositions()` from `testCIPCHelpers.h`.

- [ ] **Step 2: Implement generator API**

Create `ipcPairGenerator.h`:

```cpp
#pragma once

#include "EigenDef.h"
#include "ipc/core/surfaceIPCActiveSet.h"
#include "ipc/external/obstacleSurface.h"
#include "ipc/topology/surfaceIPCTopology.h"
#include "solver/common/solveDiagnostics.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace pgo
{
namespace Contact
{
namespace IPC
{

class IPCPairGenerator
{
public:
  struct Parameters
  {
    double dhat = 1e-1;
    double dhatExternal = 1e-1;
    double slackness = 1.0;
    double ccdThickness = 0.0;
  };

  IPCPairGenerator() = default;
  explicit IPCPairGenerator(const Parameters &params);
  IPCPairGenerator(const Parameters &params, std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces);

  IPCPairGenerator(const IPCPairGenerator &other);
  IPCPairGenerator &operator=(const IPCPairGenerator &other);

  void setParameters(const Parameters &params);
  Parameters parameters() const;
  void setMesh(const EigenSupport::MXd &surfaceRestVertices, const EigenSupport::MXi &surfaceTriangles);
  void setMovingObstacleTime(double time);

  SurfaceIPCActiveSet buildActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const;
  SurfaceIPCActiveSet buildLineSearchActiveSetSuperset(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::ConstRefVecXd surfaceStep) const;
  NonlinearOptimization::StepConstraint computeMaxStepLimit(
    EigenSupport::ConstRefVecXd surfacePositions,
    EigenSupport::ConstRefVecXd surfaceStep,
    NonlinearOptimization::StepConstraintSink *sink = nullptr) const;

  const SurfaceIPCTopology &topology() const { return topology_; }
  std::vector<ObstacleSurfaceView> obstacleViews() const;

private:
  struct ObstacleSlot
  {
    enum class Kind
    {
      Static,
      Moving,
    };
    Kind kind = Kind::Static;
    std::size_t index = 0;
    int32_t objectId = -1;
  };

  void setObstacles(std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces);

  Parameters params_;
  SurfaceIPCTopology topology_;
  std::vector<std::unique_ptr<ObstacleSurface>> staticObstacles_;
  std::vector<std::unique_ptr<ObstacleSurface>> movingObstacles_;
  std::vector<ObstacleSlot> obstacleOrder_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 3: Move pair-generation code out of `SurfaceIPCCore`**

In `ipcPairGenerator.cpp`, move the logic from:

```text
SurfaceIPCCore::setMesh
SurfaceIPCCore::buildActiveSet
SurfaceIPCCore::buildLineSearchActiveSetSuperset
SurfaceIPCCore::computeMaxStepLimit
SurfaceIPCCore::setObstacles
SurfaceIPCCore::obstacleViews
SurfaceIPCCore::setMovingObstacleTime
```

Keep the calls to existing helpers:

```cpp
buildSelfPairs(topology_, activeSet.positions, params_.dhat, activeSet.selfPairs);
buildExternalPairs(topology_, activeSet.positions, obstacleViews(), params_.dhatExternal, activeSet.externalPairs);
buildSelfPairsLineSearchSuperset(topology_, activeSet.positions, surfaceStep, params_.dhat, activeSet.selfPairs);
buildExternalPairsLineSearchSuperset(topology_, activeSet.positions, surfaceStep, obstacleViews(), params_.dhatExternal, activeSet.externalPairs);
computeSelfMaxStep(topology_, surfacePositions, surfaceStep, params_.dhat, params_.slackness, params_.ccdThickness);
computeExternalMaxStep(topology_, surfacePositions, surfaceStep, obstacleViews(), params_.dhatExternal, params_.slackness, params_.ccdThickness);
```

Use `params_.dhat`, `params_.dhatExternal`, `params_.slackness`, and `params_.ccdThickness` instead of `SurfaceIPCCore` fields.

- [ ] **Step 4: Update CMake and verify**

Add generator files to `src/core/contact/CMakeLists.txt`, add `ipcPairGenerator_gtest` target to `tests/src/core/contact/CMakeLists.txt`, then run:

```bash
cmake --build build --target ipcPairGenerator_gtest -j8
ctest --test-dir build --output-on-failure -R 'IPCPairGeneratorGTest'
```

Expected: generator tests pass without modifying `IPCContactEnergy`.

## Task 7: 引入 `IPCContactAssembler`

**Files:**

- Create: `src/core/contact/ipc/ipcContactAssembler.h`
- Create: `src/core/contact/ipc/ipcContactAssembler.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Create: `tests/src/core/contact/ipcContactAssembler_gtest.cpp`
- Modify: `tests/src/core/contact/CMakeLists.txt`

- [ ] **Step 1: Define assembler API**

Create `ipcContactAssembler.h`:

```cpp
#pragma once

#include "EigenDef.h"
#include "ipc/core/surfaceIPCActiveSet.h"
#include "ipc/external/obstacleSurface.h"
#include "ipc/topology/surfaceIPCTopology.h"

#include <vector>

namespace pgo
{
namespace Contact
{
namespace IPC
{

class IPCContactAssembler
{
public:
  struct Parameters
  {
    double dhat = 1e-1;
    double dhatExternal = 1e-1;
    double kappa = 0.1;
    double epsEE = 0.0;
  };

  IPCContactAssembler() = default;
  explicit IPCContactAssembler(const Parameters &params);

  void setParameters(const Parameters &params);
  Parameters parameters() const;

  double computeEnergy(
    const SurfaceIPCTopology &topology,
    const std::vector<ObstacleSurfaceView> &obstacleViews,
    const SurfaceIPCActiveSet &activeSet) const;
  void computeGradient(
    const SurfaceIPCTopology &topology,
    const std::vector<ObstacleSurfaceView> &obstacleViews,
    const SurfaceIPCActiveSet &activeSet,
    EigenSupport::RefVecXd surfaceGradient) const;
  void computeHessian(
    const SurfaceIPCTopology &topology,
    const std::vector<ObstacleSurfaceView> &obstacleViews,
    const SurfaceIPCActiveSet &activeSet,
    EigenSupport::SpMatD &surfaceHessian) const;
  void computeAll(
    const SurfaceIPCTopology &topology,
    const std::vector<ObstacleSurfaceView> &obstacleViews,
    const SurfaceIPCActiveSet &activeSet,
    double &surfaceEnergy,
    EigenSupport::VXd &surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;

private:
  Parameters params_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 2: Move assembly calls**

In `ipcContactAssembler.cpp`, move the assembly body from `SurfaceIPCCore::computeEnergy(const SurfaceIPCActiveSet&)`, `computeGradient`, `computeHessian`, and `computeAll`. The assembler must call these existing kernel functions with the same argument values currently used by `SurfaceIPCCore`:

```cpp
computeSelfEnergy(activeSet.positions, activeSet.selfPairs, topology.numVerts, params_.dhat, params_.kappa, params_.epsEE);
computeExternalEnergy(activeSet.positions, obstacleViews, activeSet.externalPairs, params_.dhatExternal, params_.kappa, params_.epsEE);
computeSelfGradient(activeSet.positions, activeSet.selfPairs, topology.numVerts, params_.dhat, params_.kappa, params_.epsEE, surfaceGradient);
computeExternalGradient(activeSet.positions, obstacleViews, activeSet.externalPairs, topology.numVerts, params_.dhatExternal, params_.kappa, params_.epsEE, surfaceGradient);
computeSelfHessian(activeSet.positions, activeSet.selfPairs, topology.numVerts, params_.dhat, params_.kappa, params_.epsEE, surfaceHessian);
computeExternalHessian(activeSet.positions, obstacleViews, activeSet.externalPairs, topology.numVerts, params_.dhatExternal, params_.kappa, params_.epsEE, surfaceHessian);
computeSelfAll(activeSet.positions, activeSet.selfPairs, topology.numVerts, params_.dhat, params_.kappa, params_.epsEE, surfaceEnergy, surfaceGradient, surfaceHessian);
computeExternalAll(activeSet.positions, obstacleViews, activeSet.externalPairs, topology.numVerts, params_.dhatExternal, params_.kappa, params_.epsEE, externalEnergy, surfaceGradient, surfaceHessian);
```

It must not build active sets and must not call broad phase.

- [ ] **Step 3: Add assembler tests**

Create `tests/src/core/contact/ipcContactAssembler_gtest.cpp` that builds a small active set through `IPCPairGenerator`, then evaluates assembler methods:

```cpp
TEST(IPCContactAssemblerGTest, ComputesFiniteEnergyGradientAndHessianForGeneratedActiveSet)
{
  const auto [V, F] = pgo::Contact::CIPCTest::makeTwoTriangleMesh();
  IPC::IPCPairGenerator::Parameters generatorParams;
  generatorParams.dhat = 0.5;
  generatorParams.dhatExternal = 0.5;

  IPC::IPCPairGenerator generator(generatorParams);
  generator.setMesh(V, F);
  const ES::VXd x = pgo::Contact::CIPCTest::flattenPositions(V);
  const IPC::SurfaceIPCActiveSet activeSet = generator.buildActiveSet(x);

  IPC::IPCContactAssembler::Parameters assemblerParams;
  assemblerParams.dhat = 0.5;
  assemblerParams.dhatExternal = 0.5;
  assemblerParams.kappa = 1.0;
  assemblerParams.epsEE = 0.0;
  IPC::IPCContactAssembler assembler(assemblerParams);

  ES::VXd g = ES::VXd::Zero(x.size());
  ES::SpMatD H;
  double e = 0.0;
  assembler.computeAll(generator.topology(), generator.obstacleViews(), activeSet, e, g, H);

  EXPECT_TRUE(std::isfinite(e));
  EXPECT_EQ(g.size(), x.size());
  EXPECT_EQ(H.rows(), x.size());
  EXPECT_EQ(H.cols(), x.size());
}
```

Add local helpers if the named helpers do not exist.

- [ ] **Step 4: Verify**

Run:

```bash
cmake --build build --target ipcContactAssembler_gtest surfaceIPCBarrierAssembler_gtest -j8
ctest --test-dir build --output-on-failure -R 'IPCContactAssemblerGTest|SurfaceIPCBarrierAssembler'
```

Expected: assembler tests pass and existing barrier assembler tests remain green.

## Task 8: 将 `IPCContactEnergy` 改成直接组合 `SurfaceDofMap`

**Files:**

- Modify: `src/core/contact/ipc/ipcContactEnergy.h`
- Modify: `src/core/contact/ipc/ipcContactEnergy.cpp`
- Modify: `src/core/contact/contactEnergyFactory.cpp`
- Modify: `tests/src/core/contact/ipcContactEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/contactEnergyFactory_gtest.cpp`

- [ ] **Step 1: Update IPC tests for final boundary**

In `tests/src/core/contact/ipcContactEnergy_gtest.cpp`, remove tests that require `prepareEvaluationState()` through `StatefulContactEnergy`. Replace them with tests that verify repeated exact evaluation still works through internal cache:

```cpp
TEST(IPCContactEnergyGTest, DoesNotExposeEvaluationStateAwareInterface)
{
  IPCContactEnergy energy(V, F, makeIdentityEmbedding(rest.size()), makeParams());
  EXPECT_EQ(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(&energy), nullptr);
  EXPECT_NE(dynamic_cast<const NO::LineSearchAwareEnergy *>(&energy), nullptr);
  EXPECT_FALSE(energy.isStepDependent());
}
```

Keep tests for:

- sparse embedding pullback
- line-search superset reuse
- max-step behavior
- fused `func_grad_hessian`
- moving obstacle time update

- [ ] **Step 2: Change inheritance and members**

In `ipcContactEnergy.h`, make the class:

```cpp
class IPCContactEnergy:
  public StatefulContactEnergy,
  public NonlinearOptimization::LineSearchAwareEnergy
{
public:
  IPCContactEnergy(
    const EigenSupport::MXd &surfaceRestVertices,
    const EigenSupport::MXi &surfaceTriangles,
    const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
    const IPCPairGenerator::Parameters &pairParams,
    const IPCContactAssembler::Parameters &assemblerParams,
    std::vector<std::unique_ptr<ObstacleSurface>> obstacleSurfaces = {});

  ContactModelKind contactModelKind() const override { return ContactModelKind::IPC; }
  bool isStepDependent() const override { return false; }
  void beginStep(const NonlinearOptimization::StepState &state) override;
  NonlinearOptimization::StepConstraint computeMaxStepLimit(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::ConstRefVecXd trialSimulationDisplacements,
    NonlinearOptimization::StepConstraintSink *sink = nullptr) const override;
  void beginLineSearch(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::ConstRefVecXd trialSimulationDisplacements) const override;
  void endLineSearch() const override;
  double maxValidLineSearchAlpha() const override { return 1.0; }

  double func(EigenSupport::ConstRefVecXd simulationDisplacements) const override;
  void gradient(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  void hessian(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianInPlace(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::SpMatD &simulationHessian) const override;
  void hessianAlloc(EigenSupport::SpMatD &simulationHessian) const override;
  double func_grad(EigenSupport::ConstRefVecXd simulationDisplacements, EigenSupport::RefVecXd simulationGradient) const override;
  double func_grad_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void gradient_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override;
  int isHessianTopologyFixed() const override { return 0; }
  void setMovingObstacleTime(double t);

private:
  SurfaceIPCActiveSet buildExactActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const;
  const SurfaceIPCActiveSet &activeSetForEvaluation(EigenSupport::ConstRefVecXd surfacePositions) const;

  SurfaceDofMap dofMap_;
  IPCPairGenerator pairGenerator_;
  IPCContactAssembler assembler_;
  mutable IPCActiveSetCache activeSetCache_;
};
```

Do not inherit `ActiveSetContactEnergy`, `EvaluationStateAwareEnergy`, or `MappedSurfacePotentialEnergy`.

- [ ] **Step 3: Implement simulation-space evaluation**

In `ipcContactEnergy.cpp`, replace every call to `computeSurfacePositionsFromSimulationDisplacements` with:

```cpp
const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
```

Replace surface gradient/Hessian pullback with:

```cpp
simulationGradient = dofMap_.pullbackGradient(surfaceGradient);
dofMap_.pullbackHessian(surfaceHessian, simulationHessian);
```

Use assembler through:

```cpp
const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
const std::vector<ObstacleSurfaceView> obstacleViews = pairGenerator_.obstacleViews();
assembler_.computeAll(pairGenerator_.topology(), obstacleViews, activeSet, surfaceEnergy, surfaceGradient, surfaceHessian);
```

`activeSetForEvaluation(surfacePositions)` must keep this order:

```text
line-search cache if present
exact cache if it matches surfacePositions
build exact active set at surfacePositions
```

- [ ] **Step 4: Implement max-step and line search through `SurfaceDofMap`**

`computeMaxStepLimit()`:

```cpp
const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
const EigenSupport::VXd trialSurfaceDisplacements = dofMap_.surfaceDisplacements(trialSimulationDisplacements);
return pairGenerator_.computeMaxStepLimit(surfacePositions, trialSurfaceDisplacements, sink);
```

`beginLineSearch()`:

```cpp
const EigenSupport::VXd surfacePositions = dofMap_.surfacePositions(simulationDisplacements);
const EigenSupport::VXd trialSurfaceDisplacements = dofMap_.surfaceDisplacements(trialSimulationDisplacements);
activeSetCache_.beginLineSearch(pairGenerator_.buildLineSearchActiveSetSuperset(surfacePositions, trialSurfaceDisplacements));
```

`endLineSearch()`:

```cpp
activeSetCache_.endLineSearch();
```

- [ ] **Step 5: Wire parameters**

In `contactEnergyFactory.cpp`, convert `IPCContactSpec` to both generator and assembler params:

```cpp
IPC::IPCPairGenerator::Parameters pairParams;
pairParams.dhat = spec.dhat;
pairParams.dhatExternal = spec.dhatExternal;
pairParams.slackness = spec.slackness;
pairParams.ccdThickness = spec.ccdThickness;

IPC::IPCContactAssembler::Parameters assemblyParams;
assemblyParams.dhat = spec.dhat;
assemblyParams.dhatExternal = spec.dhatExternal;
assemblyParams.kappa = spec.kappa;
assemblyParams.epsEE = spec.epsEE;
```

Update `IPCContactEnergy` constructor to accept either `IPCContactSpec` converted internally or the two parameter structs. Pick one API and keep tests using the constructor signature that is easiest to read:

```cpp
IPCContactEnergy(V, F, W, pairParams, assemblyParams, obstacles)
```

- [ ] **Step 6: Verify**

Run:

```bash
cmake --build build --target ipcContactEnergy_gtest contactEnergyFactory_gtest -j8
ctest --test-dir build --output-on-failure -R 'IPCContactEnergyGTest|ContactEnergyFactoryGTest'
```

Expected: IPC no longer exposes `EvaluationStateAwareEnergy`, still exposes `LineSearchAwareEnergy`, and max-step/line-search tests still pass.

## Task 9: 清理最终继承体系和旧 mapped classes

**Files:**

- Modify: `src/core/contact/statefulContactEnergy.h`
- Delete: `src/core/contact/mappedContactEnergy.h`
- Delete: `src/core/contact/mappedContactEnergy.cpp`
- Delete: `src/core/contact/mappedSurfacePotentialEnergy.h`
- Delete: `src/core/contact/mappedSurfacePotentialEnergy.cpp`
- Delete: `src/core/nonlinearOptimization/stepDependentEnergy.h`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `src/core/nonlinearOptimization/CMakeLists.txt`
- Modify: `tests/src/core/contact/CMakeLists.txt`
- Delete or rewrite: `tests/src/core/contact/mappedContactEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/contactEnergyFactory_gtest.cpp`
- Modify: `src/python/pypgo/contact/core.cpp`

- [ ] **Step 1: Update `StatefulContactEnergy` to final form**

In `statefulContactEnergy.h`, remove `ActiveSetContactEnergy` entirely and make the file:

```cpp
#pragma once

#include "potentialEnergy.h"
#include "stepAwareEnergy.h"

namespace pgo
{
namespace Contact
{

enum class ContactModelKind
{
  Floor,
  IPC,
  SampledPenalty,
};

class StatefulContactEnergy:
  public NonlinearOptimization::PotentialEnergy,
  public NonlinearOptimization::StepAwareEnergy
{
public:
  ~StatefulContactEnergy() override = default;

  virtual ContactModelKind contactModelKind() const = 0;
  virtual bool isStepDependent() const { return false; }

  void beginStep(const NonlinearOptimization::StepState &) override {}

  NonlinearOptimization::EnergyStateKind stateKind() const override
  {
    return NonlinearOptimization::EnergyStateKind::Displacement;
  }
};

}  // namespace Contact
}  // namespace pgo
```

Then remove explicit `public NonlinearOptimization::StepAwareEnergy` from concrete contact energy declarations that already inherit `StatefulContactEnergy`.

- [ ] **Step 2: Remove old files from CMake**

In `src/core/contact/CMakeLists.txt`, remove:

```cmake
mappedContactEnergy.h
mappedSurfacePotentialEnergy.h
mappedContactEnergy.cpp
mappedSurfacePotentialEnergy.cpp
```

In `src/core/nonlinearOptimization/CMakeLists.txt`, remove:

```cmake
stepDependentEnergy.h
```

Delete files:

```bash
git rm src/core/contact/mappedContactEnergy.h
git rm src/core/contact/mappedContactEnergy.cpp
git rm src/core/contact/mappedSurfacePotentialEnergy.h
git rm src/core/contact/mappedSurfacePotentialEnergy.cpp
git rm src/core/nonlinearOptimization/stepDependentEnergy.h
```

- [ ] **Step 3: Remove old tests**

Remove `mappedContactEnergy_gtest` from `tests/src/core/contact/CMakeLists.txt` and delete `tests/src/core/contact/mappedContactEnergy_gtest.cpp`:

```bash
git rm tests/src/core/contact/mappedContactEnergy_gtest.cpp
```

Keep the mapping tests in `surfaceDofMap_gtest` from Task 1.

- [ ] **Step 4: Update boundary tests**

In `tests/src/core/contact/contactEnergyFactory_gtest.cpp`, update `StatefulContactEnergyIsOnlyCommonContactBoundary`:

```cpp
TEST(ContactEnergyFactoryGTest, StatefulContactEnergyHasDefaultStepLifecycle)
{
  TestStatefulContactEnergy energy(3);
  const auto *stepAware = dynamic_cast<NO::StepAwareEnergy *>(&energy);
  const auto *evaluationAware = dynamic_cast<const NO::EvaluationStateAwareEnergy *>(&energy);
  const auto *lineSearchAware = dynamic_cast<const NO::LineSearchAwareEnergy *>(&energy);

  ASSERT_NE(stepAware, nullptr);
  EXPECT_EQ(evaluationAware, nullptr);
  EXPECT_EQ(lineSearchAware, nullptr);
  EXPECT_FALSE(energy.isStepDependent());

  NO::StepState state;
  EXPECT_NO_THROW(stepAware->beginStep(state));
}
```

- [ ] **Step 5: Verify no stale includes remain**

Run:

```bash
rg -n 'mappedContactEnergy|mappedSurfacePotentialEnergy|StepDependentEnergy|stepDependentEnergy|ActiveSetContactEnergy|FrictionalSampledPenaltySurfaceContactEnergy|SampledPenaltySurfaceContactEnergy' src tests pypgo
```

Expected: no matches, except historical mentions inside docs under `docs/superpowers`.

- [ ] **Step 6: Build and test**

Run:

```bash
cmake --build build --target contactEnergyFactory_gtest floorContactEnergy_gtest sampledPenaltyContactEnergy_gtest ipcContactEnergy_gtest -j8
ctest --test-dir build --output-on-failure -R 'ContactEnergyFactoryGTest|FloorContactEnergyGTest|SampledPenaltyContactEnergyGTest|IPCContactEnergyGTest'
```

Expected: selected tests pass.

## Task 10: 删除 `SurfaceIPCCore` façade 并完成 IPC 文件收口

**Files:**

- Delete: `src/core/contact/ipc/core/surfaceIPCCore.h`
- Delete: `src/core/contact/ipc/core/surfaceIPCCore.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `tests/src/core/contact/CMakeLists.txt`
- Delete or rewrite: `tests/src/core/contact/surfaceIPCCore_gtest.cpp`
- Modify: `src/core/contact/contactEnergyFactory.cpp`
- Modify: `src/core/contact/ipc/ipcContactEnergy.h`
- Modify: `src/core/contact/ipc/ipcContactEnergy.cpp`

- [ ] **Step 1: Confirm no production include remains**

Run:

```bash
rg -n 'surfaceIPCCore|SurfaceIPCCore' src/core/contact src/python pypgo
```

Expected before deletion: only `surfaceIPCCore.*`, CMake, and obsolete tests still match.

- [ ] **Step 2: Delete façade files and test target**

Run:

```bash
git rm src/core/contact/ipc/core/surfaceIPCCore.h
git rm src/core/contact/ipc/core/surfaceIPCCore.cpp
git rm tests/src/core/contact/surfaceIPCCore_gtest.cpp
```

Remove `surfaceIPCCore.*` from `src/core/contact/CMakeLists.txt`.

Remove the `surfaceIPCCore_gtest` target from `tests/src/core/contact/CMakeLists.txt`.

- [ ] **Step 3: Preserve coverage in new IPC tests**

Before deleting the old test target, move any still-relevant assertions into:

- `tests/src/core/contact/ipcPairGenerator_gtest.cpp` for active-set, line-search superset, max-step, obstacle time.
- `tests/src/core/contact/ipcContactAssembler_gtest.cpp` for energy/gradient/Hessian assembly.
- `tests/src/core/contact/ipcContactEnergy_gtest.cpp` for solver lifecycle and mapping pullback.

Each moved assertion must call `IPCPairGenerator`, `IPCContactAssembler`, or `IPCContactEnergy` directly. No test should instantiate `SurfaceIPCCore`.

- [ ] **Step 4: Verify no façade references remain**

Run:

```bash
rg -n 'surfaceIPCCore|SurfaceIPCCore' src tests pypgo
```

Expected: no matches, except docs under `docs/superpowers`.

- [ ] **Step 5: Build and test IPC**

Run:

```bash
cmake --build build --target ipcPairGenerator_gtest ipcContactAssembler_gtest ipcContactEnergy_gtest surfaceIPCMaxStep_gtest surfaceIPCExternalMaxStep_gtest surfaceIPCBarrierAssembler_gtest -j8
ctest --test-dir build --output-on-failure -R 'IPCPairGeneratorGTest|IPCContactAssemblerGTest|IPCContactEnergyGTest|SurfaceIPCMaxStep|SurfaceIPCExternalMaxStep|SurfaceIPCBarrierAssembler'
```

Expected: IPC decomposition tests and existing low-level IPC tests pass.

## Task 11: 全量 contact verification 与 docs 更新

**Files:**

- Modify: `docs/superpowers/specs/2026-06-13-contact-internal-refactor-design.md`
- Modify: `examples/scripts/generate_contact_api_demo.py`
- Modify: generated notebooks only if this repository tracks regenerated notebooks and the current workflow already requires regeneration.

- [ ] **Step 1: Run final stale-symbol scan**

Run:

```bash
rg -n 'MappedContactEnergy|MappedSurfacePotentialEnergy|MappedEvaluationContactEnergy|MappedStepAwareContactEnergy|StepDependentEnergy|ActiveSetContactEnergy|prepareEvaluationState|FrictionalSampledPenaltyEnergy|_create_frictional_sampled_penalty_contact_energy|SurfaceIPCCore' src tests pypgo examples
```

Expected: no matches in production code, tests, Python package, or examples.

- [ ] **Step 2: Update docs/spec status**

In `docs/superpowers/specs/2026-06-13-contact-internal-refactor-design.md`, add a short implementation-status note below the title:

```markdown
实施状态：phase 1 implementation plan 已拆分为 `docs/superpowers/plans/2026-06-13-contact-internal-refactor-implementation.md`。实现完成后，本 spec 中的 mapped wrapper、`StepDependentEnergy`、`EvaluationStateAwareEnergy` contact 路径应全部不存在于生产代码中。
```

- [ ] **Step 3: Update contact API example generator**

In `examples/scripts/generate_contact_api_demo.py`, replace each construction of `pc.FrictionalSampledPenaltyEnergy` with:

```python
pc.SampledPenaltyEnergy(
    contact_surface,
    triangles,
    params=pc.SampledPenaltyParameters(
        stiffness=FRICTIONAL_SCENE["contact_k"],
        samples=FRICTIONAL_SCENE["sp_samples"],
        enable_self_contact=FRICTIONAL_SCENE["sp_self_contact"],
        enable_external_contact=FRICTIONAL_SCENE["sp_external_contact"],
    ),
    friction=pc.FrictionParameters(
        friction_coeff=FRICTIONAL_SCENE["friction_coeff"],
        velocity_eps=FRICTIONAL_SCENE["velocity_eps"],
    ),
)
```

Replace text that says frictional sampled penalty is a separate energy class with text that says it is an optional mode of `SampledPenaltyEnergy`.

- [ ] **Step 4: Run full contact C++ tests**

Run:

```bash
cmake --build build --target surfaceDofMap_gtest floorContactEnergy_gtest contactEnergyFactory_gtest sampledPenaltyContactEnergy_gtest sampledPenaltyInternals_gtest ipcPairGenerator_gtest ipcContactAssembler_gtest ipcContactEnergy_gtest ipcActiveSetCache_gtest ipcGeometry_gtest spatialHashGrid_gtest surfaceIPCBarrierAssembler_gtest surfaceIPCExternalBroadPhase_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalMaxStep_gtest surfaceIPCMaxStep_gtest surfaceIPCTopology_gtest -j8
ctest --test-dir build --output-on-failure -R 'SurfaceDofMap|FloorContactEnergy|ContactEnergyFactory|SampledPenalty|IPCPairGenerator|IPCContactAssembler|IPCContactEnergy|IPCActiveSetCache|IPCGeometry|SpatialHashGrid|SurfaceIPC'
```

Expected: all selected C++ tests pass.

- [ ] **Step 5: Run Python contact/sim tests**

Run:

```bash
python -m pytest tests/pypgo/test_contact.py tests/pypgo/test_sim_config.py tests/pypgo/test_sim_scene.py tests/pypgo/test_dynamic_stepper.py -q
```

Expected: all selected Python tests pass.

- [ ] **Step 6: Final inheritance sanity check**

Run:

```bash
rg -n 'public .*PotentialEnergy|public .*StepAwareEnergy|public .*EvaluationStateAwareEnergy|public .*LineSearchAwareEnergy' src/core/contact
```

Expected:

- `StatefulContactEnergy` is the only contact type directly inheriting `PotentialEnergy`.
- `StatefulContactEnergy` is the only contact type directly inheriting `StepAwareEnergy`.
- `IPCContactEnergy` is the only contact type inheriting `LineSearchAwareEnergy`.
- No contact type inherits `EvaluationStateAwareEnergy`.

## Subagent 调度建议

按以下顺序派 fresh subagent，每个 subagent 只拿对应 task 的完整文本和必要 spec 摘要：

1. Task 1：机械 rename + mapping tests，适合 fast implementation agent。
2. Task 2：floor direct composition，适合 fast implementation agent。
3. Task 3：sampled bundle/builder/evaluator rename，适合 standard implementation agent。
4. Task 4：sampled simulation-space energy，适合 standard implementation agent。
5. Task 5：Python/API 收敛，适合 standard implementation agent。
6. Task 6：IPC pair generator，适合 standard implementation agent。
7. Task 7：IPC assembler，适合 standard implementation agent。
8. Task 8：IPC energy direct composition，适合 strongest implementation agent。
9. Task 9：继承体系 cleanup，适合 strongest implementation agent。
10. Task 10：删除 `SurfaceIPCCore` façade，适合 standard implementation agent。
11. Task 11：最终 verification/docs，适合 fast implementation agent。

Task 8 和 Task 9 是最容易引入继承歧义的两步。controller 在这两步 review 时必须检查：

- 没有 concrete contact class 通过两条路径继承 `StepAwareEnergy`。
- 没有 concrete contact class 通过两条路径继承 `PotentialEnergy`。
- `dynamic_cast<NO::EvaluationStateAwareEnergy *>` 不再命中任何 contact energy。

## Self Review

### Spec 覆盖

- 不引入 `ContactTable`：所有任务只处理 internal contact architecture。
- `SurfaceDofMap`：Task 1 建立并测试 mapping/pullback。
- floor 组合 mapping：Task 2 覆盖。
- sampled penalty 行为贴近旧版：Task 3/4 保留 handler/kernel detection 和 child energy evaluation，每次 public evaluation 构造 bundle。
- sampled penalty 不参与 line search cache：Task 4 删除 sampled cache 和 line-search/evaluation-aware 接口。
- sampled penalty optional friction：Task 4/5 统一到一个 C++/Python energy。
- Python API 收敛：Task 5 删除 separate frictional facade 和 binding。
- IPC pair generation/assembler/energy 拆分：Task 6/7/8 覆盖。
- `SurfaceIPCCore` 不作为新 façade 保留：Task 10 删除。
- 最终继承体系：Task 9/11 覆盖。

### 占位符扫描

计划中每个 task 都指定了文件、代码形状、验证命令和预期结果。没有留空的实现项；大规模迁移处使用明确的函数名和类型名约束 subagent。

### 类型一致性

- 公共 mapping 类型统一为 `pgo::Contact::SurfaceDofMap`。
- public sampled penalty 类型统一为 `pgo::Contact::SampledPenalty::SampledPenaltyContactEnergy`。
- sampled bundle 类型统一为 `SampledPenaltyEvaluationBundle`。
- IPC 拆分类型统一为 `IPCPairGenerator`、`IPCContactAssembler`、`IPCContactEnergy`。
- step dependency 查询统一为 `StatefulContactEnergy::isStepDependent()` 和 Python `is_step_dependent`。
