# Contact Internals D2-D6 Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish the D2-D6 contact-internals design decisions: keep contact lifecycle explicit but not public-manual, remove inheritance artifacts, narrow mapped-surface responsibilities, centralize active-set cache state machines, split sampled penalty glue, and clean the public factory / broad-phase duplication boundaries.

**Architecture:** Public contact identity stays small (`StatefulContactEnergy`), lifecycle capabilities stay opt-in (`ActiveSetContactEnergy`, `StepAwareEnergy`). Mapping, cache, detector, friction, and broad-phase query skeletons become separate units with narrow responsibilities; numerical kernels, IPC barrier formulas, CCD, pair predicates, and Python public API behavior remain unchanged.

**Tech Stack:** C++20, Eigen, TBB, GoogleTest, Nanobind Python facade, CMake targets under `build/base`.

---

## Scope And Non-Goals

### In Scope

- D2: Active-set lifecycle boundary verification and protection.
- D3: Non-virtual inheritance cleanup verification and dependency tightening.
- D4: `MappedSurfacePotentialEnergy` contract fix and responsibility narrowing.
- D5: IPC active-set cache extraction.
- D6: Sampled penalty internal decomposition, public factory cleanup, and broad-phase query skeleton extraction.

### Non-Goals

- Do not change IPC barrier, CCD, sampled penalty numerical kernels, distance predicates, or pair acceptance math.
- Do not add tolerance-based active-set cache matching; exact vector equality remains the only reuse policy.
- Do not remove existing public Python classes such as `pypgo.contact.IPCEnergy` or `pypgo.contact.SampledPenaltyEnergy`.
- Do not make sampled penalty support embedded/non-identity surface maps in this plan; document that as an explicit backend capability limit.
- Do not replace all broad-phase code with a new architecture in one change; only extract reusable skeletons while preserving pair ordering and counters.

## Current Repo Truth

- `src/core/contact/statefulContactEnergy.h` already has `StatefulContactEnergy` and `ActiveSetContactEnergy` separated.
- `src/core/contact/ipc/ipcContactEnergy.h` currently derives from `MappedSurfacePotentialEnergy`, `ActiveSetContactEnergy`, and `StepAwareEnergy`.
- `src/core/contact/mappedSurfacePotentialEnergy.cpp` still throws in `hessianInPlace()` / `hessianAlloc()`, which violates the practical `PotentialEnergy` contract for direct helper APIs.
- `src/core/contact/ipc/ipcContactEnergy.cpp` still owns exact and line-search active-set cache state directly.
- `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp` still mixes energy evaluation, handler ownership, active-set cache, RAII buffer cleanup, and friction configuration.
- `src/core/contact/contactEnergyFactory.h` still includes `ipc/core/surfaceIPCCore.h` only to expose `IPC::ParametersSpec = SurfaceIPCCore::Parameters`.
- `src/core/contact/ipc/broadPhase/surfaceIPCSelfBroadPhase.cpp` and `surfaceIPCExternalBroadPhase.cpp` duplicate normal vs line-search query flow.

## File Map

### Existing Files To Modify

- `src/core/contact/statefulContactEnergy.h`
- `src/core/contact/mappedSurfacePotentialEnergy.h`
- `src/core/contact/mappedSurfacePotentialEnergy.cpp`
- `src/core/contact/ipc/ipcContactEnergy.h`
- `src/core/contact/ipc/ipcContactEnergy.cpp`
- `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`
- `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp`
- `src/core/contact/sampled_penalty/sampledPenaltySpecs.h`
- `src/core/contact/contactEnergyFactory.h`
- `src/core/contact/contactEnergyFactory.cpp`
- `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhaseInternal.h`
- `src/core/contact/ipc/broadPhase/surfaceIPCSelfBroadPhase.cpp`
- `src/core/contact/ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp`
- `src/core/contact/CMakeLists.txt`
- `tests/src/core/contact/CMakeLists.txt`
- `tests/src/core/contact/contactEnergyFactory_gtest.cpp`
- `tests/src/core/contact/ipcContactEnergy_gtest.cpp`
- `tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest.cpp`
- `tests/src/core/contact/sampledPenaltyContactEnergy_gtest.cpp`
- `tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp`
- `tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest.cpp`
- `tests/pypgo/test_contact.py`

### New Files To Create

- `src/core/contact/ipc/ipcActiveSetCache.h`
- `src/core/contact/ipc/ipcActiveSetCache.cpp`
- `tests/src/core/contact/ipcActiveSetCache_gtest.cpp`
- `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.h`
- `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.cpp`
- `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.h`
- `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.cpp`
- `src/core/contact/sampled_penalty/sampledPenaltySpecs.h`
- `src/core/contact/sampled_penalty/sampledPenaltyFrictionState.h`
- `src/core/contact/sampled_penalty/sampledPenaltyFrictionState.cpp`
- `src/core/contact/sampled_penalty/sampledPenaltyContactDetector.h`
- `src/core/contact/sampled_penalty/sampledPenaltyContactDetector.cpp`
- `tests/src/core/contact/sampledPenaltyInternals_gtest.cpp`

## Done Criteria

- `rg -n "refreshActiveSet|clearActiveSet|refresh_active_set|clear_active_set" src pypgo tests -g '!build'` reports only negative `hasattr` tests if those tests still exist.
- `rg -n "public virtual|virtual .*StatefulContactEnergy" src/core/contact src/python/pypgo/bindings/contact_bindings.cpp` reports no inheritance artifacts.
- `rg -n "#include \"ipc/core/surfaceIPCCore.h\"" src/core/contact/contactEnergyFactory.h` reports no match.
- `MappedSurfacePotentialEnergy::hessianInPlace()` and `MappedSurfacePotentialEnergy::hessianAlloc()` no longer throw.
- IPC active-set cache state is owned by `IPCActiveSetCache`, not by `IPCContactEnergy`.
- `SampledPenaltyContactEnergy` has no protected virtual friction hooks such as `hasFrictionStepState()` or `configureExternalActiveEnergy()`.
- `ctest --test-dir build/base --output-on-failure -R "Contact|Evaluation|contact|evaluation"` passes.
- `PYTHONPATH="$PWD" pytest tests/pypgo/test_contact.py` passes.

---

## Task 1: Protect D2/D3 Lifecycle And Inheritance Decisions

**Files:**
- Modify: `tests/src/core/contact/contactEnergyFactory_gtest.cpp`
- Modify: `tests/pypgo/test_contact.py`
- Inspect: `src/core/contact/statefulContactEnergy.h`
- Inspect: `src/core/contact/ipc/ipcContactEnergy.h`
- Inspect: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`

- [ ] **Step 1: Add or keep C++ capability-boundary assertions**

In `tests/src/core/contact/contactEnergyFactory_gtest.cpp`, keep a test equivalent to:

```cpp
TEST(ContactEnergyFactoryGTest, StatefulContactEnergyIsOnlyCommonContactBoundary)
{
  TestStatefulContactEnergy energy(3);
  EXPECT_EQ(dynamic_cast<NO::StepAwareEnergy *>(&energy), nullptr);
  EXPECT_EQ(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(&energy), nullptr);
  EXPECT_EQ(dynamic_cast<const NO::LineSearchAwareEnergy *>(&energy), nullptr);
  EXPECT_EQ(energy.stateKind(), NO::EnergyStateKind::Displacement);
}
```

Also keep capability assertions for concrete energies:

```cpp
EXPECT_NE(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(normal.get()), nullptr);
EXPECT_NE(dynamic_cast<const NO::LineSearchAwareEnergy *>(normal.get()), nullptr);
EXPECT_EQ(dynamic_cast<NO::StepAwareEnergy *>(normal.get()), nullptr);

EXPECT_NE(dynamic_cast<const NO::EvaluationStateAwareEnergy *>(frictional.get()), nullptr);
EXPECT_NE(dynamic_cast<const NO::LineSearchAwareEnergy *>(frictional.get()), nullptr);
EXPECT_NE(dynamic_cast<NO::StepAwareEnergy *>(frictional.get()), nullptr);
EXPECT_NE(dynamic_cast<NO::StepDependentEnergy *>(frictional.get()), nullptr);
```

- [ ] **Step 2: Keep Python public API negative assertions**

In `tests/pypgo/test_contact.py`, keep assertions equivalent to:

```python
assert not hasattr(penalty, "refresh_active_set")
assert not hasattr(penalty, "clear_active_set")
assert not hasattr(ipc, "refresh_active_set")
assert not hasattr(ipc, "clear_active_set")
```

- [ ] **Step 3: Run focused lifecycle tests**

Run:

```bash
cmake --build build/base --target contactEnergyFactory_gtest pypgo_core -j 8
ctest --test-dir build/base --output-on-failure -R "ContactEnergyFactoryGTest"
PYTHONPATH="$PWD" pytest tests/pypgo/test_contact.py
```

Expected:

```text
100% tests passed
tests/pypgo/test_contact.py ... passed
```

- [ ] **Step 4: Scan for public lifecycle and virtual inheritance regressions**

Run:

```bash
rg -n "refreshActiveSet|clearActiveSet|refresh_active_set|clear_active_set" src pypgo tests -g '!build'
rg -n "public virtual|virtual .*StatefulContactEnergy" src/core/contact src/python/pypgo/bindings/contact_bindings.cpp
```

Expected:

```text
# first command: only negative hasattr assertions in tests, or no matches
# second command: no matches
```

- [ ] **Step 5: Commit D2/D3 protection**

```bash
git add tests/src/core/contact/contactEnergyFactory_gtest.cpp tests/pypgo/test_contact.py
git commit -m "test: protect contact lifecycle boundary"
```

If the working tree already contains the implementation from a previous session, include only the relevant test files in this commit.

---

## Task 2: Fix `MappedSurfacePotentialEnergy` Hessian Contract

**Files:**
- Modify: `src/core/contact/mappedSurfacePotentialEnergy.h`
- Modify: `src/core/contact/mappedSurfacePotentialEnergy.cpp`
- Modify: `tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest.cpp`

- [ ] **Step 1: Write failing test for direct Hessian allocation path**

Add this test to `tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest.cpp`:

```cpp
TEST(EmbeddedSurfaceFloorPotentialEnergyGTest, HessianInPlaceAndAllocDoNotThrow)
{
  const EigenSupport::MXd V = (EigenSupport::MXd(3, 3) <<
    0.0, 0.0, -0.1,
    1.0, 0.0,  0.0,
    0.0, 1.0,  0.0).finished();

  EigenSupport::SpMatD map(9, 9);
  map.setIdentity();

  IPC::FloorPenaltyParameters params;
  params.floorAxis = IPC::FloorAxis::Z;
  params.floorSide = IPC::FloorSide::KEEP_ABOVE;
  params.floorHeight = 0.0;
  params.floorKappa = 10.0;

  IPC::EmbeddedSurfaceFloorPotentialEnergy energy(V, map, params);
  EigenSupport::VXd x = EigenSupport::VXd::Zero(9);

  EigenSupport::SpMatD Halloc;
  EXPECT_NO_THROW(energy.hessianAlloc(Halloc));
  EXPECT_EQ(Halloc.rows(), 9);
  EXPECT_EQ(Halloc.cols(), 9);

  EigenSupport::SpMatD HinPlace;
  EXPECT_NO_THROW(energy.hessianInPlace(x, HinPlace));
  EXPECT_EQ(HinPlace.rows(), 9);
  EXPECT_EQ(HinPlace.cols(), 9);
}
```

- [ ] **Step 2: Run test and verify failure**

Run:

```bash
cmake --build build/base --target embeddedSurfaceFloorPotentialEnergy_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "EmbeddedSurfaceFloorPotentialEnergyGTest.HessianInPlaceAndAllocDoNotThrow"
```

Expected before implementation:

```text
FAILED
MappedSurfacePotentialEnergy::hessianAlloc() should not be called directly
```

- [ ] **Step 3: Implement non-throwing Hessian methods**

In `src/core/contact/mappedSurfacePotentialEnergy.cpp`, replace the throwing methods with:

```cpp
void MappedSurfacePotentialEnergy::hessianInPlace(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::SpMatD &simulationHessian) const
{
  hessian(simulationDisplacements, simulationHessian);
}

void MappedSurfacePotentialEnergy::hessianAlloc(EigenSupport::SpMatD &simulationHessian) const
{
  simulationHessian.resize(getNumDOFs(), getNumDOFs());
  simulationHessian.setZero();
}
```

Do not change `isHessianTopologyFixed()`: it must remain `0`.

- [ ] **Step 4: Run focused test**

Run:

```bash
cmake --build build/base --target embeddedSurfaceFloorPotentialEnergy_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "EmbeddedSurfaceFloorPotentialEnergyGTest.HessianInPlaceAndAllocDoNotThrow"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 5: Commit contract fix**

```bash
git add src/core/contact/mappedSurfacePotentialEnergy.cpp tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest.cpp
git commit -m "fix: complete mapped surface hessian contract"
```

---

## Task 3: Narrow `MappedSurfacePotentialEnergy` To Mapping And Pullback

**Files:**
- Modify: `src/core/contact/mappedSurfacePotentialEnergy.h`
- Modify: `src/core/contact/mappedSurfacePotentialEnergy.cpp`
- Modify: `src/core/contact/ipc/ipcContactEnergy.h`
- Modify: `src/core/contact/ipc/ipcContactEnergy.cpp`
- Test: `tests/src/core/contact/ipcContactEnergy_gtest.cpp`

- [ ] **Step 1: Preserve IPC fusion tests**

Before editing production code, run:

```bash
cmake --build build/base --target ipcContactEnergy_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "IPCContactEnergyGTest.FuncGradHessianFusesOneBroadPhaseForAllThree|IPCContactEnergyGTest.FuncGradFusesOneBroadPhaseForEnergyAndGradient|IPCContactEnergyGTest.GradientHessianFusesOneBroadPhaseForGradAndHess"
```

Expected:

```text
100% tests passed
```

These tests are the safety net that IPC fused evaluation still builds one active set.

- [ ] **Step 2: Add protected pullback helpers**

In `src/core/contact/mappedSurfacePotentialEnergy.h`, add protected helpers:

```cpp
EigenSupport::VXd pullbackSurfaceGradient(EigenSupport::ConstRefVecXd surfaceGradient) const;
void pullbackSurfaceHessian(
  const EigenSupport::SpMatD &surfaceHessian,
  EigenSupport::SpMatD &simulationHessian) const;
```

In `src/core/contact/mappedSurfacePotentialEnergy.cpp`, implement:

```cpp
EigenSupport::VXd MappedSurfacePotentialEnergy::pullbackSurfaceGradient(
  EigenSupport::ConstRefVecXd surfaceGradient) const
{
  Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackGradient);
  return surfaceFromSimulationDispMap_.transpose() * surfaceGradient;
}

void MappedSurfacePotentialEnergy::pullbackSurfaceHessian(
  const EigenSupport::SpMatD &surfaceHessian,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection pullbackProfile(SurfaceIPCProfileSections::kAdapterPullbackHessian);
  simulationHessian = surfaceFromSimulationDispMap_.transpose() * surfaceHessian * surfaceFromSimulationDispMap_;
}
```

Then replace duplicated pullback blocks inside base `gradient()`, `hessian()`, `func_grad()`, `func_grad_hessian()`, and `gradient_hessian()` with the helpers.

- [ ] **Step 3: Remove IPC-specific surface hooks from base**

In `src/core/contact/mappedSurfacePotentialEnergy.h`, remove these protected virtual declarations:

```cpp
virtual NonlinearOptimization::StepConstraint computeSurfaceMaxStepLimit(...);
virtual void beginSurfaceLineSearch(...);
virtual void endSurfaceLineSearch() const;
```

In `src/core/contact/mappedSurfacePotentialEnergy.cpp`, remove their definitions.

Keep these optional fused hooks for now because both base and IPC can still use them without exposing lifecycle semantics:

```cpp
virtual void computeSurfaceGradHessian(...);
virtual void computeSurfaceFuncGrad(...);
virtual void computeSurfaceAll(...);
```

- [ ] **Step 4: Move max-step and line-search mapping into IPC**

In `src/core/contact/ipc/ipcContactEnergy.h`, replace IPC overrides of removed surface hooks with public energy overrides:

```cpp
NonlinearOptimization::StepConstraint computeMaxStepLimit(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::ConstRefVecXd trialSimulationDisplacements,
  NonlinearOptimization::StepConstraintSink *sink = nullptr) const override;
```

Keep `beginActiveSetLineSearch()` and `endActiveSetLineSearch()` as the lifecycle entry points.

In `src/core/contact/ipc/ipcContactEnergy.cpp`, implement:

```cpp
NonlinearOptimization::StepConstraint IPCContactEnergy::computeMaxStepLimit(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::ConstRefVecXd trialSimulationDisplacements,
  NonlinearOptimization::StepConstraintSink *sink) const
{
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);
  const VXd surfaceDisplacements = computeSurfaceDisplacementsFromSimulationDisplacements(trialSimulationDisplacements);
  return surfaceIPCCore_.computeMaxStepLimit(surfacePositions, surfaceDisplacements, sink);
}
```

In `beginActiveSetLineSearch()`, build the line-search active set directly rather than calling removed surface hooks:

```cpp
const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);
const VXd trialSurfaceDisplacements =
  computeSurfaceDisplacementsFromSimulationDisplacements(trialSimulationDisplacements);
clearCachedEnergyActiveSet();
lineSearchActiveSet_ =
  surfaceIPCCore_.buildLineSearchActiveSetSuperset(surfacePositions, trialSurfaceDisplacements);
hasLineSearchActiveSet_ = true;
```

Task 4 replaces this transitional direct cache code with `IPCActiveSetCache`.

- [ ] **Step 5: Run mapped surface and IPC tests**

Run:

```bash
cmake --build build/base --target ipcContactEnergy_gtest embeddedSurfaceFloorPotentialEnergy_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "IPCContactEnergyGTest|EmbeddedSurfaceFloorPotentialEnergyGTest"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 6: Commit mapped surface narrowing**

```bash
git add src/core/contact/mappedSurfacePotentialEnergy.h src/core/contact/mappedSurfacePotentialEnergy.cpp src/core/contact/ipc/ipcContactEnergy.h src/core/contact/ipc/ipcContactEnergy.cpp tests/src/core/contact/ipcContactEnergy_gtest.cpp
git commit -m "refactor: narrow mapped surface energy responsibilities"
```

---

## Task 4: Extract `IPCActiveSetCache`

**Files:**
- Create: `src/core/contact/ipc/ipcActiveSetCache.h`
- Create: `src/core/contact/ipc/ipcActiveSetCache.cpp`
- Modify: `src/core/contact/ipc/ipcContactEnergy.h`
- Modify: `src/core/contact/ipc/ipcContactEnergy.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Create: `tests/src/core/contact/ipcActiveSetCache_gtest.cpp`
- Modify: `tests/src/core/contact/CMakeLists.txt`
- Test: `tests/src/core/contact/ipcContactEnergy_gtest.cpp`

- [ ] **Step 1: Add cache header**

Create `src/core/contact/ipc/ipcActiveSetCache.h`:

```cpp
#pragma once

#include "EigenDef.h"
#include "ipc/core/surfaceIPCActiveSet.h"

#include <functional>

namespace pgo
{
namespace Contact
{
namespace IPC
{

class IPCActiveSetCache
{
public:
  using ActiveSetBuilder = std::function<SurfaceIPCActiveSet(EigenSupport::ConstRefVecXd)>;

  const SurfaceIPCActiveSet &prepareExact(
    EigenSupport::ConstRefVecXd surfacePositions,
    const ActiveSetBuilder &build);

  const SurfaceIPCActiveSet &forEvaluation(
    EigenSupport::ConstRefVecXd surfacePositions,
    const ActiveSetBuilder &build);

  void beginLineSearch(SurfaceIPCActiveSet activeSet);
  void endLineSearch();
  void clearExact();
  void clearAll();
  bool hasExactFor(EigenSupport::ConstRefVecXd surfacePositions) const;

private:
  static bool samePositions(EigenSupport::ConstRefVecXd a, EigenSupport::ConstRefVecXd b);

  bool hasExact_ = false;
  SurfaceIPCActiveSet exact_;
  bool hasLineSearch_ = false;
  SurfaceIPCActiveSet lineSearch_;
};

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 2: Add cache implementation**

Create `src/core/contact/ipc/ipcActiveSetCache.cpp`:

```cpp
#include "ipc/ipcActiveSetCache.h"

namespace pgo
{
namespace Contact
{
namespace IPC
{

bool IPCActiveSetCache::samePositions(EigenSupport::ConstRefVecXd a, EigenSupport::ConstRefVecXd b)
{
  return a.size() == b.size() && (a.array() == b.array()).all();
}

bool IPCActiveSetCache::hasExactFor(EigenSupport::ConstRefVecXd surfacePositions) const
{
  return hasExact_ && samePositions(exact_.positions, surfacePositions);
}

const SurfaceIPCActiveSet &IPCActiveSetCache::prepareExact(
  EigenSupport::ConstRefVecXd surfacePositions,
  const ActiveSetBuilder &build)
{
  exact_ = build(surfacePositions);
  hasExact_ = true;
  return exact_;
}

const SurfaceIPCActiveSet &IPCActiveSetCache::forEvaluation(
  EigenSupport::ConstRefVecXd surfacePositions,
  const ActiveSetBuilder &build)
{
  if (hasLineSearch_) {
    lineSearch_.positions = surfacePositions;
    return lineSearch_;
  }

  if (!hasExactFor(surfacePositions))
    return prepareExact(surfacePositions, build);

  return exact_;
}

void IPCActiveSetCache::beginLineSearch(SurfaceIPCActiveSet activeSet)
{
  clearExact();
  lineSearch_ = std::move(activeSet);
  hasLineSearch_ = true;
}

void IPCActiveSetCache::endLineSearch()
{
  lineSearch_.clear();
  hasLineSearch_ = false;
}

void IPCActiveSetCache::clearExact()
{
  exact_.clear();
  hasExact_ = false;
}

void IPCActiveSetCache::clearAll()
{
  clearExact();
  endLineSearch();
}

}  // namespace IPC
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 3: Add cache unit tests**

Create `tests/src/core/contact/ipcActiveSetCache_gtest.cpp`:

```cpp
#include <gtest/gtest.h>

#include "ipc/ipcActiveSetCache.h"

namespace
{
namespace ES = pgo::EigenSupport;
namespace IPC = pgo::Contact::IPC;

IPC::SurfaceIPCActiveSet makeActiveSet(ES::ConstRefVecXd x)
{
  IPC::SurfaceIPCActiveSet activeSet;
  activeSet.positions = x;
  return activeSet;
}
}  // namespace

TEST(IPCActiveSetCacheGTest, ReusesExactStateAndRebuildsChangedState)
{
  IPC::IPCActiveSetCache cache;
  int builds = 0;
  auto build = [&](ES::ConstRefVecXd x) {
    builds++;
    return makeActiveSet(x);
  };

  ES::VXd x0 = ES::VXd::Zero(3);
  ES::VXd x1 = ES::VXd::Ones(3);

  EXPECT_EQ(&cache.forEvaluation(x0, build), &cache.forEvaluation(x0, build));
  EXPECT_EQ(builds, 1);

  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 2);
}

TEST(IPCActiveSetCacheGTest, LineSearchOverridesExactUntilEnded)
{
  IPC::IPCActiveSetCache cache;
  int builds = 0;
  auto build = [&](ES::ConstRefVecXd x) {
    builds++;
    return makeActiveSet(x);
  };

  ES::VXd x0 = ES::VXd::Zero(3);
  ES::VXd x1 = ES::VXd::Ones(3);
  cache.prepareExact(x0, build);

  IPC::SurfaceIPCActiveSet superset = makeActiveSet(x0);
  cache.beginLineSearch(std::move(superset));

  const IPC::SurfaceIPCActiveSet &lineSearch = cache.forEvaluation(x1, build);
  EXPECT_EQ(lineSearch.positions, x1);
  EXPECT_EQ(builds, 1);

  cache.endLineSearch();
  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 2);
}
```

- [ ] **Step 4: Register cache files in CMake**

In `src/core/contact/CMakeLists.txt`, add:

```cmake
ipc/ipcActiveSetCache.h
```

to `CONTACT_HEADERS`, and:

```cmake
ipc/ipcActiveSetCache.cpp
```

to `CONTACT_SOURCES`.

In `tests/src/core/contact/CMakeLists.txt`, add:

```cmake
add_executable(ipcActiveSetCache_gtest ipcActiveSetCache_gtest.cpp)
target_link_libraries(ipcActiveSetCache_gtest PRIVATE GTest::gtest_main contact)
set_property(TARGET ipcActiveSetCache_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(ipcActiveSetCache_gtest)
```

- [ ] **Step 5: Run new cache tests**

Run:

```bash
cmake --build build/base --target ipcActiveSetCache_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "IPCActiveSetCacheGTest"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 6: Replace cache members in `IPCContactEnergy`**

In `src/core/contact/ipc/ipcContactEnergy.h`, include the cache:

```cpp
#include "ipc/ipcActiveSetCache.h"
```

Remove:

```cpp
void cacheEnergyActiveSet(SurfaceIPCActiveSet activeSet) const;
const SurfaceIPCActiveSet *cachedEnergyActiveSetFor(EigenSupport::ConstRefVecXd surfacePositions) const;
const SurfaceIPCActiveSet &evaluationActiveSetFor(...);
void clearCachedEnergyActiveSet() const;
void clearAllActiveSets() const;
mutable bool hasCachedEnergyActiveSet_ = false;
mutable SurfaceIPCActiveSet cachedEnergyActiveSet_;
mutable bool hasLineSearchActiveSet_ = false;
mutable SurfaceIPCActiveSet lineSearchActiveSet_;
```

Add:

```cpp
const SurfaceIPCActiveSet &activeSetForEvaluation(EigenSupport::ConstRefVecXd surfacePositions) const;
SurfaceIPCActiveSet buildExactActiveSet(EigenSupport::ConstRefVecXd surfacePositions) const;
mutable IPCActiveSetCache activeSetCache_;
```

- [ ] **Step 7: Centralize active-set selection in `IPCContactEnergy`**

In `src/core/contact/ipc/ipcContactEnergy.cpp`, implement:

```cpp
SurfaceIPCActiveSet IPCContactEnergy::buildExactActiveSet(
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  return surfaceIPCCore_.buildActiveSet(surfacePositions);
}

const SurfaceIPCActiveSet &IPCContactEnergy::activeSetForEvaluation(
  EigenSupport::ConstRefVecXd surfacePositions) const
{
  return activeSetCache_.forEvaluation(
    surfacePositions,
    [this](EigenSupport::ConstRefVecXd x) { return buildExactActiveSet(x); });
}
```

Then change every `computeSurface*` method to request the active set once:

```cpp
const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
```

For example:

```cpp
void IPCContactEnergy::computeSurfaceGradient(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient) const
{
  const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
  surfaceIPCCore_.computeGradient(activeSet, surfaceGradient);
}
```

For `computeSurfaceAll()`, keep the local gradient behavior:

```cpp
const SurfaceIPCActiveSet &activeSet = activeSetForEvaluation(surfacePositions);
EigenSupport::VXd localGradient = EigenSupport::VXd::Zero(surfaceGradient.size());
surfaceIPCCore_.computeAll(activeSet, surfaceEnergy, localGradient, surfaceHessian);
surfaceGradient = localGradient;
```

- [ ] **Step 8: Route lifecycle hooks through cache**

In `prepareActiveSet()`:

```cpp
const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);
activeSetCache_.prepareExact(
  surfacePositions,
  [this](EigenSupport::ConstRefVecXd x) { return buildExactActiveSet(x); });
```

In `clearPreparedActiveSet()`:

```cpp
activeSetCache_.clearExact();
```

In `beginActiveSetLineSearch()`:

```cpp
const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);
const VXd trialSurfaceDisplacements =
  computeSurfaceDisplacementsFromSimulationDisplacements(trialSimulationDisplacements);
activeSetCache_.beginLineSearch(
  surfaceIPCCore_.buildLineSearchActiveSetSuperset(surfacePositions, trialSurfaceDisplacements));
```

In `endActiveSetLineSearch()`:

```cpp
activeSetCache_.endLineSearch();
```

In `setMovingObstacleTime()`:

```cpp
activeSetCache_.clearAll();
surfaceIPCCore_.setMovingObstacleTime(t);
```

- [ ] **Step 9: Run IPC cache behavior tests**

Run:

```bash
cmake --build build/base --target ipcActiveSetCache_gtest ipcContactEnergy_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "IPCActiveSetCacheGTest|IPCContactEnergyGTest"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 10: Commit IPC cache extraction**

```bash
git add src/core/contact/ipc/ipcActiveSetCache.h src/core/contact/ipc/ipcActiveSetCache.cpp src/core/contact/ipc/ipcContactEnergy.h src/core/contact/ipc/ipcContactEnergy.cpp src/core/contact/CMakeLists.txt tests/src/core/contact/ipcActiveSetCache_gtest.cpp tests/src/core/contact/CMakeLists.txt tests/src/core/contact/ipcContactEnergy_gtest.cpp
git commit -m "refactor: extract IPC active-set cache"
```

---

## Task 5: Extract Sampled Penalty Active Set RAII And Cache

**Files:**
- Create: `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.h`
- Create: `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.cpp`
- Create: `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.h`
- Create: `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.cpp`
- Create: `src/core/contact/sampled_penalty/sampledPenaltySpecs.h`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Create: `tests/src/core/contact/sampledPenaltyInternals_gtest.cpp`
- Modify: `tests/src/core/contact/CMakeLists.txt`

- [ ] **Step 1: Move sampled penalty specs into a lightweight header**

Create `src/core/contact/sampled_penalty/sampledPenaltySpecs.h`:

```cpp
#pragma once

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

struct ParametersSpec
{
  double stiffness = 1.0;
  int samples = 1;
  bool enableSelfContact = true;
  bool enableExternalContact = true;
};

struct FrictionParametersSpec
{
  double frictionCoeff = 1.0;
  double velocityEps = 1.0;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
```

In `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`, include:

```cpp
#include "sampled_penalty/sampledPenaltySpecs.h"
```

and remove the inline definitions of `ParametersSpec` and `FrictionParametersSpec`.

- [ ] **Step 2: Move active-set RAII into its own type**

Create `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.h`:

```cpp
#pragma once

#include <memory>

namespace pgo
{
namespace Contact
{
class PointPenetrationEnergy;
class PointTrianglePairCouplingEnergyWithCollision;
class PointPenetrationEnergyBuffer;
class PointTrianglePairCouplingEnergyWithCollisionBuffer;

namespace SampledPenalty
{

struct SampledPenaltyActiveSet
{
  std::shared_ptr<PointPenetrationEnergy> externalEnergy;
  PointPenetrationEnergyBuffer *externalBuffer = nullptr;
  std::shared_ptr<PointTrianglePairCouplingEnergyWithCollision> selfEnergy;
  PointTrianglePairCouplingEnergyWithCollisionBuffer *selfBuffer = nullptr;

  SampledPenaltyActiveSet() = default;
  SampledPenaltyActiveSet(const SampledPenaltyActiveSet &) = delete;
  SampledPenaltyActiveSet &operator=(const SampledPenaltyActiveSet &) = delete;
  SampledPenaltyActiveSet(SampledPenaltyActiveSet &&) noexcept = default;
  SampledPenaltyActiveSet &operator=(SampledPenaltyActiveSet &&) noexcept = default;
  ~SampledPenaltyActiveSet();

  bool empty() const;
  void clear();
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
```

Create `src/core/contact/sampled_penalty/sampledPenaltyActiveSet.cpp` by moving the current `SampledPenaltyActiveSet` destructor, `empty()`, and `clear()` logic out of `sampledPenaltyContactEnergy.cpp`.

- [ ] **Step 3: Add active-set cache**

Create `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.h`:

```cpp
#pragma once

#include "EigenDef.h"
#include "sampled_penalty/sampledPenaltyActiveSet.h"

#include <functional>
#include <memory>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

class SampledPenaltyActiveSetCache
{
public:
  using ActiveSetBuilder =
    std::function<std::unique_ptr<SampledPenaltyActiveSet>(EigenSupport::ConstRefVecXd)>;

  const SampledPenaltyActiveSet &prepareExact(
    EigenSupport::ConstRefVecXd x,
    const ActiveSetBuilder &build);
  const SampledPenaltyActiveSet &forEvaluation(
    EigenSupport::ConstRefVecXd x,
    const ActiveSetBuilder &build);
  const SampledPenaltyActiveSet &beginLineSearch(
    EigenSupport::ConstRefVecXd x,
    const ActiveSetBuilder &build);
  void endLineSearch();
  void clearExact();
  void clearAll();
  bool hasExactFor(EigenSupport::ConstRefVecXd x) const;

private:
  static bool sameState(EigenSupport::ConstRefVecXd a, EigenSupport::ConstRefVecXd b);

  std::unique_ptr<SampledPenaltyActiveSet> exact_;
  EigenSupport::VXd exactState_;
  std::unique_ptr<SampledPenaltyActiveSet> lineSearch_;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
```

Create `src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.cpp`:

```cpp
#include "sampled_penalty/sampledPenaltyActiveSetCache.h"

#include <stdexcept>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

bool SampledPenaltyActiveSetCache::sameState(EigenSupport::ConstRefVecXd a, EigenSupport::ConstRefVecXd b)
{
  return a.size() == b.size() && (a.array() == b.array()).all();
}

bool SampledPenaltyActiveSetCache::hasExactFor(EigenSupport::ConstRefVecXd x) const
{
  return exact_ && sameState(exactState_, x);
}

const SampledPenaltyActiveSet &SampledPenaltyActiveSetCache::prepareExact(
  EigenSupport::ConstRefVecXd x,
  const ActiveSetBuilder &build)
{
  exact_ = build(x);
  exactState_ = x;
  return *exact_;
}

const SampledPenaltyActiveSet &SampledPenaltyActiveSetCache::forEvaluation(
  EigenSupport::ConstRefVecXd x,
  const ActiveSetBuilder &build)
{
  if (lineSearch_)
    return *lineSearch_;
  if (!hasExactFor(x))
    return prepareExact(x, build);
  return *exact_;
}

const SampledPenaltyActiveSet &SampledPenaltyActiveSetCache::beginLineSearch(
  EigenSupport::ConstRefVecXd x,
  const ActiveSetBuilder &build)
{
  clearExact();
  lineSearch_ = build(x);
  return *lineSearch_;
}

void SampledPenaltyActiveSetCache::endLineSearch()
{
  lineSearch_.reset();
}

void SampledPenaltyActiveSetCache::clearExact()
{
  exact_.reset();
  exactState_.resize(0);
}

void SampledPenaltyActiveSetCache::clearAll()
{
  clearExact();
  endLineSearch();
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 4: Add cache tests**

Create `tests/src/core/contact/sampledPenaltyInternals_gtest.cpp`:

```cpp
#include <gtest/gtest.h>

#include "sampled_penalty/sampledPenaltyActiveSetCache.h"

namespace
{
namespace ES = pgo::EigenSupport;
namespace SP = pgo::Contact::SampledPenalty;
}

TEST(SampledPenaltyInternalsGTest, ActiveSetCacheReusesExactState)
{
  SP::SampledPenaltyActiveSetCache cache;
  int builds = 0;
  auto build = [&](ES::ConstRefVecXd) {
    builds++;
    return std::make_unique<SP::SampledPenaltyActiveSet>();
  };

  ES::VXd x0 = ES::VXd::Zero(3);
  ES::VXd x1 = ES::VXd::Ones(3);

  EXPECT_EQ(&cache.forEvaluation(x0, build), &cache.forEvaluation(x0, build));
  EXPECT_EQ(builds, 1);
  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 2);
}

TEST(SampledPenaltyInternalsGTest, LineSearchOverridesExactState)
{
  SP::SampledPenaltyActiveSetCache cache;
  int builds = 0;
  auto build = [&](ES::ConstRefVecXd) {
    builds++;
    return std::make_unique<SP::SampledPenaltyActiveSet>();
  };

  ES::VXd x0 = ES::VXd::Zero(3);
  ES::VXd x1 = ES::VXd::Ones(3);

  cache.forEvaluation(x0, build);
  cache.beginLineSearch(x0, build);
  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 2);

  cache.endLineSearch();
  cache.forEvaluation(x1, build);
  EXPECT_EQ(builds, 3);
}
```

- [ ] **Step 5: Register files in CMake**

In `src/core/contact/CMakeLists.txt`, add the new sampled penalty headers/sources to `CONTACT_HEADERS` and `CONTACT_SOURCES`, including `sampled_penalty/sampledPenaltySpecs.h`.

In `tests/src/core/contact/CMakeLists.txt`, add:

```cmake
add_executable(sampledPenaltyInternals_gtest sampledPenaltyInternals_gtest.cpp)
target_link_libraries(sampledPenaltyInternals_gtest PRIVATE GTest::gtest_main contact)
set_property(TARGET sampledPenaltyInternals_gtest PROPERTY FOLDER "tests/gtest")
pgo_gtest_discover_tests(sampledPenaltyInternals_gtest)
```

- [ ] **Step 6: Wire cache into `SampledPenaltyContactEnergy`**

In `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`, replace:

```cpp
mutable std::unique_ptr<SampledPenaltyActiveSet> activeSet_;
mutable EigenSupport::VXd activeSetState_;
mutable bool hasActiveSetState_ = false;
mutable std::unique_ptr<SampledPenaltyActiveSet> lineSearchActiveSet_;
```

with:

```cpp
mutable SampledPenaltyActiveSetCache activeSetCache_;
```

Include:

```cpp
#include "sampled_penalty/sampledPenaltyActiveSetCache.h"
```

In `.cpp`, delete the local `struct SampledPenaltyActiveSet`.

Implement lifecycle methods through the cache:

```cpp
void SampledPenaltyContactEnergy::prepareActiveSet(EigenSupport::ConstRefVecXd x) const
{
  validateStateVector(x);
  activeSetCache_.prepareExact(
    x,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}

void SampledPenaltyContactEnergy::clearPreparedActiveSet() const
{
  activeSetCache_.clearExact();
}

void SampledPenaltyContactEnergy::resetActiveSets() const
{
  activeSetCache_.clearAll();
}

void SampledPenaltyContactEnergy::beginActiveSetLineSearch(EigenSupport::ConstRefVecXd x, EigenSupport::ConstRefVecXd) const
{
  validateStateVector(x);
  activeSetCache_.beginLineSearch(
    x,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}

void SampledPenaltyContactEnergy::endActiveSetLineSearch() const
{
  activeSetCache_.endLineSearch();
}

const SampledPenaltyActiveSet &SampledPenaltyContactEnergy::evaluationActiveSet(EigenSupport::ConstRefVecXd x) const
{
  validateStateVector(x);
  return activeSetCache_.forEvaluation(
    x,
    [this](EigenSupport::ConstRefVecXd state) { return buildActiveSet(state); });
}
```

- [ ] **Step 7: Run sampled penalty internals and behavior tests**

Run:

```bash
cmake --build build/base --target sampledPenaltyInternals_gtest sampledPenaltyContactEnergy_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "SampledPenaltyInternalsGTest|SampledPenaltyContactEnergyGTest"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 8: Commit sampled penalty cache extraction**

```bash
git add src/core/contact/sampled_penalty/sampledPenaltySpecs.h src/core/contact/sampled_penalty/sampledPenaltyActiveSet.h src/core/contact/sampled_penalty/sampledPenaltyActiveSet.cpp src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.h src/core/contact/sampled_penalty/sampledPenaltyActiveSetCache.cpp src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp src/core/contact/CMakeLists.txt tests/src/core/contact/sampledPenaltyInternals_gtest.cpp tests/src/core/contact/CMakeLists.txt
git commit -m "refactor: extract sampled penalty active-set cache"
```

---

## Task 6: Replace Sampled Penalty Friction Inheritance Hooks With State Composition

**Files:**
- Create: `src/core/contact/sampled_penalty/sampledPenaltyFrictionState.h`
- Create: `src/core/contact/sampled_penalty/sampledPenaltyFrictionState.cpp`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `tests/src/core/contact/sampledPenaltyContactEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/sampledPenaltyInternals_gtest.cpp`

- [ ] **Step 1: Add friction state type**

Create `src/core/contact/sampled_penalty/sampledPenaltyFrictionState.h`:

```cpp
#pragma once

#include "EigenDef.h"
#include "sampled_penalty/sampledPenaltySpecs.h"
#include "stepAwareEnergy.h"

namespace pgo
{
namespace Contact
{
class PointPenetrationEnergy;
class PointTrianglePairCouplingEnergyWithCollision;

namespace SampledPenalty
{

class SampledPenaltyFrictionState
{
public:
  explicit SampledPenaltyFrictionState(const FrictionParametersSpec &params);

  void beginStep(const NonlinearOptimization::StepState &state, int expectedDofs);
  void configureExternal(PointPenetrationEnergy &energy, EigenSupport::ConstRefVecXd restPositions) const;
  void configureSelf(PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd restPositions) const;

private:
  FrictionParametersSpec params_;
  EigenSupport::VXd previousX_;
  double timestep_ = 0.0;
  bool hasStepState_ = false;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 2: Implement friction state**

Create `src/core/contact/sampled_penalty/sampledPenaltyFrictionState.cpp`:

```cpp
#include "sampled_penalty/sampledPenaltyFrictionState.h"

#include "sampled_penalty/kernels/pointPenetrationEnergy.h"
#include "sampled_penalty/kernels/pointTrianglePairCouplingEnergyWithCollision.h"

#include <stdexcept>

namespace pgo
{
namespace Contact
{
namespace SampledPenalty
{

SampledPenaltyFrictionState::SampledPenaltyFrictionState(const FrictionParametersSpec &params):
  params_(params)
{
  if (params_.frictionCoeff < 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires non-negative friction coefficient.");
  if (params_.velocityEps <= 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires positive velocity epsilon.");
}

void SampledPenaltyFrictionState::beginStep(const NonlinearOptimization::StepState &state, int expectedDofs)
{
  if (state.previousX == nullptr)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep requires previousX.");
  if (state.timestep <= 0.0)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep requires a positive timestep.");
  if (state.previousX->size() != expectedDofs)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy::beginStep previousX has unexpected size.");

  previousX_ = *state.previousX;
  timestep_ = state.timestep;
  hasStepState_ = true;
}

void SampledPenaltyFrictionState::configureExternal(
  PointPenetrationEnergy &energy,
  EigenSupport::ConstRefVecXd restPositions) const
{
  if (!hasStepState_)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires beginStep before active contact evaluation.");

  energy.setComputeLastPosFunction([this, restPositions](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
    p = previousX_.segment<3>(dofStart) + restPositions.segment<3>(dofStart);
  });
  energy.setFrictionCoeff(params_.frictionCoeff);
  energy.setTimestep(timestep_);
  energy.setVelEps(params_.velocityEps);
}

void SampledPenaltyFrictionState::configureSelf(
  PointTrianglePairCouplingEnergyWithCollision &energy,
  EigenSupport::ConstRefVecXd restPositions) const
{
  if (!hasStepState_)
    throw std::invalid_argument("FrictionalSampledPenaltyContactEnergy requires beginStep before active contact evaluation.");

  energy.setToLastPosFunction([this, restPositions](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
    p = previousX_.segment<3>(dofStart) + restPositions.segment<3>(dofStart);
  });
  energy.setFrictionCoeff(params_.frictionCoeff);
  energy.setTimestep(timestep_);
  energy.setVelEps(params_.velocityEps);
}

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 3: Store optional friction state in base**

In `SampledPenaltyContactEnergy`, remove protected virtual methods:

```cpp
virtual bool hasFrictionStepState() const;
virtual const EigenSupport::VXd *previousStepState() const;
virtual double stepTimestep() const;
virtual double frictionCoeff() const;
virtual double velocityEps() const;
virtual void configureExternalActiveEnergy(...);
virtual void configureSelfActiveEnergy(...);
```

Add private/protected helpers:

```cpp
void configureExternalActiveEnergy(PointPenetrationEnergy &energy) const;
void configureSelfActiveEnergy(PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd x) const;
std::optional<SampledPenaltyFrictionState> frictionState_;
```

Include `<optional>` and `sampledPenaltyFrictionState.h`.

- [ ] **Step 4: Configure normal and frictional energies without virtual dispatch**

In normal `configureExternalActiveEnergy()`:

```cpp
energy.setComputePosFunction([this](const EigenSupport::V3d &u, EigenSupport::V3d &p, int dofStart) {
  p = u + simulationRestPositions_.segment<3>(dofStart);
});
energy.setComputeLastPosFunction([](const EigenSupport::V3d &u, EigenSupport::V3d &p, int) {
  p = u;
});
energy.setCoeff(params_.stiffness);
energy.setFrictionCoeff(0.0);
energy.setTimestep(0.0);
energy.setVelEps(0.0);
if (frictionState_)
  frictionState_->configureExternal(energy, simulationRestPositions_);
```

In normal `configureSelfActiveEnergy()`:

```cpp
energy.setToPosFunction([this](const EigenSupport::V3d &u, EigenSupport::V3d &p, int dofStart) {
  p = u + simulationRestPositions_.segment<3>(dofStart);
});
energy.setToLastPosFunction([](const EigenSupport::V3d &u, EigenSupport::V3d &p, int) {
  p = u;
});
energy.setCoeff(params_.stiffness);
energy.setFrictionCoeff(0.0);
energy.setTimestep(0.0);
energy.setVelEps(0.0);
if (frictionState_)
  frictionState_->configureSelf(energy, simulationRestPositions_);
energy.computeClosestPosition(x.data());
```

- [ ] **Step 5: Simplify frictional subclass**

In `FrictionalSampledPenaltyContactEnergy`, remove these overrides and members:

```cpp
bool hasFrictionStepState() const override;
const EigenSupport::VXd *previousStepState() const override;
double stepTimestep() const override;
double frictionCoeff() const override;
double velocityEps() const override;
void configureExternalActiveEnergy(...) const override;
void configureSelfActiveEnergy(...) const override;
FrictionParametersSpec frictionParams_;
EigenSupport::VXd previousX_;
double timestep_ = 0.0;
bool hasStepState_ = false;
```

Initialize base friction state in the frictional constructor body:

```cpp
frictionState_.emplace(frictionParams);
```

Implement `beginStep()` as:

```cpp
void FrictionalSampledPenaltyContactEnergy::beginStep(const NonlinearOptimization::StepState &state)
{
  frictionState_->beginStep(state, getNumDOFs());
  resetActiveSets();
}
```

- [ ] **Step 6: Run tests and scans**

Run:

```bash
cmake --build build/base --target sampledPenaltyInternals_gtest sampledPenaltyContactEnergy_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "SampledPenaltyInternalsGTest|SampledPenaltyContactEnergyGTest"
rg -n "hasFrictionStepState|previousStepState|stepTimestep|frictionCoeff\\(\\)|velocityEps\\(\\)|configureExternalActiveEnergy.*override|configureSelfActiveEnergy.*override" src/core/contact/sampled_penalty
```

Expected:

```text
100% tests passed
# rg command: no matches for removed protected virtual hook names
```

- [ ] **Step 7: Commit friction composition**

```bash
git add src/core/contact/sampled_penalty/sampledPenaltyFrictionState.h src/core/contact/sampled_penalty/sampledPenaltyFrictionState.cpp src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp src/core/contact/CMakeLists.txt tests/src/core/contact/sampledPenaltyContactEnergy_gtest.cpp tests/src/core/contact/sampledPenaltyInternals_gtest.cpp
git commit -m "refactor: compose sampled penalty friction state"
```

---

## Task 7: Extract `SampledPenaltyContactDetector`

**Files:**
- Create: `src/core/contact/sampled_penalty/sampledPenaltyContactDetector.h`
- Create: `src/core/contact/sampled_penalty/sampledPenaltyContactDetector.cpp`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h`
- Modify: `src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `tests/src/core/contact/sampledPenaltyContactEnergy_gtest.cpp`

- [ ] **Step 1: Add detector type**

Create `src/core/contact/sampled_penalty/sampledPenaltyContactDetector.h`:

```cpp
#pragma once

#include "EigenDef.h"
#include "sampled_penalty/sampledPenaltyActiveSet.h"
#include "sampled_penalty/sampledPenaltySpecs.h"
#include "triMeshGeo.h"

#include <functional>
#include <memory>
#include <vector>

namespace pgo
{
namespace Contact
{
class PointPenetrationEnergy;
class PointTrianglePairCouplingEnergyWithCollision;
class TriangleMeshExternalContactHandler;
class TriangleMeshSelfContactHandler;

namespace SampledPenalty
{

struct SampledPenaltyActiveEnergyConfigurator
{
  std::function<void(PointPenetrationEnergy &)> configureExternal;
  std::function<void(PointTrianglePairCouplingEnergyWithCollision &, EigenSupport::ConstRefVecXd)> configureSelf;
};

class SampledPenaltyContactDetector
{
public:
  SampledPenaltyContactDetector(
    const Mesh::TriMeshGeo &surfaceMesh,
    int simulationDofCount,
    const ParametersSpec &params,
    std::vector<Mesh::TriMeshGeo> externalSurfaces,
    std::vector<int> vertexEmbeddingIndices,
    std::vector<double> vertexEmbeddingWeights);

  void updateExternalSurface(int index, const Mesh::TriMeshGeo &surface);

  std::unique_ptr<SampledPenaltyActiveSet> buildActiveSet(
    EigenSupport::ConstRefVecXd x,
    const SampledPenaltyActiveEnergyConfigurator &configurator) const;

private:
  Mesh::TriMeshGeo surfaceMesh_;
  ParametersSpec params_;
  std::vector<Mesh::TriMeshGeo> externalSurfaces_;
  std::vector<int> vertexEmbeddingIndices_;
  std::vector<double> vertexEmbeddingWeights_;
  std::shared_ptr<TriangleMeshExternalContactHandler> externalHandler_;
  std::shared_ptr<TriangleMeshSelfContactHandler> selfHandler_;
};

}  // namespace SampledPenalty
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 2: Move handler construction and active-set build into detector**

Create `sampledPenaltyContactDetector.cpp` by moving these pieces from `sampledPenaltyContactEnergy.cpp`:

- `makeSurfaceRefs()`
- external/self handler construction
- `updateExternalSurface()`
- contact detection inside `buildActiveSet()`

The detector `buildActiveSet()` must call:

```cpp
configurator.configureExternal(*activeSet->externalEnergy);
configurator.configureSelf(*activeSet->selfEnergy, x);
```

after allocating buffers, exactly where the current energy class calls `configureExternalActiveEnergy()` and `configureSelfActiveEnergy()`.

- [ ] **Step 3: Replace handler members in energy**

In `SampledPenaltyContactEnergy`, remove:

```cpp
Mesh::TriMeshGeo surfaceMesh_;
std::vector<Mesh::TriMeshGeo> externalSurfaces_;
std::vector<int> vertexEmbeddingIndices_;
std::vector<double> vertexEmbeddingWeights_;
std::shared_ptr<TriangleMeshExternalContactHandler> externalHandler_;
std::shared_ptr<TriangleMeshSelfContactHandler> selfHandler_;
```

Add:

```cpp
SampledPenaltyContactDetector detector_;
```

In the constructor initializer list, initialize `detector_` with:

```cpp
detector_(
  surfaceMesh,
  static_cast<int>(simulationRestPositions.size()),
  params,
  std::move(externalSurfaces),
  std::move(vertexEmbeddingIndices),
  std::move(vertexEmbeddingWeights))
```

- [ ] **Step 4: Route build and update through detector**

In `SampledPenaltyContactEnergy::updateExternalSurface()`:

```cpp
detector_.updateExternalSurface(index, surface);
resetActiveSets();
```

In `SampledPenaltyContactEnergy::buildActiveSet()`:

```cpp
SampledPenaltyActiveEnergyConfigurator configurator;
configurator.configureExternal = [this](PointPenetrationEnergy &energy) {
  configureExternalActiveEnergy(energy);
};
configurator.configureSelf = [this](PointTrianglePairCouplingEnergyWithCollision &energy, EigenSupport::ConstRefVecXd x) {
  configureSelfActiveEnergy(energy, x);
};
return detector_.buildActiveSet(x, configurator);
```

- [ ] **Step 5: Run sampled penalty tests**

Run:

```bash
cmake --build build/base --target sampledPenaltyContactEnergy_gtest sampledPenaltyInternals_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "SampledPenaltyContactEnergyGTest|SampledPenaltyInternalsGTest"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 6: Commit detector extraction**

```bash
git add src/core/contact/sampled_penalty/sampledPenaltyContactDetector.h src/core/contact/sampled_penalty/sampledPenaltyContactDetector.cpp src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.h src/core/contact/sampled_penalty/sampledPenaltyContactEnergy.cpp src/core/contact/CMakeLists.txt tests/src/core/contact/sampledPenaltyContactEnergy_gtest.cpp
git commit -m "refactor: extract sampled penalty detector"
```

---

## Task 8: Clean Public Contact Factory Boundary

**Files:**
- Modify: `src/core/contact/contactEnergyFactory.h`
- Modify: `src/core/contact/contactEnergyFactory.cpp`
- Modify: `src/python/pypgo/bindings/contact_bindings.cpp`
- Modify: `tests/src/core/contact/contactEnergyFactory_gtest.cpp`
- Modify: `tests/pypgo/test_contact.py`

- [ ] **Step 1: Add facade specs in public factory header**

In `src/core/contact/contactEnergyFactory.h`, remove:

```cpp
#include "ipc/core/surfaceIPCCore.h"
using ParametersSpec = SurfaceIPCCore::Parameters;
```

Add public facade specs:

```cpp
struct IPCContactSpec
{
  double dhat = 1e-1;
  double dhatExternal = 1e-1;
  double kappa = 0.1;
  double epsEE = 0.0;
  double slackness = 1.0;
  double ccdThickness = 0.0;
};

struct SampledPenaltyContactSpec
{
  double stiffness = 1.0;
  int samples = 1;
  bool enableSelfContact = true;
  bool enableExternalContact = true;
};

struct FrictionContactSpec
{
  double frictionCoeff = 1.0;
  double velocityEps = 1.0;
};
```

Keep old nested concrete specs only in `.cpp` or concrete headers. The public factory header should not include `surfaceIPCCore.h`.

- [ ] **Step 2: Return contact boundary from factory functions without changing namespace layout**

Keep the current namespace layout to limit API churn:

- `Contact::IPC::createIPCEnergy(...)`
- `Contact::SampledPenalty::createSampledPenaltyEnergy(...)`
- `Contact::SampledPenalty::createFrictionalSampledPenaltyEnergy(...)`

Change declarations inside those existing namespaces to:

```cpp
std::shared_ptr<StatefulContactEnergy> createIPCEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const IPCContactSpec &params,
  std::vector<ObstacleSpec> obstacles = {});

std::shared_ptr<StatefulContactEnergy> createSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const SampledPenaltyContactSpec &params);

std::shared_ptr<StatefulContactEnergy> createFrictionalSampledPenaltyEnergy(
  const ContactSurfaceSpec &surface,
  const EigenSupport::MXi &surfaceTriangles,
  const SampledPenaltyContactSpec &params,
  const FrictionContactSpec &friction);
```

Keep `createFloorEnergy()` returning `std::shared_ptr<PotentialEnergy>` because floor is not active-set stateful.

- [ ] **Step 3: Convert facade specs in `.cpp`**

In `contactEnergyFactory.cpp`, include concrete headers:

```cpp
#include "ipc/core/surfaceIPCCore.h"
```

Add converters:

```cpp
IPC::SurfaceIPCCore::Parameters toIPCParameters(const IPCContactSpec &spec)
{
  IPC::SurfaceIPCCore::Parameters params;
  params.dhat = spec.dhat;
  params.dhat_external = spec.dhatExternal;
  params.kappa = spec.kappa;
  params.eps_ee = spec.epsEE;
  params.slackness = spec.slackness;
  params.ccd_thickness = spec.ccdThickness;
  return params;
}

SampledPenalty::ParametersSpec toSampledPenaltyParameters(const SampledPenaltyContactSpec &spec)
{
  SampledPenalty::ParametersSpec params;
  params.stiffness = spec.stiffness;
  params.samples = spec.samples;
  params.enableSelfContact = spec.enableSelfContact;
  params.enableExternalContact = spec.enableExternalContact;
  return params;
}

SampledPenalty::FrictionParametersSpec toFrictionParameters(const FrictionContactSpec &spec)
{
  SampledPenalty::FrictionParametersSpec friction;
  friction.frictionCoeff = spec.frictionCoeff;
  friction.velocityEps = spec.velocityEps;
  return friction;
}
```

The factory definitions must remain in the existing nested namespaces:

```cpp
namespace IPC
{
std::shared_ptr<StatefulContactEnergy> createIPCEnergy(...);
}  // namespace IPC

namespace SampledPenalty
{
std::shared_ptr<StatefulContactEnergy> createSampledPenaltyEnergy(...);
std::shared_ptr<StatefulContactEnergy> createFrictionalSampledPenaltyEnergy(...);
}  // namespace SampledPenalty
```

- [ ] **Step 4: Make sampled penalty backend limit explicit**

Add helper:

```cpp
void requireSurfaceIdentitySizedMap(const ContactSurfaceSpec &surface, const char *backendName)
{
  if (surface.surfaceFromSimulationDispMap.rows() != surface.restVertices.rows() * 3 ||
    surface.surfaceFromSimulationDispMap.cols() != surface.restVertices.rows() * 3) {
    throw std::invalid_argument(std::string(backendName) +
      " currently requires a surface-identity-sized ContactSurfaceSpec.");
  }
}
```

Use it in both sampled penalty factory functions. Do not silently accept non-square maps.

- [ ] **Step 5: Update C++ and Python binding call sites**

In `src/python/pypgo/bindings/contact_bindings.cpp`, construct facade specs instead of concrete specs when calling factory functions.

For IPC:

```cpp
CT::IPCContactSpec params;
params.dhat = dhat;
params.dhatExternal = dhatExternal;
params.kappa = kappa;
```

For sampled penalty:

```cpp
CT::SampledPenaltyContactSpec params;
params.stiffness = stiffness;
params.samples = samples;
params.enableSelfContact = enableSelfContact;
params.enableExternalContact = enableExternalContact;
```

For friction:

```cpp
CT::FrictionContactSpec friction;
friction.frictionCoeff = frictionCoeff;
friction.velocityEps = velocityEps;
```

If binding still needs concrete methods such as `setMovingObstacleTime`, keep that logic inside `PyStatefulContactEnergy::setMovingObstacleTime()` using:

```cpp
if (auto *ipc = dynamic_cast<CT::IPC::IPCContactEnergy *>(energy_.get())) {
  ipc->setMovingObstacleTime(t);
  return;
}
throw std::invalid_argument("set_moving_obstacle_time is only available for IPC contact energy.");
```

- [ ] **Step 6: Add factory boundary tests**

In `contactEnergyFactory_gtest.cpp`, add:

```cpp
TEST(ContactEnergyFactoryGTest, PublicFactoryReturnsStatefulContactBoundary)
{
  const auto surface = makeIdentitySurfaceSpec(makeSingleTriangleVertices());
  const EigenSupport::MXi triangles = makeSingleTriangleIndices();

  Contact::IPCContactSpec ipcParams;
  std::shared_ptr<Contact::StatefulContactEnergy> ipc =
    Contact::IPC::createIPCEnergy(surface, triangles, ipcParams);
  EXPECT_EQ(ipc->contactModelKind(), Contact::ContactModelKind::IPC);

  Contact::SampledPenaltyContactSpec penaltyParams;
  std::shared_ptr<Contact::StatefulContactEnergy> penalty =
    Contact::SampledPenalty::createSampledPenaltyEnergy(surface, triangles, penaltyParams);
  EXPECT_EQ(penalty->contactModelKind(), Contact::ContactModelKind::SampledPenalty);
}
```

Add a compile-time boundary check by ensuring this test includes only:

```cpp
#include "contactEnergyFactory.h"
```

and not `ipc/core/surfaceIPCCore.h`.

- [ ] **Step 7: Run factory and Python tests**

Run:

```bash
cmake --build build/base --target contactEnergyFactory_gtest pypgo_core -j 8
ctest --test-dir build/base --output-on-failure -R "ContactEnergyFactoryGTest"
PYTHONPATH="$PWD" pytest tests/pypgo/test_contact.py
```

Expected:

```text
100% tests passed
tests/pypgo/test_contact.py ... passed
```

- [ ] **Step 8: Scan public header dependency**

Run:

```bash
rg -n "surfaceIPCCore.h|SurfaceIPCCore::Parameters|using ParametersSpec" src/core/contact/contactEnergyFactory.h
```

Expected:

```text
# no matches
```

- [ ] **Step 9: Commit factory cleanup**

```bash
git add src/core/contact/contactEnergyFactory.h src/core/contact/contactEnergyFactory.cpp src/python/pypgo/bindings/contact_bindings.cpp tests/src/core/contact/contactEnergyFactory_gtest.cpp tests/pypgo/test_contact.py
git commit -m "refactor: narrow contact factory facade"
```

---

## Task 9: Extract Broad-Phase Hash Query Skeleton

**Files:**
- Modify: `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhaseInternal.h`
- Modify: `src/core/contact/ipc/broadPhase/surfaceIPCSelfBroadPhase.cpp`
- Modify: `src/core/contact/ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp`
- Modify: `tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp`
- Modify: `tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest.cpp`

- [ ] **Step 1: Add characterization tests for pair contents**

Before refactoring broad phase, add tests that compare normal and line-search superset behavior on fixed tiny meshes.

In `surfaceIPCSelfBroadPhase_gtest.cpp`, add a test that:

```cpp
SelfPairSet normalPairs;
SelfPairSet supersetPairs;
buildSelfPairs(topology, positions, dhat, normalPairs);
buildSelfPairsLineSearchSuperset(topology, positions, EigenSupport::VXd::Zero(positions.size()), dhat, supersetPairs);

EXPECT_EQ(normalPairs.ptPairs.size(), supersetPairs.ptPairs.size());
EXPECT_EQ(normalPairs.eePairs.size(), supersetPairs.eePairs.size());
```

Use existing topology fixture helpers from the file. If no helper exists, keep the test local and construct a two-triangle mesh with one non-incident PT candidate and one non-adjacent EE candidate.

In `surfaceIPCExternalBroadPhase_gtest.cpp`, add an equivalent test for `ExternalPairSet` with one static obstacle.

- [ ] **Step 2: Run characterization tests**

Run:

```bash
cmake --build build/base --target surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalBroadPhase_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "SurfaceIPCSelfBroadPhase|SurfaceIPCExternalBroadPhase"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 3: Add profile names struct**

In `surfaceIPCBroadPhaseInternal.h`, add:

```cpp
struct PairQueryProfileNames
{
  std::string_view hashCandidates;
  std::string_view exactTests;
  std::string_view acceptedPairs;
};

inline void recordPairQueryCounters(
  const PairQueryProfileNames &names,
  const PairQueryCounts &counts,
  std::size_t acceptedPairCount)
{
  recordPairQueryCounters(
    names.hashCandidates,
    names.exactTests,
    names.acceptedPairs,
    counts,
    acceptedPairCount);
}
```

Keep the existing overload so this change is purely additive.

- [ ] **Step 4: Add generic hash query collection helper**

In `surfaceIPCBroadPhaseInternal.h`, add:

```cpp
template<typename PairType, typename QueryBody>
PairQueryCounts collectHashPairsParallel(
  int nTarget,
  int queryBegin,
  int queryEnd,
  QueryBody &&queryBody,
  std::vector<PairType> &outputPairs)
{
  return collectPairsParallel<PairType>(
    nTarget,
    queryBegin,
    queryEnd,
    std::forward<QueryBody>(queryBody),
    outputPairs);
}
```

This step intentionally wraps the existing helper first. Do not change call sites yet.

- [ ] **Step 5: Convert self PT and EE query sites**

In `surfaceIPCSelfBroadPhase.cpp`, replace direct calls to `collectPairsParallel<PTPair>` and `collectPairsParallel<EEPair>` with `collectHashPairsParallel`.

Also replace counter recording with `PairQueryProfileNames`:

```cpp
const PairQueryProfileNames ptProfileNames{
  SurfaceIPCProfileSections::kPairBuildSelfPTHashCandidates,
  SurfaceIPCProfileSections::kPairBuildSelfPTDistanceTests,
  SurfaceIPCProfileSections::kPairBuildSelfPTAcceptedPairs,
};
recordPairQueryCounters(ptProfileNames, counts, pairs.ptPairs.size());
```

Do not change distance predicates, topology rejection, or pair construction.

- [ ] **Step 6: Convert external PT/TP/EE query sites**

In `surfaceIPCExternalBroadPhase.cpp`, replace direct calls to `collectPairsParallel<ExternalPTPair>`, `collectPairsParallel<ExternalTPPair>`, and `collectPairsParallel<ExternalEEPair>` with `collectHashPairsParallel`.

Use `PairQueryProfileNames` for all six normal/line-search query sites:

```cpp
const PairQueryProfileNames externalPTNames{
  SurfaceIPCProfileSections::kPairBuildExternalPTHashCandidates,
  SurfaceIPCProfileSections::kPairBuildExternalPTDistanceTests,
  SurfaceIPCProfileSections::kPairBuildExternalPTAcceptedPairs,
};
```

For appended external pairs, preserve `acceptedBefore` and call:

```cpp
recordPairQueryCounters(externalPTNames, counts, pairs.ptPairs.size() - acceptedBefore);
```

- [ ] **Step 7: Run broad phase tests**

Run:

```bash
cmake --build build/base --target surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalBroadPhase_gtest surfaceIPCCore_gtest ipcContactEnergy_gtest -j 8
ctest --test-dir build/base --output-on-failure -R "SurfaceIPCSelfBroadPhase|SurfaceIPCExternalBroadPhase|SurfaceIPCCoreGTest|IPCContactEnergyGTest"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 8: Commit query skeleton extraction**

```bash
git add src/core/contact/ipc/broadPhase/surfaceIPCBroadPhaseInternal.h src/core/contact/ipc/broadPhase/surfaceIPCSelfBroadPhase.cpp src/core/contact/ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest.cpp
git commit -m "refactor: share IPC broad-phase query skeleton"
```

---

## Task 10: Final Integration Verification

**Files:**
- Inspect: all files modified by Tasks 1-9

- [ ] **Step 1: Build full base target**

Run:

```bash
cmake --build build/base -j 8
```

Expected:

```text
# command exits 0
```

Existing Eigen / duplicate-library warnings are acceptable if they match current baseline.

- [ ] **Step 2: Run focused contact and evaluation tests**

Run:

```bash
ctest --test-dir build/base --output-on-failure -R "Contact|Evaluation|contact|evaluation"
```

Expected:

```text
100% tests passed
```

- [ ] **Step 3: Run Python contact tests**

Run:

```bash
PYTHONPATH="$PWD" pytest tests/pypgo/test_contact.py
```

Expected:

```text
tests/pypgo/test_contact.py ... passed
```

- [ ] **Step 4: Run cleanup scans**

Run:

```bash
rg -n "refreshActiveSet|clearActiveSet|refresh_active_set|clear_active_set" src pypgo tests -g '!build'
rg -n "public virtual|virtual .*StatefulContactEnergy" src/core/contact src/python/pypgo/bindings/contact_bindings.cpp
rg -n "surfaceIPCCore.h|SurfaceIPCCore::Parameters|using ParametersSpec" src/core/contact/contactEnergyFactory.h
rg -n "hasFrictionStepState|previousStepState|stepTimestep|frictionCoeff\\(\\)|velocityEps\\(\\)" src/core/contact/sampled_penalty
git diff --check
```

Expected:

```text
# lifecycle scan: only negative hasattr tests, or no matches
# virtual inheritance scan: no matches
# factory public header scan: no matches
# sampled friction hook scan: no matches
# git diff --check: no output
```

- [ ] **Step 5: Commit final integration**

If Task commits were made individually and no integration changes remain, skip this step. Otherwise:

```bash
git add src/core/contact tests/src/core/contact src/python/pypgo/bindings/contact_bindings.cpp tests/pypgo/test_contact.py
git commit -m "test: verify contact internals refactor"
```

---

## Rollback And Drift Rules

- If any task changes numerical outputs, pair counts, or pair ordering unexpectedly, stop and report before adjusting tests.
- If `IPCActiveSetCache` extraction changes active-set build count in `IPCContactEnergyGTest`, stop and inspect cache lifecycle before continuing.
- If sampled penalty frictional tests fail because `beginStep()` no longer gates frictional evaluation, stop and restore the explicit `beginStep()` validation in `SampledPenaltyFrictionState`.
- If public factory cleanup requires changing Python public class names or `pypgo.contact.__all__`, stop and split that into a separate Python API migration plan.
- If broad-phase refactor requires moving distance predicates or changing AABB inflation semantics, stop and split that into a separate IPC broad-phase algorithm plan.

## Self Review

### Spec Coverage

- D2 lifecycle decision is covered by Task 1 and final cleanup scans.
- D3 no-virtual-inheritance decision is covered by Task 1 and final cleanup scans.
- D4 mapped-surface responsibility and Hessian contract are covered by Tasks 2 and 3.
- D5 IPC active-set cache extraction is covered by Task 4.
- D6 sampled penalty glue cleanup is covered by Tasks 5, 6, and 7.
- Public factory boundary cleanup is covered by Task 8.
- Broad-phase duplication reduction is covered by Task 9.

### Placeholder Scan

The plan contains no placeholder markers or open implementation choices. `sampledPenaltySpecs.h` is now an explicit Task 5 artifact, so Task 6 and Task 7 do not require implementers to resolve a header include cycle.

### Type Consistency

- `IPCActiveSetCache` uses `SurfaceIPCActiveSet` and exact position matching consistently in header, implementation, and tests.
- `SampledPenaltyActiveSetCache` uses `std::unique_ptr<SampledPenaltyActiveSet>` consistently.
- `SampledPenaltyFrictionState` uses `FrictionParametersSpec` and `StepState` consistently.
- Factory facade specs are named `IPCContactSpec`, `SampledPenaltyContactSpec`, and `FrictionContactSpec` consistently across declaration, conversion, and binding steps.

### Residual Risks

- Task 6 relies on the Task 5 `sampledPenaltySpecs.h` split. If an implementer skips Task 5, Task 6 will create an include cycle; execute tasks in order.
- Task 9 intentionally starts with a wrapper extraction instead of a full strategy rewrite; it reduces duplication in counters/query collection but does not eliminate all normal vs line-search duplication in one step.
- Broad-phase tests must preserve pair contents and ordering. If existing tests assert only counts, implementers should add content comparisons before changing query bodies.
