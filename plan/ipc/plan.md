# Stateless Surface IPC Active Set Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task in this Codex session. Do not use subagents unless the user explicitly asks for parallel agent work. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove cross-call `SurfaceIPCCore` prepared-pair cache and replace it with stateless, per-evaluation `SurfaceIPCActiveSet` objects.

**Architecture:** `SurfaceIPCCore` no longer owns mutable active-pair state. Each public `compute*(x)` entry point builds a local active set, while expert callers can explicitly build and reuse a caller-owned `SurfaceIPCActiveSet` whose `positions` snapshot is consumed by active-set compute overloads. Wrappers call stateless surface APIs directly, so same-state `func/gradient/hessianDirect` no longer share hidden cache state.

**Tech Stack:** C++17, Eigen, Sparse Matrix assembly, GoogleTest, CMake, existing IPC broad-phase and barrier assembler helpers.

---

## Review Amendments Before Execution

- The user explicitly requested **no commits**. Every former commit checkpoint in this plan is now a local diff checkpoint: inspect status/diff as useful, but do not run `git commit`.
- Use a test-first implementation order where feasible. Update the tests that describe the new active-set/fused API expectations before production code, then run a focused build and confirm the expected failure is caused by missing/renamed API or profiling symbols. After that, implement the production changes and make the tests pass.
- Keep all edits in the current checkout on `feat/ipc-external-contact`; the workspace was clean at review time.
- Final removed-symbol scans must gate `src` and `tests`. Plan documents may intentionally mention removed symbols as historical context and should not fail the implementation check.

---

## Design Contract

The final public core API should look like this:

```cpp
struct SurfaceIPCActiveSet
{
  EigenSupport::VXd positions;
  SelfPairSet selfPairs;
  ExternalPairSet externalPairs;

  void clear();
  std::size_t size() const;
};

class SurfaceIPCCore
{
public:
  SurfaceIPCActiveSet buildActiveSet(EigenSupport::ConstRefVecXd x_surf) const;

  double computeEnergy(EigenSupport::ConstRefVecXd x_surf) const;
  void computeGradient(EigenSupport::ConstRefVecXd x_surf, EigenSupport::RefVecXd g_surf) const;
  void computeHessian(EigenSupport::ConstRefVecXd x_surf, EigenSupport::SpMatD &H_surf) const;
  void computeAll(EigenSupport::ConstRefVecXd x_surf, double &energy, EigenSupport::VXd &g_surf, EigenSupport::SpMatD &H_surf) const;

  double computeEnergy(const SurfaceIPCActiveSet &activeSet) const;
  void computeGradient(const SurfaceIPCActiveSet &activeSet, EigenSupport::RefVecXd g_surf) const;
  void computeHessian(const SurfaceIPCActiveSet &activeSet, EigenSupport::SpMatD &H_surf) const;
  void computeAll(const SurfaceIPCActiveSet &activeSet, double &energy, EigenSupport::VXd &g_surf, EigenSupport::SpMatD &H_surf) const;
};
```

Delete these prepared-state APIs and storage:

```cpp
void prepareForSurfacePositions(EigenSupport::ConstRefVecXd x_surf) const;
double computeEnergyWithPreparedPairs() const;
void computeGradientWithPreparedPairs(EigenSupport::RefVecXd g_surf) const;
void computeHessianWithPreparedPairs(EigenSupport::SpMatD &H_surf) const;
void computeAllWithPreparedPairs(double &energy, EigenSupport::VXd &g_surf, EigenSupport::SpMatD &H_surf) const;
void invalidatePreparedState() const;
const SurfaceIPCPreparedState &preparedState() const;
mutable SurfaceIPCPreparedState preparedState_;
```

`SurfaceIPCActiveSet` is an implementation detail of the core layer and must **not** leak into wrapper public APIs. Wrappers instead expose fused entry points that internally build one active set and consume it once:

```cpp
class CIPCPotentialEnergy : public NonlinearOptimization::PotentialEnergy
{
public:
  // Existing virtual overrides (unchanged signatures), now internally stateless.
  // CIPC is maintained as stateless but unfused — no new entry points are added.
  double func(EigenSupport::ConstRefVecXd x) const override;
  void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  void hessianDirect(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
};

class MappedSurfacePotentialEnergy : public NonlinearOptimization::PotentialEnergy
{
public:
  // New: fused entry points that map simulation displacements once and pull
  // back the requested quantities once around a single computeSurface{FuncGrad,All}
  // hook call. Solvers (Newton, etc.) call these to amortize broad-phase.
  double func_grad(EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient) const override;
  double func_grad_hessian(EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient, EigenSupport::SpMatD &simulationHessian) const override;
  void gradient_hessian(EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient, EigenSupport::SpMatD &simulationHessian) const;

protected:
  // New virtual hooks with default impls that call the existing per-quantity
  // pure virtuals. Subclasses opt into a fused path by overriding the hook
  // that matches their solver call pattern.
  virtual void computeSurfaceFuncGrad(EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy, EigenSupport::RefVecXd surfaceGradient) const;
  virtual void computeSurfaceAll(EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy, EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;
};

class EmbeddedSurfaceIPCPotentialEnergy : public MappedSurfacePotentialEnergy
{
private:
  // Existing virtuals (unchanged signatures), now internally stateless:
  double computeSurfaceEnergy(...) const override;
  void computeSurfaceGradient(...) const override;
  void computeSurfaceHessian(...) const override;

  // New: opt into the fused paths.
  // - computeSurfaceFuncGrad: one core.buildActiveSet + computeEnergy + computeGradient
  //   (two kernel passes sharing one broad-phase build). Used by func_grad.
  // - computeSurfaceAll: one core.buildActiveSet + computeAll (single fused kernel pass).
  //   Used by func_grad_hessian and gradient_hessian.
  void computeSurfaceFuncGrad(...) const override;
  void computeSurfaceAll(...) const override;
};
```

The important invariant at the core layer is:

```cpp
// Correct: positions and pairs are created together and consumed together.
const SurfaceIPCActiveSet activeSet = core.buildActiveSet(x);
const double energy = core.computeEnergy(activeSet);

// No core API should accept both x and activeSet.
```

The wrapper layer never exposes `SurfaceIPCActiveSet`. Reuse at the wrapper level happens through the fused methods (`func_grad`, `func_grad_hessian`, `gradient_hessian`), not through caller-managed active-set objects.

## File Map

- Create: `src/core/contact/ipc/core/surfaceIPCActiveSet.h`
- Delete: `src/core/contact/ipc/core/surfaceIPCPreparedState.h`
- Modify: `src/core/contact/ipc/core/surfaceIPCCore.h`
- Modify: `src/core/contact/ipc/core/surfaceIPCCore.cpp`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `src/core/contact/CIPC.h`
- Modify: `src/core/contact/CIPC.cpp`
- Modify: `src/core/contact/mappedSurfacePotentialEnergy.h`
- Modify: `src/core/contact/mappedSurfacePotentialEnergy.cpp`
- Modify: `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h`
- Modify: `src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp`
- Modify: `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`
- Modify: `src/tools/runSim/runIPCSim.cpp`
- Modify: `tests/src/core/contact/surfaceIPCCore_gtest.cpp`
- Modify: `tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp`
- Modify: `tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest.cpp`
- Modify: `tests/src/core/contact/cipcPotentialEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/cipcProfiling_gtest.cpp`
- Modify: `src/tests/testIPCExternal/testIPCExternal.cpp`

## Task 1: Introduce `SurfaceIPCActiveSet`

**Files:**
- Create: `src/core/contact/ipc/core/surfaceIPCActiveSet.h`
- Modify: `src/core/contact/CMakeLists.txt`
- Modify: `src/core/contact/ipc/core/surfaceIPCCore.h`

- [ ] **Step 1: Add the active-set value type**

Create `src/core/contact/ipc/core/surfaceIPCActiveSet.h`:

```cpp
#pragma once

#include "EigenDef.h"
#include "ipc/core/surfaceIPCPairs.h"

#include <cstddef>

namespace pgo
{
namespace Contact
{
namespace CIPC
{

struct SurfaceIPCActiveSet
{
  EigenSupport::VXd positions;
  SelfPairSet selfPairs;
  ExternalPairSet externalPairs;

  void clear()
  {
    positions.resize(0);
    selfPairs.clear();
    externalPairs.clear();
  }

  std::size_t size() const
  {
    return selfPairs.size() + externalPairs.size();
  }
};

}  // namespace CIPC
}  // namespace Contact
}  // namespace pgo
```

- [ ] **Step 2: Register the header in CMake**

In `src/core/contact/CMakeLists.txt`, replace:

```cmake
  ipc/core/surfaceIPCPreparedState.h
```

with:

```cmake
  ipc/core/surfaceIPCActiveSet.h
```

- [ ] **Step 3: Update the core header include and public API declarations**

In `src/core/contact/ipc/core/surfaceIPCCore.h`, replace:

```cpp
#include "ipc/core/surfaceIPCPreparedState.h"
```

with:

```cpp
#include "ipc/core/surfaceIPCActiveSet.h"
```

Then replace the prepared-state members and methods with:

```cpp
  SurfaceIPCActiveSet buildActiveSet(EigenSupport::ConstRefVecXd x_surf) const;

  double computeEnergy(EigenSupport::ConstRefVecXd x_surf) const;
  void computeGradient(EigenSupport::ConstRefVecXd x_surf, EigenSupport::RefVecXd g_surf) const;
  void computeHessian(EigenSupport::ConstRefVecXd x_surf, EigenSupport::SpMatD &H_surf) const;
  void computeAll(EigenSupport::ConstRefVecXd x_surf, double &energy, EigenSupport::VXd &g_surf, EigenSupport::SpMatD &H_surf) const;

  double computeEnergy(const SurfaceIPCActiveSet &activeSet) const;
  void computeGradient(const SurfaceIPCActiveSet &activeSet, EigenSupport::RefVecXd g_surf) const;
  void computeHessian(const SurfaceIPCActiveSet &activeSet, EigenSupport::SpMatD &H_surf) const;
  void computeAll(const SurfaceIPCActiveSet &activeSet, double &energy, EigenSupport::VXd &g_surf, EigenSupport::SpMatD &H_surf) const;
```

Delete these declarations from the same header:

```cpp
  void prepareForSurfacePositions(EigenSupport::ConstRefVecXd x_surf) const;
  double computeEnergyWithPreparedPairs() const;
  void computeGradientWithPreparedPairs(EigenSupport::RefVecXd g_surf) const;
  void computeHessianWithPreparedPairs(EigenSupport::SpMatD &H_surf) const;
  void computeAllWithPreparedPairs(double &energy, EigenSupport::VXd &g_surf, EigenSupport::SpMatD &H_surf) const;
  void invalidatePreparedState() const;
  const SurfaceIPCPreparedState& preparedState() const { return preparedState_; }
```

Delete this private member:

```cpp
  mutable SurfaceIPCPreparedState preparedState_;
```

Delete the private broad-phase helper declaration entirely (its logic moves directly into `buildActiveSet()` in Task 2):

```cpp
  void findCollisionPairs(const EigenSupport::VXd &positions) const;
```

- [ ] **Step 4: Compile to capture expected failures**

Run:

```bash
cmake --build build/base_no_mkl --target surfaceIPCCore_gtest -j2
```

Expected: compile failure in `surfaceIPCCore.cpp` and tests because the implementation still references `preparedState_`, `prepareForSurfacePositions()`, and `preparedState()`.

- [ ] **Step 5: Local diff checkpoint**

```bash
git diff -- src/core/contact/ipc/core/surfaceIPCActiveSet.h src/core/contact/ipc/core/surfaceIPCCore.h src/core/contact/CMakeLists.txt
```

Do not stage or commit. This checkpoint exists only to inspect the local diff.

## Task 2: Rework `SurfaceIPCCore` to Build Local Active Sets

**Files:**
- Modify: `src/core/contact/ipc/core/surfaceIPCCore.cpp`
- Modify: `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`
- Delete: `src/core/contact/ipc/core/surfaceIPCPreparedState.h`

- [ ] **Step 1: Rename profiling sections away from prepared-state wording**

In `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`, replace:

```cpp
inline constexpr std::string_view kPrepareActivePairs = "contact.surface.prepare_active_pairs";
inline constexpr std::string_view kPreparedEnergy = "contact.surface.prepared_energy";
inline constexpr std::string_view kPreparedGradient = "contact.surface.prepared_gradient";
inline constexpr std::string_view kPreparedHessian = "contact.surface.prepared_hessian";
```

with:

```cpp
inline constexpr std::string_view kBuildActiveSet = "contact.surface.build_active_set";
inline constexpr std::string_view kActiveSetEnergy = "contact.surface.active_set_energy";
inline constexpr std::string_view kActiveSetGradient = "contact.surface.active_set_gradient";
inline constexpr std::string_view kActiveSetHessian = "contact.surface.active_set_hessian";
inline constexpr std::string_view kActiveSetCombined = "contact.surface.active_set_combined";
```

- [ ] **Step 2: Remove prepared-state copying from copy operations**

In `src/core/contact/ipc/core/surfaceIPCCore.cpp`, update the copy constructor to omit `preparedState_`:

```cpp
SurfaceIPCCore::SurfaceIPCCore(const SurfaceIPCCore &other):
  dhat(other.dhat),
  dhat_external(other.dhat_external),
  kappa(other.kappa),
  eps_ee(other.eps_ee),
  slackness(other.slackness),
  topology_(other.topology_),
  obstacles_(other.obstacles_)
{
}
```

Update assignment to omit `preparedState_`:

```cpp
SurfaceIPCCore &SurfaceIPCCore::operator=(const SurfaceIPCCore &other)
{
  if (this == &other)
    return *this;
  dhat = other.dhat;
  dhat_external = other.dhat_external;
  kappa = other.kappa;
  eps_ee = other.eps_ee;
  slackness = other.slackness;
  topology_ = other.topology_;
  obstacles_ = other.obstacles_;
  return *this;
}
```

- [ ] **Step 3: Remove cache invalidation from mutators**

In `setParameters()`, `setMesh()`, `setObstacles()`, and `updateObstacleStage()`, delete:

```cpp
  preparedState_.clear();
```

The mutators should only update parameters, topology, and obstacle state.

- [ ] **Step 4: Replace `findCollisionPairs()` and `prepareForSurfacePositions()`**

Delete `findCollisionPairs()`, `prepareForSurfacePositions()`, and `invalidatePreparedState()` implementations. Add a single public `buildActiveSet()` (no private helper — the previous draft's `buildActiveSetUnchecked` had only one caller and was dead abstraction):

```cpp
SurfaceIPCActiveSet SurfaceIPCCore::buildActiveSet(EigenSupport::ConstRefVecXd x_surf) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kBuildActiveSet);

  SurfaceIPCActiveSet activeSet;
  activeSet.positions = x_surf;
  buildSelfPairs(topology_, activeSet.positions, dhat, activeSet.selfPairs);

  if (!obstacles_.empty())
    buildExternalPairs(topology_, activeSet.positions, obstacles_, dhat_external, activeSet.externalPairs);

  if (auto logger = Logging::lgr(); logger) {
    const std::size_t selfTotal = activeSet.selfPairs.size();
    const std::size_t externalTotal = activeSet.externalPairs.size();
    SPDLOG_LOGGER_INFO(logger,
      "SurfaceIPCCore active pairs: selfPT={} selfEE={} selfTotal={} externalPT={} externalTP={} externalEE={} externalTotal={}",
      activeSet.selfPairs.ptPairs.size(), activeSet.selfPairs.eePairs.size(), selfTotal,
      activeSet.externalPairs.ptPairs.size(), activeSet.externalPairs.tpPairs.size(),
      activeSet.externalPairs.eePairs.size(), externalTotal);
  }

  return activeSet;
}
```

- [ ] **Step 5: Implement stateless `compute*` entry points**

Replace `computeEnergy(ConstRefVecXd)`, `computeGradient(ConstRefVecXd, RefVecXd)`, `computeHessian(ConstRefVecXd, SpMatD&)`, and `computeAll(ConstRefVecXd, ...)` with:

```cpp
double SurfaceIPCCore::computeEnergy(EigenSupport::ConstRefVecXd pos) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kEnergy);
  return computeEnergy(buildActiveSet(pos));
}

void SurfaceIPCCore::computeGradient(EigenSupport::ConstRefVecXd pos, EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kGradient);
  computeGradient(buildActiveSet(pos), grad);
}

void SurfaceIPCCore::computeHessian(EigenSupport::ConstRefVecXd pos, SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kHessian);
  computeHessian(buildActiveSet(pos), hess);
}

void SurfaceIPCCore::computeAll(EigenSupport::ConstRefVecXd x,
  double &energy, VXd &grad, SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kCombined);
  computeAll(buildActiveSet(x), energy, grad, hess);
}
```

- [ ] **Step 6: Implement active-set compute overloads**

Replace all `compute*WithPreparedPairs()` implementations with:

```cpp
double SurfaceIPCCore::computeEnergy(const SurfaceIPCActiveSet &activeSet) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetEnergy);
  double e = computeSelfEnergy(activeSet.positions, activeSet.selfPairs, topology_.numVerts, dhat, kappa, eps_ee);
  if (!obstacles_.empty()) {
    e += computeExternalEnergy(
      activeSet.positions, obstacles_, activeSet.externalPairs, dhat_external, kappa, eps_ee);
  }
  return e;
}

void SurfaceIPCCore::computeGradient(const SurfaceIPCActiveSet &activeSet, EigenSupport::RefVecXd grad) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetGradient);
  computeSelfGradient(activeSet.positions, activeSet.selfPairs, topology_.numVerts, dhat, kappa, eps_ee, grad);
  if (!obstacles_.empty()) {
    computeExternalGradient(
      activeSet.positions, obstacles_, activeSet.externalPairs, topology_.numVerts, dhat_external, kappa, eps_ee, grad);
  }
}

void SurfaceIPCCore::computeHessian(const SurfaceIPCActiveSet &activeSet, SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetHessian);
  computeSelfHessian(activeSet.positions, activeSet.selfPairs, topology_.numVerts, dhat, kappa, eps_ee, hess);
  if (!obstacles_.empty()) {
    computeExternalHessian(
      activeSet.positions, obstacles_, activeSet.externalPairs, topology_.numVerts, dhat_external, kappa, eps_ee, hess);
  }
  if (auto logger = Logging::lgr(); logger)
    SPDLOG_LOGGER_INFO(logger, "# nonzeros in Hessian: {}", hess.nonZeros());
}

void SurfaceIPCCore::computeAll(const SurfaceIPCActiveSet &activeSet,
  double &energy, VXd &grad, SpMatD &hess) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kActiveSetCombined);
  computeSelfAll(activeSet.positions, activeSet.selfPairs, topology_.numVerts, dhat, kappa, eps_ee, energy, grad, hess);
  if (!obstacles_.empty()) {
    double extEnergy = 0.0;
    computeExternalAll(
      activeSet.positions, obstacles_, activeSet.externalPairs, topology_.numVerts, dhat_external, kappa, eps_ee, extEnergy, grad, hess);
    energy += extEnergy;
  }
  if (auto logger = Logging::lgr(); logger)
    SPDLOG_LOGGER_INFO(logger, "# nonzeros in Hessian: {}", hess.nonZeros());
}
```

- [ ] **Step 7: Remove the prepared-state header**

Delete:

```bash
rm src/core/contact/ipc/core/surfaceIPCPreparedState.h
```

- [ ] **Step 8: Compile core to find downstream call sites**

Run:

```bash
cmake --build build/base_no_mkl --target surfaceIPCCore_gtest -j2
```

Expected: compile failures only in tests and wrappers that still reference prepared-state APIs.

- [ ] **Step 9: Local diff checkpoint**

```bash
git diff -- src/core/contact/ipc/core/surfaceIPCCore.cpp src/core/contact/ipc/profiling/surfaceIPCProfiling.h src/core/contact/ipc/core/surfaceIPCPreparedState.h
```

Do not stage or commit. This checkpoint exists only to inspect the local diff.

## Task 3: Update Wrappers and Adapter Call Sites

**Files:**
- Modify: `src/core/contact/CIPC.h`
- Modify: `src/core/contact/CIPC.cpp`
- Modify: `src/core/contact/mappedSurfacePotentialEnergy.h`
- Modify: `src/core/contact/mappedSurfacePotentialEnergy.cpp`
- Modify: `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h`
- Modify: `src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp`
- Modify: `src/tools/runSim/runIPCSim.cpp`

The wrapper layer follows two patterns:

1. The inherited virtual `(x)` overloads (`func`, `gradient`, `hessianDirect`, `computeSurface{Energy,Gradient,Hessian}`) become **stateless** — each builds its own active set inside the core. Solvers that call these separately pay one broad-phase per call.
2. `MappedSurfacePotentialEnergy` exposes **fused entry points** for solvers that want broad-phase amortization: `func_grad(x, g)` (overrides the existing `PotentialEnergy` virtual — intended as the safest first solver migration step), `func_grad_hessian(x, g, H)` (overrides the existing `PotentialEnergy` virtual), and `gradient_hessian(x, g, H)` (non-virtual extension). All three map simulation displacements to surface positions once, call one virtual hook (`computeSurfaceFuncGrad` or `computeSurfaceAll`), and pull back once. `EmbeddedSurfaceIPCPotentialEnergy` overrides both hooks to do one `core.buildActiveSet` + one or two kernel passes.

`SurfaceIPCActiveSet` is not part of any wrapper signature — it is purely a core-layer implementation detail.

- [ ] **Step 1: Update `CIPCPotentialEnergy` header — drop prepared helper only**

In `src/core/contact/CIPC.h`, delete:

```cpp
  void ensurePreparedForSurfacePositions(const VXd &x_surf) const;
```

No new methods are added to `CIPCPotentialEnergy` — it stays stateless but unfused.

- [ ] **Step 2: Remove the prepared helper implementation**

In `src/core/contact/CIPC.cpp`, delete:

```cpp
void CIPCPotentialEnergy::ensurePreparedForSurfacePositions(const VXd &x_surf) const
{
  if (!core.preparedState().isPreparedFor(x_surf))
    core.prepareForSurfacePositions(x_surf);
}
```

- [ ] **Step 3: Make the virtual `(x)` overloads stateless**

Replace the existing `func()`:

```cpp
double CIPCPotentialEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  syncCoreParametersFromWrapper();
  const VXd x_surf = toSurfacePositions(x);
  ensurePreparedForSurfacePositions(x_surf);
  return core.computeEnergyWithPreparedPairs() + computeFloorEnergy(x_surf);
}
```

with:

```cpp
double CIPCPotentialEnergy::func(EigenSupport::ConstRefVecXd x) const
{
  syncCoreParametersFromWrapper();
  const VXd x_surf = toSurfacePositions(x);
  return core.computeEnergy(x_surf) + computeFloorEnergy(x_surf);
}
```

Replace the existing `gradient()`:

```cpp
void CIPCPotentialEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  syncCoreParametersFromWrapper();
  const VXd x_surf = toSurfacePositions(x);
  ensurePreparedForSurfacePositions(x_surf);
  core.computeGradientWithPreparedPairs(grad);
  addFloorGradient(x_surf, grad);
}
```

with:

```cpp
void CIPCPotentialEnergy::gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const
{
  syncCoreParametersFromWrapper();
  const VXd x_surf = toSurfacePositions(x);
  core.computeGradient(x_surf, grad);
  addFloorGradient(x_surf, grad);
}
```

Replace the existing `hessianDirect()`:

```cpp
void CIPCPotentialEnergy::hessianDirect(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  syncCoreParametersFromWrapper();
  const VXd x_surf = toSurfacePositions(x);
  ensurePreparedForSurfacePositions(x_surf);
  core.computeHessianWithPreparedPairs(hess);
  if (auto logger = Logging::lgr(); logger)
    SPDLOG_LOGGER_INFO(logger, "Computing Hessian with {} PT pairs and {} EE pairs", core.preparedState().selfPairs.ptPairs.size(), core.preparedState().selfPairs.eePairs.size());
  addFloorHessian(x_surf, hess);
}
```

with:

```cpp
void CIPCPotentialEnergy::hessianDirect(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const
{
  syncCoreParametersFromWrapper();
  const VXd x_surf = toSurfacePositions(x);
  core.computeHessian(x_surf, hess);
  addFloorHessian(x_surf, hess);
}
```

The previous wrapper-level pair-count log is dropped — `SurfaceIPCCore::buildActiveSet()` already logs the same counts at the source.

- [ ] **Step 4: Add fused virtuals + entry points to `MappedSurfacePotentialEnergy` header**

In `src/core/contact/mappedSurfacePotentialEnergy.h`, add to the `public:` section after `computeMaxStepLimit`:

```cpp
  virtual double func_grad(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient) const override;

  virtual double func_grad_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const override;

  void gradient_hessian(
    EigenSupport::ConstRefVecXd simulationDisplacements,
    EigenSupport::RefVecXd simulationGradient,
    EigenSupport::SpMatD &simulationHessian) const;
```

Add to the `protected:` section (next to the other `computeSurface*` virtuals):

```cpp
  // Default impl calls computeSurfaceEnergy + computeSurfaceGradient.
  // Subclasses that can share broad-phase between energy and gradient
  // should override this hook (e.g. EmbeddedSurfaceIPCPotentialEnergy).
  virtual void computeSurfaceFuncGrad(
    EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy,
    EigenSupport::RefVecXd surfaceGradient) const;

  // Default impl calls computeSurfaceEnergy/Gradient/Hessian separately.
  // Subclasses that can compute all three in one broad-phase pass should
  // override this hook (e.g. EmbeddedSurfaceIPCPotentialEnergy).
  virtual void computeSurfaceAll(
    EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const;
```

- [ ] **Step 5: Implement fused hooks and entry points on `MappedSurfacePotentialEnergy`**

Append to `src/core/contact/mappedSurfacePotentialEnergy.cpp`:

```cpp
void MappedSurfacePotentialEnergy::computeSurfaceFuncGrad(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient) const
{
  surfaceEnergy = computeSurfaceEnergy(surfacePositions);
  computeSurfaceGradient(surfacePositions, surfaceGradient);
}

void MappedSurfacePotentialEnergy::computeSurfaceAll(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  surfaceEnergy = computeSurfaceEnergy(surfacePositions);
  computeSurfaceGradient(surfacePositions, surfaceGradient);
  computeSurfaceHessian(surfacePositions, surfaceHessian);
}

double MappedSurfacePotentialEnergy::func_grad(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterGradient);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);

  double surfaceEnergy = 0.0;
  VXd surfaceGradient = VXd::Zero(surfaceRestPositions_.size());

  computeSurfaceFuncGrad(surfacePositions, surfaceEnergy, surfaceGradient);

  {
    Profiling::ScopedProfileSection pullbackG(SurfaceIPCProfileSections::kAdapterPullbackGradient);
    simulationGradient = surfaceFromSimulationDispMap_.transpose() * surfaceGradient;
  }
  return surfaceEnergy;
}

double MappedSurfacePotentialEnergy::func_grad_hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  Profiling::ScopedProfileSection scopedProfile(SurfaceIPCProfileSections::kAdapterHessianDirect);
  const VXd surfacePositions = computeSurfacePositionsFromSimulationDisplacements(simulationDisplacements);

  double surfaceEnergy = 0.0;
  VXd surfaceGradient = VXd::Zero(surfaceRestPositions_.size());
  SpMatD surfaceHessian(surfaceRestPositions_.size(), surfaceRestPositions_.size());

  computeSurfaceAll(surfacePositions, surfaceEnergy, surfaceGradient, surfaceHessian);

  {
    Profiling::ScopedProfileSection pullbackG(SurfaceIPCProfileSections::kAdapterPullbackGradient);
    simulationGradient = surfaceFromSimulationDispMap_.transpose() * surfaceGradient;
  }
  {
    Profiling::ScopedProfileSection pullbackH(SurfaceIPCProfileSections::kAdapterPullbackHessian);
    simulationHessian = surfaceFromSimulationDispMap_.transpose() * surfaceHessian * surfaceFromSimulationDispMap_;
  }
  return surfaceEnergy;
}

void MappedSurfacePotentialEnergy::gradient_hessian(
  EigenSupport::ConstRefVecXd simulationDisplacements,
  EigenSupport::RefVecXd simulationGradient,
  EigenSupport::SpMatD &simulationHessian) const
{
  (void)func_grad_hessian(simulationDisplacements, simulationGradient, simulationHessian);
}
```

Profiling sections: `func_grad` reuses `kAdapterGradient` (gradient dominates this path's cost); `func_grad_hessian` reuses `kAdapterHessianDirect` (Hessian dominates). If a future refactor wants finer breakdown, add dedicated `kAdapterFunc{Grad,GradHessian}` constants.

- [ ] **Step 6: Update `EmbeddedSurfaceIPCPotentialEnergy` header — drop invalidate API, add fused overrides**

In `src/core/contact/embeddedSurfaceIPCPotentialEnergy.h`, delete:

```cpp
  void invalidatePreparedState();
```

and delete:

```cpp
  void ensurePreparedForSurfacePositions(EigenSupport::ConstRefVecXd surfacePositions) const;
```

Then add to the `private:` section, next to the other `computeSurface*` virtuals:

```cpp
  virtual void computeSurfaceFuncGrad(
    EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy,
    EigenSupport::RefVecXd surfaceGradient) const override;

  virtual void computeSurfaceAll(
    EigenSupport::ConstRefVecXd surfacePositions,
    double &surfaceEnergy,
    EigenSupport::RefVecXd surfaceGradient,
    EigenSupport::SpMatD &surfaceHessian) const override;
```

- [ ] **Step 7: Make embedded adapter surface methods stateless and add fused overrides**

In `src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp`, delete `ensurePreparedForSurfacePositions()` and `invalidatePreparedState()`. Replace the three surface compute methods with:

```cpp
double EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceEnergy(EigenSupport::ConstRefVecXd surfacePositions) const
{
  return surfaceIPCCore_.computeEnergy(surfacePositions);
}

void EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceGradient(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::RefVecXd surfaceGradient) const
{
  surfaceIPCCore_.computeGradient(surfacePositions, surfaceGradient);
}

void EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceHessian(
  EigenSupport::ConstRefVecXd surfacePositions,
  EigenSupport::SpMatD &surfaceHessian) const
{
  surfaceIPCCore_.computeHessian(surfacePositions, surfaceHessian);
}
```

Then add the two fused overrides:

```cpp
void EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceFuncGrad(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient) const
{
  const SurfaceIPCActiveSet activeSet = surfaceIPCCore_.buildActiveSet(surfacePositions);
  surfaceEnergy = surfaceIPCCore_.computeEnergy(activeSet);
  surfaceIPCCore_.computeGradient(activeSet, surfaceGradient);
}

void EmbeddedSurfaceIPCPotentialEnergy::computeSurfaceAll(
  EigenSupport::ConstRefVecXd surfacePositions,
  double &surfaceEnergy,
  EigenSupport::RefVecXd surfaceGradient,
  EigenSupport::SpMatD &surfaceHessian) const
{
  const SurfaceIPCActiveSet activeSet = surfaceIPCCore_.buildActiveSet(surfacePositions);
  EigenSupport::VXd localGradient = EigenSupport::VXd::Zero(surfaceGradient.size());
  surfaceIPCCore_.computeAll(activeSet, surfaceEnergy, localGradient, surfaceHessian);
  surfaceGradient = localGradient;
}
```


- [ ] **Step 8: Remove the obsolete runSim invalidation call**

In `src/tools/runSim/runIPCSim.cpp`, delete:

```cpp
      context.collisionHandler->invalidatePreparedState();
```

- [ ] **Step 9: Compile wrappers to expose test-only references**

Run:

```bash
cmake --build build/base_no_mkl --target cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest embeddedSurfaceFloorPotentialEnergy_gtest runIPCSim_gtest -j2
```

Expected: compile failures only in tests that still reference old prepared-state behavior, profiling constants, or the removed `invalidatePreparedState()` API. `EmbeddedSurfaceFloorPotentialEnergy` should still compile cleanly — it inherits the default `computeSurfaceFuncGrad` and `computeSurfaceAll` from `MappedSurfacePotentialEnergy`, which fan out to its existing virtuals.

- [ ] **Step 10: Local diff checkpoint**

```bash
git diff -- src/core/contact/CIPC.h src/core/contact/CIPC.cpp src/core/contact/mappedSurfacePotentialEnergy.h src/core/contact/mappedSurfacePotentialEnergy.cpp src/core/contact/embeddedSurfaceIPCPotentialEnergy.h src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp src/tools/runSim/runIPCSim.cpp
```

Do not stage or commit. This checkpoint exists only to inspect the local diff.

## Task 4: Update Core and Broad-Phase Tests

**Files:**
- Modify: `tests/src/core/contact/surfaceIPCCore_gtest.cpp`
- Modify: `tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp`
- Modify: `tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest.cpp`

- [ ] **Step 1: Replace pair-accessor test with explicit active-set test**

In `tests/src/core/contact/surfaceIPCCore_gtest.cpp`, replace `PairAccessorsRemainReadableAcrossComputes` with:

```cpp
TEST(SurfaceIPCCoreGTest, BuildActiveSetCapturesPositionsAndPairs)
{
  SurfaceIPCCore core = makeConfiguredCore();
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd x = flattenPositions(V);

  const auto activeSet = core.buildActiveSet(x);

  EXPECT_TRUE(activeSet.positions.isApprox(x));
  ASSERT_FALSE(activeSet.selfPairs.ptPairs.empty());
  EXPECT_GT(activeSet.selfPairs.size(), 0u);
  EXPECT_EQ(activeSet.externalPairs.size(), 0u);
}
```

- [ ] **Step 2: Replace prepared-pair equivalence test with active-set equivalence test**

In the same file, replace `PreparedPairsMatchDirectEnergyGradientHessian` with:

```cpp
TEST(SurfaceIPCCoreGTest, ActiveSetConsumersMatchStatelessEnergyGradientHessian)
{
  SurfaceIPCCore core = makeConfiguredCore();
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd x = flattenPositions(V);

  const double directEnergy = core.computeEnergy(x);
  ES::VXd directGradient = ES::VXd::Zero(x.size());
  core.computeGradient(x, directGradient);
  ES::SpMatD directHessian;
  core.computeHessian(x, directHessian);

  const auto activeSet = core.buildActiveSet(x);

  const double activeSetEnergy = core.computeEnergy(activeSet);
  ES::VXd activeSetGradient = ES::VXd::Zero(x.size());
  core.computeGradient(activeSet, activeSetGradient);
  ES::SpMatD activeSetHessian;
  core.computeHessian(activeSet, activeSetHessian);

  EXPECT_NEAR(activeSetEnergy, directEnergy, 1e-12);
  EXPECT_LT(relativeError(activeSetGradient, directGradient), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(activeSetHessian), sparseToDense(directHessian)), 1e-12);
}
```

- [ ] **Step 3: Replace prepared invalidation test with computeAll broad-phase reuse test**

In the same file, replace `PreparedStateAccessorIsReadOnlyAndExplicitlyInvalidated` with:

```cpp
TEST(SurfaceIPCCoreGTest, ComputeAllBuildsActiveSetOnce)
{
  SurfaceIPCCore core = makeConfiguredCore();
  const auto [V, F] = makeTwoTriangleMesh();
  (void)F;
  const ES::VXd x = flattenPositions(V);

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  double energy = 0.0;
  ES::VXd gradient = ES::VXd::Zero(x.size());
  ES::SpMatD hessian;
  core.computeAll(x, energy, gradient, hessian);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);
  EXPECT_GT(energy, 0.0);
  EXPECT_EQ(gradient.size(), x.size());
  EXPECT_EQ(hessian.rows(), x.size());
}
```

If `ProfileStat` and `findStat()` are not already available in this file, add near the top:

```cpp
#include "scopedProfileSection.h"
#include "ipc/profiling/surfaceIPCProfiling.h"

#include <algorithm>
#include <string_view>
#include <vector>
```

and add inside the anonymous namespace:

```cpp
using pgo::Profiling::ProfileStat;

const ProfileStat *findStat(const std::vector<ProfileStat> &stats, std::string_view name)
{
  const auto it = std::find_if(stats.begin(), stats.end(),
    [name](const ProfileStat &stat) { return stat.name == name; });
  return it == stats.end() ? nullptr : &(*it);
}
```

- [ ] **Step 4: Update obstacle slot test to use explicit active set**

In `ConstructorInjectedObstaclesAssignSequentialSlots`, replace:

```cpp
  core.prepareForSurfacePositions(x);
```

with:

```cpp
  const auto activeSet = core.buildActiveSet(x);
```

Then replace:

```cpp
  scanSlots(core.preparedState().externalPairs.ptPairs);
  scanSlots(core.preparedState().externalPairs.tpPairs);
  scanSlots(core.preparedState().externalPairs.eePairs);
```

with:

```cpp
  scanSlots(activeSet.externalPairs.ptPairs);
  scanSlots(activeSet.externalPairs.tpPairs);
  scanSlots(activeSet.externalPairs.eePairs);
```

- [ ] **Step 5: Delete prepared-state missing-state test**

Delete the whole test:

```cpp
TEST(SurfaceIPCCoreGTest, PreparedPairConsumersRequirePreparedState)
```

There is no invalid missing-state runtime path after active-set ownership becomes explicit.

- [ ] **Step 6: Update self broad-phase test**

In `tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp`, replace:

```cpp
  core.computeEnergy(x);

  EXPECT_EQ(canonicalPT(broadPhasePairs.ptPairs), canonicalPT(core.preparedState().selfPairs.ptPairs));
  EXPECT_EQ(canonicalEE(broadPhasePairs.eePairs), canonicalEE(core.preparedState().selfPairs.eePairs));
```

with:

```cpp
  const auto activeSet = core.buildActiveSet(x);

  EXPECT_EQ(canonicalPT(broadPhasePairs.ptPairs), canonicalPT(activeSet.selfPairs.ptPairs));
  EXPECT_EQ(canonicalEE(broadPhasePairs.eePairs), canonicalEE(activeSet.selfPairs.eePairs));
```

- [ ] **Step 7: Update external broad-phase test**

In `tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest.cpp`, replace:

```cpp
  core.prepareForSurfacePositions(x);
```

with:

```cpp
  const auto activeSet = core.buildActiveSet(x);
```

Then replace:

```cpp
  EXPECT_EQ(canonicalPT(pairs.ptPairs), canonicalPT(core.preparedState().externalPairs.ptPairs));
  EXPECT_EQ(canonicalTP(pairs.tpPairs), canonicalTP(core.preparedState().externalPairs.tpPairs));
  EXPECT_EQ(canonicalEE(pairs.eePairs), canonicalEE(core.preparedState().externalPairs.eePairs));
```

with:

```cpp
  EXPECT_EQ(canonicalPT(pairs.ptPairs), canonicalPT(activeSet.externalPairs.ptPairs));
  EXPECT_EQ(canonicalTP(pairs.tpPairs), canonicalTP(activeSet.externalPairs.tpPairs));
  EXPECT_EQ(canonicalEE(pairs.eePairs), canonicalEE(activeSet.externalPairs.eePairs));
```

- [ ] **Step 8: Run focused core tests**

Run:

```bash
cmake --build build/base_no_mkl --target surfaceIPCCore_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalBroadPhase_gtest -j2
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest
```

Expected: all three test binaries pass.

- [ ] **Step 9: Local diff checkpoint**

```bash
git diff -- tests/src/core/contact/surfaceIPCCore_gtest.cpp tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest.cpp
```

Do not stage or commit. This checkpoint exists only to inspect the local diff.

## Task 5: Update Wrapper, Adapter, Profiling, and Legacy Tests

**Files:**
- Modify: `tests/src/core/contact/cipcPotentialEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`
- Modify: `tests/src/core/contact/cipcProfiling_gtest.cpp`
- Modify: `src/tests/testIPCExternal/testIPCExternal.cpp`

- [ ] **Step 1: Rename wrapper cache test to stateless rebuild test (assert on `kBuildActiveSet`)**

In `tests/src/core/contact/cipcPotentialEnergy_gtest.cpp`, rename:

```cpp
TEST(CIPCPotentialEnergyGTest, ReusesPreparedPairsAcrossEnergyGradientHessianForSameState)
```

to:

```cpp
TEST(CIPCPotentialEnergyGTest, SeparateEvaluationsBuildIndependentActiveSetsForSameState)
```

If the existing test queries `pgo::Contact::SurfaceIPCProfileSections::kPairBuildStatic`, switch the lookup to `kBuildActiveSet` (the semantic counter for "one active set was built") and rename the local variable accordingly:

```cpp
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);
  ASSERT_NE(activeSetBuild, nullptr);
```

Then replace:

```cpp
  EXPECT_EQ(pairBuild->callCount, 1u);
```

with:

```cpp
  EXPECT_EQ(activeSetBuild->callCount, 6u);
```

This test calls `func`, `gradient`, and `hessianDirect` twice through the inherited `(x)` overloads — each delegates to a fresh `buildActiveSet(x)`, so 3 × 2 = 6 active-set builds.

- [ ] **Step 2: Rename embedded adapter cache test to stateless rebuild test (assert on `kBuildActiveSet`)**

In `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`, rename:

```cpp
TEST(EmbeddedSurfaceIPCPotentialEnergyGTest, ReusesPreparedPairsAcrossEnergyGradientHessianForSameState)
```

to:

```cpp
TEST(EmbeddedSurfaceIPCPotentialEnergyGTest, SeparateEvaluationsBuildIndependentActiveSetsForSameState)
```

Switch the stat lookup to `kBuildActiveSet` (same change as Step 1) and replace:

```cpp
  EXPECT_EQ(pairBuild->callCount, 1u);
```

with:

```cpp
  EXPECT_EQ(activeSetBuild->callCount, 6u);
```

- [ ] **Step 2b: Add Embedded adapter fused-API reuse tests**

The renamed tests above pin "via separate `(x)` overload calls → rebuild every time". The new fused entry points on `MappedSurfacePotentialEnergy` / `EmbeddedSurfaceIPCPotentialEnergy` need their own coverage proving that a single fused call serves the requested quantities from one broad-phase pass and that the values match the per-method path.

Add to `tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp`:

```cpp
TEST(EmbeddedSurfaceIPCPotentialEnergyGTest, FuncGradFusesOneBroadPhaseForEnergyAndGradient)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd simDispl = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    simDispl[3 * vi + 2] = 0.01;

  EmbeddedSurfaceIPCPotentialEnergy energy(V, F, makeIdentityEmbedding(rest.size()), makeParams());

  ES::VXd g = ES::VXd::Zero(simDispl.size());

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  const double e = energy.func_grad(simDispl, g);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);  // one build for E + g.

  // Values must match the per-method path.
  const double eRef = energy.func(simDispl);
  ES::VXd gRef = ES::VXd::Zero(simDispl.size());
  energy.gradient(simDispl, gRef);

  EXPECT_NEAR(e, eRef, 1e-12);
  EXPECT_LT(relativeError(g, gRef), 1e-12);
}

TEST(EmbeddedSurfaceIPCPotentialEnergyGTest, FuncGradHessianFusesOneBroadPhaseForAllThree)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd simDispl = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    simDispl[3 * vi + 2] = 0.01;

  EmbeddedSurfaceIPCPotentialEnergy energy(V, F, makeIdentityEmbedding(rest.size()), makeParams());

  ES::VXd g = ES::VXd::Zero(simDispl.size());
  ES::SpMatD H;
  energy.hessianDirect(simDispl, H);  // size H once. Done BEFORE enabling profiling.
  H.setZero();
  g.setZero();

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  const double e = energy.func_grad_hessian(simDispl, g, H);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);
  const ProfileStat *combinedStat = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetCombined);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  ASSERT_NE(combinedStat, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);  // one build for E + g + H.
  EXPECT_EQ(combinedStat->callCount, 1u);

  // Values must match the per-method path.
  const double eRef = energy.func(simDispl);
  ES::VXd gRef = ES::VXd::Zero(simDispl.size());
  energy.gradient(simDispl, gRef);
  ES::SpMatD HRef;
  energy.hessianDirect(simDispl, HRef);

  EXPECT_NEAR(e, eRef, 1e-12);
  EXPECT_LT(relativeError(g, gRef), 1e-12);
  EXPECT_LT(relativeError(sparseToDense(H), sparseToDense(HRef)), 1e-12);
}

TEST(EmbeddedSurfaceIPCPotentialEnergyGTest, GradientHessianFusesOneBroadPhaseForGradAndHess)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd rest = flattenPositions(V);
  ES::VXd simDispl = ES::VXd::Zero(rest.size());
  for (int vi = 3; vi < 6; ++vi)
    simDispl[3 * vi + 2] = 0.01;

  EmbeddedSurfaceIPCPotentialEnergy energy(V, F, makeIdentityEmbedding(rest.size()), makeParams());

  ES::VXd g = ES::VXd::Zero(simDispl.size());
  ES::SpMatD H;
  energy.hessianDirect(simDispl, H);  // size once.
  H.setZero();

  pgo::Profiling::setProfilingEnabled(true);
  pgo::Profiling::resetProfileStatistics();

  energy.gradient_hessian(simDispl, g, H);

  const auto stats = pgo::Profiling::snapshotProfileStatistics();
  const ProfileStat *activeSetBuild = findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet);

  pgo::Profiling::setProfilingEnabled(false);
  pgo::Profiling::resetProfileStatistics();

  ASSERT_NE(activeSetBuild, nullptr);
  EXPECT_EQ(activeSetBuild->callCount, 1u);  // one build for g + H.
}
```

`CIPCPotentialEnergy` retains only the renamed `SeparateEvaluationsBuildIndependentActiveSetsForSameState` test — it stays stateless but does not grow fused tests because it is not the active wrapper path.

- [ ] **Step 3: Update profiling test constants**

In `tests/src/core/contact/cipcProfiling_gtest.cpp`, replace both occurrences of:

```cpp
  EXPECT_NE(findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kPrepareActivePairs), nullptr);
```

with:

```cpp
  EXPECT_NE(findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kBuildActiveSet), nullptr);
```

Replace both occurrences of:

```cpp
  EXPECT_NE(findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kPreparedEnergy), nullptr);
```

with:

```cpp
  EXPECT_NE(findStat(stats, pgo::Contact::SurfaceIPCProfileSections::kActiveSetEnergy), nullptr);
```

- [ ] **Step 4: Update legacy external IPC tests to inspect active sets**

In `src/tests/testIPCExternal/testIPCExternal.cpp`, replace each:

```cpp
core.prepareForSurfacePositions(x);
```

or:

```cpp
emptyCore.prepareForSurfacePositions(x);
```

with a local active set variable scoped to the test, for example:

```cpp
const auto activeSet = core.buildActiveSet(x);
```

Use distinct names when there are multiple cores in one test:

```cpp
const auto emptyActiveSet = emptyCore.buildActiveSet(x);
const auto activeSet = core.buildActiveSet(x);
```

Then replace all prepared-state pair reads:

```cpp
core.preparedState().externalPairs
emptyCore.preparedState().externalPairs
```

with:

```cpp
activeSet.externalPairs
emptyActiveSet.externalPairs
```

For the pair coverage test, the final shape should be:

```cpp
const auto activeSet = core.buildActiveSet(x);
const auto &ptPairs = activeSet.externalPairs.ptPairs;
const auto &tpPairs = activeSet.externalPairs.tpPairs;
const auto &eePairs = activeSet.externalPairs.eePairs;
```

- [ ] **Step 5: Run wrapper and profiling tests**

Run:

```bash
cmake --build build/base_no_mkl --target cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest cipcProfiling_gtest -j2
./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/cipcProfiling_gtest
```

Expected: all three test binaries pass, with pair-build counts now reflecting stateless separate evaluations.

- [ ] **Step 6: Compile legacy IPC external test target if available**

Run:

```bash
cmake --build build/base_no_mkl --target testIPCExternal -j2
```

If the target name is not present in the active build preset, verify compilation through the broad contact test target used by this checkout:

```bash
cmake --build build/base_no_mkl --target contact -j2
```

Expected: no references remain to `prepareForSurfacePositions()`, `preparedState()`, or `compute*WithPreparedPairs()`.

- [ ] **Step 7: Local diff checkpoint**

```bash
git diff -- tests/src/core/contact/cipcPotentialEnergy_gtest.cpp tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest.cpp tests/src/core/contact/cipcProfiling_gtest.cpp src/tests/testIPCExternal/testIPCExternal.cpp
```

Do not stage or commit. This checkpoint exists only to inspect the local diff.

## Task 6: Remove All Prepared-State References and Run Full Contact Validation

**Files:**
- Verify: whole repository

- [ ] **Step 1: Search for removed symbols**

Run:

```bash
rg -n 'SurfaceIPCPreparedState|preparedState_|preparedState\(|prepareForSurfacePositions|computeEnergyWithPreparedPairs|computeGradientWithPreparedPairs|computeHessianWithPreparedPairs|computeAllWithPreparedPairs|invalidatePreparedState|kPrepareActivePairs|kPreparedEnergy|kPreparedGradient|kPreparedHessian' src tests
```

(Use single quotes so the shell passes `preparedState\(` to ripgrep as a single backslash + literal parenthesis. Double backslash inside a Markdown code block would arrive as `\\(` and silently miss matches.)

Expected: no matches in `src` or `tests`. Historical plan documents may still mention removed symbols, but implementation and test code must not.

- [ ] **Step 2: Run focused IPC/contact build**

Run:

```bash
cmake --preset base_no_mkl
cmake --build build/base_no_mkl --target ipcGeometry_gtest surfaceIPCTopology_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalBroadPhase_gtest surfaceIPCMaxStep_gtest surfaceIPCExternalMaxStep_gtest surfaceIPCBarrierAssembler_gtest surfaceIPCCore_gtest cipcProfiling_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest embeddedSurfaceFloorPotentialEnergy_gtest runIPCSim_gtest -j2
```

Expected: build succeeds.

- [ ] **Step 3: Run focused IPC/contact tests**

Run:

```bash
./build/base_no_mkl/tests/src/core/contact/ipcGeometry_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCTopology_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCMaxStep_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCExternalMaxStep_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCBarrierAssembler_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/cipcProfiling_gtest
./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
```

Expected: all listed tests pass.

- [ ] **Step 4: Run formatting and diff hygiene checks**

Run:

```bash
git diff --check
```

Expected: no whitespace errors.

- [ ] **Step 5: Inspect final public API diff**

Run:

```bash
git diff -- src/core/contact/ipc/core/surfaceIPCCore.h src/core/contact/ipc/core/surfaceIPCActiveSet.h src/core/contact/CIPC.h src/core/contact/embeddedSurfaceIPCPotentialEnergy.h
```

Expected:
- `SurfaceIPCCore` exposes `buildActiveSet()` and active-set compute overloads.
- No core API exposes `preparedState()`, `prepareForSurfacePositions()`, or `compute*WithPreparedPairs()`.
- `EmbeddedSurfaceIPCPotentialEnergy` no longer exposes `invalidatePreparedState()`.
- `CIPCPotentialEnergy` has no prepared helper declaration.

- [ ] **Step 6: Final local diff checkpoint**

```bash
git status --short
git diff --stat
```

Do not stage or commit. The user will review the final uncommitted workspace.

## Risk Notes

- **Cache removal is the goal, not a regression.** This refactor intentionally eliminates the cross-call `preparedState_` cache. Callers using the inherited per-quantity `(x)` overloads of both wrappers will rebuild the active set on every call. The renamed `SeparateEvaluationsBuildIndependentActiveSetsForSameState` tests pin this behavior.
- **Wrapper reuse path is the fused API, not active-set objects.** `SurfaceIPCActiveSet` stays a core-layer implementation detail. The primary reuse path is through `MappedSurfacePotentialEnergy` (and its `EmbeddedSurfaceIPCPotentialEnergy` subclass). Three fused entry points are exposed: `func_grad(x, g)` (best as first solver migration step — fuses `func` + `gradient`), `func_grad_hessian(x, g, H)`, and `gradient_hessian(x, g, H)`. Each maps simulation displacements to surface positions once, calls one virtual hook (`computeSurfaceFuncGrad` or `computeSurfaceAll`), and pullbacks once. `EmbeddedSurfaceIPCPotentialEnergy` overrides both hooks to do one `core.buildActiveSet` per fused call. The Embedded tests (`FuncGradFusesOneBroadPhaseForEnergyAndGradient`, `FuncGradHessianFusesOneBroadPhaseForAllThree`, `GradientHessianFusesOneBroadPhaseForGradAndHess`) pin the single-broad-phase guarantee.
- **CIPC wrapper is stateless but unfused.** `CIPCPotentialEnergy` gets the same stateless treatment (`func`/`gradient`/`hessianDirect` call `core.compute*(x_surf)` statelessly), but it does not add fused entry points — it is not the active wrapper path. It retains only the renamed `SeparateEvaluationsBuildIndependentActiveSetsForSameState` test. The latent crash (base `PotentialEnergy::func_grad_hessian` calling the throwing `hessian()`) remains known but is moot since no solver calls through `CIPCPotentialEnergy` via that path.
- **Solver migration is a follow-up PR.** `NewtonSolver` currently calls `func(x)` then `gradient(x, grad)` separately at `NewtonSolver.cpp:173` / `:183`. The first, safest migration step is replacing that pair with `func_grad(x, grad)`, saving one broad-phase build per Newton iteration with no risk of wasted Hessian work. The `func_grad_hessian` and `gradient_hessian` migrations require more care (converged-iteration Hessian waste) and should be profiled first. This PR only exposes the fused API; `NewtonSolver` is unchanged.
- **No API accepts both `x` and `activeSet`.** Active-set consumers read positions from `activeSet.positions`. This prevents mismatching `x1` with pairs from `x0`.
- **Thread-safety improvement (side benefit).** Removing `mutable preparedState_` makes `SurfaceIPCCore::compute*(x) const` and active-set compute overloads safe to call concurrently on a shared `const SurfaceIPCCore &`. Previously the implicit cache made every "const" method a hidden writer.
- **`EmbeddedSurfaceFloorPotentialEnergy` is unaffected.** It inherits the default `MappedSurfacePotentialEnergy` virtual hooks (`computeSurfaceFuncGrad`, `computeSurfaceAll`), which fan out to its existing per-quantity virtuals. Its fused paths become correct but do not gain broad-phase amortization since it has no broad phase to amortize.

## Self-Review

- Spec coverage: the plan removes cross-call `preparedState_`, keeps active pairs as per-evaluation data inside `SurfaceIPCActiveSet`, preserves `computeAll()` as the convenience single-evaluation reuse point at the core, and adds **fused wrapper entry points** (`func_grad` override + `func_grad_hessian` override + `gradient_hessian` extension) on `MappedSurfacePotentialEnergy`, consumed by `EmbeddedSurfaceIPCPotentialEnergy` via two new `protected` virtual hooks (`computeSurfaceFuncGrad`, `computeSurfaceAll`).
- Encapsulation: `SurfaceIPCActiveSet` does **not** appear in any wrapper public signature. It is strictly a .cpp implementation detail of `EmbeddedSurfaceIPCPotentialEnergy`'s virtual hook overrides.
- Focused scope: `CIPCPotentialEnergy` gets stateless treatment only (its `func`/`gradient`/`hessianDirect` call stateless `core.compute*(x_surf)`). No fused API is added there — it is not the active wrapper path. The primary fused-API delivery vehicle is `MappedSurfacePotentialEnergy` → `EmbeddedSurfaceIPCPotentialEnergy`.
- Placeholder scan: no unresolved placeholder steps; every code-changing task lists concrete files, snippets, commands, and expected results.
- Profiling coverage: all active-set consumers have paired profile sections (`kActiveSetEnergy/Gradient/Hessian/Combined`). Fused tests assert on `kBuildActiveSet` (proves one broad-phase build) and `kActiveSetCombined` (proves the fused kernel path was taken).
- Test assertions target semantic counters (`kBuildActiveSet`, `kActiveSetCombined`) rather than implementation-detail counters (`kPairBuildStatic`).
- Out of scope (intentionally): solver call sites are unchanged. The first migration step is replacing `func(x)`+`gradient(x, g)` with `func_grad(x, g)` in `NewtonSolver` — one line, one PR.
