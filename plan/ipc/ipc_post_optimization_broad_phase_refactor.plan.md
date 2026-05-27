# IPC Post-Optimization Broad Phase Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变 `SurfaceIPCCore` 公开 API、IPC 数值语义、pair identity 和 profiling section 名称的前提下，收口性能优化后集中在 `surfaceIPCBroadPhase.cpp` 的复杂度。

**Architecture:** 保留公开入口 `ipc/broadPhase/surfaceIPCBroadPhase.h`。把 broad phase 内部工具提取到 internal header，把 self 与 external broad phase 实现拆到独立 `.cpp`，让 `surfaceIPCBroadPhase.cpp` 不再承载 900+ 行 mixed responsibility。测试先补 direct line-search superset builder 覆盖，再执行源文件拆分。

**Tech Stack:** C++20, Eigen, TBB, CMake, GoogleTest, existing `contact` library.

---

## 范围与非目标

- 本轮只改 `src/core/contact/ipc/broadPhase/`、`src/core/contact/CMakeLists.txt` 和 focused broad phase tests。
- 不修改 `SurfaceIPCCore` public methods，也不改变 `SurfaceIPCActiveSet` / `SelfPairSet` / `ExternalPairSet` 数据结构。
- 不改 `SpatialHashGrid` 算法、不引入 BVH、不做 incremental active-set、不改 barrier / CCD / solver。
- 不改 profiling section/counter 字符串；已有 profile 消费方不需要迁移。
- 所有 pair ordering 允许保持当前 TBB merge 顺序；测试必须比较 canonical pair identity，而不是依赖原始顺序。

## 目标文件结构

```text
src/core/contact/ipc/broadPhase/
  spatialHashGrid.h/.cpp
  surfaceIPCBroadPhase.h                 # public API, unchanged
  surfaceIPCBroadPhase.cpp               # compatibility translation unit only
  surfaceIPCBroadPhaseInternal.h         # internal AABB/query/counter helpers
  surfaceIPCSelfBroadPhase.cpp           # buildSelfPairs* implementation
  surfaceIPCExternalBroadPhase.cpp       # buildExternalPairs* implementation
```

`surfaceIPCBroadPhaseInternal.h` 不加入 public API 文档；它只服务 broad-phase `.cpp` 文件。

## 任务清单

### Task 1: 补 direct line-search superset builder 测试

**Files:**
- Modify: `tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp`
- Modify: `tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest.cpp`

- [x] **Step 1.1: 给 self broad phase 测试加入 superset subset 断言**

在 `surfaceIPCSelfBroadPhase_gtest.cpp` 中新增 helper：

```cpp
bool containsSelfPT(const std::vector<PTPair> &pairs, const PTPair &target)
{
  return std::find_if(pairs.begin(), pairs.end(),
    [&](const PTPair &pair) { return canonicalPT({ pair }).front() == canonicalPT({ target }).front(); }) != pairs.end();
}

bool containsSelfEE(const std::vector<EEPair> &pairs, const EEPair &target)
{
  return std::find_if(pairs.begin(), pairs.end(),
    [&](const EEPair &pair) { return canonicalEE({ pair }).front() == canonicalEE({ target }).front(); }) != pairs.end();
}
```

然后新增测试：

```cpp
TEST(SurfaceIPCSelfBroadPhaseGTest, LineSearchSupersetContainsExactSelfPairsAtTrialStates)
{
  const auto [V, F] = makeTwoTriangleMesh();
  const ES::VXd x = flattenPositions(V);
  ES::VXd dx = ES::VXd::Zero(x.size());
  for (int vi = 3; vi < 6; ++vi)
    dx[3 * vi + 2] = -0.08;

  SurfaceIPCTopology topology;
  topology.setMesh(V, F);

  SelfPairSet superset;
  buildSelfPairsLineSearchSuperset(topology, x, dx, 0.1, superset);

  bool sawExactPairs = false;
  for (double alpha : { 0.0, 0.25, 0.5, 1.0 }) {
    SelfPairSet exact;
    buildSelfPairs(topology, x + alpha * dx, 0.1, exact);
    sawExactPairs = sawExactPairs || exact.size() > 0;

    for (const auto &pair : exact.ptPairs)
      EXPECT_TRUE(containsSelfPT(superset.ptPairs, pair));
    for (const auto &pair : exact.eePairs)
      EXPECT_TRUE(containsSelfEE(superset.eePairs, pair));
  }

  EXPECT_TRUE(sawExactPairs);
}
```

- [x] **Step 1.2: 给 external broad phase 测试加入 direct superset subset 断言**

在 `surfaceIPCExternalBroadPhase_gtest.cpp` 中新增 local `contains*` helper，使用已有 `canonicalPT` / `canonicalTP` / `canonicalEE`。

新增测试：

```cpp
TEST(SurfaceIPCExternalBroadPhaseGTest, LineSearchSupersetContainsExactExternalPairsAtTrialStates)
{
  ES::MXd dynV(4, 3);
  dynV << 0.0, 0.0, 0.32,
    1.0, 0.0, 0.32,
    0.0, 1.0, 0.32,
    1.0, 1.0, 0.32;
  ES::MXi dynF(2, 3);
  dynF << 0, 1, 2,
    1, 3, 2;

  auto [obsV, obsF] = makeUnitSquareMesh();
  const ES::VXd obsRest = flattenRows(obsV);
  ObstacleSurface obs(obsV, obsF,
    pgo::Contact::CIPC::makeLinearTrajectorySampler(obsRest, ES::V3d::Zero()));
  obs.setObjectId(3);
  obs.update(0.0);

  SurfaceIPCTopology topology;
  topology.setMesh(dynV, dynF);

  const ES::VXd x = flattenRows(dynV);
  ES::VXd dx = ES::VXd::Zero(x.size());
  for (int vi = 0; vi < dynV.rows(); ++vi)
    dx[3 * vi + 2] = -0.28;

  std::vector<ObstacleSurface> obstacles;
  obstacles.emplace_back(std::move(obs));

  ExternalPairSet superset;
  buildExternalPairsLineSearchSuperset(topology, x, dx, obstacles, 0.35, superset);

  bool sawExactPairs = false;
  for (double alpha : { 0.0, 0.25, 0.5, 1.0 }) {
    ExternalPairSet exact;
    buildExternalPairs(topology, x + alpha * dx, obstacles, 0.35, exact);
    sawExactPairs = sawExactPairs || exact.size() > 0;

    for (const auto &pair : exact.ptPairs)
      EXPECT_TRUE(containsExternalPT(superset.ptPairs, pair));
    for (const auto &pair : exact.tpPairs)
      EXPECT_TRUE(containsExternalTP(superset.tpPairs, pair));
    for (const auto &pair : exact.eePairs)
      EXPECT_TRUE(containsExternalEE(superset.eePairs, pair));
  }

  EXPECT_TRUE(sawExactPairs);
}
```

- [x] **Step 1.3: 运行 focused broad phase 测试**

Run:

```bash
cmake --build --preset base_no_mkl_release --target surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest
```

Expected: 两个测试 target 编译并通过。若新增测试失败，先确认是测试 fixture 问题还是当前 superset 行为确实缺 pair；不进入 Task 2。

### Task 2: 提取 broad-phase internal helper

**Files:**
- Create: `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhaseInternal.h`
- Modify: `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhase.cpp`

- [x] **Step 2.1: 创建 internal helper header**

把当前 `surfaceIPCBroadPhase.cpp` 中这些 anonymous-namespace helper 搬入 `pgo::Contact::CIPC::broad_phase_detail`：

- `buildVertexAABBs`
- `buildTriangleAABBs`
- `buildEdgeAABBs`
- `buildSweptVertexAABBs`
- `buildSweptTriangleAABBs`
- `buildSweptEdgeAABBs`
- `computeUnionAABB`
- `PairQueryCounts`
- `addCounts`
- `recordPairQueryCounters`
- `collectPairsParallel`
- `obsVtx`

`collectPairsParallel` 和 AABB builders 是 templates，必须留在 header 中。`recordPairQueryCounters` 可以是 `inline` function，避免新 `.cpp` 链接点。

- [x] **Step 2.2: 让现有 broad phase 实现使用 internal helper**

在 `surfaceIPCBroadPhase.cpp` 中 include:

```cpp
#include "ipc/broadPhase/surfaceIPCBroadPhaseInternal.h"
```

删除本文件内重复 helper 定义，并在文件顶部加入：

```cpp
using namespace broad_phase_detail;
```

- [x] **Step 2.3: 运行 broad phase focused tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCExternalBroadPhase_gtest
```

Expected: behavior unchanged.

### Task 3: 拆 self/external broad phase translation units

**Files:**
- Create: `src/core/contact/ipc/broadPhase/surfaceIPCSelfBroadPhase.cpp`
- Create: `src/core/contact/ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp`
- Modify: `src/core/contact/ipc/broadPhase/surfaceIPCBroadPhase.cpp`
- Modify: `src/core/contact/CMakeLists.txt`

- [x] **Step 3.1: 移动 self broad phase 实现**

把以下 public functions 从 `surfaceIPCBroadPhase.cpp` 移到 `surfaceIPCSelfBroadPhase.cpp`：

- `buildSelfPairs`
- `buildSelfPairsLineSearchSuperset`

新文件 include:

```cpp
#include "ipc/broadPhase/surfaceIPCBroadPhase.h"
#include "ipc/broadPhase/surfaceIPCBroadPhaseInternal.h"
#include "ipc/geometry/ipcDistancePrimitives.h"
#include "ipc/profiling/surfaceIPCProfiling.h"
#include "scopedProfileSection.h"
```

保留 namespace `pgo::Contact::CIPC`，并使用 `using namespace broad_phase_detail;`。

- [x] **Step 3.2: 移动 external broad phase 实现**

把以下 public functions 从 `surfaceIPCBroadPhase.cpp` 移到 `surfaceIPCExternalBroadPhase.cpp`：

- `buildExternalPairs`
- `buildExternalPairsLineSearchSuperset`

新文件 include 与 self 文件一致，并额外依赖 obstacle cache 已经通过 public header 间接可见。

- [x] **Step 3.3: 保留 compatibility translation unit**

`surfaceIPCBroadPhase.cpp` 保留为仅 include public header 的空 translation unit：

```cpp
#include "ipc/broadPhase/surfaceIPCBroadPhase.h"
```

这样保留源文件路径，减少外部脚本或 IDE 对旧文件存在性的假设破坏。

- [x] **Step 3.4: 更新 CMake sources**

在 `src/core/contact/CMakeLists.txt` 的 `CONTACT_SOURCES` 中加入：

```cmake
ipc/broadPhase/surfaceIPCSelfBroadPhase.cpp
ipc/broadPhase/surfaceIPCExternalBroadPhase.cpp
```

保留 `ipc/broadPhase/surfaceIPCBroadPhase.cpp`。

- [x] **Step 3.5: 运行 build 验证**

Run:

```bash
cmake --build --preset base_no_mkl_release --target contact surfaceIPCSelfBroadPhase_gtest surfaceIPCExternalBroadPhase_gtest
```

Expected: no duplicate symbol, no missing symbol, no include error。

### Task 4: Focused correctness verification

**Files:** No source edits unless verification finds a bug.

- [x] **Step 4.1: 运行 IPC broad/core focused tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target \
  ipcGeometry_gtest \
  spatialHashGrid_gtest \
  surfaceIPCTopology_gtest \
  surfaceIPCSelfBroadPhase_gtest \
  surfaceIPCExternalBroadPhase_gtest \
  surfaceIPCMaxStep_gtest \
  surfaceIPCExternalMaxStep_gtest \
  surfaceIPCBarrierAssembler_gtest \
  surfaceIPCCore_gtest \
  cipcPotentialEnergy_gtest \
  embeddedSurfaceIPCPotentialEnergy_gtest \
  embeddedSurfaceFloorPotentialEnergy_gtest \
  runIPCSim_gtest

ctest --test-dir build/base_no_mkl \
  -R "ipcGeometry|spatialHashGrid|surfaceIPCTopology|surfaceIPCSelfBroadPhase|surfaceIPCExternalBroadPhase|surfaceIPCMaxStep|surfaceIPCExternalMaxStep|surfaceIPCBarrierAssembler|surfaceIPCCore|cipcPotentialEnergy|embeddedSurfaceIPCPotentialEnergy|embeddedSurfaceFloorPotentialEnergy|runIPCSim" \
  --output-on-failure
```

Expected: all matched tests pass.

- [x] **Step 4.2: 运行 diff hygiene**

Run:

```bash
git diff --check
git status --short
```

Expected: no whitespace errors。`git status` 只显示本计划、测试补强、broad-phase 拆分和 CMake 更新。

## 回滚策略

- 如果 Task 1 新测试揭示当前 superset 行为缺 pair，停止并转为 bugfix，不执行源文件拆分。
- 如果 Task 2 后 broad phase tests 失败，回滚 internal helper extraction，比较 helper 移动中是否改变了 `stamp`、`visited` 或 profile counter 累计。
- 如果 Task 3 后链接失败，优先检查 CMake source list 和是否旧 `.cpp` 仍保留函数定义导致 duplicate symbol。
- 如果 Task 4 中 core/barrier/adapter tests 失败，说明 source split 引入行为变化；回滚到 Task 2 通过点，再按 self/external 分别重做。

## 完成标准

- `surfaceIPCBroadPhase.cpp` 不再包含 self/external broad phase 主实现。
- self 和 external broad phase implementation 分别位于 `surfaceIPCSelfBroadPhase.cpp` 与 `surfaceIPCExternalBroadPhase.cpp`。
- AABB/TLS/counter helper 只定义在 `surfaceIPCBroadPhaseInternal.h` 一处。
- Direct builder superset tests 覆盖 self 和 external line-search paths。
- Focused IPC/contact tests 与 `git diff --check` 通过。

## 自审记录

- **Spec coverage:** 覆盖用户要求的详细计划、执行前自审、执行后正确性验证。计划明确不做大架构重写，只做性能优化后的收口重构。
- **Placeholder scan:** 未发现占位式任务描述；每个任务有具体文件、函数和验证命令。
- **Repo truth:** 当前 repo 已存在 `surfaceIPCBroadPhase.h/.cpp`、self/external broad phase gtests、`base_no_mkl_release` preset 和对应 CMake targets。计划未引用不存在的测试 target。
- **Risk review:** 最大风险是 helper header template 移动造成 namespace/include 漏项，以及 source split 造成 duplicate/missing symbol；Task 2/3 分开验证可以定位。
- **Residual risk:** 这是结构重构，不保证性能变化；本轮验证目标是行为等价和构建正确性，不把 profile 改善作为完成标准。

## 执行记录

- 新增 direct self/external line-search superset builder tests。
- 新增 `surfaceIPCBroadPhaseInternal.h`，集中 AABB builder、TLS pair collection、profiling counter helper 和 obstacle vertex accessor。
- 新增 `surfaceIPCSelfBroadPhase.cpp` 与 `surfaceIPCExternalBroadPhase.cpp`，分别承载 self/external broad phase 实现。
- `surfaceIPCBroadPhase.cpp` 收缩为 compatibility translation unit。
- `src/core/contact/CMakeLists.txt` 已加入新 broad phase `.cpp`。
- 验证中 `ctest` 当前 build tree 未发现测试条目，因此直接运行对应 gtest executables 作为实际 correctness gate。
