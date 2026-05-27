# IPC Core Refactor Plan: `SurfaceIPCCore` 拆分与目录分层

Source context:
- `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/ipc_friction.plan.md`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/contact/ipc/core/surfaceIPCCore.h`
- `/Users/jinceyang/Desktop/codebase/libpgo/src/core/contact/ipc/core/surfaceIPCCore.cpp`

## Summary

这份计划的目标是把当前过大的 `SurfaceIPCCore` 拆成更清晰的 IPC core 子模块，
并为后续 external contact、friction、prepared-state cache / broad phase optimization 留出稳定边界。

当前 `SurfaceIPCCore.cpp` 同时承担：

- IPC distance primitive 与分类逻辑；
- barrier 函数；
- CCD；
- Hessian PSD projection；
- surface topology 构建；
- self broad phase pair build；
- barrier energy / gradient / Hessian assembly；
- contact-feasible max step。

这已经超过一个 “core class” 应有的职责范围。重构方向固定为：

- `SurfaceIPCCore` 继续作为 surface-space IPC 对外 facade；
- IPC 数学、拓扑、broad phase、assembly、max-step 逐步拆成可测试小模块；
- 优先做行为保持型拆分，不把目录整理和算法优化混在同一批；
- 保留现有 public API，避免在 Phase 2/3 之前影响 `CIPCPotentialEnergy` 与
  `EmbeddedSurfaceIPCPotentialEnergy`。

## Current Repo Truth

截至本计划落地时，repo 已经完成一项前置拆分：

- `SpatialHashGrid` 已从 `SurfaceIPCCore.cpp` anonymous namespace 中抽出；
- 新文件：
  - `src/core/contact/ipc/broadPhase/spatialHashGrid.h`
  - `src/core/contact/ipc/broadPhase/spatialHashGrid.cpp`
  - `tests/src/core/contact/spatialHashGrid_gtest.cpp`
- 第一版 `SpatialHashGrid` 是纯 AABB broad-phase 工具，保持原实现语义：
  - `unordered_map<int64_t, vector<int>>`
  - 大质数 XOR cell hash
  - 调用方提供 `visitedStamp` 和 `stamp` 去重
  - 不做 workspace reuse / temporal coherence / prepared-state cache

这项拆分视为 **R0: broad phase utility extraction**，不在后续阶段重复规划。

## Target Directory Shape

最终建议把 IPC 相关实现从 `src/core/contact/` 的平铺结构中分到一层 `ipc/` 目录下。
目录深度保持克制：只引入一层 `ipc/` 和少数职责目录，不做过深嵌套。

目标结构：

```text
src/core/contact/
  CIPC.h/.cpp
  mappedSurfacePotentialEnergy.h/.cpp
  embeddedSurfaceIPCPotentialEnergy.h/.cpp
  embeddedSurfaceFloorPotentialEnergy.h/.cpp
  triangleMesh*.h/.cpp
  point*.h/.cpp

  ipc/
    core/
      surfaceIPCCore.h/.cpp
      surfaceIPCPairs.h

    geometry/
      ipcDistancePrimitives.h/.cpp
      ipcBarrier.h/.cpp
      ipcCCD.h/.cpp
      ipcHessianProjection.h/.cpp
      generated/
        CIPC_autogen.h
        CIPC_autogen_ll.h

    broadPhase/
      spatialHashGrid.h/.cpp
      surfaceIPCSelfBroadPhase.h/.cpp

    topology/
      surfaceIPCTopology.h/.cpp

    profiling/
      surfaceIPCProfiling.h
```

迁移原则：

- `CIPC.h/.cpp` 与 embedded surface energy wrappers 保留在 `src/core/contact/`
  顶层，作为现有 include 用户的稳定入口；
- `SurfaceIPCCore`、profiling、generated IPC math header 都收进 `ipc/` 目录；
- 不保留 top-level forwarding header；repo 内 include 必须一次性迁移到真实
  `ipc/...` 路径，外部 include 用户接受一次显式迁移；
- tests 可以先保持在 `tests/src/core/contact/`，等 IPC 子模块稳定后再考虑镜像目录。

## R1: Geometry Math Extraction

### Goal

把不依赖 `SurfaceIPCCore` 状态的 IPC 数学函数从旧的 top-level
`surfaceIPCCore.h/.cpp` 拆到 `ipc/geometry/`。

### Files

新增：

- `src/core/contact/ipc/geometry/ipcDistancePrimitives.h/.cpp`
- `src/core/contact/ipc/geometry/ipcBarrier.h/.cpp`
- `src/core/contact/ipc/geometry/ipcCCD.h/.cpp`
- `src/core/contact/ipc/geometry/ipcHessianProjection.h/.cpp`

迁移：

- `PTDistType` / `EEDistType`
- namespace `distance`
- namespace `barrier`
- namespace `ccd`
- `projectToPSD`

拆分公共 pair 类型：

- `PTPair` / `EEPair` 放到 `ipc/core/surfaceIPCPairs.h`；
- `SurfaceIPCCore::getPTPairs()` / `getEEPairs()` 继续返回这些类型；
- lower-level IPC helpers include `surfaceIPCPairs.h`，不反向 include facade。

### Rules

- 不改函数签名，不改 namespace 语义；
- 不改 `ipc/geometry/generated/CIPC_autogen.h` /
  `ipc/geometry/generated/CIPC_autogen_ll.h` 的生成内容；
- `ipc/core/surfaceIPCCore.h` 只 include geometry headers，不再直接声明所有数学函数；
- CMake 手写文件列表同步更新；
- 添加或保留现有 `surfaceIPCCore_gtest`、`cipcPotentialEnergy_gtest`、
  `embeddedSurfaceIPCPotentialEnergy_gtest` 作为行为回归。

### Validation

必须通过：

```bash
cmake --build build/base_no_mkl --target surfaceIPCCore_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest -j2
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
```

## R2: Broad Phase Directory Migration

### Goal

把已经抽出的 `SpatialHashGrid` 放进 `ipc/broadPhase/`，并为后续 self/external
broad phase 拆分预留位置。

### Files

移动：

- `src/core/contact/spatialHashGrid.h`
  -> `src/core/contact/ipc/broadPhase/spatialHashGrid.h`
- `src/core/contact/spatialHashGrid.cpp`
  -> `src/core/contact/ipc/broadPhase/spatialHashGrid.cpp`

不新增 forwarding header。所有 repo 内 include 改为：

```cpp
#include "ipc/broadPhase/spatialHashGrid.h"
```

### Rules

- 这一步仍然只做文件位置迁移；
- 不引入 `CellKey{x,y,z}`；
- 不做 bucket/workspace reuse；
- 不把 active pair cache 放进 `SpatialHashGrid`。

### Validation

必须通过：

```bash
cmake --build build/base_no_mkl --target spatialHashGrid_gtest surfaceIPCCore_gtest -j2
./build/base_no_mkl/tests/src/core/contact/spatialHashGrid_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
```

## R3: Surface Topology Extraction

### Goal

把 `setMesh()` 中和 rest topology / weights 有关的状态抽成独立 value object，
让 `SurfaceIPCCore` 不再直接拥有所有拓扑构建细节。

### Proposed Type

新增：

- `src/core/contact/ipc/topology/surfaceIPCTopology.h/.cpp`

建议类型：

```cpp
struct SurfaceIPCTopology
{
  int numVerts = 0;
  std::vector<std::array<int, 3>> triangles;
  std::vector<std::array<int, 2>> edges;
  std::vector<double> vertexArea;
  std::vector<double> triArea;
  std::vector<double> edgeLength;

  void setMesh(const EigenSupport::MXd &V, const EigenSupport::MXi &F);
  int numSurfaceDOFs() const { return 3 * numVerts; }
};
```

迁移：

- `buildEdges()`
- `buildAreaWeights(...)`
- `triangles_`
- `edges_`
- `vertexArea_`
- `triArea_`
- `edgeLength_`

不迁移：

- `ptPairs_` / `eePairs_`，它们属于当前 geometry state 下的 active pair 工作集；
- `dhat / kappa / eps_ee / slackness`，它们属于 IPC 参数；
- `contactClampCount_ / minContactFeasibleAlphaThisSolve_`，它们属于 solver stats。

### Cleanup Candidate

`allDOFs_ / vertexTriAdj_ / edgeVertAdj_` 当前只在 `SurfaceIPCCore` 中 build/copy，
未被实际 contact path 使用。R3 前应先用 `rg` 再确认一次：

```bash
rg -n "allDOFs_|vertexTriAdj_|edgeVertAdj_" src tests
```

如果仍无使用者，R3 可以删除这些成员与 `buildAdjacency()`，并用现有 IPC tests
证明行为不变。

### Validation

必须通过 R1/R2 中所有 contact tests，并额外检查：

- `SurfaceIPCCore::getNumSurfaceVertices()`
- `SurfaceIPCCore::getNumSurfaceDOFs()`
- `CIPCPotentialEnergy::setMesh(...)`
- `EmbeddedSurfaceIPCPotentialEnergy` constructor path

## R4: Self Broad Phase Extraction

### Goal

把 `SurfaceIPCCore::findCollisionPairs(...)` 拆成独立 self broad phase builder。
这一步为 Phase 2 external contact 提供并行结构：self/external 都能复用
`SpatialHashGrid`，但 pair identity 仍由 `SurfaceIPCCore` 统一出口管理。

### Proposed Type

新增：

- `src/core/contact/ipc/broadPhase/surfaceIPCSelfBroadPhase.h/.cpp`

建议接口：

```cpp
class SurfaceIPCSelfBroadPhase
{
public:
  void buildPairs(
    const SurfaceIPCTopology &topology,
    EigenSupport::ConstRefVecXd positions,
    double dhat,
    std::vector<PTPair> &ptPairs,
    std::vector<EEPair> &eePairs) const;
};
```

### Rules

- `buildPairs(...)` 只负责 candidate build + geometric distance threshold filter；
- 不计算 barrier energy / gradient / Hessian；
- 不参与 CCD max-step；
- 不持有跨调用 cache；
- pair ordering 必须与当前 `findCollisionPairs(...)` 保持一致。

### Validation

除了 contact tests，还需要新增一个 focused test：

- 给定固定 two-triangle mesh 和 positions；
- 旧路径与新 `SurfaceIPCSelfBroadPhase` 产生的 PT/EE pair 数量与内容一致；
- 如果旧路径已被移除，则以 `SurfaceIPCCore::computeEnergy(...)` 后的
  `getPTPairs()` / `getEEPairs()` 作为行为出口验证。

## R5: Max-Step And Assembly Split

### Goal

在前面拆分稳定后，再考虑拆出剩余高风险部分：

- barrier assembly：
  - energy
  - gradient
  - Hessian
  - combined `computeAll`
- contact max-step：
  - swept AABB build
  - PT CCD
  - EE CCD
  - contact clamp stats update

### Suggested Types

可选新增：

- `src/core/contact/ipc/core/surfaceIPCBarrierAssembler.h/.cpp`
- `src/core/contact/ipc/core/surfaceIPCMaxStep.h/.cpp`

### Rules

- R5 只能在 R1-R4 完成且 tests 稳定后执行；
- 不在 R5 同时引入 prepared-state cache；
- 不改 `computeEnergy / computeGradient / computeHessian / computeAll /
  computeMaxStepSize` 的 public API；
- 如果开始引入 prepared state，必须另开 plan，因为它会改变 lifecycle 语义。

## Public API And Compatibility

### Must Stay Stable

以下 API 在本计划内保持稳定：

- `SurfaceIPCCore::setMesh`
- `SurfaceIPCCore::computeEnergy`
- `SurfaceIPCCore::computeGradient`
- `SurfaceIPCCore::computeHessian`
- `SurfaceIPCCore::computeAll`
- `SurfaceIPCCore::computeMaxStepSize`
- `SurfaceIPCCore::getPTPairs`
- `SurfaceIPCCore::getEEPairs`
- `CIPCPotentialEnergy`
- `EmbeddedSurfaceIPCPotentialEnergy`

### Include Compatibility

本计划采用 no-forwarding policy：

- repo 内 include 全量更新到真实 `ipc/...` 路径；
- 不保留 top-level forwarding header；
- 外部 include 用户需要迁移 include path；
- 不在同一批既移动 header 又重命名 class / namespace。

### Namespace

首期保持：

```cpp
namespace pgo::Contact::CIPC
```

不引入新的 `pgo::Contact::IPC` namespace，避免产生命名迁移和 include 迁移两类风险。

## Testing Matrix

每个 R 阶段至少运行：

```bash
cmake --build build/base_no_mkl --target spatialHashGrid_gtest surfaceIPCCore_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest -j2
./build/base_no_mkl/tests/src/core/contact/spatialHashGrid_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
git diff --check
```

如果阶段涉及 run-time IPC wiring，还要补：

```bash
cmake --build build/base_no_mkl --target runIPCSim_gtest -j2
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
```

## Non-Goals

本重构计划明确不做：

- 不改 IPC barrier 数学；
- 不改 CCD 算法；
- 不改 pair threshold 规则；
- 不改 pair ordering；
- 不引入 temporal coherence；
- 不引入 wrapper-level exact cache；
- 不引入 `SurfaceIPCCore::prepareForState(...)`；
- 不做 external IPC；
- 不做 friction；
- 不把 `runSim` / `runShellSim` 默认路径迁移到 unified IPC。

## Recommended Execution Order

1. R1 geometry math extraction
2. R2 broad phase directory migration
3. R3 topology extraction and unused adjacency cleanup
4. R4 self broad phase extraction
5. R5 assembly / max-step split

推荐每个 R 阶段单独提交，避免把“移动文件”“改 include”“改 ownership”
和“行为性修改”混在同一个 diff 里。
