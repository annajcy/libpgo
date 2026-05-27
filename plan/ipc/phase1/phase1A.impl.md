# Phase 1A Implementation Record

Source plan: `/Users/jinceyang/Desktop/codebase/libpgo/plan/ipc/phase1A.plan.md`

## Implemented

`Phase 1A` 已整体落地，当前 repo-truth 为：

- scoped profiling 基础设施已前置完成，并继续保留为仓库级 `profiling` core 库
- 新增 `SurfaceIPCCore`，承接 surface-space IPC 的 low-level 数学与拓扑/候选对构建
- `CIPCPotentialEnergy` 已收成 thin wrapper：
  - 保留 legacy public 参数与 `PotentialEnergy` 契约
  - `hessian()` / `createHessian()` 继续作为 throw override 保留
  - public `findCollisionPairs(const VXd &)` 已删除
  - 通过 `mutable SurfaceIPCCore core;` + `syncCoreParametersFromWrapper() const` 做参数同步
  - floor penalty 保持在 wrapper，并明确为 post-pass
- profiling 已迁到真实边界：
  - `contact.surface.*` 接在 `surfaceIPCCore.cpp`
  - `contact.wrapper.sync` 接在 `syncCoreParametersFromWrapper()`
  - `contact.wrapper.floor_post_pass` 只包 floor energy/gradient/Hessian 补项 helper
- `CIPC_autogen.h` / `CIPC_autogen_ll.h` 的直接 include ownership 已迁到 `surfaceIPCCore.cpp`
- `runShellSim` 继续通过 `CIPCPotentialEnergy` 路径工作，但 self-contact 数学实现只剩 `SurfaceIPCCore` 一份

## Code Shape

实现后的职责边界固定为：

- `src/core/contact/surfaceIPCCore.h/.cpp`
  - `PTDistType / EEDistType`
  - `PTPair / EEPair`
  - `distance::* / barrier::* / ccd::* / projectToPSD`
  - `setMesh`
  - `findCollisionPairs`
  - `computeMaxStepSize`
  - `computeEnergy / computeGradient / computeHessian / computeAll`
- `src/core/contact/CIPC.h/.cpp`
  - wrapper 参数与输入语义转换
  - wrapper -> core 参数同步
  - floor post-pass
  - thin delegation 到 `SurfaceIPCCore`

`SurfaceIPCCore::setParameters(...)` 当前已按 plan 固定为：

- 只更新参数 storage
- 不触发 topology / adjacency / area weight 的 re-preparation
- topology 相关预处理仍仅在 `setMesh(...)` 中完成

wrapper 路径上的参数 source-of-truth 也已固定为：

- `CIPCPotentialEnergy` public fields 是唯一 source of truth
- wrapper 入口前统一调用 `syncCoreParametersFromWrapper() const`
- wrapper-level 测试只通过 wrapper public fields 改参数，不绕过 wrapper 直接写 core

## Tests And Validation

本批新增并保留通过的测试：

- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/profiling/scopedProfileSection_gtest.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/contact/cipcProfiling_gtest.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/contact/surfaceIPCCore_gtest.cpp`
- `/Users/jinceyang/Desktop/codebase/libpgo/tests/src/core/contact/cipcPotentialEnergy_gtest.cpp`

最终验证命令：

```bash
cmake --build --preset base_no_mkl_debug --target scopedProfileSection_gtest surfaceIPCCore_gtest cipcPotentialEnergy_gtest runSim_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "ScopedProfileSection|SurfaceIPCCore|CIPCPotentialEnergy|RunShellSimCliLoggingGTest|CIPCProfilingGTest"
```

本地验证结果：通过。

## Intentional Drift From Source Plan

相对 source plan，有一条实现期确认后的测试语义收窄：

- source plan 写的是 `surfaceIPCCore` 做 gradient/Hessian FD 校验
- 当前 repo-truth 下 `computeHessian()` 延续旧实现语义，会对每个 local Hessian 做 `projectToPSD`
- 因此返回值不是“原始 gradient Jacobian”的严格中心差分结果
- 实际落地测试改为验证：
  - energy/gradient 仍做严格 FD
  - returned Hessian 仍要求对称
  - returned Hessian 仍要求 PSD
  - returned Hessian 与未投影的 gradient FD Hessian 在整体量级上保持接近

这不是新引入的 Phase 1A 语义变化，而是旧 `CIPC` 语义在 core 抽取后被显式记录下来。
