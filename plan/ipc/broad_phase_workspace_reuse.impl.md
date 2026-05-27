# Broad Phase Prepared Active Pairs 实现记录

源计划：`plan/ipc/broad_phase_workspace_reuse.plan.md`

## 实现摘要

- [x] Task 0：记录优化前 baseline profiling 到 `plan/ipc/broad_phase_prepared_pairs.baseline.md`。
- [x] Task 1：`SurfaceIPCCore` 增加 prepared active pair state 和无 `x_surf` 参数的 prepared consumers。
- [x] Task 2：`CIPCPotentialEnergy` 委托 `SurfaceIPCCore` 判断 prepared positions，`func()` / `gradient()` / `hessianDirect()` 复用 prepared pairs。
- [x] Task 3：`EmbeddedSurfaceIPCPotentialEnergy` 委托 `SurfaceIPCCore` 判断 mapped surface positions。
- [x] Task 4：新增 prepared profiling section，并更新 profiling 回归测试。
- [x] Task 5：全量 IPC/contact 回归。

## 已实现行为

- `SurfaceIPCCore::prepareForSurfacePositions(x_surf)` 会保存 `x_surf` 并构建 static PT/EE active pairs。
- `computeEnergyWithPreparedPairs()` / `computeGradientWithPreparedPairs()` / `computeHessianWithPreparedPairs()` / `computeAllWithPreparedPairs()` 只消费 prepared state，不再接收 `x_surf`。
- 未 prepared 时调用 prepared consumers 会抛出 `std::logic_error`。
- 旧 `computeEnergy(x)` / `computeGradient(x)` / `computeHessian(x)` / `computeAll(x)` 保持可用，内部变成 `prepareForSurfacePositions(x)` 加 prepared consumer。
- `CIPCPotentialEnergy` 和 `EmbeddedSurfaceIPCPotentialEnergy` 不再保存自己的 surface positions cache；它们直接调用 `SurfaceIPCCore::isPreparedFor()`，由 core 作为唯一 prepared-state owner。
- 同一 state 连续 `func/gradient/hessianDirect` 只触发一次 `contact.surface.pair_build.static`。
- `computeMaxStepSize()` 保持原来的 swept broad phase 路径，不复用 static prepared pairs。

## 与源计划的实现期澄清

- 源计划写到 `setParameters()` 可保守 invalidate；实现中 `SurfaceIPCCore::setParameters()` 仍会 invalidate。
- 为了让 core prepared state 真正命中，`CIPCPotentialEnergy::syncCoreParametersFromWrapper()` 只有在 wrapper 参数值发生变化时才调用 `core.setParameters()`。否则每次 `func/gradient/hessianDirect` 都会自己打掉 prepared state。
- Wrapper profiling 从 legacy `contact.surface.energy` 迁移到 prepared 路径的 `contact.surface.prepare_active_pairs` 和 `contact.surface.prepared_energy`。旧 core direct API 仍保留 `contact.surface.energy` 外层 section。

## 验证记录

- [x] RED：`cmake --build build/base_no_mkl --target surfaceIPCCore_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest -j2` 在实现前因 `SurfaceIPCCore` prepared API 缺失而编译失败。
- [x] GREEN build：`cmake --build build/base_no_mkl --target surfaceIPCCore_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest cipcProfiling_gtest -j2`
- [x] `./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest`
- [x] `./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest`
- [x] `./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest`
- [x] `./build/base_no_mkl/tests/src/core/contact/cipcProfiling_gtest`
- [x] Full build：`cmake --build build/base_no_mkl --target ipcGeometry_gtest surfaceIPCTopology_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCMaxStep_gtest surfaceIPCBarrierAssembler_gtest spatialHashGrid_gtest surfaceIPCCore_gtest cipcProfiling_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest embeddedSurfaceFloorPotentialEnergy_gtest runIPCSim_gtest -j2`
- [x] Full run：`ipcGeometry_gtest`, `surfaceIPCTopology_gtest`, `surfaceIPCSelfBroadPhase_gtest`, `surfaceIPCMaxStep_gtest`, `surfaceIPCBarrierAssembler_gtest`, `spatialHashGrid_gtest`, `surfaceIPCCore_gtest`, `cipcProfiling_gtest`, `cipcPotentialEnergy_gtest`, `embeddedSurfaceIPCPotentialEnergy_gtest`, `embeddedSurfaceFloorPotentialEnergy_gtest`, `runIPCSim_gtest`
- [x] Include hygiene：`rg -n '#include "(surfaceIPCCore|surfaceIPCProfiling|spatialHashGrid|CIPC_autogen|CIPC_autogen_ll)\.h"|../../surfaceIPCCore' src tests` 无输出。
- [x] Whitespace：`git diff --check`
