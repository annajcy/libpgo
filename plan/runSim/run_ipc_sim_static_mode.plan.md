# runIPCSim Static Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在新 `runIPCSim` 入口里支持 `sim-type = static`，默认 IPC backend 和 `--legacy` backend 都可用，同时保留已经迁移的 dynamic 行为。

**Architecture:** `runIPCSim` 继续保持单入口：setup 负责构建 `IpcSimulationContext`，runtime config 负责选择 `dynamic/static`，app 层按 mode 分发到 dynamic loop 或 static solver。static solver 复用现有 context/output/backend 抽象；IPC static 把 IPC/floor energy 加进静态 Newton energy，legacy static 严格复刻旧 `runSim static`，不把 penalty contact 加进 static energy。

**Tech Stack:** C++17, CMake, GoogleTest, `NonlinearOptimization::NewtonSolver`, `NonlinearOptimization::PotentialEnergies`, `PredefinedPotentialEnergies::LinearPotentialEnergy`, existing `runIPCSimCore`.

---

## 当前前提

- 当前工作区已经包含上一阶段的 `runIPCSim --legacy` / contact backend 重构改动，但尚未提交。
- 执行本 plan 前，先把上一阶段改动提交，避免 static mode 改动和 legacy 迁移改动混在同一个 commit 中。
- 本 plan 不恢复旧 `runSim` / `runShellSim` executable。static 支持进入 `runIPCSim`。

## 行为决策

1. `sim-type = dynamic`
   - 默认 IPC backend：行为保持当前 `runIPCSim` dynamic 语义。
   - `--legacy` backend：行为保持当前已迁移的 volume penalty contact dynamic 语义。

2. `sim-type = static`
   - 默认 IPC backend：支持 shell/tet/cubic static solve。
   - `--legacy` backend：只支持 tet/cubic volume static solve；shell legacy 仍然拒绝。
   - static 模式不支持 restart；如果 config 里 `restart-from-u = true`，直接报错。
   - static 模式忽略 `init-vel` 的动力学意义，但仍允许字段存在，保持配置兼容。
   - static 模式要求 `init-disp = [0, 0, 0]`，延续当前 setup 对初始位移的约束。

3. legacy static contact 语义
   - 旧 `runSim static` 只装配 `elasticEnergy + pullingEnergies - gravityLinearEnergy`，没有把 penalty external/self contact 加进 static energy。
   - 因此 `runIPCSim --legacy` + `sim-type = static` 也不加入 penalty contact energy。
   - `external-objects` 和 contact 参数可以存在，用于兼容旧 JSON，但 static solve 不使用它们。

4. IPC static contact 语义
   - `collisionHandler` 加入 static energy。
   - `floorPotentialEnergies` 加入 static energy。
   - `surface-pressure-force` 若启用，作为满 ramp 的线性外力加入 static energy。
   - dynamic obstacle 的 static pose 取最终 step 的 obstacle time：`staticObstacleTime = timestep * max(numSimSteps - 1, 0)`。
   - floor motion 的 static height 取最终 frame：`finalFrame = max(numSimSteps - 1, 0)`。

5. static 输出语义
   - 使用 unified output layout：`<output>/surface/ret0000.obj`。
   - 写 `<output>/states/deform0000.u`，其中 `uvel` 和 `uacc` 为 0，便于后续工具/测试统一读取。
   - 若 `output-von-mises = true` 且是 volume simulation，写 `<output>/stress/von_mises0000.json`。
   - 不写旧 `runSim static` 的单文件 `output` 路径；`output` 始终表示 output folder。

## 文件结构

### 修改

- `src/tools/runSim/runIPCSimConfig.h/.cpp`
  - 增加 `RunIPCSimSimulationMode` enum。
  - `RunIPCSimRuntimeConfig` 增加 `simulationMode`。
  - `parseRunIPCSimRuntimeConfig()` 接受 `dynamic/static`。

- `src/tools/runSim/runIPCSimApp.cpp`
  - 根据 `runtimeConfig.simulationMode` 分发 dynamic/static。
  - static 模式拒绝 restart。

- `src/tools/runSim/runIPCSimContactBackend.h`
  - 增加 static energy hook。

- `src/tools/runSim/runIPCSimIpcContactBackend.cpp`
  - 实现 IPC static hook：设置 obstacle time、floor final height，并把 IPC/floor energies 加入 static energy。

- `src/tools/runSim/runIPCSimLegacyPenaltyContact.h/.cpp`
  - 实现 legacy static hook：默认 no-op，明确旧 static 不含 penalty contact。

- `src/tools/runSim/runIPCSimStaticSolve.h/.cpp`
  - 新增 static solver 模块。
  - 负责设置 pulling target、装配 static energy、运行 Newton、写 unified outputs。

- `src/tools/runSim/runIPCSimOutput.h/.cpp`
  - 增加 helper：从 displacement 写 surface frame，避免 dynamic loop 和 static solver 复制 surface projection 逻辑。

- `src/tools/runSim/runIPCSimLoop.cpp`
  - 改用 output helper 写 surface。

- `src/tools/runSim/CMakeLists.txt`
  - 把 `runIPCSimStaticSolve.cpp` 加入 `RUN_IPC_SIM_CORE_SOURCES`。

- `tests/src/tools/runIPCSim_gtest.cpp`
  - 增加 static mode 测试。

### 不修改

- 不恢复 `src/tools/runSim/runSim.cpp`。
- 不恢复 `src/tools/runSim/runShellSim.cpp`。
- 不恢复 `examples/legacy`。
- 不改 C/Python `pgo_run_sim_from_config()`；它不是本 plan 范围。

---

## Task 0: 提交上一阶段 legacy 迁移改动

**Files:** no source edits.

- [ ] **Step 1: 确认当前工作区是上一阶段计划内改动**

Run:

```bash
git status --short
```

Expected:

- 显示上一阶段 `runIPCSim --legacy`、删除旧 executable、fixtures 迁移、README 更新等改动。
- 不应有本 static plan 的源码改动。

- [ ] **Step 2: 跑上一阶段验证**

Run:

```bash
git diff --check
cmake --build --preset base_no_mkl_release --target runIPCSim runIPCSim_gtest runSimShared_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runSimShared_gtest
```

Expected:

- `git diff --check` PASS。
- build PASS。
- `runIPCSim_gtest` PASS。
- `runSimShared_gtest` PASS。

- [ ] **Step 3: Commit 上一阶段**

Run:

```bash
git add README.md examples/ipc/README.md src/python/pypgo/pgo_test_01.py src/tools/runSim tests
git add -u examples/legacy src/tools/runSim tests
git commit -m "feat: migrate legacy volume contact to runIPCSim"
```

Expected:

- commit 成功。
- `git status --short` 只剩本 plan 文件或为空。

---

## Task 1: 为 static mode 写失败测试

**Files:**

- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: 增加 static config helper**

在已有 test helpers 附近增加一个 helper，用现有 dynamic helper 生成 config 后替换 `sim-type`：

```cpp
std::string makeStaticConfigFromVolumeConfig(std::string configText)
{
  const std::string dynamicToken = "\"sim-type\": \"dynamic\"";
  const std::string staticToken = "\"sim-type\": \"static\"";
  const std::size_t pos = configText.find(dynamicToken);
  if (pos == std::string::npos)
    throw std::runtime_error("test config does not contain dynamic sim-type");
  configText.replace(pos, dynamicToken.size(), staticToken);
  return configText;
}
```

如果当前 test file 已经有 JSON object-based helper，优先用 JSON API 修改 `sim-type`，不要手写 fragile string replacement。

- [ ] **Step 2: 增加 CLI parse 测试**

新增测试：

```cpp
TEST(RunIPCSimConfigGTest, RuntimeConfigAcceptsStaticMode)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "static.json";
  writeTextFile(configPath, R"({
    "g": [0, -9.81, 0],
    "init-vel": [0, 0, 0],
    "init-disp": [0, 0, 0],
    "scale": 1.0,
    "timestep": 0.001,
    "num-timestep": 1,
    "dump-interval": 1,
    "damping-params": [0, 0],
    "sim-type": "static",
    "solver-eps": 1e-4,
    "solver-max-iter": 5,
    "output": "ret-static"
  })");

  pgo::ConfigFileJSON config;
  ASSERT_TRUE(config.open(configPath.string().c_str()));
  const pgo::RunIPCSim::RunIPCSimRuntimeConfig runtimeConfig =
    pgo::RunIPCSim::parseRunIPCSimRuntimeConfig(config);
  EXPECT_EQ(runtimeConfig.simulationMode, pgo::RunIPCSim::RunIPCSimSimulationMode::Static);
}
```

Expected now:

- This test fails to compile because `RunIPCSimSimulationMode` does not exist.

- [ ] **Step 3: 增加默认 IPC static smoke**

新增测试：

```cpp
TEST(RunIPCSimStaticGTest, StaticTetIpcWritesUnifiedSurfaceAndState)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "tet-static-ipc.json";
  const fs::path outputDir = tempDir.path() / "tet-static-output";
  writeTextFile(configPath, makeStaticConfigFromVolumeConfig(
    makeVolumeConfig(tetIPCExampleDir(), outputDir, "tet-mesh", "box.veg", 1, 1)));

  ASSERT_EQ(pgo::RunIPCSim::runFromConfig(configPath, {}), 0);
  EXPECT_TRUE(fs::exists(outputDir / "surface" / "ret0000.obj"));
  EXPECT_TRUE(fs::exists(outputDir / "states" / "deform0000.u"));
}
```

如果当前 helper 名不是 `makeVolumeConfig` / `tetIPCExampleDir`，使用文件里现有同等 helper 名称。

Expected now:

- 编译或运行失败，因为 static mode 当前被 `parseRunIPCSimRuntimeConfig()` 拒绝。

- [ ] **Step 4: 增加 legacy static smoke**

新增测试：

```cpp
TEST(RunIPCSimStaticGTest, StaticLegacyTetWritesUnifiedSurfaceAndState)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "tet-static-legacy.json";
  const fs::path outputDir = tempDir.path() / "tet-static-legacy-output";
  writeTextFile(configPath, makeStaticConfigFromVolumeConfig(
    makeLegacyVolumeConfig(fs::path(kLegacyTetBoxDir), outputDir, "tet-mesh", 1, 1)));

  pgo::RunIPCSim::RunIPCSimOptions options;
  options.contactBackendKind = pgo::RunIPCSim::ContactBackendKind::LegacyPenalty;
  ASSERT_EQ(pgo::RunIPCSim::runFromConfig(configPath, options), 0);
  EXPECT_TRUE(fs::exists(outputDir / "surface" / "ret0000.obj"));
  EXPECT_TRUE(fs::exists(outputDir / "states" / "deform0000.u"));
}
```

Expected now:

- FAIL for same static-mode rejection.

- [ ] **Step 5: 增加 static restart rejection 测试**

新增测试：

```cpp
TEST(RunIPCSimStaticGTest, StaticRejectsRestartFromU)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "static-restart.json";
  const fs::path outputDir = tempDir.path() / "static-restart-output";
  std::string configText = makeStaticConfigFromVolumeConfig(
    makeVolumeConfig(tetIPCExampleDir(), outputDir, "tet-mesh", "box.veg", 1, 1));
  const std::string outputField = "\"output\": ";
  const std::size_t insertPos = configText.rfind(outputField);
  ASSERT_NE(insertPos, std::string::npos);
  configText.insert(insertPos, "  \"restart-from-u\": true,\n");
  writeTextFile(configPath, configText);

  EXPECT_NE(pgo::RunIPCSim::runFromConfig(configPath, {}), 0);
}
```

Expected after implementation:

- PASS and log contains a clear static/restart error if this test captures logs; if it does not capture logs, exit code is enough.

- [ ] **Step 6: Run failing tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimConfigGTest.RuntimeConfigAcceptsStaticMode:RunIPCSimStaticGTest.*'
```

Expected:

- Before implementation, FAIL due to missing enum/static rejection.

---

## Task 2: Runtime config 支持 `dynamic/static`

**Files:**

- Modify: `src/tools/runSim/runIPCSimConfig.h`
- Modify: `src/tools/runSim/runIPCSimConfig.cpp`

- [ ] **Step 1: 增加 mode enum**

在 `RunIPCSimRuntimeConfig` 前增加：

```cpp
enum class RunIPCSimSimulationMode
{
  Dynamic,
  Static,
};
```

在 `RunIPCSimRuntimeConfig` 中增加字段：

```cpp
RunIPCSimSimulationMode simulationMode = RunIPCSimSimulationMode::Dynamic;
```

- [ ] **Step 2: 更新 parser**

把当前：

```cpp
const std::string simType = config.getString("sim-type");
if (simType != "dynamic")
  throw std::invalid_argument("runIPCSim phase2 only supports `sim-type = dynamic`.");
```

替换为：

```cpp
const std::string simType = config.getString("sim-type");
if (simType == "dynamic") {
  runtimeConfig.simulationMode = RunIPCSimSimulationMode::Dynamic;
}
else if (simType == "static") {
  runtimeConfig.simulationMode = RunIPCSimSimulationMode::Static;
}
else {
  throw std::invalid_argument("runIPCSim only supports `sim-type = dynamic` or `sim-type = static`.");
}
```

- [ ] **Step 3: Run config test**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimConfigGTest.RuntimeConfigAcceptsStaticMode'
```

Expected:

- PASS.
- static smoke tests still fail because app has no static solver.

---

## Task 3: 抽取 surface/state 输出 helper

**Files:**

- Modify: `src/tools/runSim/runIPCSimOutput.h`
- Modify: `src/tools/runSim/runIPCSimOutput.cpp`
- Modify: `src/tools/runSim/runIPCSimLoop.cpp`

- [ ] **Step 1: 在 output header 增加 helper 声明**

在 `RunIPCSimOutput` public section 增加：

```cpp
void writeStateAndSurfaceFrame(
  int frame,
  int outputFrame,
  const IpcSimulationContext &context,
  const EigenSupport::VXd &u,
  const EigenSupport::VXd &uvel,
  const EigenSupport::VXd &uacc,
  double scale,
  bool writeStateFile,
  bool writeSurfaceFile) const;
```

- [ ] **Step 2: 实现 helper**

在 `runIPCSimOutput.cpp` 中实现：

```cpp
void RunIPCSimOutput::writeStateAndSurfaceFrame(
  int frame,
  int outputFrame,
  const IpcSimulationContext &context,
  const ES::VXd &u,
  const ES::VXd &uvel,
  const ES::VXd &uacc,
  double scale,
  bool writeStateFile,
  bool writeSurfaceFile) const
{
  if (writeStateFile)
    writeState(frame, u, uvel, uacc);

  if (!writeSurfaceFile)
    return;

  pgo::Mesh::TriMeshGeo mesh = context.surfaceMesh;
  ES::VXd usurf(context.surfaceRestPositions.size());
  ES::mv(context.surfaceFromSimulationDispMap, u, usurf);
  const ES::VXd psurf = context.surfaceRestPositions + usurf;
  for (int vi = 0; vi < mesh.numVertices(); ++vi)
    mesh.pos(vi) = psurf.segment<3>(vi * 3) / scale;
  writeSurface(outputFrame, mesh);
}
```

- [ ] **Step 3: dynamic loop 使用 helper**

把 `runIPCSimLoop.cpp` 里写 state/surface 的重复逻辑替换为：

```cpp
const bool dumpDeformThisFrame = runtimeConfig.dumpDeformEveryFrame || (framei % runtimeConfig.frameGap == 0);
const bool dumpSurfaceThisFrame = (framei % runtimeConfig.frameGap == 0);
output.writeStateAndSurfaceFrame(
  framei,
  framei / runtimeConfig.frameGap,
  context,
  session.u,
  session.uvel,
  session.uacc,
  runtimeConfig.scale,
  dumpDeformThisFrame,
  dumpSurfaceThisFrame);
```

保留 `output-von-mises` 逻辑不变。

- [ ] **Step 4: Run dynamic regression**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimCliGTest.TetOneTimestepSmokeWritesDeformAndRet:RunIPCSimCliGTest.OneTimestepShellSmokeSucceeds'
```

Expected:

- PASS.
- Output paths remain `states/deform0000.u` and `surface/ret0000.obj`.

---

## Task 4: 给 contact backend 增加 static energy hook

**Files:**

- Modify: `src/tools/runSim/runIPCSimContactBackend.h`
- Modify: `src/tools/runSim/runIPCSimIpcContactBackend.cpp`
- Modify: `src/tools/runSim/runIPCSimLegacyPenaltyContact.cpp`

- [ ] **Step 1: 前向声明 static energy aggregate**

在 `runIPCSimContactBackend.h` 增加：

```cpp
namespace pgo::NonlinearOptimization
{
class PotentialEnergies;
}
```

在 `RunIPCSimContactBackend` 增加 pure virtual：

```cpp
virtual void addStaticEnergies(
  const RunIPCSimRuntimeConfig &runtimeConfig,
  IpcSimulationContext &context,
  NonlinearOptimization::PotentialEnergies &energyAll) = 0;
```

- [ ] **Step 2: IPC backend 实现 static hook**

在 `runIPCSimIpcContactBackend.cpp` include：

```cpp
#include "potentialEnergies.h"
```

在 `IpcContactBackend` 中实现：

```cpp
void addStaticEnergies(const RunIPCSimRuntimeConfig &runtimeConfig,
  IpcSimulationContext &context,
  NonlinearOptimization::PotentialEnergies &energyAll) override
{
  const int finalFrame = runtimeConfig.numSimSteps > 0 ? runtimeConfig.numSimSteps - 1 : 0;
  for (std::size_t fi = 0; fi < context.floorPotentialEnergies.size(); ++fi)
    context.floorPotentialEnergies[fi]->setFloorHeight(floorHeightAtFrame(context.floorMotionStates[fi], finalFrame));

  const double staticObstacleTime = runtimeConfig.timestep * static_cast<double>(finalFrame);
  context.collisionHandler->setObstacleTime(staticObstacleTime);
  energyAll.addPotentialEnergy(context.collisionHandler, 1.0);

  for (const auto &forceModel : context.extraGeneralImplicitForceModels)
    energyAll.addPotentialEnergy(forceModel, 1.0);
}
```

Important:

- If `extraGeneralImplicitForceModels` currently contains the same floor energy objects as `floorPotentialEnergies`, this adds floors exactly once because it iterates `extraGeneralImplicitForceModels`; do not add `floorPotentialEnergies` separately.
- `collisionHandler` and floor energies are `NonlinearOptimization::PotentialEnergy` compatible.

- [ ] **Step 3: legacy backend 实现 no-op static hook**

在 `LegacyPenaltyContactBackend` 中实现：

```cpp
void addStaticEnergies(const RunIPCSimRuntimeConfig &,
  IpcSimulationContext &,
  NonlinearOptimization::PotentialEnergies &) override
{
}
```

Add a short comment:

```cpp
// Old runSim static mode did not add legacy penalty contact energies.
```

- [ ] **Step 4: 编译验证**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSimCore
```

Expected:

- PASS.

---

## Task 5: 新增 static solver 模块

**Files:**

- Create: `src/tools/runSim/runIPCSimStaticSolve.h`
- Create: `src/tools/runSim/runIPCSimStaticSolve.cpp`
- Modify: `src/tools/runSim/CMakeLists.txt`

- [ ] **Step 1: 创建 header**

Create `src/tools/runSim/runIPCSimStaticSolve.h`:

```cpp
#pragma once

namespace pgo::RunIPCSim
{
struct IpcSimulationContext;
struct RunIPCSimRuntimeConfig;
class RunIPCSimOutput;

void runIPCSimStaticSolve(
  const RunIPCSimRuntimeConfig &runtimeConfig,
  IpcSimulationContext &context,
  const RunIPCSimOutput &output);
}  // namespace pgo::RunIPCSim
```

- [ ] **Step 2: 创建 implementation includes**

Create `src/tools/runSim/runIPCSimStaticSolve.cpp` with includes:

```cpp
#include "runIPCSimStaticSolve.h"

#include "EigenSupport.h"
#include "linearPotentialEnergy.h"
#include "NewtonSolver.h"
#include "potentialEnergies.h"
#include "runIPCSimConfig.h"
#include "runIPCSimOutput.h"
#include "runIPCSimSetup.h"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
```

If the exact header names differ in this repo, use `rg --files src/core | rg 'linearPotentialEnergy|potentialEnergies|NewtonSolver'` to select the existing casing.

- [ ] **Step 3: 实现 pulling final target helper**

In `runIPCSimStaticSolve.cpp`:

```cpp
namespace pgo::RunIPCSim
{
namespace ES = pgo::EigenSupport;

namespace
{
void setStaticPullingTargets(IpcSimulationContext &context)
{
  for (std::size_t pi = 0; pi < context.pullingEnergies.size(); ++pi) {
    context.pullingEnergies[pi]->setTargetPos(context.pullingTargets[pi].data());
    std::cout << "Static attachment " << pi << " target: "
              << context.pullingTargets[pi].transpose().head(3) << std::endl;
  }
}
}  // namespace
```

- [ ] **Step 4: 实现 static solve 主体**

Append:

```cpp
void runIPCSimStaticSolve(
  const RunIPCSimRuntimeConfig &runtimeConfig,
  IpcSimulationContext &context,
  const RunIPCSimOutput &output)
{
  const int n3 = static_cast<int>(context.simulationRestPosition.size());
  if (n3 <= 0)
    throw std::runtime_error("runIPCSim static solve received an empty simulation state.");

  setStaticPullingTargets(context);

  ES::VXd staticForce = ES::VXd::Zero(n3);
  ES::mv(context.M, ES::VXd::NullaryExpr(n3, [&](int i) {
    return runtimeConfig.gravity[i % 3];
  }), staticForce);

  if (context.surfacePressureForceEnabled)
    staticForce.noalias() += context.surfacePressureSimulationForce;

  auto externalForcesEnergy =
    std::make_shared<PredefinedPotentialEnergies::LinearPotentialEnergy>(staticForce);

  auto energyAll = std::make_shared<NonlinearOptimization::PotentialEnergies>(n3);
  energyAll->addPotentialEnergy(context.elasticEnergy, 1.0);
  for (const auto &pullingEnergy : context.pullingEnergies)
    energyAll->addPotentialEnergy(pullingEnergy, 1.0);
  energyAll->addPotentialEnergy(externalForcesEnergy, -1.0);
  context.contactBackend->addStaticEnergies(runtimeConfig, context, *energyAll);
  energyAll->init();

  NonlinearOptimization::NewtonSolver::SolverParam solverParam;
  ES::VXd u = ES::VXd::Zero(n3);
  energyAll->printEnergy(u);

  NonlinearOptimization::NewtonSolver solver(
    u.data(), solverParam, energyAll, std::vector<int>(), nullptr);
  solver.solve(u.data(), runtimeConfig.solverMaxIter, runtimeConfig.solverEps, 2);

  ES::VXd uvel = ES::VXd::Zero(n3);
  ES::VXd uacc = ES::VXd::Zero(n3);
  output.writeStateAndSurfaceFrame(
    0, 0, context, u, uvel, uacc, runtimeConfig.scale, true, true);

  if (runtimeConfig.outputVonMises)
    output.writeVonMisesStressJson(0, runtimeConfig.timestep, context, u);
}
}  // namespace pgo::RunIPCSim
```

Implementation note:

- If the `ES::VXd::NullaryExpr` lambda fails due to Eigen alias/type issues, replace it with an explicit loop:

```cpp
ES::VXd g(n3);
for (int vi = 0; vi < n3 / 3; ++vi)
  g.segment<3>(vi * 3) = runtimeConfig.gravity;
ES::mv(context.M, g, staticForce);
```

Prefer the explicit loop for readability if editing manually.

- [ ] **Step 5: CMake 加入新 source**

In `src/tools/runSim/CMakeLists.txt`, add:

```cmake
  runIPCSimStaticSolve.cpp
```

inside `RUN_IPC_SIM_CORE_SOURCES`.

- [ ] **Step 6: 编译 static solver**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSimCore
```

Expected:

- PASS.

---

## Task 6: App 层分发 dynamic/static

**Files:**

- Modify: `src/tools/runSim/runIPCSimApp.cpp`

- [ ] **Step 1: include static solver**

Add:

```cpp
#include "runIPCSimStaticSolve.h"
```

- [ ] **Step 2: 在 `runFromConfig()` 里分发 static**

在 `restoreRestartStateIfRequested()` 之前改造当前流程。

Current dynamic path:

```cpp
RunIPCSimSession session = createRunIPCSimSession(runtimeConfig, context);
restoreRestartStateIfRequested(runtimeConfig, output, session);
context.contactBackend->initializeAfterRestart(runtimeConfig, context, session);
runIPCSimLoop(runtimeConfig, context, session, output);
```

Replace with:

```cpp
if (runtimeConfig.simulationMode == RunIPCSimSimulationMode::Static) {
  if (runtimeConfig.restartFromU)
    throw std::invalid_argument("runIPCSim static mode does not support `restart-from-u`.");
  runIPCSimStaticSolve(runtimeConfig, context, output);
}
else {
  RunIPCSimSession session = createRunIPCSimSession(runtimeConfig, context);
  restoreRestartStateIfRequested(runtimeConfig, output, session);
  context.contactBackend->initializeAfterRestart(runtimeConfig, context, session);
  runIPCSimLoop(runtimeConfig, context, session, output);
}
```

- [ ] **Step 3: Run static smoke tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimStaticGTest.*'
```

Expected:

- `StaticTetIpcWritesUnifiedSurfaceAndState` PASS.
- `StaticLegacyTetWritesUnifiedSurfaceAndState` PASS.
- `StaticRejectsRestartFromU` PASS.

---

## Task 7: 增强 static coverage

**Files:**

- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: 增加 static IPC floor 测试**

新增测试覆盖 `addStaticEnergies()` 中 floor final height 逻辑：

```cpp
TEST(RunIPCSimStaticGTest, StaticCubicIpcWithFloorWritesSurfaceAndState)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "cubic-static-floor.json";
  const fs::path outputDir = tempDir.path() / "cubic-static-floor-output";
  std::string configText = makeStaticConfigFromVolumeConfig(
    makeVolumeConfig(cubicIPCExampleDir(), outputDir, "cubic-mesh", "box.veg", 1, 1));
  const std::string ipcField = "\"ipc-kappa\": 3000.0,";
  const std::size_t insertPos = configText.find(ipcField);
  ASSERT_NE(insertPos, std::string::npos);
  configText.insert(insertPos + ipcField.size(), R"(
  "floors": [
    { "axis": "y", "side": "lower", "height": -0.15, "kappa": 4000.0 }
  ],)");
  writeTextFile(configPath, configText);

  ASSERT_EQ(pgo::RunIPCSim::runFromConfig(configPath, {}), 0);
  EXPECT_TRUE(fs::exists(outputDir / "surface" / "ret0000.obj"));
  EXPECT_TRUE(fs::exists(outputDir / "states" / "deform0000.u"));
}
```

If string insertion is too brittle with current helpers, construct this test config using `nlohmann::json` and file-local path helpers.

- [ ] **Step 2: 增加 static von Mises 测试**

新增测试：

```cpp
TEST(RunIPCSimStaticGTest, StaticVolumeWritesVonMisesWhenRequested)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "tet-static-stress.json";
  const fs::path outputDir = tempDir.path() / "tet-static-stress-output";
  std::string configText = makeStaticConfigFromVolumeConfig(
    makeVolumeConfig(tetIPCExampleDir(), outputDir, "tet-mesh", "box.veg", 1, 1));
  const std::string outputField = "\"output\": ";
  const std::size_t insertPos = configText.rfind(outputField);
  ASSERT_NE(insertPos, std::string::npos);
  configText.insert(insertPos, "  \"output-von-mises\": true,\n");
  writeTextFile(configPath, configText);

  ASSERT_EQ(pgo::RunIPCSim::runFromConfig(configPath, {}), 0);
  EXPECT_TRUE(fs::exists(outputDir / "stress" / "von_mises0000.json"));
}
```

- [ ] **Step 3: 增加 static shell IPC smoke**

新增测试：

```cpp
TEST(RunIPCSimStaticGTest, StaticShellIpcWritesUnifiedSurfaceAndState)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "shell-static-ipc.json";
  const fs::path outputDir = tempDir.path() / "shell-static-output";
  writeTextFile(configPath, makeStaticConfigFromShellConfig(
    makeShellConfig(fs::path(kShellExampleDir), outputDir, 1, 1)));

  ASSERT_EQ(pgo::RunIPCSim::runFromConfig(configPath, {}), 0);
  EXPECT_TRUE(fs::exists(outputDir / "surface" / "ret0000.obj"));
  EXPECT_TRUE(fs::exists(outputDir / "states" / "deform0000.u"));
}
```

If no `makeStaticConfigFromShellConfig()` exists, reuse `makeStaticConfigFromVolumeConfig()` only after renaming it to `makeStaticConfig()` so it is not volume-specific.

- [ ] **Step 4: 增加 legacy shell static rejection test**

新增测试：

```cpp
TEST(RunIPCSimStaticGTest, StaticLegacyShellIsRejected)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "shell-static-legacy.json";
  const fs::path outputDir = tempDir.path() / "shell-static-legacy-output";
  writeTextFile(configPath, makeStaticConfig(
    makeShellConfig(fs::path(kShellExampleDir), outputDir, 1, 1)));

  pgo::RunIPCSim::RunIPCSimOptions options;
  options.contactBackendKind = pgo::RunIPCSim::ContactBackendKind::LegacyPenalty;
  EXPECT_NE(pgo::RunIPCSim::runFromConfig(configPath, options), 0);
}
```

- [ ] **Step 5: Run expanded static test set**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimStaticGTest.*'
```

Expected:

- All static tests PASS.

---

## Task 8: 文档更新

**Files:**

- Modify: `README.md`
- Modify: `examples/ipc/README.md`

- [ ] **Step 1: README 说明 static mode**

在 `runIPCSim` usage 附近加入：

```markdown
`runIPCSim` supports both dynamic and static solves through the config field
`"sim-type"`. Use `"dynamic"` for time stepping and `"static"` for a single
Newton solve written to the unified output layout.
```

中文 README 风格如当前段落为英文，则保持英文；如果附近已是中文说明，则用中文。

- [ ] **Step 2: README 说明 legacy static**

在 `--legacy` 段落补充：

```markdown
With `"sim-type": "static"`, `--legacy` preserves the old `runSim` static
volume behavior: elastic energy, fixed-vertex pulling, and gravity are solved
once; legacy penalty contact is only active for dynamic legacy runs.
```

- [ ] **Step 3: examples/ipc README 增加一句**

在 examples overview 里说明 static configs can be created by switching `sim-type` to `static` for supported IPC cases. 不需要新增大体量 examples。

- [ ] **Step 4: 文档引用 hygiene**

Run:

```bash
rg -n "runShellSim|examples/legacy|sim-type = dynamic only|only supports `sim-type = dynamic`" README.md examples src tests
```

Expected:

- No stale docs saying static is unsupported.
- No `examples/legacy` / `runShellSim` docs references.

---

## Task 9: Full validation

**Files:** no source edits unless validation exposes a bug.

- [ ] **Step 1: Build changed targets**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim runIPCSim_gtest runSimShared_gtest
```

Expected:

- PASS.

- [ ] **Step 2: Run focused tests**

Run:

```bash
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runSimShared_gtest
```

Expected:

- PASS.
- `runIPCSim_gtest` includes dynamic IPC, dynamic legacy, static IPC, static legacy.
- `runSimShared_gtest` remains unchanged except shared helper compatibility.

- [ ] **Step 3: Verify old targets still removed**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runSim
cmake --build --preset base_no_mkl_release --target runShellSim
```

Expected:

- Both fail with `No rule to make target`.

- [ ] **Step 4: Search hygiene**

Run:

```bash
rg -n "runShellSim|PGO_TEST_RUN_SHELL_SIM_BIN|examples/legacy|legacy/shell" README.md src tests examples
rg -n "only supports `sim-type = dynamic`|phase2 only supports" src tests README.md examples
```

Expected:

- First command: no matches.
- Second command: no stale static rejection messages.

- [ ] **Step 5: Diff hygiene**

Run:

```bash
git diff --check
git status --short
```

Expected:

- `git diff --check` PASS.
- `git status --short` shows only static-mode source/test/doc changes since the legacy migration commit.

- [ ] **Step 6: Commit**

Run:

```bash
git add README.md examples/ipc/README.md src/tools/runSim tests/src/tools/runIPCSim_gtest.cpp
git commit -m "feat: support static mode in runIPCSim"
```

Expected:

- commit succeeds.

---

## Done Criteria

- `runIPCSim` accepts `sim-type = dynamic` and `sim-type = static`.
- Default IPC static works for volume and shell smoke tests.
- IPC static includes IPC collision energy and floor energy.
- `--legacy` static works for volume legacy JSON and writes unified output.
- `--legacy` shell static is rejected with the same shell legacy policy as dynamic.
- legacy dynamic penalty contact behavior remains green.
- static mode rejects `restart-from-u`.
- `runSim` / `runShellSim` executable targets remain removed.
- Docs no longer imply `runIPCSim` only supports dynamic mode.
- `git diff --check` passes.

## Rollback Checkpoints

- After Task 2, rollback only touches runtime config parsing.
- After Task 3, dynamic output regression tests must pass before adding static solver.
- After Task 4, backend API changes must compile before app dispatch is modified.
- After Task 6, static smoke tests should pass; if not, keep dynamic loop unchanged and debug static solver only.
- After Task 7, if one expanded static test fails, keep the minimal Task 6 static smoke passing and narrow the failure by backend/mode.

---

## Self Review

### Coverage Check

- User requirement “static mode IPC 也得支持”: covered by Tasks 1, 4, 5, 6, 7.
- User requirement “legacy 也得保留”: covered by Tasks 1, 5, 6, 7; legacy dynamic remains under Task 9 full `runIPCSim_gtest`.
- Static dynamic split: covered by Task 2 and Task 6.
- Old `runSim static` behavior: captured in behavior decision 3 and implemented by legacy backend no-op static hook.
- Unified output layout: covered by Task 3 and Task 5.
- Removed old executables stay removed: covered by Task 9 Step 3.

### Placeholder Scan

- No placeholder markers are present.
- Code snippets name concrete files, functions, and tests.
- Where helper names may differ in current test file, plan instructs implementer to reuse exact existing helper names instead of inventing new behavior.

### Type Consistency Check

- `RunIPCSimSimulationMode` is introduced before use.
- `addStaticEnergies()` signature is added to the backend interface before IPC/legacy implementations.
- `runIPCSimStaticSolve()` depends on `writeStateAndSurfaceFrame()`, which is added in Task 3 before Task 5.
- `runFromConfig()` dispatch references `RunIPCSimSimulationMode::Static`, introduced in Task 2.

### Risk Review

- Main risk: IPC static energy assembly may reveal that `EmbeddedSurfaceIPCPotentialEnergy` expects dynamic line-search state. Mitigation: static tests include default IPC and floor IPC; if needed, add a backend helper to clear/cache active sets before adding the energy.
- Main behavior risk: static obstacle time uses final frame time. This is explicitly chosen and tested indirectly through IPC static smoke; future tests can add a moving obstacle case if needed.
- Legacy static contact is intentionally no-op. This matches old `runSim static`; if product intent changes to “static penalty contact too”, that should be a follow-up plan because it changes physics behavior.
