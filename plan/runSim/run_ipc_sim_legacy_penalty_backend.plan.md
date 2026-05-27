# Run IPC Sim Legacy Penalty Backend Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把旧 `runSim` 的 volume penalty contact 模式迁入新 `runIPCSim --legacy` 入口，删除 `runSim` / `runShellSim` executable 和 shell legacy examples，同时保留 IPC 主路径的当前行为。

**Architecture:** `runIPCSim` 继续作为唯一仿真 CLI。CLI 层用 `--legacy` 选择 `ContactBackendKind::LegacyPenalty`，默认仍为 `ContactBackendKind::Ipc`；app/setup/session/output 继续复用刚拆出的模块，frame loop 通过 contact backend hook 调用 IPC 或 penalty 接触逻辑。旧 shell legacy 不迁移，遇到 shell-style legacy config 时明确报错。

**Tech Stack:** C++20, CMake, argparse, nlohmann/json, Eigen, TBB, GoogleTest, existing `contact`, `simulation`, `solidDeformationModel`, `runIPCSimCore`.

---

## 当前结论

- `runSim.cpp` 的核心差异是 volume simulation 使用 penalty-based contact：
  - external contact: `TriangleMeshExternalContactHandler` + `PointPenetrationEnergy`
  - self contact: `TriangleMeshSelfContactHandler` + `PointTrianglePairCouplingEnergyWithCollision`
- `runShellSim.cpp` 不是纯 legacy penalty 路径，它还直接添加 `CIPCPotentialEnergy`，并且当前 `runSim_gtest` 中 `RunShellSimCliLoggingGTest.DeformStateIsWrittenEveryTimestep` 稳定失败：
  - `CIPCPotentialEnergy::hessian() should not be called directly. Use hessianDirect() instead.`
- 因此本计划迁移 **volume legacy only**，直接删除 shell legacy executable 和 shell legacy examples。
- `runSimVolumeMeshIO.*`、`runSimFEMSetup.*`、`runSimCliLogging.*` 已经被 `runIPCSimCore` 复用，不在本轮删除；它们只是名字还带 `runSim`，后续可单独做命名整理。

## 行为定义

### CLI

```bash
build/base_no_mkl/bin/runIPCSim scene.json
```

默认 IPC backend，保持当前要求：

- shell IPC config 走 `buildShellIpcSimulation()`
- volume IPC config 走 `buildVolumeIpcSimulation()`
- volume IPC 仍要求 `ipc-dhat` 和 `ipc-kappa`
- 输出仍为：
  - `output/states/deform%04d.u`
  - `output/surface/ret%04d.obj`
  - `output/stress/von_mises%04d.json`
  - `output/runIPCSim.log`

```bash
build/base_no_mkl/bin/runIPCSim --legacy scene.json
```

legacy penalty backend，只接受 volume config：

- 必须有且只能有一个 `tet-mesh` 或 `cubic-mesh`
- 必须有 `surface-mesh`
- 必须有 `elastic-material = stable-neo` 或 `stvk-vol`
- 接受旧 contact 字段：
  - `contact-stiffness`
  - `contact-sample`
  - `contact-samples`
  - `contact-friction-coeff`
  - `contact-vel-eps`
  - `external-objects`
- `contact-sample` 和 `contact-samples` 都支持；两者同时存在时报错，避免静默选择。
- 旧 shell config，即没有 `tet-mesh` / `cubic-mesh` 且有 `surface-mesh` / `elastic-material = koiter-stvk`，在 `--legacy` 下返回错误：

```text
runIPCSim --legacy only supports volume legacy configs with `tet-mesh` or `cubic-mesh`; shell legacy has been removed.
```

### 输出布局

`--legacy` 接受旧 JSON schema，但不保留旧 `runSim` 的 flat output layout。它使用统一 `runIPCSim` output layout：

- state: `output/states/deform0000.u`
- surface: `output/surface/ret0000.obj`
- log: `output/runIPCSim.log`

这是有意的：`--legacy` 表示旧 contact model，不表示旧 runner/output ABI。

### 非目标

- 不迁移 `runShellSim` 行为。
- 不维护 shell legacy example。
- 不改变默认 IPC backend 的 config schema、输出路径、日志摘要和 floor/surface-pressure 行为。
- 不在本轮重命名 `runSimVolumeMeshIO.*`、`runSimFEMSetup.*`、`runSimCliLogging.*`。
- 不在本轮删除或重写 `src/c/pgo_c.cpp::pgo_run_sim_from_config()` 和 `pypgo.run_sim_from_config()`；这是一条独立 C/Python API 路径，后续可以单独退役或改接 `runIPCSim --legacy`。

## 目标文件布局

### 新增

- `src/tools/runSim/runIPCSimContactBackend.h`
  - 定义 `ContactBackendKind` 和 `RunIPCSimContactBackend` interface。
- `src/tools/runSim/runIPCSimIpcContactBackend.cpp`
  - 把当前 IPC collision/floor/surface-pressure per-frame 行为移入 backend。
- `src/tools/runSim/runIPCSimLegacyPenaltyContact.h/.cpp`
  - 解析 legacy contact config，加载 external objects，维护 penalty contact handlers、active energy buffers、kinematic obstacle updates。
- `tests/fixtures/legacy/tet/box/*`
  - 迁移 tests 仍需要的 tet volume fixture。
- `tests/fixtures/legacy/tet/bottom.obj`
  - 保留 tet legacy `box.json` 中 `external-objects[].filename = ../bottom.obj` 的路径有效性。
- `tests/fixtures/legacy/cubic/box/*`
  - 迁移 tests 仍需要的 cubic volume fixture。
- `tests/fixtures/legacy/cubic/bottom.obj`
  - 保留 cubic legacy `box.json` 中 `external-objects[].filename = ../bottom.obj` 的路径有效性。
- `tests/fixtures/legacy/tet/torus.veg`
  - 迁移 C/Python/core tests 仍需要的 tet fixture。
- `tests/fixtures/shell/shell.obj`
  - 迁移 core shell mesh tests 仍需要的 shell mesh fixture。
- `tests/fixtures/shell/shell-fixed.txt`
  - 迁移 `runIPCSim_gtest` shell IPC tests 仍需要的 fixed-vertices fixture。
- `tests/fixtures/config/shell_paths.json`
  - 替代 `examples/legacy/shell/shell.json`，只用于 `ConfigFileJSON` path resolution test。
- `tests/src/tools/runSimShared_gtest.cpp`
  - 从 `runSim_gtest.cpp` 拆出仍有价值的 shared helper tests，删除 `runShellSim` executable tests。

### 修改

- `src/tools/runSim/runIPCSimApp.h/.cpp`
  - `RunIPCSimOptions` 增加 `ContactBackendKind contactBackendKind`。
  - `buildIpcSimulation()` 改为 `buildRunIPCSimSimulation(config, options)` 或等价名称。
- `src/tools/runSim/runIPCSimCli.h/.cpp`
  - 增加 `--legacy`。
- `src/tools/runSim/runIPCSimSetup.h/.cpp`
  - 拆出 reusable volume base context。
  - IPC path 创建 IPC backend。
  - legacy path 创建 penalty backend。
- `src/tools/runSim/runIPCSimLoop.cpp`
  - 通过 `RunIPCSimContactBackend` hook 调用 backend。
- `src/tools/runSim/runIPCSimSession.cpp`
  - 初始 `session.usurf` 需要在 restore 后可由 backend 初始化。
- `src/tools/runSim/runIPCSimOutput.*`
  - 不改变路径，只增加必要测试覆盖。
- `src/tools/runSim/CMakeLists.txt`
  - `runIPCSimCore` 加入新增 backend sources。
  - 删除 `runSim` 和 `runShellSim` executable targets。
- `tests/src/tools/CMakeLists.txt`
  - 删除 `runShellSim` dependency。
  - `runSim_gtest` 改名为 `runSimShared_gtest`。
  - 更新 fixture compile definitions。
- `tests/src/tools/runIPCSim_gtest.cpp`
  - 增加 `--legacy` CLI 和 runner smoke tests。
  - 将 shell IPC fixture 路径从 `examples/legacy/shell` 改为 `tests/fixtures/shell`。
- `tests/src/core/CMakeLists.txt`
- `tests/src/core/solidDeformationModel/CMakeLists.txt`
- `tests/CMakeLists.txt`
- `src/python/pypgo/pgo_test_01.py`
- `tests/pypgo/test_pgo_smoke.py`
  - 更新 fixture 路径。
- `README.md`
  - 删除 `runSim` / `runShellSim` legacy docs。
  - 增加 `runIPCSim --legacy` volume penalty backend 说明。

### 删除

- `src/tools/runSim/runSim.cpp`
- `src/tools/runSim/runShellSim.cpp`
- `tests/src/tools/runSim_gtest.cpp`，由 `runSimShared_gtest.cpp` 替代。
- `examples/legacy/shell/`
- `examples/legacy/README.md`
- `examples/legacy/cubic/README.md`
- `examples/legacy/cubic/media/`
- 面向用户的 legacy volume example cases，在 fixtures 迁移完成后删除 `examples/legacy/`。

## Backend Interface 设计

新增 `src/tools/runSim/runIPCSimContactBackend.h`：

```cpp
#pragma once

#include <memory>
#include <string>

namespace pgo::Simulation
{
class ImplicitBackwardEulerTimeIntegrator;
}

namespace pgo::RunIPCSim
{
struct IpcSimulationContext;
struct RunIPCSimRuntimeConfig;
struct RunIPCSimSession;

enum class ContactBackendKind
{
  Ipc,
  LegacyPenalty,
};

class RunIPCSimContactBackend
{
public:
  virtual ~RunIPCSimContactBackend() = default;

  virtual ContactBackendKind kind() const = 0;
  virtual std::string description() const = 0;

  virtual void initializeAfterRestart(const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) = 0;

  virtual void beginFrame(int frame, const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) = 0;

  virtual void addForces(int frame, const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) = 0;

  virtual void afterStep(int frame, const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) = 0;

  virtual void logSummary(const IpcSimulationContext &context,
    const RunIPCSimSession &session) const = 0;
};
}  // namespace pgo::RunIPCSim
```

`IpcSimulationContext` 增加：

```cpp
std::shared_ptr<RunIPCSimContactBackend> contactBackend;
```

`runIPCSimLoop()` 的核心顺序变为：

```cpp
session.integrator->clearGeneralImplicitForceModel();
updatePullingTargets(framei, runtimeConfig, context);
context.contactBackend->beginFrame(framei, runtimeConfig, context, session);
context.contactBackend->addForces(framei, runtimeConfig, context, session);
applySurfacePressureForceIfNeeded(framei, runtimeConfig, context, session);
session.integrator->setqState(session.u, session.uvel, session.uacc);
session.integrator->doTimestep(1, 3, 1);
session.integrator->getq(session.u);
session.integrator->getqvel(session.uvel);
session.integrator->getqacc(session.uacc);
context.contactBackend->afterStep(framei, runtimeConfig, context, session);
context.contactBackend->logSummary(context, session);
writeOutputs(...);
```

IPC backend 的 `addForces()` 负责添加当前 `collisionHandler` 和 floor force models；legacy backend 的 `addForces()` 负责按当前 `usurf` 执行 external/self contact detection，并把临时 penalty energies 添加到 integrator。

## Task 1: 添加 failing tests，锁定 `--legacy` CLI 和 volume-only 行为

**Files:**
- Modify: `tests/src/tools/CMakeLists.txt`
- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: 给 `runIPCSim_gtest` 暂时暴露 legacy volume fixture dirs**

在 `tests/src/tools/CMakeLists.txt` 的 `runIPCSim_gtest` compile definitions 中加入：

```cmake
  LIBPGO_TEST_LEGACY_TET_BOX_DIR="${CMAKE_SOURCE_DIR}/examples/legacy/tet/box"
  LIBPGO_TEST_LEGACY_CUBIC_BOX_DIR="${CMAKE_SOURCE_DIR}/examples/legacy/cubic/box"
```

Task 6 会把这两个路径切到 `tests/fixtures/legacy/...`。

- [ ] **Step 2: 添加 CLI parse tests**

在现有 CLI tests 附近添加：

```cpp
TEST(RunIPCSimCliGTest, LegacyFlagSelectsLegacyPenaltyBackend)
{
  const char *argv[] = { "runIPCSim", "--legacy", "scene.json" };
  const auto options = pgo::RunIPCSim::parseRunIPCSimCli(3, const_cast<char **>(argv));
  EXPECT_EQ(options.configPath, fs::path("scene.json"));
  EXPECT_EQ(options.runOptions.contactBackendKind, pgo::RunIPCSim::ContactBackendKind::LegacyPenalty);
}

TEST(RunIPCSimCliGTest, DefaultBackendIsIpc)
{
  const char *argv[] = { "runIPCSim", "scene.json" };
  const auto options = pgo::RunIPCSim::parseRunIPCSimCli(2, const_cast<char **>(argv));
  EXPECT_EQ(options.runOptions.contactBackendKind, pgo::RunIPCSim::ContactBackendKind::Ipc);
}
```

- [ ] **Step 3: 添加 legacy volume smoke config helper**

在 test helper 区域添加：

```cpp
std::string makeLegacyVolumeConfig(const fs::path &volumeMesh, const fs::path &surfaceMesh,
  const fs::path &outputDir, const char *meshKey, const char *material, int numTimesteps)
{
  std::ostringstream json;
  json << "{\n"
       << "  \"" << meshKey << "\": " << quotePath(volumeMesh) << ",\n"
       << "  \"surface-mesh\": " << quotePath(surfaceMesh) << ",\n"
       << "  \"fixed-vertices\": [],\n"
       << "  \"g\": [0, -9.81, 0],\n"
       << "  \"init-vel\": [0, 0, 0],\n"
       << "  \"init-disp\": [0, 0, 0],\n"
       << "  \"scale\": 1.0,\n"
       << "  \"timestep\": 0.001,\n"
       << "  \"num-timestep\": " << numTimesteps << ",\n"
       << "  \"damping-params\": [0, 0],\n"
       << "  \"sim-type\": \"dynamic\",\n"
       << "  \"contact-stiffness\": 1000,\n"
       << "  \"contact-sample\": 2,\n"
       << "  \"contact-friction-coeff\": 0.0,\n"
       << "  \"contact-vel-eps\": 1e-5,\n"
       << "  \"solver-eps\": 1e-4,\n"
       << "  \"solver-max-iter\": 5,\n"
       << "  \"elastic-material\": \"" << material << "\",\n"
       << "  \"dump-interval\": 1,\n"
       << "  \"output\": " << quotePath(outputDir) << "\n"
       << "}\n";
  return json.str();
}
```

- [ ] **Step 4: 添加 runner smoke tests**

```cpp
TEST(RunIPCSimLegacyGTest, LegacyTetConfigRunsOneStepAndWritesUnifiedOutput)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "legacy-tet.json";
  const fs::path outputDir = tempDir.path() / "legacy-tet-output";
  writeTextFile(configPath, makeLegacyVolumeConfig(
    fs::path(LIBPGO_TEST_LEGACY_TET_BOX_DIR) / "box.veg",
    fs::path(LIBPGO_TEST_LEGACY_TET_BOX_DIR) / "box.obj",
    outputDir, "tet-mesh", "stable-neo", 1));

  pgo::RunIPCSim::RunIPCSimOptions options;
  options.contactBackendKind = pgo::RunIPCSim::ContactBackendKind::LegacyPenalty;
  EXPECT_EQ(pgo::RunIPCSim::runFromConfig(configPath, options), 0);
  EXPECT_TRUE(fs::exists(outputDir / "states" / "deform0000.u"));
  EXPECT_TRUE(fs::exists(outputDir / "surface" / "ret0000.obj"));
}

TEST(RunIPCSimLegacyGTest, LegacyCubicConfigRunsOneStepAndWritesUnifiedOutput)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "legacy-cubic.json";
  const fs::path outputDir = tempDir.path() / "legacy-cubic-output";
  writeTextFile(configPath, makeLegacyVolumeConfig(
    fs::path(LIBPGO_TEST_LEGACY_CUBIC_BOX_DIR) / "box.veg",
    fs::path(LIBPGO_TEST_LEGACY_CUBIC_BOX_DIR) / "box.obj",
    outputDir, "cubic-mesh", "stable-neo", 1));

  pgo::RunIPCSim::RunIPCSimOptions options;
  options.contactBackendKind = pgo::RunIPCSim::ContactBackendKind::LegacyPenalty;
  EXPECT_EQ(pgo::RunIPCSim::runFromConfig(configPath, options), 0);
  EXPECT_TRUE(fs::exists(outputDir / "states" / "deform0000.u"));
  EXPECT_TRUE(fs::exists(outputDir / "surface" / "ret0000.obj"));
}
```

- [ ] **Step 5: 添加 shell legacy rejection test**

```cpp
TEST(RunIPCSimLegacyGTest, LegacyShellConfigIsRejected)
{
  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "legacy-shell.json";
  writeTextFile(configPath, makeShellIPCConfig(tempDir.path(), 0));

  pgo::RunIPCSim::RunIPCSimOptions options;
  options.contactBackendKind = pgo::RunIPCSim::ContactBackendKind::LegacyPenalty;
  EXPECT_EQ(pgo::RunIPCSim::runFromConfig(configPath, options), 1);
}
```

- [ ] **Step 6: 运行 tests，确认失败原因是缺少新 API**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
```

Expected:

- FAIL
- 编译错误包含 `ContactBackendKind` 或 `contactBackendKind` 未定义。

## Task 2: 接入 CLI option 和 app-level backend selection

**Files:**
- Create: `src/tools/runSim/runIPCSimContactBackend.h`
- Modify: `src/tools/runSim/runIPCSimApp.h`
- Modify: `src/tools/runSim/runIPCSimApp.cpp`
- Modify: `src/tools/runSim/runIPCSimCli.cpp`
- Modify: `src/tools/runSim/CMakeLists.txt`

- [ ] **Step 1: 新增 backend enum 和 interface header**

创建 `runIPCSimContactBackend.h`，内容使用本计划 “Backend Interface 设计” 中的完整 interface。

- [ ] **Step 2: 修改 `RunIPCSimOptions`**

在 `runIPCSimApp.h` 中加入 include：

```cpp
#include "runIPCSimContactBackend.h"
```

并改为：

```cpp
struct RunIPCSimOptions
{
  bool enableCliLog = false;
  ContactBackendKind contactBackendKind = ContactBackendKind::Ipc;
};
```

- [ ] **Step 3: 修改 CLI parser**

在 `configureRunIPCSimArgumentParser()` 中加入：

```cpp
program.add_argument("--legacy")
  .help("Run volume legacy penalty-contact configs through the runIPCSim entry")
  .default_value(false)
  .implicit_value(true);
```

在 `readRunIPCSimCliOptions()` 中加入：

```cpp
options.runOptions.contactBackendKind = program.get<bool>("--legacy")
  ? ContactBackendKind::LegacyPenalty
  : ContactBackendKind::Ipc;
```

- [ ] **Step 4: 修改 app context selection**

把 `buildIpcSimulation()` 改成：

```cpp
IpcSimulationContext buildRunIPCSimSimulation(const pgo::ConfigFileJSON &config,
  const RunIPCSimOptions &options)
{
  if (options.contactBackendKind == ContactBackendKind::LegacyPenalty)
    return buildVolumeLegacyPenaltySimulation(config);

  const bool hasTetMesh = config.exist("tet-mesh");
  const bool hasCubicMesh = config.exist("cubic-mesh");
  const bool useVolumePath = hasTetMesh || hasCubicMesh;
  return useVolumePath ? buildVolumeIpcSimulation(config) : buildShellIpcSimulation(config);
}
```

在 `runFromConfig()` 中调用：

```cpp
IpcSimulationContext context = buildRunIPCSimSimulation(config, options);
```

`buildVolumeLegacyPenaltySimulation()` 先只在 header 声明，Task 4 实现。

- [ ] **Step 5: 运行 CLI parse tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
```

Expected:

- 编译继续失败在 `buildVolumeLegacyPenaltySimulation` 未声明或未定义。
- CLI parse test 相关的类型错误消失。

## Task 3: 把 IPC 接触逻辑包成 backend，不改变默认行为

**Files:**
- Create: `src/tools/runSim/runIPCSimIpcContactBackend.cpp`
- Modify: `src/tools/runSim/runIPCSimSetup.h`
- Modify: `src/tools/runSim/runIPCSimSetup.cpp`
- Modify: `src/tools/runSim/runIPCSimLoop.cpp`
- Modify: `src/tools/runSim/CMakeLists.txt`

- [ ] **Step 1: 在 `IpcSimulationContext` 中增加 backend owner**

在 `runIPCSimSetup.h` include：

```cpp
#include "runIPCSimContactBackend.h"
```

增加字段：

```cpp
std::shared_ptr<RunIPCSimContactBackend> contactBackend;
```

保留现有 IPC contact fields：

```cpp
std::shared_ptr<Contact::CIPC::EmbeddedSurfaceIPCPotentialEnergy> collisionHandler;
std::vector<std::shared_ptr<NonlinearOptimization::PotentialEnergy>> extraGeneralImplicitForceModels;
std::vector<std::shared_ptr<Contact::CIPC::EmbeddedSurfaceFloorPotentialEnergy>> floorPotentialEnergies;
std::vector<IpcFloorMotionState> floorMotionStates;
```

本 task 只做 adapter，不迁移字段所有权，降低行为风险。

- [ ] **Step 2: 实现 IPC backend adapter**

`runIPCSimIpcContactBackend.cpp`：

```cpp
#include "runIPCSimContactBackend.h"

#include "embeddedSurfaceFloorPotentialEnergy.h"
#include "embeddedSurfaceIPCPotentialEnergy.h"
#include "implicitBackwardEulerTimeIntegrator.h"
#include "runIPCSimLogging.h"
#include "runIPCSimSession.h"
#include "runIPCSimSetup.h"

namespace pgo::RunIPCSim
{
class IpcContactBackend final : public RunIPCSimContactBackend
{
public:
  ContactBackendKind kind() const override { return ContactBackendKind::Ipc; }
  std::string description() const override { return "ipc"; }

  void initializeAfterRestart(const RunIPCSimRuntimeConfig &, IpcSimulationContext &, RunIPCSimSession &) override {}

  void beginFrame(int frame, const RunIPCSimRuntimeConfig &, IpcSimulationContext &context, RunIPCSimSession &) override
  {
    for (std::size_t fi = 0; fi < context.floorPotentialEnergies.size(); ++fi)
      context.floorPotentialEnergies[fi]->setFloorHeight(floorHeightAtFrame(context.floorMotionStates[fi], frame));
  }

  void addForces(int frame, const RunIPCSimRuntimeConfig &runtimeConfig,
    IpcSimulationContext &context, RunIPCSimSession &session) override
  {
    const double tCurr = static_cast<double>(frame) * runtimeConfig.timestep;
    context.collisionHandler->setObstacleTime(tCurr + runtimeConfig.timestep);
    session.integrator->addGeneralImplicitForceModel(context.collisionHandler, 0, 0);
    for (const auto &forceModel : context.extraGeneralImplicitForceModels)
      session.integrator->addGeneralImplicitForceModel(forceModel, 0, 0);
  }

  void afterStep(int, const RunIPCSimRuntimeConfig &, IpcSimulationContext &, RunIPCSimSession &) override {}

  void logSummary(const IpcSimulationContext &context, const RunIPCSimSession &session) const override
  {
    logRunIPCSimMaxStepSummary(context.elasticEnergy, context.collisionHandler, session.integrator);
  }
};

std::shared_ptr<RunIPCSimContactBackend> makeIpcContactBackend()
{
  return std::make_shared<IpcContactBackend>();
}
}  // namespace pgo::RunIPCSim
```

在 `runIPCSimSetup.h` 声明：

```cpp
std::shared_ptr<RunIPCSimContactBackend> makeIpcContactBackend();
```

- [ ] **Step 3: setup 中挂接 IPC backend**

在 `buildShellIpcSimulation()` 和 `buildVolumeIpcSimulation()` 返回前加入：

```cpp
context.contactBackend = makeIpcContactBackend();
```

- [ ] **Step 4: loop 改为 backend hook**

替换 `runIPCSimLoop.cpp` 中直接添加 IPC handler/floor 的代码：

```cpp
context.contactBackend->beginFrame(framei, runtimeConfig, context, session);
context.contactBackend->addForces(framei, runtimeConfig, context, session);
```

在 `doTimestep()` 后替换 summary：

```cpp
context.contactBackend->afterStep(framei, runtimeConfig, context, session);
context.contactBackend->logSummary(context, session);
```

在 no-step 分支替换为：

```cpp
context.contactBackend->logSummary(context, session);
```

保留 `surface-pressure-force` 逻辑在 loop 中，位置在 `addForces()` 之后、`setqState()` 之前。

- [ ] **Step 5: CMake 加入 IPC backend source**

在 `RUN_IPC_SIM_CORE_SOURCES` 添加：

```cmake
  runIPCSimIpcContactBackend.cpp
```

- [ ] **Step 6: 验证默认 IPC 行为未变**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimCliGTest.*:RunIPCSimSetupGTest.*'
```

Expected:

- build 仍可能因为 legacy backend 未实现失败；如果已临时声明 stub，则 selected IPC tests PASS。
- 不允许出现 IPC smoke test 输出路径变化。

## Task 4: 实现 volume legacy penalty backend

**Files:**
- Create: `src/tools/runSim/runIPCSimLegacyPenaltyContact.h`
- Create: `src/tools/runSim/runIPCSimLegacyPenaltyContact.cpp`
- Modify: `src/tools/runSim/runIPCSimSetup.h`
- Modify: `src/tools/runSim/runIPCSimSetup.cpp`
- Modify: `src/tools/runSim/CMakeLists.txt`

- [ ] **Step 1: 定义 legacy contact config**

`runIPCSimLegacyPenaltyContact.h`：

```cpp
#pragma once

#include "EigenSupport.h"
#include "runIPCSimContactBackend.h"
#include "triMeshGeo.h"

#include <memory>
#include <string>
#include <vector>

namespace pgo
{
class ConfigFileJSON;
}

namespace pgo::RunIPCSim
{
struct LegacyPenaltyContactConfig
{
  double stiffness = 0.0;
  int samples = 0;
  double frictionCoeff = 0.0;
  double velocityEps = 0.0;
};

LegacyPenaltyContactConfig parseLegacyPenaltyContactConfig(const pgo::ConfigFileJSON &config);
std::shared_ptr<RunIPCSimContactBackend> makeLegacyPenaltyContactBackend(
  const pgo::ConfigFileJSON &config,
  const LegacyPenaltyContactConfig &contactConfig,
  const pgo::Mesh::TriMeshGeo &surfaceMesh,
  const EigenSupport::SpMatD &surfaceFromSimulationDispMap,
  int simulationDofCount,
  double scale);
}  // namespace pgo::RunIPCSim
```

- [ ] **Step 2: 解析 contact fields**

`parseLegacyPenaltyContactConfig()` 行为：

```cpp
LegacyPenaltyContactConfig parseLegacyPenaltyContactConfig(const pgo::ConfigFileJSON &config)
{
  const bool hasContactSample = config.exist("contact-sample");
  const bool hasContactSamples = config.exist("contact-samples");
  if (hasContactSample && hasContactSamples)
    throw std::invalid_argument("runIPCSim --legacy accepts either `contact-sample` or `contact-samples`, not both.");

  LegacyPenaltyContactConfig parsed;
  parsed.stiffness = config.getDouble("contact-stiffness", 1);
  parsed.samples = hasContactSamples ? config.getInt("contact-samples", 1) : config.getInt("contact-sample", 1);
  parsed.frictionCoeff = config.getDouble("contact-friction-coeff", 1);
  parsed.velocityEps = config.getDouble("contact-vel-eps", 1);

  if (parsed.stiffness < 0.0)
    throw std::invalid_argument("runIPCSim --legacy requires non-negative `contact-stiffness`.");
  if (parsed.samples <= 0)
    throw std::invalid_argument("runIPCSim --legacy requires positive contact sample count.");
  if (parsed.velocityEps <= 0.0)
    throw std::invalid_argument("runIPCSim --legacy requires positive `contact-vel-eps`.");

  return parsed;
}
```

- [ ] **Step 3: 实现 external object loading**

Legacy backend 内部定义：

```cpp
struct LegacyKinematicObject
{
  pgo::Mesh::TriMeshGeo mesh;
  EigenSupport::V3d movement = EigenSupport::V3d::Zero();
};
```

从 `external-objects[]` 加载：

- `filename` 用 `config.resolvePath()`
- mesh vertices 乘 `scale`
- `movement` 保存为每个 object 的 total movement

构建 handler 前先创建：

```cpp
std::vector<pgo::Mesh::TriMeshRef> refs;
for (auto &object : objects)
  refs.emplace_back(object.mesh);
```

`refs` 必须在 handler 构造期间有效；handler 构造后 backend 继续持有 `objects`，每次更新 obstacle mesh 后重新调用 `updateExternalSurface()`.

- [ ] **Step 4: 实现 backend class**

关键字段：

```cpp
LegacyPenaltyContactConfig config_;
std::vector<LegacyKinematicObject> objects_;
std::shared_ptr<Contact::TriangleMeshExternalContactHandler> externalContactHandler_;
std::shared_ptr<Contact::TriangleMeshSelfContactHandler> selfContactHandler_;
std::shared_ptr<Contact::PointPenetrationEnergy> activeExternalEnergy_;
Contact::PointPenetrationEnergyBuffer *activeExternalBuffer_ = nullptr;
std::shared_ptr<Contact::PointTrianglePairCouplingEnergyWithCollision> activeSelfEnergy_;
Contact::PointTrianglePairCouplingEnergyWithCollisionBuffer *activeSelfBuffer_ = nullptr;
```

`initializeAfterRestart()`：

```cpp
EigenSupport::mv(context.surfaceFromSimulationDispMap, session.u, session.usurf);
const double denom = runtimeConfig.numSimSteps > 1 ? static_cast<double>(runtimeConfig.numSimSteps - 1) : 1.0;
for (std::size_t oi = 0; oi < objects_.size(); ++oi) {
  const EigenSupport::V3d movement = objects_[oi].movement / denom * static_cast<double>(session.frameStart + 1);
  for (int vi = 0; vi < objects_[oi].mesh.numVertices(); ++vi)
    objects_[oi].mesh.pos(vi) += movement;
  if (externalContactHandler_)
    externalContactHandler_->updateExternalSurface(static_cast<int>(oi), pgo::Mesh::TriMeshRef(objects_[oi].mesh));
}
```

`beginFrame()`：

```cpp
releaseActiveBuffers();
```

`addForces()`：

- 如果 `externalContactHandler_` 存在：
  - `externalContactHandler_->execute(session.usurf.data())`
  - 有 colliding samples 时 build `PointPenetrationEnergy`
  - 设置 current position function:

```cpp
auto posFunc = [&context](const EigenSupport::V3d &u, EigenSupport::V3d &p, int dofStart) {
  p = u + context.simulationRestPosition.segment<3>(dofStart);
};
auto lastPosFunc = [&context, &session](const EigenSupport::V3d &, EigenSupport::V3d &p, int dofStart) {
  p = session.u.segment<3>(dofStart) + context.simulationRestPosition.segment<3>(dofStart);
};
```

  - 设置 `coeff/friction/timestep/velEps/buffer`
  - `session.integrator->addGeneralImplicitForceModel(activeExternalEnergy_, 0, 0)`
- 如果 `selfContactHandler_` 存在：
  - `selfContactHandler_->execute(session.usurf.data())`
  - 有 colliding pairs 时调用 `handleContactDCD(0, 100)`
  - build `PointTrianglePairCouplingEnergyWithCollision`
  - 设置 `toPos/toLastPos/coeff/friction/timestep/velEps/buffer`
  - 调用 `computeClosestPosition(session.u.data())`
  - 添加到 integrator

`afterStep()`：

```cpp
releaseActiveBuffers();
EigenSupport::mv(context.surfaceFromSimulationDispMap, session.u, session.usurf);
const double denom = runtimeConfig.numSimSteps > 1 ? static_cast<double>(runtimeConfig.numSimSteps - 1) : 1.0;
for (std::size_t oi = 0; oi < objects_.size(); ++oi) {
  const EigenSupport::V3d stepMovement = objects_[oi].movement / denom;
  for (int vi = 0; vi < objects_[oi].mesh.numVertices(); ++vi)
    objects_[oi].mesh.pos(vi) += stepMovement;
  if (externalContactHandler_)
    externalContactHandler_->updateExternalSurface(static_cast<int>(oi), pgo::Mesh::TriMeshRef(objects_[oi].mesh));
}
```

`logSummary()`：

```cpp
logRunIPCSimMaxStepSummary(context.elasticEnergy, nullptr, session.integrator);
```

当前 `logRunIPCSimMaxStepSummary()` 不解引用 `collisionHandler`，legacy backend 直接传 `nullptr`，不新增 summary API。

- [ ] **Step 5: 实现 `buildVolumeLegacyPenaltySimulation()`**

在 `runIPCSimSetup.h` 声明：

```cpp
IpcSimulationContext buildVolumeLegacyPenaltySimulation(const ConfigFileJSON &jconfig);
```

在 `runIPCSimSetup.cpp` 实现：

```cpp
IpcSimulationContext buildVolumeLegacyPenaltySimulation(const pgo::ConfigFileJSON &jconfig)
{
  validateZeroInitialDisplacement(jconfig);
  if (!jconfig.exist("tet-mesh") && !jconfig.exist("cubic-mesh"))
    throwConfigError("runIPCSim --legacy only supports volume legacy configs with `tet-mesh` or `cubic-mesh`; shell legacy has been removed.");
  if (jconfig.exist("tet-mesh") && jconfig.exist("cubic-mesh"))
    throwConfigError("runIPCSim --legacy expects exactly one of `tet-mesh` or `cubic-mesh`.");

  const double scale = jconfig.getDouble("scale", 1);
  const SolidDeformationModel::DeformationModelElasticMaterial elasticMat = parseVolumeElasticMaterial(jconfig);
  const bool enableMaterialMaxStep = parseEnableMaterialMaxStep(jconfig);
  const RunSim::ResolvedRunSimPaths resolvedPaths = RunSim::resolveRunSimPaths(jconfig);
  std::unique_ptr<VolumetricMeshes::VolumetricMesh> volumetricMesh =
    RunSim::loadValidatedVolumeMesh(RunSim::parseVolumeMeshInputConfig(jconfig), scale);

  pgo::Mesh::TriMeshGeo surfaceMesh;
  ES::VXd surfaceRestPositions;
  loadSurfaceMeshAndRestPositions(resolvedPaths.surfaceMeshFilename, scale, surfaceMesh, surfaceRestPositions);

  pgo::InterpolationCoordinates::BarycentricCoordinates bc(
    surfaceMesh.numVertices(), surfaceRestPositions.data(), volumetricMesh.get());
  ES::SpMatD W = bc.generateInterpolationMatrix();

  ES::SpMatD M;
  VolumetricMeshes::GenerateMassMatrix::computeMassMatrix(volumetricMesh.get(), M, true);

  RunSim::InitializedVolumetricSimulation initialized =
    RunSim::initializeVolumetricSimulation(*volumetricMesh, elasticMat,
      SolidDeformationModel::DeformationModelPlasticMaterial::VOLUMETRIC_DOF6,
      enableMaterialMaxStep);

  ES::VXd zero = ES::VXd::Zero(initialized.restPosition.size());
  ES::SpMatD K;
  initialized.elasticEnergy->createHessian(K);
  initialized.elasticEnergy->hessian(zero, K);

  std::vector<std::shared_ptr<ConstraintPotentialEnergies::MultipleVertexPulling>> pullingEnergies;
  std::vector<ES::VXd> pullingTargets;
  std::vector<ES::VXd> pullingTargetRests;
  buildPullingConstraints(jconfig, resolvedPaths.fixedVertexFilenames, initialized.restPosition, K,
    pullingEnergies, pullingTargets, pullingTargetRests);

  LegacyPenaltyContactConfig legacyContactConfig = parseLegacyPenaltyContactConfig(jconfig);

  IpcSimulationContext context;
  context.M = std::move(M);
  context.simulationRestPosition = std::move(initialized.restPosition);
  context.surfaceRestPositions = std::move(surfaceRestPositions);
  context.plasticParams = std::move(initialized.plasticity);
  context.surfaceFromSimulationDispMap = std::move(W);
  context.simulationMeshOwner = initialized.simMesh;
  context.deformationModelManagerOwner = initialized.dmm;
  context.deformationModelAssemblerOwner = initialized.assembler;
  context.elasticEnergy = initialized.elasticEnergy;
  context.pullingEnergies = std::move(pullingEnergies);
  context.pullingTargets = std::move(pullingTargets);
  context.pullingTargetRests = std::move(pullingTargetRests);
  context.surfaceMesh = std::move(surfaceMesh);
  context.contactBackend = makeLegacyPenaltyContactBackend(jconfig, legacyContactConfig,
    context.surfaceMesh, context.surfaceFromSimulationDispMap,
    static_cast<int>(context.simulationRestPosition.size()), scale);
  return context;
}
```

Add log before return:

```cpp
std::cout << "runIPCSim legacy volume penalty contact parameters: "
          << "contact-stiffness=" << legacyContactConfig.stiffness << ", "
          << "contact-samples=" << legacyContactConfig.samples << ", "
          << "contact-friction-coeff=" << legacyContactConfig.frictionCoeff << ", "
          << "contact-vel-eps=" << legacyContactConfig.velocityEps << std::endl;
```

- [ ] **Step 6: app calls backend initialization after restart**

In `runFromConfig()`, after `restoreRestartStateIfRequested(...)`:

```cpp
context.contactBackend->initializeAfterRestart(runtimeConfig, context, session);
```

- [ ] **Step 7: CMake 加入 legacy backend source**

在 `RUN_IPC_SIM_CORE_SOURCES` 添加：

```cmake
  runIPCSimLegacyPenaltyContact.cpp
```

- [ ] **Step 8: 运行 legacy tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimLegacyGTest.*'
```

Expected:

- PASS。
- smoke 输出存在 `states/deform0000.u` 和 `surface/ret0000.obj`。

## Task 5: 删除旧 executable targets 和 shell legacy tests

**Files:**
- Delete: `src/tools/runSim/runSim.cpp`
- Delete: `src/tools/runSim/runShellSim.cpp`
- Modify: `src/tools/runSim/CMakeLists.txt`
- Rename/Delete: `tests/src/tools/runSim_gtest.cpp`
- Create: `tests/src/tools/runSimShared_gtest.cpp`
- Modify: `tests/src/tools/CMakeLists.txt`

- [ ] **Step 1: 删除 CMake old targets**

从 `src/tools/runSim/CMakeLists.txt` 删除：

```cmake
add_libpgo_tools(runSim "runSim.cpp;runSimVolumeMeshIO.cpp;runSimFEMSetup.cpp;runSimCliLogging.cpp" "${AVAILABLE_LIBS}")
target_link_libraries(runSim PRIVATE argparse::argparse)

add_libpgo_tools(runShellSim "runShellSim.cpp;runSimCliLogging.cpp" "${AVAILABLE_LIBS}")
target_link_libraries(runShellSim PRIVATE argparse::argparse)
```

保留 `runSimVolumeMeshIO.cpp`、`runSimFEMSetup.cpp`、`runSimCliLogging.cpp` 在 `RUN_IPC_SIM_CORE_SOURCES` 中。

- [ ] **Step 2: 删除 source files**

Run:

```bash
git rm src/tools/runSim/runSim.cpp src/tools/runSim/runShellSim.cpp
```

- [ ] **Step 3: 拆分 tools tests**

创建 `tests/src/tools/runSimShared_gtest.cpp`，从 `runSim_gtest.cpp` 保留这些 suites：

- `RunSimVolumeMeshIOGTest.*`
- `RunSimCliLoggingGTest.*`

删除这些 shell executable tests：

- `RunShellSimCliLoggingGTest.LogFlagWritesCliOutputNextToConfig`
- `RunShellSimCliLoggingGTest.DeformStateIsWrittenEveryTimestep`

删除 helper：

- `makeShellSimConfig(...)`
- `shellExampleDir()`
- `runShellSimBinaryPath()`

- [ ] **Step 4: 更新 `tests/src/tools/CMakeLists.txt`**

替换 target：

```cmake
add_executable(runSimShared_gtest
  runSimShared_gtest.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runSimCliLogging.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runSimVolumeMeshIO.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runSimFEMSetup.cpp)
target_link_libraries(runSimShared_gtest PRIVATE
  GTest::gtest_main
  configFileJSON
  contact
  interpolationCoordinates
  solidDeformationModel
  volumetricMesh
  mesh)
target_include_directories(runSimShared_gtest PRIVATE
  ${CMAKE_SOURCE_DIR}/src/tools/runSim
  ${CMAKE_SOURCE_DIR}/src/core/solidDeformationModel)
target_compile_definitions(runSimShared_gtest PRIVATE
  LIBPGO_TEST_TET_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/box/box.veg"
  LIBPGO_TEST_TET_BOX_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/box/box.obj"
  LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg"
  LIBPGO_TEST_CUBIC_BOX_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.obj")
set_property(TARGET runSimShared_gtest PROPERTY FOLDER "tests/gtest")
gtest_discover_tests(runSimShared_gtest)
```

删除 `PGO_TEST_RUN_SHELL_SIM_BIN` 和 `add_dependencies(runSim_gtest runShellSim)`。

- [ ] **Step 5: 验证 CMake 不再暴露 old executable**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim runIPCSim_gtest runSimShared_gtest
test ! -e ./build/base_no_mkl/bin/runSim
test ! -e ./build/base_no_mkl/bin/runShellSim
```

Expected:

- build PASS。
- 两个 `test ! -e` PASS。

如果旧 binary 因之前构建残留仍存在，改用 configure-level 验证：

```bash
cmake --build --preset base_no_mkl_release --target runSim
```

Expected:

- FAIL with `No rule to make target` 或等价 target-not-found 信息。

## Task 6: 迁移 fixtures，删除 `examples/legacy`

**Files:**
- Create: `tests/fixtures/legacy/...`
- Create: `tests/fixtures/shell/shell.obj`
- Create: `tests/fixtures/config/shell_paths.json`
- Delete: `examples/legacy/...`
- Modify: test CMake files and Python test fixture paths.

- [ ] **Step 1: 创建 fixture directories**

Run:

```bash
mkdir -p tests/fixtures/legacy/tet/box
mkdir -p tests/fixtures/legacy/cubic/box
mkdir -p tests/fixtures/legacy/tet
mkdir -p tests/fixtures/shell
mkdir -p tests/fixtures/config
```

- [ ] **Step 2: 迁移仍被 tests 使用的 volume fixtures**

Run:

```bash
git mv examples/legacy/tet/box/box.veg tests/fixtures/legacy/tet/box/box.veg
git mv examples/legacy/tet/box/box.obj tests/fixtures/legacy/tet/box/box.obj
git mv examples/legacy/tet/box/box.json tests/fixtures/legacy/tet/box/box.json
git mv examples/legacy/tet/bottom.obj tests/fixtures/legacy/tet/bottom.obj
git mv examples/legacy/cubic/box/box.veg tests/fixtures/legacy/cubic/box/box.veg
git mv examples/legacy/cubic/box/box.obj tests/fixtures/legacy/cubic/box/box.obj
git mv examples/legacy/cubic/box/box.json tests/fixtures/legacy/cubic/box/box.json
git mv examples/legacy/cubic/bottom.obj tests/fixtures/legacy/cubic/bottom.obj
git mv examples/legacy/tet/torus.veg tests/fixtures/legacy/tet/torus.veg
```

- [ ] **Step 3: 迁移 shell mesh fixture，不保留 shell legacy example**

Run:

```bash
git mv examples/legacy/shell/shell.obj tests/fixtures/shell/shell.obj
git mv examples/legacy/shell/shell-fixed.txt tests/fixtures/shell/shell-fixed.txt
```

创建 `tests/fixtures/config/shell_paths.json`：

```json
{
  "surface-mesh": "../shell/shell.obj",
  "fixed-vertices": [
    {
      "filename": "../shell/fixed.txt",
      "movement": [0, 0, 0],
      "coeff": 100000
    }
  ],
  "external-objects": [
    {
      "filename": "../shell/obstacle.obj",
      "movement": [0, 0, 0]
    }
  ],
  "output": "ret-shell-path-test"
}
```

这个 JSON 只用于 path resolution，不要求 `fixed.txt` 或 `obstacle.obj` 存在。

- [ ] **Step 4: 更新 core test CMake paths**

`tests/src/core/CMakeLists.txt`：

```cmake
target_compile_definitions(configFileJSON_gtest PRIVATE
  LIBPGO_TEST_BOX_JSON="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/box/box.json"
  LIBPGO_TEST_CUBIC_BOX_JSON="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.json"
  LIBPGO_TEST_SHELL_JSON="${CMAKE_SOURCE_DIR}/tests/fixtures/config/shell_paths.json")
```

`tests/src/core/solidDeformationModel/CMakeLists.txt`：

```cmake
LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg"
LIBPGO_TEST_SHELL_OBJ="${CMAKE_SOURCE_DIR}/tests/fixtures/shell/shell.obj"
LIBPGO_TEST_CUBIC_BOX_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box/box.veg"
```

`tests/CMakeLists.txt`：

```cmake
target_compile_definitions(pgo_c_gtest PRIVATE
  LIBPGO_TEST_TORUS_VEG="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/torus.veg")
```

- [ ] **Step 5: 更新 tools test fixture paths**

`tests/src/tools/CMakeLists.txt`:

```cmake
target_compile_definitions(runIPCSim_gtest PRIVATE
  LIBPGO_TEST_SHELL_EXAMPLE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures/shell"
  LIBPGO_TEST_LEGACY_TET_BOX_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/tet/box"
  LIBPGO_TEST_LEGACY_CUBIC_BOX_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures/legacy/cubic/box"
  LIBPGO_TEST_IPC_TET_EXAMPLE_DIR="${CMAKE_SOURCE_DIR}/examples/ipc/tet/box-hang"
  LIBPGO_TEST_IPC_CUBIC_EXAMPLE_DIR="${CMAKE_SOURCE_DIR}/examples/ipc/cubic/box-hang"
  PGO_TEST_RUN_IPC_SIM_BIN="$<TARGET_FILE:runIPCSim>")
```

- [ ] **Step 6: 更新 Python fixture paths**

`tests/pypgo/test_pgo_smoke.py`：

```python
FIXTURES_DIR = Path(__file__).resolve().parents[1] / "fixtures" / "legacy" / "tet"
TORUS_VEG = FIXTURES_DIR / "torus.veg"
```

`src/python/pypgo/pgo_test_01.py`：

```python
TORUS_VEG = Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "legacy" / "tet" / "torus.veg"
```

- [ ] **Step 7: 删除 remaining legacy example tree**

Run:

```bash
git rm -r examples/legacy
```

If `git rm -r examples/legacy` fails because moved files left empty directories only, run:

```bash
find examples/legacy -type f -print
```

Expected:

- Lists only user-facing legacy examples/media/docs that are no longer test fixtures.

Then remove the listed files with `git rm`.

- [ ] **Step 8: 验证没有 test 指向 `examples/legacy`**

Run:

```bash
rg -n "examples/legacy|legacy/shell|runShellSim|PGO_TEST_RUN_SHELL_SIM_BIN" tests src/tools README.md
```

Expected:

- No matches for `examples/legacy`, `legacy/shell`, `runShellSim`, `PGO_TEST_RUN_SHELL_SIM_BIN`.
- Matches for `runSimVolumeMeshIO`, `runSimFEMSetup`, `runSimCliLogging` are allowed.

## Task 7: 更新 README 和旧入口文档

**Files:**
- Modify: `README.md`
- Delete: `examples/legacy/README.md`
- Delete: `examples/legacy/cubic/README.md`

- [ ] **Step 1: 删除 README 中 legacy executable 入口**

删除或改写这些内容：

- `For non-IPC legacy examples (runSim, runShellSim, pgo_run_sim.py, pgo_dump_abc.py)...`
- `The non-IPC example suite has been moved to examples/legacy/`
- `runSim` / `runShellSim` 命令示例
- 指向 `examples/legacy/cubic/README.md` 的链接

- [ ] **Step 2: 增加 `runIPCSim --legacy` 文档**

加入：

````markdown
### Legacy Penalty Contact

`runIPCSim` also supports the old volume penalty-contact model through `--legacy`:

```bash
build/base_no_mkl/bin/runIPCSim --legacy path/to/legacy-volume.json
```

This mode accepts old volume configs with `tet-mesh` or `cubic-mesh` and contact fields such as
`contact-stiffness`, `contact-sample`, `contact-friction-coeff`, and `contact-vel-eps`.
It does not support the removed shell legacy runner. Use the default `runIPCSim` IPC path for shell IPC examples.
Legacy mode uses the unified `runIPCSim` output layout under `output/states/` and `output/surface/`.
````

- [ ] **Step 3: 验证 docs 中没有死链接**

Run:

```bash
rg -n "examples/legacy|runShellSim|build/.*/bin/runSim|pgo_run_sim.py" README.md examples src tests
```

Expected:

- No matches in README/examples/tests.
- `src/python/pypgo/pgo_run_sim.py` remains because C/Python legacy API removal is a declared non-goal; if `rg` reports that filename from its own path, record it as remaining C/Python API follow-up, not this plan's blocker.

## Task 8: Full validation

**Files:** no source edits unless validation exposes a bug in prior tasks.

- [ ] **Step 1: Build changed targets**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim runIPCSim_gtest runSimShared_gtest configFileJSON_gtest deformationModelAssembler_gtest deformationModelEnergyMaxStep_gtest simulationMesh_gtest pgo_c_gtest
```

Expected:

- PASS。

- [ ] **Step 2: Run focused tests**

Run:

```bash
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runSimShared_gtest
./build/base_no_mkl/tests/src/core/configFileJSON_gtest
./build/base_no_mkl/tests/src/core/solidDeformationModel/deformationModelAssembler_gtest
./build/base_no_mkl/tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest
./build/base_no_mkl/tests/src/core/solidDeformationModel/simulationMesh_gtest
./build/base_no_mkl/tests/pgo_c_gtest
```

Expected:

- PASS。
- `runIPCSim_gtest` includes IPC default path and legacy penalty path.
- No `runShellSim` test remains.

- [ ] **Step 3: Verify removed targets**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runSim
cmake --build --preset base_no_mkl_release --target runShellSim
```

Expected:

- Both fail with target-not-found / no rule to make target.

- [ ] **Step 4: Run search hygiene**

Run:

```bash
rg -n "runShellSim|PGO_TEST_RUN_SHELL_SIM_BIN|examples/legacy|legacy/shell" .
```

Expected:

- No matches in tracked source/docs/tests.
- Matches inside `.git/`, build outputs, or generated caches are ignored by rerunning:

```bash
rg -n "runShellSim|PGO_TEST_RUN_SHELL_SIM_BIN|examples/legacy|legacy/shell" README.md src tests examples plan
```

Expected:

- Only this plan file may mention removed names.

- [ ] **Step 5: Diff hygiene**

Run:

```bash
git diff --check
git status --short
```

Expected:

- `git diff --check` PASS。
- `git status --short` shows only planned source/test/docs/fixture moves and deletions.

## Rollback checkpoints

- After Task 2, rollback is limited to CLI/app option wiring.
- After Task 3, default IPC path should still pass focused IPC tests; if it fails, revert Task 3 before starting legacy backend.
- After Task 4, legacy backend is isolated behind `--legacy`; if it fails, default IPC should remain usable.
- After Task 5, old executable targets are gone; if downstream still requires them, restore only CMake targets and source files, not shell legacy examples.
- After Task 6, `examples/legacy` is gone; if a test needs a missing asset, move that asset into `tests/fixtures`, not back under `examples/legacy`.

## Done criteria

- `runIPCSim` default IPC behavior remains green under `runIPCSim_gtest`.
- `runIPCSim --legacy` accepts tet/cubic old volume JSON and writes unified output.
- `runIPCSim --legacy` rejects shell legacy JSON with a clear error.
- `runSim` and `runShellSim` CMake targets no longer exist.
- `examples/legacy` no longer exists.
- Tests and docs no longer reference `examples/legacy` or `runShellSim`.
- Shell mesh assets needed by core tests live under `tests/fixtures/shell`.
- Volume assets needed by tests live under `tests/fixtures/legacy`.
- `git diff --check` passes.

## Self Review

### Coverage check

- User requirement “legacy 模式迁移到新的入口中，添加 `--legacy`”: covered by Tasks 1, 2, 4, 8.
- User requirement “接受旧 JSON 文件，用旧 contact 模型”: covered by Task 4, including `contact-stiffness`, `contact-sample(s)`, friction, velocity epsilon, external objects, external/self penalty contact.
- User decision “shell legacy 直接删”: covered by Tasks 5 and 6, with explicit shell legacy rejection in Task 1/4.
- “尽量复用刚刚重构好的 sim 入口代码”: covered by reusing `runIPCSimCli`, `runIPCSimApp`, `runIPCSimConfig`, `runIPCSimOutput`, `runIPCSimSession`, and `runIPCSimLoop`; only contact behavior moves behind backend hooks.
- Fixtures and docs cleanup: covered by Tasks 6 and 7.
- Validation: covered by Task 8.

### Ambiguity fixes applied

- Clarified that `--legacy` means legacy contact backend, not old output layout.
- Clarified that `runSimVolumeMeshIO.*`, `runSimFEMSetup.*`, and `runSimCliLogging.*` remain because `runIPCSimCore` uses them.
- Clarified that C/Python `pgo_run_sim_from_config()` is not removed in this plan.
- Clarified `contact-sample` vs `contact-samples` conflict behavior.
- Clarified shell legacy rejection message.

### Residual risks

- `TriangleMeshExternalContactHandler` and `TriangleMeshSelfContactHandler` allocate temporary energy buffers; Task 4 explicitly requires `releaseActiveBuffers()` in `beginFrame()` and `afterStep()` to prevent leaks across solver failures.
- Existing `runSim.cpp` supported `init-disp`; current `runIPCSim` rejects non-zero `init-disp`. This plan keeps the new entry invariant and rejects non-zero displacement in `--legacy`.
- Moving fixtures can uncover hidden assumptions in Python packaging or external scripts. The full search hygiene step catches repo-local references; external users need release notes if this is published.
