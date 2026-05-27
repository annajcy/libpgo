# Run IPC Sim Modular Entry Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变 `runIPCSim` CLI、config schema、输出路径、日志文本和仿真语义的前提下，把入口重构成薄 `main()`、typed config、output manager、session、frame loop 和 app runner 的清晰模块。

**Architecture:** 新增 `runIPCSimCore` static library 承载入口以外的可测试逻辑，`runIPCSim.cpp` 只保留 `main()`，`runIPCSimCli.*` 负责 argparse，`runIPCSimApp.*` 负责顶层编排。`runIPCSimSetup.*` 继续负责 shell/volume IPC context 构建，本轮只小幅公开 `floorHeightAtFrame()` 供 frame loop 复用。

**Tech Stack:** C++20, CMake, argparse, nlohmann/json, Eigen, TBB, GoogleTest, existing `runSim` / `simulation` / `contact` libraries.

---

## 当前问题

`src/tools/runSim/runIPCSim.cpp` 目前同时承担：

- CLI parsing。
- `ConfigFileJSON` 字段读取。
- output directory 清理、创建和 frame path 生成。
- CLI log redirect、spdlog 初始化、profiling 生命周期。
- shell/volume IPC context 选择。
- integrator 和 runtime state 初始化。
- restart state scan。
- 每帧 simulation loop。
- deformation state、surface OBJ、von Mises stress JSON 输出。
- max-step summary 和 profiling summary 日志。

入口可读性已经被这些细节淹没；后续添加新输出、新 config 或新 per-frame force 时，容易继续膨胀 `main()`。本计划把入口改成“故事线”：

```cpp
int main(int argc, char *argv[])
{
  return pgo::RunIPCSim::runCli(argc, argv);
}
```

而 `runFromConfig()` 负责清晰编排：

```cpp
int runFromConfig(const std::filesystem::path &configPath, const RunIPCSimOptions &options)
{
  pgo::ConfigFileJSON config;
  if (!openRunIPCSimConfig(configPath, config))
    return 0;
  const RunIPCSimRuntimeConfig runtimeConfig = parseRunIPCSimRuntimeConfig(config);
  RunIPCSimOutput output(runtimeConfig.outputFolder);
  output.prepare(runtimeConfig.restartFromU);
  RunIPCSimRunScope runScope(config, runtimeConfig, options, output);
  if (!runtimeConfig.restartFromU)
    std::cout << "restart-from-u=false; clearing output folder " << runtimeConfig.outputFolder << "." << std::endl;
  pgo::Mesh::initPredicates();
  IpcSimulationContext context = buildIpcSimulation(config);
  RunIPCSimSession session = createRunIPCSimSession(runtimeConfig, context);
  restoreRestartStateIfRequested(runtimeConfig, output, session);
  runIPCSimLoop(runtimeConfig, context, session, output);
  runScope.logProfileSummaryIfEnabled();
  return 0;
}
```

## 范围与非目标

- 修改 `src/tools/runSim/runIPCSim.cpp`，但最终只保留薄 `main()`。
- 新增 `runIPCSimCli.*`、`runIPCSimApp.*`、`runIPCSimConfig.*`、`runIPCSimOutput.*`、`runIPCSimLogging.*`、`runIPCSimSession.*`、`runIPCSimLoop.*`。
- 小幅修改 `runIPCSimSetup.h/.cpp`，公开 `floorHeightAtFrame()`。
- 修改 `src/tools/runSim/CMakeLists.txt`，新增 `runIPCSimCore` static library。
- 修改 `tests/src/tools/CMakeLists.txt`，让 `runIPCSim_gtest` 链接 `runIPCSimCore`，不再直接编译 `runIPCSimSetup.cpp` 等 core sources。
- 修改 `tests/src/tools/runIPCSim_gtest.cpp`，增加直接覆盖 typed config、output path 和 `runFromConfig()` 的 characterization tests。
- 不改变 JSON config schema。
- 不改变 `runIPCSim` CLI 参数：仍为 `runIPCSim [--log] <config>`。
- 不改变输出目录结构：`states/deform%04d.u`、`surface/ret%04d.obj`、`stress/von_mises%04d.json`、`runIPCSim.log`。
- 不重构 `runShellSim`。
- 不拆 `buildShellIpcSimulation()` / `buildVolumeIpcSimulation()` 内部大流程；那是后续第二轮。
- 不改变 `IpcSimulationContext` 的字段语义。

## 目标文件布局

### `src/tools/runSim/runIPCSim.cpp`

最终只包含：

```cpp
#include "runIPCSimCli.h"

int main(int argc, char *argv[])
{
  return pgo::RunIPCSim::runCli(argc, argv);
}
```

### `src/tools/runSim/runIPCSimCli.h/.cpp`

职责：argparse 到 typed CLI options，并把错误输出保持在 CLI 层。

```cpp
namespace pgo::RunIPCSim
{
struct RunIPCSimCliOptions
{
  std::filesystem::path configPath;
  RunIPCSimOptions runOptions;
};

RunIPCSimCliOptions parseRunIPCSimCli(int argc, char *argv[]);
int runCli(int argc, char *argv[]);
}  // namespace pgo::RunIPCSim
```

`runCli()` 捕获 argparse error，打印 `program` usage，返回 `1`；成功解析后调用 `runFromConfig()`.

### `src/tools/runSim/runIPCSimApp.h/.cpp`

职责：打开 config，编排 runtime config、output、logging、context、session、loop。

```cpp
namespace pgo::RunIPCSim
{
struct RunIPCSimOptions
{
  bool enableCliLog = false;
};

bool openRunIPCSimConfig(const std::filesystem::path &configPath, pgo::ConfigFileJSON &config);
IpcSimulationContext buildIpcSimulation(const pgo::ConfigFileJSON &config);
int runFromConfig(const std::filesystem::path &configPath, const RunIPCSimOptions &options);
}  // namespace pgo::RunIPCSim
```

`openRunIPCSimConfig()` 必须保持当前行为：`ConfigFileJSON::open()` 返回 false 时 `runFromConfig()` 返回 `0`，不把 config open failure 变成 process failure。`runFromConfig()` 捕获 runtime exception，使用 `SPDLOG_LOGGER_ERROR(Logging::lgr(), "{}", err.what())`，返回 `1`，并确保 profiling reset。

### `src/tools/runSim/runIPCSimConfig.h/.cpp`

职责：只读 config 字段，不做文件系统副作用，不 build IPC context。

```cpp
namespace pgo::RunIPCSim
{
struct RunIPCSimRuntimeConfig
{
  EigenSupport::V3d gravity = EigenSupport::V3d::Zero();
  EigenSupport::V3d initialVelocity = EigenSupport::V3d::Zero();
  double timestep = 0.0;
  double scale = 1.0;
  double solverEps = 0.0;
  int solverMaxIter = 0;
  std::array<double, 2> dampingParams = { 0.0, 0.0 };
  int numSimSteps = 0;
  int frameGap = 1;
  bool restartFromU = false;
  bool dumpDeformEveryFrame = false;
  bool outputVonMises = false;
  bool enableProfiling = false;
  std::filesystem::path outputFolder;
};

RunIPCSimRuntimeConfig parseRunIPCSimRuntimeConfig(const pgo::ConfigFileJSON &config);
}  // namespace pgo::RunIPCSim
```

必须保持当前行为：

- `g`, `init-vel`, `timestep`, `scale`, `solver-eps`, `solver-max-iter`, `damping-params`, `num-timestep`, `dump-interval`, `output` 仍为 required。
- `sim-type` 必须等于 `dynamic`，否则抛出 `runIPCSim phase2 only supports `sim-type = dynamic`.`。
- `restart-from-u` 默认 `false`。
- `dump_deform_every_frame` 默认 `false`。
- `output-von-mises` 默认 `false`。
- `profiling` 默认 `false`。

### `src/tools/runSim/runIPCSimOutput.h/.cpp`

职责：路径、目录、state/surface/stress 输出。

```cpp
namespace pgo::RunIPCSim
{
struct OutputDirectories
{
  std::filesystem::path root;
  std::filesystem::path states;
  std::filesystem::path surface;
  std::filesystem::path stress;
};

class RunIPCSimOutput
{
public:
  explicit RunIPCSimOutput(std::filesystem::path outputFolder);

  const OutputDirectories &directories() const { return outputDirs_; }
  std::filesystem::path logPath() const;
  std::filesystem::path statePath(int frame) const;
  std::filesystem::path surfacePath(int outputFrame) const;
  std::filesystem::path stressPath(int frame) const;

  void prepare(bool restartFromU) const;
  int loadLatestRestartState(int numSimSteps, int n3,
    EigenSupport::VXd &u, EigenSupport::VXd &uvel, EigenSupport::VXd &uacc) const;
  void writeState(int frame, const EigenSupport::VXd &u,
    const EigenSupport::VXd &uvel, const EigenSupport::VXd &uacc) const;
  void writeSurface(int outputFrame, const pgo::Mesh::TriMeshGeo &mesh) const;
  void writeVonMisesStressJson(int frame, double timestep,
    const IpcSimulationContext &context, const EigenSupport::VXd &displacement) const;

private:
  OutputDirectories outputDirs_;
};

std::filesystem::path framePath(const std::filesystem::path &dir, const char *prefix, int frame, const char *extension);
}  // namespace pgo::RunIPCSim
```

`prepare(false)` 保持当前行为：`remove_all(outputFolder)` 后创建 root/states/surface/stress，并由 caller 打印 `restart-from-u=false; clearing output folder ...`。

### `src/tools/runSim/runIPCSimLogging.h/.cpp`

职责：CLI log redirect、spdlog init、profiling 生命周期、summary logs。

```cpp
namespace pgo::RunIPCSim
{
class RunIPCSimRunScope
{
public:
  RunIPCSimRunScope(const pgo::ConfigFileJSON &config,
    const RunIPCSimRuntimeConfig &runtimeConfig,
    const RunIPCSimOptions &options,
    const RunIPCSimOutput &output);
  ~RunIPCSimRunScope();

  RunIPCSimRunScope(const RunIPCSimRunScope &) = delete;
  RunIPCSimRunScope &operator=(const RunIPCSimRunScope &) = delete;

  void logProfileSummaryIfEnabled() const;

private:
  bool profilingEnabled_ = false;
  std::unique_ptr<RunSim::ScopedRunSimCliLogRedirect> logRedirect_;
};

void logRunIPCSimMaxStepSummary(
  const std::shared_ptr<pgo::SolidDeformationModel::DeformationModelEnergy> &elasticEnergy,
  const std::shared_ptr<pgo::Contact::CIPC::EmbeddedSurfaceIPCPotentialEnergy> &collisionHandler,
  const std::shared_ptr<pgo::Simulation::ImplicitBackwardEulerTimeIntegrator> &integrator);
}  // namespace pgo::RunIPCSim
```

构造顺序必须保持：

1. `RunIPCSimOutput::prepare()` 先创建 output folder。
2. `RunIPCSimRunScope` 根据 `options.enableCliLog` 创建 `ScopedRunSimCliLogRedirect(output.logPath().string())`。
3. 调用 `pgo::Logging::init(nullptr, RunSim::resolveConfiguredLogLevel(config))`。
4. 如果 profiling enabled，调用 `Profiling::setProfilingEnabled(true)` 和 `Profiling::resetProfileStatistics()`。
5. 如果 `restartFromU=false`，在 scope 构造后打印 `restart-from-u=false; clearing output folder ...`，使该消息继续进入 `runIPCSim.log`。

`runFromConfig()` 必须在 `RunIPCSimRunScope` 析构之前调用 `runScope.logProfileSummaryIfEnabled()`；析构时如果 profiling enabled，调用 `Profiling::setProfilingEnabled(false)` 和 `Profiling::resetProfileStatistics()`。

### `src/tools/runSim/runIPCSimSession.h/.cpp`

职责：runtime vectors、gravity force、external force、integrator 初始化和 restart state restore。

```cpp
namespace pgo::RunIPCSim
{
struct RunIPCSimSession
{
  std::shared_ptr<pgo::Simulation::ImplicitBackwardEulerTimeIntegrator> integrator;
  EigenSupport::VXd u;
  EigenSupport::VXd uvel;
  EigenSupport::VXd uacc;
  EigenSupport::VXd usurf;
  EigenSupport::VXd gravityForce;
  EigenSupport::VXd fext;
  int frameStart = -1;
};

RunIPCSimSession createRunIPCSimSession(const RunIPCSimRuntimeConfig &runtimeConfig,
  const IpcSimulationContext &context);
void restoreRestartStateIfRequested(const RunIPCSimRuntimeConfig &runtimeConfig,
  const RunIPCSimOutput &output, RunIPCSimSession &session);
}  // namespace pgo::RunIPCSim
```

`createRunIPCSimSession()` 必须保持：

- `g.segment<3>(vi * 3) = runtimeConfig.gravity`。
- `gravityForce = M * g`。
- `fext = gravityForce`。
- 初始化 `ImplicitBackwardEulerTimeIntegrator(context.M, context.elasticEnergy, damping[0], damping[1], timestep, solverMaxIter, solverEps)`。
- 添加 `context.pullingEnergies` 为 implicit force model。
- `setExternalForce(fext.data())`。
- `u/uvel/uacc/usurf` 初始为 zero。
- `uvel` 每个顶点 segment 设为 `runtimeConfig.initialVelocity`。

`restoreRestartStateIfRequested()` 必须保持：

- `restartFromU=false` 不读 restart state，`frameStart` 保持 `-1`。
- `restartFromU=true` 从 `numSimSteps - 1` 到 `0` 倒序找 `states/deform%04d.u`。
- 找到后读 `n3 x 3` matrix，设置 `u/uvel/uacc`，打印 `Restarting from frame X`。
- 找不到时打印 `No restart state found in ... Starting from frame 0.`。

### `src/tools/runSim/runIPCSimLoop.h/.cpp`

职责：每帧流程，完全不读 JSON、不碰 CLI、不创建目录。

```cpp
namespace pgo::RunIPCSim
{
void runIPCSimLoop(const RunIPCSimRuntimeConfig &runtimeConfig,
  IpcSimulationContext &context,
  RunIPCSimSession &session,
  const RunIPCSimOutput &output);
}  // namespace pgo::RunIPCSim
```

必须保持当前 frame loop 顺序：

1. `integrator->clearGeneralImplicitForceModel()`。
2. 更新 pulling target，并打印 `Frame X, attachment Y target: ...`。
3. 添加 `context.collisionHandler`。
4. 更新 floor heights。
5. 添加 `context.extraGeneralImplicitForceModels`。
6. 如果 `surfacePressureForceEnabled`，更新 `fext` 并 `setExternalForce()`。
7. `setqState(u, uvel, uacc)`。
8. `collisionHandler->setObstacleTime(tCurr + timestep)`。
9. `doTimestep(1, 3, 1)`。
10. `getq/getqvel/getqacc`。
11. `logRunIPCSimMaxStepSummary()`。
12. 按 `dumpDeformEveryFrame || framei % frameGap == 0` 写 state。
13. 如果 `outputVonMises` 写 stress JSON。
14. 按 `framei % frameGap == 0` 写 surface OBJ，路径保持 `ret(framei / frameGap).obj`。
15. 如果没有执行任何 timestep，仍调用一次 `logRunIPCSimMaxStepSummary()`。

### `src/tools/runSim/runIPCSimSetup.h/.cpp`

保留 `IpcSimulationContext` 和 builder，新增公开函数：

```cpp
double floorHeightAtFrame(const IpcFloorMotionState &motion, int frame);
```

把当前 anonymous namespace 内同名函数移动为 namespace-level function，`parseFloorsConfig()` 和 `runIPCSimLoop()` 共同调用它。

## 任务清单

### Task 1: 添加模块化入口的 characterization tests

**Files:**
- Modify: `tests/src/tools/runIPCSim_gtest.cpp`
- Modify: `tests/src/tools/CMakeLists.txt`

- [ ] **Step 1.1: 在 test include 区预先引用新 headers**

在 `tests/src/tools/runIPCSim_gtest.cpp` 的 include 区加入：

```cpp
#include "runIPCSimApp.h"
#include "runIPCSimConfig.h"
#include "runIPCSimOutput.h"
```

Expected RED: 此时 headers 不存在，`runIPCSim_gtest` 编译失败，报 `file not found`。

- [ ] **Step 1.2: 增加 runtime config parsing test**

在 `VolumeSetupRespectsDisabledMaterialMaxStepFlag` 前新增：

```cpp
TEST(RunIPCSimConfigGTest, RuntimeConfigParsesRequiredAndOptionalFields)
{
  initializeRunIPCSimTestEnvironment();

  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "shell-runtime-config.json";
  writeTextFile(configPath,
    addBoolConfigField(
      addBoolConfigField(makeShellIPCConfig(tempDir.path(), 3, true, false, 0.002, 3000.0, 2),
        "restart-from-u", true),
      "dump_deform_every_frame", true));

  pgo::ConfigFileJSON config;
  ASSERT_TRUE(config.open(configPath.string().c_str()));
  config.handle()["profiling"] = true;
  config.handle()["output-von-mises"] = false;

  const pgo::RunIPCSim::RunIPCSimRuntimeConfig runtime =
    pgo::RunIPCSim::parseRunIPCSimRuntimeConfig(config);

  EXPECT_EQ(runtime.numSimSteps, 3);
  EXPECT_EQ(runtime.frameGap, 2);
  EXPECT_TRUE(runtime.restartFromU);
  EXPECT_TRUE(runtime.dumpDeformEveryFrame);
  EXPECT_FALSE(runtime.outputVonMises);
  EXPECT_TRUE(runtime.enableProfiling);
  EXPECT_DOUBLE_EQ(runtime.scale, 1.0);
  EXPECT_DOUBLE_EQ(runtime.timestep, 0.001);
  EXPECT_EQ(runtime.outputFolder.filename(), "shell-output");
  EXPECT_NEAR(runtime.gravity[1], -9.81, 1e-12);
}
```

- [ ] **Step 1.3: 增加 output path test**

新增：

```cpp
TEST(RunIPCSimOutputGTest, OutputPathsPreserveCurrentLayout)
{
  ScopedTempDir tempDir;
  const fs::path outputDir = tempDir.path() / "ipc-output";
  const pgo::RunIPCSim::RunIPCSimOutput output(outputDir);

  EXPECT_EQ(output.directories().root, outputDir);
  EXPECT_EQ(output.directories().states, outputDir / "states");
  EXPECT_EQ(output.directories().surface, outputDir / "surface");
  EXPECT_EQ(output.directories().stress, outputDir / "stress");
  EXPECT_EQ(output.logPath(), outputDir / "runIPCSim.log");
  EXPECT_EQ(output.statePath(7), outputDir / "states" / "deform0007.u");
  EXPECT_EQ(output.surfacePath(3), outputDir / "surface" / "ret0003.obj");
  EXPECT_EQ(output.stressPath(11), outputDir / "stress" / "von_mises0011.json");
}
```

- [ ] **Step 1.4: 增加 direct app runner test**

新增：

```cpp
TEST(RunIPCSimAppGTest, RunFromConfigNoTimestepsMatchesCliSuccess)
{
  initializeRunIPCSimTestEnvironment();

  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "shell-runner-zero-step.json";
  const fs::path logPath = tempDir.path() / "shell-output" / "runIPCSim.log";

  writeTextFile(configPath, makeShellIPCConfig(tempDir.path(), 0));

  pgo::RunIPCSim::RunIPCSimOptions options;
  options.enableCliLog = true;

  EXPECT_EQ(pgo::RunIPCSim::runFromConfig(configPath, options), 0);
  ASSERT_TRUE(fs::exists(logPath));
  const std::string contents = readTextFile(logPath);
  EXPECT_NE(contents.find("runIPCSim phase2 shell IPC parameters:"), std::string::npos);
  EXPECT_NE(contents.find("max-step summary"), std::string::npos);
}
```

- [ ] **Step 1.5: 暂时把新 source 加进 test target**

在 `tests/src/tools/CMakeLists.txt` 的 `runIPCSim_gtest` source list 中，先加入将要创建的 core sources：

```cmake
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimApp.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimConfig.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimOutput.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimLogging.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimSession.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimLoop.cpp
```

This is temporary for Task 1/2. Task 3 replaces direct source inclusion with `runIPCSimCore` linkage.

- [ ] **Step 1.6: Run RED build**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
```

Expected: fails because new headers/sources do not exist. The failure must mention one of `runIPCSimApp.h`, `runIPCSimConfig.h`, `runIPCSimOutput.h`, or a missing source path.

### Task 2: 新增 config/output/logging/app/session/loop modules

**Files:**
- Create: `src/tools/runSim/runIPCSimApp.h`
- Create: `src/tools/runSim/runIPCSimApp.cpp`
- Create: `src/tools/runSim/runIPCSimConfig.h`
- Create: `src/tools/runSim/runIPCSimConfig.cpp`
- Create: `src/tools/runSim/runIPCSimOutput.h`
- Create: `src/tools/runSim/runIPCSimOutput.cpp`
- Create: `src/tools/runSim/runIPCSimLogging.h`
- Create: `src/tools/runSim/runIPCSimLogging.cpp`
- Create: `src/tools/runSim/runIPCSimSession.h`
- Create: `src/tools/runSim/runIPCSimSession.cpp`
- Create: `src/tools/runSim/runIPCSimLoop.h`
- Create: `src/tools/runSim/runIPCSimLoop.cpp`
- Modify: `src/tools/runSim/runIPCSimSetup.h`
- Modify: `src/tools/runSim/runIPCSimSetup.cpp`

- [ ] **Step 2.1: Create `runIPCSimConfig.h/.cpp`**

Implement the struct and parser from the target layout. Parser code must use the same field names and defaults:

```cpp
const ES::V3d gravity = ES::Mp<ES::V3d>(config.getValue<std::array<double, 3>>("g", 1).data());
const ES::V3d initialVelocity = ES::Mp<ES::V3d>(config.getValue<std::array<double, 3>>("init-vel", 1).data());
const std::string simType = config.getString("sim-type");
if (simType != "dynamic")
  throw std::invalid_argument("runIPCSim phase2 only supports `sim-type = dynamic`.");
```

The implementation must assign every `RunIPCSimRuntimeConfig` field explicitly.

- [ ] **Step 2.2: Create `runIPCSimOutput.h/.cpp`**

Move these functions from current `runIPCSim.cpp` into the class:

- `makeOutputDirectories()` as constructor logic.
- `framePath()`.
- `createOutputSubdirectories()`.
- `clearOutputDirectory()`.
- `writeVonMisesStressJson()`.

Add state/surface helpers:

```cpp
int RunIPCSimOutput::loadLatestRestartState(int numSimSteps, int n3,
  ES::VXd &u, ES::VXd &uvel, ES::VXd &uacc) const;
void RunIPCSimOutput::writeState(int frame, const ES::VXd &u,
  const ES::VXd &uvel, const ES::VXd &uacc) const;
void RunIPCSimOutput::writeSurface(int outputFrame, const pgo::Mesh::TriMeshGeo &mesh) const;
```

`loadLatestRestartState()` returns found frame or `-1`. It must use `ES::readMatrix(statePath(frame).string().c_str(), uMat)` and preserve the current column mapping.

- [ ] **Step 2.3: Create `runIPCSimLogging.h/.cpp`**

Move from current `runIPCSim.cpp`:

- `logRunIPCSimMaxStepSummary()`.
- `logProfileSummary()`, renamed to private helper inside `.cpp`.
- `resolveRunIPCSimLogPath()` folded into `RunIPCSimOutput::logPath()`.

Implement `RunIPCSimRunScope` RAII:

```cpp
RunIPCSimRunScope::~RunIPCSimRunScope()
{
  if (profilingEnabled_) {
    pgo::Profiling::setProfilingEnabled(false);
    pgo::Profiling::resetProfileStatistics();
  }
}
```

- [ ] **Step 2.4: Expose `floorHeightAtFrame()` from setup**

In `runIPCSimSetup.h`, add:

```cpp
double floorHeightAtFrame(const IpcFloorMotionState &motion, int frame);
```

In `runIPCSimSetup.cpp`, move the current anonymous namespace `floorHeightAtFrame()` to namespace scope inside `pgo::RunIPCSim`. Keep implementation byte-for-byte equivalent.

- [ ] **Step 2.5: Create `runIPCSimSession.h/.cpp`**

Move session initialization from current `main()` lines around gravity, integrator creation, `u/uvel/uacc/usurf`, and initial velocity. The function must return a fully initialized `RunIPCSimSession`.

`restoreRestartStateIfRequested()` prints:

- `Restarting from frame X` when state found.
- `No restart state found in <states-dir>. Starting from frame 0.` when requested but missing.
- `Starting from frame 0.` when not requested.

- [ ] **Step 2.6: Create `runIPCSimLoop.h/.cpp`**

Move the current frame loop into `runIPCSimLoop()`. It should call:

- `floorHeightAtFrame()` from setup.
- `RunIPCSimOutput::writeState()`.
- `RunIPCSimOutput::writeVonMisesStressJson()`.
- `RunIPCSimOutput::writeSurface()`.
- `logRunIPCSimMaxStepSummary()`.

Keep the surface scale division exactly:

```cpp
mesh.pos(vi) = psurf.segment<3>(vi * 3) / runtimeConfig.scale;
```

- [ ] **Step 2.7: Create `runIPCSimApp.h/.cpp`**

Implement:

```cpp
bool openRunIPCSimConfig(const std::filesystem::path &configPath, pgo::ConfigFileJSON &config)
{
  if (config.open(configPath.string().c_str()) != true)
    return false;
  return true;
}
```

Implement `buildIpcSimulation()`:

```cpp
const bool hasTetMesh = config.exist("tet-mesh");
const bool hasCubicMesh = config.exist("cubic-mesh");
const bool useVolumePath = hasTetMesh || hasCubicMesh;
return useVolumePath ? buildVolumeIpcSimulation(config) : buildShellIpcSimulation(config);
```

`runFromConfig()` must preserve `output-von-mises` validation before context build:

```cpp
if (runtimeConfig.outputVonMises && !config.exist("tet-mesh") && !config.exist("cubic-mesh"))
  throw std::invalid_argument("runIPCSim `output-von-mises` requires `tet-mesh` or `cubic-mesh`.");
```

It must set `tbb::global_control c(tbb::global_control::max_allowed_parallelism, 64);` at the top of the try block, matching current `main()`.

It must call `runScope.logProfileSummaryIfEnabled()` immediately after `runIPCSimLoop()` and before returning from the try block.

- [ ] **Step 2.8: Run tests with direct source inclusion**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
```

Expected: all `runIPCSim_gtest` tests pass.

### Task 3: Introduce `runIPCSimCore` library and thin executable

**Files:**
- Modify: `src/tools/runSim/CMakeLists.txt`
- Modify: `src/tools/runSim/runIPCSim.cpp`
- Create: `src/tools/runSim/runIPCSimCli.h`
- Create: `src/tools/runSim/runIPCSimCli.cpp`
- Modify: `tests/src/tools/CMakeLists.txt`

- [ ] **Step 3.1: Add `runIPCSimCore` to CMake**

Replace current line:

```cmake
add_libpgo_tools(runIPCSim "runIPCSim.cpp;runSimCliLogging.cpp;runSimVolumeMeshIO.cpp;runSimFEMSetup.cpp;runIPCSimSetup.cpp" "${AVAILABLE_LIBS}")
```

with:

```cmake
set(RUN_IPC_SIM_CORE_SOURCES
  runSimCliLogging.cpp
  runSimVolumeMeshIO.cpp
  runSimFEMSetup.cpp
  runIPCSimSetup.cpp
  runIPCSimApp.cpp
  runIPCSimConfig.cpp
  runIPCSimOutput.cpp
  runIPCSimLogging.cpp
  runIPCSimSession.cpp
  runIPCSimLoop.cpp
)

add_library(runIPCSimCore STATIC ${RUN_IPC_SIM_CORE_SOURCES})
target_link_libraries(runIPCSimCore PUBLIC ${AVAILABLE_LIBS})
target_link_libraries(runIPCSimCore PRIVATE compilation_flag)
target_include_directories(runIPCSimCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_property(TARGET runIPCSimCore PROPERTY FOLDER "tools/internal")

add_libpgo_tools(runIPCSim "runIPCSim.cpp;runIPCSimCli.cpp" "")
target_link_libraries(runIPCSim PRIVATE runIPCSimCore argparse::argparse)
```

Also add the existing debug compilation flag path for local consistency:

```cmake
if(PGO_RELEASE_MODE_DEBUG)
  target_link_libraries(runIPCSimCore PRIVATE compilation_flag_for_debug)
endif()
```

- [ ] **Step 3.2: Create `runIPCSimCli.h/.cpp`**

`runIPCSimCli.cpp` should contain the argparse code currently in `main()`:

```cpp
argparse::ArgumentParser program("Run IPC Simulation");
program.add_argument("config").help("Config File").required();
program.add_argument("--log")
  .help("Write command-line output to a .log file next to the config file")
  .default_value(false)
  .implicit_value(true);
```

`runCli()` catches parse errors and prints:

```cpp
std::cerr << err.what() << std::endl;
std::cerr << program;
return 1;
```

- [ ] **Step 3.3: Replace `runIPCSim.cpp` with thin main**

Final file:

```cpp
#include "runIPCSimCli.h"

int main(int argc, char *argv[])
{
  return pgo::RunIPCSim::runCli(argc, argv);
}
```

- [ ] **Step 3.4: Link tests to `runIPCSimCore`**

In `tests/src/tools/CMakeLists.txt`, remove direct core source list entries from `runIPCSim_gtest`:

```cmake
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runSimCliLogging.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runSimVolumeMeshIO.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runSimFEMSetup.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimSetup.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimApp.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimConfig.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimOutput.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimLogging.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimSession.cpp
  ${CMAKE_SOURCE_DIR}/src/tools/runSim/runIPCSimLoop.cpp
```

and add `runIPCSimCore` to `target_link_libraries(runIPCSim_gtest PRIVATE ...)`.

- [ ] **Step 3.5: Build and run**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
```

Expected: `runIPCSim_gtest` passes and executable still exists at `./build/base_no_mkl/bin/runIPCSim`.

### Task 4: Shrink and validate app orchestration

**Files:**
- Modify: `src/tools/runSim/runIPCSimApp.cpp`
- Modify: `src/tools/runSim/runIPCSimLogging.cpp`
- Modify: `src/tools/runSim/runIPCSimSession.cpp`
- Modify: `src/tools/runSim/runIPCSimLoop.cpp`
- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 4.1: Add failure-path direct runner test**

Add:

```cpp
TEST(RunIPCSimAppGTest, RunFromConfigReturnsFailureForMissingRequiredIPCFields)
{
  initializeRunIPCSimTestEnvironment();

  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "shell-runner-missing-ipc.json";
  writeTextFile(configPath, makeShellIPCConfig(tempDir.path(), 0, false));

  pgo::RunIPCSim::RunIPCSimOptions options;
  EXPECT_NE(pgo::RunIPCSim::runFromConfig(configPath, options), 0);
}
```

- [ ] **Step 4.2: Ensure app is pure orchestration**

Inspect `runIPCSimApp.cpp`. It should not contain:

- `ES::writeMatrix`
- `ES::readMatrix`
- `mesh.save`
- `argparse`
- `for (int framei`
- `computeVonMisesStresses`
- `surfacePressureForce`

If any appear, move them to `Output`, `Cli`, `Loop`, or `Session` respectively.

- [ ] **Step 4.3: Ensure session has no file/config responsibility**

Inspect `runIPCSimSession.cpp`. It should not contain:

- `ConfigFileJSON`
- `std::filesystem::remove_all`
- `mesh.save`
- `computeVonMisesStresses`

- [ ] **Step 4.4: Ensure loop has no config/CLI responsibility**

Inspect `runIPCSimLoop.cpp`. It should not contain:

- `ConfigFileJSON`
- `argparse`
- `std::filesystem::remove_all`
- `ScopedRunSimCliLogRedirect`

- [ ] **Step 4.5: Run focused verification**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest runSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runSim_gtest
```

Expected: both gtest executables pass.

### Task 5: Final cleanup and verification

**Files:**
- No source edits unless verification finds drift.

- [ ] **Step 5.1: Full focused build**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim runIPCSim_gtest runSim_gtest
```

Expected: all targets build.

- [ ] **Step 5.2: Run direct tests**

Run:

```bash
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runSim_gtest
```

Expected:

- `runIPCSim_gtest` passes all tests, including CLI smoke tests and new direct runner/config/output tests.
- `runSim_gtest` passes, proving shared `runSimCliLogging`, volume mesh IO and FEM setup were not broken.

- [ ] **Step 5.3: Hygiene**

Run:

```bash
git diff --check
git status --short
```

Expected:

- No whitespace errors.
- Changed files limited to:
  - `src/tools/runSim/CMakeLists.txt`
  - `src/tools/runSim/runIPCSim.cpp`
  - new `src/tools/runSim/runIPCSim*.h/.cpp`
  - `src/tools/runSim/runIPCSimSetup.h/.cpp`
  - `tests/src/tools/CMakeLists.txt`
  - `tests/src/tools/runIPCSim_gtest.cpp`

## 回滚策略

- 如果 Task 1 的 new tests 不能形成明确 compile failure，先修测试 include/source list，不进入实现。
- 如果 Task 2 后 direct source inclusion 仍无法通过 `runIPCSim_gtest`，不要进入 CMake library split；先检查搬移逻辑是否改变输出路径、日志顺序或 restart 行为。
- 如果 Task 3 后 executable 找不到，优先检查 `runIPCSimCore` link/include 和 `add_libpgo_tools()` 调用，不改 runtime code。
- 如果 Task 4 后 CLI smoke tests 失败，但 direct runner tests 通过，优先检查 `runIPCSimCli.cpp` argparse 行为。
- 如果 `runSim_gtest` 失败，检查 CMake link 是否误改 shared `runSim` helper，不要改 `runShellSim` 行为。

## 完成标准

- `runIPCSim.cpp` 只有 thin main。
- `runIPCSimApp.cpp` 是顶层编排，不含 frame-loop、file-output、argparse 细节。
- `runIPCSimConfig.*` 是唯一负责 runtime config scalar/options parsing 的模块。
- `runIPCSimOutput.*` 是唯一负责 output directories、state/surface/stress path 和写文件的模块。
- `runIPCSimSession.*` 是唯一负责 integrator/runtime vectors 初始化和 restart restore 的模块。
- `runIPCSimLoop.*` 是唯一负责 per-frame update/doTimestep/output dispatch 的模块。
- `runIPCSimCore` 被 `runIPCSim` executable 和 `runIPCSim_gtest` 共享。
- 现有 CLI、日志、输出路径、restart、profiling、IPC setup smoke tests 继续通过。

## 自审记录

- **Spec coverage:** 用户要求“入口尽可能清晰”，计划把 `main()` 降到 1 个调用，并把 app/config/output/logging/session/loop/setup 分为明确模块。
- **Repo truth:** 当前 `src/tools/runSim/CMakeLists.txt` 用 `add_libpgo_tools()` 直接构建 executable；计划改成 `runIPCSimCore` static library + thin executable，测试 target 链接同一 core，并按现有 `cubicMesherCore` 模式让 executable 显式链接 core 和 `argparse::argparse`。
- **Behavior preservation:** 计划明确锁住 CLI 参数、config 字段、日志文本、output path、restart scan、surface dump index、profiling lifecycle 和 frame loop 顺序。
- **Test gates:** 每轮都有 `runIPCSim_gtest`，最终加 `runSim_gtest` 验证 shared helpers。新增 direct tests 覆盖 runtime config、output path 和 `runFromConfig()`。
- **Risk:** 最大风险是 CMake link 依赖和日志初始化顺序；Task 3/4 已把它们单独 gate 出来。
- **Non-goals:** 不拆 shell/volume IPC setup 内部，不改 config schema，不碰 `runShellSim`，不改变 `IpcSimulationContext` 语义。
