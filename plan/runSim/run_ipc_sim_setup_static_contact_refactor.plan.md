# runIPCSim Setup Static Contact Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 继续模块化 `runIPCSim`，把 setup、static failure policy、legacy static 语义测试、contact 新旧边界整理成可验证的小步重构。

**Architecture:** 第一阶段先拆 `runIPCSimSetup.cpp`，不改变行为；第二阶段给 static solver 增加明确返回状态策略；第三阶段用 box-hang / drop 两类 legacy static fixture 固化语义；第四阶段只做 contact 文件分组和 CMake 边界整理，不重写 contact 算法。每阶段单独验证并提交，避免把机械移动和行为变化混在一起。

**Tech Stack:** C++17, CMake, GoogleTest, `runIPCSimCore`, `NonlinearOptimization::NewtonSolver`, legacy penalty contact handlers, IPC contact backend.

---

## 当前事实

- 当前分支：`codex/try_optimize_ipc`。
- 已有提交：
  - `5b41d5ac feat: migrate legacy volume contact to runIPCSim`
  - `1c85caff feat: support static mode in runIPCSim`
  - `f91c2501 fix: stabilize runIPCSim static solve`
- `src/tools/runSim/runIPCSimSetup.cpp` 约 834 行，当前混合了 path/config 解析、shell setup、volume IPC setup、legacy setup、attachment、floor、obstacle、surface pressure 等责任。
- `runIPCSimStaticSolve.cpp` 当前调用 `NewtonSolver::solve(...)`，但没有检查 solver return code；即使 line search failed，仍会写输出并让 `runFromConfig()` 返回 0。
- `runIPCSimLegacyPenaltyContact.cpp::addStaticEnergies()` 当前是 no-op，符合旧 `runSim static`：旧 static 分支不加入 penalty contact energy。
- legacy static 的正确语义需要区分：
  - box-hang：有 fixed/pulling constraint，static 有意义。
  - drop box：无 fixed，static penalty contact 未参与，系统无下界或病态，不应当被当作成功收敛例子。

## 非目标

- 不改变 IPC dynamic、legacy dynamic 的数值行为。
- 不在本轮实现 legacy penalty static contact energy。
- 不重写 IPC contact core、CCD、broad phase 或 barrier formulation。
- 不恢复旧 `runSim` / `runShellSim` executable。
- 不把所有 contact 源码一次性大迁移到新命名空间；第四阶段只做低风险文件分组和 include/CMake 更新。

## 总体验证命令

每个阶段至少运行该阶段的 focused tests；最终阶段运行：

```bash
git diff --check
cmake --build --preset base_no_mkl_release --target runIPCSim runIPCSim_gtest runSimShared_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runSimShared_gtest
```

Expected:

- build exit code 0。
- `runIPCSim_gtest`: all tests pass。
- `runSimShared_gtest`: all tests pass。
- `git diff --check`: no output, exit code 0。

---

## 阶段 1：拆分 `runIPCSimSetup.cpp`

**目标:** 将 setup 的职责拆成小文件，保持 public API 和行为不变。

**风险:** 中等。主要风险是机械移动时漏 include、匿名 namespace helper 可见性变化、CMake source list 漏文件。

**Commit:** `refactor: split runIPCSim setup modules`

### 目标文件布局

新增：

- `src/tools/runSim/runIPCSimAttachmentSetup.h`
- `src/tools/runSim/runIPCSimAttachmentSetup.cpp`
- `src/tools/runSim/runIPCSimObstacleSetup.h`
- `src/tools/runSim/runIPCSimObstacleSetup.cpp`
- `src/tools/runSim/runIPCSimSurfacePressureSetup.h`
- `src/tools/runSim/runIPCSimSurfacePressureSetup.cpp`
- `src/tools/runSim/runIPCSimFloorSetup.h`
- `src/tools/runSim/runIPCSimFloorSetup.cpp`
- `src/tools/runSim/runIPCSimVolumeSetup.h`
- `src/tools/runSim/runIPCSimVolumeSetup.cpp`
- `src/tools/runSim/runIPCSimShellSetup.h`
- `src/tools/runSim/runIPCSimShellSetup.cpp`
- `src/tools/runSim/runIPCSimLegacySetup.h`
- `src/tools/runSim/runIPCSimLegacySetup.cpp`
- `src/tools/runSim/runIPCSimSetupCommon.h`
- `src/tools/runSim/runIPCSimSetupCommon.cpp`

保留：

- `src/tools/runSim/runIPCSimSetup.h`
  - 继续暴露 `IpcSimulationContext`、`IpcFloorMotionState` 等需要被 loop/backend/output 使用的类型。
  - 继续声明：
    - `IpcSimulationContext buildShellIpcSimulation(const ConfigFileJSON &jconfig);`
    - `IpcSimulationContext buildVolumeIpcSimulation(const ConfigFileJSON &jconfig);`
    - `IpcSimulationContext buildVolumeLegacyPenaltySimulation(const ConfigFileJSON &jconfig);`
- `src/tools/runSim/runIPCSimSetup.cpp`
  - 最终只保留薄 wrapper 或删除。如果删除，需要同步 `CMakeLists.txt`。

修改：

- `src/tools/runSim/CMakeLists.txt`
- `tests/src/tools/runIPCSim_gtest.cpp`，只在必要时更新 include 或 test helper。

### 拆分边界

#### `runIPCSimSetupCommon.*`

放置无业务 backend 偏向的通用 helper：

- `rejectIfPresent(...)`
- `makeIdentityEmbedding(int n3)`
- `validateZeroInitialDisplacement(...)`
- `loadSurfaceMeshAndRestPositions(...)`
- `parseVolumeElasticMaterial(...)`
- `parseEnableMaterialMaxStep(...)`

Header 内容：

```cpp
#pragma once

#include "EigenSupport.h"
#include "configFileJSON.h"
#include "deformationModel.h"
#include "triMeshGeo.h"

#include <string>

namespace pgo::RunIPCSim
{
void rejectIfPresent(const ConfigFileJSON &config, const char *field, const char *reason);
EigenSupport::SpMatD makeIdentityEmbedding(int n3);
void validateZeroInitialDisplacement(const ConfigFileJSON &config);
void loadSurfaceMeshAndRestPositions(const std::string &surfaceMeshFilename, double scale,
  Mesh::TriMeshGeo &surfaceMesh, EigenSupport::VXd &surfaceRestPositions);
SolidDeformationModel::DeformationModelElasticMaterial parseVolumeElasticMaterial(const ConfigFileJSON &config);
bool parseEnableMaterialMaxStep(const ConfigFileJSON &config);
}  // namespace pgo::RunIPCSim
```

#### `runIPCSimAttachmentSetup.*`

放置 fixed vertices / pulling constraints：

- `buildPullingConstraints(...)`

Header 内容：

```cpp
#pragma once

#include "EigenSupport.h"
#include "configFileJSON.h"
#include "multiVertexPullingSoftConstraints.h"

#include <memory>
#include <string>
#include <vector>

namespace pgo::RunIPCSim
{
void buildPullingConstraints(const ConfigFileJSON &config,
  const std::vector<std::string> &fixedVertexFilenames,
  const EigenSupport::VXd &simulationRestPosition,
  const EigenSupport::SpMatD &K,
  std::vector<std::shared_ptr<ConstraintPotentialEnergies::MultipleVertexPulling>> &pullingEnergies,
  std::vector<EigenSupport::VXd> &pullingTargets,
  std::vector<EigenSupport::VXd> &pullingTargetRests);
}  // namespace pgo::RunIPCSim
```

#### `runIPCSimFloorSetup.*`

放置 floor config 和 motion：

- `ParsedFloorConfig`
- `parseFloorsConfig(...)`
- `floorHeightAtFrame(...)`
- `parseFloorAxis(...)`
- `parseFloorSide(...)`
- string conversion helpers

注意：如果 `ParsedFloorConfig` 当前只在 setup 内部使用，可以只放 `.cpp`，并用一个 public factory 函数返回 floor potential/motion。若 IPC backend 仍需要 `IpcFloorMotionState`，该类型保留在 `runIPCSimSetup.h`。

建议 public API：

```cpp
std::vector<ParsedFloorConfig> parseFloorsConfig(const ConfigFileJSON &config);
double floorHeightAtFrame(const IpcFloorMotionState &motion, int frame);
```

#### `runIPCSimObstacleSetup.*`

放置 external IPC obstacle parsing：

- `parseExternalObjects(...)`

Header 内容：

```cpp
#pragma once

#include "surfaceIPCCore.h"
#include "configFileJSON.h"

#include <vector>

namespace pgo::RunIPCSim
{
std::vector<Contact::CIPC::ObstacleSurface> parseExternalObjects(
  const ConfigFileJSON &config, double scale, std::vector<bool> *staticFlags);
}  // namespace pgo::RunIPCSim
```

#### `runIPCSimSurfacePressureSetup.*`

放置 surface pressure config 和 force 计算：

- `ParsedSurfacePressureForceConfig`
- `parseSurfacePressureForceConfig(...)`
- `computeSurfacePressureSimulationForce(...)`

#### `runIPCSimVolumeSetup.*`

放置 volume IPC setup：

- `makeVolumeIPCParams(...)`
- `buildVolumeIpcSimulation(...)`

`buildVolumeIpcSimulation(...)` 的签名保持不变，并继续被 `runIPCSimApp.cpp` 间接调用。

#### `runIPCSimShellSetup.*`

放置 shell IPC setup：

- `buildShellIpcSimulation(...)`
- shell-only material validation / mass setup / identity embedding setup。

#### `runIPCSimLegacySetup.*`

放置 legacy volume setup：

- `buildVolumeLegacyPenaltySimulation(...)`
- legacy-only validation。

### Task 1.1: 建立空文件和 CMake wiring

- [ ] **Step 1: 创建 header/source 文件**

Create the files listed above with namespace skeleton and includes only.

- [ ] **Step 2: 更新 `src/tools/runSim/CMakeLists.txt`**

Add new `.cpp` files to `RUN_IPC_SIM_CORE_SOURCES`:

```cmake
  runIPCSimSetupCommon.cpp
  runIPCSimAttachmentSetup.cpp
  runIPCSimFloorSetup.cpp
  runIPCSimObstacleSetup.cpp
  runIPCSimSurfacePressureSetup.cpp
  runIPCSimShellSetup.cpp
  runIPCSimVolumeSetup.cpp
  runIPCSimLegacySetup.cpp
```

Keep `runIPCSimSetup.cpp` temporarily in the source list until all code is moved.

- [ ] **Step 3: Build to verify empty files are harmless**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSimCore
```

Expected: build succeeds.

### Task 1.2: Move common helpers

- [ ] **Step 1: Move helper declarations to `runIPCSimSetupCommon.h`**

Move declarations for:

- `rejectIfPresent`
- `makeIdentityEmbedding`
- `validateZeroInitialDisplacement`
- `loadSurfaceMeshAndRestPositions`
- `parseVolumeElasticMaterial`
- `parseEnableMaterialMaxStep`

- [ ] **Step 2: Move implementations to `runIPCSimSetupCommon.cpp`**

Keep function bodies identical. Do not change error strings.

- [ ] **Step 3: Include `runIPCSimSetupCommon.h` from old `runIPCSimSetup.cpp`**

Remove duplicate definitions from old file.

- [ ] **Step 4: Build**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSimCore
```

Expected: build succeeds.

### Task 1.3: Move attachment setup

- [ ] **Step 1: Move `buildPullingConstraints(...)`**

Move implementation unchanged to `runIPCSimAttachmentSetup.cpp`.

- [ ] **Step 2: Include new header from shell/volume/legacy setup callers**

For temporary state, include it in `runIPCSimSetup.cpp`. After `runIPCSimVolumeSetup.cpp` and others are moved, include it there.

- [ ] **Step 3: Run focused tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimStaticGTest.*:RunIPCSimCliGTest.StaticTetCliSmokeWritesUnifiedSurfaceAndState'
```

Expected: all selected tests pass.

### Task 1.4: Move floor, obstacle, and surface pressure setup

- [ ] **Step 1: Move floor parsing/motion helpers**

Move:

- `ParsedFloorConfig`
- `parseFloorAxis`
- `floorAxisToString`
- `parseFloorSide`
- `floorSideToString`
- `parseFloorsConfig`
- `floorHeightAtFrame`

If `ParsedFloorConfig` is needed by `buildVolumeIpcSimulation(...)`, expose it from `runIPCSimFloorSetup.h`.

- [ ] **Step 2: Move surface pressure helpers**

Move:

- `ParsedSurfacePressureForceConfig`
- `parseSurfacePressureForceConfig`
- `computeSurfacePressureSimulationForce`

- [ ] **Step 3: Move obstacle helper**

Move:

- `parseExternalObjects(...)`

- [ ] **Step 4: Run focused tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimSetupGTest.*:RunIPCSimCliGTest.*Floor*:RunIPCSimCliGTest.*SurfacePressure*:RunIPCSimCliGTest.TetMovingUpperFloorSmokeWritesStress'
```

Expected: all selected tests pass.

### Task 1.5: Move shell / volume / legacy setup builders

- [ ] **Step 1: Move `buildShellIpcSimulation(...)`**

Move implementation to `runIPCSimShellSetup.cpp`.

Header:

```cpp
#pragma once

#include "runIPCSimSetup.h"
#include "configFileJSON.h"

namespace pgo::RunIPCSim
{
IpcSimulationContext buildShellIpcSimulation(const ConfigFileJSON &config);
}  // namespace pgo::RunIPCSim
```

- [ ] **Step 2: Move `buildVolumeIpcSimulation(...)`**

Move implementation to `runIPCSimVolumeSetup.cpp`.

- [ ] **Step 3: Move `buildVolumeLegacyPenaltySimulation(...)`**

Move implementation to `runIPCSimLegacySetup.cpp`.

- [ ] **Step 4: Make `runIPCSimSetup.cpp` empty or remove it**

Preferred: remove `runIPCSimSetup.cpp` from `RUN_IPC_SIM_CORE_SOURCES` after all functions are moved. Keep `runIPCSimSetup.h` as shared context/types header.

- [ ] **Step 5: Build and run full runIPCSim tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

Run:

```bash
git add src/tools/runSim tests/src/tools/runIPCSim_gtest.cpp
git commit -m "refactor: split runIPCSim setup modules"
```

Expected: commit succeeds.

---

## 阶段 2：static solver 返回状态策略

**目标:** static solve 不再无条件把 `LineSearchFailed` / `NonFinite` 当成功。让“可接受失败”和“真实失败”有明确 policy。

**风险:** 中等。会改变 static CLI 对 ill-posed case 的 exit code，需要保留已验证可行 static cases。

**Commit:** `fix: enforce runIPCSim static solver status`

### 行为决策

1. 默认 strict：
   - `Converged`: success。
   - `LineSearchFailed`, `StepTooSmall`, `NonFinite`, `MaxIterations`, `LinearSolveFailed`: failure，`runFromConfig()` 返回 1。
2. 不新增 config flag。先保持简单，避免把“失败是否可接受”变成用户配置。
3. 写输出规则：
   - solver failure 时不写 `states/deform0000.u` / `surface/ret0000.obj`。
   - 先检查 ret，再调用 output。
4. 日志：
   - failure 抛 `std::runtime_error("runIPCSim static solve failed: <status>")`。
   - 使用 `NewtonSolver::solveStatusToString(ret)`。

### Task 2.1: 写失败测试

**Files:**

- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: 增加 no-fixed legacy/static/drop 失败测试**

Add:

```cpp
TEST(RunIPCSimStaticGTest, StaticLegacyDropWithoutAttachmentFails)
{
  initializeRunIPCSimTestEnvironment();

  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "legacy-static-drop.json";
  const fs::path outputDir = tempDir.path() / "legacy-static-drop-output";
  writeTextFile(configPath, makeStaticConfig(makeLegacyVolumeConfig(
    fs::path(kLegacyTetBoxDir) / "box.veg",
    fs::path(kLegacyTetBoxDir) / "box.obj",
    outputDir, "tet-mesh", "stable-neo", 1)));

  pgo::RunIPCSim::RunIPCSimOptions options;
  options.contactBackendKind = pgo::RunIPCSim::ContactBackendKind::LegacyPenalty;
  EXPECT_NE(pgo::RunIPCSim::runFromConfig(configPath, options), 0);
  EXPECT_FALSE(fs::exists(surfacePath(outputDir, 0)));
  EXPECT_FALSE(fs::exists(statePath(outputDir, 0)));
}
```

If `makeLegacyVolumeConfig(...)` currently always adds fixed vertices, create a dedicated helper that emits `"fixed-vertices": []`.

- [ ] **Step 2: Verify test fails before implementation**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter=RunIPCSimStaticGTest.StaticLegacyDropWithoutAttachmentFails
```

Expected before implementation: test fails because current code returns 0 and writes output.

### Task 2.2: Enforce solver ret in static solve

**Files:**

- Modify: `src/tools/runSim/runIPCSimStaticSolve.cpp`

- [ ] **Step 1: Capture solver ret**

Change:

```cpp
solver.solve(u.data(), runtimeConfig.solverMaxIter, runtimeConfig.solverEps, 2);
```

to:

```cpp
const int solverRet = solver.solve(u.data(), runtimeConfig.solverMaxIter, runtimeConfig.solverEps, 2);
if (solverRet != static_cast<int>(NonlinearOptimization::NewtonSolver::SolveStatus::Converged)) {
  throw std::runtime_error(std::string("runIPCSim static solve failed: ") +
    NonlinearOptimization::NewtonSolver::solveStatusToString(solverRet));
}
```

Add `<string>` include if needed.

- [ ] **Step 2: Run focused tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimStaticGTest.*'
```

Expected: static tests pass after updating tests that assumed failure still writes output.

### Task 2.3: Add CLI failure coverage

- [ ] **Step 1: Add CLI test for static failure exit code**

Add:

```cpp
TEST(RunIPCSimCliGTest, StaticFailureReturnsNonZeroAndDoesNotWriteOutput)
{
  const fs::path binary = runIPCSimBinaryPath();
  ASSERT_FALSE(binary.empty());
  ASSERT_TRUE(fs::exists(binary));

  ScopedTempDir tempDir;
  const fs::path configPath = tempDir.path() / "legacy-static-drop-cli.json";
  const fs::path outputDir = tempDir.path() / "legacy-static-drop-cli-output";
  writeTextFile(configPath, makeStaticConfig(makeLegacyVolumeConfig(
    fs::path(kLegacyTetBoxDir) / "box.veg",
    fs::path(kLegacyTetBoxDir) / "box.obj",
    outputDir, "tet-mesh", "stable-neo", 1)));

  std::ostringstream command;
  command << shellExecutable(binary)
          << " --legacy "
          << quotePath(configPath);

  ASSERT_NE(runCommand(command.str()), 0);
  EXPECT_FALSE(fs::exists(surfacePath(outputDir, 0)));
  EXPECT_FALSE(fs::exists(statePath(outputDir, 0)));
}
```

- [ ] **Step 2: Run focused CLI tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimCliGTest.Static*:RunIPCSimStaticGTest.*'
```

Expected: selected tests pass.

- [ ] **Step 3: Commit**

Run:

```bash
git add src/tools/runSim/runIPCSimStaticSolve.cpp tests/src/tools/runIPCSim_gtest.cpp
git commit -m "fix: enforce runIPCSim static solver status"
```

Expected: commit succeeds.

---

## 阶段 3：legacy static 语义测试与 fixture 整理

**目标:** 用测试固化 “legacy static box-hang 可行，legacy static drop 不应成功” 的语义，避免再次把两类 case 混淆。

**风险:** 低到中。主要是测试 helper 构造旧 JSON 时路径 resolution 容易错。

**Commit:** `test: cover legacy static hang and drop semantics`

### 行为定义

- `--legacy + sim-type=static + fixed-vertices`:
  - 允许。
  - 不加入 penalty contact。
  - 只要 Newton converges，返回 0 并写 output。
- `--legacy + sim-type=static + fixed-vertices=[] + external-objects`:
  - 当前不支持作为成功 static case。
  - 在阶段 2 strict policy 下，如果 solver 不收敛，返回非 0，不写 output。
- 文档中明确：`--legacy` 表示旧 penalty backend，不表示 static contact energy 已实现。

### Task 3.1: 添加 legacy box-hang config helper

**Files:**

- Modify: `tests/src/tools/runIPCSim_gtest.cpp`

- [ ] **Step 1: Add helper**

Add near legacy config helpers:

```cpp
std::string makeLegacyBoxHangConfig(const fs::path &volumeMesh,
  const fs::path &surfaceMesh,
  const fs::path &fixedVertexFile,
  const fs::path &outputDir,
  const std::string &meshKey,
  const std::string &elasticMaterial)
{
  std::ostringstream json;
  json << "{\n"
       << "  \"" << meshKey << "\": " << quotePath(volumeMesh) << ",\n"
       << "  \"surface-mesh\": " << quotePath(surfaceMesh) << ",\n"
       << "  \"fixed-vertices\": [\n"
       << "    {\n"
       << "      \"filename\": " << quotePath(fixedVertexFile) << ",\n"
       << "      \"movement\": [0, 0, 0],\n"
       << "      \"coeff\": 10000.0\n"
       << "    }\n"
       << "  ],\n"
       << "  \"g\": [0, -9.81, 0],\n"
       << "  \"init-vel\": [0, 0, 0],\n"
       << "  \"init-disp\": [0, 0, 0],\n"
       << "  \"scale\": 1.0,\n"
       << "  \"timestep\": 0.001,\n"
       << "  \"num-timestep\": 2000,\n"
       << "  \"damping-params\": [0, 0],\n"
       << "  \"sim-type\": \"static\",\n"
       << "  \"contact-stiffness\": 1000,\n"
       << "  \"contact-sample\": 6,\n"
       << "  \"contact-friction-coeff\": 0.0,\n"
       << "  \"contact-vel-eps\": 1e-5,\n"
       << "  \"solver-eps\": 1e-6,\n"
       << "  \"solver-max-iter\": 500,\n"
       << "  \"elastic-material\": \"" << elasticMaterial << "\",\n"
       << "  \"dump-interval\": 10,\n"
       << "  \"output\": " << quotePath(outputDir) << "\n"
       << "}\n";
  return json.str();
}
```

### Task 3.2: Add focused legacy static hang tests

- [ ] **Step 1: Test tet hang**

Add:

```cpp
TEST(RunIPCSimStaticGTest, StaticLegacyTetBoxHangConvergesWithAttachment)
{
  initializeRunIPCSimTestEnvironment();

  ScopedTempDir tempDir;
  const fs::path fixedFile = tempDir.path() / "box-fixed.txt";
  writeTextFile(fixedFile, "0\n");

  const fs::path configPath = tempDir.path() / "legacy-tet-box-hang-static.json";
  const fs::path outputDir = tempDir.path() / "legacy-tet-box-hang-output";
  writeTextFile(configPath, makeLegacyBoxHangConfig(
    fs::path(kLegacyTetBoxDir) / "box.veg",
    fs::path(kLegacyTetBoxDir) / "box.obj",
    fixedFile,
    outputDir,
    "tet-mesh",
    "stable-neo"));

  pgo::RunIPCSim::RunIPCSimOptions options;
  options.contactBackendKind = pgo::RunIPCSim::ContactBackendKind::LegacyPenalty;
  EXPECT_EQ(pgo::RunIPCSim::runFromConfig(configPath, options), 0);
  EXPECT_TRUE(fs::exists(surfacePath(outputDir, 0)));
  EXPECT_TRUE(fs::exists(statePath(outputDir, 0)));
}
```

- [ ] **Step 2: Test cubic hang**

Add the same test with:

- `kLegacyCubicBoxDir`
- `"cubic-mesh"`
- output folder `legacy-cubic-box-hang-output`

- [ ] **Step 3: Keep drop failure test from Phase 2**

Ensure its name and assertion make the contrast clear:

```cpp
TEST(RunIPCSimStaticGTest, StaticLegacyDropWithoutAttachmentFails)
```

### Task 3.3: Document semantics in IPC README

**Files:**

- Modify: `examples/ipc/README.md`

- [ ] **Step 1: Add static mode note**

Add a paragraph near static mode docs:

```markdown
`sim-type = static` solves a one-shot equilibrium problem. It is not equivalent to
running a dynamic drop until it settles. A static case must be well-posed on its own:
for example, `box-hang` uses `fixed-vertices` as an attachment. Legacy `--legacy`
static mode preserves old `runSim static` behavior and does not add penalty contact
energies; drop-style legacy configs without attachments are not valid static
equilibrium examples.
```

### Task 3.4: Run tests and commit

- [ ] **Step 1: Focused tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest --gtest_filter='RunIPCSimStaticGTest.StaticLegacy*:RunIPCSimCliGTest.Static*'
```

Expected: selected tests pass.

- [ ] **Step 2: Commit**

Run:

```bash
git add tests/src/tools/runIPCSim_gtest.cpp examples/ipc/README.md
git commit -m "test: cover legacy static hang and drop semantics"
```

Expected: commit succeeds.

---

## 阶段 4：整理 contact 新旧模块边界

**目标:** 把 legacy penalty contact 和 IPC contact 的文件布局/target 边界变清晰，降低后续继续重构的认知成本。

**风险:** 中等偏高。文件移动会影响 include、CMake、tests、install/IDE grouping。禁止在本阶段改 contact 算法。

**Commit:** `refactor: separate legacy and ipc contact modules`

### 目标目录布局

当前 `src/core/contact` 混合旧 penalty 和 IPC 文件。目标：

```text
src/core/contact/
  common/
    contactEnergyUtilities.*
    CCDKernel.*
    CIPC.*                # 如果仍被旧/新共享，暂留 common；否则后续再移
  legacy_penalty/
    pointPenetrationEnergy.*
    pointTrianglePairCouplingEnergyWithCollision.*
    triangleMeshExternalContactHandler.*
    triangleMeshSelfContactDetection.*
    triangleMeshSelfContactHandler.*
  ipc/
    core/...
    embeddedSurfaceIPCPotentialEnergy.*
    embeddedSurfaceFloorPotentialEnergy.*
    mappedSurfacePotentialEnergy.*
```

实际迁移规则：

- 先移动明显只被 legacy backend 使用的 files 到 `legacy_penalty/`。
- IPC `ipc/core/*` 已经有子目录，不做大改。
- `embeddedSurfaceIPCPotentialEnergy.*` 和 `embeddedSurfaceFloorPotentialEnergy.*` 移到 `ipc/`。
- `mappedSurfacePotentialEnergy.*` 如果只被 IPC embedded surface 使用，也移到 `ipc/`；如果被旧 penalty 或 simulation helper 共享，暂留在 `src/core/contact/` root，并在阶段 4 的 commit message 中记录“kept shared mapped surface potential at contact root”。
- Root `src/core/contact/CMakeLists.txt` 继续定义同一个 `contact` target，不拆 library target。先整理路径，避免大面积 link target 变更。

### Task 4.1: Map includes before moving

- [ ] **Step 1: Generate include usage map**

Run:

```bash
rg -n '#include "(pointPenetrationEnergy|pointTrianglePairCouplingEnergyWithCollision|triangleMeshExternalContactHandler|triangleMeshSelfContactDetection|triangleMeshSelfContactHandler|embeddedSurfaceIPC|embeddedSurfaceFloor|mappedSurfacePotentialEnergy)' src tests
```

Save output manually in the implementation notes or commit message.

- [ ] **Step 2: Decide file moves**

Only move a file if all includes can be updated mechanically. If a file is included by both legacy and IPC paths, leave it in root/common for this phase.

### Task 4.2: Move legacy penalty files

- [ ] **Step 1: Create directory**

Run:

```bash
mkdir -p src/core/contact/legacy_penalty
```

- [ ] **Step 2: Move files with `git mv`**

Run:

```bash
git mv src/core/contact/pointPenetrationEnergy.h src/core/contact/legacy_penalty/pointPenetrationEnergy.h
git mv src/core/contact/pointPenetrationEnergy.cpp src/core/contact/legacy_penalty/pointPenetrationEnergy.cpp
git mv src/core/contact/pointTrianglePairCouplingEnergyWithCollision.h src/core/contact/legacy_penalty/pointTrianglePairCouplingEnergyWithCollision.h
git mv src/core/contact/pointTrianglePairCouplingEnergyWithCollision.cpp src/core/contact/legacy_penalty/pointTrianglePairCouplingEnergyWithCollision.cpp
git mv src/core/contact/triangleMeshExternalContactHandler.h src/core/contact/legacy_penalty/triangleMeshExternalContactHandler.h
git mv src/core/contact/triangleMeshExternalContactHandler.cpp src/core/contact/legacy_penalty/triangleMeshExternalContactHandler.cpp
git mv src/core/contact/triangleMeshSelfContactDetection.h src/core/contact/legacy_penalty/triangleMeshSelfContactDetection.h
git mv src/core/contact/triangleMeshSelfContactDetection.cpp src/core/contact/legacy_penalty/triangleMeshSelfContactDetection.cpp
git mv src/core/contact/triangleMeshSelfContactHandler.h src/core/contact/legacy_penalty/triangleMeshSelfContactHandler.h
git mv src/core/contact/triangleMeshSelfContactHandler.cpp src/core/contact/legacy_penalty/triangleMeshSelfContactHandler.cpp
```

- [ ] **Step 3: Update `src/core/contact/CMakeLists.txt`**

Replace source/header paths with `legacy_penalty/...`.

- [ ] **Step 4: Update includes**

Run:

```bash
rg -l '#include "(pointPenetrationEnergy|pointTrianglePairCouplingEnergyWithCollision|triangleMeshExternalContactHandler|triangleMeshSelfContactDetection|triangleMeshSelfContactHandler)' src tests \
  | xargs perl -0pi -e 's/#include "pointPenetrationEnergy.h"/#include "legacy_penalty\\/pointPenetrationEnergy.h"/g; s/#include "pointTrianglePairCouplingEnergyWithCollision.h"/#include "legacy_penalty\\/pointTrianglePairCouplingEnergyWithCollision.h"/g; s/#include "triangleMeshExternalContactHandler.h"/#include "legacy_penalty\\/triangleMeshExternalContactHandler.h"/g; s/#include "triangleMeshSelfContactDetection.h"/#include "legacy_penalty\\/triangleMeshSelfContactDetection.h"/g; s/#include "triangleMeshSelfContactHandler.h"/#include "legacy_penalty\\/triangleMeshSelfContactHandler.h"/g'
```

If the command matches zero files, inspect includes manually with `rg`.

- [ ] **Step 5: Build**

Run:

```bash
cmake --build --preset base_no_mkl_release --target contact runIPCSim_gtest runSimShared_gtest
```

Expected: build succeeds.

### Task 4.3: Move IPC wrapper files

- [ ] **Step 1: Move files**

Run:

```bash
git mv src/core/contact/embeddedSurfaceIPCPotentialEnergy.h src/core/contact/ipc/embeddedSurfaceIPCPotentialEnergy.h
git mv src/core/contact/embeddedSurfaceIPCPotentialEnergy.cpp src/core/contact/ipc/embeddedSurfaceIPCPotentialEnergy.cpp
git mv src/core/contact/embeddedSurfaceFloorPotentialEnergy.h src/core/contact/ipc/embeddedSurfaceFloorPotentialEnergy.h
git mv src/core/contact/embeddedSurfaceFloorPotentialEnergy.cpp src/core/contact/ipc/embeddedSurfaceFloorPotentialEnergy.cpp
```

Move `mappedSurfacePotentialEnergy.*` only if include map shows it is IPC-only:

```bash
git mv src/core/contact/mappedSurfacePotentialEnergy.h src/core/contact/ipc/mappedSurfacePotentialEnergy.h
git mv src/core/contact/mappedSurfacePotentialEnergy.cpp src/core/contact/ipc/mappedSurfacePotentialEnergy.cpp
```

- [ ] **Step 2: Update CMake and includes**

Update `src/core/contact/CMakeLists.txt`.

Run:

```bash
rg -l '#include "(embeddedSurfaceIPCPotentialEnergy|embeddedSurfaceFloorPotentialEnergy|mappedSurfacePotentialEnergy)' src tests \
  | xargs perl -0pi -e 's/#include "embeddedSurfaceIPCPotentialEnergy.h"/#include "ipc\\/embeddedSurfaceIPCPotentialEnergy.h"/g; s/#include "embeddedSurfaceFloorPotentialEnergy.h"/#include "ipc\\/embeddedSurfaceFloorPotentialEnergy.h"/g; s/#include "mappedSurfacePotentialEnergy.h"/#include "ipc\\/mappedSurfacePotentialEnergy.h"/g'
```

- [ ] **Step 3: Build**

Run:

```bash
cmake --build --preset base_no_mkl_release --target contact runIPCSim_gtest
```

Expected: build succeeds.

### Task 4.4: Final validation and commit

- [ ] **Step 1: Full validation**

Run:

```bash
git diff --check
cmake --build --preset base_no_mkl_release --target runIPCSim runIPCSim_gtest runSimShared_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
./build/base_no_mkl/tests/src/tools/runSimShared_gtest
```

Expected: all pass.

- [ ] **Step 2: Commit**

Run:

```bash
git add src/core/contact src/tools/runSim tests
git commit -m "refactor: separate legacy and ipc contact modules"
```

Expected: commit succeeds.

---

## 执行顺序与停止条件

1. 完成阶段 1 后必须 full `runIPCSim_gtest`。如果行为变了，停止并回查机械移动。
2. 阶段 2 是行为变化，必须先有失败测试，再改 implementation。
3. 阶段 3 只加语义测试和文档，不应改 solver/contact 行为。
4. 阶段 4 只移动文件和 include，不应改函数体逻辑。
5. 任一阶段遇到以下情况必须暂停：
   - static IPC box-hang/dragon 开始失败。
   - legacy dynamic smoke 开始失败。
   - 文件移动导致 public include path 需要下游大面积调整。
   - `runIPCSimSetup.cpp` 拆分后出现重复 helper 或循环 include。

## 自审结果

### 发现 1：阶段 2 strict policy 可能让既有 `StaticLegacyTetWritesUnifiedSurfaceAndState` 测试失效

**风险:** 当前已有 legacy static 测试如果使用无 fixed 的 legacy fixture，会在 strict policy 后失败。

**修正:** 阶段 2 的测试更新必须同步检查现有 legacy static tests。旧的“无 fixed legacy static writes output”测试应改名并调整为 failure test；真正的 success test 放在阶段 3 的 `StaticLegacyTetBoxHangConvergesWithAttachment` / `StaticLegacyCubicBoxHangConvergesWithAttachment`。

### 发现 2：阶段 4 一次性移动 `mappedSurfacePotentialEnergy.*` 可能过度

**风险:** 如果 `mappedSurfacePotentialEnergy.*` 被旧 penalty 或其他 core tests 间接依赖，移动到 `ipc/` 会引入不必要 include churn。

**修正:** 阶段 4 明确要求先跑 include usage map；`mappedSurfacePotentialEnergy.*` 只有在 IPC-only 时才移动，否则留在 root 或后续单独处理。

### 发现 3：阶段 1 拆分过细，可能导致实现者先花太多时间处理 include

**风险:** 过细拆分会让第一阶段变成 include 体力活，增加 merge conflict 面。

**修正:** 阶段 1 允许先保留 `runIPCSimSetup.cpp` 到所有函数都迁移完，再删除；每个子步骤都有 build checkpoint。若执行中发现拆分成本过高，优先完成 `Attachment/Floor/Obstacle/SurfacePressure` 四个 helper 拆分，`Shell/Volume/Legacy` builder 可作为同阶段后半段继续。

### 发现 4：阶段 2 不新增 config flag 是刻意选择，但需要承认兼容性变化

**风险:** 以前 static line-search failure 返回 0 并写 output；strict policy 会改变 CLI 行为。

**修正:** 计划中将该阶段独立成单独 commit，并要求新增 failure tests 和 README 说明。若用户之后要求兼容旧 permissive 行为，再另加显式 config/CLI flag，不能静默保持旧行为。

### 发现 5：计划没有要求真实 dragon/box-hang 手工 smoke

**风险:** 单测覆盖较小，不能证明真实大例子仍能跑。

**修正:** 阶段 2 或最终验证后，额外运行至少一个真实 static example：

```bash
# 临时复制 examples/ipc/tet/box-hang/box-ipc.json，改 sim-type=static 和 output=/tmp/...
./build/base_no_mkl/bin/runIPCSim <temp-static-box-hang.json>
```

如果时间允许，也运行 tet/cubic dragon static；这两个耗时较长，不作为每个阶段强制命令，但作为最终 release confidence check。
