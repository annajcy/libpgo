# Newton Solver Failure Handling 与 Adaptive Substep Plan

> **For implementers:** 这份计划修复 `NewtonSolver` 把未收敛状态报告为成功、time integrator 静默接受失败步的问题，并在 `runIPCSim` 层增加可选的 nonlinear-solve-driven adaptive substep。实施时按 phase 顺序推进，每个 phase 都要先写失败测试，再改实现。

## 1. 背景与问题定义

当前 `runIPCSim` shell IPC 示例中，日志出现大量：

```text
lineSearchAlpha=7.888609052210118e-31
acceptedStepMaxNorm=...
Iter=...; dx = ...; dx too small.
Txxxx: ||g||=...; Solver Ret: 0
```

其中有些 timestep 的最终残差明显大于 `solver-eps`，但 `NewtonSolver::solve()` 仍返回 `0`，`ImplicitBackwardEulerTimeIntegrator::doTimestep()` 随后无条件把 `z` 写入 `q1/qvel1/qacc1` 并 `proceedTimestep()`。这会让一个未满足隐式步平衡方程的状态继续进入后续仿真，并污染速度：

```text
qvel1 = (z - q) / h
qacc1 = (qvel1 - qvel) / h
```

这不一定马上代表 IPC 几何已经非法，但它意味着当前 timestep 没有被正确 accept/reject。

## 2. 目标

完成后应满足：

1. `NewtonSolver` 能区分收敛、最大迭代、line search failed、step too small、非有限数值等状态；
2. `dx too small` 只有在当前 residual 已低于 `epsilon` 时才算成功；
3. `ImplicitBackwardEulerTimeIntegrator` 不再静默接受非零 solver return；
4. `runIPCSim` 可选择开启 adaptive substep：solver 失败时拒绝当前 substep，恢复旧状态，减小 timestep 重试；
5. 默认行为保持保守：核心 integrator 失败时 fail-fast，`runIPCSim` 通过显式 config 才启用 adaptive retry；
6. 日志能清楚说明 timestep 是 accepted、rejected、retried，还是最终 failed。

## 3. 非目标

本计划不解决以下问题：

- 不改变 IPC barrier 数学；
- 不改变 shell case 的固定点设置、材料参数、阻尼参数；
- 不把所有 integrator 统一重构为 adaptive 框架；
- 不实现基于局部误差估计的精度型 adaptive timestep；
- 不把 `TRBDF2TimeIntegrator` 一并迁移，除非后续单独立项；
- 不试图用放宽 `solver-eps` 掩盖 line search 卡死。

本计划的 adaptive timestep 是 **nonlinear solve failure driven substepping**，不是 high-order error control。

## 4. Repo Truth

关键文件与当前职责：

- `src/core/nonlinearOptimization/NewtonSolver.h`
  - 声明 `NewtonSolver::solve(...)`；
  - 当前返回 `int`，但没有明确状态码语义。
- `src/core/nonlinearOptimization/NewtonSolver.cpp`
  - 当前 `dx too small` 和 `line search failed` 都只是 `break`；
  - 函数尾部无条件 `return 0`。
- `src/core/simulation/timeIntegratorSolver.cpp`
  - `SO_NEWTON` 路径调用 `NewtonSolver::solve(...)` 并返回 ret；
  - 不解释 ret。
- `src/core/simulation/implicitBackwardEulerTimeIntegrator.cpp`
  - `doTimestep(...)` 调用 solver 后只打印 `solverRet`；
  - 无论 `solverRet` 是什么，都会计算 `q1/qvel1/qacc1` 并推进。
- `src/core/simulation/timeIntegrator.h/.cpp`
  - 已有 `setTimestep(...)`、`getTimestep()`、`setqState(...)`、`getq/getqvel/getqacc(...)`；
  - 已有 `proceedTimestep()`；
  - 没有 timestep reject / restore 的显式 API。
- `src/tools/runSim/runIPCSim.cpp`
  - 主循环中调用 `intg->doTimestep(1, 3, 1)`；
  - 已有 max-step diagnostics logging；
  - 是最适合先暴露 adaptive retry config 的入口。
- `tests/src/tools/runIPCSim_gtest.cpp`
  - 已有临时 config、运行二进制、读日志/输出文件的测试工具。

## 5. 设计决策

### 5.1 `NewtonSolver` 返回码必须表达原因

新增状态码，保留 `0 == success`：

```cpp
enum class SolveStatus : int
{
  Converged = 0,
  MaxIterations = 1,
  LineSearchFailed = 2,
  StepTooSmall = 3,
  NonFinite = 4,
  LinearSolveFailed = 5
};
```

如果为了最小改动不想切换所有调用点为 enum class，可以先在 `NewtonSolver` 内定义普通 `enum SolveStatus`，但日志和测试必须使用明确名字。

### 5.2 成功条件只看最终 residual，而不是“走到了 break”

`break` 路径要根据当前 residual 决定是 `Converged` 还是失败状态码，而不是默认成功。具体判据（包括相对收敛容差）见 §11.2。

循环跑满 `numIter` 且最终未达收敛阈值，返回 `MaxIterations`。

### 5.3 `ImplicitBackwardEulerTimeIntegrator` 先 fail-fast

核心 integrator 不应该自动吞掉失败，也不应该默认做 adaptive，因为这会改变所有调用者的时间推进语义。

Phase 2 的默认行为：

```text
solverRet == 0:
  compute q1/qvel1/qacc1 and optionally proceedTimestep()

solverRet != 0:
  do not update q1/qvel1/qacc1
  do not call proceedTimestep()
  either throw or return failure through new API
```

为了兼容旧的 `void doTimestep(...)`，本计划推荐增加新 API：

```cpp
int tryTimestep(int updateq = 1, int verbose = 0, int printResidual = 0);
```

并让旧 `doTimestep(...)` 调用 `tryTimestep(...)`，如果失败则 throw。这样旧调用者不会继续静默推进，新调用者可以选择处理失败。

### 5.4 Adaptive substep 放在 `runIPCSim` 层

`runIPCSim` 是 CLI/demo/experiment 入口，适合先做可配置 retry。默认关闭，配置打开。

推荐 config 字段：

```json
{
  "adaptive-timestep": true,
  "adaptive-min-timestep": 1e-6,
  "adaptive-max-consecutive-retries": 10,
  "adaptive-max-substeps-per-frame": 64,
  "adaptive-shrink": 0.5,
  "adaptive-growth": 1.0,
  "adaptive-growth-after-success": 3
}
```

字段语义：

- `adaptive-timestep`: 是否开启 nonlinear solve failure driven substepping；
- `adaptive-min-timestep`: 单个 substep 最小 dt，低于它仍失败则 abort（与"帧已结束"判据无关，仅作 abort 用）；
- `adaptive-max-consecutive-retries`: 当前 retry 链允许的最大连续失败次数（成功归零）；
- `adaptive-max-substeps-per-frame`: 单个 nominal frame 内允许的总 substep 数（不归零，防止"每个 substep 都需要 1 次 retry"无限循环）；
- `adaptive-shrink`: 失败时 `dtTry *= shrink`；
- `adaptive-growth`: 成功后下一 substep 的扩张系数。**默认 1.0**（不扩张）以避免临界 dt 附近震荡；
- `adaptive-growth-after-success`: 必须连续 N 次成功才允许 growth > 1.0 触发，防止"小 dt 成功 → 扩大 → 失败 → 收缩"的乒乓行为。

帧结束判据用相对量 `remaining < 1e-9 * nominalDt`，与 `adaptive-min-timestep` 解耦——min-timestep 仅作 abort 阈值，不参与"这帧是否结束"判断。

### 5.4.1 Forced motion 与外力的 sub-frame 处理

启用 adaptive 后，nominal frame 内被拆分成多个 substep，但 `runIPCSim` 主循环原本只在 nominal frame 边界查询 `anim.json` 的 attachment 目标和外力。如果 substep 都用 nominal frame 起点的目标值：

- spring 拉力的"目标速度"在 substep 内变成 0，到 nominal frame 边界突然跳变；
- implicit Euler 看到一个 staircase 形状的外部位移，相当于在系统上加了高频扰动；
- **adaptive 自己制造了它要解决的不收敛**。

强制要求：每个 substep 必须根据 `tElapsedInFrame = nominalDt - remaining` **重算**外部状态（attachment target、fext、动画约束），通常用 nominal frame 之间的线性插值：

```
target_sub = target[frame] + (target[frame+1] - target[frame]) * (tElapsedInFrame / nominalDt)
```

这要求 `runIPCSim` 在 substep 循环里调用 `refreshExternalState(tElapsedInFrame, subDt)`，不能把这一步留给 integrator——integrator 不知道 nominal frame 边界。

### 5.4.2 TimeIntegrator 状态快照接口

`ImplicitBackwardEulerTimeIntegrator` 的内部状态远不止 `q/qvel/qacc`，还包括 `z`（Newton 试解）、`q1/qvel1/qacc1`（pending state）、`b`（cached force）、`solverRet`、`constraintsChanged`/`generalForceModelChanged` 标志等。零散的 `setq/setqvel/...` 既不完整也容易漏。

强制要求 `TimeIntegrator` 暴露一对 opaque 接口：

```cpp
class TimeIntegratorStateSnapshot;  // 子类各自实现
std::unique_ptr<TimeIntegratorStateSnapshot> snapshotState() const;
void restoreState(const TimeIntegratorStateSnapshot &snap);
```

由 `ImplicitBackwardEulerTimeIntegrator` 自己在 `snapshotState()` 中负责存所有需要的字段，Phase D wrapper 只负责调用快照/恢复，不需要知道哪些字段需要存。这避免了未来加状态字段时漏存，也避免了零散 setter 被外部代码误用。

旧的零散 setter（`setqState/setTimestep/setTimestepID`）建议在 Phase D 完成后改成 protected，或保留 public 但加注释说明非 adaptive 路径不应使用。

### 5.5 日志需要区分 nominal timestep 与 substep

**`timestepID` 语义**：`timestepID` 必须**始终等于 nominal frame index**，**不**随 substep 推进。原因：

- dump 输出按 `timestepID` 命名（`ret0340.obj`），如果 substep 推进 `timestepID`，nominal frame 6 拆成 4 个 substep → `timestepID` 跳到 9，下一帧的输出就错位写到 `ret0009.obj` 上。
- 动画 / attachment 查询同样按 nominal frame index，不能让 substep 偏移它。

实现层面：在 `tryTimestep` 增加 `bool advanceTimestepID = true` 参数，adaptive wrapper 调用时传 `false`，nominal frame 完整接受后再单独 `++timestepID`。

当前 `Txxxx` 是 nominal timestep id。开启 adaptive 后，一个 nominal timestep 内可能有多个 substep。

日志建议：

```text
Adaptive timestep T2320 substep 0: dt=0.001 remaining=0.001
Adaptive timestep T2320 substep 0 rejected: solverRet=StepTooSmall residual=0.0095 nextDt=0.0005
Adaptive timestep T2320 substep 1 accepted: dt=0.0005 remaining=0.0005 solverRet=Converged
Adaptive timestep T2320 substep 2 accepted: dt=0.0005 remaining=0 solverRet=Converged
```

不开启 adaptive 时，失败应明确：

```text
ImplicitBackwardEuler timestep failed: solverRet=StepTooSmall residual=...
```

### 5.6 Timestep 日志由 `TimeIntegrator` 负责，不放进 `NewtonSolver`

`NewtonSolver` 是通用优化器，不应该知道 `TimeIntegrator::timestepID`，也不应该维护 timestep/substep log prefix。Phase A 只要求它返回可信状态码，并提供稳定的状态名 helper。

实际调试需要把 `dx too small`、`line search failed`、`StepTooSmall` 等失败定位到 `T2330` 这类 timestep 时，由 `ImplicitBackwardEulerTimeIntegrator` 在调用 solver 前后打印上下文：

```text
ImplicitBackwardEuler timestep begin: T2330 dt=0.001
ImplicitBackwardEuler timestep end: T2330 solverRet=StepTooSmall residual=0.00953457 accepted=false
```

如果 adaptive substep 开启，则由 `runIPCSim` 的 adaptive wrapper 打印 nominal frame 与 substep：

```text
adaptive timestep reject: frame=2330 substep=0 dt=0.001 solverRet=StepTooSmall residual=0.00953457 nextDt=0.0005
adaptive timestep accept: frame=2330 substep=1 dt=0.0005 remaining=0.0005 solverRet=Converged
```

这样日志仍然能定位失败 timestep，但 `NewtonSolver` 保持无 caller context 的通用职责。

## 6. Implementation Phases

### Phase A: `NewtonSolver` 状态码与单元测试

**目标：** 让 `NewtonSolver::solve()` 的返回值可信；不在 `NewtonSolver` 内加入 timestep/substep 日志语义。

**修改文件：**

- `src/core/nonlinearOptimization/NewtonSolver.h`
- `src/core/nonlinearOptimization/NewtonSolver.cpp`
- `tests/src/core/nonlinearOptimization/NewtonSolver_gtest.cpp`，如不存在则新建
- `tests/src/core/nonlinearOptimization/CMakeLists.txt`

**步骤：**

1. 在 `NewtonSolver.h` 中加入 `SolveStatus`，提供 helper：

   ```cpp
   static const char *solveStatusToString(int status);
   ```

   或 free function：

   ```cpp
   const char *newtonSolveStatusToString(int status);
   ```

2. 在 `solve()` 内维护：

   ```cpp
   int status = static_cast<int>(SolveStatus::MaxIterations);
   double finalGradNorm = std::numeric_limits<double>::infinity();
   ```

3. 每次算完 gradient 后更新 `finalGradNorm`。

4. 主循环顶部根据当前 `gradNorm` 判断收敛并 break。判据细节（绝对 + 相对）见 §11.2。

5. `eng1 > eng` 的 line search failed 分支中：根据 residual 决定 `Converged` 或 `LineSearchFailed`，输出包含 iteration、当前 residual、status。具体判据（含 FP-limit fallback）见 §11.2。

6. `stepSize < 1e-15` 分支中：根据 residual 决定 `Converged` 或 `StepTooSmall`，输出包含 iteration、step size、当前 residual、status。具体判据（含 FP-limit fallback）见 §11.2。

7. 如果 `deltax`、`eng`、`eng1`、`gradNorm` 非有限，返回 `NonFinite`，不要 `abort()`。

8. 函数尾部返回 `status`，不再无条件 `return 0`。

9. 添加测试：

   - 一个简单二次能量，Newton 应返回 `Converged`；
   - `solveStatusToString(...)` 对每个状态返回稳定字符串。
   - 失败 case 的覆盖在 §11.2 refinement 之后需要更新（旧的"residual 大于 eps 必然返回非 0"假设已经被相对判据破坏）。

**验证命令：**

```bash
cmake --build build/base_no_mkl_debug --target nonlinearOptimization_gtest
build/base_no_mkl_debug/tests/src/core/nonlinearOptimization/nonlinearOptimization_gtest --gtest_filter='*Newton*'
```

如果当前测试 target 名称不同，以 `ctest -N | rg Newton` 或现有 `CMakeLists.txt` 为准。

### Phase B: Time integrator accept/reject 语义

**目标：** solver 失败不再静默推进 `q/qvel/qacc`。

**修改文件：**

- `src/core/simulation/timeIntegrator.h`
- `src/core/simulation/implicitBackwardEulerTimeIntegrator.h`
- `src/core/simulation/implicitBackwardEulerTimeIntegrator.cpp`
- `tests/src/core/simulation/...` 或 `tests/src/tools/runIPCSim_gtest.cpp`

**步骤：**

1. 在 `ImplicitBackwardEulerTimeIntegrator` 增加：

   ```cpp
   int tryTimestep(int updateq = 1, int verbose = 0, int printResidual = 0);
   ```

2. 把当前 `doTimestep(...)` 主体移动到 `tryTimestep(...)`。

3. 调 solver 前由 `ImplicitBackwardEulerTimeIntegrator` 打印 timestep begin 日志，不向 `NewtonSolver` 传递 timestep：

   ```cpp
   if (verbose) {
     std::cout << "ImplicitBackwardEuler timestep begin: T" << timestepID
               << " dt=" << timestep << std::endl;
   }
   ```

4. 在 solver 返回后立即判断：

   ```cpp
   if (verbose) {
     std::cout << "ImplicitBackwardEuler timestep end: T" << timestepID
               << " solverRet=" << newtonSolveStatusToString(solverRet)
               << " accepted=" << (solverRet == 0 ? "true" : "false")
               << std::endl;
   }

   if (solverRet != 0) {
     if (printResidual) { print residual and energy components; }
     TimeIntegrator::doTimestep(0, verbose, printResidual);
     return solverRet;
   }
   ```

   注意失败路径不要写 `q1/qvel1/qacc1`，也不要 `proceedTimestep()`。

5. `doTimestep(...)` 改成：

   ```cpp
   const int ret = tryTimestep(updateq, verbose, printResidual);
   if (ret != 0) {
     throw std::runtime_error("ImplicitBackwardEulerTimeIntegrator timestep failed with solverRet=" + std::to_string(ret));
   }
   ```

6. 添加测试：

   - 构造一个会返回非 0 的 mock/failing potential 或使用测试 seam；
   - 调用 `tryTimestep(updateq=1)` 后确认 `q/qvel/qacc/timestepID` 不变；
   - 调用 `doTimestep(updateq=1)` 确认抛异常。
   - 触发 solver failure 时，`ImplicitBackwardEulerTimeIntegrator` 的日志包含 `T<id>` 与 `solverRet=StepTooSmall`。

**设计注意：**

如果缺少方便的 mock energy，不要为了测试大改 simulation 架构。可以先在 `NewtonSolver` 层覆盖返回码，再用 `runIPCSim` 集成测试覆盖失败时进程返回非 0。

### Phase C: `runIPCSim` fail-fast 与配置解析

**目标：** 默认不启用 adaptive 时，solver 失败让 CLI 明确失败，而不是写出后续帧。

**修改文件：**

- `src/tools/runSim/runIPCSim.cpp`
- `src/tools/runSim/runIPCSimSetup.h`
- `src/tools/runSim/runIPCSimSetup.cpp`
- `tests/src/tools/runIPCSim_gtest.cpp`

**步骤：**

1. 增加配置结构：

   ```cpp
   struct AdaptiveTimestepConfig
   {
     bool enabled = false;
     double minTimestep = 1e-6;
     int maxRetries = 10;
     double shrink = 0.5;
     double growth = 1.25;
   };
   ```

2. 在 setup 中解析：

   ```json
   "adaptive-timestep"
   "adaptive-min-timestep"
   "adaptive-max-retries"
   "adaptive-shrink"
   "adaptive-growth"
   ```

3. 校验：

   - `adaptive-min-timestep > 0`
   - `adaptive-max-retries >= 0`
   - `0 < adaptive-shrink && adaptive-shrink < 1`
   - `adaptive-growth >= 1`

4. 默认路径继续调用 `doTimestep(...)` 或 `tryTimestep(...)` 后 fail-fast：

   ```cpp
   int ret = intg->tryTimestep(1, 3, 1);
   if (ret != 0) {
     throw std::runtime_error("runIPCSim timestep failed...");
   }
   ```

5. 测试无 adaptive 的失败路径：

   - 用一个最小测试配置触发 Newton failure；
   - 断言 `runIPCSim` 返回非 0；
   - 断言日志包含 solver failure 字符串。

### Phase D: Adaptive substep 主循环

**目标失败模式：** 单次 nominal step 内接触集变化或几何穿越导致 Newton 二次模型失效。典型表现：Hessian nnz 在相邻 Newton iter 之间跳变（如 shell T507 的 207 → 279）、`||grad||` 非单调（先降后涨）、line search 在新激活的 barrier 项前被打回原形。详见 §11.5 的诊断和理论说明。

**对不属于此类的失败 Phase D 不会有效**，特别是 §11.1 描述的 FP-saturation 失败——那类已经在 Phase A refinement 里被相对收敛判据兜住，不会走到 Phase D。

**目标：** `runIPCSim` 开启 adaptive 时，在 nominal timestep 内自动 retry smaller dt。

**修改文件：**

- `src/tools/runSim/runIPCSim.cpp`
- `src/tools/runSim/runIPCSimSetup.{h,cpp}`（新 config 字段解析）
- `src/core/simulation/timeIntegrator.{h,cpp}`（新增 `snapshotState/restoreState` 接口、`tryTimestep` 增加 `advanceTimestepID` 参数）
- `src/core/simulation/implicitBackwardEulerTimeIntegrator.{h,cpp}`（实现快照子类，覆盖所有内部状态）
- `tests/src/tools/runIPCSim_gtest.cpp`

**前置依赖（必须先做）：**

1. **TimeIntegrator 状态快照接口**：见 §5.4.2。新增 opaque 类 `TimeIntegratorStateSnapshot` + `snapshotState()/restoreState(...)`。`ImplicitBackwardEulerTimeIntegrator` 的子类快照必须包含：`q/qvel/qacc/timestepID/timestep`，加上 `z/q1/qvel1/qacc1/b/solverRet/constraintsChanged/generalForceModelChanged`。
2. **`tryTimestep` 加 `advanceTimestepID` 参数**：见 §5.5。adaptive wrapper 内全程传 `false`，整个 nominal frame 完成后由 wrapper 自己 `++timestepID`。
3. **`runIPCSim` 抽出 `refreshExternalState(tElapsedInFrame, subDt)`**：见 §5.4.1。把当前主循环里 per-frame 的 attachment / fext / 动画目标计算包进一个函数，接受 frame 内的相对时间偏移。

**伪代码（runIPCSim 层）：**

```cpp
bool runAdaptiveNominalStep(integrator, nominalDt, config)
{
  double remaining = nominalDt;
  double dtTry = nominalDt;
  int consecutiveRetries = 0;
  int totalSubsteps = 0;
  int consecutiveSuccess = 0;

  while (remaining > 1e-9 * nominalDt) {  // 帧结束判据：相对量
    const double subDt = std::min(dtTry, remaining);
    const double tElapsedInFrame = nominalDt - remaining;

    // 1) 重算外部状态（动画目标 / fext / 约束），这一步必须每个 substep 都做
    refreshExternalState(tElapsedInFrame, subDt);

    // 2) 完整状态快照（包括 z/q1/etc，由 integrator 内部决定存什么）
    auto snap = integrator->snapshotState();

    integrator->setTimestep(subDt);
    const int ret = integrator->tryTimestep(/*updateq=*/1, verbose, /*advanceTimestepID=*/false);
    ++totalSubsteps;

    if (ret == 0) {
      remaining -= subDt;
      consecutiveRetries = 0;
      ++consecutiveSuccess;

      // growth 仅在连续成功 N 次后允许，避免临界 dt 附近震荡
      if (consecutiveSuccess >= config.growthAfterSuccess && config.growth > 1.0) {
        dtTry = std::min(subDt * config.growth, remaining);
      } else {
        dtTry = std::min(subDt, remaining);
      }
      continue;
    }

    // 失败：恢复状态、收缩 dt、计数
    integrator->restoreState(*snap);
    consecutiveSuccess = 0;
    ++consecutiveRetries;
    dtTry = subDt * config.shrink;

    const bool dtTooSmall = dtTry < config.minTimestep;
    const bool tooManyRetries = consecutiveRetries > config.maxConsecutiveRetries;
    const bool tooManySubsteps = totalSubsteps > config.maxSubstepsPerFrame;
    if (dtTooSmall || tooManyRetries || tooManySubsteps) {
      integrator->setTimestep(nominalDt);
      return false;
    }
  }

  integrator->setTimestep(nominalDt);
  ++integrator->timestepID();  // nominal frame 完整接受，统一前进
  return true;
}
```

**重要细节：**

- `timestepID` 不随 substep 推进；每个 accepted substep 内 wrapper 也不能让 integrator 自己加，必须靠 `advanceTimestepID=false`。
- 失败 retry 前必须 `restoreState` 完整恢复，包括 `z/q1/qvel1/qacc1` 等内部缓存——否则下一次 `tryTimestep` 的 initial guess（依赖 `q + qvel*h`）虽然对了，但缓存的 trial state 可能污染 `z`。这是为什么用 opaque snapshot/restore 而不是零散 setter。
- `setTimestep` 改变 dt 后，`updateA/updateD/updateb` 在 `tryTimestep` 入口会重算（实测确认，见 §8 "Cached solver state after dt changes"），无需手动 invalidate。Newton 内部的 symbolic factorization 仅依赖稀疏 pattern，与 dt 无关。
- adaptive wrapper 内默认 `verbose ≤ 1`（只打 begin/end），避免 retry 链上每个 substep 都 dump 几十行 Newton iter 信息。详细 log 应该在 fail 时单独写到 dedicated debug file。
- 一阶离散化误差按 substep 数累积，**adaptive 路径的物理与 non-adaptive 不 bit-identical**。研究/对比实验需要明确这一点（详见 Phase E 文档要求）。

### Phase E: 日志、文档与示例

**目标：** 用户能看懂为什么 timestep 被拒绝或拆分。

**修改文件：**

- `src/tools/runSim/runIPCSim.cpp`
- `examples/ipc/README.md`
- `README.md` 如需要
- 可选：`examples/ipc/shell/shell-ipc.json` 不建议默认开启 adaptive，避免改变现有示例语义

**日志要求：**

- 默认失败：

  ```text
  runIPCSim timestep failed: frame=2320 solverRet=StepTooSmall
  ```

- adaptive reject：

  ```text
  adaptive timestep reject: frame=2320 substep=0 dt=0.001 solverRet=StepTooSmall nextDt=0.0005
  ```

- adaptive accept：

  ```text
  adaptive timestep accept: frame=2320 substep=1 dt=0.0005 remaining=0.0005
  ```

- adaptive final fail：

  ```text
  adaptive timestep failed: frame=2320 retries=10 minDt=1e-6
  ```

**文档说明：**

- adaptive timestep 是 solve-failure retry，不是精度控制；
- 它能避免未收敛状态被静默推进；
- 它不能保证物理设置合理，例如自由角在重力下仍可能卷曲；
- 如果 adaptive 经常触发，应该检查 `dt`、阻尼、材料、接触参数、固定点设置；
- **一阶离散化误差按 substep 数累积**：implicit Euler 是一阶方法，把一个 nominal frame 拆成多个 substep 后，这一帧的物理与不开 adaptive 的 run 不再 bit-identical（数值耗散按子步数累积）。对动画无所谓，但**研究/对比实验不要在 adaptive run 之间或 adaptive vs non-adaptive 之间做精确数值对比**。
- adaptive 路径每个 substep 必须重新查询动画 attachment 目标 / fext（详见 §5.4.1），所以 attachment 文件必须支持 sub-frame 插值——目前的 nominal-frame-key 文件格式默认用线性插值，自定义 attachment 接口需要显式实现 sub-frame 查询。

## 7. Testing Strategy

### Unit Tests

1. `NewtonSolver` convergence returns `Converged`;
2. `solveStatusToString` returns stable names;
3. `ImplicitBackwardEulerTimeIntegrator::tryTimestep` failure does not mutate accepted state.

Note: a once-planned "step-too-small with residual above eps returns `StepTooSmall`" test was removed when §11.2 added the relative-convergence branch. Such a state can now be classified as `Converged` if `gradNorm < lambda0 * 1e-4`. Future tests for failure paths must construct a case where both absolute and relative thresholds are missed.

### Integration Tests

1. `runIPCSim` without adaptive fails loudly on a forced Newton failure case;
2. `runIPCSim` with adaptive logs at least one reject and then succeeds on a case where smaller dt converges;
3. Existing smoke tests still pass for shell/tet/cubic cases that converge normally.

**Phase D 测试 fixture 配方（必须具体到能直接用，否则没人能写）：**

由于 §11.2 的 Phase A refinement 已经把 FP-saturation 失败模式吃掉了，构造一个"non-adaptive 失败 + adaptive 成功"的最小 case 不再 trivial。推荐 fixture：

- 用 `examples/ipc/cubic/box-with-sphere`（已知能稳定触发接触场景）；
- 把 `timestep` 从 `0.001` 调大到 `0.005`——大 dt 让一步内 sphere 与 floor 之间多个新 contact pair 同时激活，制造 contact-set instability；
- 保持其他参数不变。

期望行为：
- non-adaptive：在某个早期 timestep 触发 `solverRet=StepTooSmall`（且 Hessian nnz 在 Newton iter 之间跳变，验证是 contact-set instability 而非别的原因），`runIPCSim` 返回非 0；
- adaptive=on 且 `min-timestep=1e-5`：同一帧首次失败被 reject，dt 收缩到 0.0025 或更小后 substep 成功，`runIPCSim` 跑完。

测试断言：
- 日志包含至少一行 `adaptive timestep reject:`；
- 日志包含相同 `frame=` 上的 `adaptive timestep accept:`；
- 进程返回 0；
- 输出帧数等于 `num-timestep`（dump 文件按 nominal frame 命名，验证 §5.5 的 `timestepID` 不被 substep 偏移）。

如果以上 fixture 在实际跑时 contact-set instability 不稳定触发，备用：用 `cubic/box-squash` 把 `movement` 从 `[0,0,-2.55]` 调到 `[0,0,-15.0]`（更剧烈推压），同样把 `timestep` 调大，让 box 在一步内深度自接触。

### Manual Reproduction

After implementation, rerun the current shell IPC case:

```bash
build/base_no_mkl_debug/bin/runIPCSim examples/ipc/shell/shell-ipc.json
```

Expected default behavior after this fix:

- a timestep that genuinely fails to converge (residual above both absolute eps and the §11.2 relative thresholds) stops the run with non-zero failure;
- no later `retXXXX.obj` files are written after the failed timestep.

Then test adaptive by adding:

```json
"adaptive-timestep": true,
"adaptive-min-timestep": 1e-6,
"adaptive-max-retries": 10,
"adaptive-shrink": 0.5,
"adaptive-growth": 1.25
```

Expected adaptive behavior:

- failing nominal steps are retried with smaller dt;
- log shows reject/accept substeps;
- if even min dt fails, the run exits non-zero rather than silently continuing.

## 8. Risk Assessment

### Risk: Existing tests assume `Solver Ret: 0`

Some logs or tests may implicitly assume Newton always returns 0. Update tests to assert meaningful status instead of hardcoding success when residual is above eps.

### Risk: Throwing in `doTimestep` changes old callers

This is intentional for fail-fast correctness. If a caller needs custom handling, migrate it to `tryTimestep`.

### Risk: Adaptive hides bad setups

Adaptive is off by default. Documentation should tell users that frequent retry means the base timestep/setup is suspect.

### Risk: timestepID semantics become confusing

Keep dump indexing outside integrator. Log nominal frame id and substep id explicitly. Do not rely on integrator `timestepID` alone to name output frames in adaptive mode.

### Risk: Cached solver state after dt changes

**已验证。** 改变 `timestep` 后：

- `updateA/updateD/updateb` 在每次 `tryTimestep` 入口都会调用，重算所有依赖 dt 的矩阵 / 向量项；
- Newton solver 的 symbolic factorization 仅依赖稀疏 pattern，与 dt 无关；数值因子化每次 `solve` 都重做；
- 因此 dt 在 substep 之间变化无需手动 invalidate 任何缓存。

这一节保留是为了未来如果加新的 dt-相关 cache（比如 quasi-Newton 的 H 估计、contact warm start 等）时记得重新检查。

## 9. Suggested Commit Split

1. `fix(newton): report nonconverged solve statuses`
2. `fix(sim): reject failed implicit Euler steps`
3. `feat(runIPCSim): parse adaptive timestep config`
4. `feat(runIPCSim): retry failed steps with adaptive substeps`
5. `docs(ipc): document Newton failure and adaptive timestep behavior`

## 10. Completion Checklist

- [ ] `NewtonSolver` no longer returns 0 for `dx too small` with residual above eps.
- [ ] `ImplicitBackwardEulerTimeIntegrator` exposes `tryTimestep`.
- [ ] `doTimestep` throws or otherwise fails loudly on nonzero solver status.
- [ ] `runIPCSim` default mode stops on failed timestep.
- [ ] `runIPCSim` adaptive mode rejects/restores/retries failed substeps.
- [ ] Logs include solver status names, nominal frame id, substep id, dt, and retry result.
- [ ] Unit tests cover success and failure return statuses.
- [ ] Integration tests cover fail-fast and adaptive retry.
- [ ] Existing shell/tet/cubic smoke tests still pass.

## 11. Post-Diagnosis Refinement (2026-04-25)

After Phase A and Phase B landed, the six `examples/ipc` cases were rerun under the
new fail-fast semantics. **All six failed**, and inspection of the generated logs
showed the dominant failure mode was not the one this plan originally anticipated.

### 11.1 Observed failure mode

In every case, `feasibleAlpha == 1` and the IPC `materialClampCount /
contactClampCount` were both zero up to the failing timestep. The solver was
not being defeated by CCD or material inversion. Instead, the line search was
saturating at `lineSearchAlpha = 7.88861e-31` (== `0.5^100`, the iteration cap of
`LSM_SIMPLE`) or at similarly tiny powers of two.

The mechanism is floating-point precision relative to the energy magnitude:

```
eng        ≈ -1.48e6           (shell case at T340)
||grad||   ≈ 1.28e-4           (Newton has reduced gradient by ~6 orders of magnitude)
deltax_max ≈ 1.94e-9
predicted ΔE = grad · deltax ≈ 1e-13
|ΔE / eng| ≈ 7e-20             (well below double-precision epsilon)
```

When the predicted energy decrease falls below `|eng| * eps_machine`, the
floating-point comparison `eng1 < eng` becomes meaningless: `LSM_SIMPLE`'s strict
descent test never succeeds, and α is driven all the way down to its iteration
cap. `acceptedStepMaxNorm` then trips the absolute `< 1e-15` step-too-small
break. `NewtonSolver` therefore reported `StepTooSmall` even though Newton had
effectively converged in any practical sense.

The original concern that adaptive substepping (Phase D) would be the right
remedy turned out to be wrong for these cases. Smaller `dt` does not change
`|eng|` enough to escape the FP wall, and the failures were not in the
basin-of-attraction regime that Phase D targets.

### 11.2 Refinements to Phase A

Three changes to the core solver, all targeted at the FP-precision failure mode:

1. **Default line search switched to `LSM_BACKTRACK`** in
   `NewtonSolver::SolverParam::lsm` (header default) and in
   `minimizeEnergy.cpp`. Armijo backtracking with `c1 = 1e-4` accepts α=1 as
   soon as `eng + c1·α·grad·dx` rounds to `eng` in floating point, so it does
   not chase the FP wall the way strict-descent halving does.

2. **Relative convergence as an OR branch with absolute eps.** In the main
   Newton loop, accept convergence when either
   `gradNorm < epsilon` (absolute) or `gradNorm < lambda0 * 1e-5` (relative,
   where `lambda0` is the initial gradient max-norm). Five orders of magnitude
   of gradient reduction is consistent with PETSc-default Newton convergence
   and gives a meaningful answer when `epsilon` is unreachable in double
   precision.

3. **Looser FP-limit fallback in step-too-small / line-search-failed exits.**
   When the iteration is about to break because the step is below `1e-15` or
   line search produced `eng1 > eng`, accept as `Converged` if
   `gradNorm < lambda0 * 1e-4` (4 orders of magnitude). This catches the case
   where Newton has made substantial progress but FP noise stops it just shy of
   the tighter relative threshold.

The factor split (1e-5 in normal iter, 1e-4 in fallback) is intentional:
in normal iteration the solver should converge tightly when it can; only when
the FP precision wall is reached do we accept a looser relative criterion.

### 11.3 Refinement to Phase B

The `acceptedNearConvergedStalled` accept policy added in Phase B (residualMax
within `2 * eps`) was **removed**. Its job was to mask the FP-saturation
failure at the integrator layer. With the relative-convergence checks in
`NewtonSolver`, that situation now returns `Converged` from the solver
itself, and the integrator can go back to the simple rule:

```
acceptedTimestep = (solverRet == 0)
```

The integrator no longer needs to second-guess the solver, and the absolute
`2 * eps` ratio (which has no access to `lambda0` and was therefore unable to
distinguish FP-saturated convergence from a system whose gradient has barely
moved) is gone.

### 11.4 Empirical validation

Six examples, all run with `build/base_no_mkl/bin/runIPCSim`:

| case | original failure | after solver-only fix | + coeff=1e4, max-iter=200 |
|------|-----------------|----------------------|---------------------------|
| `cubic/box-with-sphere` | T315 fail | **T1999 ✅** | (not rerun) |
| `cubic/box-hang`        | T6 fail   | T1068 fail   | **T1999 ✅** |
| `cubic/box-squash`      | T2 fail   | T109 fail    | **T199 ✅**  |
| `tet/box-hang`          | T29 fail  | T1044 fail   | **T1999 ✅** |
| `tet/box-squash`        | T1 fail   | T79 fail     | **T199 ✅**  |
| `shell`                 | T340 fail | T523 fail    | T507 fail (self-contact) |

Five of six cases now run to their configured `num-timestep`. The remaining
shell failure is a different physical regime: with the softer fixed-vertex
spring, the shell sags far enough under gravity to develop self-contact (IPC
Hessian goes from zero non-zeros pre-T500 to 200+), and Newton converges 5
orders of magnitude in gradient before stalling at `||grad|| ≈ 0.026` against
`|eng| ≈ 5.26e6` — relative `5e-9`, which is at the FP floor for that
energy magnitude. This is no longer an FP-precision-vs-eps mismatch; it is the
system genuinely entering a stiff contact configuration.

### 11.5 Status of Phase D (adaptive substep)

Phase D is **not** implemented. The diagnosis above shows that adaptive
substepping is the wrong fix for the FP-saturation failure mode that triggered
this plan. It remains the right tool for the residual shell case: when the
nominal step lands in a hard self-contact configuration, halving `dt` keeps
the contact set from changing as drastically inside one step and gives Newton
a better-conditioned subproblem. Phase D should be revisited after the user
exercises the post-refinement state with longer simulations and decides
whether the residual shell-style failures are common enough to justify the
implementation cost.
