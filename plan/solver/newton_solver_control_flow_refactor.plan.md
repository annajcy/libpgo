# Newton Solver Control Flow Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变 Newton 求解语义、line-search 行为、IPC active-set reuse 语义和 public API 的前提下，把 `NewtonSolver::solve()` 从单个长流程收口为可验证的小步骤。

**Architecture:** 本轮只重构 hot-path control flow。保留 `NewtonSolver` public API、`PotentialEnergy` fused evaluation API、`LineSearch` public API 和 `TimeIntegratorSolver` 调用方式；先补 characterization tests，再提取 small private helpers 和 RAII line-search scope。Knitro/Ipopt wrapper 不在本轮范围。

**Tech Stack:** C++20, Eigen, CMake, GoogleTest, existing `nonlinearOptimization` / `simulation` libraries.

---

## 现状判断

`src/core/nonlinearOptimization/NewtonSolver.cpp` 目前约 600 行，其中 `NewtonSolver::solve()` 同时处理：

- fixed DOF reset 与 gradient filtering
- fused `func_grad_hessian()` evaluation
- convergence / loose convergence 判定
- Hessian topology fixed / non-fixed 的 reduced system 构建
- symbolic/numeric linear solver lifecycle
- material/contact max-step clamp
- line-search active-set scope
- backtracking initial trial energy reuse
- accepted step diagnostics
- verbose logging 与 final status 写回

这些逻辑现在是正确且有测试覆盖的，但后续继续优化 IPC、material max-step、line-search policy 时，主循环太容易被局部修改搅乱。重构目标是让每个步骤可命名、可测试、可复用。

## 范围与非目标

- 只修改 `src/core/nonlinearOptimization/NewtonSolver.h/.cpp` 和 `tests/src/core/NewtonSolver_gtest.cpp`。
- 可以小幅修改 `src/core/nonlinearOptimization/solveDiagnostics.h`，但只新增字段/record helper，不删除现有字段。
- 不修改 `PotentialEnergy` public virtual methods。
- 不修改 `LineSearch` 算法和 public API。
- 不修改 `TimeIntegratorSolver`、`ImplicitBackwardEulerTimeIntegrator`、`TRBDF2TimeIntegrator`，除非测试暴露必须同步的 include。
- 不触碰 `knitroOptimizer.*` / `IpoptOptimizer.*`。
- 不把 enum 全面迁移为 `enum class`，避免 API churn。
- 不追求性能提升；本轮完成标准是行为等价和控制流边界清晰。

## 目标结构

### `NewtonSolver.h`

新增 private/protected helper types：

```cpp
struct IterationState
{
  int iter = 0;
  double energy = 0.0;
  double gradMaxNorm = 0.0;
  double gradNorm = 0.0;
  double lambda0 = 1.0;
  double relThreshold = 0.0;
  bool absConverged = false;
  bool relConverged = false;
};

struct StepAcceptance
{
  double feasibleAlpha = 1.0;
  double lineSearchAlpha = 1.0;
  double effectiveAlpha = 1.0;
  double acceptedEnergy = 0.0;
  double acceptedStepMaxNorm = 0.0;
  enum class NonFiniteReason
  {
    None,
    FeasibleAlpha,
    TrialEnergy,
    LineSearchResult
  };

  NonFiniteReason nonFiniteReason = NonFiniteReason::None;

  bool nonFinite() const { return nonFiniteReason != NonFiniteReason::None; }
};
```

新增 private/protected methods：

```cpp
void applyFixedValues();
IterationState evaluateCurrentState(int iter, double epsilon, double lambda0, bool hasInitialGradNorm);
bool isConverged(const IterationState &state) const;
bool prepareReducedSystem(double lambdaScale, double lambda0);
void ensureLinearSolver(bool fixedHessianTopology);
bool solveReducedNewtonDirection(bool fixedHessianTopology);
bool expandReducedStep();
StepAcceptance runLineSearchStep(double currentEnergy, int verbose, int printGap, int iter);
bool looseRelativeConverged(double gradMaxNorm, double lambda0) const;
void recordConvergedFinalGradient();
```

这些 helper 都先服务现有 `solve()`；不向外暴露。

## 任务清单

### Task 1: 补主循环 characterization tests

**Files:**
- Modify: `tests/src/core/NewtonSolver_gtest.cpp`

- [ ] **Step 1.1: 增加 line-search scope 在 non-finite trial energy 下也成对释放的测试**

新增 test energy：

```cpp
class TestNonFiniteTrialEnergy : public PotentialEnergy
{
public:
  double func(ES::ConstRefVecXd x) const override
  {
    funcCalls++;
    if (std::abs(x[0]) < 1e-14)
      return std::numeric_limits<double>::quiet_NaN();
    return 0.5 * x.squaredNorm();
  }

  void gradient(ES::ConstRefVecXd x, ES::RefVecXd grad) const override { grad = x; }
  void hessian(ES::ConstRefVecXd, ES::SpMatD &hess) const override { hess.setIdentity(); }
  void createHessian(ES::SpMatD &hess) const override { hess.resize(1, 1); hess.setIdentity(); }
  void getDOFs(std::vector<int> &dofs) const override { dofs = { 0 }; }
  int getNumDOFs() const override { return 1; }
  MaxStepResult computeMaxStepLimit(ES::ConstRefVecXd, ES::ConstRefVecXd) const override { return MaxStepResult::unconstrained(); }
  void beginLineSearch(ES::ConstRefVecXd, ES::ConstRefVecXd) const override { beginLineSearchCalls++; }
  void endLineSearch() const override { endLineSearchCalls++; }

  mutable int funcCalls = 0;
  mutable int beginLineSearchCalls = 0;
  mutable int endLineSearchCalls = 0;
};
```

同时在 test file include 区增加：

```cpp
#include <limits>
```

这个 fixture 从 `x=2` 出发，一轮 Newton direction 后 first trial point 应该接近 `x=0`；NaN 触发条件必须绑定 trial point，而不是 `x=1`。

新增测试：

```cpp
TEST(NewtonSolverGTest, NonFiniteTrialEnergyEndsLineSearchScope)
{
  initializeLogging();

  auto energy = std::make_shared<TestNonFiniteTrialEnergy>();
  ES::VXd x(1);
  x[0] = 2.0;

  NewtonSolver::SolverParam solverParam;
  solverParam.lsm = NewtonSolver::LSM_BACKTRACK;
  const std::vector<int> fixedDOFs;
  NewtonSolver solver(x.data(), solverParam, energy, fixedDOFs);

  const int ret = solver.solve(x.data(), 1, 1e-12, 0);

  EXPECT_EQ(ret, static_cast<int>(NewtonSolver::SolveStatus::NonFinite));
  EXPECT_EQ(energy->beginLineSearchCalls, 1);
  EXPECT_EQ(energy->endLineSearchCalls, 1);
}
```

- [ ] **Step 1.2: 增加 diagnostics 记录 accepted alpha 的 characterization test**

在现有 `SolveDiagnosticsRecordsMaxStepBreakdown` 或新增测试中断言：

```cpp
EXPECT_DOUBLE_EQ(diagnostics.minLineSearchAlpha, 1.0);
EXPECT_DOUBLE_EQ(diagnostics.minEffectiveAlpha, 0.25);
```

这个测试锁定 feasible alpha 与 effective alpha 的当前关系。

- [ ] **Step 1.3: 跑 RED/GREEN baseline**

Run:

```bash
cmake --build --preset base_no_mkl_release --target NewtonSolver_gtest
./build/base_no_mkl/tests/src/core/NewtonSolver_gtest
```

Expected: 当前代码应通过这些 characterization tests。若 Step 1.1 失败，先修 RAII guard 的确切行为，再进入 Task 2。

### Task 2: 提取 line-search scope 和 step acceptance helper

**Files:**
- Modify: `src/core/nonlinearOptimization/NewtonSolver.h`
- Modify: `src/core/nonlinearOptimization/NewtonSolver.cpp`

- [ ] **Step 2.1: 在 `NewtonSolver.cpp` 文件局部提取 `LineSearchScope`**

把当前 `solve()` 内部 local struct 移到 anonymous namespace：

```cpp
namespace
{
class LineSearchScope
{
public:
  LineSearchScope(const PotentialEnergy_const_p &energy, EigenSupport::ConstRefVecXd x,
    EigenSupport::ConstRefVecXd dx, bool active):
    energy_(energy), active_(active)
  {
    if (active_)
      energy_->beginLineSearch(x, dx);
  }

  ~LineSearchScope()
  {
    if (active_)
      energy_->endLineSearch();
  }

  LineSearchScope(const LineSearchScope &) = delete;
  LineSearchScope &operator=(const LineSearchScope &) = delete;

private:
  const PotentialEnergy_const_p &energy_;
  bool active_ = false;
};
}  // namespace
```

- [ ] **Step 2.2: 新增 `StepAcceptance` struct**

在 `NewtonSolver.h` protected section 新增：

```cpp
struct StepAcceptance
{
  double feasibleAlpha = 1.0;
  double lineSearchAlpha = 1.0;
  double effectiveAlpha = 1.0;
  double acceptedEnergy = 0.0;
  double acceptedStepMaxNorm = 0.0;
  enum class NonFiniteReason
  {
    None,
    FeasibleAlpha,
    TrialEnergy,
    LineSearchResult
  };

  NonFiniteReason nonFiniteReason = NonFiniteReason::None;

  bool nonFinite() const { return nonFiniteReason != NonFiniteReason::None; }
};
```

- [ ] **Step 2.3: 提取 `runLineSearchStep()`**

从 `solve()` 中移动 line-search 分支内这些逻辑到：

```cpp
NewtonSolver::StepAcceptance NewtonSolver::runLineSearchStep(
  double currentEnergy, int verbose, int printGap, int iter)
```

职责：

- 调用 `energy->computeMaxStepLimit(x, deltax)` 并记录 `solveDiagnostics.recordMaxStep(maxStep)`。
- 用 feasible alpha 缩放 `deltax`。
- 对 `LSM_BACKTRACK` / `LSM_SIMPLE` 启用 `LineSearchScope`。
- 保持 backtracking `backtrackingWithInitialValue()` 的 initial trial energy reuse。
- 返回 `StepAcceptance`，不直接修改 `x`。
- 保留当前 non-finite verbose 文本粒度：`feasible alpha is non-finite`、`trial energy is non-finite`、`line search energy is non-finite` 不能被合并成一个模糊错误。

`solve()` 保留：

```cpp
const StepAcceptance accepted = runLineSearchStep(eng, verbose, printGap, iter);
if (accepted.nonFinite()) { status = static_cast<int>(SolveStatus::NonFinite); break; }
if (accepted.acceptedEnergy > eng) { ... existing failure branch ... }
x += deltax * accepted.lineSearchAlpha;
```

`runLineSearchStep()` 内部按 `StepAcceptance::NonFiniteReason` 设置原因，并在 helper 内打印与当前代码一致的 verbose message；`solve()` 只负责设置 status 和退出循环。

- [ ] **Step 2.4: 跑 Newton tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target NewtonSolver_gtest
./build/base_no_mkl/tests/src/core/NewtonSolver_gtest
```

Expected: all Newton solver tests pass, especially:

- `BacktrackingReusesInitialTrialEnergy`
- `GoldenLineSearchDoesNotUseBoundedActiveSetScope`
- `NonFiniteTrialEnergyEndsLineSearchScope`
- `SolveDiagnosticsRecordsMaxStepBreakdown`

### Task 3: 提取 state evaluation 与 convergence helpers

**Files:**
- Modify: `src/core/nonlinearOptimization/NewtonSolver.h`
- Modify: `src/core/nonlinearOptimization/NewtonSolver.cpp`
- Modify: `tests/src/core/NewtonSolver_gtest.cpp`

- [ ] **Step 3.1: 新增 `IterationState` struct**

在 `NewtonSolver.h` protected section 新增：

```cpp
struct IterationState
{
  int iter = 0;
  double energy = 0.0;
  double gradMaxNorm = 0.0;
  double gradNorm = 0.0;
  double lambda0 = 1.0;
  double relThreshold = 0.0;
  bool absConverged = false;
  bool relConverged = false;
  bool nonFiniteEnergy = false;
  bool nonFiniteGradient = false;
};
```

- [ ] **Step 3.2: 提取 `evaluateCurrentState()`**

实现：

```cpp
NewtonSolver::IterationState NewtonSolver::evaluateCurrentState(
  int iter, double epsilon, double lambda0, bool hasInitialGradNorm)
{
  IterationState state;
  state.iter = iter;

  std::memset(grad.data(), 0, sizeof(double) * grad.size());
  state.energy = energy->func_grad_hessian(x, grad, sysFull);
  if (!std::isfinite(state.energy)) {
    state.nonFiniteEnergy = true;
    return state;
  }

  sysFull.makeCompressed();
  filterVector(grad);
  state.gradMaxNorm = grad.cwiseAbs().maxCoeff();
  state.gradNorm = grad.norm();

  if (!grad.allFinite() || !std::isfinite(state.gradMaxNorm)) {
    state.nonFiniteGradient = true;
    return state;
  }

  state.lambda0 = hasInitialGradNorm ? lambda0 : state.gradMaxNorm;
  constexpr double relTolFactor = 1e-5;
  state.relThreshold = state.lambda0 * relTolFactor;
  state.absConverged = state.gradMaxNorm < epsilon;
  state.relConverged = state.gradMaxNorm < state.relThreshold;
  return state;
}
```

- [ ] **Step 3.3: 用 helper 替换 `solve()` 内 evaluation block**

`solve()` 主循环改为：

```cpp
IterationState state = evaluateCurrentState(iter, epsilon, lambda0, hasInitialGradNorm);
if (state.nonFiniteEnergy) { ... }
if (state.nonFiniteGradient) { ... }
if (!hasInitialGradNorm) { lambda0 = state.lambda0; ... }
if (state.absConverged || state.relConverged) { ... }
```

保持 verbose output 文本稳定，尤其 `||grad||_max`、`relThreshold` 和 status 字符串。

- [ ] **Step 3.4: 跑 Newton tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target NewtonSolver_gtest
./build/base_no_mkl/tests/src/core/NewtonSolver_gtest
```

Expected: all Newton tests pass。

### Task 4: 提取 reduced linear system helpers

**Files:**
- Modify: `src/core/nonlinearOptimization/NewtonSolver.h`
- Modify: `src/core/nonlinearOptimization/NewtonSolver.cpp`

- [ ] **Step 4.1: 提取 `prepareReducedSystem()`**

从 `solve()` 移动以下逻辑：

- `energy->isHessianTopologyFixed()`
- fixed topology 下 `ES::transferBigToSmall(sysFull, A11, A11Mapping, 1)`
- non-fixed topology 下 `ES::removeRowsCols(sysFull, fixedDOFs, A11)`
- damping diagonal update
- `ES::transferBigToSmall(grad, rhs, rhsb2s, 1)`
- `rhs *= -1.0`

目标签名：

```cpp
bool NewtonSolver::prepareReducedSystem(double lambdaScale, double lambda0)
```

返回值表示 `fixedHessianTopology`，供后续 solve direction 使用。

- [ ] **Step 4.2: 提取 `ensureLinearSolver()`**

目标签名：

```cpp
void NewtonSolver::ensureLinearSolver(bool fixedHessianTopology)
```

职责：如果 `!fixedHessianTopology || solver == nullptr`，创建并 analyze 当前 `A11`。

- [ ] **Step 4.3: 提取 `solveReducedNewtonDirection()`**

目标签名：

```cpp
bool NewtonSolver::solveReducedNewtonDirection(bool fixedHessianTopology)
```

职责：

- `Profiling::ScopedProfileSection scopedProfile("solver.linear_solve")`
- factorize + solve
- fixed topology 下保持当前行为：`solver.reset()`
- 检查 `deltaxSmall.allFinite()`，返回 false 表示失败

注意：当前代码没有显式处理 factorization failure；本轮不新增行为，只搬移代码。

- [ ] **Step 4.4: 提取 `expandReducedStep()`**

目标签名：

```cpp
bool NewtonSolver::expandReducedStep()
```

职责：

- `std::memset(deltax.data(), 0, sizeof(double) * n3)`
- `ES::transferSmallToBig(deltaxSmall, deltax, rhss2b)`
- 检查 `deltax.allFinite()`

- [ ] **Step 4.5: 跑 Newton tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target NewtonSolver_gtest
./build/base_no_mkl/tests/src/core/NewtonSolver_gtest
```

Expected: all Newton tests pass。

### Task 5: 最终 solver focused 验证

**Files:** No source edits unless verification finds a bug.

- [ ] **Step 5.1: 构建 solver/simulation focused tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target \
  NewtonSolver_gtest \
  implicitBackwardEulerTimeIntegrator_gtest \
  deformationModelEnergyMaxStep_gtest \
  embeddedSurfaceIPCPotentialEnergy_gtest \
  runIPCSim_gtest
```

Expected: all targets build。

- [ ] **Step 5.2: 直接运行 focused gtests**

Run:

```bash
./build/base_no_mkl/tests/src/core/NewtonSolver_gtest
./build/base_no_mkl/tests/src/core/implicitBackwardEulerTimeIntegrator_gtest
./build/base_no_mkl/tests/src/core/solidDeformationModel/deformationModelEnergyMaxStep_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
```

Expected:

- Newton status tests pass。
- implicit Euler residual/energy cache tests pass。
- material max-step diagnostics tests pass。
- IPC adapter fused evaluation tests pass。
- `runIPCSim_gtest` passes。

- [ ] **Step 5.3: Hygiene**

Run:

```bash
git diff --check
git status --short
```

Expected:

- no whitespace errors。
- changed files limited to `NewtonSolver.h/.cpp`, `NewtonSolver_gtest.cpp`, and optional `solveDiagnostics.h` if Task 1/diagnostics required it。

## 回滚策略

- 如果 Task 1 characterization tests 失败，先修测试 fixture 或确认当前行为缺陷；不要进入结构拆分。
- 如果 Task 2 后 `BacktrackingReusesInitialTrialEnergy` 失败，说明 initial trial energy reuse 被破坏，回滚 `runLineSearchStep()` 抽取。
- 如果 Task 2 后 `GoldenLineSearchDoesNotUseBoundedActiveSetScope` 失败，说明 bounded IPC line-search scope 被错误用于 golden/Brent，必须停止。
- 如果 Task 3 后 verbose log tests 失败，优先保持原日志文本；这类输出已被 runSim tests 消费。
- 如果 Task 4 后 non-fixed topology tests 失败，检查 `isHessianTopologyFixed()` 分支是否仍在每轮 rebuild solver/analyze pattern。
- 如果任何 runSim smoke 失败，回滚最近一个 task，不做“顺手修 solver 策略”的行为改动。

## 完成标准

- `NewtonSolver::solve()` 主循环不再内联 line-search scope、state evaluation、linear solve setup 的大块细节。
- `LineSearchScope` 是文件局部 RAII object，begin/end 成对由测试锁定。
- `runLineSearchStep()` 保留 backtracking initial trial energy reuse。
- `evaluateCurrentState()` 是唯一调用 `func_grad_hessian()` 并计算 convergence flags 的路径。
- Reduced-system setup / solve / expansion 是独立 helper。
- 所有 focused solver/simulation/IPC adapter tests 通过。

## 自审记录

- **Spec coverage:** 覆盖 solver 体系是否重构的判断落点：只处理 Newton hot path，不动 Knitro/Ipopt 和 public energy API。
- **Repo truth:** 计划引用的文件和测试 target 都存在：`NewtonSolver_gtest`、`implicitBackwardEulerTimeIntegrator_gtest`、`deformationModelEnergyMaxStep_gtest`、`embeddedSurfaceIPCPotentialEnergy_gtest`、`runIPCSim_gtest`。
- **Behavior risk:** 最大风险是 line-search scope 和 initial trial energy reuse；Task 1/2 已把这两件事作为 gate。
- **Non-goal clarity:** 不把 solver policy 改进、enum migration、Knitro/Ipopt cleanup 混入本轮。
- **Execution note:** 这个计划适合 4-5 个小提交执行；每个 task 后都跑 `NewtonSolver_gtest`，最后再跑跨 simulation/IPC focused tests。
