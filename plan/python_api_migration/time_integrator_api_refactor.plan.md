# Time Integrator API Refactor Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **状态日期：** 2026-06-01
> **适用范围：** C++ `simulation` time-integrator architecture + Python `pypgo.sim` dynamic step API.
> **执行约束：** 不改变 implicit Euler / TRBDF2 数值公式、Newton line-search / damping / max-step 语义、dynamic timestep acceptance policy、IPC active-set timing 或现有 `runIPCSim` 输出语义。本计划先把 residual energy 和 step lifecycle 服务化，再绑定 Python API。

**Goal:** 把 IBE 和 TRBDF2 迁移到统一的 dynamic step service，其中 residual energy 是自包含 stage problem，Python 通过 `DynamicSimulation.step()` / `run()` 驱动动态仿真。

**Architecture:** C++ 分为 `DynamicState`、IBE/TRBDF2 stage builder、`DynamicStepper` backend、run-sim facade、nanobind binding。**residual energy 和 model 聚合不新建类**：复用已有的 `NonlinearOptimization::PotentialEnergies`（在 `energy_api_refactor.plan.md` 中重命名为 `EnergySet`）做 energy 组合，复用 `PredefinedPotentialEnergies::QuadraticPotentialEnergy(A, l)` 表达 stage 的 `½xᵀAx + lᵀx` 项。每个 stage 的 residual problem = 一个 `EnergySet{ QuadraticEnergy(A_s, l_s), elastic, attachments, contact, floor... }`。IBE 是 single-stage backend，TRBDF2 是 two-stage backend；两者用同一套组合方式构造 stage residual，并返回统一 `DynamicStepResult`。

**Tech Stack:** C++17, Eigen sparse/dense, existing `PotentialEnergy` / `PotentialEnergies`(→`EnergySet`) / `QuadraticPotentialEnergy` / `LinearPotentialEnergy`，`solver_api_refactor.plan.md` 的 `minimize(problem, x0, NewtonOptions)` + `FixedVariables`（M3 落地前用 `EnergyOptimizer::minimize` Newton 路径过渡），nanobind, pytest, GoogleTest。

---

## 目标

建立一个可长期绑定到 Python 的 dynamic stepping 边界：

```cpp
namespace pgo::Simulation
{

enum class TimeIntegratorKind
{
  ImplicitEuler,
  TRBDF2,
};

struct DynamicState
{
  EigenSupport::VXd displacement;
  EigenSupport::VXd velocity;
  EigenSupport::VXd acceleration;
  std::uint64_t timestepId = 0;
  double time = 0.0;  // 仿真时间（B1：供 StepAwareEnergy::beginStep 的 StepState 使用）
};

struct DynamicStepResult
{
  DynamicState state;
  NonlinearOptimization::SolverResult solver;
  std::vector<NonlinearOptimization::SolverResult> stageResults;
  bool accepted = false;
};

struct DynamicStepRequest
{
  // 每帧变化量。immutable problem（mass / persistent terms / damping / solver /
  // fixed DOF 集合）在 stepper 构造时给定，不在每步重传/重拷贝。
  EigenSupport::VXd externalForce;
  std::optional<EigenSupport::VXd> fixedValues;  // 默认取当前 state 在 fixed DOF 上的值
};

class DynamicStepper
{
public:
  virtual ~DynamicStepper() = default;
  virtual DynamicStepResult step(
    const DynamicState &state,
    const DynamicStepRequest &request) = 0;
};

}  // namespace pgo::Simulation
```

Python 第一版：

```python
import pypgo as pgo

state = pgo.sim.DynamicState(
    displacement=u0,
    velocity=v0,
    acceleration=a0,
)

simulation = pgo.sim.DynamicSimulation(
    mass=mass,
    energy=total_energy,
    state=state,
    timestep=dt,
    integrator="trbdf2",
    damping=(mass_damping, stiffness_damping),
    solver=pgo.solver.NewtonOptions(max_iter=50, tol=1e-6),
)

frame = simulation.step(external_force=fext, fixed_dofs=fixed)
print(frame.displacement, frame.velocity, frame.acceleration)
print(frame.solver_result.status)
print(frame.stage_results)
```

不让 Python 用户看到：

- `TimeIntegrator::addGeneralImplicitForceModel()` / `clearGeneralImplicitForceModel()`；
- `TimeIntegrator::setqState()` + `doTimestep(updateq, verbose, printResidual)` 调用顺序；
- `ImplicitBackwardEulerEnergy(ImplicitBackwardEulerTimeIntegrator *)`；
- `TRBDF2TimeIntegratorEnergy(TRBDF2TimeIntegrator *, A, b)`；
- `stage` mutable member；
- `uRangeLow == uRangeHi` 这种 fixed DOF encoding；
- run-sim contact backend 每帧 `beginFrame/addForces/afterStep` 的内部顺序。

## 当前架构和数据流

当前 C++ dynamic 路径：

```text
runIPCSimLoop
  -> session.integrator->clearGeneralImplicitForceModel()
  -> update pulling target
  -> contactBackend.beginFrame(framei, runtimeConfig, context, session)
  -> contactBackend.addForces(framei, runtimeConfig, context, session)
  -> integrator->setqState(u, uvel, uacc)
  -> integrator->doTimestep(1, 3, 1)
  -> integrator->getq/getqvel/getqacc
  -> contactBackend.afterStep(framei, runtimeConfig, context, session)
  -> output.writeStateAndSurfaceFrame(framei, frameIndex, context, u, uvel, uacc, scale)
```

`TimeIntegrator` 当前职责过宽：

- owns dynamic state buffers: `q`, `qvel`, `qacc`, `q1`, `qvel1`, `qacc1`;
- owns external force `f_ext`;
- owns fixed DOF / bounds encoding: `uRangeLow`, `uRangeHi`, `fixedDOFs`;
- owns implicit energy list and per-model buffers/mappings: `implicitModelsAll`, `implicitModelsAll_K`, `implicitModelsAll_Kmaping`;
- owns constraints and solver option;
- owns `lastSolverResult`;
- also manages lifecycle flags: `generalForceModelChanged`, `constraintsChanged`.

IBE current flow:

```text
ImplicitBackwardEulerTimeIntegrator::tryTimestep
  -> assembleImplicitModels()
  -> updateD()
  -> updateA()
  -> updateb()
  -> initialize z
  -> solver->solve(needRenew, z, g, lambda, uRangeLow, uRangeHi, constraintsRangeLow, constraintsRangeHi, eulerEnergy, constraints, nIter, eps, verbose, solverConfigFilename, solverOption)
  -> accepted status?
  -> update q1/qvel1/qacc1
  -> proceedTimestep()
```

TRBDF2 current flow:

```text
TRBDF2TimeIntegrator::doTimestep
  -> assembleImplicitModels()
  -> updateD()
  -> updateA1(), updateb1()
  -> solve z1 with trEnergy
  -> update qy/qvely/qaccy
  -> updateA2(), updateb2()
  -> solve z2 with bdf2Energy
  -> update q1/qvel1/qacc1
  -> proceedTimestep()
```

核心问题：

- `ImplicitBackwardEulerEnergy` 通过 `intg*` 读取 `A/b/temp0/implicitModelsAll/hessianAll`；
- `TRBDF2TimeIntegratorEnergy` 通过 `intg*` 读取 `temp0/implicitModelsAll/hessianAll`，并引用 stage-specific `A/b`；
- residual energy 不是自包含 problem，而是 integrator mutable state 的视图；
- TRBDF2 stage result 没有明确 value object，调试只能从 mutable `stage` 和 `lastSolverResult` 推断；
- Python 如果直接绑定当前类，会暴露历史调用顺序和内部 lifecycle。

## 目标数学模型

所有 implicit stage 都统一为：

```math
J_s(x) = \frac12 x^T A_s x + \Phi(x) + l_s^T x
```

```math
\nabla J_s(x) = A_s x + \nabla\Phi(x) + l_s
```

```math
\nabla^2 J_s(x) = A_s + \nabla^2\Phi(x)
```

其中：

- `x` 是 stage solution displacement；
- `A_s` 是 mass/damping/time-step 贡献；
- `l_s` 是 linear term；
- `Phi(x)` 是 implicit energies 之和，包括 elastic、attachments、contact、floor 等。

### Implicit Euler

```math
A_{BE} = \frac{1}{h^2}M + \frac{1}{h}D_n
```

```math
b_{BE} = f_{ext} + \frac{1}{h}M v_n + A_{BE}u_n
```

```math
J_{BE}(x) = \frac12 x^T A_{BE}x + \Phi(x) - b_{BE}^T x
```

在统一 residual energy 里：

```math
l_{BE} = -b_{BE}
```

状态更新：

```math
u_{n+1}=x
```

```math
v_{n+1}=\frac{x-u_n}{h}
```

```math
a_{n+1}=\frac{v_{n+1}-v_n}{h}
```

### TRBDF2 stage 1

```math
\alpha = \frac{2}{\gamma h}
```

```math
A_1 = \alpha^2 M + \alpha D_n
```

```math
l_1 = -(2\alpha Mv_n + Ma_n + D_nv_n + f_{ext}) - A_1u_n
```

```math
J_1(x_1) = \frac12 x_1^TA_1x_1 + \Phi(x_1) + l_1^Tx_1
```

Stage intermediate state:

```math
u_y=x_1
```

```math
v_y=\alpha(x_1-u_n)-v_n
```

```math
a_y=\alpha^2(x_1-u_n)-2\alpha v_n-a_n
```

### TRBDF2 stage 2

Use the existing beta coefficients from `TRBDF2TimeIntegrator::updateCoeffs()`:

```math
A_2 = \beta_4 M + \beta_7D_n
```

```math
l_2 =
M(\beta_0u_n+\beta_1u_y+\beta_2v_n+\beta_3v_y)
+D_n(\beta_5u_n+\beta_6u_y)
-f_{ext}
-A_2u_n
```

```math
J_2(x_2) = \frac12 x_2^TA_2x_2 + \Phi(x_2) + l_2^Tx_2
```

Final update:

```math
u_{n+1}=x_2
```

```math
v_{n+1}=\beta_5u_n+\beta_6u_y+\beta_7(x_2-u_n)
```

```math
a_{n+1}=\beta_0u_n+\beta_1u_y+\beta_2v_n+\beta_3v_y+\beta_4(x_2-u_n)
```

## 设计决策

### 1. Stage residual 用 `EnergySet` + `QuadraticEnergy` 组合，不新建 residual energy 类

每个 implicit stage 的 residual problem 表达为：

```math
J_s(x)=\frac12x^TA_sx+\Phi(x)+l_s^Tx
```

其中 `½xᵀA_s x + l_sᵀx` 直接用已有的 `PredefinedPotentialEnergies::QuadraticPotentialEnergy(A_s, l_s)`（见 `genericPotentialEnergies/quadraticPotentialEnergy.h`：`// the energy is 1/2 xT A x + bT x`），`Φ(x)` 是 implicit energies 之和（elastic / attachments / contact / floor）。整个 stage residual = 一个 `EnergySet`：

```text
EnergySet{ QuadraticEnergy(A_s, l_s), elastic, attachments, contact, floor... }
```

IBE 的 legacy `b` 通过 `l_s = -b` 转换；TRBDF2 的 legacy `b1/b2` 直接作为 `l_s`。`QuadraticEnergy` 的 `b` 参数吸收 IBE 的 `-b^Tx` 与 TRBDF2 的 `+b^Tx` 符号差异，公共表达里只剩统一的 `l_s`。

**不新建 `ImplicitResidualEnergy` 类**：它原本要做的 `½xᵀAx + Φ(x) + lᵀx` 的 func/grad/hess + fused / maxStep / 拓扑判定转发，`EnergySet` 已全部具备（见 §2）。stage builder 只负责算出 `A_s`/`l_s` 并组装 `EnergySet`，不再持有 `TimeIntegrator *`。

### 2. model 聚合复用 `EnergySet`（原 `PotentialEnergies`），不新建 `ImplicitModelAssembly`

`src/core/nonlinearOptimization/potentialEnergies.{h,cpp}` 的 `PotentialEnergies`（在 `energy_api_refactor.plan.md` 中重命名为 `EnergySet`）已经提供本 plan 原先想给 `ImplicitModelAssembly` 的全部能力，且更通用：

- `init()` 构建合并 `hessianAll` 模板 + per-energy `small2Big` mapping；
- `func / gradient / hessianInPlace / gradient_hessian / hessian / func_grad_hessian`；
- `computeMaxStepLimit` 经跨来源内联 min（所有子节点通过 `StepConstraintSink` 各自 report）合并；
- `isHessianTopologyFixed()` 聚合；
- `beginLineSearch / endLineSearch`（`LineSearchAwareEnergy`）—— IPC active-set 在 line-search 段内冻结的生命周期已内置；
- 支持 per-energy DOF 子集（比"全 DOF"更通用），固定拓扑用预分配 buffer + mapping，非固定拓扑走 safe one-shot，保留 IPC active-set 行为。

**唯一需要补的一处**：`EnergySet::func_grad_hessian` 当前实现是 `gradient_hessian(...); return func(x);`（`energySet.cpp:327`），对非固定拓扑（IPC）会触发两次 active-set 构建；而手写 helper 是单次 fused `func_grad_hessian`（`implicitBackwardEulerTimeIntegratorHelper.cpp:144`，注释 "1 buildActiveSet"）。需把 `EnergySet::func_grad_hessian` 改成对每个 term 调一次 `func_grad_hessian`、一次累加 value/grad/hess。见 Task T2。

per-term 的 damping 元数据（`stiffnessDamping`/`massDamping`）不是 energy 的属性，用一个轻量 value object 承载，只服务于 damping 装配（§3）；`Φ` 的聚合直接拿 `term.energy` 喂 `EnergySet`：

```cpp
struct ImplicitModelTerm
{
  NonlinearOptimization::PotentialEnergy_p energy;
  double stiffnessDamping = 0.0;
  double massDamping = 0.0;
};
```

### 3. Damping 是独立 assembly

`D_n` construction is a separate operation，输入是 `ImplicitModelTerm` 列表 + 全局 mass，独立于 residual energy：

```cpp
EigenSupport::SpMatD assembleRayleighDamping(
  const std::vector<ImplicitModelTerm> &terms,
  const EigenSupport::SpMatD &mass,
  EigenSupport::ConstRefVecXd state);
```

Rules（与现有 `TimeIntegrator::updateD()` 行为一致）：

- mass damping contributes `massDamping * M`（所有 term 共用全局 `mass`）；
- stiffness damping contributes `stiffnessDamping * Hessian_i(u_n)`；
- non-fixed-topology energies are skipped for damping Hessian, preserving existing behavior.

### 4. IBE/TRBDF2 是 backend，用同一套 `EnergySet` 组合构造 stage residual

没有 residual energy 子类，只有两个 stage builder，各自算 `A_s`/`l_s` 并组装同一种 `EnergySet`：

```text
ImplicitEulerStageBuilder  -> 1 个 stage
TRBDF2StageBuilder         -> 2 个 stage（γ<1）/ 1 个 stage（γ==1）
```

`TRBDF2` returns two stage solver results。Python exposes both through `frame.stage_results`；`frame.solver_result` is the final stage result unless a previous stage failed in a non-accepted way。

### 5. New service rejects constraints in first Python dynamic API

The existing `TimeIntegratorSolver` has IPOPT/Knitro/constraints paths. Python dynamic API M6 first version targets the run-sim main path:

```text
unconstrained dynamic implicit solve + fixed DOFs + Newton
```

Constraints remain covered by `constraints_api_refactor.plan.md` and can later plug into the same stage problem shape through a constrained solver backend. This plan must not remove legacy constrained C++ code.

### 6. Contact lifecycle stays outside residual energy

Residual energy only evaluates `Phi(x)`. Per-frame contact lifecycle belongs to step orchestration:

```text
begin_step(time, timestep, previous_x)
refresh_active_set(initial_guess)
solve residual problem
after_step / obstacle advancement
```

注意：line-search 段内（单次 Newton 迭代内）的 active-set 冻结生命周期已经存在，即 `LineSearchAwareEnergy` 的 `beginLineSearch / endLineSearch`，由 `EnergySet` 自动 `dynamic_cast` 转发——复用 `EnergySet` 即免费获得。需要新设计的只是 per-frame（跨 timestep）的 contact 生命周期，它属于 step orchestration。

First C++ refactor preserves current `runIPCSim` contact timing. A later contact plan migrates `RunIPCSimContactBackend` to `StepAwareEnergy` / `StatefulContactEnergy`。contact energies（IPC 和 legacy penalty）作为持久 energy 对象存入 `DynamicProblem.persistentTerms`，随 stepper 构造一次，不 per-frame add/remove（D1 已拍板，见「设计决策（已拍板）」）。

### 7. 跨 plan 依赖，不 fork

本 plan 必须建立在以下姊妹 plan 的产物上，而不是平行造一套：

- `solver_api_refactor.plan.md`：长期 C++ 求解入口是 `minimize(problem, x0, NewtonOptions)`，fixed DOF 用 first-class 的 `FixedVariables`；`SolverControl` struct 被本 plan 直接复用于 `DynamicProblem::solver`（C1），不重复定义。本 plan 的 `solveStageProblem` 走这条；M3 落地前可临时用 `EnergyOptimizer::minimize` 的 Newton 路径（它已用 `xlow[i]==xhi[i]` canonical 化 fixed DOF），但不把 `xlow==xhi` 编码泄漏到本 plan 的公共 API。
- `energy_api_refactor.plan.md`：`PotentialEnergies` → `EnergySet`；`LinearPotentialEnergy` / `QuadraticPotentialEnergy` 改 owning-by-value 并暴露为 `pgo.energy.LinearEnergy` / `QuadraticEnergy`。本 plan 直接消费这些类，不引入并行的聚合 / 二次型类。
- **`contact_api_refactor.plan.md` C2**（`StepAwareEnergy` + `StatefulContactEnergy`）：T7.3/T7.4 的 `dynamic_cast<StepAwareEnergy*>` / `dynamic_cast<StatefulContactEnergy*>` 调用依赖 C2 产出的 `stepAwareEnergy.h`。C2 必须在 T7 之前完成，或 T7 在两者并行时做条件编译过渡。
- 复用 `acceptsDynamicSolveStatus`（已在 `solverResult.h:41` / `solverResult.cpp:101`，语义已与本 plan 需要的一致），不重新定义。

### 全局执行顺序（跨 plan）

本 plan 是**全局第 5 个（最后）**（Contact plan 完成后启动）。执行顺序：

```
1. Solver plan          → 产出 SolverControl, minimize, FixedVariables
2. Constraints plan     → 产出 ConstraintSet（Time Integrator 暂不消费，但提前完成）
3. Implicit Surface plan → 自包含，不交互
4. Contact plan         → 产出 StepAwareEnergy, StatefulContactEnergy, StepState
5. Time Integrator plan  ← 本 plan
```

**为什么最后**：本 plan 消费 Solver（`SolverControl` + `minimize`）+ Contact（`StepAwareEnergy::beginStep` + `StatefulContactEnergy::refreshActiveSet`）。两者都就绪后，T7 不需要条件编译过渡，T9 直接对已迁移的 `beginStep/refreshActiveSet` 流程包装。

## 设计决策（已拍板）

- **D1（EnergySet 跨帧生命周期 / contact terms）**：两个持久容器，构造一次，不 rebuild：
  - **FixedHessianTemplateEnergySet**：`QuadraticEnergy(A_s, l_s)` + elastic + attachments + legacy penalty contact（`PointPenetrationEnergy`、`PointTrianglePairCouplingEnergyWithCollision`，均为固定拓扑：pair 集合构造时确定，per-frame 只改值）。
  - **DynamicHessianTemplateEnergySet**：IPC（`EmbeddedSurfaceIPCPotentialEnergy`，显式 `isHessianTopologyFixed()=0`），动态拓扑，always triplet scatter。
  - 两个容器均**构造一次，永不 per-frame rebuild**。旧有 `clearGeneralImplicitForceModel + addGeneralImplicitForceModel` 模式完全消除。per-frame contact 状态更新（legacy penalty 的 `computeClosestPosition(u)`）由 step orchestration 在 solve 前调用，不影响容器。
  - `DynamicStepRequest` 不含 `transientTerms`；contact energies 全部归入 `DynamicProblem.persistentTerms`。
  - **⚠ 拓扑分类的阶段依赖（B2）**：以上分类基于**当前 `legacyPenaltyContact.cpp` 实现**——`activeExternalEnergy_` / `activeSelfEnergy_` 构造时 pair 集固定，per-frame 只改 `contactStatus` 值，拓扑不变。contact plan C6 完成后，`SampledPenaltyContactEnergy::refreshActiveSet()` 会重建 active pair 集（pair 数目变化），导致 `isHessianTopologyFixed()` 应返回 `0`——届时 sampled penalty 也应移入 `DynamicHessianTemplateEnergySet`。此次 C++ refactor 使用当前实现；contact plan 落地时须同步更新本决策。

- **D2（`QuadraticEnergy` 原地更新）**：给 `QuadraticPotentialEnergy` 新增两个 setter；stepper 持有 `shared_ptr<QuadraticPotentialEnergy>`（non-const）每步调用，`EnergySet` 通过 `shared_ptr<const PotentialEnergy>` 只读访问同一对象。`cache` 仅含临时 buffer，无需清理。
  ```cpp
  void setLinearTerm(EigenSupport::VXd b);            // l_s 每步变
  void setAValues(const EigenSupport::SpMatD &A);     // A_s 值变但 pattern 不变（变步长时）
  ```

- **D3（stage residual 组合形态）**：**扁平 `EnergySet`**，不嵌套。TRBDF2 两个 stage 各持一个独立的扁平 `EnergySet`（各含各自的 `QuadraticEnergy`）。结合 D2 setter，`QuadraticEnergy` 对象持久存在，每步 solve 前调 `setLinearTerm(l_s)` 原地更新；`hessianAll` 模板只建一次，不因 step 推进而 rebuild。

- **D4（stepper 状态归属）**：持久态（`EnergySet`、`QuadraticEnergy`、buffer）**放在 stepper 内**。`ImplicitEulerStepper` / `TRBDF2Stepper` 构造时接收 `DynamicProblem`，内部建好持久 energy 对象并持有。Python binding（`PyDynamicSimulation`）直接包 stepper，不引入独立 core 层。

- **D5（fixed DOF index 不变性）**：`fixedDofs` index 集合放在 `DynamicProblem`（构造时确定，**immutable**）。`DynamicStepRequest` 仅含 `fixedValues`（prescribed 位移值，per-step 变）。需要换集合时重建 `DynamicProblem` 触发一次 re-init，语义清晰，避免 per-step solver 重配。

## File Map

### 新增

- `src/core/simulation/dynamicState.h`
- `src/core/simulation/dynamicStepOptions.h`（含 `ImplicitModelTerm`、`DynamicProblem`、`DynamicStepRequest`；`SolverControl` 来自 `solver_api_refactor.plan.md` 的 `optimizationService.h`，不重复定义）
- `src/core/simulation/rayleighDampingAssembly.h`
- `src/core/simulation/rayleighDampingAssembly.cpp`
- `src/core/simulation/stageResidual.h`（`StageResidualHandle`、`initStageResidual`（构造时建持久 EnergySet）、`prepareStageResidual`（per-step 原地更新）、`ImplicitStageProblem` value object）
- `src/core/simulation/stageResidual.cpp`
- `src/core/simulation/implicitEulerStageBuilder.h`
- `src/core/simulation/implicitEulerStageBuilder.cpp`
- `src/core/simulation/trbdf2StageBuilder.h`
- `src/core/simulation/trbdf2StageBuilder.cpp`
- `src/core/simulation/dynamicStepper.h`
- `src/core/simulation/dynamicStepper.cpp`
- `src/core/simulation/dynamicStepService.h`
- `src/core/simulation/dynamicStepService.cpp`
- `src/python/pypgo/bindings/simulation_bindings.cpp`
- `tests/src/core/dynamicStepper_gtest.cpp`
- `tests/pypgo/test_dynamic_stepper.py`

### 修改

- `src/core/simulation/CMakeLists.txt`：编入新增 simulation service files。
- `src/core/nonlinearOptimization/energySet.cpp`：把 `func_grad_hessian` 改成对每个 term 调一次 `func_grad_hessian` 的单遍 fused 实现（保留固定拓扑的 mapping 路径），消除非固定拓扑 IPC 的二次 active-set 构建。见 §2 / Task T2。
- `src/core/simulation/implicitBackwardEulerTimeIntegratorHelper.h/.cpp`：**删除**——`ImplicitBackwardEulerEnergy` 由 `EnergySet{ QuadraticEnergy(A,−b), Φ }` 取代，不再持有 `ImplicitBackwardEulerTimeIntegrator *`。
- `src/core/simulation/TRBDF2TimeIntegratorHelper.h/.cpp`：**删除**——`TRBDF2TimeIntegratorEnergy` 由 `EnergySet{ QuadraticEnergy(A_s, b_s), Φ }` 取代，不再持有 `TRBDF2TimeIntegrator *`。
- `src/core/simulation/implicitBackwardEulerTimeIntegrator.h/.cpp`：改为使用 stage builder / service，保持 legacy class API source-compatible。
- `src/core/simulation/TRBDF2TimeIntegrator.h/.cpp`：改为使用 TRBDF2 stage builder / service，保留 legacy class API source-compatible。
- `src/tools/sim/runIPCSim/app/session.h/.cpp`：session 持有 `DynamicState` / `DynamicStepper` facade，迁移期可保留 legacy integrator adapter。
- `src/tools/sim/runIPCSim/app/loop.cpp`：改成调用 dynamic step facade，并保持 output/restart/contact 顺序。
- `src/tools/sim/runIPCSim/contact/contactBackend.h`：新增可选 step-service hook 或 adapter，避免直接依赖 `ImplicitBackwardEulerTimeIntegrator`。
- `src/python/pypgo/CMakeLists.txt`：编入 `simulation_bindings.cpp`，链接 `simulation`、`nonlinearOptimization`、`constraintPotentialEnergies`。
- `src/python/pypgo/bindings/module.cpp`：注册 simulation bindings。
- `pypgo/sim.py`：新增 `DynamicState`、`DynamicFrame`、`DynamicOptions`、`DynamicSimulation`。
- `pypgo/__init__.py`：M6 完成后保持 `sim` public module。
- `tests/src/core/CMakeLists.txt`：新增 `dynamicStepper_gtest` target。
- `plan/python_api_migration/milestones.md`：更新 M6 细节。
- `plan/python_api_migration/api_coverage.md`：更新 dynamic loop/time-integrator 条目。

### 不动

- `NewtonSolver` 数值路径。
- Existing `PotentialEnergy` math semantics.
- Existing IBE/TRBDF2 formulas.
- Existing `runIPCSim` output layout.
- Existing static solve path.
- Existing IPOPT/Knitro code paths, except adapter call sites needed for compilation.

## Task 拆分

### Task T0: Characterization audit and red tests

**Files:**

- Modify: `tests/src/core/implicitBackwardEulerTimeIntegrator_gtest.cpp`
- Modify: `tests/src/core/CMakeLists.txt`
- Create: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T0.1: Add formula characterization tests for IBE coefficients**

Add a C++ test fixture with 2 DOFs, diagonal mass, quadratic energy, nonzero external force, nonzero velocity, and `dt=0.25`. The test computes expected:

```cpp
const double h = 0.25;
const EigenSupport::SpMatD expectedA =
  (1.0 / (h * h)) * mass + (1.0 / h) * damping;
const EigenSupport::VXd expectedLinear =
  -(externalForce + (1.0 / h) * mass * velocity + expectedA * displacement);
```

Then it constructs the new `ImplicitEulerStageBuilder` and verifies `stage.energy` matches:

```cpp
EXPECT_LT((EigenSupport::MXd(stage.A) - EigenSupport::MXd(expectedA)).norm(), 1e-12);
EXPECT_LT((stage.linear - expectedLinear).norm(), 1e-12);
```

Expected before Task T2/T3 implementation: compile failure because `ImplicitEulerStageBuilder` does not exist.

- [ ] **Step T0.2: Add formula characterization tests for TRBDF2 coefficients**

Use `gamma=0.5`, `dt=0.25`, diagonal mass, nonzero state and force. Verify:

```cpp
const double alpha = 2.0 / (gamma * h);
const EigenSupport::SpMatD expectedA1 =
  alpha * alpha * mass + alpha * damping;
```

Verify stage 1 `linear` equals:

```cpp
-(2.0 * alpha * mass * velocity + mass * acceleration + damping * velocity + externalForce)
  - expectedA1 * displacement;
```

Then solve or inject a known `x1`, compute `uy/vy/ay`, and verify stage 2:

```cpp
const EigenSupport::SpMatD expectedA2 = beta4 * mass + beta7 * damping;
const EigenSupport::VXd expectedLinear2 =
  mass * (beta0 * displacement + beta1 * uy + beta2 * velocity + beta3 * vy)
  + damping * (beta5 * displacement + beta6 * uy)
  - externalForce
  - expectedA2 * displacement;
```

Expected before Task T4 implementation: compile failure because `TRBDF2StageBuilder` does not exist.

- [ ] **Step T0.3: Add parity tests against legacy classes**

For IBE:

```cpp
ImplicitBackwardEulerTimeIntegrator legacy(mass, energy, 0.0, 0.0, dt, 20, 1e-10);
DynamicProblem problem = makeEquivalentProblem(mass, energy, dt, /*damping*/ 0.0, 0.0, 20, 1e-10);
DynamicStepRequest request = makeRequest(fext);   // 默认 fixedValues / 无 transient terms
DynamicState state = makeState(u, v, a);

const SolverResult legacyResult = legacy.tryTimestep(1, 0, 0);
auto stepper = makeDynamicStepper(TimeIntegratorKind::ImplicitEuler, problem);
const DynamicStepResult serviceResult = stepper->step(state, request);

EXPECT_EQ(serviceResult.solver.status, legacyResult.status);
EXPECT_LT((serviceResult.state.displacement - legacyQ).norm(), 1e-10);
EXPECT_LT((serviceResult.state.velocity - legacyV).norm(), 1e-10);
EXPECT_LT((serviceResult.state.acceleration - legacyA).norm(), 1e-10);
```

For TRBDF2, compare `q/qvel/qacc` after `doTimestep(1, 0, 0)` with `DynamicStepper` result and assert `stageResults.size() == 2` when `gamma < 1`.

Expected before Task T6 implementation: compile failure.

- [ ] **Step T0.4: Verify current tests still pass before refactor**

Run:

```bash
cmake --build --preset base_no_mkl_release --target implicitBackwardEulerTimeIntegrator_gtest
./build/base_no_mkl/tests/src/core/implicitBackwardEulerTimeIntegrator_gtest
```

Expected: existing tests pass before enabling new failing tests in the build target.

### Task T1: Add value objects and target build wiring

**Files:**

- Create: `src/core/simulation/dynamicState.h`
- Create: `src/core/simulation/dynamicStepOptions.h`
- Modify: `src/core/simulation/CMakeLists.txt`

- [ ] **Step T1.1: Define `TimeIntegratorKind`, `DynamicState`, and validation helpers**

Target API:

```cpp
namespace pgo::Simulation
{

enum class TimeIntegratorKind
{
  ImplicitEuler,
  TRBDF2,
};

struct DynamicState
{
  EigenSupport::VXd displacement;
  EigenSupport::VXd velocity;
  EigenSupport::VXd acceleration;
  std::uint64_t timestepId = 0;
  double time = 0.0;  // 仿真时间，供 StepAwareEnergy::beginStep 的 StepState 使用
};

void validateDynamicState(const DynamicState &state, int numDofs);

}  // namespace pgo::Simulation
```

Validation:

- `numDofs > 0`;
- all three vectors have `numDofs`;
- all values are finite;
- `time` is finite.

- [ ] **Step T1.2: Define `ImplicitModelTerm`、immutable `DynamicProblem`、per-step `DynamicStepRequest`**

把"问题不变量"和"每帧变化量"分开，避免每步重传/重拷贝 mass（per-term damping 系数随 `ImplicitModelTerm` 走，不再有全局 `massDamping`/`stiffnessDamping`）。`solver` 字段复用 solver plan 的 `NonlinearOptimization::SolverControl`，不重复定义同构 struct：

```cpp
// #include "nonlinearOptimization/optimizationService.h"  // for SolverControl

namespace pgo::Simulation
{

struct ImplicitModelTerm
{
  NonlinearOptimization::PotentialEnergy_p energy;
  double stiffnessDamping = 0.0;
  double massDamping = 0.0;
};

// immutable across steps
struct DynamicProblem
{
  EigenSupport::SpMatD mass;
  std::vector<ImplicitModelTerm> persistentTerms;  // elastic / attachments / contact
  std::vector<int> fixedDofs;                       // immutable across steps（D5）
  double timestep = 0.0;
  NonlinearOptimization::SolverControl solver;      // 复用 solver plan SolverControl，不新建同构 struct
};

// per-step
struct DynamicStepRequest
{
  EigenSupport::VXd externalForce;
  std::optional<EigenSupport::VXd> fixedValues;     // 默认取 state 在 fixedDofs 上的值
  // contact energies 已在 DynamicProblem.persistentTerms 持久持有（D1），此处无 transientTerms
};

void validateDynamicProblem(const DynamicProblem &problem, int numDofs);
void validateDynamicStepRequest(const DynamicStepRequest &request, const DynamicProblem &problem, int numDofs);

}  // namespace pgo::Simulation
```

Validation:

- mass is square and has `numDofs` rows；
- `timestep > 0`；
- 每个 term 的 damping 参数 finite 且非负；
- `fixedDofs` 排序/canonical 化；duplicate/out-of-range 抛 `std::invalid_argument`；
- `externalForce.size() == numDofs`；`fixedValues`（若给）尺寸与 `fixedDofs` 一致。

- [ ] **Step T1.3: Wire headers into `simulation` target**

Modify `src/core/simulation/CMakeLists.txt`:

```cmake
set(SIMULATION_HEADERS
  timeIntegrator.h
  timeIntegratorSolver.h
  dynamicState.h
  dynamicStepOptions.h
  rayleighDampingAssembly.h
  stageResidual.h
  implicitEulerStageBuilder.h
  trbdf2StageBuilder.h
  dynamicStepper.h
  dynamicStepService.h
  implicitBackwardEulerTimeIntegratorHelper.h
  implicitBackwardEulerTimeIntegrator.h
  TRBDF2TimeIntegratorHelper.h
  TRBDF2TimeIntegrator.h
)
```

Run:

```bash
cmake --build --preset base_no_mkl_release --target simulation
```

Expected: `simulation` target builds.

### Task T2: 复用 `EnergySet` 聚合 + 修 fused IPC 路径

不新建聚合器。本 task 确认 `EnergySet`（原 `PotentialEnergies`）作为 `Φ` 的聚合，并补上单遍 fused 路径，使复用与手写 helper 性能等价。

**Files:**

- Modify: `src/core/nonlinearOptimization/energySet.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T2.1: 确认 `Φ` 聚合走 `EnergySet`，记录映射**

从 `DynamicProblem.persistentTerms` 取 `term.energy` 列表，构造一个扁平 `EnergySet`（`addPotentialEnergy` + `init()`，或 energy_api 重命名后的等价构造）；D1 已拍板，无 `transientTerms`。不写新的 `func/gradient/hessian/computeMaxStepLimit/isHessianTopologyFixed`——全部由 `EnergySet` 提供；`beginLineSearch/endLineSearch` 由 `EnergySet` 自动 `dynamic_cast` 转发。

- [ ] **Step T2.2: 修 `EnergySet::func_grad_hessian` 的单遍 fused 实现**

现状（`energySet.cpp:327`）：

```cpp
double EnergySet::func_grad_hessian(x, grad, hess) const
{
  gradient_hessian(x, grad, hess);   // 非固定拓扑 term 这里 build 一次 active set
  return func(x);                    // func 又触发一次 -> IPC 双重 active-set 构建
}
```

改成对每个 term 调一次 `func_grad_hessian`、一次累加 value/grad/hess（固定拓扑仍走 `hessianMatrixMappings` + `addSmallToBig`；非固定拓扑走 term 自己的 `func_grad_hessian` 再 scatter）。这是 §2 唯一需要补的点，且惠及所有 `EnergySet` 调用方。

- [ ] **Step T2.3: 加 fused 路径 + active-set 计数 parity 测试**

Tests：

- 两个固定拓扑二次 term：`func_grad_hessian` 的 value/grad/hess 与分开调用一致；
- 一个固定拓扑 + 一个"计数 stub"非固定拓扑 term：`func_grad_hessian` 只触发该 term 的 `func_grad_hessian` **一次**（计数器==1），不再额外触发 `func` / `gradient_hessian`；
- `computeMaxStepLimit` 经跨来源内联 min（所有子节点通过 `StepConstraintSink` 各自 report）返回绑定 alpha；
- 改动前后固定拓扑场景的 value/grad/hess 数值不变。

Run:

```bash
cmake --build --preset base_no_mkl_release --target dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest --gtest_filter=EnergySetFused*
```

Expected: fused 路径测试通过；非固定拓扑 active-set 构建计数为 1。

### Task T3: Add Rayleigh damping assembly

**Files:**

- Create: `src/core/simulation/rayleighDampingAssembly.h`
- Create: `src/core/simulation/rayleighDampingAssembly.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T3.1: Implement damping assembly**

Target API:

```cpp
EigenSupport::SpMatD assembleRayleighDamping(
  const std::vector<ImplicitModelTerm> &terms,
  const EigenSupport::SpMatD &mass,
  EigenSupport::ConstRefVecXd state);
```

Formula:

```math
D_n = \sum_i m_i M + \sum_i k_i \nabla^2\Phi_i(u_n)
```

Current-code compatibility（对齐 `TimeIntegrator::updateD()`）：

- all terms use the global `mass` for mass damping（贡献 `massDamping * M`）;
- fixed-topology terms contribute stiffness damping（`stiffnessDamping * hessianInPlace(u_n)`）;
- non-fixed-topology terms do not contribute stiffness damping。

- [ ] **Step T3.2: Add damping tests**

Tests:

- mass damping with identity mass returns `massDamping * I`;
- stiffness damping with quadratic energy returns `stiffnessDamping * K`;
- non-fixed-topology energy is skipped for stiffness damping;
- zero damping returns sparse zero matrix with compatible dimensions.

Run:

```bash
cmake --build --preset base_no_mkl_release --target dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest --gtest_filter=RayleighDampingAssembly*
```

Expected: all `RayleighDampingAssembly*` tests pass.

### Task T4: stage residual via `EnergySet` + `QuadraticEnergy` 组合

不新建 residual energy 类。D3 已拍板：`EnergySet` 构造一次，`QuadraticEnergy` 原地更新。本 task 把 stage residual 的 **construction** 和 **per-step update** 明确分开为两个 helper，消除 B3（每步重建 EnergySet 违背 D3）。

**Files:**

- Create: `src/core/simulation/stageResidual.h`
- Create: `src/core/simulation/stageResidual.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T4.1: Define `StageResidualHandle` 和诊断字段**

```cpp
// 构造时建立一次，stepper 持有
struct StageResidualHandle
{
  std::shared_ptr<NonlinearOptimization::EnergySet> energySet;
  std::shared_ptr<PredefinedPotentialEnergies::QuadraticPotentialEnergy> stageQuad;
};

// 每步传给 solveStageProblem 的轻量描述（不含 EnergySet ownership）
struct ImplicitStageProblem
{
  const NonlinearOptimization::EnergySet *energySet;  // 非拥有，指向 handle.energySet
  EigenSupport::VXd initialGuess;
  EigenSupport::SpMatD A;      // 诊断/公式校验用
  EigenSupport::VXd linear;    // 诊断/公式校验用
};
```

- [ ] **Step T4.2: Implement `initStageResidual`（构造时调一次）**

```cpp
StageResidualHandle initStageResidual(
  const EigenSupport::SpMatD &A_initial,
  const EigenSupport::VXd &l_initial,
  const std::vector<ImplicitModelTerm> &terms);  // DynamicProblem.persistentTerms
```

行为：

- 构造 `shared_ptr<QuadraticPotentialEnergy> stageQuad = make_shared<QuadraticPotentialEnergy>(A_initial, l_initial)`；
- 组装扁平 `EnergySet{ stageQuad, terms[i].energy... }`（D3：扁平，不嵌套）；
- 返回 `StageResidualHandle{ energySet, stageQuad }`，`hessianAll` 模板在此建立，后续不 rebuild。

- [ ] **Step T4.3: Implement `prepareStageResidual`（per-step 调用）**

```cpp
ImplicitStageProblem prepareStageResidual(
  StageResidualHandle &handle,
  EigenSupport::SpMatD A_s,
  EigenSupport::VXd l_s,
  EigenSupport::VXd initialGuess);
```

行为：

- 调 `handle.stageQuad->setLinearTerm(std::move(l_s))`（每步 l_s 变）；
- 若 A_s 与上步不同（变步长时），调 `handle.stageQuad->setAValues(A_s)`；
- 返回 `ImplicitStageProblem{ handle.energySet.get(), initialGuess, A_s, l_s }`（无新分配，无 EnergySet rebuild）。

- [ ] **Step T4.4: Add stage residual tests**

Tests：

- `initStageResidual` 只构造一次 EnergySet；`hessianAll` 模板地址在多次 `prepareStageResidual` 后不变；
- 2D 二次 fixture：`energySet->func_grad_hessian(x)` 等于 `½xᵀAx + Φ(x) + lᵀx`（有限差分校验）；
- `prepareStageResidual` 用不同 `l_s` 更新后，gradient 正确反映新线性项；
- `func_grad_hessian` 命中 T2.2 的单遍 fused 路径；
- 任一 term 非固定拓扑时 `energySet->isHessianTopologyFixed()` 为 false；
- `computeMaxStepLimit()` 经 `EnergySet` 合并。

Run:

```bash
cmake --build --preset base_no_mkl_release --target dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest --gtest_filter=StageResidual*
```

Expected: all `StageResidual*` tests pass.

### Task T5: Add IBE stage builder and updater

**Files:**

- Create: `src/core/simulation/implicitEulerStageBuilder.h`
- Create: `src/core/simulation/implicitEulerStageBuilder.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T5.1: 复用 T4.1 的 `ImplicitStageProblem`**

stage problem value object 已在 T4.1 定义（`energy` 是组合好的 `EnergySet`）。本 task 不再重复定义。

- [ ] **Step T5.2: Implement IBE coefficient calculator**

Builder 是纯函数，只计算 `A_s` / `l_s` / `initialGuess`，不构造或重建 EnergySet（EnergySet 由 stepper 持有，通过 T4.3 `prepareStageResidual` 更新）：

```cpp
struct IBEStageCoefficients
{
  EigenSupport::SpMatD A;
  EigenSupport::VXd linear;
  EigenSupport::VXd initialGuess;
};

class ImplicitEulerStageBuilder
{
public:
  IBEStageCoefficients compute(
    const DynamicState &state,
    const DynamicProblem &problem,
    const DynamicStepRequest &request,
    const EigenSupport::SpMatD &damping) const;
};
```

Rules:

- `A = M / h^2 + D / h`；
- `linear = -(f_ext + M v / h + A u)`；
- `initialGuess = state.displacement` by default；velocity extrapolation 落地后 `initialGuess = state.displacement + state.velocity * timestep`。
- 调用方（stepper）拿到 `coeffs` 后调 `prepareStageResidual(handle, coeffs.A, coeffs.linear, coeffs.initialGuess)`（T4.3）原地更新。

- [ ] **Step T5.3: Implement IBE state update**

Target API:

```cpp
DynamicState updateImplicitEulerState(
  const DynamicState &state,
  EigenSupport::ConstRefVecXd solution,
  double timestep);
```

Formula:

```cpp
next.displacement = solution;
next.velocity = (solution - state.displacement) / timestep;
next.acceleration = (next.velocity - state.velocity) / timestep;
next.timestepId = state.timestepId + 1;
next.time = state.time + timestep;  // B1：时间推进，供下步 StepAwareEnergy::beginStep 使用
```

- [ ] **Step T5.4: Add IBE formula and parity tests**

Tests:

- `A` and `linear` match formulas from T0.1;
- state update matches formula;
- single-step solve on a 1D quadratic matches legacy `ImplicitBackwardEulerTimeIntegrator`.

Run:

```bash
cmake --build --preset base_no_mkl_release --target dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest --gtest_filter=ImplicitEuler*
```

Expected: all `ImplicitEuler*` tests pass.

### Task T6: Add TRBDF2 stage builder and updater

**Files:**

- Create: `src/core/simulation/trbdf2StageBuilder.h`
- Create: `src/core/simulation/trbdf2StageBuilder.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T6.1: Define TRBDF2 coefficients**

Target API:

```cpp
struct TRBDF2Coefficients
{
  double gamma = 0.5;
  double alpha = 0.0;
  double beta[8] = {};
};

TRBDF2Coefficients computeTRBDF2Coefficients(double gamma, double timestep);
```

Validation:

- `0 < gamma <= 1`;
- `timestep > 0`.

Coefficients must match existing `TRBDF2TimeIntegrator::updateCoeffs()`.

- [ ] **Step T6.2: Implement stage 1 builder and update**

Target API:

Builder 是纯函数，只计算 `A_s` / `l_s` / `initialGuess`，不构造或重建 EnergySet（同 T5.2 设计原则）：

```cpp
struct TRBDF2StageCoefficients
{
  EigenSupport::SpMatD A;
  EigenSupport::VXd linear;
  EigenSupport::VXd initialGuess;
};

class TRBDF2StageBuilder
{
public:
  TRBDF2StageCoefficients computeStage1(
    const DynamicState &state,
    const DynamicProblem &problem,
    const DynamicStepRequest &request,
    const EigenSupport::SpMatD &damping,
    const TRBDF2Coefficients &coeffs) const;

  TRBDF2IntermediateState updateAfterStage1(
    const DynamicState &state,
    EigenSupport::ConstRefVecXd stage1Solution,
    const TRBDF2Coefficients &coeffs) const;
};
```

Stage 1:

```cpp
A1 = coeffs.alpha * coeffs.alpha * mass + coeffs.alpha * damping;
linear1 = -(2 * coeffs.alpha * mass * velocity + mass * acceleration + damping * velocity + externalForce)
  - A1 * displacement;
```

Intermediate:

```cpp
uy = x1;
vy = coeffs.alpha * (x1 - displacement) - velocity;
ay = coeffs.alpha * coeffs.alpha * (x1 - displacement) - 2 * coeffs.alpha * velocity - acceleration;
```

- [ ] **Step T6.3: Implement stage 2 builder and final update**

Target API:

```cpp
TRBDF2StageCoefficients computeStage2(
  const DynamicState &state,
  const TRBDF2IntermediateState &intermediate,
  const DynamicProblem &problem,
  const DynamicStepRequest &request,
  const EigenSupport::SpMatD &damping,
  const TRBDF2Coefficients &coeffs) const;

DynamicState updateAfterStage2(
  const DynamicState &state,
  const TRBDF2IntermediateState &intermediate,
  EigenSupport::ConstRefVecXd stage2Solution,
  const TRBDF2Coefficients &coeffs) const;
```

Stage 2:

```cpp
A2 = coeffs.beta[4] * mass + coeffs.beta[7] * damping;
linear2 =
  mass * (coeffs.beta[0] * displacement + coeffs.beta[1] * uy
    + coeffs.beta[2] * velocity + coeffs.beta[3] * vy)
  + damping * (coeffs.beta[5] * displacement + coeffs.beta[6] * uy)
  - externalForce
  - A2 * displacement;
```

Final update:

```cpp
next.displacement = x2;
next.velocity = coeffs.beta[5] * displacement + coeffs.beta[6] * uy
  + coeffs.beta[7] * (x2 - displacement);
next.acceleration = coeffs.beta[0] * displacement + coeffs.beta[1] * uy
  + coeffs.beta[2] * velocity + coeffs.beta[3] * vy
  + coeffs.beta[4] * (x2 - displacement);
next.timestepId = state.timestepId + 1;
next.time = state.time + timestep;  // B1：时间推进，供下步 StepAwareEnergy::beginStep 使用
```

- [ ] **Step T6.4: Handle `gamma == 1` single-stage case**

When `gamma >= 1 - 1e-9`, TRBDF2 returns the stage-1 intermediate state as final state, matching legacy behavior.

- [ ] **Step T6.5: Add TRBDF2 formula and parity tests**

Tests:

- coefficients match legacy formulas;
- stage 1 `A/linear` match formulas from T0.2;
- stage 2 `A/linear` match formulas from T0.2;
- two-stage solve on a 1D quadratic matches legacy `TRBDF2TimeIntegrator`;
- `gamma == 1` returns one stage result and matches legacy state.

Run:

```bash
cmake --build --preset base_no_mkl_release --target dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest --gtest_filter=TRBDF2*
```

Expected: all `TRBDF2*` tests pass.

### Task T7: Add `DynamicStepper` and solver bridge

**Files:**

- Create: `src/core/simulation/dynamicStepper.h`
- Create: `src/core/simulation/dynamicStepper.cpp`
- Create: `src/core/simulation/dynamicStepService.h`
- Create: `src/core/simulation/dynamicStepService.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T7.1: 复用 `acceptsDynamicSolveStatus`（不重新定义）**

`acceptsDynamicSolveStatus` 已存在（`solverResult.h:41` / `solverResult.cpp:101`），语义即：

```cpp
return status == SolveStatus::Converged ||
  status == SolveStatus::MaxIterations ||
  status == SolveStatus::StepTooSmall;
```

直接 include 复用；`DynamicStepResult.accepted` 用它。IBE legacy 路径已经在用同一个 helper（`implicitBackwardEulerTimeIntegrator.cpp:57`）。

- [ ] **Step T7.2: Implement stage solve bridge**

Target API:

```cpp
NonlinearOptimization::OptimizationResult solveStageProblem(
  const ImplicitStageProblem &stage,
  const std::vector<int> &fixedDofs,
  EigenSupport::ConstRefVecXd fixedValues,
  const NonlinearOptimization::SolverControl &solver);  // 复用 solver plan SolverControl（C1）
```

Behavior:

- 目标实现：构造 `OptimizationProblem{ stage.energy, FixedVariables{ fixedDofs, fixedValues } }`，调 `minimize(problem, stage.initialGuess, NewtonOptions)`（`solver_api_refactor.plan.md`）。fixed DOF 走 first-class `FixedVariables`，**不**把 `xlow==xhi` 编码泄漏到本 plan 的公共 API。
- M3 落地前的过渡：用 `EnergyOptimizer::minimize(x, stage.energy, xlow, xhi, ST_NEWTON, ...)` Newton 路径（它内部已用 `xlow[i]==xhi[i]` canonical 化 fixed DOF），由本 bridge 把 `fixedDofs`/`fixedValues` 翻成 `xlow/xhi`，仅限内部。solver 服务落地后替换为上面的 `minimize(problem, ...)`。
- Always return owned solution vector。

- [ ] **Step T7.3: Implement IBE backend**

Target API:

```cpp
class ImplicitEulerStepper final : public DynamicStepper
{
public:
  explicit ImplicitEulerStepper(DynamicProblem problem);   // 持有 immutable 问题 + 持久态（见 D4）
  DynamicStepResult step(const DynamicState &state, const DynamicStepRequest &request) override;
};
```

Stepper 内部持有 `StageResidualHandle stageHandle_`（构造时 `initStageResidual` 建立）。Flow:

```text
validate state + request

// B1：对所有 StepAwareEnergy 调 beginStep（仿照 EnergySet 对 LineSearchAwareEnergy 的 dynamic_cast 模式）
for term in problem.persistentTerms:
    if auto *aware = dynamic_cast<StepAwareEnergy*>(term.energy.get()):
        aware->beginStep({time=state.time, timestep=problem.timestep, previousX=&state.displacement})
    if auto *contact = dynamic_cast<StatefulContactEnergy*>(term.energy.get()):
        contact->refreshActiveSet(state.displacement)   // 以当前位移作为初始 active set

assemble D（assembleRayleighDamping）

// B3：纯系数计算，不重建 EnergySet
coeffs = ImplicitEulerStageBuilder.compute(state, problem, request, D)
stage = prepareStageResidual(stageHandle_, coeffs.A, coeffs.linear, coeffs.initialGuess)
   // -> setLinearTerm(l_s) 原地更新 QuadraticEnergy，hessianAll 模板不变

solveStageProblem(stage, problem.fixedDofs, fixedValues, problem.solver)
if accepted -> updateImplicitEulerState  // 含 time += timestep
else -> keep previous state, accepted=false, 不推进 timestep
return DynamicStepResult with one stage result
```

The "not accepted" behavior preserves current IBE `tryTimestep` behavior: failed non-accepted status does not advance state。

- [ ] **Step T7.4: Implement TRBDF2 backend**

Target API:

```cpp
class TRBDF2Stepper final : public DynamicStepper
{
public:
  explicit TRBDF2Stepper(DynamicProblem problem, double gamma = 0.5);
  DynamicStepResult step(const DynamicState &state, const DynamicStepRequest &request) override;
};
```

Stepper 内部持有两个 `StageResidualHandle`：`stage1Handle_` 和 `stage2Handle_`（构造时各调一次 `initStageResidual` 建立）。Flow:

```text
validate state + request

// B1：对所有 StepAwareEnergy 调 beginStep（同 T7.3）
for term in problem.persistentTerms:
    if auto *aware = dynamic_cast<StepAwareEnergy*>(term.energy.get()):
        aware->beginStep({time=state.time, timestep=problem.timestep, previousX=&state.displacement})
    if auto *contact = dynamic_cast<StatefulContactEnergy*>(term.energy.get()):
        contact->refreshActiveSet(state.displacement)

assemble D（assembleRayleighDamping）

// Stage 1：纯系数计算，不重建 EnergySet
coeffs1 = TRBDF2StageBuilder.computeStage1(state, problem, request, D, trbdf2Coeffs)
stage1 = prepareStageResidual(stage1Handle_, coeffs1.A, coeffs1.linear, coeffs1.initialGuess)
solveStageProblem(stage1, ...)
if stage 1 not accepted -> return previous state, accepted=false
if gamma == 1 -> return stage 1 intermediate state

// Stage 2：refreshActiveSet 用 stage1 解更新 contact active set
for term in problem.persistentTerms:
    if auto *contact = dynamic_cast<StatefulContactEnergy*>(term.energy.get()):
        contact->refreshActiveSet(stage1Solution)

intermediate = TRBDF2StageBuilder.updateAfterStage1(state, stage1Solution, trbdf2Coeffs)
coeffs2 = TRBDF2StageBuilder.computeStage2(state, intermediate, problem, request, D, trbdf2Coeffs)
stage2 = prepareStageResidual(stage2Handle_, coeffs2.A, coeffs2.linear, coeffs2.initialGuess)
solveStageProblem(stage2, ...)
if stage 2 not accepted -> return previous state, accepted=false
return final state（含 time += timestep）and two stage results
```

This adds explicit TRBDF2 failure behavior. Legacy code did not throw in the same typed way; tests must document the new service behavior while leaving legacy API unchanged until migration.

- [ ] **Step T7.5: Add factory**

Target API:

```cpp
std::unique_ptr<DynamicStepper> makeDynamicStepper(
  TimeIntegratorKind kind,
  DynamicProblem problem,
  double trbdf2Gamma = 0.5);
```

Validation:

- reject unknown enum values；
- reject invalid gamma for TRBDF2；
- `validateDynamicProblem(problem, ...)`。

- [ ] **Step T7.6: Add dynamic step service tests**

Tests:

- IBE returns one stage result;
- TRBDF2 returns two stage results when `gamma < 1`;
- TRBDF2 returns one stage result when `gamma == 1`;
- fixed DOFs remain fixed;
- non-accepted failure keeps previous state;
- accepted `MaxIterations` advances state like legacy dynamic policy.

Run:

```bash
cmake --build --preset base_no_mkl_release --target dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest --gtest_filter=DynamicStepper*
```

Expected: all `DynamicStepper*` tests pass.

### Task T8: Migrate legacy `TimeIntegrator` classes to service internals

**Files:**

- Modify: `src/core/simulation/implicitBackwardEulerTimeIntegrator.h/.cpp`
- Modify: `src/core/simulation/implicitBackwardEulerTimeIntegratorHelper.h/.cpp`
- Modify: `src/core/simulation/TRBDF2TimeIntegrator.h/.cpp`
- Modify: `src/core/simulation/TRBDF2TimeIntegratorHelper.h/.cpp`
- Test: `tests/src/core/implicitBackwardEulerTimeIntegrator_gtest.cpp`

- [ ] **Step T8.1: Keep legacy public API source-compatible**

Existing call sites must still compile:

```cpp
ImplicitBackwardEulerTimeIntegrator integrator(
  mass, energy, 0.0, 0.0, 0.01, 20, 1e-8);
integrator.addGeneralImplicitForceModel(model, 0, 0);
SolverResult result = integrator.tryTimestep(1, 0, 0);
```

```cpp
TRBDF2TimeIntegrator integrator(
  mass, energy, 0.0, 0.0, 0.5, 0.01, 20, 1e-8);
integrator.doTimestep(1, 0, 0);
```

- [ ] **Step T8.2: Route IBE legacy implementation through `ImplicitEulerStepper`**

`tryTimestep()` should:

- 构造一次 `DynamicProblem`（mass、persistent terms + per-term damping、fixed DOFs、solver options）持有为成员，持久复用 `ImplicitEulerStepper`；
- 每步组装 `DynamicState`（`q/qvel/qacc`）+ `DynamicStepRequest`（external force、fixed values、transient terms）；
- call `ImplicitEulerStepper::step`；
- copy result state back into `q1/qvel1/qacc1`；
- preserve logging strings used by existing tests。

- [ ] **Step T8.3: Route TRBDF2 legacy implementation through `TRBDF2Stepper`**

`doTimestep()` should:

- assemble `DynamicState` + `DynamicStepRequest`（`DynamicProblem` 构造一次持有为成员）；
- call `TRBDF2Stepper::step`；
- expose `lastSolverResult` as the final accepted stage result；
- preserve `getLastSolutionTR()` / `getLastSolutionBDF2()` by storing stage solution vectors in legacy buffers。

- [ ] **Step T8.4: 删除 helper residual energy 类**

legacy 类改走 service 后，`ImplicitBackwardEulerEnergy` / `TRBDF2TimeIntegratorEnergy` 不再被引用，由 `EnergySet{ QuadraticEnergy, Φ }` 取代——**直接删除**两个 helper 文件（不留 wrap alias，理由同 §2/§4）。删除后：

```text
rg -l "ImplicitBackwardEulerEnergy|TRBDF2TimeIntegratorEnergy" src/core/simulation
```

Expected: 无匹配（两个 helper 文件已删除，且无残留 include）；不存在任何持有 `ImplicitBackwardEulerTimeIntegrator *` / `TRBDF2TimeIntegrator *` 的 residual energy 类。

- [ ] **Step T8.5: Run legacy tests**

Run:

```bash
cmake --build --preset base_no_mkl_release --target implicitBackwardEulerTimeIntegrator_gtest dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/implicitBackwardEulerTimeIntegrator_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest
```

Expected: all tests pass.

### Task T9: Migrate `runIPCSim` dynamic loop to step facade

**Files:**

- Modify: `src/tools/sim/runIPCSim/app/session.h/.cpp`
- Modify: `src/tools/sim/runIPCSim/app/loop.cpp`
- Modify: `src/tools/sim/runIPCSim/contact/contactBackend.h`
- Modify: `src/tools/sim/runIPCSim/contact/ipcContactBackend.cpp`
- Modify: `src/tools/sim/runIPCSim/contact/legacyPenaltyContact.cpp`
- Test: existing run-sim gtests and representative smoke commands

- [ ] **Step T9.1: Add `RunIPCSimDynamicStepper` facade**

Target shape:

```cpp
struct RunIPCSimStepRequest
{
  int frame = 0;
  DynamicState state;
  EigenSupport::VXd externalForce;
};

struct RunIPCSimStepResult
{
  DynamicStepResult dynamic;
  EigenSupport::VXd surfaceDisplacement;
};
```

The facade owns:

- selected `DynamicStepper`;
- immutable problem data: mass, elastic energy, pulling energies, contact energies;
- current mutable contact/session state needed by existing backends.

- [ ] **Step T9.2: Preserve current loop order**

The migrated `loop.cpp` must preserve:

```text
clear previous transient contact models
update attachment target
contact beginFrame
contact addForces
surface pressure external-force ramp
dynamic step
copy state
contact afterStep
contact summary
write output
```

No behavior change is allowed in this task.

- [ ] **Step T9.3: Remove direct `ImplicitBackwardEulerTimeIntegrator` dependency from contact backend interface**

Replace `RunIPCSimSession::integrator` usage in contact backends with a narrow force-model sink:

```cpp
class DynamicForceModelSink
{
public:
  virtual ~DynamicForceModelSink() = default;
  virtual void addGeneralImplicitForceModel(
    NonlinearOptimization::PotentialEnergy_p energy,
    double stiffnessDamping,
    double massDamping) = 0;
};
```

`RunIPCSimContactBackend::addForces(frame, runtimeConfig, context, sink)` receives the sink instead of mutating the legacy integrator directly.

- [ ] **Step T9.4: Add run-sim parity checks**

Run representative short cases already listed in `parity_matrix.md` after M4/M5 fixtures are available. Before Python config fixtures land, run the existing C++ executable smoke cases used by the repo.

Expected:

- output paths unchanged;
- dynamic accepted statuses unchanged;
- static path unaffected;
- IPC summary logging still present.

### Task T10: Python core bindings

**Files:**

- Create: `src/python/pypgo/bindings/simulation_bindings.cpp`
- Modify: `src/python/pypgo/bindings/module.cpp`
- Modify: `src/python/pypgo/CMakeLists.txt`
- Modify: `pypgo/sim.py`
- Test: `tests/pypgo/test_dynamic_stepper.py`

- [ ] **Step T10.1: Bind private core value objects**

Expose under `pypgo._core`（命名规范：`PyXxx`）：

```text
PyDynamicState
PyDynamicStepResult
PyDynamicSimulation
```

Core binding rules:

- accept/return NumPy `float64` vectors through existing Eigen conversion helpers;
- accept mass as `pypgo.sparse.SparseMatrix` / `_core` sparse wrapper after M2 sparse wrapper is available;
- `PyDynamicSimulation` 直接包 stepper（D4），C++ ownership 在其内部；
- release GIL during `step()` and `run()`.

- [ ] **Step T10.2: Register bindings**

In `module.cpp`:

```cpp
void init_simulation_bindings(nb::module_ &m);

NB_MODULE(_core, m) {
  m.def("build_info", []() {
    nb::dict info;
    info["module"] = "pypgo._core";
    info["binding"] = "nanobind";
    info["mesh_geo"] = true;
    return info;
  });

  init_mesh_geo_bindings(m);
  init_mesh_bindings(m);
  init_sparse_bindings(m);
  init_dense_bindings(m);
  init_energy_bindings(m);
  init_simulation_bindings(m);
}
```

In `CMakeLists.txt`, append:

```cmake
bindings/simulation_bindings.cpp
```

and link:

```cmake
simulation
nonlinearOptimization
constraintPotentialEnergies
```

- [ ] **Step T10.3: Add Python `DynamicState` and `DynamicFrame`**

Target public API in `pypgo/sim.py`:

```python
@dataclass(frozen=True)
class DynamicState:
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    timestep_id: int = 0


@dataclass(frozen=True)
class DynamicFrame:
    frame_index: int
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    solver_result: pgo.solver.SolverResult
    stage_results: Sequence[pgo.solver.SolverResult]
    accepted: bool
```

Arrays returned to Python must be owned copies, not views into C++ mutable state.

- [ ] **Step T10.4: Add Python `DynamicSimulation`**

Target public API:

```text
DynamicSimulation.__init__(
  *,
  mass,
  energy,
  state: DynamicState,
  timestep: float,
  integrator: Literal["implicit_euler", "trbdf2"] = "implicit_euler",
  damping: tuple[float, float] = (0.0, 0.0),
  solver: pgo.solver.NewtonOptions | None = None,
  trbdf2_gamma: float = 0.5,
) -> None

DynamicSimulation.state -> DynamicState

DynamicSimulation.step(
  *,
  external_force: np.ndarray | Sequence[float] | None = None,
  fixed_dofs: Sequence[int] | None = None,
  fixed_values: np.ndarray | Sequence[float] | None = None,
) -> DynamicFrame

DynamicSimulation.run(num_steps: int, **step_kwargs) -> list[DynamicFrame]
```

Validation:

- `num_steps >= 0`;
- `integrator` string is one of `"implicit_euler"`, `"trbdf2"`;
- `external_force` defaults to zero vector;
- fixed values default to current state values at fixed DOFs;
- dtype/shape normalized through `_arrays.py`.

- [ ] **Step T10.5: Add Python tests**

Tests:

- import `pypgo.sim.DynamicSimulation`;
- one-step IBE quadratic solve returns expected displacement shape and owned arrays;
- one-step TRBDF2 returns `len(frame.stage_results) == 2`;
- unknown integrator raises `ValueError`;
- fixed DOFs remain fixed;
- `run(0)` returns empty list and does not mutate state;
- returned frame arrays do not alias input arrays.

Run:

```bash
python -m pytest -q tests/pypgo/test_dynamic_stepper.py
```

Expected: all tests pass.

### Task T11: Python run-sim integration layer

**Files:**

- Modify: `pypgo/sim.py`
- Test: `tests/pypgo/test_dynamic_sim.py` after M4 config/context builder lands

- [ ] **Step T11.1: Add `DynamicSimulation.from_context`**

Target API:

```python
@classmethod
def from_context(
    cls,
    context: SimulationContext,
    *,
    integrator: Literal["implicit_euler", "trbdf2"] = "implicit_euler",
    solver: pgo.solver.NewtonOptions | None = None,
    trbdf2_gamma: float = 0.5,
) -> DynamicSimulation
```

Behavior:

- uses context mass, elastic/contact/floor/attachment energies;
- initializes state from context rest displacement, initial velocity, acceleration;
- config-driven output remains in `pypgo.sim` runner, not inside the low-level dynamic simulation object.

- [ ] **Step T11.2: Add output-facing frame adapter**

Target shape:

```python
def write_frame(self, frame: DynamicFrame, output: OutputLayout) -> None
```

This method is part of M5/M6 run-sim migration and uses output services, not the low-level C++ stepper directly.

### Task T12: Documentation and migration matrix updates

**Files:**

- Modify: `plan/python_api_migration/milestones.md`
- Modify: `plan/python_api_migration/api_coverage.md`
- Modify: `plan/python_api_migration/parity_matrix.md`
- Optional Modify: `plan/python_api_migration/future_work.md`

- [ ] **Step T12.1: Update M6 deliverables**

Add explicit M6 bullets:

- `DynamicSimulation` supports `"implicit_euler"` and `"trbdf2"`;
- `DynamicFrame.stage_results` exposes TRBDF2 stage diagnostics;
- low-level dynamic step API is separate from config/output runner;
- first Python dynamic API supports unconstrained Newton + fixed DOFs.

- [ ] **Step T12.2: Update API coverage**

Change dynamic loop row to mention:

```text
C++ boundary: DynamicStepper service + PyDynamicSimulation binding
Python API: pypgo.sim.DynamicSimulation
```

- [ ] **Step T12.3: Update parity matrix**

Add TRBDF2-specific smoke tests:

- direct low-level Python TRBDF2 quadratic one-step;
- C++ TRBDF2 parity against legacy class;
- later config-level TRBDF2 case once config schema accepts integrator choice.

## Validation Commands

Run after C++ service tasks:

```bash
cmake --build --preset base_no_mkl_release --target simulation dynamicStepper_gtest implicitBackwardEulerTimeIntegrator_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/implicitBackwardEulerTimeIntegrator_gtest
```

Run after Python binding tasks:

```bash
python -m pytest -q tests/pypgo/test_dynamic_stepper.py
python -m pytest -q tests/pypgo/test_package_scaffold.py
```

Run after run-sim migration tasks:

```bash
python -m pytest -q tests/pypgo/test_dynamic_sim.py -m "not slow"
```

Expected:

- C++ service tests pass;
- legacy integrator tests pass;
- Python dynamic stepper tests pass;
- static solve tests remain unaffected;
- run-sim dynamic parity tests pass once M4/M5 fixtures are available.

## Done Criteria

- 不存在持有 `ImplicitBackwardEulerTimeIntegrator *` / `TRBDF2TimeIntegrator *` 的 residual energy 类；两个 helper 文件已删除，residual 由 `EnergySet{ QuadraticEnergy, Φ }` 表达。
- IBE and TRBDF2 通过 explicit stage builder（纯系数计算）+ `initStageResidual`（构造一次持久 EnergySet）+ `prepareStageResidual`（per-step 原地更新 QuadraticEnergy）构造 stage residual，不新增聚合/二次型类，`hessianAll` 模板构造一次不 rebuild。
- `EnergySet::func_grad_hessian` 为单遍 fused 实现，非固定拓扑（IPC）的 active-set 构建次数相对手写 helper 不退化。
- `DynamicStepper` supports IBE and TRBDF2 through one public service interface（immutable problem 构造 + per-step request）。
- TRBDF2 stage solver results are visible in C++ and Python.
- Python can run one-step IBE and TRBDF2 dynamic solves from in-memory mass/energy/state data.
- Legacy C++ `ImplicitBackwardEulerTimeIntegrator` and `TRBDF2TimeIntegrator` call sites still compile.
- Existing dynamic acceptance semantics are preserved.
- `runIPCSim` output behavior is unchanged after migration.

## Risks and Mitigations

- **Risk: sign error in residual linear term.**  
  Mitigation: T0/T5/T6 formula tests assert exact `A` and `linear` before solver behavior is considered.

- **Risk: IPC/contact active-set rebuild count regresses.**  
  Mitigation: T2.2 把 `EnergySet::func_grad_hessian` 改成单遍调用每个 term 的 `func_grad_hessian`（保留手写 helper 的 "1 buildActiveSet" 语义）；T2.3 用计数 stub 断言每步只触发一次，profiling parity 比较改动前后 active-set 构建计数。

- **Risk: 每步重建 `EnergySet` 的 hessian 模板带来逐帧开销（现有代码仅在 `generalForceModelChanged` 时重建）。**  
  Mitigation: D1/D2/D3 均已拍板（见「设计决策（已拍板）」）：持久 `EnergySet`（构造一次）+ `QuadraticEnergy::setLinearTerm` 原地更新，`hessianAll` 模板只建一次，无重建开销。性能 parity 对照点是 dynamic loop 的 per-step 新分配计数（应为零）。

- **Risk: TRBDF2 failure semantics become inconsistent with legacy class.**  
  Mitigation: service behavior is explicit in T7 tests; legacy API remains source-compatible until run-sim migration validates acceptance policy.

- **Risk: Python API exposes too much low-level lifecycle.**  
  Mitigation: Python public API only exposes `DynamicSimulation.step/run`; contact force model injection stays in C++ context/facade.

- **Risk: implementing before solver service exists duplicates solver logic.**  
  Mitigation: T7.2 过渡期复用现成的 `EnergyOptimizer::minimize` Newton 路径（不自己再写一份 Newton + fixed-DOF 装配），最终 M6 API 依赖 `solver_api_refactor.plan.md` 的 `minimize(problem, x0, NewtonOptions)` + `FixedVariables`。
