# Time Integrator API Refactor Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **状态日期：** 2026-06-01
> **适用范围：** C++ `simulation` time-integrator architecture + Python `pypgo.sim` dynamic step API.
> **执行约束：** 不改变 implicit Euler / TRBDF2 数值公式、Newton line-search / damping / max-step 语义、dynamic timestep acceptance policy、IPC active-set timing 或现有 `runIPCSim` 输出语义。本计划先把 residual energy 和 step lifecycle 服务化，再绑定 Python API。

**Goal:** 把 IBE 和 TRBDF2 迁移到统一的 dynamic step service，其中 residual energy 是自包含 stage problem，Python 通过 `DynamicSimulation.step()` / `run()` 驱动动态仿真。

**Architecture:** C++ 分为 `DynamicState`、`ImplicitModelAssembly`、`ImplicitResidualEnergy`、IBE/TRBDF2 stage builder、`DynamicStepper` backend、run-sim facade、nanobind binding。IBE 是 single-stage backend，TRBDF2 是 two-stage backend；两者都构造同一种 `ImplicitResidualEnergy` 并返回统一 `DynamicStepResult`。

**Tech Stack:** C++17, Eigen sparse/dense, existing `PotentialEnergy`, existing or M3 `OptimizationService`, nanobind, pytest, GoogleTest.

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
};

struct DynamicStepResult
{
  DynamicState state;
  NonlinearOptimization::SolverResult solver;
  std::vector<NonlinearOptimization::SolverResult> stageResults;
  bool accepted = false;
};

class DynamicStepper
{
public:
  virtual ~DynamicStepper() = default;
  virtual DynamicStepResult step(
    const DynamicState &state,
    const DynamicStepInputs &inputs) = 0;
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

### 1. `ImplicitResidualEnergy` 用 `linear`，不使用 legacy `b`

新 residual energy 永远表达：

```math
J(x)=\frac12x^TAx+\Phi(x)+linear^Tx
```

IBE 的 legacy `b` 通过 `linear = -b` 转换；TRBDF2 的 legacy `b1/b2` 直接作为 `linear`。这样不会把 IBE 的 `-b^Tx` 和 TRBDF2 的 `+b^Tx` 符号差异泄漏到公共类名里。

### 2. `ImplicitModelAssembly` 取代 residual energy 对 integrator state 的读取

`ImplicitResidualEnergy` 不持有 `TimeIntegrator *`。它持有或共享一个 explicit assembly object：

```cpp
struct ImplicitModelTerm
{
  NonlinearOptimization::PotentialEnergy_const_p energy;
  double stiffnessDamping = 0.0;
  double massDamping = 0.0;
};

class ImplicitModelAssembly
{
public:
  ImplicitModelAssembly(int numDofs, std::vector<ImplicitModelTerm> terms);

  int numDofs() const;
  const std::vector<ImplicitModelTerm> &terms() const;
  const EigenSupport::SpMatD &hessianPattern() const;

  void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const;
  void hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const;
  double funcGradHessian(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::RefVecXd grad,
    EigenSupport::SpMatD &hess) const;

  NonlinearOptimization::MaxStepResult computeMaxStepLimit(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::ConstRefVecXd dx) const;
};
```

Fixed-topology energies reuse preallocated `K` buffers and mappings. Non-fixed-topology energies use their safe one-shot / fused calls, preserving IPC active-set behavior.

### 3. Damping 是独立 assembly

`D_n` construction is a separate operation:

```cpp
EigenSupport::SpMatD assembleRayleighDamping(
  const ImplicitModelAssembly &assembly,
  const EigenSupport::SpMatD &mass,
  EigenSupport::ConstRefVecXd state);
```

Rules:

- mass damping contributes `massDamping * M`;
- stiffness damping contributes `stiffnessDamping * Hessian_i(u_n)`;
- non-fixed-topology energies are skipped for damping Hessian, preserving existing behavior.

### 4. IBE/TRBDF2 是 backend，不是 residual energy subclasses

There is one residual energy class and two stage builders:

```text
ImplicitResidualEnergy
  used by ImplicitEulerStageBuilder
  used by TRBDF2StageBuilder
```

`TRBDF2` returns two stage solver results. Python exposes both through `frame.stage_results`; `frame.solver_result` is the final stage result unless a previous stage failed in a non-accepted way.

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

First C++ refactor preserves current `runIPCSim` contact timing. A later contact plan migrates `RunIPCSimContactBackend` to `StepAwareEnergy` / `StatefulContactEnergy`.

## File Map

### 新增

- `src/core/simulation/dynamicState.h`
- `src/core/simulation/dynamicStepOptions.h`
- `src/core/simulation/implicitModelAssembly.h`
- `src/core/simulation/implicitModelAssembly.cpp`
- `src/core/simulation/rayleighDampingAssembly.h`
- `src/core/simulation/rayleighDampingAssembly.cpp`
- `src/core/simulation/implicitResidualEnergy.h`
- `src/core/simulation/implicitResidualEnergy.cpp`
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
- `src/core/simulation/implicitBackwardEulerTimeIntegratorHelper.h/.cpp`：迁移或包裹到 `ImplicitResidualEnergy`，最终不再持有 `ImplicitBackwardEulerTimeIntegrator *`。
- `src/core/simulation/TRBDF2TimeIntegratorHelper.h/.cpp`：迁移或包裹到 `ImplicitResidualEnergy`，最终不再持有 `TRBDF2TimeIntegrator *`。
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
DynamicStepInputs inputs = makeEquivalentInputs(mass, energy, dt, 0.0, 0.0, 20, 1e-10);
DynamicState state = makeState(u, v, a);

const SolverResult legacyResult = legacy.tryTimestep(1, 0, 0);
const DynamicStepResult serviceResult =
  makeDynamicStepper(TimeIntegratorKind::ImplicitEuler).step(state, inputs);

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
};

void validateDynamicState(const DynamicState &state, int numDofs);

}  // namespace pgo::Simulation
```

Validation:

- `numDofs > 0`;
- all three vectors have `numDofs`;
- all values are finite.

- [ ] **Step T1.2: Define `DynamicSolverOptions` and `DynamicStepInputs`**

Target API:

```cpp
namespace pgo::Simulation
{

struct DynamicSolverOptions
{
  int maxIterations = 50;
  double tolerance = 1e-6;
  int verbose = 0;
};

struct DynamicStepInputs
{
  EigenSupport::SpMatD mass;
  std::vector<ImplicitModelTerm> implicitTerms;
  EigenSupport::VXd externalForce;
  std::vector<int> fixedDofs;
  std::optional<EigenSupport::VXd> fixedValues;
  double timestep = 0.0;
  double massDamping = 0.0;
  double stiffnessDamping = 0.0;
  DynamicSolverOptions solver;
};

void validateDynamicStepInputs(const DynamicStepInputs &inputs, int numDofs);

}  // namespace pgo::Simulation
```

Validation:

- mass is square and has `numDofs` rows;
- `externalForce.size() == numDofs`;
- `timestep > 0`;
- damping parameters are finite and non-negative;
- `fixedDofs` are sorted/canonicalized before solver use;
- duplicate/out-of-range fixed DOFs throw `std::invalid_argument`.

- [ ] **Step T1.3: Wire headers into `simulation` target**

Modify `src/core/simulation/CMakeLists.txt`:

```cmake
set(SIMULATION_HEADERS
  timeIntegrator.h
  timeIntegratorSolver.h
  dynamicState.h
  dynamicStepOptions.h
  implicitModelAssembly.h
  rayleighDampingAssembly.h
  implicitResidualEnergy.h
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

### Task T2: Extract `ImplicitModelAssembly`

**Files:**

- Create: `src/core/simulation/implicitModelAssembly.h`
- Create: `src/core/simulation/implicitModelAssembly.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T2.1: Define `ImplicitModelTerm` and constructor**

Target API:

```cpp
struct ImplicitModelTerm
{
  NonlinearOptimization::PotentialEnergy_const_p energy;
  double stiffnessDamping = 0.0;
  double massDamping = 0.0;
};

class ImplicitModelAssembly
{
public:
  ImplicitModelAssembly(int numDofs, std::vector<ImplicitModelTerm> terms);

  int numDofs() const;
  const std::vector<ImplicitModelTerm> &terms() const;
  const EigenSupport::SpMatD &hessianPattern() const;
  const std::vector<int> &allDofs() const;
};
```

Constructor rules:

- reject `numDofs <= 0`;
- reject null energy;
- reject energy DOF mismatch;
- build `allDofs` as every integer from `0` through `numDofs - 1`;
- build a combined Hessian sparsity pattern from all fixed-topology energies.

- [ ] **Step T2.2: Implement evaluation functions**

Target API:

```cpp
double func(EigenSupport::ConstRefVecXd x) const;
void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const;
void hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const;
void hessianInPlaceForFixedTopology(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const;
void gradientHessian(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad, EigenSupport::SpMatD &hess) const;
double funcGradHessian(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad, EigenSupport::SpMatD &hess) const;
```

Behavior:

- fixed-topology terms use preallocated per-term Hessian buffers and `addSmallToBig`;
- non-fixed-topology terms use `gradient_hessian` or `func_grad_hessian`;
- `funcGradHessian` preserves the existing IPC fused-evaluation benefit.

- [ ] **Step T2.3: Implement max-step aggregation**

Target API:

```cpp
NonlinearOptimization::MaxStepResult computeMaxStepLimit(
  EigenSupport::ConstRefVecXd x,
  EigenSupport::ConstRefVecXd dx) const;
```

Behavior:

- call each term's `computeMaxStepLimit(x, dx)`;
- merge using `NonlinearOptimization::mergeMaxStepResults`;
- return unconstrained when no term clamps.

- [ ] **Step T2.4: Add assembly tests**

Tests:

- two quadratic fixed-topology terms sum energy/gradient/Hessian;
- one fixed-topology + one non-fixed-topology term calls the fused non-fixed path once;
- null energy throws;
- DOF mismatch throws;
- max-step aggregation returns min alpha.

Run:

```bash
cmake --build --preset base_no_mkl_release --target dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest --gtest_filter=ImplicitModelAssembly*
```

Expected: all `ImplicitModelAssembly*` tests pass.

### Task T3: Add Rayleigh damping assembly

**Files:**

- Create: `src/core/simulation/rayleighDampingAssembly.h`
- Create: `src/core/simulation/rayleighDampingAssembly.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T3.1: Implement damping assembly**

Target API:

```cpp
EigenSupport::SpMatD assembleRayleighDamping(
  const ImplicitModelAssembly &assembly,
  const EigenSupport::SpMatD &mass,
  EigenSupport::ConstRefVecXd state);
```

Formula:

```math
D_n = \sum_i m_i M_i + \sum_i k_i \nabla^2\Phi_i(u_n)
```

Current-code compatibility:

- all terms use the global `mass` for mass damping;
- fixed-topology terms contribute stiffness damping;
- non-fixed-topology terms do not contribute stiffness damping.

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

### Task T4: Add self-contained `ImplicitResidualEnergy`

**Files:**

- Create: `src/core/simulation/implicitResidualEnergy.h`
- Create: `src/core/simulation/implicitResidualEnergy.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T4.1: Define residual problem data**

Target API:

```cpp
struct ImplicitResidualProblemData
{
  EigenSupport::SpMatD quadratic;
  EigenSupport::VXd linear;
  std::shared_ptr<const ImplicitModelAssembly> assembly;
};
```

Validation:

- `quadratic` is square;
- `linear.size() == quadratic.rows()`;
- `assembly != nullptr`;
- `assembly->numDofs() == quadratic.rows()`.

- [ ] **Step T4.2: Implement `ImplicitResidualEnergy`**

Target API:

```cpp
class ImplicitResidualEnergy final : public NonlinearOptimization::PotentialEnergy
{
public:
  explicit ImplicitResidualEnergy(ImplicitResidualProblemData data);

  double func(EigenSupport::ConstRefVecXd x) const override;
  void gradient(EigenSupport::ConstRefVecXd x, EigenSupport::RefVecXd grad) const override;
  void hessianInPlace(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  void hessian(EigenSupport::ConstRefVecXd x, EigenSupport::SpMatD &hess) const override;
  void hessianAlloc(EigenSupport::SpMatD &hess) const override;
  void getDOFs(std::vector<int> &dofs) const override;
  int getNumDOFs() const override;
  int isHessianTopologyFixed() const override;
  double func_grad_hessian(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::RefVecXd grad,
    EigenSupport::SpMatD &hess) const override;
  void gradient_hessian(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::RefVecXd grad,
    EigenSupport::SpMatD &hess) const override;
  NonlinearOptimization::MaxStepResult computeMaxStepLimit(
    EigenSupport::ConstRefVecXd x,
    EigenSupport::ConstRefVecXd dx) const override;
};
```

Formula:

```cpp
J(x) = 0.5 * x.dot(A * x) + assembly.func(x) + linear.dot(x);
grad = A * x + assembly.gradient(x) + linear;
hess = A + assembly.hessian(x);
```

- [ ] **Step T4.3: Add residual energy tests**

Tests:

- value/gradient/Hessian match finite differences on a 2D quadratic fixture;
- `func_grad_hessian` calls assembly fused path;
- `isHessianTopologyFixed()` returns false when any term is non-fixed;
- `computeMaxStepLimit()` delegates to assembly.

Run:

```bash
cmake --build --preset base_no_mkl_release --target dynamicStepper_gtest
./build/base_no_mkl/tests/src/core/dynamicStepper_gtest --gtest_filter=ImplicitResidualEnergy*
```

Expected: all `ImplicitResidualEnergy*` tests pass.

### Task T5: Add IBE stage builder and updater

**Files:**

- Create: `src/core/simulation/implicitEulerStageBuilder.h`
- Create: `src/core/simulation/implicitEulerStageBuilder.cpp`
- Test: `tests/src/core/dynamicStepper_gtest.cpp`

- [ ] **Step T5.1: Define stage problem value object**

Target API:

```cpp
struct ImplicitStageProblem
{
  std::shared_ptr<ImplicitResidualEnergy> energy;
  EigenSupport::SpMatD A;
  EigenSupport::VXd linear;
  EigenSupport::VXd initialGuess;
};
```

- [ ] **Step T5.2: Implement IBE builder**

Target API:

```cpp
class ImplicitEulerStageBuilder
{
public:
  ImplicitStageProblem build(
    const DynamicState &state,
    const DynamicStepInputs &inputs,
    const std::shared_ptr<const ImplicitModelAssembly> &assembly,
    const EigenSupport::SpMatD &damping) const;
};
```

Rules:

- `initialGuess = state.displacement` by default;
- when velocity extrapolation support is added, `initialGuess = state.displacement + state.velocity * timestep`;
- `A = M / h^2 + D / h`;
- `linear = -(f_ext + M v / h + A u)`.

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

```cpp
class TRBDF2StageBuilder
{
public:
  ImplicitStageProblem buildStage1(
    const DynamicState &state,
    const DynamicStepInputs &inputs,
    const std::shared_ptr<const ImplicitModelAssembly> &assembly,
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
ImplicitStageProblem buildStage2(
  const DynamicState &state,
  const TRBDF2IntermediateState &intermediate,
  const DynamicStepInputs &inputs,
  const std::shared_ptr<const ImplicitModelAssembly> &assembly,
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

- [ ] **Step T7.1: Define `DynamicStepResult` and accepted-status helper**

Target API:

```cpp
bool acceptsDynamicSolveStatus(NonlinearOptimization::SolveStatus status);
```

Behavior must match current dynamic acceptance:

```cpp
return status == SolveStatus::Converged ||
  status == SolveStatus::MaxIterations ||
  status == SolveStatus::StepTooSmall;
```

`DynamicStepResult.accepted` uses this helper.

- [ ] **Step T7.2: Implement stage solve bridge**

Target API:

```cpp
NonlinearOptimization::OptimizationResult solveStageProblem(
  const ImplicitStageProblem &stage,
  const DynamicStepInputs &inputs);
```

Behavior:

- If `optimizationService` from `solver_api_refactor.plan.md` exists, call `minimize(problem, stage.initialGuess, NewtonOptions)`.
- If implementation order reaches this task before solver service lands, add a narrow internal adapter that constructs `NewtonSolver` with canonicalized fixed DOFs and explicit fixed values, then replace it with `OptimizationService` in the solver plan integration step.
- Always return owned solution vector.

- [ ] **Step T7.3: Implement IBE backend**

Target API:

```cpp
class ImplicitEulerStepper final : public DynamicStepper
{
public:
  DynamicStepResult step(const DynamicState &state, const DynamicStepInputs &inputs) override;
};
```

Flow:

```text
validate state + inputs
build ImplicitModelAssembly
assemble D
build IBE stage
solve stage
if accepted -> update state
else -> keep previous state and increment no timestep
return DynamicStepResult with one stage result
```

The "not accepted" behavior preserves current IBE `tryTimestep` behavior: failed non-accepted status does not advance state.

- [ ] **Step T7.4: Implement TRBDF2 backend**

Target API:

```cpp
class TRBDF2Stepper final : public DynamicStepper
{
public:
  explicit TRBDF2Stepper(double gamma = 0.5);
  DynamicStepResult step(const DynamicState &state, const DynamicStepInputs &inputs) override;
};
```

Flow:

```text
validate state + inputs
build ImplicitModelAssembly
assemble D
build/solve stage 1
if stage 1 not accepted -> return previous state, accepted=false
if gamma == 1 -> return stage 1 state
build/solve stage 2
if stage 2 not accepted -> return previous state, accepted=false
return final state and two stage results
```

This adds explicit TRBDF2 failure behavior. Legacy code did not throw in the same typed way; tests must document the new service behavior while leaving legacy API unchanged until migration.

- [ ] **Step T7.5: Add factory**

Target API:

```cpp
std::unique_ptr<DynamicStepper> makeDynamicStepper(
  TimeIntegratorKind kind,
  double trbdf2Gamma = 0.5);
```

Validation:

- reject unknown enum values;
- reject invalid gamma for TRBDF2.

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

- assemble `DynamicState` from `q/qvel/qacc`;
- assemble `DynamicStepInputs` from mass, energies, external force, fixed DOFs, damping and solver options;
- call `ImplicitEulerStepper::step`;
- copy result state back into `q1/qvel1/qacc1`;
- preserve logging strings used by existing tests.

- [ ] **Step T8.3: Route TRBDF2 legacy implementation through `TRBDF2Stepper`**

`doTimestep()` should:

- assemble `DynamicState` and `DynamicStepInputs`;
- call `TRBDF2Stepper::step`;
- expose `lastSolverResult` as the final accepted stage result;
- preserve `getLastSolutionTR()` / `getLastSolutionBDF2()` by storing stage solution vectors in legacy buffers.

- [ ] **Step T8.4: Replace helper energy pointer ownership**

After legacy classes route through service, either delete helper residual energy classes or keep thin aliases that wrap `ImplicitResidualEnergy` without storing `intg*`. The final state must satisfy:

```text
rg "ImplicitBackwardEulerTimeIntegrator \\*" src/core/simulation/*Helper*
rg "TRBDF2TimeIntegrator \\*" src/core/simulation/*Helper*
```

Expected: no residual energy class stores an integrator pointer.

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

Expose under `pypgo._core`:

```text
DynamicStateCore
DynamicStepResultCore
DynamicSimulationCore
```

Core binding rules:

- accept/return NumPy `float64` vectors through existing Eigen conversion helpers;
- accept mass as `pypgo.sparse.SparseMatrix` / `_core` sparse wrapper after M2 sparse wrapper is available;
- keep C++ ownership inside `DynamicSimulationCore`;
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
C++ boundary: DynamicStepper service + DynamicSimulationCore binding
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

- `ImplicitResidualEnergy` does not store `ImplicitBackwardEulerTimeIntegrator *` or `TRBDF2TimeIntegrator *`.
- IBE and TRBDF2 construct stage problems through explicit builders.
- `DynamicStepper` supports IBE and TRBDF2 through one public service interface.
- TRBDF2 stage solver results are visible in C++ and Python.
- Python can run one-step IBE and TRBDF2 dynamic solves from in-memory mass/energy/state data.
- Legacy C++ `ImplicitBackwardEulerTimeIntegrator` and `TRBDF2TimeIntegrator` call sites still compile.
- Existing dynamic acceptance semantics are preserved.
- `runIPCSim` output behavior is unchanged after migration.

## Risks and Mitigations

- **Risk: sign error in residual linear term.**  
  Mitigation: T0/T5/T6 formula tests assert exact `A` and `linear` before solver behavior is considered.

- **Risk: IPC/contact active-set rebuild count regresses.**  
  Mitigation: `ImplicitModelAssembly::funcGradHessian` must preserve fused calls for non-fixed-topology energies; profiling parity should compare active-set build counters before/after.

- **Risk: TRBDF2 failure semantics become inconsistent with legacy class.**  
  Mitigation: service behavior is explicit in T7 tests; legacy API remains source-compatible until run-sim migration validates acceptance policy.

- **Risk: Python API exposes too much low-level lifecycle.**  
  Mitigation: Python public API only exposes `DynamicSimulation.step/run`; contact force model injection stays in C++ context/facade.

- **Risk: implementing before solver service exists duplicates solver logic.**  
  Mitigation: T7 allows a temporary narrow Newton adapter, but the final M6 API must depend on `OptimizationService` from `solver_api_refactor.plan.md`.
