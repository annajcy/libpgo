# `pypgo.sim` — 动力学时间积分模块架构

> 包目录：`pypgo/sim/`（4 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

把弹性体动力学的一个时间步表达为**带二次惯性项的非线性最小化**，复用整套静力学基础设施：能量是 [`PotentialEnergy`](../energy/base.md)，求解是 [`NewtonOptimizer`](../solver/optimizer.md)。运动方程

$$M\,\ddot{\mathbf u} + D\,\dot{\mathbf u} + \nabla E(\mathbf u) = \mathbf f_{\text{ext}}$$

（$M$ 质量阵、$D$ Rayleigh 阻尼、$E$ 总势能——弹性 + 接触 + 软固定）被隐式离散为每步一个（或两个）**阶段问题**：

$$\mathbf u^{n+1}=\arg\min_{\mathbf u}\ \underbrace{\tfrac12\,\mathbf u^\top A\,\mathbf u+\mathbf l^\top\mathbf u}_{\text{阶段二次型（惯性+阻尼+外力）}}\;+\;E(\mathbf u)$$

阶段二次型的 $(A,\mathbf l)$ 由积分器决定（[stepper.md](stepper.md)）；驻点方程正是离散运动方程。能量最小化形式天然兼容 IPC 的可行步长机制——这就是 incremental potential 框架。

## 理论流水线：一步动力学

```
DynamicState (u, v, a, t) ──> stepper 组装 A, l ──> EnergySet{½uᵀAu+lᵀu, E} ──> Newton ──> 状态更新 (u,v,a) ──> DynamicFrame
                              └ begin_step 派发给接触能量（x^t, t）
```

**阶段 0 — `begin_step` 派发**。每步开始，C++ stepper 把 `StepState{time, timestep, previousX=u^n}` 派发给每个实现 `StepAwareEnergy` 的持久能量项（`src/core/simulation/common/dynamicStepperUtils.cpp:30-41`）——摩擦接触拿到 $\mathbf x^t$、IPC 把运动障碍物推进到步末（[../contact/base.md](../contact/base.md)）。

**阶段 1 — Rayleigh 阻尼组装**（`common/rayleighDampingAssembly.cpp`）。每步以当前位移重算

$$D = \alpha_M\,M + \alpha_K\,K(\mathbf u^n),\qquad K=\nabla^2E\big|_{\mathbf u^n}$$

刚度阻尼只取 Hessian 拓扑固定的能量项（39-43 行）——接触能量活动集随状态变化，**不进入** $D$。

**阶段 2 — 阶段二次型**。以 Backward Euler 为例（`backwardEuler/backwardEulerStageBuilder.cpp:14-26`）：

$$A=\frac{M}{h^2}+\frac{D}{h},\qquad
\mathbf l=-\Big(\mathbf f_{\text{ext}}+\frac{1}{h}M\mathbf v^n+A\,\mathbf u^n\Big)$$

驻点 $\nabla=0$ 即

$$\frac{1}{h^2}M\big(\mathbf u-\tilde{\mathbf x}\big)+\frac{1}{h}D\big(\mathbf u-\mathbf u^n\big)+\nabla E(\mathbf u)=\mathbf f_{\text{ext}},
\qquad \tilde{\mathbf x}=\mathbf u^n+h\,\mathbf v^n$$

——经典 incremental potential：$\min_{\mathbf u}\ \frac{1}{2h^2}\|\mathbf u-\tilde{\mathbf x}\|_M^2+\frac{1}{2h}\|\mathbf u-\mathbf u^n\|_D^2+E(\mathbf u)-\mathbf f_{\text{ext}}^\top\mathbf u$，惯性预测位置 $\tilde{\mathbf x}$ 是 $\mathbf u^n+h\mathbf v^n$。TRBDF2 的两阶段系数见 [stepper.md](stepper.md)。

**阶段 3 — Newton 求解**。阶段二次型装成持久 `QuadraticPotentialEnergy` 与用户能量并入一个 `EnergySet`（`common/stageResidual.cpp:11-46`，逐步只原位换 $A,\mathbf l$ 值、Hessian 模板不重建），固定 DOF 经 `fixVariables` 钉死后交给 optimizer（`dynamicStepperUtils.cpp:54-73`）。

**阶段 4 — 状态更新与接受判定**。求解状态 ∈ {Converged, MaxIterations, StepTooSmall} 视为接受（`acceptsDynamicSolveStatus`，`nonlinearOptimization/solver/common/solverResult.cpp:101-106`），按积分器公式更新 $(\mathbf u,\mathbf v,\mathbf a,t)$；**拒绝则状态原样保留**（不前进），由调用方决定终止/诊断（[simulation.md](simulation.md)）。

## 模块 ↔ 数学 ↔ 阶段 主表

| 文件 | 数学对象 | 公式 | 文档 |
|---|---|---|---|
| `state.py` | 运动学状态 / 步结果 | $(\mathbf u,\mathbf v,\mathbf a,t)$ | [state.md](state.md) |
| `stepper.py` | 时间积分器 | BE：$A=M/h^2+D/h$；TRBDF2：TR(γh)+BDF2 | [stepper.md](stepper.md) |
| `simulation.py` | 仿真引擎 | $\min\ \tfrac12\mathbf u^\top A\mathbf u+\mathbf l^\top\mathbf u+E(\mathbf u)$ | [simulation.md](simulation.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## C++ 引擎对应

| Python | C++ | 位置 |
|---|---|---|
| `DynamicState` | `Simulation::DynamicState` | `src/core/simulation/dynamicState.h` |
| `DynamicStepper` 层级 | `PyDynamicStepper`（kind 标签）→ `Simulation::DynamicStepper` | `src/python/pypgo/simulation/dynamic/core.h`、`src/core/simulation/dynamicStepper.h` |
| `BackwardEulerDynamicStepper` | `Simulation::BackwardEulerStepper` | `src/core/simulation/backwardEuler/` |
| `TRBDF2DynamicStepper` | `Simulation::TRBDF2Stepper` | `src/core/simulation/trbdf2/` |
| `DynamicSimulation` | `PyDynamicSimulation`（持 stepper + 状态） | `src/python/pypgo/simulation/dynamic/core.cpp` |
| 问题/请求值对象 | `DynamicProblem` / `DynamicStepRequest` | `src/core/simulation/dynamicStepOptions.h` |
| 阶段残差复用 | `initStageResidual` / `prepareStageResidual` | `src/core/simulation/common/stageResidual.cpp` |
| `begin_step` 派发 | `dispatchBeginStep` | `src/core/simulation/common/dynamicStepperUtils.cpp:30-41` |

绑定层：`src/python/pypgo/simulation/bindings.cpp`。

## 不变量拆分

C++ 把每步成本拆成两层（`dynamicStepOptions.h` 的设计注释）：

- **`DynamicProblem`（构造期不变）**：质量阵、持久能量项（含每项的 $\alpha_M,\alpha_K$）、固定 DOF 索引集、步长 $h$——`DynamicSimulation` 构造时一次交付，之后不再拷贝；
- **`DynamicStepRequest`（逐步）**：外力 $\mathbf f_{\text{ext}}$、可选的固定 DOF 取值（默认沿用当前位移值）。

求解器配置（容差/迭代数/线搜索）不在两者之内——挂在传给 `step()` 的 [`Optimizer`](../solver/optimizer.md) 对象上。

## 贯穿示例

```python
import numpy as np
import pypgo

n = sim_mesh.num_dofs
M = pypgo.fem.mass_matrix(sim_mesh, mass_field)        # 见 ../fem/mass.md
total = pypgo.energy.EnergySet([(deform, 1.0), (ipc, 1.0)])

sim = pypgo.sim.DynamicSimulation(
    mass=M,
    state=pypgo.sim.DynamicState(
        displacement=np.zeros(n), velocity=np.zeros(n), acceleration=np.zeros(n)),
    timestep=1e-3,
    energy=total,
    integrator=pypgo.sim.TRBDF2DynamicStepper(gamma=0.5),
    damping=(0.0, 0.01),                               # (α_M, α_K)
    fixed_dofs=bottom_dofs,
)

opt = pypgo.solver.NewtonOptimizer(max_iterations=50, gradient_tolerance=1e-6)
for _ in range(200):
    frame = sim.step(external_force=gravity, optimizer=opt)
    if not frame.accepted:
        break
print(sim.state.time, frame.solver_result.status)
```

真实调用见 `pypgo/tools/sim/_runners.py`、`examples/dynamic_sim_api_demo.ipynb`、`tests/pypgo/test_dynamic_stepper.py`。

## 积分器选择速查

| 需求 | 积分器 |
|---|---|
| 鲁棒、数值阻尼大（接触场景默认） | `BackwardEulerDynamicStepper`（一阶，L-稳定） |
| 二阶精度、能量耗散小 | `TRBDF2DynamicStepper`（每步两次 Newton） |
