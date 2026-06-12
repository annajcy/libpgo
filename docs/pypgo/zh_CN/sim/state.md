# `pypgo/sim/state.py` — 逐帧运动学状态与步结果

> 源文件：`pypgo/sim/state.py`（39 行）。模块架构见 [overview.md](overview.md)。
>
> C++ 对应：`Simulation::DynamicState`（`src/core/simulation/dynamicState.h:26-33`）、`Simulation::DynamicStepResult`（`dynamicStepper.h:29-35`）。两个类都是纯 Python 冻结 dataclass——数据载体，无计算。

## class `DynamicState`（冻结 dataclass）

```python
DynamicState(displacement, velocity, acceleration, timestep_id=0, time=0.0)
```

跨时间步携带的运动学状态。这是积分器的**完整**状态：给定 $(\mathbf u^n,\mathbf v^n,\mathbf a^n)$ 与外力，下一步完全确定（Backward Euler 只用 $\mathbf u,\mathbf v$；TRBDF2 的阶段 1 还要 $\mathbf a$，见 [stepper.md](stepper.md)）。

| 字段 | 数学量 | 形状/类型 |
|---|---|---|
| `displacement` | $\mathbf u^n$（相对静止位置的位移） | `(num_dofs,)` |
| `velocity` | $\mathbf v^n$ | `(num_dofs,)` |
| `acceleration` | $\mathbf a^n$ | `(num_dofs,)` |
| `timestep_id` | 已完成步数 | int |
| `time` | 仿真时刻 $t$——`begin_step` 的 `StepState.time` 来源（`dynamicState.h:32` 注释） | float |

两个用途：① `DynamicSimulation(state=...)` 的初值（通常全零静止态）；② `DynamicSimulation.state` 属性的返回类型（当前状态快照）。`__post_init__` 不做形状校验——校验推迟到 `DynamicSimulation` 构造的 `sized_vector`（[../_utils.md](../_utils.md)）。

## class `DynamicFrame`（冻结 dataclass）

```python
DynamicFrame(frame_index, displacement, velocity, acceleration,
             solver_result, stage_results, accepted)
```

单次 [`DynamicSimulation.step`](simulation.md) 的结果。

| 字段 | 含义 |
|---|---|
| `frame_index` | Python 侧帧计数（从 0 起，**含被拒绝的帧**） |
| `displacement` / `velocity` / `acceleration` | 步后状态 $(\mathbf u^{n+1},\mathbf v^{n+1},\mathbf a^{n+1})$；帧被拒绝时为**原状态**（C++ 不前进，`backwardEulerStepper.cpp:51`） |
| `solver_result` | 最终阶段的 [`SolverResult`](../solver/result.md)（`x` 填步后位移，`final_objective` 恒为 `None`——C++ 不回传目标值） |
| `stage_results` | 每个阶段一个 `SolverResult`：隐式 Euler 1 条、TRBDF2（$\gamma<1$）2 条（阶段 1 失败时仅 1 条）。各阶段不保留解向量，`x` 为空数组 |
| `accepted` | 是否接受本步：求解状态 ∈ {Converged, MaxIterations, StepTooSmall}（`acceptsDynamicSolveStatus`，`solverResult.cpp:101-106`）；LinearSolveFailed / ExternalSolverFailure → `False` |

诊断接触行为时常用 `solver_result.diagnostics`：`min_feasible_alpha` / `contact_clamp_count` 反映 IPC 的 CCD 收紧了多少步长（见 [../solver/result.md](../solver/result.md)、[../contact/energies.md](../contact/energies.md) 的 `max_step`）。

## 用法示例

```python
import numpy as np
import pypgo

state0 = pypgo.sim.DynamicState(
    displacement=np.zeros(n), velocity=np.zeros(n), acceleration=np.zeros(n))

frame = sim.step(external_force=gravity)
frame.accepted                       # True/False
frame.solver_result.iterations       # 最终阶段 Newton 迭代数
[s.converged for s in frame.stage_results]   # TRBDF2: [bool, bool]
```

## 交叉链接

- 生产方：[simulation.md](simulation.md)（`DynamicSimulation.step` / `.state`）
- 状态更新公式：[stepper.md](stepper.md)
- `SolverResult` / `SolveDiagnostics` 字段：[../solver/result.md](../solver/result.md)
