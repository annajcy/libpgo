# `pypgo/contact/base.py` — `StatefulContactMixin` 步状态混入

> 源文件：`pypgo/contact/base.py`（15 行）。模块架构见 [overview.md](overview.md)。
>
> C++ 协议：`src/core/nonlinearOptimization/stepAwareEnergy.h`（`StepAwareEnergy` / `StepState`）、`stepDependentEnergy.h`（`StepDependentEnergy` 标记类）；Python peer 转发在 `src/python/pypgo/contact/core.cpp:190-207`。

## 定位

接触能量是**长寿命**对象，但其求值可能依赖**当前时间步的状态**：摩擦需要上一步位移 $\mathbf x^t$ 定义切向滑移速度，运动障碍物需要时刻 $t$ 摆放位姿。本混入把 C++ 的步生命周期协议暴露给 Python，由 `SampledPenaltyEnergy` / `IPCEnergy` / `FrictionalSampledPenaltyEnergy` 多继承使用（[energies.md](energies.md)）。

## 数学背景：为什么需要 $\mathbf x^t$

隐式时间积分把一步动力学写成增量势能最小化（见 [../sim/overview.md](../sim/overview.md)）。摩擦势是**半隐式**的：以步内位移差近似切向相对速度

$$\mathbf v_\tau \approx \frac{1}{h}\,(I-\mathbf n\mathbf n^\top)\big(\mathbf x^{t+1}-\mathbf x^t\big)$$

其中 $\mathbf x^t$ 在整步求解期间**冻结**——它必须在 Newton 迭代开始前注入，这正是 `begin_step` 的时机。具体摩擦势 $f_0$ 公式见 [energies.md](energies.md)。

## class `StatefulContactMixin`

无构造函数、无自有状态的纯混入；假定宿主类已有 `_handle`（`_core.PyStatefulContactEnergy` peer）。

### 方法 `begin_step(*, time, timestep, previous_x=None)`

```python
energy.begin_step(time=0.0, timestep=1e-3, previous_x=x_prev)
```

| 参数 | 含义 | C++ 落点 |
|---|---|---|
| `time` | 当前步起始时刻 $t$ | `StepState.time` |
| `timestep` | 步长 $h$ | `StepState.timestep` |
| `previous_x` | 上一步仿真位移 $\mathbf x^t$（`(num_dofs,)`，可省） | `StepState.previousX` |

转发到 C++ peer 的 `begin_step(float(time), float(timestep), previous)`。C++ 侧（`src/python/pypgo/contact/core.cpp:190-207`）`dynamic_cast` 到 `StepAwareEnergy`：

- **不是** `StepAwareEnergy` 的能量（如纯 `SampledPenaltyEnergy`）→ **静默 no-op**，调用安全；
- `IPCEnergy` → `beginStep` 把运动障碍物推进到**步末时刻** $t+h$（`ipcContactEnergy.cpp:119-122`：`setMovingObstacleTime(state.time + state.timestep)`），并清空活动集缓存——隐式积分求的是步末状态，障碍物取步末位姿与之自洽；
- `FrictionalSampledPenaltyEnergy` → 校验并存储 $\mathbf x^t$ 与 $h$ 供摩擦势使用（`sampledPenaltyFrictionState.cpp:28-40`，要求 `previousX != nullptr` 且 `timestep > 0`），同时重置活动集。

**谁来调用**：

1. **动力学循环**——不需要 Python 调用。C++ stepper 在每步开始把 `StepState{time, timestep, previousX=当前位移}` 派发给每个持久能量项（`src/core/simulation/common/dynamicStepperUtils.cpp:30-41` 的 `dispatchBeginStep`，由 `BackwardEulerStepper::step` / `TRBDF2Stepper::step` 调用）；
2. **静力学求解**——求解器没有步概念，须手动初始化一次。真实模式见 `pypgo/tools/sim/_runners.py:36-37`：

```python
for e in bundle.stateful_contacts:
    e.begin_step(time=0.0, timestep=1.0, previous_x=x0)
```

### 属性 `is_step_dependent`

```python
energy.is_step_dependent -> bool
```

C++ 侧 `dynamic_cast<const StepDependentEnergy*>` 判定（`core.cpp:185-188`）：能量**求值结果**是否依赖逐步历史。当前只有 `FrictionalSampledPenaltyEnergy` 为 `True`（摩擦势含 $\mathbf x^t$）；`IPCEnergy` 为 `False`——它虽是 `StepAwareEnergy`（要接收障碍物时刻），但给定障碍物位姿后能量是位置的纯函数。调用方可据此判断"是否必须在每步提供 `previous_x`"。

## 交叉链接

- 宿主能量类：[energies.md](energies.md)
- 动力学循环如何自动派发：[../sim/simulation.md](../sim/simulation.md)、[../sim/stepper.md](../sim/stepper.md)
- 势能基类（`begin_step` 之外的求值接口）：[../energy/base.md](../energy/base.md)
