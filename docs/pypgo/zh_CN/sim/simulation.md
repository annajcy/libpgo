# `pypgo/sim/simulation.py` — `DynamicSimulation` 仿真引擎

> 源文件：`pypgo/sim/simulation.py`（167 行）。模块架构见 [overview.md](overview.md)。
>
> C++ peer：`PyDynamicSimulation`（持有 `Simulation::DynamicStepper` + 当前 `DynamicState`，`src/python/pypgo/simulation/dynamic/core.cpp:66-141`）；阶段方程见 [stepper.md](stepper.md)。

## 定位

把"质量 + 能量 + 阻尼 + 固定 DOF + 积分器"装配成一个**长寿命**的动力学引擎：构造时一次交付不变量（C++ `DynamicProblem`），之后每帧只传外力与可选的固定 DOF 取值（`DynamicStepRequest`）。每次 `step()` 求解一步增量势能（见 [overview.md](overview.md)），返回 [`DynamicFrame`](state.md)；当前状态由 C++ 侧持有并随接受的步前进。

## class `DynamicSimulation`

```python
DynamicSimulation(*, mass, state, timestep, energy=None,
                  integrator=None, damping=(0.0, 0.0), fixed_dofs=None)
```

| 参数 | 含义 | 校验/落点 |
|---|---|---|
| `mass` | 质量阵 $M$（任何可被 [`sparse_to_coo_lists`](../sparse.md) 接受的稀疏/稠密方阵；DOF 数 $n$ 由它定） | COO 三元组重建为 C++ `SpMatD`（`core.cpp:23-34, 83`） |
| `state` | 初始 [`DynamicState`](state.md)；三个向量都必须是 `(n,)`（`sized_vector` 校验） | C++ `DynamicState`（位移/速度/加速度拷入） |
| `timestep` | 步长 $h>0$ | `DynamicProblem.timestep` |
| `energy` | 总势能 $E$（[`PotentialEnergy`](../energy/base.md)，通常是 [`EnergySet`](../energy/sets.md)；`None` = 纯惯性） | 单个持久项 `ImplicitModelTerm`（`core.cpp:87-95`） |
| `integrator` | [`DynamicStepper`](stepper.md) 实例、`"implicit_euler"` / `"trbdf2"` 字符串或 `None`（默认隐式 Euler；字符串 `"trbdf2"` 取 $\gamma=0.5$） | `makeDynamicStepper(kind, problem, γ)` |
| `damping` | Rayleigh 阻尼系数对 $(\alpha_M,\alpha_K)$ | 见下文 |
| `fixed_dofs` | 硬固定的 DOF 索引列表（**构造期不变**） | `DynamicProblem.fixedDofs` |

### Rayleigh 阻尼

每步开始以当前位移重组装（`src/core/simulation/common/rayleighDampingAssembly.cpp`）：

$$D=\alpha_M\,M+\alpha_K\,K(\mathbf u^n),\qquad K=\nabla^2E\big|_{\mathbf u^n}$$

刚度项要求能量 Hessian 拓扑固定（`isHessianTopologyFixed`，39-43 行）。注意级联：接触能量声明拓扑不固定（`mappedSurfacePotentialEnergy.h:50` 返回 0），而 `EnergySet` 只有全部子项固定才算固定（`energySet.cpp:225-232`）——本门面把整个 `energy` 作为**单一**持久项，故 `EnergySet` 里只要含接触，$\alpha_K$ 就会被整体跳过、只剩 $\alpha_M M$ 生效。需要"弹性有刚度阻尼 + 接触无阻尼"的精细组合时要直接用 C++ 层的多 `ImplicitModelTerm`（Python 门面暂只暴露单项）。

### 固定 DOF

`fixed_dofs` 经求解器的 `fixVariables` 机制钉死（非罚方法；`dynamicStepperUtils.cpp:54-73`），初始猜测同步钉到给定值。每步取值默认沿用当前位移（即"固定在原处"，`dynamicStepperUtils.cpp:43-52`）；要做**指定运动**（dirichlet 随时间变化），逐步传 `fixed_values`。

### 属性 `num_dofs` / `state`

`num_dofs` 返回 $n$；`state` 返回当前状态的 [`DynamicState`](state.md) 快照（直读 C++，含 `timestep_id` 与 `time`——`time` 只随**被接受**的步前进）。

### 方法 `step(*, external_force=None, fixed_values=None, optimizer=None)`

执行一步。GIL 在 C++ `step` 期间释放（`core.cpp:114-117`），长求解不会阻塞其他 Python 线程。

| 参数 | 含义 |
|---|---|
| `external_force` | $\mathbf f_{\text{ext}}$，`(n,)`；`None` = 零 |
| `fixed_values` | `fixed_dofs` 对应的指定位移值（与 `fixed_dofs` 等长）；`None` = 固定在当前值 |
| `optimizer` | [`Optimizer`](../solver/optimizer.md)；`None` = 默认 `NewtonOptimizer()`。**求解器配置每步可换**，与构造期不变量正交 |

内部流程（C++，详见 [overview.md](overview.md) 理论流水线）：派发 `begin_step` 给接触能量 → 组装 $D$ 与阶段二次型 → 逐阶段 Newton → 接受判定与状态更新。返回 [`DynamicFrame`](state.md)。

**接受/拒绝语义**（as-implemented）：

- `accepted = True` ⟺ 各阶段求解状态 ∈ {Converged, MaxIterations, StepTooSmall}（`acceptsDynamicSolveStatus`，`solverResult.cpp:101-106`）——注意 **MaxIterations 也接受**（动力学容忍未严格收敛的步）；
- 拒绝（线性求解失败/外部求解器错误）时 **C++ 状态不前进**：`frame.displacement` 等于步首状态、`sim.state.time` 不变。Python 层**不自动重试或缩步**——回退策略留给调用方（参考 `pypgo/tools/sim/_runners.py:146`：`if not frame.accepted: break`）；
- `frame_index` 是纯 Python 计数，无论接受与否都自增。

### 方法 `run(num_steps, **step_kwargs)`

便捷循环：`[self.step(**kw) for _ in range(num_steps)]`，返回 `DynamicFrame` 列表。**不**在拒绝时提前终止、每步参数恒定——需要逐步变化（移动目标、变外力、拒绝即停）就手写循环（如 [overview.md](overview.md) 贯穿示例）。

## 用法示例

```python
import numpy as np
import pypgo

n = 3
sim = pypgo.sim.DynamicSimulation(
    mass=np.eye(n),
    state=pypgo.sim.DynamicState(np.zeros(n), np.zeros(n), np.zeros(n)),
    timestep=0.05,
    energy=pypgo.energy.QuadraticEnergy(10.0 * np.eye(n)),   # ½·10·‖u‖² 弹簧
    integrator="trbdf2",
)

frame = sim.step(external_force=np.array([1.0, -2.0, 0.5]))
frame.accepted, len(frame.stage_results)    # True, 2（TRBDF2 两阶段）
sim.state.timestep_id                       # 1

# 指定运动：底面随时间下压
for k in range(100):
    vals = np.full(len(bottom_dofs), -0.001 * k)
    frame = sim.step(external_force=gravity, fixed_values=vals)
    if not frame.accepted:
        break
```

真实调用见 `pypgo/tools/sim/_runners.py`（动力学 CLI 主循环）、`tests/pypgo/test_dynamic_stepper.py`、`tests/pypgo/test_sparse_and_sim.py`。

## 交叉链接

- 阶段方程与状态更新公式：[stepper.md](stepper.md)
- 状态/结果类型：[state.md](state.md)
- 接触能量的 `begin_step` 自动派发：[../contact/base.md](../contact/base.md)
- 求解器与诊断：[../solver/optimizer.md](../solver/optimizer.md)、[../solver/result.md](../solver/result.md)
- 质量阵来源：[../fem/mass.md](../fem/mass.md)
