# `pypgo/sim/__init__.py` — 包公开面

> 源文件：`pypgo/sim/__init__.py`（25 行）。模块架构见 [overview.md](overview.md)。

纯再导出模块。包 docstring 点明分工：`state` 是状态类型、`stepper` 是预期增长的一侧（新积分器加在那里）、`simulation` 是引擎。历史注记：`SimulationMesh` 与壳材料工具已迁往 `pypgo.fem.mesh`（[../fem/mesh.md](../fem/mesh.md)），不再从本包导出。

## 导出表（`__all__`，6 个符号）

| 符号 | 来源 | 类型 | 文档 |
|---|---|---|---|
| `DynamicState` | `state.py` | 冻结 dataclass（$\mathbf u,\mathbf v,\mathbf a,t$） | [state.md](state.md) |
| `DynamicFrame` | `state.py` | 冻结 dataclass（单步结果） | [state.md](state.md) |
| `DynamicStepper` | `stepper.py` | 积分器基类门面 | [stepper.md](stepper.md) |
| `BackwardEulerDynamicStepper` | `stepper.py` | 隐式 Euler 积分器 | [stepper.md](stepper.md) |
| `TRBDF2DynamicStepper` | `stepper.py` | 两阶段 TRBDF2 积分器 | [stepper.md](stepper.md) |
| `DynamicSimulation` | `simulation.py` | 仿真引擎 | [simulation.md](simulation.md) |

```python
import pypgo.sim as sim
sim.DynamicSimulation, sim.DynamicState, sim.TRBDF2DynamicStepper  # 均可直接访问
```
