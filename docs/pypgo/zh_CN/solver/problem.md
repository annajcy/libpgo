# `pypgo/solver/problem.py` — 优化问题定义

> 源文件：`pypgo/solver/problem.py`（92 行）。模块架构见 [overview.md](overview.md)。
>
> 描述**数学问题本身**——目标函数和变量界——与求解器选择（[optimizer.md](optimizer.md)）解耦。同一 problem 可交给不同 optimizer、或修改界后重复求解。

## 共同数学框架

$$\min_{\mathbf x\in\mathbb R^n} E(\mathbf x)\qquad
\text{s.t.}\quad \boldsymbol\ell \le \mathbf x \le \mathbf u$$

盒式约束的两种用法：

1. **固定 DOF（Dirichlet 边界）**：令 $\ell_i = u_i = v_i$，DOF $i$ 被钉死在 $v_i$。Newton 求解时这些 DOF 的行列直接从线性系统中删去（系统**降维**，而不是罚），见 [overview.md](overview.md) 流程第 3 步。
2. **不等式界**：$\ell_i < u_i$ 的一般盒式约束。

---

## class `Bounds`

变量上下界的纯数据载体（`__slots__ = ("lower", "upper")`，无任何校验/计算）。

```python
Bounds(lower=None, upper=None)
```

| 参数 | 说明 |
|---|---|
| `lower` | `(n,)` array-like 或 `None`（该侧无界） |
| `upper` | 同上 |

---

## class `OptimizationProblem`

非线性优化问题：目标能量 + 可编辑的变量界。

```python
OptimizationProblem(*, objective)
```

| 参数 | 说明 |
|---|---|
| `objective` | [`PotentialEnergy`](../energy/base.md)（含 [`EnergySet`](../energy/sets.md) 等一切能量）；类型不符抛 `TypeError`。C++ peer 经 `_core._create_optimization_problem(objective._handle)` 构造 |

### 属性 `objective`

```python
problem.objective -> PotentialEnergy
```

公开属性，构造时传入的目标能量。

### 属性 `variable_bounds`

```python
problem.variable_bounds -> Bounds   # 可整体替换、可改字段
```

普通 Python 对象，可以随时编辑；[`Optimizer.solve`](base.md) 在进入 C++ 前调用私有方法 `_sync_variable_bounds_to_handle()` 整体推送（`None` 一侧以空数组 + `False` 标志传给 `set_variable_bounds`，表示无界）。**延迟同步**是有意设计：界的编辑不触发任何 C++ 调用。

### `fix_variables(dofs, values, *, num_dofs=None)`

```python
problem.fix_variables(dofs, values, *, num_dofs=None) -> None
```

| 参数 | 说明 |
|---|---|
| `dofs` | 要固定的 DOF 索引序列；必须**唯一**且在 `[0, num_dofs)` 内，否则抛 `ValueError` |
| `values` | 与 `dofs` 等长的固定值（一维 float64） |
| `num_dofs` | 总 DOF 数；`None` 时取 `objective.num_dofs` |

实现用法①（硬固定）：把指定 DOF 的上下界同时设为给定值

$$\ell_{d} = u_{d} = v_d,\quad d\in\texttt{dofs}$$

其余 DOF 维持已有界（无界处填 $\mp\infty$），最后整体替换 `variable_bounds`。已有界数组长度与 `num_dofs` 不符时抛 `ValueError`。

### 与软固定的对比

| | 硬固定（本类） | 软固定（[VertexAttachment](../energy/attachment.md)） |
|---|---|---|
| 机制 | 从线性系统删行列 | 罚能量项 |
| 精确性 | 精确满足 | 受 `coeff` 控制的近似 |
| 适用 formulation | 节点位移 DOF | 任意（含 Hermite 导数 DOF，配 `EmbeddedVertexAttachment`） |
| 改目标 | 改 bounds 即可 | `set_targets` |

## 用法示例

```python
import numpy as np
import pypgo.solver as ps

problem = ps.OptimizationProblem(objective=total_energy)

# 固定底面顶点（DOF 级）：
fixed_dofs = (bottom_ids[:, None] * 3 + np.arange(3)).ravel()
problem.fix_variables(fixed_dofs, np.zeros(fixed_dofs.size))

# 或一般盒式界：
problem.variable_bounds = ps.Bounds(
    lower=np.full(n, -1.0), upper=np.full(n, 1.0))

res = ps.NewtonOptimizer().solve(problem, x0)
```

## 交叉链接

- 目标能量：[../energy/base.md](../energy/base.md) · [../energy/sets.md](../energy/sets.md)
- 求解入口：[base.md](base.md) · [optimizer.md](optimizer.md)
- 软固定替代：[../energy/attachment.md](../energy/attachment.md)
