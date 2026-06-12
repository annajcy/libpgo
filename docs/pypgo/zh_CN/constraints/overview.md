# `pypgo.constraints` — 硬约束模块架构

> 包目录：`pypgo/constraints/`（4 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

表示向量值**硬约束函数**

$$C:\mathbb R^n\to\mathbb R^m$$

及其一、二阶导数，并提供区间界封装 $\ell\le C(x)\le u$。本包只负责"约束的定义与求导"；**强制方式**在别处决定：

- **软强制**（目前的主路径）：经 [../energy/penalty.md](../energy/penalty.md) 转成罚能量 $\tfrac w2\|C\|^2$ 加入目标函数；
- **逐 DOF 硬界**：简单的盒式约束（固定顶点）走 [../solver/problem.md](../solver/problem.md) 的 `variable_bounds`，不经过本包。

## 理论流水线中的位置

```
constraints.Linear / ConstraintFunctionSet      （定义 C(x), J, Σλₖ∇²Cₖ）
        │
        ├── Bounded(lower, upper)               （附加界）
        ▼
energy.ConstraintPenalty / ConstraintViolationPenalty   （软化为势能）
        ▼
solver.OptimizationProblem                      （进入总目标）
```

## 模块 ↔ 数学 ↔ 阶段 主表

| 文件 | 数学对象 | 公式 | 文档 |
|---|---|---|---|
| `base.py` | 约束抽象 | $C(x)$、$J=\partial C/\partial x$、$\sum_k\lambda_k\nabla^2C_k$ | [base.md](base.md) |
| `functions.py` | 具体约束 | $C(x)=Ax+c$；约束拼接 | [functions.md](functions.md) |
| `bounded.py` | 区间界 | $\ell\le C(x)\le u$ | [bounded.md](bounded.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## C++ 引擎对应

| Python | C++ | 位置 |
|---|---|---|
| `ConstraintFunction` 接口 | `NonlinearOptimization` 约束函数虚接口 | `src/core/nonlinearOptimization/constraints/constraintFunctions.h` |
| `Linear` | `LinearConstraintFunctions` | `src/core/nonlinearOptimization/constraints/linearConstraintFunctions.cpp` |
| `ConstraintFunctionSet` | `ConstraintSet`（拼接） | `src/core/nonlinearOptimization/constraints/constraintSet.cpp` |
| （罚能量消费方） | `PotentialEnergy(Bounded)ConstraintFunctions` | `src/core/nonlinearOptimization/constraints/potentialEnergyFromConstraintFunctions.cpp` |

绑定层：`src/python/pypgo/constraints/bindings.cpp`。

## 设计约定

- `ConstraintFunction` 是稳定契约（只读句柄，handle peer 模式同 [../energy/base.md](../energy/base.md)）；具体约束类型集中在 `functions.py`，是预期增长的一侧。
- `Bounded` 是纯 Python 冻结 dataclass——界以 NumPy 数组存在 Python 侧，构造罚能量时才传给 C++。

## 贯穿示例

```python
import numpy as np
import pypgo

# C(x) = [x0 + x1 - 1, x2] ∈ R²，要求 C(x) = 0
A = np.array([[1.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]])
C = pypgo.constraints.Linear(A, offset=[-1.0, 0.0])
C.num_constraints, C.num_dofs, C.is_linear   # (2, 3, True)

x = np.array([0.3, 0.7, 0.0])
C.value(x)              # array([0., 0.]) — 满足
J = C.jacobian(x)       # SparseMatrix == A

# 软强制后进入能量栈
penalty = pypgo.energy.ConstraintPenalty(C, weight=1e6)
```
