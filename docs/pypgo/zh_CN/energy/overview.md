# `pypgo.energy` — 通用势能模块架构

> 包目录：`pypgo/energy/`（6 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

pypgo 把一切力学量都表达为**势能** $E(\mathbf x)$：弹性是势能（[../fem/overview.md](../fem/overview.md)），接触是势能（[../contact/overview.md](../contact/overview.md)），边界条件（软固定）也是势能。求解器（[../solver/overview.md](../solver/overview.md)）只面对一个统一的接口：

$$E(\mathbf x),\qquad \mathbf g = \nabla E(\mathbf x),\qquad \mathbf H = \nabla^2 E(\mathbf x)$$

本包定义这个统一接口（`PotentialEnergy`），并提供与具体物理无关的通用能量类型：代数能量、软固定、约束罚、能量组合。

## 理论流水线中的位置

```
fem.DeformationEnergy ┐
contact.IPCEnergy     ├──> energy.EnergySet（Σ wᵢEᵢ）──> solver.OptimizationProblem
energy.VertexAttachment┘
```

变分力学的基本结构：总能量是各物理项的加权和，平衡态是总能量的驻点

$$\mathbf x^* = \arg\min_{\mathbf x} \sum_i w_i E_i(\mathbf x)$$

力是能量的负梯度 $\mathbf f = -\nabla E$，刚度是 Hessian $\mathbf K = \nabla^2 E$。

## 模块 ↔ 数学 ↔ 阶段 主表

| 文件 | 数学对象 | 公式 | 文档 |
|---|---|---|---|
| `base.py` | 势能抽象 | $E,\ \nabla E,\ \nabla^2 E,\ \alpha_{\max}$ | [base.md](base.md) |
| `algebraic.py` | 线性 / 二次型 | $b^\top x$；$\tfrac12 x^\top Ax+b^\top x$ | [algebraic.md](algebraic.md) |
| `attachment.py` | 顶点软固定 | $\tfrac{c}{2}\sum_i\|u_i+\bar x_i - t_i\|^2$；$c\|W_s u\|^2$ | [attachment.md](attachment.md) |
| `penalty.py` | 约束 → 罚能量 | $\tfrac{w}{2}\|C(x)\|^2$；$\tfrac{w}{2}\|r(x)\|^2$ | [penalty.md](penalty.md) |
| `sets.py` | 能量组合 | $\sum_i w_i E_i$ | [sets.md](sets.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## C++ 引擎对应

| Python | C++ 类 | 位置 |
|---|---|---|
| `PotentialEnergy` 接口 | `NonlinearOptimization::PotentialEnergy`（虚基类） | `src/core/nonlinearOptimization/potentialEnergy.h` |
| `LinearEnergy` | `PredefinedPotentialEnergies::LinearPotentialEnergy` | `src/core/genericPotentialEnergies/linearPotentialEnergy.h` |
| `QuadraticEnergy` | `PredefinedPotentialEnergies::QuadraticPotentialEnergy` | `src/core/genericPotentialEnergies/quadraticPotentialEnergy.cpp` |
| `VertexAttachment` | `ConstraintPotentialEnergies::MultipleVertexPulling` | `src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.cpp` |
| `EmbeddedVertexAttachment` | （纯 Python 组装 → `QuadraticPotentialEnergy`） | `pypgo/energy/attachment.py` |
| `ConstraintPenalty` 等 | `NonlinearOptimization::PotentialEnergy(Bounded)ConstraintFunctions` | `src/core/nonlinearOptimization/constraints/potentialEnergyFromConstraintFunctions.cpp` |
| `EnergySet` | `NonlinearOptimization::EnergySet` | `src/core/nonlinearOptimization/` |

绑定层：`src/python/pypgo/energy/bindings.cpp`（注册 `_core._create_*` 工厂）+ `core.cpp`（包装实现）。

## handle peer 约定（本包 docstring 原文的要点）

- 每个 Python 门面把具体的 C++ `PyXXXX` peer 存在 `_handle`；
- 所有能量 peer 继承 `_core.PyPotentialEnergy`，C++ 内部需要核心 `PotentialEnergy*` 时通过 `potentialEnergyHandle()` 取得；
- 求值经虚函数派发，Python 用户接触不到 `hessianInPlace` / `hessianAlloc` / `isHessianTopologyFixed`；
- `PotentialEnergy` 是稳定契约；具体能量按概念分文件（algebraic / penalty / attachment / sets），是预期增长的一侧。

## 贯穿示例

```python
import numpy as np
import pypgo

# 二次能量 ½xᵀAx + bᵀx：最小点是 Ax = -b 的解
A = np.array([[2.0, 0.0], [0.0, 4.0]])
b = np.array([-2.0, -8.0])
quad = pypgo.energy.QuadraticEnergy(A, b)

quad.value(np.zeros(2))      # 0.0
quad.gradient(np.zeros(2))   # array([-2., -8.]) = b
H = quad.hessian(np.zeros(2))  # SparseMatrix == A

# 与另一项组成加权和
total = pypgo.energy.EnergySet([(quad, 1.0)])
problem = pypgo.solver.OptimizationProblem(objective=total)
res = pypgo.solver.NewtonOptimizer().solve(problem, np.zeros(2))
res.x   # ≈ [1., 2.]（解 Ax=-b）
```
