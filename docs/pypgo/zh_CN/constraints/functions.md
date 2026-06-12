# `pypgo/constraints/functions.py` — 具体约束函数

> 源文件：`pypgo/constraints/functions.py`（49 行，薄封装）。模块架构见 [overview.md](overview.md)。
>
> 具体约束类型的家：目前有线性约束 `Linear` 和约束拼接器 `ConstraintFunctionSet`。按包内约定这是"预期增长的一侧"——新约束类型加在这里，文件变重时再升格为 `functions/` 子包。两个类都是 [`ConstraintFunction`](base.md)，继承 `value`/`jacobian`/`hessian` 与 `num_dofs`/`num_constraints`/`is_linear`。

---

## class `Linear`

线性向量约束：

$$C(x) = A\,x + c,\qquad J = A\ \text{(常量)},\qquad \nabla^2 C_k = \mathbf 0$$

其中 $A\in\mathbb R^{m\times n}$ 稀疏、$c\in\mathbb R^m$（参数名 `offset`，缺省零向量）。满足 `is_linear=True`——配 [`ConstraintPenalty`](../energy/penalty.md) 时罚能量为精确二次型。能表达的典型约束：

- 固定某 DOF 在值 $v$：$x_i - v = 0$（$A$ 一行一个 1，offset $=-v$）
- 顶点对绑定：$x_i - x_j = 0$
- 平面约束：$\mathbf n^\top \mathbf x_i - d = 0$

C++ 实现：`LinearConstraintFunctions`（`src/core/nonlinearOptimization/constraints/linearConstraintFunctions.cpp`），经 `_core._create_linear_constraint`。

```python
Linear(A, offset=None)
```

| 参数 | 说明 |
|---|---|
| `A` | 任何 [`as_sparse_matrix`](../sparse.md) 接受的形式（稠密 ndarray / `SparseMatrix` / `PySparseMatrix` / COO 5 元组） |
| `offset` | `(m,)` float64，`None` → 零向量；形状必须等于 $A$ 的行数，否则抛 `ValueError`。**拷贝**后传入 C++ |

---

## class `ConstraintFunctionSet`

把多个约束函数按输入顺序**按行拼接**成一个大约束：

$$C(x) = \begin{bmatrix}C^{(1)}(x)\\ \vdots\\ C^{(K)}(x)\end{bmatrix}\in\mathbb R^{\sum_k m_k},\qquad
J = \begin{bmatrix}J^{(1)}\\ \vdots\\ J^{(K)}\end{bmatrix}$$

各成员的 DOF 维 $n$ 必须兼容（在同一全局 DOF 空间上）。拼接保持行序：前 $m_1$ 个分量来自第一个成员，以此类推。

C++ 实现：`ConstraintSet`（`src/core/nonlinearOptimization/constraints/constraintSet.cpp`），经 `_core._create_constraint_function_set`。

```python
ConstraintFunctionSet(constraints)
```

| 参数 | 说明 |
|---|---|
| `constraints` | `ConstraintFunction` 的可迭代序列；非 `ConstraintFunction` 成员抛带项号的 `TypeError` |

## 数学自检

- `Linear` 满足 `is_linear=True`，`jacobian(x)` 与 $A$ 恒等、与 $x$ 无关 ✓
- `offset=None` 时 $C(\mathbf 0)=\mathbf 0$ ✓
- 拼接保持行序：`both.value(x)[0]` 来自第一个成员 ✓

## 用法示例

```python
import numpy as np
import pypgo

n = 6   # 2 个顶点 × 3 DOF
# 约束 1：顶点 0 的 y 分量固定在 0.5 → x[1] - 0.5 = 0
A1 = np.zeros((1, n)); A1[0, 1] = 1.0
c1 = pypgo.constraints.Linear(A1, offset=[-0.5])

# 约束 2：两顶点 z 分量相等 → x[2] - x[5] = 0
A2 = np.zeros((1, n)); A2[0, 2], A2[0, 5] = 1.0, -1.0
c2 = pypgo.constraints.Linear(A2)

both = pypgo.constraints.ConstraintFunctionSet([c1, c2])
both.num_constraints   # 2
```

## 交叉链接

- 抽象接口：[base.md](base.md)
- 矩阵输入约定：[../sparse.md](../sparse.md)
- 软化使用：[../energy/penalty.md](../energy/penalty.md)；加界：[bounded.md](bounded.md)
