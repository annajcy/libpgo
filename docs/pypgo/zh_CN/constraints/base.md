# `pypgo/constraints/base.py` — `ConstraintFunction` 约束抽象

> 源文件：`pypgo/constraints/base.py`（54 行，只读句柄）。模块架构见 [overview.md](overview.md)。
>
> 向量值硬约束函数的统一接口。与 [../energy/base.md](../energy/base.md) 的 `PotentialEnergy` 平行：同样是 handle peer 模式的不可变只读句柄，C++ peer 类型为 `_core.PyConstraintFunctions`。C++ 虚接口：`src/core/nonlinearOptimization/constraints/constraintFunctions.h`。

## 共同数学框架

约束函数及其导数：

$$C:\mathbb R^n\to\mathbb R^m,\qquad
J(x)=\frac{\partial C}{\partial x}\in\mathbb R^{m\times n}\ \text{(稀疏)}$$

二阶信息以**乘子加权和**形式暴露（这是约束优化的标准形态——拉格朗日函数 $\mathcal L = f + \lambda^\top C$ 的 Hessian 中约束的贡献项）：

$$H_\lambda(x) = \sum_{k=1}^{m}\lambda_k\,\nabla^2 C_k(x)\ \in\mathbb R^{n\times n}$$

罚能量 $\tfrac w2\|C\|^2$ 的精确 Hessian 即 $w(J^\top J + H_{C(x)})$——乘子取 $\lambda=C(x)$（见 [../energy/penalty.md](../energy/penalty.md)）。线性约束的 $H_\lambda\equiv\mathbf 0$。

---

## class `ConstraintFunction`

向量值硬约束的只读句柄。**不供用户直接构造**——`__init__` 要求 `_core.PyConstraintFunctions` peer（否则抛 `TypeError`），常规入口是 [functions.md](functions.md) 中的具体子类。不可变（`__setattr__`/`__delattr__` 抛 `AttributeError`）；所有求值输入自动转 `float64` C 连续。

```python
ConstraintFunction(handle)
#   handle: _core.PyConstraintFunctions — C++ peer
```

### 属性 `num_dofs`

```python
c.num_dofs -> int
```

DOF 维度 $n$（定义域维度）。

### 属性 `num_constraints`

```python
c.num_constraints -> int
```

约束个数 $m$（值域维度）。

### 属性 `is_linear`

```python
c.is_linear -> bool
```

`True` 表示 $C$ 是线性的——`hessian` 恒为零矩阵，且 [`ConstraintPenalty`](../energy/penalty.md) 退化为精确二次型。

### `value(x)`

```python
c.value(x) -> ndarray   # (m,)
```

约束值 $C(x)\in\mathbb R^m$。C++ 虚函数：`func`。

### `jacobian(x)`

```python
c.jacobian(x) -> SparseMatrix   # m × n
```

Jacobian $J(x)$，以 [`SparseMatrix`](../sparse.md) 返回。C++ 虚函数：`jacobian`。

### `hessian(x, multipliers)`

```python
c.hessian(x, multipliers) -> SparseMatrix   # n × n
```

乘子加权 Hessian $H_\lambda(x)=\sum_k\lambda_k\nabla^2C_k(x)$。`multipliers` 形状 `(m,)`。**注意返回矩阵的维度是 DOF 维 $n\times n$ 而非约束维**。C++ 虚函数：`hessian`。

## 数学自检

- 形状约定：`value` 是 $m$ 维（约束个数），`jacobian` 是 $m\times n$，`hessian` 是 $n\times n$（DOF 维）✓
- `is_linear=True` ⟹ `hessian(x, λ)` 为零矩阵 ✓（[functions.md](functions.md) 中的 `Linear`）

## 用法示例

```python
import numpy as np
import pypgo

C = pypgo.constraints.Linear(np.array([[1.0, -1.0]]))   # C(x) = x0 - x1
C.num_dofs, C.num_constraints, C.is_linear   # 2, 1, True
C.value(np.array([3.0, 1.0]))                # array([2.])
J = C.jacobian(np.zeros(2))                  # 1 × 2，恒等于 A
H = C.hessian(np.zeros(2), np.ones(1))       # 2 × 2 零矩阵（线性）
```

## 交叉链接

- 具体实现：[functions.md](functions.md)
- 加界：[bounded.md](bounded.md)
- 软化为能量：[../energy/penalty.md](../energy/penalty.md)
