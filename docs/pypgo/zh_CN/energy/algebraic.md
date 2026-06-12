# `pypgo/energy/algebraic.py` — 线性与二次能量

> 源文件：`pypgo/energy/algebraic.py`（57 行，薄封装）。模块架构见 [overview.md](overview.md)。
>
> 闭式代数能量，是能量栈的"积木"：恒定外力（线性项）、任意常 Hessian 项（二次型）。`EmbeddedVertexAttachment`（[attachment.md](attachment.md)）等更高层能量在内部也归约为 `QuadraticEnergy`。两个类都继承 [`PotentialEnergy`](base.md) 的全部求值接口与不可变语义。

---

## class `LinearEnergy`

线性势能——恒定外力的势。**as-implemented**（`src/core/genericPotentialEnergies/linearPotentialEnergy.h:26`，`func` 即 `x.dot(b_)`）：

$$E(\mathbf x) = \mathbf b^\top\mathbf x,\qquad \nabla E = \mathbf b,\qquad \nabla^2 E = \mathbf 0$$

对应恒定外力 $\mathbf f = -\mathbf b$（如重力在位移坐标下的势能）。Hessian 恒为零矩阵（`hessianInPlace` 是空操作）。

```python
LinearEnergy(b)
```

| 参数 | 说明 |
|---|---|
| `b` | `(n,)` float64（经 [`float_vector`](../_utils.md) 规整）。**拷贝**进 C++ 所有权存储，之后改原数组不影响能量 |

绑定工厂：`_core._create_linear_energy`（`src/python/pypgo/energy/bindings.cpp`）。

---

## class `QuadraticEnergy`

二次势能。**as-implemented**（`src/core/genericPotentialEnergies/quadraticPotentialEnergy.cpp:131-148`，`func` 为 `vTMv(A,x)*0.5 + b·x`、`gradient` 为 `mv(A,x) + b`）：

$$E(\mathbf x) = \tfrac12\,\mathbf x^\top A\,\mathbf x + \mathbf b^\top\mathbf x,\qquad
\nabla E = A\mathbf x + \mathbf b,\qquad \nabla^2 E = A$$

> **应传入对称矩阵 $A$**：梯度按 $A\mathbf x+\mathbf b$ 计算（而非对称化的 $\tfrac12(A+A^\top)\mathbf x+\mathbf b$）。能量值本身只感知 $A$ 的对称部分，但梯度/Hessian 会原样使用 $A$——非对称输入会导致梯度与能量不一致。

```python
QuadraticEnergy(A, b=None)
```

| 参数 | 说明 |
|---|---|
| `A` | 稠密 `(m,m)` ndarray / `SparseMatrix` / `PySparseMatrix` / COO 5 元组——任何 [`as_coo`](../sparse.md) 接受的形式。稠密输入自动转稀疏 COO（零元丢弃） |
| `b` | `(m,)` float64 或 `None`（无线性项） |

绑定工厂：`_core._create_quadratic_energy_from_coo` / `_create_quadratic_energy_from_coo_with_b`。`hessian(x)` 直接返回常量 $A$；Hessian 拓扑构造时固定。

## 数学自检

- $E(\mathbf 0)=0$ 对两者均成立（无常数项）✓
- `QuadraticEnergy` 在 $\mathbf x^*=-A^{-1}\mathbf b$ 处梯度为零 ✓
- 量纲：$A$ 是刚度量纲（能量/长度²），$\mathbf b$ 是力量纲（能量/长度）。

## 用法示例

```python
import numpy as np
from pypgo.energy import LinearEnergy, QuadraticEnergy

# 最小化 ½xᵀAx + bᵀx，解析解 x* = -A⁻¹b
A = np.array([[4.0, 1.0], [1.0, 3.0]])
b = np.array([1.0, 2.0])
e = QuadraticEnergy(A, b)
e.value(np.zeros(2))                       # 0.0
np.allclose(e.gradient(np.zeros(2)), b)    # True

g = LinearEnergy(np.array([0.0, 9.8]))     # 恒力势
```

## 交叉链接

- 求值接口：[base.md](base.md)
- 矩阵输入约定：[../sparse.md](../sparse.md)（`as_coo` 接受的形式）
- 在其上构建的能量：[attachment.md](attachment.md)（`EmbeddedVertexAttachment`）
