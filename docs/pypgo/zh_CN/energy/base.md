# `pypgo/energy/base.py` — `PotentialEnergy` 势能抽象

> 源文件：`pypgo/energy/base.py`（70 行，C++ 虚接口的只读句柄）。模块架构见 [overview.md](overview.md)。
>
> 整个 pypgo 数值栈的中心抽象。所有能量类型——本包的代数能量、`pypgo.fem.DeformationEnergy`、`pypgo.contact` 的接触能量——都是 `PotentialEnergy`，求解器只面对这一个接口。C++ 虚基类：`src/core/nonlinearOptimization/potentialEnergy.h`。

## 共同数学框架

一个势能是定义在 DOF 向量 $\mathbf x\in\mathbb R^n$ 上的标量场及其前两阶导数：

$$E:\mathbb R^n\to\mathbb R,\qquad
\mathbf g(\mathbf x)=\nabla E\in\mathbb R^n,\qquad
\mathbf H(\mathbf x)=\nabla^2 E\in\mathbb R^{n\times n}\ \text{(稀疏对称)}$$

变分力学解释：$\mathbf x$ 通常是位移（`state_kind` 区分位移/位置），力 $\mathbf f=-\mathbf g$，刚度矩阵 $\mathbf K=\mathbf H$。

额外的第四个量是**可行步长上限**：给定当前点 $\mathbf x$ 与搜索方向 $\Delta\mathbf x$，

$$\alpha_{\max} = \sup\{\alpha \ge 0:\ \mathbf x + \alpha\,\Delta\mathbf x \in \operatorname{dom} E\}$$

对多数能量 $\alpha_{\max}=+\infty$；对有定义域边界的能量（IPC 屏障要求距离 $d>0$、neo-Hookean 要求 $\det\mathbf F>0$）这是 Newton 线搜索不越界的关键（CCD 即在此处接入，见 [../contact/energies.md](../contact/energies.md)）。

---

## class `PotentialEnergy`

任意 C++ 势能的只读句柄。**不供用户直接构造**——`__init__` 要求传入 `_core.PyPotentialEnergy` peer（否则抛 `TypeError`），常规入口是各具体子类（`LinearEnergy`、`QuadraticEnergy`、`VertexAttachment`、`EnergySet` 以及 fem/contact 的领域能量）。

```python
PotentialEnergy(handle)
#   handle: _core.PyPotentialEnergy — C++ peer
```

两个贯穿性设计：

- **不可变**——`__setattr__`/`__delattr__` 抛 `AttributeError`；peer 由 `object.__setattr__` 绕过限制存入。
- **`isinstance` 全栈通用**——FEM/接触/代数能量都满足 `isinstance(e, PotentialEnergy)`，这是 [`EnergySet`](sets.md) 与 [`OptimizationProblem`](../solver/problem.md) 的类型检查依据。

### 属性 `num_dofs`

```python
e.num_dofs -> int
```

DOF 维度 $n$。所有求值方法的输入 `x` 必须是 `(n,)`。

### 属性 `dofs`

```python
e.dofs -> ndarray
```

该能量涉及的全局 DOF 索引（C++ `getDOFs`）。[`EnergySet`](sets.md) 据此把各项对齐到全局 DOF 空间。

### 属性 `state_kind`

```python
e.state_kind -> str
```

状态语义（位移 / 位置）。混合不同 `state_kind` 的能量是逻辑错误——求和前应核对一致。

### `zero_state()`

```python
e.zero_state() -> ndarray   # (n,) 全零
```

返回 $\mathbf 0\in\mathbb R^n$，作为初值/缓冲的便捷构造。

### `value(x)`

```python
e.value(x) -> float
```

能量值 $E(\mathbf x)$。输入自动转 `float64`。C++ 虚函数：`PotentialEnergy::func`。

### `gradient(x)`

```python
e.gradient(x) -> ndarray   # (n,)
```

梯度 $\nabla E(\mathbf x)$（即负内力）。C++ 虚函数：`PotentialEnergy::gradient`。

### `hessian(x)`

```python
e.hessian(x) -> SparseMatrix   # n × n
```

Hessian $\nabla^2 E(\mathbf x)$，以 [`pypgo.sparse.SparseMatrix`](../sparse.md) 返回。C++ 侧走 `hessianInPlace`：稀疏拓扑在构造时由 `hessianAlloc` 固定一次，之后每次求值只原位填值——这也是 Newton 求解器能复用符号分解的前提。

### `max_step(x, dx)`

```python
e.max_step(x, dx) -> float
```

可行步长上限 $\alpha_{\max}(\mathbf x,\Delta\mathbf x)$（C++ `computeMaxStepLimit`）。Newton 主循环取 $\alpha=\min(\alpha_{\text{ls}}, \alpha_{\max})$（见 [../solver/optimizer.md](../solver/optimizer.md)）。

## 数学自检

- 对二次能量 $E=\tfrac12\mathbf x^\top A\mathbf x$：`gradient(0) = 0`、`hessian(x)` 与 $A$ 恒等、`value(0) = 0` ✓（见 [algebraic.md](algebraic.md)）
- 梯度的有限差分一致性：$\frac{E(\mathbf x+h\mathbf e_i)-E(\mathbf x-h\mathbf e_i)}{2h}\approx g_i$——引擎自带 FD 测试设施（`src/core/nonlinearOptimization/` 下的 finiteDifference 工具）。

## 用法示例

```python
import numpy as np
from pypgo.energy import QuadraticEnergy, PotentialEnergy

e = QuadraticEnergy(np.eye(3))
isinstance(e, PotentialEnergy)   # True
x = e.zero_state()
e.value(x), e.gradient(x)        # 0.0, [0, 0, 0]
H = e.hessian(x)                 # SparseMatrix(shape=(3, 3), nnz=3)
```

## 交叉链接

- 具体能量：[algebraic.md](algebraic.md) · [attachment.md](attachment.md) · [penalty.md](penalty.md) · [sets.md](sets.md)
- 消费方：[../solver/problem.md](../solver/problem.md)（`OptimizationProblem(objective=...)`）
- Hessian 载体：[../sparse.md](../sparse.md)
