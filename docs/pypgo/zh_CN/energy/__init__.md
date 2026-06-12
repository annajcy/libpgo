# `pypgo/energy/__init__.py` — 公开面

> 源文件：`pypgo/energy/__init__.py`（29 行，纯重导出）。模块架构见 [overview.md](overview.md)。

## 定位

把按概念分布在各文件中的能量类型重导出为**扁平公开面**：用户写 `pypgo.energy.QuadraticEnergy` 而不需要知道它住在 `algebraic.py`。模块 docstring 同时是本包 handle-peer 约定的权威说明（要点已收录于 [overview.md](overview.md)）。

## 导出表

| 符号 | 来源文件 | 数学对象 |
|---|---|---|
| `PotentialEnergy` | `base.py` | 势能抽象基类 |
| `LinearEnergy` | `algebraic.py` | $b^\top x$ |
| `QuadraticEnergy` | `algebraic.py` | $\tfrac12 x^\top Ax + b^\top x$ |
| `VertexAttachment` | `attachment.py` | 顶点软固定 |
| `EmbeddedVertexAttachment` | `attachment.py` | 嵌入点软固定 |
| `ConstraintPenalty` | `penalty.py` | $\tfrac{w}{2}\|C(x)\|^2$ |
| `ConstraintViolationPenalty` | `penalty.py` | 违界罚 |
| `EnergySet` | `sets.py` | $\sum_i w_i E_i$ |

`__all__` 与上表一致（按字母序）。
