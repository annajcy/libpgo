# `pypgo/sparse.py` — 稀疏矩阵封装

> 源文件：`pypgo/sparse.py`（178 行，`_core.PySparseMatrix` 的薄封装）。所属架构见 [overview.md](overview.md)。
>
> pypgo 内部的统一稀疏矩阵货币。能量的 Hessian（[energy/base.md](energy/base.md)）、约束的 Jacobian（[constraints/base.md](constraints/base.md)）、嵌入插值矩阵（[mesh/geometry/core.md](mesh/geometry/core.md)）都以 `SparseMatrix` 形式在 Python 与 C++ 之间往返。底层是 Eigen 的压缩稀疏矩阵（`EigenSupport::SpMatD`），矩阵-向量乘在可用时走 MKL 加速。

## 共同数学框架

稀疏矩阵 $A\in\mathbb R^{m\times n}$ 以 COO（坐标）三元组 $(r_k, c_k, v_k)$ 表示，**重复条目求和合并**（Eigen `setFromTriplets` 语义）：

$$A_{ij} = \sum_{k:\ (r_k,c_k)=(i,j)} v_k$$

这正是 FEM 装配需要的行为：各单元对同一矩阵元的贡献直接叠加。本模块所有"COO 进"的入口都继承该语义。

---

## class `SparseMatrix`

拥有所有权的稀疏矩阵句柄。直接构造需传入 `_core.PySparseMatrix` peer（否则抛 `TypeError`）；常规入口是 `from_coo` 类方法或下面的转换函数 `as_sparse_matrix`。

```python
SparseMatrix(core_obj)
#   core_obj: _core.PySparseMatrix — C++ peer，由工厂/转换函数提供
```

### classmethod `from_coo(shape, rows, cols, values)`

```python
SparseMatrix.from_coo(shape, rows, cols, values) -> SparseMatrix
```

| 参数 | 说明 |
|---|---|
| `shape` | `(num_rows, num_cols)` 二元组，分量非负 |
| `rows`, `cols` | COO 行/列索引，一维等长整型 array-like |
| `values` | COO 值，一维等长浮点 array-like；**重复条目由 C++ 构造器求和** |

三个数组必须同为一维且等长，否则抛 `ValueError`。C++ 入口：`_core.create_sparse_matrix`（`src/python/pypgo/sparse/`）。

### 属性 `shape`

```python
A.shape -> tuple[int, int]   # (m, n)
```

### 属性 `nnz`

```python
A.nnz -> int
```

存储的非零元个数（合并后）。

### `to_coo()`

```python
A.to_coo() -> (rows, cols, values)   # int64 (nnz,), int64 (nnz,), float64 (nnz,)
```

导出 COO 三元组（拷贝）。

### `to_dense()`

```python
A.to_dense() -> np.ndarray   # float64 (m, n) 拷贝
```

导出稠密副本。仅用于小矩阵调试/测试——大网格的 Hessian 稠密化会爆内存。

### `__matmul__`（`A @ x`）

```python
A @ x   # x: (n,) → (m,)；X: (n, k) → (m, k)
```

矩阵-向量 / 矩阵-矩阵乘 $y = Ax$，乘法委托给 C++（`PySparseMatrix.matvec` / `matmat`，MKL 可用时加速）。要求 `x.shape[0] == A.shape[1]`，否则抛 `ValueError`；非 1-D/2-D 输入返回 `NotImplemented`。1-D 进 1-D 出、2-D 进 2-D 出。

> 没有实现 `__add__`、转置等其它代数运算——有意保持的最小面：复杂代数请 `to_coo()` 后交给 SciPy，或在 C++ 侧完成。

---

## func `as_sparse_matrix(A)`

```python
as_sparse_matrix(A) -> SparseMatrix
```

把矩阵状输入统一成 `SparseMatrix`，接受四种形式：

| 输入 | 处理 |
|---|---|
| `SparseMatrix` | 原样返回 |
| `_core.PySparseMatrix` | 直接包一层 |
| 稠密 2-D array-like | `np.nonzero` 取非零元转 COO（**零元被丢弃**） |
| 5 元组 `(rows, cols, ri, ci, vals)` | 按 COO 构造 |

4 元组被显式拒绝（`TypeError`，有歧义：分不清是缺 shape 还是缺 values）；其余类型抛带类型名的 `TypeError`。

## func `as_sparse_handle(value, *, name="value")`

```python
as_sparse_handle(value, *, name="value") -> _core.PySparseMatrix
```

`as_sparse_matrix` 的反方向：从 `SparseMatrix`（取其 `_handle`）或裸 `_core.PySparseMatrix` 提取 C++ 句柄，供门面模块把矩阵传回 C++ 构造器。其它类型抛 `TypeError`（消息带 `name`）。

## func `as_coo(A)`

```python
as_coo(A) -> (rows, cols, row_list, col_list, values)
#   rows, cols: int — 矩阵维度
#   row_list, col_list: list[int]；values: float64 ndarray
```

`as_sparse_matrix` + `to_coo`，并把矩阵维度一起返回、行列索引转成 Python `list`——这是部分 C++ COO 构造器期望的形参形状（如 [`QuadraticEnergy`](energy/algebraic.md) 的工厂）。接受与 `as_sparse_matrix` 相同的输入。

## func `sparse_to_coo_lists(matrix)`

```python
sparse_to_coo_lists(matrix) -> (n, rows, cols, values)   # 全部 Python list / int
```

针对**方阵**的另一种 COO-list 形式。除 `SparseMatrix` 与稠密方阵外，还接受 SciPy 稀疏矩阵（鸭子类型探测 `.tocoo()`）。非方阵的稠密输入抛 `ValueError`。

## 用法示例

```python
import numpy as np
from pypgo.sparse import SparseMatrix, as_sparse_matrix

A = SparseMatrix.from_coo((3, 3), rows=[0, 1, 2, 0], cols=[0, 1, 2, 0],
                          values=[1.0, 2.0, 3.0, 1.0])   # (0,0) 处求和 → 2.0
A.shape        # (3, 3)
A.nnz          # 3
A @ np.ones(3) # array([2., 2., 3.])

B = as_sparse_matrix(np.diag([1.0, 2.0]))   # 稠密 → 稀疏（零元丢弃）
```

## 交叉链接

- 生产 Hessian/Jacobian：[energy/base.md](energy/base.md)（`hessian`）、[constraints/base.md](constraints/base.md)（`jacobian`/`hessian`）
- 消费 COO：[energy/algebraic.md](energy/algebraic.md)（`QuadraticEnergy(A, ...)` 经 `as_coo`）
- 嵌入矩阵的来源与用途：[energy/attachment.md](energy/attachment.md)（`EmbeddedVertexAttachment(embedding=...)`）
