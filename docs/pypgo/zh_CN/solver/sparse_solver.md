# `pypgo/solver/sparse_solver.py` — Newton 步稀疏线性求解后端

> 源文件：`pypgo/solver/sparse_solver.py`（41 行，后端选择工厂）。模块架构见 [overview.md](overview.md)。

## 定位

选择求解 Newton 线性系统

$$\big(\nabla^2E + \lambda\mathbf I\big)\,\Delta\mathbf x = -\nabla E$$

的稀疏直接法后端。每个工厂返回一个 `_core.PySparseSolver` 子类的不可变句柄，传给 [`NewtonOptimizer(sparse_solver=...)`](optimizer.md)。后端调度在 `src/core/nonlinearOptimization/solver/newton/newtonSparseSolverBackend.cpp`。

## 数学背景

系统矩阵是稀疏对称矩阵。阻尼开启（$\lambda>0$）时正定；纯 Newton（$\lambda=0$）下在鞍点/非凸区域可能不定——因此采用 **LDLT**（$A = LDL^\top$，$D$ 块对角）类分解而非 Cholesky：LDLT 对对称不定矩阵也适用。

## 后端

| 工厂 | 后端 | 说明 |
|---|---|---|
| `Auto()` | 自动 | 让 C++ 侧挑可用的最优后端（有 MKL 用 PARDISO，否则 Eigen） |
| `EigenLDLT()` | Eigen `SimplicialLDLT` | 总是可用；单线程，中小规模够用 |
| `MKLPardiso()` | Intel MKL PARDISO | 多线程超节点分解，大网格（>10⁵ DOF）显著更快；需 MKL 构建 |
| `OrigPardiso()` | 原版（非 MKL）PARDISO | 历史后端，需相应许可/构建 |

公共基类型 `SparseSolver = _core.PySparseSolver`。后端今天没有可调参数；用对象而非字符串是为了与 `line_search` 形式对齐、给未来的逐后端选项留位置（模块 docstring 言明）。

## 用法

```python
import pypgo.solver as ps

opt = ps.NewtonOptimizer(sparse_solver=ps.Auto())        # 默认
opt = ps.NewtonOptimizer(sparse_solver=ps.MKLPardiso())  # 强制 MKL
```

请求了不可用的后端时，求解返回 [`SolveStatus.UNSUPPORTED_BACKEND`](result.md)。

## 交叉链接

- 消费方：[optimizer.md](optimizer.md)；失败状态：[result.md](result.md)
- 稀疏矩阵本体：[../sparse.md](../sparse.md)
