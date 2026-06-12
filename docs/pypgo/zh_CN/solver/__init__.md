# `pypgo/solver/__init__.py` — 公开面

> 源文件：`pypgo/solver/__init__.py`（53 行，纯重导出）。模块架构见 [overview.md](overview.md)。

## 定位

重导出扁平公开面，保证 `pypgo.solver.NewtonOptimizer` 等路径稳定。docstring 声明分层：result/problem 是稳定数据层，优化器是增长侧。

## 导出表

| 符号 | 来源 | 角色 | 文档 |
|---|---|---|---|
| `OptimizationProblem`, `Bounds` | `problem.py` | 问题定义 | [problem.md](problem.md) |
| `Optimizer` | `base.py` | 优化器抽象 | [base.md](base.md) |
| `NewtonOptimizer` | `optimizer.py` | 阻尼 Newton | [optimizer.md](optimizer.md) |
| `LineSearch`, `Golden`, `Brents`, `Backtrack`, `Simple` | `line_search.py` | 线搜索策略 | [line_search.md](line_search.md) |
| `SparseSolver`, `Auto`, `EigenLDLT`, `MKLPardiso`, `OrigPardiso` | `sparse_solver.py` | 线性求解后端 | [sparse_solver.md](sparse_solver.md) |
| `SolverResult`, `SolveStatus`, `SolveDiagnostics` | `result.py` | 结果数据 | [result.md](result.md) |

习惯写法：`import pypgo.solver as ps`，然后 `ps.NewtonOptimizer(line_search=ps.Backtrack())`。
