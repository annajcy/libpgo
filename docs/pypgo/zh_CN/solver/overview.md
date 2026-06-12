# `pypgo.solver` — 非线性优化模块架构

> 包目录：`pypgo/solver/`（7 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

求解 pypgo 的核心数学问题——带盒式约束的非线性能量最小化：

$$\mathbf x^* \;=\; \arg\min_{\boldsymbol\ell\,\le\,\mathbf x\,\le\,\mathbf u}\; E(\mathbf x)$$

其中 $E$ 是任意 [`PotentialEnergy`](../energy/base.md)（静力学：总势能；动力学：增量势能，见 [../sim/overview.md](../sim/overview.md)）。求解方法是**阻尼 Newton + 线搜索 + 可行步长限制**。

## 理论流水线：一次 Newton 求解

每次迭代（C++ 主循环 `src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp`）：

1. **求值**：$E_k,\ \mathbf g_k=\nabla E(\mathbf x_k),\ \mathbf H_k=\nabla^2E(\mathbf x_k)$（一次虚函数调用 `func_grad_hessian` 同时算三者）
2. **收敛判据**：$\|\mathbf g_k\|_\infty<\varepsilon$（绝对）或 $\|\mathbf g_k\|_\infty<\lambda_0\cdot\text{tol}_{rel}$（相对，$\lambda_0$ 为初始梯度无穷范数）
3. **约简系统**：删去被固定 DOF 的行列（盒式约束中 $\ell_i=u_i$ 的 DOF）
4. **阻尼**（`damping=True` 时）：

   $$\big(\mathbf H_k + \lambda\,\mathbf I\big)\,\Delta\mathbf x = -\mathbf g_k,
   \qquad \lambda = s_k\,\lambda_0$$

   阻尼尺度 $s_k$ 的自适应规则（`NewtonSolver.cpp:25-27, 483-494`）：梯度不增时 $s_{k+1}=0.9\,s_k$；$\|\mathbf g\|_\infty<10^{-4}$ 或 $s<10^{-8}$ 时直接归零（退化为纯 Newton 以获得二阶收敛）。这是 Levenberg–Marquardt 风格的正则化，使远离极小点时的步长更保守、Hessian 不定时仍可解。
5. **稀疏求解**：LDLT / PARDISO 解上式（[sparse_solver.md](sparse_solver.md)）
6. **可行步长**：$\alpha_{\text{feas}} = $ `energy.max_step(x, Δx)`——IPC 的 CCD、本构的反转防护在此生效
7. **线搜索**：在 $(0, \alpha_{\text{feas}}]$ 内选 $\alpha$（[line_search.md](line_search.md)），更新 $\mathbf x_{k+1}=\mathbf x_k+\alpha\,\Delta\mathbf x$

## 模块 ↔ 数学 ↔ 阶段 主表

| 文件 | 角色 | 数学内容 | 文档 |
|---|---|---|---|
| `problem.py` | 问题定义 | $\min E$ s.t. $\ell\le x\le u$ | [problem.md](problem.md) |
| `base.py` | 求解流程抽象 | `solve(problem, x0) → result` | [base.md](base.md) |
| `optimizer.py` | 具体优化器 | 阻尼 Newton | [optimizer.md](optimizer.md) |
| `line_search.py` | 步长策略 | Armijo / 黄金分割 / Brent / 简单收缩 | [line_search.md](line_search.md) |
| `sparse_solver.py` | 线性求解后端 | LDLT / MKL PARDISO | [sparse_solver.md](sparse_solver.md) |
| `result.py` | 结果数据层 | 状态、诊断 | [result.md](result.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

分层约定（包 docstring）：result/problem 是**稳定数据层**；具体优化器在 `optimizer.py` 中增长（未来 LBFGS、trust-region 等都是局部改动）。

## C++ 引擎对应

| 组件 | C++ 位置 |
|---|---|
| Newton 主循环 | `src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp` |
| 优化器外壳 | `src/core/nonlinearOptimization/solver/newton/NewtonOptimizer.cpp` |
| 线搜索算法 | `src/core/nonlinearOptimization/solver/newton/lineSearch.cpp` |
| 线搜索策略对象 | `src/core/nonlinearOptimization/solver/newton/newtonLineSearchPolicy.cpp` |
| 稀疏后端选择 | `src/core/nonlinearOptimization/solver/newton/newtonSparseSolverBackend.cpp` |

绑定层：`src/python/pypgo/solver/bindings.cpp` + `core.cpp`。

## 贯穿示例

```python
import numpy as np
import pypgo
import pypgo.solver as ps

problem = ps.OptimizationProblem(objective=total_energy)
problem.fix_variables(fixed_dofs, np.zeros(len(fixed_dofs)))   # 硬固定

opt = ps.NewtonOptimizer(
    max_iterations=80,
    gradient_tolerance=1e-6,
    damping=True,
    line_search=ps.Backtrack(armijo_c=1e-4, shrink=0.5),
    sparse_solver=ps.Auto(),
)
res = opt.solve(problem, total_energy.zero_state())
assert res.converged, res.status
u = res.x
```
