# `pypgo/solver/optimizer.py` — `NewtonOptimizer` 阻尼 Newton 优化器

> 源文件：`pypgo/solver/optimizer.py`（61 行，薄封装）。模块架构见 [overview.md](overview.md)。

## 定位

pypgo 目前唯一的具体优化器：投影/约简式阻尼 Newton 法，带可插拔线搜索与稀疏线性求解后端。Python 层只做参数装配（构造 `_core.PyNewtonOptimizerOptions` → `PyNewtonOptimizer`）；算法全部在 C++（`src/core/nonlinearOptimization/solver/newton/NewtonSolver.cpp`，约 750 行）。

## 数学：阻尼 Newton 迭代

每步解阻尼线性系统并做受限线搜索（as-implemented；详细流程见 [overview.md](overview.md)）：

$$\big(\nabla^2E(\mathbf x_k) + \lambda_k\mathbf I\big)\,\Delta\mathbf x = -\nabla E(\mathbf x_k),
\qquad
\mathbf x_{k+1} = \mathbf x_k + \alpha_k\,\Delta\mathbf x$$

**阻尼** $\lambda_k = s_k\,\lambda_0$（`damping=True` 时；加在约简系统的对角上，`NewtonSolver.cpp` `prepareReducedSystem`）：

- $\lambda_0$ = 初始迭代的梯度无穷范数 $\|\mathbf g_0\|_\infty$（问题自适应的尺度）
- $s_k$：梯度不增时衰减 $s_{k+1}=0.9\,s_k$，梯度增大时保持；$\|\mathbf g\|_\infty<10^{-4}$ 或 $s_k<10^{-8}$ 时归零（常数见 `NewtonSolver.cpp:25-27`）

效果：远离解时近似最速下降（大 $\lambda$、步态稳健），接近解时退化为纯 Newton（二阶收敛）。

**步长** $\alpha_k = \min(\alpha_{\text{ls}},\ \alpha_{\text{feas}})$：线搜索结果与能量可行步长上限（`max_step`，CCD/反转防护）之较小者。

**收敛**：$\|\mathbf g_k\|_\infty < \varepsilon$（`gradient_tolerance`）或相对判据 $\|\mathbf g_k\|_\infty < \lambda_0\cdot\text{tol}_{rel}$。

## API

```python
NewtonOptimizer(*,
    max_iterations=50,        # 最大 Newton 迭代数
    gradient_tolerance=1e-6,  # ε：‖∇E‖∞ 绝对收敛阈值
    damping=True,             # 是否启用对角阻尼
    line_search=None,         # LineSearch 策略，None → Backtrack()
    verbose=0,                # C++ 侧日志级别
    sparse_solver=None)       # SparseSolver 后端，None → Auto()
```

- 构造时校验 + 一次性建 C++ peer，**之后不可改参数**——换参数就新建一个（构造很廉价）。
- `line_search` 接受 [line_search.md](line_search.md) 的策略对象；`sparse_solver` 接受 [sparse_solver.md](sparse_solver.md) 的后端对象。类型错误时报带示例的 `TypeError`。
- `solve(problem, x0)` 继承自 [`Optimizer`](base.md)。

## 公式 ↔ 函数表

| 数学量 | 公式 | API 参数 | C++ |
|---|---|---|---|
| 收敛阈值 | $\|\mathbf g\|_\infty<\varepsilon$ | `gradient_tolerance` | `NewtonSolver.cpp` `evaluateCurrentState`/`isConverged` |
| 阻尼系统 | $(\mathbf H+\lambda\mathbf I)\Delta x=-\mathbf g$ | `damping` | `prepareReducedSystem` |
| 阻尼衰减 | $s\leftarrow 0.9\,s$ | （固定常数） | `updateDampingScale` |
| 步长 | $\alpha=\min(\alpha_{\text{ls}},\alpha_{\text{feas}})$ | `line_search` | 主循环 + `computeMaxStepLimit` |
| 线性求解 | LDLT / PARDISO | `sparse_solver` | `newtonSparseSolverBackend.cpp` |

## 用法

```python
import pypgo.solver as ps

opt = ps.NewtonOptimizer(
    max_iterations=80,
    gradient_tolerance=1e-6,
    line_search=ps.Backtrack(armijo_c=1e-4),
    sparse_solver=ps.MKLPardiso(),
)
res = opt.solve(problem, x0)
res.converged, res.iterations, res.final_gradient_norm
```

## 实务提示

- 含 IPC 接触的问题不要关 `damping`：屏障 Hessian 病态时阻尼是稳定器。
- `verbose=1` 起会打印每步的能量 / 梯度 / α，诊断"为什么不收敛"先开这个。
- 收不收敛先看 [`SolverResult.status`](result.md)：`LINE_SEARCH_FAILED` 与 `MAX_ITERATIONS` 的对策不同（前者查能量正确性/可行步长，后者放宽迭代数或初值）。

## 交叉链接

- 求解流程：[base.md](base.md)；问题定义：[problem.md](problem.md)；结果：[result.md](result.md)
- 策略对象：[line_search.md](line_search.md) · [sparse_solver.md](sparse_solver.md)
