# `pypgo/solver/base.py` — `Optimizer` 抽象与共享求解流程

> 源文件：`pypgo/solver/base.py`（33 行，薄封装）。模块架构见 [overview.md](overview.md)。
>
> 所有优化器的抽象基类，handle peer 模式（peer 类型 `_core.PyOptimizer`）。把"求解一个问题"的共享流程集中在唯一的 `solve` 方法里，具体算法（Newton 等）只负责构造各自的 peer。

---

## class `Optimizer`

抽象基类。用户不直接实例化——入口是 [`NewtonOptimizer`](optimizer.md)。

```python
Optimizer(handle)
#   handle: _core.PyOptimizer — 由具体子类构造并传入；类型不符抛 TypeError
```

### `solve(problem, x0)`

```python
opt.solve(problem: OptimizationProblem, x0) -> SolverResult
```

| 参数 | 说明 |
|---|---|
| `problem` | [`OptimizationProblem`](problem.md)；类型不符抛 `TypeError` |
| `x0` | 初值 $\mathbf x_0\in\mathbb R^n$，一维 float64（经 [`float_vector`](../_utils.md)），$n$ = 目标能量的 `num_dofs` |

实现就是三行，对应三个设计决定：

```python
problem._sync_variable_bounds_to_handle()   # ① Python 侧可变界 → C++
x0_arr = float_vector("x0", x0)             # ② 校验初值
return _result_from_core(self._handle.solve(problem._handle, x0_arr))  # ③ C++ 求解 → 数据类
```

1. **界的延迟同步**——`OptimizationProblem.variable_bounds` 在 Python 侧自由编辑，每次 `solve` 前才整体推给 C++（见 [problem.md](problem.md)），所以同一 problem 可以改界后反复求解；
2. 初值校验失败抛带 `"x0"` 的 `ValueError`；
3. C++ 返回 dict，由 [`_result_from_core`](result.md) 转成冻结的 `SolverResult`。

## 用法示例

```python
import pypgo.solver as ps

opt = ps.NewtonOptimizer()                       # 具体子类
res = opt.solve(problem, total_energy.zero_state())
res.converged, res.x
```

## 交叉链接

- 问题侧：[problem.md](problem.md)；结果侧：[result.md](result.md)
- 具体优化器：[optimizer.md](optimizer.md)
