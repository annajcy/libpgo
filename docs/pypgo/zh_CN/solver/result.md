# `pypgo/solver/result.py` — 求解状态与结果

> 源文件：`pypgo/solver/result.py`（71 行，纯 Python 数据层）。模块架构见 [overview.md](overview.md)。
>
> `Optimizer.solve` 的返回数据：最终迭代点、收敛状态、迭代统计与诊断。全部为冻结 dataclass / `IntEnum`，C++ 以 dict 形式返回、由模块私有函数 `_result_from_core` 转换。本模块无任何计算逻辑。

---

## class `SolveStatus`（IntEnum）

终止状态码。与 C++ 侧状态一一对应（`_result_from_core` 中 `SolveStatus(int(data["status"]))`）。

| 值 | 含义 | 常见原因与对策 |
|---|---|---|
| `CONVERGED = 0` | $\|\nabla E\|_\infty$ 达到阈值（绝对或相对判据） | — |
| `MAX_ITERATIONS = 1` | 用尽 `max_iterations` | 放宽迭代数 / 改善初值 / 检查能量缩放 |
| `LINE_SEARCH_FAILED = 2` | 找不到使能量下降的步长 | 检查梯度正确性、可行步长是否被钳得过小 |
| `STEP_TOO_SMALL = 3` | 步长低于机器可分辨 | 通常已接近驻点但梯度阈值过严 |
| `NON_FINITE = 4` | 能量/梯度出现 NaN/Inf | 单元反转、参数非法、初值越过定义域 |
| `LINEAR_SOLVE_FAILED = 5` | 稀疏分解失败 | Hessian 严重病态；开 `damping` 或换后端 |
| `EXTERNAL_SOLVER_FAILURE = 100` | 外部后端（PARDISO 等）内部错误 | 查 `raw_status_code` |
| `UNSUPPORTED_BACKEND = 101` | 请求的后端未编译进来 | 换 [`Auto()`](sparse_solver.md) |

---

## class `SolveDiagnostics`（冻结 dataclass）

最近一次求解的细粒度诊断。全部字段可为 `None`（后端未提供时）。

| 字段 | 含义 |
|---|---|
| `min_feasible_alpha` | 整个求解中最小的可行步长上限 $\alpha_{\text{feas}}$（接近 0 说明 CCD/反转防护在强烈钳制） |
| `min_line_search_alpha` | 最小线搜索步长 |
| `min_effective_alpha` | 最小实际步长 $\min(\alpha_{\text{ls}},\alpha_{\text{feas}})$ |
| `material_clamp_count` | 本构防护（如反转钳制）触发次数 |
| `contact_clamp_count` | 接触步长钳制次数 |
| `final_gradient_norm` / `final_gradient_max_norm` | 同 `SolverResult` 字段（冗余副本） |

### classmethod `from_dict(data)`

```python
SolveDiagnostics.from_dict(data: dict) -> SolveDiagnostics
```

从 C++ 的诊断 dict 构造，**只保留已识别的键**（用 `dataclasses.fields` 过滤）——C++ 侧可以自由增删诊断字段而不破坏 Python 构造（前向兼容设计）。

---

## class `SolverResult`（冻结 dataclass）

单次 [`Optimizer.solve`](base.md) 的结果。

| 字段 | 类型 | 含义 |
|---|---|---|
| `x` | `(n,) ndarray` | 最终迭代点 $\mathbf x^*$ |
| `status` | `SolveStatus` | 终止状态 |
| `converged` | `bool` | 是否收敛（含相对判据，比 `status==CONVERGED` 更宽容的兜底逻辑在 C++ 侧） |
| `iterations` | `int` | 实际 Newton 迭代数 |
| `raw_status_code` | `int` | 后端原始状态码（调试外部求解器用） |
| `final_objective` | `float \| None` | 最终能量 $E(\mathbf x^*)$ |
| `final_gradient_norm` | `float \| None` | $\|\nabla E\|_2$ |
| `final_gradient_max_norm` | `float \| None` | $\|\nabla E\|_\infty$（收敛判据用的范数） |
| `diagnostics` | `SolveDiagnostics` | 细粒度诊断 |

---

## func `_result_from_core(data)`（模块私有）

```python
_result_from_core(data: dict) -> SolverResult
```

C++ dict → `SolverResult` 的转换器：逐字段做类型规整（`x` 转 float64 ndarray、`status` 转枚举、诊断走 `SolveDiagnostics.from_dict`）。由 [`Optimizer.solve`](base.md) 调用，不属于公开 API。

## 用法示例

```python
res = opt.solve(problem, x0)
if not res.converged:
    print(f"未收敛: {res.status.name}, iters={res.iterations}, "
          f"‖g‖∞={res.final_gradient_max_norm:.2e}")
    d = res.diagnostics
    if d.min_effective_alpha is not None and d.min_effective_alpha < 1e-8:
        print("步长被钳到几乎为零 — 检查接触/反转")
u = res.x
```

## 交叉链接

- 生产方：[base.md](base.md) · [optimizer.md](optimizer.md)
- 动力学中逐帧结果的载体：[../sim/state.md](../sim/state.md)（`DynamicFrame.solver_result`）
