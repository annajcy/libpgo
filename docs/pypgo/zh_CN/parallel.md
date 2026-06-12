# `pypgo/parallel.py` — 引擎并行度控制

> 源文件：`pypgo/parallel.py`（63 行，`_core` 并行控制的薄封装）。所属架构见 [overview.md](overview.md)。
>
> 控制 libpgo C++ 引擎内部并行循环（`pgo::parallel`，基于 TBB；C++ 侧见 `src/core/parallelism/`）的默认工作线程数。FEM 装配、接触检测等热点循环都受此开关影响。
>
> **作用域限定**：只管 libpgo 自己的并行循环，**不承诺**控制第三方线程池（BLAS、Eigen、libigl、OpenVDB）——模块 docstring 言明。

公开符号只有三个（`__all__ = ["get_num_threads", "set_num_threads", "thread_limit"]`）。所有入口共享同一校验（私有 `_normalize_num_threads`）：`num_threads` 必须是**正整数或 `None`**，`0`、负数抛 `ValueError`。

---

## func `set_num_threads(num_threads)`

```python
set_num_threads(num_threads: int | None) -> None
```

| 参数 | 说明 |
|---|---|
| `num_threads` | 正整数：设为默认工作线程上限；`None`：恢复后端自动值 |

C++ 入口：正整数走 `_core._parallel_set_num_threads`，`None` 走 `_core._parallel_reset_num_threads`。是全局、进程级的设置。

## func `get_num_threads()`

```python
get_num_threads() -> int | None
```

返回当前 libpgo 默认线程上限；`None` 表示自动（未显式限制）。C++ 入口：`_core._parallel_get_num_threads`。

## func `thread_limit(num_threads)`

```python
thread_limit(num_threads: int | None) -> 上下文管理器
```

临时覆盖线程上限的上下文管理器（内部实现类 `_ThreadLimit`）：

- `__enter__`：先 `get_num_threads()` 记住旧值，再 `set_num_threads(num_threads)`；
- `__exit__`：恢复旧值（**包括 `None`**），不吞异常（返回 `False`）。

恢复语义是"恢复进入前的值"，嵌套使用安全。参数校验发生在构造时（进入 `with` 之前）。

## 用法示例

```python
import pypgo.parallel as pp

pp.set_num_threads(8)          # 全局：限 8 线程
pp.get_num_threads()           # 8

with pp.thread_limit(1):       # 临时单线程（如复现数值、做基准）
    result = optimizer.solve(problem, x0)
# 离开 with 后恢复为 8

pp.set_num_threads(None)       # 恢复自动
```

> 单线程对调试非确定性（浮点求和顺序导致的微小差异）很有用：TBB 并行归约的结果在线程数不同时可能在最后几位上不同。

## 交叉链接

- 受影响的热点：[fem/energy.md](fem/energy.md)（装配）、[contact/energies.md](contact/energies.md)（接触检测）
- 求解器整体流程：[solver/overview.md](solver/overview.md)
