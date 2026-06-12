# `pypgo/constraints/bounded.py` — `Bounded` 区间界封装

> 源文件：`pypgo/constraints/bounded.py`（39 行，纯 Python dataclass）。模块架构见 [overview.md](overview.md)。
>
> 给约束函数附加逐元素上下界，表达不等式约束 $\ell \le C(x) \le u$。本身不做任何计算——它是约束 + 界的**数据载体**，唯一的消费方是 [`ConstraintViolationPenalty`](../energy/penalty.md)（违界罚能量）。

---

## class `Bounded`（冻结 dataclass）

$$\ell \;\le\; C(x) \;\le\; u,\qquad \ell, u\in\mathbb R^m$$

```python
@dataclass(frozen=True)
Bounded(functions, *, lower, upper)
```

| 字段 | 说明 |
|---|---|
| `functions` | [`ConstraintFunction`](base.md)——约束本体 $C$；类型错误抛 `TypeError` |
| `lower` | $\ell$：标量（`np.full` 广播到全部 $m$ 个约束）或 `(m,)` 数组；keyword-only |
| `upper` | $u$：同上；keyword-only |

构造行为（`__post_init__`，借助私有 `_bounds_vector`）：

- `lower`/`upper` 规整为 `(m,)` float64 **拷贝**（$m$ = `functions.num_constraints`），标量自动广播；
- 形状既非标量也非 `(m,)` 时抛 `ValueError`；
- 冻结 dataclass，构造后不可变（界数组用 `object.__setattr__` 写入）。

> **不校验 $\ell\le u$**——颠倒的界会使违界罚处处为正，由调用方保证。

### 表达能力

| 需求 | 取值 |
|---|---|
| 等式 $C_k(x)=0$ | $\ell_k=u_k=0$ |
| 单边 $C_k(x)\ge 0$ | $\ell_k=0,\ u_k=+\infty$ |
| 单边 $C_k(x)\le b$ | $\ell_k=-\infty,\ u_k=b$ |
| 区间 | 任意 $\ell_k\le u_k$ |

## 用法示例

```python
import numpy as np
import pypgo

# 0 ≤ x0 - x1 ≤ 2
gap = pypgo.constraints.Linear(np.array([[1.0, -1.0]]))
b = pypgo.constraints.Bounded(gap, lower=0.0, upper=2.0)

# 单边约束：lower=0, upper=∞
b2 = pypgo.constraints.Bounded(some_constraint, lower=0.0, upper=np.inf)

soft = pypgo.energy.ConstraintViolationPenalty(b, weight=1e4)
```

## 交叉链接

- 约束本体：[base.md](base.md) · [functions.md](functions.md)
- 消费方：[../energy/penalty.md](../energy/penalty.md)（`ConstraintViolationPenalty`）
- 注意区分：逐 DOF 的盒式界（固定顶点）请直接用 [../solver/problem.md](../solver/problem.md) 的 `variable_bounds`/`fix_variables`，不需要经过本类
