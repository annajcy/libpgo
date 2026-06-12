# `pypgo/energy/penalty.py` — 约束派生的罚能量

> 源文件：`pypgo/energy/penalty.py`（50 行，薄封装）。模块架构见 [overview.md](overview.md)。
>
> 把硬约束（[../constraints/overview.md](../constraints/overview.md)）软化为势能项的桥梁：约束 $C(x)$ 不再被精确强制，而是作为二次罚加入总能量，由权重 $w$ 控制"硬度"。这是 pypgo 中连接 `constraints` 包与 `energy` 栈的唯一通道。

## 共同实现机制：权重的包装方式

两个类的 C++ 内核都**不带权重**（纯 $\tfrac12\|\cdot\|^2$，`src/core/nonlinearOptimization/constraints/potentialEnergyFromConstraintFunctions.cpp:54,170`）；绑定层把内核包进**单项 `EnergySet`**，把 `weight` 作为该项的组合权重（`src/python/pypgo/energy/core.cpp` 的 `createConstraintPenalty`）。语义上等价于 $E = w\cdot\tfrac12\|\cdot\|^2$。

共同点：两者都不可变（继承 [`PotentialEnergy`](base.md)）；`weight` 与原约束对象以只读属性保留。**改权重须重建对象**（或自己把内核能量放进 [`EnergySet`](sets.md) 用 `set_weight` 调）。

---

## class `ConstraintPenalty`

零残差罚：目标 $C(x)=\mathbf 0$ 的软化。**as-implemented**（C++ 类 `PotentialEnergyConstraintFunctions`，内核见 `potentialEnergyFromConstraintFunctions.cpp:54`）：

$$E(x) = \frac{w}{2}\,\|C(x)\|^2$$

导数（链式法则，$J = \partial C/\partial x$）：

$$\nabla E = w\,J^\top C(x),\qquad
\nabla^2 E = w\Big(J^\top J + \sum_k C_k(x)\,\nabla^2 C_k(x)\Big)$$

第二项即约束 Hessian 的乘子加权和（multipliers $=C(x)$，见 [../constraints/base.md](../constraints/base.md) 的 `hessian(x, multipliers)`）；线性约束时该项为零，能量整体退化为二次型（Gauss–Newton 与精确 Newton 一致）。

```python
ConstraintPenalty(constraints, weight=1.0)
```

| 参数 | 说明 |
|---|---|
| `constraints` | [`pypgo.constraints.ConstraintFunction`](../constraints/base.md)，类型不符抛 `TypeError` |
| `weight` | 罚权重 $w$，转 `float` 后以只读属性 `e.weight` 保留 |

绑定工厂：`_core._create_constraint_penalty`。

### 属性 `constraints`

```python
e.constraints -> ConstraintFunction
```

构造时传入的约束对象（原样保留，便于事后查看 $C$、做诊断求值）。

---

## class `ConstraintViolationPenalty`

违界罚：目标 $\ell \le C(x)\le u$ 的软化。先计算**单边残差**（as-implemented，C++ 类 `PotentialEnergyBoundedConstraintFunctions`，`potentialEnergyFromConstraintFunctions.cpp:149-171`）：

$$r_k(x) = \begin{cases}
C_k(x) - \ell_k, & C_k(x) < \ell_k\\
C_k(x) - u_k, & C_k(x) > u_k\\
0, & \text{界内}
\end{cases}
\qquad
E(x) = \frac{w}{2}\,\|r(x)\|^2$$

界内能量恒为零、一阶光滑（$C^1$，残差在边界处连续地从 0 出发），但二阶导在边界处跳变——active set 改变时 Hessian 不连续，Newton 在边界附近可能需要更多迭代。

```python
ConstraintViolationPenalty(bounded, weight=1.0)
```

| 参数 | 说明 |
|---|---|
| `bounded` | [`pypgo.constraints.Bounded`](../constraints/bounded.md)（约束 + `lower`/`upper`），类型不符抛 `TypeError`。其 `functions._handle` 与界数组被拆开传给 C++ 工厂 |
| `weight` | 罚权重 $w$，只读属性 `e.weight` |

绑定工厂：`_core._create_constraint_violation_penalty`。

### 属性 `bounded`

```python
e.bounded -> Bounded
```

构造时传入的 `Bounded` 对象（含约束本体与上下界）。

## 数学自检

- 约束满足 / 界内时 $E=0$、$\nabla E=\mathbf 0$ ✓
- $E\ge 0$ 恒成立（平方和）✓
- 线性约束时 Hessian $= w\,J^\top J \succeq 0$（半正定）✓

## 用法示例

```python
import numpy as np
import pypgo

# 约束：x0 + x1 = 1（线性，C(x) = Ax - 1）
A = np.array([[1.0, 1.0]])
C = pypgo.constraints.Linear(A, offset=[-1.0])

soft = pypgo.energy.ConstraintPenalty(C, weight=1e4)
soft.value(np.array([0.5, 0.5]))   # 0.0（约束满足）
soft.value(np.array([1.0, 1.0]))   # ½·1e4·(2-1)² = 5000.0

# 区间约束：0 ≤ x0 - x1 ≤ 2
B = pypgo.constraints.Bounded(
    pypgo.constraints.Linear(np.array([[1.0, -1.0]])), lower=0.0, upper=2.0)
soft2 = pypgo.energy.ConstraintViolationPenalty(B, weight=1e3)
```

## 交叉链接

- 约束的定义：[../constraints/base.md](../constraints/base.md) · [../constraints/functions.md](../constraints/functions.md) · [../constraints/bounded.md](../constraints/bounded.md)
- 权重包装的载体：[sets.md](sets.md)（`EnergySet`）
- 组合进总能量：[sets.md](sets.md)
