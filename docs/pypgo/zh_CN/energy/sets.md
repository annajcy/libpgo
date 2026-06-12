# `pypgo/energy/sets.py` — `EnergySet` 能量组合

> 源文件：`pypgo/energy/sets.py`（41 行，薄封装）。模块架构见 [overview.md](overview.md)。
>
> 变分力学的"总能量"构造器：把形变、接触、软固定等各物理项线性组合成单一目标函数，交给求解器。这是每个仿真脚本必经的汇聚点。

## 共同数学框架

$$E_{\text{total}}(\mathbf x) = \sum_{i} w_i\,E_i(\mathbf x)$$

线性性使导数直接相加：

$$\nabla E_{\text{total}} = \sum_i w_i\,\nabla E_i,\qquad
\nabla^2 E_{\text{total}} = \sum_i w_i\,\nabla^2 E_i$$

各项的 DOF 集合可以不同（每个 $E_i$ 自带 `dofs` 索引，见 [base.md](base.md)）；C++ 侧按全局 DOF 对齐并合并稀疏拓扑。Hessian 的合并稀疏模板在构造时确定一次，之后每次求值原位累加。

---

## class `EnergySet`

加权能量和。自身就是 [`PotentialEnergy`](base.md)，完整继承求值接口（`value`/`gradient`/`hessian`/`max_step`/`dofs`/`state_kind`/`zero_state`），所以**可以嵌套**——`EnergySet` 可以作为另一个 `EnergySet` 的项。

继承接口中值得单独说明的是 `max_step`：取各项可行步长的最小值

$$\alpha_{\max} = \min_i \alpha_{\max}^{(i)}$$

——这保证含 IPC 项的总能量仍然继承无穿透步长限制。

C++ 实现：`NonlinearOptimization::EnergySet`，经 `_core._create_energy_set`（绑定见 `src/python/pypgo/energy/bindings.cpp`）。

```python
EnergySet(terms)
```

| 参数 | 说明 |
|---|---|
| `terms` | `list[(PotentialEnergy, float)]` ——（能量, 权重）二元组列表。每个能量必须是 `PotentialEnergy`（含 fem/contact 领域能量与嵌套 `EnergySet`），否则抛带项号的 `TypeError`；权重转 `float` |

### 属性 `num_terms`

```python
es.num_terms -> int
```

组合的项数（C++ `num_terms`）。

### `set_weight(i, w)`

```python
es.set_weight(i: int, w: float) -> None
```

把第 $i$ 项的权重改为 $w$（$w_i \leftarrow w$，C++ `set_weight`，**无需重建稀疏拓扑**）。这是本类唯一的"可变"操作——修改的是 C++ 侧权重数组，不破坏 Python 层不可变约定的拓扑部分。用它做罚系数 continuation（逐步加大 `coeff`）比重建整个能量栈高效得多。

## 数学自检

- 单项 `EnergySet([(e, 1.0)])` 与 `e` 求值逐点一致 ✓
- $w_i=0$ 时该项对能量/梯度/Hessian 无贡献 ✓
- 线性性：`value` 对权重是线性函数 ✓

## 用法示例

```python
import pypgo

total = pypgo.energy.EnergySet([
    (deformation_energy, 1.0),   # FEM 弹性
    (pin_energy,         1.0),   # 软固定
    (ipc_energy,         1.0),   # 接触
])
total.num_terms        # 3
total.set_weight(2, 0.5)   # 临时降低接触权重

problem = pypgo.solver.OptimizationProblem(objective=total)
```

## 交叉链接

- 被组合的能量：[algebraic.md](algebraic.md) · [attachment.md](attachment.md) · [penalty.md](penalty.md) · [../fem/energy.md](../fem/energy.md) · [../contact/energies.md](../contact/energies.md)
- 消费方：[../solver/problem.md](../solver/problem.md)
