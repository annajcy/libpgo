# `pypgo/energy/attachment.py` — 顶点软固定能量

> 源文件：`pypgo/energy/attachment.py`（178 行）。模块架构见 [overview.md](overview.md)。
>
> 以罚能量形式实现的"软"Dirichlet 边界条件：把选中顶点拉向目标位置（`VertexAttachment`），或把**嵌入曲面点**钉在静止位置（`EmbeddedVertexAttachment`）。相比直接固定 DOF（[../solver/problem.md](../solver/problem.md) 的 `fix_variables`），软固定保留全部 DOF、刚度可调，且对带导数 DOF 的高阶 formulation（tricubic Hermite）是唯一正确的固定方式。

---

## class `VertexAttachment`

仿真网格顶点的软固定罚。C++ 实现：`MultipleVertexPulling`（`src/core/constraintPotentialEnergies/multiVertexPullingSoftConstraints.cpp`）。

设顶点 $i$ 的静止位置为 $\bar{\mathbf x}_i$、位移为 $\mathbf u_i$、目标为 $\mathbf t_i$。**as-implemented**（`multiVertexPullingSoftConstraints.cpp:55-79`）：

$$E(\mathbf u) \;=\; \frac{c}{2}\sum_{i\in S}\big\|\,\mathbf m_i\odot(\mathbf u_i+\bar{\mathbf x}_i-\mathbf t_i)\,\big\|^2$$

其中 $c$ = `coeff`，$\mathbf m_i$ 是逐分量掩码（Python 层恒为 $\mathbf 1$），`is_displacement=False` 时去掉 $\bar{\mathbf x}_i$（状态直接是位置）。导数：

$$\nabla_{\mathbf u_i}E = c\,(\mathbf u_i+\bar{\mathbf x}_i-\mathbf t_i),\qquad
\nabla^2 E\big|_{ii} = c\,\mathbf I_3\ \text{(仅对角)}$$

> **注意系数约定**：Python docstring 写作 `coef * ||u_i - target_i||^2`，但 C++ 实际带 $\tfrac12$ 因子——能量是 $\frac{c}{2}\|\cdot\|^2$、Hessian 对角为 $c$（`multiVertexPullingSoftConstraints.cpp:69,122`）。本文档以 C++ 实现为准。
>
> 另外，Python 走 `sim_mesh` 路径时把 `rest_positions` 传为全零（`attachment.py:57`），因此即使 `is_displacement=True`，**实际生效的形式是 $\frac c2\sum\|\mathbf u_i-\mathbf t_i\|^2$**（目标按位移解释）；只有 C++ 直接构造并给出真实 rest 位置时 $\bar{\mathbf x}_i$ 才参与。

```python
VertexAttachment(*,
    sim_mesh=None,
    koff=None,
    vertex_indices,
    target_positions,
    coeff=1e6,
    is_displacement=True)
```

| 参数 | 说明 |
|---|---|
| `sim_mesh` | `SimulationMesh`：提供 `num_vertices * 3` 个 DOF 与**单位对角**的 Hessian 稀疏模板；rest 位置取零（见上） |
| `koff` | 后备入口：自带 Hessian 稀疏模板（任何 [`as_coo`](../sparse.md) 接受的形式），DOF 数取其行数。`sim_mesh` 与 `koff` 必须给其一 |
| `vertex_indices` | `(m,)` int — 要约束的顶点索引 |
| `target_positions` | `(3m,)` float — 平铺目标 `(x0,y0,z0, x1,...)`；长度必须恰为 `3*len(vertex_indices)`，否则抛 `ValueError` |
| `coeff` | 罚刚度 $c$，默认 `1e6` |
| `is_displacement` | 状态是位移（默认）还是位置 |

绑定工厂：`_core._create_vertex_attachment`。

### `set_targets(target_positions)`

```python
va.set_targets(target_positions) -> None
```

更新目标 $\mathbf t \leftarrow \mathbf t'$（C++ `MultipleVertexPulling::setTargetPositions`），长度必须与构造时一致。这是本类唯一的可变操作，用于逐帧拖拽目标/脚本化运动，无需重建能量。

---

## class `EmbeddedVertexAttachment`

**嵌入**顶点的软固定（钉在静止位置）。设 $W\in\mathbb R^{3m\times n}$ 是仿真 DOF → 嵌入顶点位移的插值矩阵（如 formulation 的曲面嵌入矩阵，见 [../fem/formulations.md](../fem/formulations.md)），$W_s$ 是被选嵌入顶点对应的行块。能量把这些**物理点**钉在位移为零处：

$$E(\mathbf u) \;=\; c\,\|W_s\,\mathbf u\|^2
\;=\;\tfrac12\,\mathbf u^\top\underbrace{\big(2c\,W_s^\top W_s\big)}_{A}\,\mathbf u$$

实现上是**纯 Python 组装**（`attachment.py:119-173`）：从 $W$ 的 COO 中筛出选中行、逐行做外积 $2c\,\mathbf w_r\mathbf w_r^\top$ 累加合并成稀疏 $A$，然后构造 [`QuadraticEnergy(A)`](algebraic.md) 并接管其 `_handle`——没有独立的 C++ 类。

为什么要它：对 tricubic Hermite 等带导数 DOF 的 formulation，硬性 clamp 节点 DOF 会错误地约束导数自由度；约束作用在插值后的物理点上则与 formulation 无关，导数 DOF 只通过插值参与。

> **系数约定差异**：`EmbeddedVertexAttachment` 是 $c\|\cdot\|^2$（不带 ½），`VertexAttachment` 是 $\frac c2\|\cdot\|^2$——尽管 docstring 声称两者约定一致，**同一 `coeff` 数值下前者刚度是后者的 2 倍**。迁移/对比实验时注意。

```python
EmbeddedVertexAttachment(*,
    embedding=None,
    vertex_indices,
    coeff=1e5,
    num_dofs=None)
```

| 参数 | 说明 |
|---|---|
| `embedding` | `SparseMatrix`（$3m\times n$）：DOF → 嵌入点位移；`None` 表示恒等映射（嵌入顶点**就是**仿真顶点，每点 3 DOF），此时 $A$ 退化为选中 DOF 上的对角 $2c$ |
| `vertex_indices` | `(k,)` int — 嵌入顶点集合中的索引，**自动去重排序**（`np.unique`）；空集抛 `ValueError` |
| `coeff` | 罚刚度 $c$，默认 `1e5` |
| `num_dofs` | `embedding=None` 时必填（仿真 DOF 数 $n$）；否则从 `embedding` 列数推断，给了且不一致抛 `ValueError` |

校验：索引越界（`embedding` 行数 / `num_dofs`）抛 `ValueError`。`repr` 额外报告嵌入顶点数（`_num_embedded`）。

## 数学自检

- 目标即当前位置时能量为零：`VertexAttachment` 中 $\mathbf u_i = \mathbf t_i$（位移解释）→ $E=0$ ✓；`EmbeddedVertexAttachment` 中 $\mathbf u=\mathbf 0$ → $E=0$ ✓
- Hessian 常量、半正定（$W_s^\top W_s\succeq 0$；对角 $c>0$）✓
- 维度：$W_s\in\mathbb R^{3k\times n}$、$A\in\mathbb R^{n\times n}$，能量定义在全部 $n$ 个仿真 DOF 上 ✓

## 用法示例

```python
import numpy as np
import pypgo

# 1) 经典软固定：把底面顶点钉在原位
pin = pypgo.energy.VertexAttachment(
    sim_mesh=sim_mesh,
    vertex_indices=bottom_ids,
    target_positions=np.zeros(bottom_ids.size * 3),
    coeff=1e6)
pin.set_targets(new_targets)   # 逐帧更新目标

# 2) Hermite 网格：钉嵌入曲面点而非节点 DOF
#    嵌入矩阵来自 formulation（见 fem/formulations.md）
W = formulation.surface_embedding_matrix(volume_mesh, surface_vertices)  # 3m × n
pin2 = pypgo.energy.EmbeddedVertexAttachment(
    embedding=W, vertex_indices=surface_fixed_ids, coeff=1e5)
```

## 交叉链接

- 归约目标：[algebraic.md](algebraic.md)（`QuadraticEnergy`）
- 硬固定替代方案：[../solver/problem.md](../solver/problem.md)（`fix_variables`，lower==upper）
- 嵌入矩阵来源：[../fem/formulations.md](../fem/formulations.md)（`surface_embedding_matrix`）、[../mesh/geometry/core.md](../mesh/geometry/core.md)
