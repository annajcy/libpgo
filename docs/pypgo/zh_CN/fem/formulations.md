# `pypgo/fem/formulations.py` — FEM Formulation 体系

> 源文件：`pypgo/fem/formulations.py`（169 行，镜像 C++ `Formulation` 类层次的薄封装）。模块架构见 [overview.md](overview.md)。
>
> C++ 实现：`src/core/solidDeformationModel/formulations/`（formulation 与求积规则）；绑定 `src/python/pypgo/fem/formulation/`。

## 共同数学框架

Formulation 回答离散化的三个问题：**形函数空间**（位移场如何从节点 DOF 插值）、**DOF 布局**（每顶点几个自由度）、**求积规则**（能量积分在哪些点采样）。装配器对 formulation 完全泛型，只经虚接口取形函数导数与求积点。

参考单元 $\xi\in\hat\Omega$ 上：

$$\mathbf u(\xi) = \sum_{a} N_a(\xi)\,\mathbf u_a,\qquad
\mathbf F(\xi) = \mathbf I + \sum_a \mathbf u_a\,\nabla_X N_a(\xi)^\top,\qquad
E_e \approx \sum_q w_q\,\Psi\big(\mathbf F(\xi_q)\big)\,\det J(\xi_q)$$

---

## class `Formulation`

抽象基类，持有 C++ `PyFormulation` peer（`_handle`）。

### 属性 `name`

```python
fm.name -> str
```

C++ 侧标识（`"tet_linear"` / `"cubic_linear"` / `"cubic_tricubic_hermite"` 等），是 [`deformation_energy`](energy.md) 工厂派发到 C++ 的键。

---

## class `VolumetricFormulation`

体 formulation 的中间基类，附带三个动力学/几何算子（全部委托 `_core.compute_formulation_*`）。

### `mass_matrix(sim_mesh, mass_field)`

```python
fm.mass_matrix(sim_mesh, mass_field) -> SparseMatrix   # n × n
```

一致质量矩阵

$$M=\int_\Omega \rho\,N^\top N\,dV$$

用 formulation 的质量求积规则装配（`TetLinear` 用 2 次精度的 `TetDegree2Quadrature`——质量被积函数 $N^\top N$ 是二次的，单点不够）。`mass_field` 必须是 [`VolumeMassField`](mass.md)（kg/m³），类型不符抛 `TypeError`。消费方：[动力学的惯性项](../sim/overview.md)。

### `body_force(sim_mesh, acceleration, mass_field)`

```python
fm.body_force(sim_mesh, acceleration, mass_field) -> ndarray (n,)
```

常加速度 $\mathbf g$（3-向量）的广义体力

$$\mathbf f_g=\int_\Omega \rho\,N^\top\mathbf g\,dV$$

重力势能 $=-\mathbf f_g^\top\mathbf u$：用 [`LinearEnergy(-f_g)`](../energy/algebraic.md) 加入能量栈（`pypgo/tools/sim/_scene.py` 即此用法）。

### `surface_embedding_matrix(volume, surface_vertices)`

```python
fm.surface_embedding_matrix(volume, surface_vertices) -> SparseMatrix   # 3m × n
```

仿真 DOF → 表面点位移的插值矩阵 $W$：对每个表面点找到包含单元、求形函数值，$\delta\mathbf x_s=W\,\mathbf u$。`volume` 是 [`VolumeMesh`](../mesh/volume/core.md)，`surface_vertices` 形状 `(m, 3)`。

对 Hermite formulation，导数 DOF 经插值自然参与——这是 [`EmbeddedVertexAttachment`](../energy/attachment.md) 与接触嵌入（[../contact/surface.md](../contact/surface.md)）formulation 无关性的来源（Hermite 实现见 `cubicTricubicHermiteFormulation.cpp` 的 `buildSurfaceEmbeddingMatrix`）。

---

## class `ShellFormulation`

壳 formulation 的中间基类。

### `mass_matrix(sim_mesh, mass_field)`

集中（lumped）壳质量矩阵；`mass_field` 必须是 [`ShellMassField`](mass.md)（kg/m²）。

### `body_force(sim_mesh, acceleration, mass_field)`

集中壳体力（同体的语义，密度换为面密度 $\rho h$）。

### `body_force_parameter_jacobian(sim_mesh, acceleration, mass_field)`

```python
fm.body_force_parameter_jacobian(sim_mesh, acceleration, mass_field) -> SparseMatrix
```

$\partial\mathbf f_g/\partial\mathbf b$：质量场依赖弹性参数（如 [`ShellDensityElasticThickness`](mass.md) 的厚度通道）时自重对参数的导数。供 [`SelfWeightGravity.parameter_jacobian`](mass.md) → [可微层](torch.md) 的混合导数修正。

---

## class `TetLinear`

```python
TetLinear()
```

4 节点四面体、线性形函数、12 局部 DOF（`tetLinearFormulation.cpp:19-21`）。线性形函数 ⟹ $\mathbf F$ 逐元素**常量**（常应变单元），能量求积单点即精确：重心 $\xi=(\tfrac14,\tfrac14,\tfrac14)$、权 $w=\tfrac16$（= 参考四面体体积；`tetLinearDefaultQuadrature.cpp:8-18`）。

## class `CubicLinear`

```python
CubicLinear()
```

8 节点六面体、三线性形函数、24 局部 DOF（`cubicLinearFormulation.cpp:18-20`）。$\mathbf F$ 在单元内变化，能量求积 2×2×2 Gauss–Legendre（8 点、参考权 $\tfrac18$；`gaussLegendreHexQuadrature.h:14-22`）。配体素化网格（[../mesh/processing/volume.md](../mesh/processing/volume.md) 的 `cubic_mesher`）。

## class `CubicTricubicHermite`

```python
CubicTricubicHermite()
```

规则网格六面体上的**三三次 Hermite** 插值（`cubicTricubicHermiteFormulation.cpp:133-135`）：

- 每顶点 **24 DOF** = 8 个 Hermite 量（$f,\ \partial_x,\ \partial_y,\ \partial_z,\ \partial_{xy},\ \partial_{xz},\ \partial_{yz},\ \partial_{xyz}$）× 3 个位移分量；8 顶点 ⟹ 64 节点、192 局部 DOF；
- 位移场跨单元 $C^1$ 光滑；
- $\mathbf F$ 是高阶多项式，2×2×2 会欠积分非线性材料，故用 **4×4×4** Gauss–Legendre（64 点、每轴 7 次精确；`gaussLegendreHexQuadrature.h:25-36` 注释言明原因）。

**边界条件注意**：直接 clamp 节点 DOF 会同时锁死导数自由度（过约束）；正确做法是经 `surface_embedding_matrix` 用 [`EmbeddedVertexAttachment`](../energy/attachment.md) 钉物理点。

## class `KoiterShell`

```python
KoiterShell()
```

三角面网格上的 Koiter 壳 formulation（`shellFormulation/koiterShellFormulation.cpp`），按单元算第一/第二基本形式，交给 [`KoiterStVK`](elastic.md) 本构。每顶点 3 DOF。

---

## 用法示例

```python
import pypgo

fm = pypgo.fem.CubicTricubicHermite()
fm.name                                   # 'cubic_tricubic_hermite'
W = fm.surface_embedding_matrix(volume, surface_pts)    # 3m × n
M = fm.mass_matrix(sim_mesh, pypgo.fem.volume_density(volume))
f_g = fm.body_force(sim_mesh, [0, -9.8, 0], pypgo.fem.volume_density(volume))
```

## 数学自检

- 局部 DOF = 节点数 × 3：$4\times3=12$、$8\times3=24$、$64\times3=192$ ✓
- tet 单点权重 $\tfrac16$ = 参考四面体体积；hex 8 点权重和 $8\times\tfrac18=1$ = 单位六面体体积 ✓
- 形函数单位分解 $\sum_aN_a\equiv1$ ⟹ 刚体平移零能量

## 交叉链接

- 消费方：[energy.md](energy.md)（`deformation_energy(formulation=...)`）
- 质量场类型契约：[mass.md](mass.md)；嵌入矩阵用途：[../energy/attachment.md](../energy/attachment.md)、[../contact/surface.md](../contact/surface.md)
