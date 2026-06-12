# `pypgo/fem/energy.py` — 形变能量工厂与参数导数面

> 源文件：`pypgo/fem/energy.py`（417 行，fem 包最大模块）。模块架构见 [overview.md](overview.md)。
>
> 本文件把网格 × formulation × 本构 × 参数场组装成 `DeformationEnergy`，并暴露对**材料参数**的完整导数面——这是材料优化与可微仿真（[torch.md](torch.md)）的基础。

## 共同数学框架

离散形变能量同时是位移与材料参数的函数：

$$E(\mathbf u;\,\mathbf b,\mathbf a)
=\sum_e\sum_q w_q\,\Psi\big(\mathbf F_e(\mathbf u;\xi_q),\ \mathbf b_e\big)\,\det\mathbf A(\mathbf a_e)\,\det J_q,
\qquad \mathbf F_e=\mathbf F\,\mathbf A(\mathbf a_e)^{-1}$$

- $\mathbf b$ = 弹性参数场、$\mathbf a$ = 塑性参数场（[fields.md](fields.md)）；
- 求积点 $(\xi_q,w_q)$ 与形函数来自 [formulation](formulations.md)，$\Psi$ 来自[本构](elastic.md)，$\mathbf A$ 来自[塑性模型](plastic.md)；
- 装配循环在 C++：`src/core/solidDeformationModel/deformation/deformationModelAssembler.cpp`（约 1100 行），按单元并行，每个求积点先做 SVD $\mathbf F_e=U\Sigma V^\top$ 再调本构三件套；工厂/管理在 `deformationModelManager.cpp`。

涉及的导数（C++ 装配器各有专用模板与 `compute_*` 例程）：

| 导数 | 含义 | Python API |
|---|---|---|
| $\partial E/\partial\mathbf u$ | 内力（负） | `gradient`（继承） |
| $\partial^2E/\partial\mathbf u^2$ | 切线刚度 | `hessian`（继承） |
| $\partial E/\partial\mathbf b$, $\partial E/\partial\mathbf a$ | 参数梯度 | `elastic_gradient` / `plastic_gradient` |
| $\partial^2E/\partial\mathbf b^2$ 等 | 参数 Hessian | `elastic_hessian` / `plastic_hessian` / `plastic_elastic_hessian` |
| $\partial^2E/\partial\mathbf u\,\partial\mathbf b$（即 $\partial\mathbf f/\partial\mathbf b$） | 混合导数 | `elastic_jacobian` / `plastic_jacobian` |

---

## class `DeformationEnergy`

[`PotentialEnergy`](../energy/base.md) 子类（`state_kind == "displacement"`），由 `deformation_energy()` 工厂创建、不直接构造。除继承的求值接口外，逐成员如下。

### 属性 `rest_position`

`(num_dofs,)` 静止位形 $\bar{\mathbf x}$（注意按 DOF 平铺；Hermite 时含导数 DOF 的静止值）。当前位形 $=\bar{\mathbf x}+\mathbf u$。

### 属性 `num_vertices`

几何顶点数；`num_dofs / num_vertices` 即每顶点 DOF 步长（线性 3、Hermite 24）。

### 属性 `num_elastic_params` / `num_plastic_params`

**每元素**参数通道数 $c$（体本构为 0、Koiter 为 5；塑性 = `dofs`）。

### 属性 `num_elastic_dofs` / `num_plastic_dofs`

全网格参数 DOF 总数（elementwise 时 $=n_e\cdot c$，constant 时 $=c$）。

### 属性 `elastic_model` / `plastic_model`

模型名字符串（`"stable_neo"` 等，见 [elastic.md](elastic.md) 的 `name`）。

### 属性 `elastic_field` / `plastic_field`

[`ParameterField`](fields.md) 只读视图。

### `set_elastic_values(values)` / `set_plastic_values(values)`

写入参数场。`values` 接受平铺或 `(num_value_rows, num_channels)`，形状按场校验（`_field_values_array`）。写入后能量/导数立即反映新参数；共享该场的对象（如 [`ShellDensityElasticThickness`](mass.md)）同步看到。

### `elastic_gradient(u)` / `plastic_gradient(u)`

```python
deform.elastic_gradient(u) -> (num_elastic_dofs,) ndarray
```

固定位移 $\mathbf u$ 下能量对参数的梯度 $\partial E/\partial\mathbf b$（或 $\partial E/\partial\mathbf a$）。

### `elastic_hessian(u)` / `plastic_hessian(u)` / `plastic_elastic_hessian(u)`

参数二阶导（`SparseMatrix`）：$\partial^2E/\partial\mathbf b^2$、$\partial^2E/\partial\mathbf a^2$、混合 $\partial^2E/\partial\mathbf a\,\partial\mathbf b$。

### `elastic_jacobian(u)` / `plastic_jacobian(u)`

```python
deform.elastic_jacobian(u) -> SparseMatrix (num_dofs × num_elastic_dofs)
```

**力-参数混合导数** $\dfrac{\partial^2E}{\partial\mathbf u\,\partial\mathbf b}=\dfrac{\partial(\nabla_{\mathbf u}E)}{\partial\mathbf b}$。绑定层在当前位形 $\bar{\mathbf x}+\mathbf u$ 处调装配器的 `compute_df_db` / `compute_df_da`（`src/python/pypgo/energy/core.cpp` 的 `PyDeformationEnergy::elasticJacobian/plasticJacobian`）。这是隐函数定理灵敏度 $\frac{d\mathbf u^*}{d\theta}=-H^{-1}\frac{\partial^2E}{\partial\mathbf u\,\partial\theta}$ 的右端块（[torch.md](torch.md) backward 的核心）。

### `element_von_mises(u)`

```python
deform.element_von_mises(u) -> (num_elements,) ndarray
```

逐元素 von Mises 应力。C++（`deformation/volume/volumetricDeformationModel.cpp:333-356`）在每个求积点算 Cauchy 应力再取标准 von Mises：

$$\boldsymbol\sigma=\frac{1}{\det\mathbf F_e}\,\mathbf P\,\mathbf F_e^\top,\qquad
\sigma_{vM}=\sqrt{\tfrac12\big[(\sigma_{11}-\sigma_{22})^2+(\sigma_{22}-\sigma_{33})^2+(\sigma_{33}-\sigma_{11})^2\big]+3\big(\sigma_{23}^2+\sigma_{31}^2+\sigma_{12}^2\big)}$$

应力后处理（[../animation/stress_stats.md](../animation/stress_stats.md)、[../animation/stress_vdb.md](../animation/stress_vdb.md)）消费此输出。

---

## class `DeformationOptions`（dataclass）

```python
DeformationOptions(enforce_spd: bool = True, enable_material_max_step: bool = True)
```

| 字段 | 含义 |
|---|---|
| `enforce_spd` | 逐元素 Hessian 的 SPD 投影（特征值截断；经本构的解析特征分解，如 StableNeo 的 twist/flip 特征系）。保证 Newton 方向是下降方向；做导数有限差分验证时应关掉（投影破坏精确二阶一致性） |
| `enable_material_max_step` | 本构层的最大步长限制（多项式求根防单元反转，`deformation/materialMaxStepPolynomialUtils.cpp`），并入 [`max_step`](../energy/base.md) |

---

## func `deformation_energy(...)`

```python
deformation_energy(sim_mesh, *,
    elastic,          # ElasticModel（或带 name/_to_string 的对象）
    elastic_field,    # ElementwiseField / ConstantField
    plastic,          # PlasticModel
    plastic_field,    # ElementwiseField / ConstantField
    formulation=None, # 必填：TetLinear()/CubicLinear()/CubicTricubicHermite()/KoiterShell()
    options=None,     # DeformationOptions
) -> DeformationEnergy
```

主工厂。流程（`energy.py:302-356`）：

1. 校验 `sim_mesh` / `formulation` / 模型类型；
2. 解析参数场初值：`values=None` → 让 C++ 按网格材料 payload 播种；显式值按"每元素通道数"（弹性经 `_elastic_num_channels` 询问 C++ 的 `ElasticModelFactory::parameterSpec`）校验形状；
3. 以**字符串名**派发（`elastic.name`、`plastic.name`、`formulation.name`）调 `_core._create_deformation_energy`（绑定实现 `src/python/pypgo/energy/core.cpp` 的 `createDeformationEnergy`，内部建 `DeformationModelManager` + 装配器）。

formulation 与网格类型必须匹配（tet formulation 配 tet 网格等），不匹配在 C++ 侧报错。

## func `elastic_material_energy(deformation_energy, *, fixed_displacement)`

```python
elastic_material_energy(deform, fixed_displacement=u_star) -> ElasticMaterialEnergy
```

视角切换：**固定位移 $\mathbf u^\*$，把弹性参数 $\mathbf b$ 当作优化变量**的能量 $\tilde E(\mathbf b)=E(\mathbf u^*;\mathbf b)$。返回的对象是普通 [`PotentialEnergy`](../energy/base.md)（`num_dofs == num_elastic_dofs`），可以直接进 [`OptimizationProblem`](../solver/problem.md) 做材料反演。`fixed_displacement` 须为 `(num_dofs,)`。

## func `plastic_material_energy(deformation_energy, *, fixed_displacement)`

同上，但变量是塑性参数 $\mathbf a$。

---

## class `ElasticMaterialEnergy` / class `PlasticMaterialEnergy`

上述工厂的返回类型。除继承的求值接口外：

### 属性 `deformation_energy`

回到原 `DeformationEnergy` 的引用。

### 属性 `fixed_displacement`

构造时冻结的位移拷贝（返回副本）。

---

## 用法示例

```python
import numpy as np
import pypgo

deform = pypgo.fem.deformation_energy(
    sim_mesh,
    formulation=pypgo.fem.TetLinear(),
    elastic=pypgo.fem.StableNeo(),
    elastic_field=pypgo.fem.ElementwiseField(),
    plastic=pypgo.fem.VolumetricPlasticity(dofs=6),
    plastic_field=pypgo.fem.ElementwiseField(),
    options=pypgo.fem.DeformationOptions(enforce_spd=True),
)

u = deform.zero_state()
deform.value(u)                      # 0.0（静止态）
K = deform.hessian(u)                # 切线刚度（静止线性化）
J_a = deform.plastic_jacobian(u)     # ∂f/∂a — 灵敏度分析

# 材料反演：固定平衡位移，优化塑性场
mat_e = pypgo.fem.plastic_material_energy(deform, fixed_displacement=u_star)
problem = pypgo.solver.OptimizationProblem(objective=mat_e)
```

## 数学自检

- 静止态：$\mathbf u=\mathbf 0$、$\mathbf A=\mathbf I$ ⟹ $\mathbf F_e=\mathbf I$ ⟹ $E=0$、$\nabla E=\mathbf 0$（所有本构在 $\mathbf I$ 处能量/应力为零，见 [elastic.md](elastic.md) 各模型自检）✓
- 维度：`elastic_jacobian` 是 $n\times n_b$（DOF × 参数 DOF），不是方阵 ✓
- `enforce_spd=True` 时 `hessian` 的最小特征值 $\ge 0$（投影后）；做 FD 校验请用 `enforce_spd=False` ✓

## 交叉链接

- 输入要素：[mesh.md](mesh.md) · [formulations.md](formulations.md) · [elastic.md](elastic.md) · [plastic.md](plastic.md) · [fields.md](fields.md)
- 求值契约：[../energy/base.md](../energy/base.md)；组合：[../energy/sets.md](../energy/sets.md)
- 可微消费方：[torch.md](torch.md)
