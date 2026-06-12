# `pypgo/fem/elastic.py` — 弹性本构模型

> 源文件：`pypgo/fem/elastic.py`（68 行）。模块架构见 [overview.md](overview.md)。
>
> Python 层每个类只是一个 `_core.make_*()` 工厂调用 + 句柄持有；**全部数学在 C++**（`src/core/solidDeformationModel/material/elastic/`）。本篇逐类讲解每个本构的能量密度 $\Psi$ 及其导数的 as-implemented 形式（已逐一与 C++ 源核对）。

## 共同数学框架

体弹性模型实现统一的 C++ 虚接口 `ElasticModel`（`material/elastic/elasticModel.h`），围绕形变梯度 $\mathbf F\in\mathbb R^{3\times3}$ 的三件套：

| 量 | 公式 | C++ 虚函数 |
|---|---|---|
| 能量密度 | $\Psi(\mathbf F)$，单位 J/m³ | `compute_psi` |
| 第一类 Piola–Kirchhoff 应力 | $\mathbf P=\dfrac{\partial\Psi}{\partial\mathbf F}\in\mathbb R^{3\times3}$ | `compute_P` |
| 应力切线 | $\dfrac{\partial\mathbf P}{\partial\mathbf F}\in\mathbb R^{9\times9}$ | `compute_dPdF` |

接口同时接收装配器预先算好的 SVD $\mathbf F=U\,\mathrm{diag}(\sigma_1,\sigma_2,\sigma_3)\,V^\top$——奇异值形式的模型（StVK、StableNeo 的 Hessian 特征分析）直接使用，避免重复分解。

**材料参数从哪里来**：体模型的 $(\mu,\lambda)$ 在构造单元时从 VEG 网格的 `ENuMaterial` 区域**烘焙**进模型（`elasticModelFactory.cpp` 的 `create`：$\mu,\lambda$ 由 $E,\nu$ 换算），`getNumParameters()==0`——即体模型**没有运行期可调的弹性参数通道**；要换材料就改 VEG 或重建能量。例外是 Koiter 壳（5 个可调通道，见下文 `KoiterStVK`）。

---

## class `ElasticModel`

抽象基类。持有 C++ peer（`_handle`，类型 `_core.PyElasticModel`），不做任何计算。

### 属性 `name`

```python
model.name -> str
```

返回 C++ 侧的模型标识字符串（`"stable_neo"`、`"stvk"`、`"stvk_vol"`、`"linear"`、`"mooney_rivlin"`、`"koiter_stvk"`），是 [`deformation_energy`](energy.md) 工厂派发到 C++ `ElasticModelFactory::materialFromModelId` 的键（`elasticModelFactory.cpp`）。

---

## class `StableNeo`

```python
StableNeo()
```

Stable neo-Hookean（Smith, de Goes & Kim 2018，Pixar 实现的派生版本）。大形变首选：对单元反转（$\det\mathbf F\le0$）鲁棒，能量处处有定义。

**as-implemented**（`elasticModelStableNeoHookeanMaterial.cpp:33-54`）。构造时重参数化 $\hat\lambda=\lambda+\mu$、$r=\mu/\hat\lambda$：

$$\Psi(\mathbf F)=\frac{\mu}{2}\big(I_C-3\big)
+\frac{\hat\lambda}{2}\big(J-1-r\big)^2
-\frac{\hat\lambda}{2}r^2,
\qquad I_C=\|\mathbf F\|_F^2,\quad J=\det\mathbf F$$

最后的常数项把静止态能量平移到零。应力（`compute_P`，同文件 66-79 行）：

$$\mathbf P=\mu\,\mathbf F+\hat\lambda\,(J-1-r)\,\frac{\partial J}{\partial\mathbf F},
\qquad \frac{\partial J}{\partial\mathbf F}=\big[\mathbf f_1\times\mathbf f_2\ \big|\ \mathbf f_2\times\mathbf f_0\ \big|\ \mathbf f_0\times\mathbf f_1\big]\ (\text{余子式矩阵})$$

**canonical 对照**：这正是 Smith et al. 的 $\Psi=\frac{\mu}{2}(I_C-3)+\frac{\lambda'}{2}(J-\alpha)^2$，其中 $\lambda'=\lambda+\mu$、$\alpha=1+\mu/\lambda'$，外加使 $\Psi(\mathbf I)=0$ 的常数平移。注意：**没有** $-\mu\ln J$ 项，故对 $J\le0$ 也有定义（这是"stable"的含义）。

**自检**：$\mathbf F=\mathbf I$ 时 $I_C=3$、$J=1$，$\Psi=0+\frac{\hat\lambda}{2}r^2-\frac{\hat\lambda}{2}r^2=0$ ✓；$\mathbf P(\mathbf I)=\mu\mathbf I+\hat\lambda(-r)\mathbf I=\mu\mathbf I-\mu\mathbf I=\mathbf 0$ ✓（静止无应力）。

`compute_dPdF` 用 SVD 特征系（twist/flip 特征向量）做解析特征分解，支撑 [`enforce_spd`](energy.md) 的逐元素 SPD 投影。

---

## class `StVK`

```python
StVK()
```

Saint Venant–Kirchhoff。Green 应变的二次能量，中等形变经典模型；大压缩下会失稳（能量非凸、无反转防护），需要时配合 `enable_material_max_step`（见 [energy.md](energy.md)）。

**canonical**：

$$\Psi=\mu\,\|\mathbf E\|_F^2+\frac{\lambda}{2}\,\mathrm{tr}^2(\mathbf E),
\qquad \mathbf E=\tfrac12\big(\mathbf F^\top\mathbf F-\mathbf I\big)$$

**as-implemented**（`elasticModel3DSTVKMaterial.cpp:76-88`）：用奇异值的"低阶不变量" $I_1=\sigma_1+\sigma_2+\sigma_3$、$I_2=\sigma_1^2+\sigma_2^2+\sigma_3^2$、$I_3=\sigma_1\sigma_2\sigma_3$ 写成等价形式

$$\Psi=\frac{\lambda}{8}(I_2-3)^2
+\frac{\mu}{8}\big(8I_1I_3+I_2^2+2I_1^2I_2-4I_2-I_1^4+6\big)$$

两式恒等（多项式恒等式 $\sum_i\sigma_i^4=\tfrac12(8I_1I_3+I_2^2+2I_1^2I_2-I_1^4)$，本文档撰写时已用数值样例验证）。`compute_P` 经 $\partial\Psi/\partial I_k$ 链式合成（同文件 90 行起）。

**自检**：$\sigma=(1,1,1)$ 时 $I_1=3,I_2=3,I_3=1$，两项均为零 ✓。

---

## class `StVKVolume`

```python
StVKVolume()
```

StVK + 体积保持罚（C++ `STVK_VOL`，组合材料 `ElasticModelCombinedMaterial<2>`，见 `elasticModelFactory.cpp` 的 `STVK_VOL` 分支）：

$$\Psi=\underbrace{\frac{\lambda}{8}(I_C-3)^2+\frac{\mu}{4}\big(I\!I_C-2I_C+3\big)}_{\text{不变量形式 StVK（invariantBasedMaterialStVK.cpp:27-38）}}
\;+\;\underbrace{\frac{s}{2}\,(\det\mathbf F-1)^2}_{\text{体积项（elasticModelVolumeMaterial.cpp:13-18）}}$$

其中 $I_C=\mathrm{tr}(\mathbf C)$、$I\!I_C=\mathrm{tr}(\mathbf C^2)$、$\mathbf C=\mathbf F^\top\mathbf F$；StVK 部分与上面 canonical 形式恒等（$\mu\|\mathbf E\|^2=\frac\mu4(I\!I_C-2I_C+3)$）。体积项系数 $s$ 取自 VEG 材料的 compression ratio。用于需要额外抗体积变化的 StVK 场景。

---

## class `LinearElastic`

```python
LinearElastic()
```

线弹性（小形变）。**as-implemented**（`elasticModelLinearMaterial.cpp:13-30`）：

$$\boldsymbol\varepsilon=\tfrac12(\mathbf F+\mathbf F^\top)-\mathbf I,\qquad
\Psi=\mu\,\|\boldsymbol\varepsilon\|_F^2+\frac{\lambda}{2}\,\mathrm{tr}^2(\boldsymbol\varepsilon),\qquad
\mathbf P=2\mu\,\boldsymbol\varepsilon+\lambda\,\mathrm{tr}(\boldsymbol\varepsilon)\,\mathbf I$$

$\partial\mathbf P/\partial\mathbf F$ 是常张量 ⟹ 能量是位移的精确二次型，Hessian 常量——适合验证装配正确性与作线性化基准；**不旋转不变**（大旋转会产生伪应变）。

---

## class `MooneyRivlin`

```python
MooneyRivlin()
```

广义 Mooney–Rivlin 级数（近不可压橡胶类材料）。**as-implemented**（`elasticModel3DMooneyRivlin.cpp:114` 起）：基于等容不变量

$$\bar I_1=J^{-2/3}I_1,\qquad \bar I_2=J^{-4/3}I_2,\qquad
I_1=\mathrm{tr}\,\mathbf C,\quad I_2=\tfrac12\big(I_1^2-\mathrm{tr}(\mathbf C^2)\big)$$

$$W=\sum_{p,q=0}^{N}C_{pq}\,(\bar I_1-3)^p(\bar I_2-3)^q\;+\;\text{体积（}D\text{ 系数）项}$$

参数 $(N, C_{pq}, M, D)$ 来自 VEG 的 `MooneyRivlinMaterial`（见 [../mesh/volume/material.md](../mesh/volume/material.md)），构造时烘焙。部分导数用数值微分实现（文件约 310 行）。**要求 VEG 材料是 Mooney–Rivlin 类型**——配 `ENuMaterial` 网格会在构造时抛错（`elasticModelFactory.cpp` 的派发逻辑）。

---

## class `KoiterStVK`

```python
KoiterStVK()
```

Koiter 薄壳模型，配 [`KoiterShell`](formulations.md) formulation。能量按第一/第二基本形式分解为膜 + 弯曲两项（**as-implemented**，`elasticModel2DFundamentalFormsSTVK.cpp:154-250`）：

$$\mathbf M_a=\bar{\mathbf a}^{-1}\mathbf a-\mathbf I_2,\qquad
\Psi_{\text{膜}}=h\Big(\frac{\alpha}{2}\,\mathrm{tr}^2\mathbf M_a+\beta\,\mathrm{tr}(\mathbf M_a^2)\Big)$$

$$\mathbf M_b=\bar{\mathbf a}^{-1}(\mathbf b-\bar{\mathbf b}),\qquad
\Psi_{\text{弯}}=\frac{h^3}{12}\Big(\frac{\alpha}{2}\,\mathrm{tr}^2\mathbf M_b+\beta\,\mathrm{tr}(\mathbf M_b^2)\Big)$$

其中 $\mathbf a,\mathbf b\in\mathbb R^{2\times2}$ 是当前第一/第二基本形式、$\bar{\mathbf a},\bar{\mathbf b}$ 为静止值，$\alpha=\frac{E\nu}{(1+\nu)(1-2\nu)}$、$\beta=\frac{E}{2(1+\nu)}$（3D Lamé 常数形式）。

这是**唯一带运行期参数通道的模型**——每元素 5 通道：

| 通道 | 含义 | 进入 |
|---|---|---|
| 0, 1 | $E_{\text{膜}},\ \nu_{\text{膜}}$ | $\Psi_{\text{膜}}$ 的 $\alpha,\beta$ |
| 2, 3 | $E_{\text{弯}},\ \nu_{\text{弯}}$ | $\Psi_{\text{弯}}$ 的 $\alpha,\beta$ |
| 4 | 厚度 $h$ | 膜 $\propto h$、弯曲 $\propto h^3/12$；亦被 [`ShellDensityElasticThickness(channel=4)`](mass.md) 读取 |

这使壳的厚度/刚度可经 [`set_elastic_values`](energy.md) 在线修改，也是 [`ElasticStaticEquilibriumLayer`](torch.md) 做厚度优化的基础。

---

## class `KoiterStVKShellMaterial`（冻结 dataclass）

```python
KoiterStVKShellMaterial(name="shell", thickness=0.001, E_membrane=1e6, nu_membrane=0.4)
```

壳材料的**构造期**参数载体（与上面运行期通道相区分）：交给 [`SimulationMesh.create_shell`](mesh.md) 设定壳网格的初始 $h, E, \nu$，以及壳配置文件 I/O（`read_shell_config`/`write_shell_config`）。`ShellMaterialLike` 是其类型别名。

| 字段 | 含义 |
|---|---|
| `name` | 材料名（仅标识） |
| `thickness` | 初始厚度 $h$（m） |
| `E_membrane`, `nu_membrane` | 初始 $E,\nu$ |

---

## 用法示例

```python
import pypgo

# 体网格 + stable neo-Hookean（μ, λ 来自 VEG 材料区域）
deform = pypgo.fem.deformation_energy(
    sim_mesh,
    formulation=pypgo.fem.TetLinear(),
    elastic=pypgo.fem.StableNeo(),
    elastic_field=pypgo.fem.ElementwiseField(),
    plastic=pypgo.fem.VolumetricPlasticity(dofs=0),
    plastic_field=pypgo.fem.ElementwiseField(),
)

# 壳：构造期材料 + 运行期通道
mat = pypgo.fem.KoiterStVKShellMaterial(thickness=2e-3, E_membrane=5e5, nu_membrane=0.35)
shell_mesh = pypgo.fem.SimulationMesh.create_shell(surface, mat)
```

## 模型选择速查

| 需求 | 模型 |
|---|---|
| 大形变、要鲁棒 | `StableNeo` |
| 中等形变、教科书基准 | `StVK`（或 `StVKVolume` 加体积保持） |
| 小形变 / 验证装配 | `LinearElastic` |
| 橡胶（实验拟合系数） | `MooneyRivlin` |
| 薄壳 | `KoiterStVK` |

## 交叉链接

- 消费方与参数派发：[energy.md](energy.md)（`deformation_energy(elastic=...)`）
- $(\mu,\lambda)$ 的来源：[../mesh/volume/material.md](../mesh/volume/material.md)（`ENuMaterial.lam/mu`）
- 壳厚度与质量耦合：[mass.md](mass.md)（`ShellDensityElasticThickness`）
