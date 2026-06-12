# `pypgo/fem/mass.py` — 质量场与自重载荷

> 源文件：`pypgo/fem/mass.py`（119 行）。模块架构见 [overview.md](overview.md)。
>
> 质量场回答"每个积分区域有多少质量"，与本构材料分离。体场单位 **kg/m³**，壳场单位 **kg/m²**（$\rho h$）。消费方是 [formulation 的动力学算子](formulations.md)：$M=\int_\Omega\rho\,N^\top N\,dV$ 与 $\mathbf f_g=\int_\Omega\rho\,N^\top\mathbf g\,dV$。

## 类型层次

```
VolumeMassField (kg/m³)           ShellMassField (kg/m²)
└── VolumeDensity                 ├── ShellArealDensity          （常数 ρh）
                                  ├── ShellDensityThickness      （ρ × 固定 h）
                                  └── ShellDensityElasticThickness（ρ × 实时 h，读弹性场）
```

formulation 的 `mass_matrix`/`body_force` 对类型强校验：体 formulation 只收 `VolumeMassField`，壳只收 `ShellMassField`——单位混用在 API 层直接报 `TypeError`。

---

## class `VolumeMassField` / class `ShellMassField`

抽象基类，各持有一个 C++ 质量场句柄（`_handle`）。无公开方法；存在的意义是类型标签 + 单位契约。

---

## class `VolumeDensity`

```python
VolumeDensity(density)
```

体密度场 $\rho(\mathbf X)$，逐元素分片常数。

| 参数 | 含义 |
|---|---|
| `density` 标量 | 全网格常数 $\rho$ → `_core.make_constant_volume_density` |
| `density` 1-D 数组 | 逐元素 $\rho_e$（长度 = 单元数）→ `_core.make_elementwise_volume_density` |

其他维度抛 `ValueError`。

---

## func `volume_density(volume) → VolumeDensity`

从 [`VolumeMesh`](../mesh/volume/core.md) 的 VEG 材料区域读出逐元素密度：遍历 `to_veg_file().to_volume_regions()`，把每个区域的 `material.density` 写入对应单元。**这是体网格最常用的入口**——密度与 $(\mu,\lambda)$ 一样来自 VEG 材料定义，保证质量与刚度出自同一材料描述。

---

## class `ShellArealDensity`

```python
ShellArealDensity(areal_density: float)
```

常数面密度 $\rho h$（kg/m²）——直接给乘积，不区分 $\rho$ 与 $h$。最简单的壳质量场。

---

## class `ShellDensityThickness`

```python
ShellDensityThickness(*, density: float, thickness)
```

$\rho\times h$，**固定**厚度：`thickness` 标量（全网格）或 1-D 逐元素数组。厚度此后不随弹性参数变化——适合厚度不参与优化的场景。

---

## class `ShellDensityElasticThickness`

```python
ShellDensityElasticThickness(*, density: float, parameter_field, channel: int = 4)
```

$\rho\times h$，厚度 $h$ **实时读取**弹性[参数场](fields.md)的指定通道。默认 `channel=4` 正是 [Koiter 壳本构的厚度通道](elastic.md)（C++ 侧 `param[4]`，见 `elasticModel2DFundamentalFormsSTVK.cpp`）。

关键性质：**与能量的弹性场共享存储**——调用 [`DeformationEnergy.set_elastic_values`](energy.md) 改了厚度后，这里看到的质量同步更新，无需手动同步。这使"厚度可微优化"中自重随厚度变化的链条闭合（见 `SelfWeightGravity` 与 [torch.md](torch.md)）。

| 参数 | 含义 |
|---|---|
| `density` | 体密度 $\rho$（kg/m³） |
| `parameter_field` | [`ParameterField`](fields.md)（通常取 `deform.elastic_field`） |
| `channel` | 厚度所在通道（Koiter 为 4） |

---

## class `SelfWeightGravity`

```python
SelfWeightGravity(*, formulation, sim_mesh, mass_field, acceleration)
```

壳自重的**外载荷提供者**：实现 [`ElasticStaticEquilibriumLayer`](torch.md) 的 `external_load` 协议。与一次性算好的 `LinearEnergy(-f_g)` 不同，它在每次前向求解时**按当前参数值重新求值**——质量依赖厚度时自重不是常数。

| 参数 | 校验 |
|---|---|
| `formulation` | 必须是 `ShellFormulation` |
| `sim_mesh` | 壳 [`SimulationMesh`](mesh.md) |
| `mass_field` | `ShellMassField`（典型为 `ShellDensityElasticThickness`） |
| `acceleration` | 3-向量 $\mathbf g$ |

### `force() → ndarray`

当前参数下的广义自重 $\mathbf f_g(\theta)=\int\rho(\theta)\,N^\top\mathbf g\,dV$，委托 [`ShellFormulation.body_force`](formulations.md)。

### `parameter_jacobian() → SparseMatrix`

$\partial\mathbf f_g/\partial\theta$，委托 `ShellFormulation.body_force_parameter_jacobian`。可微层 backward 中修正混合导数用（内层梯度是 $\nabla_uE-\mathbf f_g(\theta)$，对 $\theta$ 的混合导数要减去这一项，见 [torch.md](torch.md) 的 `_parameter_jacobian`）。

---

## 用法示例

```python
import pypgo

# 体：密度来自 VEG 材料
mass = pypgo.fem.volume_density(volume)
M = fm.mass_matrix(sim_mesh, mass)
f_g = fm.body_force(sim_mesh, [0, -9.8, 0], mass)

# 壳：厚度耦合质量 + 可微自重
deform = pypgo.fem.deformation_energy(...)        # KoiterShell + KoiterStVK
mass_h = pypgo.fem.ShellDensityElasticThickness(
    density=300.0, parameter_field=deform.elastic_field, channel=4)
load = pypgo.fem.SelfWeightGravity(
    formulation=fm_shell, sim_mesh=shell_mesh,
    mass_field=mass_h, acceleration=[0, -9.8, 0])
```

## 交叉链接

- 消费方：[formulations.md](formulations.md)（`mass_matrix`/`body_force`）、[torch.md](torch.md)（`external_load=`）
- 厚度通道语义：[elastic.md](elastic.md)（`KoiterStVK` 的 5 通道表）
- 密度来源：[../mesh/volume/material.md](../mesh/volume/material.md)
