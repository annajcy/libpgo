# `pypgo.fem` — FEM 形变能量模块架构

> 包目录：`pypgo/fem/`（9 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

把连续介质弹性理论离散成可求值的势能。本包覆盖 FEM 流水线的全部要素，最终产物是一个 [`DeformationEnergy`](energy.md)（满足 [`PotentialEnergy`](../energy/base.md) 接口），可直接进入能量组合与求解。

## 理论流水线

连续问题 → 离散能量的标准 FEM 路径，以及每一步由哪个文件负责：

$$
\underbrace{\text{运动学: } \mathbf F = \mathbf I + \frac{\partial\mathbf u}{\partial\mathbf X}}_{\text{formulation 提供形函数}}
\;\to\;
\underbrace{\text{本构: } \Psi(\mathbf F)}_{\texttt{elastic.py}}
\;\to\;
\underbrace{E(\mathbf u)=\sum_e\sum_q w_q\,\Psi\!\big(\mathbf F(\mathbf u;\xi_q)\big)\,\det J_q}_{\text{装配, \texttt{energy.py}}}
$$

1. **网格准备**（[mesh.md](mesh.md)）：`SimulationMesh` 把体网格（VEG，含材料）或壳面网格包装成 C++ 仿真网格；
2. **Formulation**（[formulations.md](formulations.md)）：决定形函数空间、每顶点 DOF 数、求积规则——即 $\mathbf F(\mathbf u;\xi)$ 如何从离散 DOF 算出；
3. **本构**（[elastic.md](elastic.md)）：能量密度 $\Psi(\mathbf F)$ 及其导数 $\mathbf P=\partial\Psi/\partial\mathbf F$、$\partial\mathbf P/\partial\mathbf F$；
4. **塑性**（[plastic.md](plastic.md)）：可选的内变量参数化（乘法分解 $\mathbf F=\mathbf F_e\mathbf F_p$ 风格的修正）；
5. **质量与体力**（[mass.md](mass.md)）：$M=\int_\Omega \rho\,N^\top N\,dV$、$\mathbf f_g=\int_\Omega \rho\,N^\top\mathbf g\,dV$；
6. **参数场**（[fields.md](fields.md)）：逐元素 / 全局共享的材料参数存储；
7. **装配**（[energy.md](energy.md)）：把上述要素组合成 `DeformationEnergy`，同时暴露对材料参数的导数；
8. **可微层**（[torch.md](torch.md)）：隐函数定理把"参数 → 平衡态"包装成 PyTorch 自动微分节点。

## 模块 ↔ 数学 ↔ 阶段 主表

| 文件 | 数学对象 | 关键公式 | 文档 |
|---|---|---|---|
| `mesh.py` | 离散域 $\Omega_h$ | — | [mesh.md](mesh.md) |
| `formulations.py` | 形函数 / DOF 布局 / 求积 | $\mathbf u(\xi)=\sum_a N_a(\xi)\,\mathbf u_a$ | [formulations.md](formulations.md) |
| `elastic.py` | 能量密度 | $\Psi(\mathbf F)$，各模型见下 | [elastic.md](elastic.md) |
| `plastic.py` | 塑性内变量 | $\mathbf F_e=\mathbf F\,\mathbf F_p^{-1}$ 风格参数化 | [plastic.md](plastic.md) |
| `mass.py` | 质量场 | $M=\int\rho N^\top N$, $\mathbf f_g=\int\rho N^\top\mathbf g$ | [mass.md](mass.md) |
| `fields.py` | 参数场 | $\theta\in\mathbb R^{n_e\times c}$ | [fields.md](fields.md) |
| `energy.py` | 离散总能量与参数导数 | $E(\mathbf u)$, $\partial E/\partial\theta$, $\partial^2E/\partial\mathbf u\,\partial\theta$ | [energy.md](energy.md) |
| `torch.py` | 可微平衡 | $\frac{d\mathbf u^*}{d\theta}=-H^{-1}\frac{\partial^2E}{\partial\mathbf u\,\partial\theta}$ | [torch.md](torch.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## 支持的组合

| Formulation | 单元 | 每顶点 DOF | 求积 | 配套本构 |
|---|---|---|---|---|
| `TetLinear` | 4 节点四面体 | 3 | 1 点（重心） | 全部体模型 |
| `CubicLinear` | 8 节点六面体 | 3 | 2×2×2 Gauss | 全部体模型 |
| `CubicTricubicHermite` | 六面体，三三次 Hermite | 24 | 4×4×4 Gauss | 全部体模型 |
| `KoiterShell` | 三角壳 | 3 | （壳专用） | `KoiterStVK` |

体模型：`StVK` / `StableNeo` / `LinearElastic` / `StVKVolume` / `MooneyRivlin`。

## C++ 引擎对应

| 阶段 | C++ 位置（`src/core/solidDeformationModel/`） |
|---|---|
| Formulation / 形函数 / 求积 | `formulations/formulation/`、`formulations/quadrature/` |
| 本构模型 | `material/elastic/`（每模型一对 .h/.cpp） |
| 塑性 | `material/plastic/` |
| 逐元素形变模型 | `deformation/volume/volumetricDeformationModel.cpp`、`deformation/shell/` |
| 装配器 | `deformation/deformationModelAssembler.cpp`（约 1100 行） |
| 工厂 / 管理 | `deformation/deformationModelManager.cpp`、`material/elastic/elasticModelFactory.cpp` |

绑定层：`src/python/pypgo/fem/{formulation,elastic,plastic,mass}/bindings.cpp` 与 `src/python/pypgo/energy/core.cpp`（`_create_deformation_energy`）。

## 贯穿示例

```python
import pypgo

veg = pypgo.mesh.read_veg("examples/assets/veg/tet/box.veg")
volume = pypgo.mesh.VolumeMesh.from_veg_file(veg)
sim_mesh = pypgo.fem.SimulationMesh.create_volumetric(volume)

fm = pypgo.fem.TetLinear()
deform = pypgo.fem.deformation_energy(
    sim_mesh,
    formulation=fm,
    elastic=pypgo.fem.StableNeo(),
    elastic_field=pypgo.fem.ElementwiseField(),   # (μ,λ) 由 VEG 材料区域决定
    plastic=pypgo.fem.VolumetricPlasticity(dofs=0),
    plastic_field=pypgo.fem.ElementwiseField(),
)

# 质量与重力（动力学 / 重力势用）
mass_field = pypgo.fem.volume_density(volume)
M = fm.mass_matrix(sim_mesh, mass_field)                       # SparseMatrix
f_g = fm.body_force(sim_mesh, [0, -9.8, 0], mass_field)        # (n,) ndarray

u = deform.zero_state()
deform.value(u)        # 0.0 — 静止态能量为零
```
