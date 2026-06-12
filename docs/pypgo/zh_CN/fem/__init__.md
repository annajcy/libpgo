# `pypgo/fem/__init__.py` — 公开面

> 源文件：`pypgo/fem/__init__.py`（102 行，纯重导出）。模块架构见 [overview.md](overview.md)。

## 定位

把 FEM 流水线各环节的类型重导出为扁平公开面，按概念分组（与 `__all__` 的分组注释一致）。

## 导出表

| 分组 | 符号 | 文档 |
|---|---|---|
| Formulations | `Formulation`, `VolumetricFormulation`, `ShellFormulation`, `TetLinear`, `CubicLinear`, `CubicTricubicHermite`, `KoiterShell` | [formulations.md](formulations.md) |
| 质量场 | `VolumeMassField`, `VolumeDensity`, `volume_density`, `ShellMassField`, `ShellArealDensity`, `ShellDensityThickness`, `ShellDensityElasticThickness`, `SelfWeightGravity` | [mass.md](mass.md) |
| 弹性本构 | `ElasticModel`, `StableNeo`, `StVK`, `StVKVolume`, `LinearElastic`, `MooneyRivlin`, `KoiterStVK` | [elastic.md](elastic.md) |
| 塑性 | `PlasticModel`, `VolumetricPlasticity`, `ShellPlasticity` | [plastic.md](plastic.md) |
| 参数场 | `ConstantField`, `ElementwiseField`, `ParameterField` | [fields.md](fields.md) |
| 能量 | `DeformationEnergy`, `DeformationOptions`, `ElasticMaterialEnergy`, `PlasticMaterialEnergy`, `deformation_energy`, `elastic_material_energy`, `plastic_material_energy` | [energy.md](energy.md) |
| 网格 | `SimulationMesh`, `KoiterStVKShellMaterial`, `read_shell_config`, `write_shell_config` | [mesh.md](mesh.md) |
| PyTorch | `ElasticStaticEquilibriumLayer`, `PlasticStaticEquilibriumLayer` | [torch.md](torch.md) |

注意：导入 `pypgo.fem` 会触发 `torch.py` 的导入，因此**需要 PyTorch 可用**（`import torch`）。
