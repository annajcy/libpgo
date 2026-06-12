# `pypgo/implicit/__init__.py` — 包公开面

> 源文件：`pypgo/implicit/__init__.py`（41 行）。模块架构见 [overview.md](overview.md)。

纯 re-export，无逻辑。docstring 声明演化约定：`ImplicitField` 是稳定契约，形状原语（及 `thicken_mesh_surface`）在 `fields.py` 一侧增长，`grid` 持采样栅格规格，`extract` 持 marching cubes 与可选 OpenVDB 后端。

## 导出表（`__all__`，13 个符号）

| 符号 | 来源 | 类别 | 文档 |
|---|---|---|---|
| `ImplicitField` | `base.py` | 场抽象基类 | [base.md](base.md) |
| `GridField` | `base.py` | 栅格采样场 | [base.md](base.md) |
| `GridSpec` | `grid.py` | 采样栅格规格 | [grid.md](grid.md) |
| `SphereField` | `fields.py` | 球 SDF | [fields.md](fields.md) |
| `BoxField` | `fields.py` | 轴对齐盒场 | [fields.md](fields.md) |
| `MeshUnsignedDistanceField` | `fields.py` | 网格无符号距离场 | [fields.md](fields.md) |
| `thicken_mesh_surface` | `fields.py` | 网格加厚流水线 | [fields.md](fields.md) |
| `extract_marching_cubes` | `extract.py` | MC 提取 | [extract.md](extract.md) |
| `has_openvdb` | `extract.py` | OpenVDB 能力探测 | [extract.md](extract.md) |
| `OpenVDBOptions` | `extract.py` | OpenVDB 参数 | [extract.md](extract.md) |
| `build_openvdb_shell_from_mesh` | `extract.py` | 网格 → level-set 壳 | [extract.md](extract.md) |
| `build_openvdb_from_grid_field` | `extract.py` | GridField → level-set | [extract.md](extract.md) |
| `extract_openvdb` | `extract.py` | level-set → 网格 | [extract.md](extract.md) |

## 用法示例

```python
from pypgo.implicit import GridSpec, SphereField, extract_marching_cubes

spec = GridSpec([-1, -1, -1], [1, 1, 1], 64)
grid = SphereField([0, 0, 0], 0.8).sample_to_grid(spec)
mesh = extract_marching_cubes(grid)
```
