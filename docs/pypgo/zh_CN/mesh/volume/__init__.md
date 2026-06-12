# `pypgo/mesh/volume/__init__.py` — 公开面

> 源文件：`pypgo/mesh/volume/__init__.py`（29 行，纯重导出）。子包架构见 [overview.md](overview.md)。

## 定位

把材料载体（`material.py`）与 VEG 数据模型/句柄（`core.py`）重导出为扁平公开面。本子包由 `pypgo.mesh` 经 `__getattr__` 惰性加载（首次访问 `pypgo.mesh.volume` 才 import）。

## 导出表

| 符号 | 来源文件 | 对象 |
|---|---|---|
| `ENuMaterial` | `material.py` | 各向同性线弹性材料 $(E,\nu,\rho)$，含 $\lambda,\mu$ 换算 |
| `MooneyRivlinMaterial` | `material.py` | Mooney–Rivlin 超弹材料 $(\mu_{01},\mu_{10},v_1,\rho)$ |
| `MaterialLike` | `material.py` | 二者的 union 类型别名 |
| `MeshSet` | `core.py` | 命名元素集合 |
| `MeshRegion` | `core.py` | 材料 ↔ 集合配对 |
| `VegFile` | `core.py` | VEG 文件纯数据模型 |
| `VolumeMesh` | `core.py` | Vega 体网格句柄 |
| `read_msh` | `core.py` | Gmsh `.msh` → `TetMeshData` |
| `read_veg` / `write_veg` | `core.py` | `.veg` 读写 |

`__all__`（18-29 行）与上表一致（按字母序）。
