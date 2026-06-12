# `pypgo.mesh.volume` — 体网格、材料与 VEG I/O 子包架构

> 包目录：`pypgo/mesh/volume/`（3 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

FEM 仿真需要的不只是几何：还要知道**每个元素是什么材料**。本子包定义 Vega FEM `.veg` 格式的完整数据模型并提供两级抽象：

1. **纯数据层**（`VegFile` + `MeshSet`/`MeshRegion`/材料 dataclass）——可自由读改写的 Python 值对象；
2. **句柄层**（`VolumeMesh`）——构造校验后的 C++ `VolumetricMeshes::TetMesh`/`CubicMesh` peer，是 [`pypgo.fem.SimulationMesh.create_volumetric`](../../fem/mesh.md) 的唯一输入。

VEG 的"材料分区"模型：

$$\text{VegFile}=\bigl(\underbrace{(\mathbf V,\mathbf E)}_{\text{mesh\_data}},\ \underbrace{\{M_k\}}_{\text{materials}},\ \underbrace{\{S_j\}}_{\text{sets（命名元素集合）}},\ \underbrace{\{(k_r, j_r)\}}_{\text{regions（材料}\to\text{集合）}}\bigr)$$

每个 region 把一个材料赋给一个元素集合；`VolumeMesh` 构造时强制**完全划分**——每个元素恰属一个 region。

## 理论流水线中的位置

```
TetMeshData/CubicMeshData（../data.md）          .veg 文件
        │ + ENuMaterial / MooneyRivlinMaterial       │ read_veg
        ▼                                            ▼
   VolumeMesh.create_from_single_material      VegFile（纯数据，可编辑）
        │                                            │ VolumeMesh.from_veg_file
        └──────────────► VolumeMesh ◄────────────────┘
                            │ fem.SimulationMesh.create_volumetric
                            ▼
              FEM 装配：(μ, λ) 从 ENuMaterial 烘焙进本构（../../fem/elastic.md）
```

材料参数走向：`ENuMaterial(E, ν)` → C++ `VolumetricMesh::ENuMaterial::getLambda()/getMu()` → 本构构造期烘焙（运行期不可改，换材料需重建能量）。

## 文件 ↔ 职责 主表

| 文件 | 对象 | 关键数学 | 文档 |
|---|---|---|---|
| `material.py` | `ENuMaterial`、`MooneyRivlinMaterial`、`MaterialLike` | $\lambda=\frac{E\nu}{(1+\nu)(1-2\nu)}$，$\mu=\frac{E}{2(1+\nu)}$ | [material.md](material.md) |
| `core.py` | `MeshSet`/`MeshRegion`/`VegFile`、`VolumeMesh`、`read_veg`/`write_veg`/`read_msh` | 分区完全性校验 | [core.md](core.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## C++ 引擎对应

| Python | C++ | 位置 |
|---|---|---|
| `VolumeMesh` | `VolumetricMeshes::TetMesh` / `CubicMesh`（Vega FEM 4.0） | `src/core/volumetricMesh/` |
| `ENuMaterial` | `VolumetricMesh::ENuMaterial` | `src/core/volumetricMesh/volumetricMeshENuMaterial.h` |
| `MooneyRivlinMaterial` | `VolumetricMesh::MooneyRivlinMaterial` | `src/core/volumetricMesh/volumetricMeshMooneyRivlinMaterial.h` |
| `read_veg` / `write_veg` | `VolumetricMeshes::readVegFile` / `writeVegFile` | `src/core/volumetricMesh/vegFile.h` |
| `read_msh` | `VolumetricMeshes::loadMshFile` | `src/core/volumetricMesh/loadMshFile.h` |
| `extract_surface_mesh` | `GenerateSurfaceMesh::computeMesh` | `src/core/volumetricMesh/generateSurfaceMesh.h` |

绑定层：`src/python/pypgo/mesh/volume/bindings.cpp` + `volume/core.cpp`（`create_volume_mesh_multi`、`PyVegPayload`、材料 payload 工厂）。

## 贯穿示例

```python
import pypgo
from pypgo.mesh.volume import ENuMaterial, VegFile, VolumeMesh, read_veg, write_veg

# 路径 A：从网格生成
tets = pypgo.mesh.tet_mesher(pypgo.mesh.create_box(bmin=[0,0,0], bmax=[1,1,1]),
                             backend="tetgen")
soft = ENuMaterial(name="soft", E=5e4, nu=0.45, density=1000.0)
veg = VegFile.from_single_material(tets, soft)
write_veg("box.veg", veg)

# 路径 B：读盘 + 多材料编辑
veg2 = read_veg("box.veg")
veg2.materials[0].E = 1e6                  # 纯数据，可直接改
vol = VolumeMesh.from_veg_file(veg2)

# 进入 FEM
sim_mesh = pypgo.fem.SimulationMesh.create_volumetric(vol)
```
