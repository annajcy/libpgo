# `pypgo.mesh` — 网格数据、几何与处理模块架构

> 包目录：`pypgo/mesh/`（4 个顶层文件 + 3 个子包，共 12 个 .py）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

pypgo 中一切仿真对象都从**网格**出发：弹性体是体网格（[../fem/overview.md](../fem/overview.md)），接触面与壳是三角面网格（[../contact/overview.md](../contact/overview.md)）。本包负责网格生命周期中"进入求解器之前"的全部环节：

1. **数据容器**（`data.py`）——顶点 $\mathbf V\in\mathbb R^{n\times3}$ + 元素索引 $\mathbf E\in\mathbb Z^{m\times k}$（$k=3/4/8$）的纯存储，附带包围盒/体积/质心等积分量；
2. **几何查询与嵌入**（`geometry/`）——法向、面积、连通分量、最小包围球，以及把体网格位移插值到任意点的重心坐标嵌入 $x_s=\sum_i\beta_i\,x_{v_i}$；
3. **体网格与材料**（`volume/`）——Vega FEM `.veg` 数据模型（网格 + 材料 + 集合 + 区域）与 `VolumeMesh` 句柄，是 [`pypgo.fem.SimulationMesh`](../fem/mesh.md) 的直接输入；
4. **网格生成与修复**（`processing/`）——TetGen/fTetWild 四面体化、体素化六面体化、CGAL 重网格化与质量诊断；
5. **可视化**（`visualize.py`）——PyVista 后端的统一绘图门面。

## 理论流水线中的位置

```
OBJ / 程序化形状（create_box/sphere/...）
        │ read_obj
        ▼
TriMeshData ──processing.check_surface_quality / cgal_*──> 干净闭合曲面
        │ processing.tet_mesher / cubic_mesher
        ▼
TetMeshData / CubicMeshData ──+ volume.ENuMaterial──> volume.VolumeMesh（.veg 往返）
        │ fem.SimulationMesh.create_volumetric                │ geometry.SurfaceEmbedding
        ▼                                                     ▼
   FEM 能量装配（../fem/）                        体位移 → 表面位移（渲染/接触/软固定）
```

正向（建模 → 求解）走左列；反向（求解结果 → 表面）走右列的嵌入插值

$$\mathbf u_s = W\,\mathbf u_v,\qquad W\in\mathbb R^{3m\times3n}\ \text{（重心坐标插值矩阵）}$$

## 文件 ↔ 职责 主表

| 文件 | 数学/数据对象 | 关键公式 | 文档 |
|---|---|---|---|
| `data.py` | 网格数据容器 | 四面体体积 $V=\tfrac16\lvert\det[\mathbf e_1\,\mathbf e_2\,\mathbf e_3]\rvert$；六面体 5-四面体分解 | [data.md](data.md) |
| `geometry/core.py` | 几何门面 + 重心嵌入 | $x_s=\sum_i\beta_i x_{v_i}$，$\sum_i\beta_i=1$ | [geometry/core.md](geometry/core.md) |
| `geometry/algorithms.py` | 连通分量、最小包围球 | Welzl 增量算法 | [geometry/algorithms.md](geometry/algorithms.md) |
| `volume/core.py` | VEG 数据模型与 `VolumeMesh` | — | [volume/core.md](volume/core.md) |
| `volume/material.py` | 材料参数载体 | $\lambda=\frac{E\nu}{(1+\nu)(1-2\nu)}$，$\mu=\frac{E}{2(1+\nu)}$ | [volume/material.md](volume/material.md) |
| `processing/surface.py` | 曲面质量/修复 | 退化判据 $A_f\le\varepsilon$、非流形边计数 | [processing/surface.md](processing/surface.md) |
| `processing/volume.py` | 体网格生成 | TetGen / fTetWild / 体素化 | [processing/volume.md](processing/volume.md) |
| `visualize.py` | PyVista 绘图 | — | [visualize.md](visualize.md) |
| `__init__.py` | 公开面 + OBJ I/O + 形状工厂 | — | [\_\_init\_\_.md](__init__.md) |
| `geometry/__init__.py` / `volume/__init__.py` / `processing/__init__.py` | 子包公开面 | — | [geometry/\_\_init\_\_.md](geometry/__init__.md) · [volume/\_\_init\_\_.md](volume/__init__.md) · [processing/\_\_init\_\_.md](processing/__init__.md) |

## C++ 引擎对应

| Python | C++ 类/函数 | 位置 |
|---|---|---|
| `TriMeshData` / `TetMeshData` / `CubicMeshData` | `Mesh::MeshData<3/4/8>` | `src/core/mesh/meshData.h` |
| `TriMeshGeo` / `TetMeshGeo` / `CubicMeshGeo` | `Mesh::TriMeshGeo` 等 | `src/core/mesh/triMeshGeo.h` 等 |
| `BarycentricEmbedding` | `InterpolationCoordinates::BarycentricCoordinates` | `src/core/interpolationCoordinates/barycentricCoordinates.cpp` |
| 连通分量函数 | `Mesh::getConnectedComponentsByEdge` 等 | `src/core/mesh/triMeshNeighbor.cpp` |
| `VolumeMesh` | `VolumetricMeshes::TetMesh` / `CubicMesh`（Vega FEM） | `src/core/volumetricMesh/` |
| `read_veg` / `write_veg` | `VolumetricMeshes::readVegFile` / `writeVegFile` | `src/core/volumetricMesh/vegFile.h` |
| `tet_mesher` | `tet_mesher::generateTetgenMesh` / `generateTetwildMesh` | `src/core/volumetricMeshMeshing/tetgenBackend.cpp`、`tetwildBackend.cpp` |
| `cubic_mesher` | `cubic_mesher::createTriangleMeshCubicMesh` | `src/core/volumetricMeshMeshing/triangleMeshVoxelizer.cpp` |
| `cgal_*` / `merge_close_vertices` / `raw_surface_cleanup` | `CGALInterface::*` | `src/core/cgalInterface/cgalInterface.cpp` |
| `check_self_intersections`（`check_surface_quality` 内部） | `Mesh::TriMeshBVTree::selfIntersectionExact` | `src/core/mesh/boundingVolumeTree.h` |

绑定层：`src/python/pypgo/mesh/geo/bindings.cpp` + `geo/core.cpp`（网格/算法/处理），`src/python/pypgo/mesh/volume/bindings.cpp` + `volume/core.cpp`（VEG/VolumeMesh/嵌入）。

## 设计分层（包 docstring 的要点）

- `data.py` 零内部依赖，其他模块（geometry/volume/processing）单向 import 它，杜绝循环导入；
- 几何门面（`geometry/core.py`）持有 C++ peer（`_handle`），重计算在 C++；纯 Python 算法集中在 `geometry/algorithms.py`；
- `pypgo.mesh.geometry` 与 `pypgo.mesh.volume` 经 `__getattr__` **惰性加载**——`import pypgo.mesh` 不会拉起 SciPy 稀疏等重依赖；
- CGAL / fTetWild 是**可选构建后端**，用 `has_cgal_remesher()` / `has_tetwild()` 探测，未编译时对应函数抛 `RuntimeError`。

## 贯穿示例

```python
import numpy as np
import pypgo

# 1. 程序化曲面 → 质量检查 → 四面体化
box = pypgo.mesh.create_box(bmin=[0, 0, 0], bmax=[1, 1, 1])
assert pypgo.mesh.check_surface_quality(box).is_clean
tets = pypgo.mesh.tet_mesher(box, backend="tetgen")          # TetMeshData
print(pypgo.mesh.volume_mesh_info(tets))                     # 体积 ≈ 1, 质心 ≈ (0.5,0.5,0.5)

# 2. 加材料 → VolumeMesh →（→ fem.SimulationMesh）
mat = pypgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45, density=1000.0)
vol = pypgo.mesh.volume.VolumeMesh.create_from_single_material(tets, mat)
sim_mesh = pypgo.fem.SimulationMesh.create_volumetric(vol)

# 3. 表面嵌入：体位移 → 表面位移
surface = vol.extract_surface_mesh()
emb = pypgo.mesh.SurfaceEmbedding(surface, vol)
u_vol = np.zeros(3 * tets.num_vertices)
deformed = emb.deform(u_vol)                                 # TriMeshData（拓扑不变）

# 4. 可视化（notebook）
pypgo.mesh.plot_surface([box, deformed], titles=["input", "deformed"])
```
