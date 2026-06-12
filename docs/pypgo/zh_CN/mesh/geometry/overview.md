# `pypgo.mesh.geometry` — 几何查询与重心嵌入子包架构

> 包目录：`pypgo/mesh/geometry/`（3 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

[data.py](../data.md) 的容器只存 $(\mathbf V,\mathbf E)$，不回答任何几何问题。本子包补上两类能力：

1. **几何门面**（`core.py`）——`TriMeshGeo`/`TetMeshGeo`/`CubicMeshGeo` 持 C++ `Mesh::*MeshGeo` peer，提供法向、面积等派生量与逐元素索引查询；
2. **重心坐标嵌入**（`core.py`）——把体网格上的位移场线性插值到任意目标点（典型：嵌入的渲染表面）：

$$\mathbf x_s=\sum_{i=1}^{K}\beta_i\,\mathbf x_{v_i},\qquad \sum_i\beta_i=1$$

3. **纯 Python 算法**（`algorithms.py`）——三角网格连通分量族与最小包围球（Welzl）。

设计上"门面薄、算法纯"：`core.py` 是低频变动的胶水（存储与重计算在 C++），`algorithms.py` 零 C++ 几何门面依赖、直接操作数据容器。

## 理论流水线中的位置

```
TetMeshData/CubicMeshData + 材料 ──> volume.VolumeMesh
                                          │
        目标点（表面顶点 / 任意点）──────┤  BarycentricEmbedding：定位元素 + 解 β
                                          ▼
              W ∈ R^{3m×3n}（interpolation_matrix）
                 │                        │
                 │ u_s = W u_v            │ 作为约束雅可比
                 ▼                        ▼
       SurfaceEmbedding.deform     energy.EmbeddedVertexAttachment（软固定嵌入点）
```

嵌入矩阵 $W$ 同时是"前向插值"和"反向施力"的桥：能量 $E(\mathbf u_s)=E(W\mathbf u_v)$ 对体 DOF 的梯度是 $W^\top\nabla E$——这正是 [../../energy/attachment.md](../../energy/attachment.md) 的 `EmbeddedVertexAttachment` 与 [../../contact/surface.md](../../contact/surface.md) 接触嵌入的数学基础。

## 文件 ↔ 职责 主表

| 文件 | 对象 | 关键数学 | 文档 |
|---|---|---|---|
| `core.py` | 几何门面 ×3、`BarycentricEmbedding`、`SurfaceEmbedding`、`surface_to_volume_interpolation_matrix` | 面积/法向公式；$\beta$ 求解（四面体 Cramer、六面体三线性）；$W$ 结构 | [core.md](core.md) |
| `algorithms.py` | 连通分量族 ×6、`minimum_bounding_sphere` | 边/点邻接 BFS；Welzl 增量最小球 | [algorithms.md](algorithms.md) |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## C++ 引擎对应

| Python | C++ | 位置 |
|---|---|---|
| `TriMeshGeo` 等门面 | `Mesh::TriMeshGeo` / `TetMeshGeo` / `CubicMeshGeo` | `src/core/mesh/*MeshGeo.h` |
| `BarycentricEmbedding` | `InterpolationCoordinates::BarycentricCoordinates` | `src/core/interpolationCoordinates/barycentricCoordinates.cpp` |
| 四面体 $\beta$ | `Mesh::getTetBarycentricWeights` | `src/core/mesh/geometryQuery.cpp:496` |
| 六面体 $\beta$ | `CubicMesh::computeBarycentricWeights`（三线性） | `src/core/volumetricMesh/cubicMesh.cpp:622` |
| 连通分量 | `Mesh::getConnectedComponentsBy{Edge,Vertex}` 等 | `src/core/mesh/triMeshNeighbor.cpp` |
| 最小包围球 | （纯 Python） | `algorithms.py` |

绑定层：`src/python/pypgo/mesh/geo/bindings.cpp`（门面与分量函数）、`src/python/pypgo/mesh/volume/core.cpp:171-231`（`PyBarycentricEmbedding`）。

## 贯穿示例

```python
import numpy as np
import pypgo

box = pypgo.mesh.create_box(bmin=[0,0,0], bmax=[1,1,1])
tets = pypgo.mesh.tet_mesher(box, backend="tetgen")
mat = pypgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45)
vol = pypgo.mesh.volume.VolumeMesh.create_from_single_material(tets, mat)

# 几何门面：法向与面积
geo = pypgo.mesh.geometry.TriMeshGeo.from_mesh_data(box)
geo.face_areas.sum()          # ≈ 6.0（单位盒表面积）
geo.vertex_normals            # (n,3) 面积加权顶点法向

# 重心嵌入：任意点 → 体 DOF 的线性映射
pts = np.array([[0.5, 0.5, 0.5], [0.25, 0.75, 0.5]])
emb = pypgo.mesh.geometry.BarycentricEmbedding(pts, vol)
emb.embedding_weights.sum(axis=1)     # ≈ [1, 1]（单位分解）
W = emb.interpolation_matrix          # SparseMatrix, 形状 (6, 3n)

# 连通分量与包围球
parts = pypgo.mesh.geometry.split_components(box)   # [TriMeshData]
c, r = pypgo.mesh.geometry.minimum_bounding_sphere(box)
```
