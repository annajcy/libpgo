# `pypgo/mesh/geometry/core.py` — 几何门面与重心坐标嵌入

> 源文件：`pypgo/mesh/geometry/core.py`（291 行）。子包架构见 [overview.md](overview.md)。
>
> 三个几何门面持 C++ `Mesh::*MeshGeo` peer（绑定 `src/python/pypgo/mesh/geo/bindings.cpp:47-72`）；`TriMeshGeo` 的法向/面积在 **Python/NumPy** 侧计算（本篇给出公式）。`BarycentricEmbedding` 委托 C++ `InterpolationCoordinates::BarycentricCoordinates`（`src/core/interpolationCoordinates/barycentricCoordinates.cpp`，已逐段核对）。

## class `TriMeshGeo`

```python
TriMeshGeo(vertices, triangles)            # (n,3) + (m,3)
TriMeshGeo.from_mesh_data(tri_mesh_data)   # 从 TriMeshData 转
```

三角曲面几何门面。与 [`TriMeshData`](../data.md) 的关系：Data 是纯存储通货，Geo 在其上提供派生几何量；二者经 `from_mesh_data` / `to_mesh_data` 零损往返。

### 属性 `vertices` / `triangles` / `num_vertices` / `num_triangles`

存储访问：`(n,3) float64` / `(m,3) int64` 快照与计数（委托 C++）。

### 属性 `face_areas`

每面面积（57-60 行）：

$$A_f=\frac12\,\bigl\|(\mathbf v_1-\mathbf v_0)\times(\mathbf v_2-\mathbf v_0)\bigr\|$$

### 属性 `face_normals`

单位面法向（63-67 行）：

$$\mathbf n_f=\frac{(\mathbf v_1-\mathbf v_0)\times(\mathbf v_2-\mathbf v_0)}{\|\cdot\|}$$

方向遵循顶点环绕的右手定则；退化面（叉积为零）除数被替换为 1，返回零向量而非 NaN。

### 属性 `vertex_normals`

**面积加权**顶点法向（70-78 行）：先算未归一化面法向 $\tilde{\mathbf n}_f = \mathbf n_f A_f$（恰为 $\frac12$ 叉积本身），散射累加到三个顶点后归一化：

$$\mathbf n_v=\frac{\sum_{f\ni v}A_f\,\mathbf n_f}{\bigl\|\sum_{f\ni v}A_f\,\mathbf n_f\bigr\|}$$

面积加权使大面主导顶点法向；孤立顶点（无邻接面）得零向量（同样有零保护）。

### `tri_vtx_id(tri_id, local_vertex_id) -> int`

单点索引查询 $\mathbf E[t,\ell]$（免整表拷贝）。

### `to_mesh_data() -> TriMeshData`

转回数据容器。

---

## class `TetMeshGeo`

```python
TetMeshGeo(vertices, tets)                 # (n,3) + (m,4)
TetMeshGeo.from_mesh_data(tet_mesh_data)
```

四面体几何门面：`vertices` / `tets` / `num_vertices` / `num_tets` / `tet_vtx_id(tet_id, local_vertex_id)` / `to_mesh_data()`。纯索引与存储访问，无派生量——体积/质心请用 [`TetMeshData`](../data.md)。

## class `CubicMeshGeo`

```python
CubicMeshGeo(vertices, cubes)              # (n,3) + (m,8)
CubicMeshGeo.from_mesh_data(cubic_mesh_data)
```

六面体几何门面：`vertices` / `cubes` / `num_vertices` / `num_cubes` / `cube_vtx_id(cube_id, local_vertex_id)` / `to_mesh_data()`。

---

## class `BarycentricEmbedding`

```python
BarycentricEmbedding(target_locations, volume_mesh)
```

| 参数 | 含义 | 校验 |
|---|---|---|
| `target_locations` | $(m,3)$ 目标点（表面顶点、标记点等） | 形状 |
| `volume_mesh` | [`VolumeMesh`](../volume/core.md)（带材料的 Vega 体网格） | 类型 |

构造时 C++ 侧（`barycentricCoordinates.cpp:63-127`）对每个目标点 $\mathbf x$：

1. **元素定位**：对全部元素 AABB 建 BVH（惯性划分），取最近候选盒集合，逐个做 `containsVertex` 精确包含测试；
2. **外点回退**：若 $\mathbf x$ 不在任何元素内，则在候选集中选**元素中心最近**者（外推，$\beta$ 可越出 $[0,1]$）；
3. **解重心权** $\beta$，按元素类型分两式：

**四面体**（`geometryQuery.cpp:496-513`，Cramer 法）：把 $\mathbf x$ 依次替换第 $i$ 个顶点构成行列式 $D_i$，

$$\beta_i=\frac{D_i}{D_0},\qquad D_0=\det\begin{bmatrix}x_1&y_1&z_1&1\\ x_2&y_2&z_2&1\\ x_3&y_3&z_3&1\\ x_4&y_4&z_4&1\end{bmatrix}$$

即有符号子体积比，自动满足 $\sum_i\beta_i=1$。

**六面体**（`cubicMesh.cpp:622-637`，三线性）：求局部坐标 $(\alpha,\beta,\gamma)\in[0,1]^3$ 后

$$w_{000}=(1{-}\alpha)(1{-}\beta)(1{-}\gamma),\quad w_{100}=\alpha(1{-}\beta)(1{-}\gamma),\ \dots,\quad w_{111}=\alpha\beta\gamma$$

（8 个三线性形函数，按 Vega 局部顶点序排列。）

插值本身就是

$$\mathbf x_s=\sum_{i=1}^{K}\beta_i\,\mathbf x_{v_i},\qquad K=\text{num\_element\_vertices}\in\{4,8\}$$

### 属性 `num_target_locations` / `num_element_vertices`

目标点数 $m$ 与每元素顶点数 $K$。

### 属性 `embedding_indices` / `embedding_weights` / `embedding_elements`

| 属性 | 形状 | 含义 |
|---|---|---|
| `embedding_indices` | $(m,K)$ int64 | 第 $i$ 个点宿主元素的全局顶点号 $v_{i1..iK}$ |
| `embedding_weights` | $(m,K)$ float64 | 对应 $\beta_{i1..iK}$（行和 $=1$） |
| `embedding_elements` | $(m,)$ int64 | 宿主元素号 |

### 属性 `interpolation_matrix`

稀疏插值矩阵（[`SparseMatrix`](../../sparse.md)）

$$W\in\mathbb R^{3m\times3n},\qquad W_{3i+d,\ 3v_{ij}+d}=\beta_{ij}\quad(d=0,1,2)$$

每行恰 $K$ 个非零；三坐标分量共享同一组权（C++ `generateInterpolationMatrix` 经 `createWeightMatrix` 以 expand=3 展开，`barycentricCoordinates.cpp:165-171`）。

### `interpolation_matrix_coo()`

返回 $W$ 的 COO 三元组（行、列、值），便于交给 SciPy。

### `deform(volume_disp) -> np.ndarray`

应用插值：输入展平的 $3n$ 体位移，输出 $(3m,)$ 目标点位移 $\mathbf u_s=W\mathbf u_v$（C++ 直接按 $(\text{indices},\text{weights})$ 求和，不组装矩阵）。长度不符抛错。

---

## func `surface_to_volume_interpolation_matrix(surface_mesh, volume_mesh) -> SparseMatrix`

```python
W = surface_to_volume_interpolation_matrix(tri_mesh_data, volume_mesh)  # (3m, 3n)
```

便捷封装：`BarycentricEmbedding(surface_mesh.vertices, volume_mesh).interpolation_matrix`。`W @ vol_disp.ravel()` 给出表面位移。

> **与 [`formulation.surface_embedding_matrix`](../../fem/formulations.md) 的关系**：本函数是**纯节点重心**版本，列空间是体网格节点位移 $3n$；formulation 版本则按各自的 DOF 布局生成（对 `CubicTricubicHermite` 包含导数 DOF 列）。线性 formulation（TetLinear/CubicLinear）下两者一致；Hermite 网格上做软固定/接触必须用 formulation 版本（见 [../../energy/attachment.md](../../energy/attachment.md) 的 `EmbeddedVertexAttachment`）。

---

## class `SurfaceEmbedding`

```python
SurfaceEmbedding(surface_mesh, volume_mesh)
```

把体网格位移映射到**嵌入三角表面**的高层封装：构造期保存静止表面 `surface_mesh`（`TriMeshData`）并预算 $W$（一次定位、多次复用）。

### 属性 `rest_surface` / `interpolation_matrix`

静止表面与 $W$（`SparseMatrix`）。

### `displacement(volume_displacement) -> np.ndarray`

接受 $(3n,)$ 或 $(n,3)$ 体位移（内部 `_flat_volume_displacement` 统一展平并核对长度），返回 $(m,3)$ 表面位移

$$\mathbf u_s = W\,\mathbf u_v$$

### `deform(volume_displacement) -> TriMeshData`

返回变形表面（拓扑不变）：

$$\mathbf x_s = \bar{\mathbf x}_s + W\,\mathbf u_v$$

即 `TriMeshData(rest.vertices + displacement, rest.elements)`。这是"粗体网格仿真 + 细表面渲染"工作流的最后一步。

## 用法示例

```python
import numpy as np
import pypgo

veg = pypgo.mesh.volume.read_veg("examples/assets/veg/tet/box.veg")
vol = pypgo.mesh.volume.VolumeMesh.from_veg_file(veg)
surface = vol.extract_surface_mesh()

emb = pypgo.mesh.SurfaceEmbedding(surface, vol)

# 解一个静力问题后……（见 ../../fem/、../../solver/）
u = np.zeros(3 * vol.num_vertices)
u[1::3] = -0.1                       # 整体下移
moved = emb.deform(u)                # TriMeshData
np.allclose(moved.vertices[:, 1], surface.vertices[:, 1] - 0.1)   # True
```

## 数学自检

- 单位分解：`emb.embedding_weights.sum(axis=1) ≈ 1`（重心坐标性质）；
- 复现性：目标点取体网格自身顶点时，$W$ 是（置换后的）选择矩阵，`deform(u) == u` 的对应行；
- 刚体平移：$\mathbf u_v\equiv\mathbf t$ ⟹ $\mathbf u_s\equiv\mathbf t$（由 $\sum\beta_i=1$ 保证）。

## 交叉链接

- 数据容器与往返：[../data.md](../data.md)
- $W$ 的消费方：[../../energy/attachment.md](../../energy/attachment.md)（`EmbeddedVertexAttachment` 的 $c\,\|W_s u\|^2$ 型能量）、[../../contact/surface.md](../../contact/surface.md)
- formulation 感知版嵌入矩阵：[../../fem/formulations.md](../../fem/formulations.md)（`surface_embedding_matrix`）
- 稀疏矩阵类型：[../../sparse.md](../../sparse.md)
