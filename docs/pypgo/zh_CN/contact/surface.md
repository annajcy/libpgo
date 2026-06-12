# `pypgo/contact/surface.py` — 接触表面与顶点嵌入元数据

> 源文件：`pypgo/contact/surface.py`（112 行）。模块架构见 [overview.md](overview.md)。
>
> C++ peer：`PyContactSurface`（持有 `Contact::ContactSurfaceSpec`），`src/python/pypgo/contact/core.cpp:158-241`；表面↔仿真链式法则在 `src/core/contact/mappedSurfacePotentialEnergy.cpp`。

## 共同数学框架

所有接触能量都在**表面位置** $\mathbf x_s\in\mathbb R^{3n_s}$ 上定义，而求解变量是**仿真位移** $\mathbf u\in\mathbb R^{n_{\text{sim}}}$。`ContactSurface` 封装这层仿射映射：

$$\mathbf x_s(\mathbf u) = \bar{\mathbf x}_s + S\,\mathbf u,\qquad S\in\mathbb R^{3n_s\times n_{\text{sim}}}\ \text{（稀疏）}$$

其中 $\bar{\mathbf x}_s$ 是表面静止位置（`rest_vertices` 展平），$S$ 是 `surface_from_simulation` 位移插值矩阵。链式法则由 C++ 基类 `MappedSurfacePotentialEnergy` 统一实现（`mappedSurfacePotentialEnergy.cpp:48-74`）：

$$E(\mathbf u)=E_s(\mathbf x_s),\qquad
\nabla_{\mathbf u}E = S^\top\,\nabla_{\!s}E_s,\qquad
\nabla^2_{\mathbf u}E = S^\top H_s\, S$$

`max_step` 的 CCD 同样先把 $(\mathbf u,\Delta\mathbf u)$ 映成 $(\mathbf x_s, S\Delta\mathbf u)$ 再做（`ipcContactEnergy.cpp:108-117`）。由于 $S$ 常数，表面能量的凸性 / PSD 投影性质被 pullback 保持。

---

## class `ContactVertexEmbedding`（冻结 dataclass）

```python
ContactVertexEmbedding(indices, weights)
```

**可选**的每接触顶点嵌入元数据：记录每个表面顶点由哪些仿真顶点以何权重插值。这是给上层工具（如摩擦后处理、调试可视化）用的**纯 Python 数据**，不进入 C++——C++ 只认矩阵 $S$。

| 字段 | 形状 | 约束（`__post_init__` 强制） |
|---|---|---|
| `indices` | `(n_s, k)` int64 | 二维、非负 |
| `weights` | `(n_s, k)` float64 | 与 `indices` 同形、有限、**每行和为 1**（重心权重 $\sum_j w_{ij}=1$） |

### 属性 `embedding_arity`

每顶点参与插值的仿真顶点数 $k$（`indices.shape[1]`），如四面体重心嵌入 $k=4$。

---

## class `ContactSurface`（冻结 dataclass）

字段：`_handle`（C++ peer）、`rest_vertices`（`(n_s,3)` 副本）、`vertex_embedding`（可选 `ContactVertexEmbedding`）。不直接构造，用三个静态工厂之一。

### 静态方法 `identity(rest_vertices, *, vertex_embedding=None)`

$S=I_{3n_s}$（C++ 侧 `makeIdentityMap`，`core.cpp:24-34`）：表面顶点即仿真顶点，$\mathbf x_s=\bar{\mathbf x}_s+\mathbf u$。适用于壳 / 表面网格本身就是仿真网格的场景（`pypgo/tools/sim/_scene.py:324` 的壳场景即用此构造）。

### 静态方法 `embedded(rest_vertices, surface_from_simulation, *, vertex_embedding=None)`

显式给出 $S$（任何可被 [`pypgo.sparse.as_sparse_handle`](../sparse.md) 接受的稀疏矩阵）。C++ 校验 `S.rows() == 3 * n_s`（`core.cpp:238-239`）。适用于体网格仿真：表面是嵌入面，$S$ 是把体位移插值到表面顶点的重心矩阵。

### 静态方法 `from_surface_embedding(embedding, *, vertex_embedding=None)`

便捷封装：从 [`pypgo.mesh.SurfaceEmbedding`](../mesh/geometry/core.md)（鸭子类型，要求 `rest_surface` 与 `interpolation_matrix` 属性）取出 $\bar{\mathbf x}_s$ 与 $S$ 转调 `embedded`。这是体网格流程的标准入口。

### 属性 `num_surface_vertices` / `num_surface_dofs` / `num_simulation_dofs`

分别返回 $n_s$、$3n_s$（$=\operatorname{rows}(S)$）、$n_{\text{sim}}$（$=\operatorname{cols}(S)$），直读 C++ spec。

## 用法示例

```python
import pypgo

# 体网格：嵌入表面（真实流程，pypgo/tools/sim/_scene.py:239）
surface = pypgo.contact.ContactSurface.embedded(surf_vertices, surface_map)

# 壳/表面仿真：恒等映射
surface2 = pypgo.contact.ContactSurface.identity(rest_vertices)

surface.num_surface_dofs      # 3 * n_s
surface.num_simulation_dofs   # 体网格 DOF 数

# 交给任意接触能量
ipc = pypgo.contact.IPCEnergy(surface, surface_triangles)
ipc.num_dofs == surface.num_simulation_dofs   # True：能量定义在仿真 DOF 上
```

## 交叉链接

- 消费方：[energies.md](energies.md)（所有能量第一个参数）
- $S$ 的来源：[../mesh/geometry/core.md](../mesh/geometry/core.md)（`SurfaceEmbedding`）
- 稀疏矩阵协议：[../sparse.md](../sparse.md)
