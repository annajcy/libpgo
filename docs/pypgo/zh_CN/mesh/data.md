# `pypgo/mesh/data.py` — 网格数据容器

> 源文件：`pypgo/mesh/data.py`（182 行）。模块架构见 [overview.md](overview.md)。
>
> 本模块是 `pypgo.mesh` 的**最底层**：纯数据（顶点 + 元素索引），不含几何解释，对包内其余模块零依赖（杜绝循环导入）。存储在 C++ `Mesh::MeshData<K>`（`src/core/mesh/meshData.h`），Python 侧持有 `_handle` peer；体积/质心/包围盒等积分量则在 **Python/NumPy 侧**按需计算（本篇逐一给出公式）。

## 共同数学框架

一个 $K$-元素网格是二元组 $(\mathbf V, \mathbf E)$：

$$\mathbf V\in\mathbb R^{n\times3}\ \text{（顶点位置）},\qquad
\mathbf E\in\{0,\dots,n-1\}^{m\times K}\ \text{（元素索引）},\qquad
K=\begin{cases}3 & \text{三角形}\\ 4 & \text{四面体}\\ 8 & \text{六面体}\end{cases}$$

三个容器类共享私有基类 `_MeshDataBase`（57-116 行），只在 `_element_width`（$K$）与 C++ peer 类型上分化。

**四面体体积**（内部函数 `_tet_volumes`，47-49 行）：对每个四面体 $(p_0,p_1,p_2,p_3)$ 取边矩阵 $\mathbf D=[\,p_1-p_0\ \,p_2-p_0\ \,p_3-p_0\,]$，

$$V=\frac{1}{6}\,\bigl|\det\mathbf D\bigr|$$

取绝对值 ⟹ 对顶点定向不敏感（反向四面体不会产生负体积抵消）。该函数同时服务于 `TetMeshData` 与 `CubicMeshData`（后者先做 5-四面体分解）。

## 枚举 `MeshDataType`

`_core.MeshDataType` 的别名（C++ `Mesh::MeshDataType`，绑定见 `src/python/pypgo/mesh/geo/bindings.cpp:18-21`）：`Triangle` / `Tet` / `Cubic`。由各容器的 `mesh_type` 属性及 [`VolumeMesh.mesh_type`](volume/core.md) 返回。

## 共同 API（`_MeshDataBase`，三个类全部继承）

### 构造 `__init__(vertices, elements=None)`

```python
TriMeshData(vertices, elements)   # (n,3) float-like + (m,3) int-like
TetMeshData(vertices, elements)   # (m,4)
CubicMeshData(vertices, elements) # (m,8)
```

| 参数 | 含义 | 校验 |
|---|---|---|
| `vertices` | $(n,3)$ 顶点数组 | 经 `float_matrix` 转 `float64`，形状必须 $(n,3)$ |
| `elements` | $(m,K)$ 索引数组 | 经 `index_matrix` 校验非负且 $<n$ |

另一条路径：传入对应的 C++ peer（`_core.PyTriMeshData` 等）且 `elements=None` 时直接接管句柄（包内 wrap 用，63-65 行）。

### 属性 `vertices` / `elements`

每次访问从 C++ 拷出并 reshape：`vertices -> (n,3) float64`、`elements -> (m,K) int64`。是**快照**而非视图——修改返回数组不会写回网格。

### 属性 `mesh_type` / `num_vertices` / `num_elements`

委托 C++（`mesh_type()` 返回上述枚举）。

### 属性 `bbox`

```python
mesh.bbox -> (bmin, bmax)   # 各为 (3,) float64
```

$$\mathbf b_{\min}=\min_i \mathbf v_i,\qquad \mathbf b_{\max}=\max_i \mathbf v_i\quad(\text{逐坐标})$$

空网格抛 `ValueError`（92-96 行）。

### `element_vtx_id(element_id, local_vertex_id) -> int`

返回元素 `element_id` 的第 `local_vertex_id` 个全局顶点号，即 $\mathbf E[e,\ell]$（C++ 侧单点查询，免整表拷贝）。

### `take_elements(indices)`

按元素索引取子网格：返回 `self.__class__(self.vertices, self.elements[idx])`。**注意（as-implemented，101-103 行）：顶点数组原样保留、不压缩**——未被引用的顶点成为孤立顶点，需要时再过 [`remove_isolated_vertices`](processing/surface.md)。

### classmethod `concatenate(meshes)`

拼接同类型网格列表：顶点纵向堆叠，元素索引加顶点数前缀和偏移

$$\mathbf E'_k = \mathbf E_k + \sum_{j<k} n_j$$

空列表抛 `ValueError`，类型不一致抛 `TypeError`（105-116 行）。

---

## class `TriMeshData`

```python
TriMeshData(vertices, triangles)
```

三角曲面网格（`MeshData<3>`）。只有共同 API，无派生积分量——面积/法向等几何量在 [`TriMeshGeo`](geometry/core.md) 门面上。它是整个包的"通货"：OBJ I/O、形状工厂、processing 全部函数、[`SurfaceEmbedding`](geometry/core.md)、[接触面](../contact/surface.md)都以它为输入输出。

---

## class `TetMeshData`

```python
TetMeshData(vertices, tets)
```

四面体体网格（`MeshData<4>`）。在共同 API 外增加两个积分量：

### 属性 `volume`

$$V_{\text{tot}}=\sum_{e=1}^{m}\frac{1}{6}\bigl|\det[\,p^e_1-p^e_0\ \ p^e_2-p^e_0\ \ p^e_3-p^e_0\,]\bigr|$$

（134-137 行，向量化实现：`points = vertices[elements]` 后批量 `det`。）

### 属性 `center_of_mass`

体积加权的元素质心平均（密度均匀假设下的真质心）：

$$\mathbf c=\frac{\sum_e V_e\,\bar{\mathbf p}^e}{\sum_e V_e},\qquad
\bar{\mathbf p}^e=\frac14\sum_{i=0}^{3}p^e_i$$

四面体的形心恰是 4 顶点算术平均，故该式精确。退化保护（141-144 行）：全部 $V_e=0$ 或无元素时退回顶点算术平均。

---

## class `CubicMeshData`

```python
CubicMeshData(vertices, cubes)
```

六面体（立方体）体网格（`MeshData<8>`）。顶点局部编号沿 Vega/VTK 六面体约定：底面 $0,1,2,3$ 逆时针、顶面 $4,5,6,7$ 对应其上方。

### 5-四面体分解（类属性 `_tet_decomposition`，156-165 行）

六面体的积分量经固定的 **5-四面体分解**计算：

$$\{(0,1,3,4),\ (1,2,3,6),\ (1,3,4,6),\ (1,4,5,6),\ (3,4,6,7)\}$$

四个"角四面体"+ 一个中央四面体 $(1,3,4,6)$，对（可能经线性变换的）六面体无缝覆盖且互不重叠。`_decomposed_tets()`（167-169 行）把 $(m,8,3)$ 的立方体顶点重排为 $(5m,4,3)$ 的四面体顶点。

### 属性 `volume`

$$V_{\text{tot}}=\sum_{e}\sum_{t=1}^{5}\frac16\bigl|\det\mathbf D^{e,t}\bigr|$$

对轴对齐立方体退化为 $\sum_e h_e^3$（数值自检：单位立方体得 $1.0$，本文档撰写时已验证）。

### 属性 `center_of_mass`

与 `TetMeshData` 同式，但权重与形心取自 $5m$ 个分解四面体；同样有全零体积退回顶点平均的保护（176-182 行）。

> 注意区别：这里的 5-四面体分解只服务**几何积分量**；FEM 装配中六面体使用三线性形函数（[../fem/formulations.md](../fem/formulations.md) 的 `CubicLinear`），并不做四面体化。

---

## 内部辅助（模块私有，供包内引用）

| 函数 | 作用 |
|---|---|
| `_flat_vertices` / `_flat_indices` | Python 数组 → 扁平 list（喂 C++ 工厂） |
| `_array_from_core` | C++ 扁平向量 → `(.., K)` ndarray（geometry 门面也用它） |
| `_wrap_mesh_data_core` | 按 C++ peer 类型派发回三个容器类（[volume/core.md](volume/core.md) 的 `read_veg` 用） |
| `_tet_volumes` | 上述四面体体积批量公式 |

## 用法示例

```python
import numpy as np
import pypgo

verts = np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1.]])
tet = pypgo.mesh.TetMeshData(verts, [[0,1,2,3]])
tet.volume           # 0.1666... = 1/6
tet.center_of_mass   # array([0.25, 0.25, 0.25])
tet.bbox             # (array([0.,0.,0.]), array([1.,1.,1.]))

two = pypgo.mesh.TetMeshData.concatenate([tet, tet])
two.num_elements     # 2（第二份索引自动偏移 +4）
```

## 交叉链接

- 几何查询（法向/面积）与嵌入：[geometry/core.md](geometry/core.md)
- 加材料成为可仿真体网格：[volume/core.md](volume/core.md)（`VolumeMesh`）
- 体积/质心的汇总报表：[processing/volume.md](processing/volume.md)（`volume_mesh_info`）
- 绘制：[visualize.md](visualize.md)
