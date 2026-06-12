# `pypgo/mesh/geometry/algorithms.py` — 连通分量与最小包围球

> 源文件：`pypgo/mesh/geometry/algorithms.py`（207 行）。子包架构见 [overview.md](overview.md)。
>
> 连通分量族是 C++ 委托（`src/core/mesh/triMeshNeighbor.cpp`，绑定 `src/python/pypgo/mesh/geo/bindings.cpp:107-112`）；最小包围球是**纯 Python** 实现的 Welzl 算法。全部函数只接受数据容器（[../data.md](../data.md)），不依赖几何门面。

## 连通性的两种定义

| 关系 | 定义 | 强弱 |
|---|---|---|
| **边连通**（edge-connected） | 两三角形共享一条**边**视为相邻 | 强 |
| **点连通**（vertex-connected） | 两三角形共享一个**顶点**即视为相邻 | 弱（会把只在尖点处接触的分量并到一起） |

边连通用于"按壳分割"；点连通用于"凡有接触就算一体"（如提取外壳时不漏掉尖点连接的部分）。

---

## func `triangle_component_ids(tri_data) -> (component_ids, component_sizes)`

**边连通**分量标签：

- `component_ids` — `(m,)` int64，三角形号 → 分量号；
- `component_sizes` — `(C,)` int64，分量号 → 三角形数（**未排序**）。

C++ `Mesh::computeTriangleEdgeComponentIDs`。适合需要逐三角形标签（上色、筛选）而非分组列表的场景。

## func `connected_components_by_edge(tri_data) -> list[np.ndarray]`

按**边连通**拆分：返回每分量一个 int64 三角形索引数组。顺序由 C++ 库给出（通常按尺寸降序，docstring 36-38 行的措辞是 "typically"——不要依赖严格排序）。

## func `connected_components_by_vertex(tri_data) -> list[np.ndarray]`

按**点连通**拆分，其余同上。分量数 $\le$ 边连通分量数。

## func `filter_small_components(tri_data, *, min_triangles, keep_largest=-1) -> TriMeshData`

删除三角形数 $<$ `min_triangles` 的**边连通**分量：

| 参数 | 含义 |
|---|---|
| `min_triangles` | 阈值（保留 $\ge$ 该值的分量） |
| `keep_largest > 0` | 阈值过滤后再只保留最大的 N 个 |
| `keep_largest = -1` | 保留全部过阈值分量（默认） |

C++ `Mesh::filterSmallTriangleComponentsByEdge`；返回网格**已移除孤立顶点**。典型用途：清理扫描/marching-cubes 产物中的碎屑。

## func `get_outer_component(tri_data) -> TriMeshData`

提取**最外层**的点连通分量。算法（C++ `getOneOuterTriMeshConnectedComponentByVertex`，`triMeshNeighbor.cpp:1015-1032`，已核对）：

1. 找 **y 坐标最高的顶点**所在三角形——闭合嵌套壳体中，全局最高点必属外壳；
2. 以它为种子做点邻接 BFS，收下整个点连通分量；
3. Python 绑定侧（`geo/core.cpp:446-461`）取补集删除其余三角形并移除孤立顶点。

适合分离嵌套壳/空腔网格的外表面。**假设**：外壳确实包含全局最高点（对开放/交叠几何不保证）；"外"由 +y 方向定义。

## func `split_components(tri_data) -> list[TriMeshData]`

按边连通拆成子网格列表（按分量三角形数降序，继承 C++ 顺序）。**as-implemented 注意**（87-95 行）：实现是 `take_elements(ids)`，每个子网格**保留完整顶点数组**（docstring 声称 compacted，与实现不符）——子网格含未引用顶点，必要时对每个结果再调 [`remove_isolated_vertices`](../processing/surface.md)。

---

## func `minimum_bounding_sphere(mesh_data, *, padding=0.0) -> (center, radius)`

全部顶点的**最小包围球**，纯 Python 实现的 Welzl 算法（增量形式，102-207 行）。接受三类容器中任何一种（只要求 `.vertices` / `.num_vertices`）。

返回：

- `center` — `(3,)` float64；
- `radius` — float，结果乘 $(1+\text{padding})$（便于做含安全边距的剔除球/布置球）。

### 算法（as-implemented）

经典 Welzl 期望线性时间的增量版本，递归被展开为四层循环（160-207 行）：

1. 顶点列表用 `random.Random(0)` 洗牌——**种子固定为 0**，结果可复现；
2. 顺扫各点 $p_i$：若在当前球内（容差 $10^{-12}\max(1,r)$ 相对化，`_mbs_contains`）跳过；否则 $p_i$ 必在新最小球边界上，以 $\{p_i\}$ 重启内层扫描；
3. 同理逐层把违反点加入边界支撑集，最深处由 4 个边界点确定球。

边界支撑集大小 $k$ 的球构造：

| $k$ | 构造 | 实现 |
|---|---|---|
| 2 | 直径球：$c=\frac{a+b}2$，$r=\frac{\|b-a\|}2$ | `_mbs_from_2` |
| 3 | 三点外接圆（球心在三角形平面内）：$c=a+\dfrac{(\mathbf n\times\mathbf{ab})\,\|\mathbf{ac}\|^2+(\mathbf{ac}\times\mathbf n)\,\|\mathbf{ab}\|^2}{2\,\|\mathbf n\|^2}$，$\mathbf n=\mathbf{ab}\times\mathbf{ac}$ | `_mbs_from_3`（共线时分母 $<10^{-30}$ 返回 `None`） |
| 4 | 四点外接球：解 $2(b-a;\,c-a;\,d-a)\,c=(\lVert b\rVert^2-\lVert a\rVert^2;\ \dots)$ | `_mbs_from_4`（共面奇异返回 `None`） |

**退化回退** `_mbs_from_boundary`（132-157 行）：当 4 点共面无法定球时，穷举边界点的全部 2/3/4 点子集，取**包含全部边界点的最小**候选球——保证共线/共面输入也有正确答案。

### 复杂度与适用性

期望 $O(n)$ 球检测次数，但实现为纯 Python 逐点循环（顶点逐个取出、`pts` 为 Python list）——适合 $10^4$ 量级顶点的工具性调用（包围球初始化、相机取景、碰撞预剔除），不适合每帧热路径。

## 用法示例

```python
import pypgo
from pypgo.mesh.geometry import (
    connected_components_by_edge, filter_small_components,
    minimum_bounding_sphere, split_components,
)

mesh = pypgo.mesh.read_obj("scan.obj")

comps = connected_components_by_edge(mesh)
print([len(c) for c in comps])                       # 各分量三角形数

clean = filter_small_components(mesh, min_triangles=100)   # 去碎屑
shells = split_components(clean)                            # 拆壳

c, r = minimum_bounding_sphere(clean, padding=0.05)  # 留 5% 边距
```

## 交叉链接

- 数据容器与 `take_elements` 语义：[../data.md](../data.md)
- 孤立顶点清理 / 更激进的修复：[../processing/surface.md](../processing/surface.md)
- 分量诊断在网格生成前的作用：[../processing/overview.md](../processing/overview.md)
