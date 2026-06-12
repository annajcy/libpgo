# `pypgo/mesh/processing/surface.py` — 曲面质量诊断与修复

> 源文件：`pypgo/mesh/processing/surface.py`（323 行）。子包架构见 [overview.md](overview.md)。
>
> 质检判据在 **Python/NumPy** 侧实现（除自交检测走 C++ BVH 精确判定）；修复函数中 `remove_isolated_vertices` 是 C++ 网格库，其余全部委托 CGAL（编译期可选，运行期先过 `has_cgal_remesher()`，缺失抛 `RuntimeError`）。绑定见 `src/python/pypgo/mesh/geo/bindings.cpp:115-134`。

## class `QualityReport`（冻结 dataclass）

`check_surface_quality` 的结果。各字段判据（与实现逐条对应）：

| 字段 | 类型 | 判据（as-implemented） |
|---|---|---|
| `degenerate_tris` | `list[int]` | 面积 $A_f=\frac12\|(\mathbf v_1{-}\mathbf v_0)\times(\mathbf v_2{-}\mathbf v_0)\|\le\varepsilon_A$（默认 $\varepsilon_A=10^{-12}$）的三角形号 |
| `short_edges` | `list[(int,int)]` | 长度 $\|\mathbf v_a-\mathbf v_b\|<\tau$ 的无向边（$\tau\le0$ 时跳过该项检查） |
| `non_manifold_edges` | `list[(int,int)]` | 被 **3 个及以上**三角形共享的无向边（`count > 2`，48 行） |
| `flipped_tris` | `list[int]` | 定向不一致：同一**有向**边出现两次（相邻面环绕方向相同 ⟹ 法向相对翻转），记后出现的面（`_flipped_tris`，89-101 行） |
| `has_self_intersections` | `bool` | C++ `TriMeshBVTree::selfIntersectionExact`（精确谓词，BVH 加速；`geo/core.cpp:286-297`） |
| `is_clean` | `bool` | 以上五项全空/假 |

判据背后的几何事实：闭合定向流形上每条无向边恰被 2 个三角形以**相反方向**各使用一次——`count > 2` 破坏流形性，同向重复破坏定向一致性；边界边（`count == 1`）不在本报告内（由下文 `raw_surface_cleanup` 的 `boundary_or_nonmanifold_edges` 统计覆盖）。

## func `check_surface_quality(tri_data, short_edge_threshold=0.0, degenerate_area_threshold=1e-12) -> QualityReport`

一次遍历产出上表全部判据。复杂度：边记录与计数 $O(m)$（Python 循环），自交检测为 C++ BVH。`short_edge_threshold` 默认 0 即**默认不查短边**——需按网格尺度自行给阈值（如最短目标边长的一半）。

---

## 修复函数

## func `remove_isolated_vertices(tri_data) -> TriMeshData`

删除未被任何三角形引用的顶点并紧致重编号（C++ `Mesh::removeIsolatedVertices`）。无需 CGAL。是 [`take_elements`/`split_components`](../geometry/algorithms.md) 之后的标准收尾。

## class `MergeCloseVerticesResult`（冻结 dataclass）

| 字段 | 含义 |
|---|---|
| `surface` | 合并后的 `TriMeshData` |
| `merged_vertices` | 被合并掉的顶点数 |
| `eps` | 实际使用的距离阈值 |

## func `merge_close_vertices(tri_data, *, eps=None) -> MergeCloseVerticesResult`

合并距离小于 `eps` 的顶点并清理结果三角形（CGAL `mergeCloseVertices`）。`eps=None` 时传 $-1$ 进 C++，由 CGAL 侧**自动选取**阈值——返回值的 `eps` 字段报告实际取值。修复"顶点焊接失败"型缝隙（OBJ 导出常见病）。

## class `RawSurfaceCleanupStats` / `RawSurfaceCleanupReport` / `RawSurfaceCleanupResult`（冻结 dataclass）

`raw_surface_cleanup` 的过程账本：

- `RawSurfaceCleanupStats`（前/后快照）：`vertices`、`triangles`、`invalid_triangles`（退化三角形数）、`components`（连通分量数）、`boundary_or_nonmanifold_edges`（计数 $\ne2$ 的边数）、`is_manifold`；
- `RawSurfaceCleanupReport`：回显参数（`expected_components`、`short_edge_threshold`、`max_passes`、`max_collapses`）+ `before`/`after` 快照 + 结论位 `topology_preserved`（分量数与流形性未被破坏）与 `cleanup_complete`（退化三角形清零）+ 操作计数（`attempted/accepted_deletions`、`attempted/accepted_collapses`、`rejected_by_topology`、`rejected_by_invalid_count`）。`to_dict()` 输出展平字典（适合 JSON 日志）；
- `RawSurfaceCleanupResult`：`surface` + `report`。

## func `raw_surface_cleanup(tri_data, *, expected_components=None, short_edge_threshold=1e-5, max_passes=3, max_collapses=10000) -> RawSurfaceCleanupResult`

**保守的**退化三角形清理（CGAL `rawSurfaceCleanup`）：迭代尝试删除退化面/塌缩短边（阈值 `short_edge_threshold`），每步在 C++ 侧校验**拓扑不变**——会改变分量数或流形性的操作被拒绝（计入 `rejected_by_topology`）、会增加退化三角形的被拒绝（计入 `rejected_by_invalid_count`）。`expected_components=None` 时以输入网格自身的分量数为基准；`max_passes`/`max_collapses` 限制迭代量。用 `report.cleanup_complete` 判断是否还需人工干预。

---

## CGAL 重网格族

四个函数共同点：输入输出都是 `TriMeshData`，全在 CGAL 精确内核上运行，需 `has_cgal_remesher()`。

## func `cgal_smooth(tri_data, *, num_iter=10, sharp_angle=180.0) -> TriMeshData`

角度-面积联合平滑（CGAL `smoothMesh`，迭代均衡三角形内角与面积分布）。

| 参数 | 含义 |
|---|---|
| `num_iter` | 平滑迭代次数 |
| `sharp_angle` | 二面角（度）超过此值的边视为**特征边**冻结不动；默认 180 ⟹ 不保护任何边 |

## func `cgal_isotropic_remesh(tri_data, *, target_edge_length, num_iter=10, sharp_angle=180.0) -> TriMeshData`

各向同性重网格（CGAL `isotropicRemeshing`）：分裂/塌缩/翻边/松弛把边长逼向 `target_edge_length`。是给 `tet_mesher` 喂均匀输入、控制接触网格分辨率（[../../contact/surface.md](../../contact/surface.md)）的主力。`sharp_angle` 语义同上。

## func `cgal_repair_self_intersections(tri_data, *, method="autorefine") -> (TriMeshData, bool)`

自交修复（CGAL `repairSelfIntersections`）。返回 `(修复网格, all_fixed)`——务必检查第二项，修不净时 `False`。

| `method` | 策略 |
|---|---|
| `"autorefine"`（默认） | 在交线处细分并重组 |
| `"autorefine-only"` | 只细分不删除 |
| `"remove"` | 直接删除自交面片（留洞） |

非法名抛 `ValueError`。

## func `cgal_simplify(tri_data, *, target_ratio) -> TriMeshData`

边塌缩简化到原**边数**的 `target_ratio` 倍（CGAL `simplifyMeshGH`，Garland–Heckbert 二次误差度量；绑定层 `remesh_cgal.cpp:24-27` 固定选用 `"ptri"` 即 probabilistic-triangle 策略，`cgalInterface.cpp:1128-1156`）。

## 用法示例

```python
import pypgo

mesh = pypgo.mesh.read_obj("raw_scan.obj")
rep = pypgo.mesh.check_surface_quality(mesh, short_edge_threshold=1e-6)
print(rep.is_clean, len(rep.degenerate_tris), len(rep.non_manifold_edges))

if pypgo.mesh.has_cgal_remesher():
    mesh = pypgo.mesh.merge_close_vertices(mesh).surface
    res = pypgo.mesh.raw_surface_cleanup(mesh, short_edge_threshold=1e-6)
    print(res.report.to_dict())
    mesh, ok = pypgo.mesh.cgal_repair_self_intersections(res.surface)
    assert ok
    mesh = pypgo.mesh.cgal_isotropic_remesh(mesh, target_edge_length=0.02)

assert pypgo.mesh.check_surface_quality(mesh).is_clean
```

## 交叉链接

- 后端探测与体网格生成：[volume.md](volume.md)
- 连通分量工具（拆分/取外壳）：[../geometry/algorithms.md](../geometry/algorithms.md)
- 干净曲面的去向：[volume.md](volume.md)（`tet_mesher`/`cubic_mesher`）、[../../contact/surface.md](../../contact/surface.md)
