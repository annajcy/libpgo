# `pypgo/mesh/geometry/__init__.py` — 公开面

> 源文件：`pypgo/mesh/geometry/__init__.py`（35 行，纯重导出）。子包架构见 [overview.md](overview.md)。

## 定位

把门面/嵌入（`core.py`）与纯 Python 算法（`algorithms.py`）重导出为扁平公开面。本子包由 `pypgo.mesh` 经 `__getattr__` **惰性加载**（`import pypgo.mesh` 不触发本文件；首次访问 `pypgo.mesh.geometry` 时才 import）。`SurfaceEmbedding` 额外被提升到 `pypgo.mesh` 顶层。

## 导出表

| 符号 | 来源文件 | 对象 |
|---|---|---|
| `TriMeshGeo` / `TetMeshGeo` / `CubicMeshGeo` | `core.py` | C++ 几何门面 |
| `BarycentricEmbedding` | `core.py` | 重心坐标嵌入 $x_s=\sum_i\beta_i x_{v_i}$ |
| `SurfaceEmbedding` | `core.py` | 体位移 → 表面网格 |
| `surface_to_volume_interpolation_matrix` | `core.py` | 嵌入矩阵 $W\in\mathbb R^{3m\times3n}$ |
| `triangle_component_ids` | `algorithms.py` | 三角形 → 分量标签 |
| `connected_components_by_edge` / `connected_components_by_vertex` | `algorithms.py` | 分量三角形索引列表 |
| `filter_small_components` | `algorithms.py` | 删小分量 |
| `get_outer_component` | `algorithms.py` | 取外层分量 |
| `split_components` | `algorithms.py` | 拆分为子网格 |
| `minimum_bounding_sphere` | `algorithms.py` | Welzl 最小包围球 |

`__all__`（21-35 行）与上表一致（按字母序）。
