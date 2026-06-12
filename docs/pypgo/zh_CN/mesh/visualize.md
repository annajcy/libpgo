# `pypgo/mesh/visualize.py` — PyVista 可视化门面

> 源文件：`pypgo/mesh/visualize.py`（287 行，纯 Python，依赖 `pyvista`）。模块架构见 [overview.md](overview.md)。
>
> 本模块不做任何数值计算：职责是 (1) 管理一个**模块级默认后端**状态，(2) 把 pypgo 网格容器转成 PyVista 数据结构，(3) 提供三个固定布局的绘图函数。所有 `plot_*` 均从 `pypgo.mesh` 顶层可达。

## 后端管理

模块状态机（12-19 行）：pypgo 后端名 → PyVista `jupyter_backend` 的映射

| pypgo 后端 | PyVista `jupyter_backend` | 含义 |
|---|---|---|
| `"jupyter"`（默认） | `trame` | notebook 内交互视图 |
| `"static"` | `static` | 渲染为静态图片（适合 CI / 导出） |
| `"none"` | `none` | 不在 notebook 内嵌（弹独立窗口） |

每个 `plot_*` 都接受 `backend=` 参数做**单次覆盖**；缺省时用模块默认（`_show_plotter`，46-48 行）。

### `set_backend(backend)`

设置模块默认后端。非法名抛 `ValueError`（合法集 `{"jupyter", "static", "none"}`，大小写不敏感）。

### `reset_backend()`

重置为 `"jupyter"`。

### `get_backend() -> str`

返回当前模块默认后端名。

---

## 网格 → PyVista 转换

### `to_pyvista_surface(surface_data) -> pv.PolyData`

`TriMeshData` → `PolyData`。PyVista 的面数组采用"前缀计数"扁平格式，对纯三角网格即每面 4 个整数：

```
faces = [3, v0, v1, v2,  3, v0, v1, v2, ...]
```

实现（85-91 行）：`column_stack([full(m, 3), elements]).ravel()`。非 `TriMeshData` 抛 `TypeError`。

### `to_pyvista_volume(volume_data) -> pv.UnstructuredGrid`

`TetMeshData` / `CubicMeshData` → `UnstructuredGrid`。按元素宽度选择 cell 类型（101-108 行）：

| 元素宽度 | VTK cell 类型 |
|---|---|
| 4 | `CellType.TETRA` |
| 8 | `CellType.HEXAHEDRON` |

cells 数组同样是前缀计数格式 `[K, i0, ..., i_{K-1}, ...]`，cell_types 为每元素一个 `uint8`。六面体顶点序与 [data.md](data.md) 的 Vega/VTK 约定一致，无需重排。

---

## 绘图函数

### `plot_surface(meshes, *, titles=None, show_edges=True, colors=None, window_size=(900, 360), backend=None)`

把一个或多个 `TriMeshData` 渲染为 $1\times n$ 并排子图。单个网格可直接传（自动包成列表）。

| 参数 | 含义 | 默认 |
|---|---|---|
| `titles` | 每子图左上角文字 | 无 |
| `show_edges` | 画线框 | `True` |
| `colors` | 每网格颜色（循环取用） | `"lightgray"` |
| `window_size` | 像素尺寸 | `(900, 360)` |
| `backend` | 单次后端覆盖 | 模块默认 |

每个子图设置等轴测视角（`view_isometric` + zoom 1.2）。

### `plot_volume_surface(meshes, *, titles=None, show_edges=True, colors=None, scalars=None, scalar_bar_titles=None, clims=None, window_size=(900, 360), backend=None)`

渲染体网格（`TetMeshData`/`CubicMeshData`）的**抽取表面**，可叠加逐元素标量场。

- `scalars`：`None`（纯色）或每网格一个 `(num_elements,)` 数组（单网格可直接传数组）。形状不符抛 `ValueError`（205-208 行）。标量作为 **cell data** 写入 `UnstructuredGrid` 并以 `preference="cell"` 上色——适合可视化逐元素量（如能量密度、材料参数，见 [../fem/fields.md](../fem/fields.md) 的 `ElementwiseField`）。
- `scalar_bar_titles`：每子图色条标题。
- `clims`：每子图 `(lo, hi)` 或 `None`（自动）。固定色域便于多面板对比，防止小幅值场被大幅值场"洗白"（docstring 173-177 行）。长度不符抛 `ValueError`。
- 无标量时表面经 `extract_surface(algorithm="dataset_surface")` 抽取后纯色渲染；有标量时直接渲染网格体以保留 cell data。

### `plot_points_on_mesh(mesh, points, *, title=None, mesh_color="lightgray", mesh_opacity=0.3, point_color="red", point_size=10, render_points_as_spheres=True, show_edges=False, window_size=(900, 650), backend=None)`

单视图：半透明网格 + 叠加点云。典型用途：检查固定点/附着点选取（如 [../energy/attachment.md](../energy/attachment.md) 的顶点集合、`dragon-surface-fixed.txt` 这类资产）。

- `mesh` 可为三类容器任意一种；体网格走抽取表面。
- `points` 必须 `(n,3)`（`_normalize_points` 校验，73-77 行）。

### `write_points_obj(path, points) -> None`

把 `(n,3)` 点云写成只含 `v x y z` 行的 OBJ（无面片）。自动创建父目录。与 `plot_points_on_mesh` 互补——用于把选点结果持久化或交给外部工具检查。

## 用法示例

```python
import pypgo

pypgo.mesh.set_backend("static")          # CI / 脚本环境出静态图

box = pypgo.mesh.create_box(bmin=[0,0,0], bmax=[1,1,1])
tets = pypgo.mesh.tet_mesher(box, backend="tetgen")

pypgo.mesh.plot_surface(box, titles=["box"])
pypgo.mesh.plot_volume_surface(
    [tets, tets],
    scalars=[None, tets.vertices[tets.elements].mean(axis=1)[:, 1]],  # 元素心 y 坐标
    scalar_bar_titles=[None, "centroid y"],
    clims=[None, (0.0, 1.0)],
)
pypgo.mesh.plot_points_on_mesh(tets, tets.vertices[:8], point_color="blue",
                               backend="jupyter")   # 单次覆盖回交互
```

## 交叉链接

- 数据容器：[data.md](data.md)
- 体网格表面抽取（带材料的 `VolumeMesh` 版本）：[volume/core.md](volume/core.md)（`extract_surface_mesh`）
- 逐元素标量的来源：[../fem/fields.md](../fem/fields.md)、[../fem/energy.md](../fem/energy.md)
