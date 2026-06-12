# `pypgo/mesh/__init__.py` — 公开面、OBJ I/O 与形状工厂

> 源文件：`pypgo/mesh/__init__.py`（132 行）。模块架构见 [overview.md](overview.md)。

## 定位

除重导出外，本文件还**直接定义**了 6 个模块级函数：OBJ 读写（2 个）和程序化形状工厂（4 个），全部委托 C++（`src/python/pypgo/mesh/geo/core.cpp` 的 `read_obj`/`write_obj`/`create_*_mesh`，C++ 形状生成在 `src/core/mesh/createTriMesh.h`）。子模块 `geometry`、`volume` 经 `__getattr__` 惰性加载（104-109 行）。

## 本文件定义的函数

### `read_obj(path) -> TriMeshData`

读 OBJ 曲面网格。C++ 侧 `Mesh::TriMeshGeo::load` 解析后转为 `MeshData<3>`，失败抛 `RuntimeError`。

### `write_obj(path, surface_data) -> None`

写三角网格到 OBJ。`surface_data` 必须是 `TriMeshData`（否则 `TypeError`）。

### `create_box(*, bmin, bmax) -> TriMeshData`

轴对齐盒曲面。`bmin`/`bmax` 为长度 3 的序列。

### `create_sphere(*, radius, axis_subdiv, height_subdiv) -> TriMeshData`

经纬细分球面。C++ 侧校验 `radius > 0`、细分数 `>= 2`。

### `create_cylinder(*, radius, height, axis_subdiv, height_subdiv) -> TriMeshData`

带端盖圆柱。C++ 侧校验 `radius, height > 0`、`axis_subdiv >= 3`、`height_subdiv >= 1`。

### `create_torus(*, radial_res, tubular_res, radius, thickness) -> TriMeshData`

圆环面。C++ 侧校验分辨率 `>= 3`、`radius, thickness > 0`。

## 重导出表

| 符号 | 来源 | 文档 |
|---|---|---|
| `MeshDataType`, `TriMeshData`, `TetMeshData`, `CubicMeshData` | `data.py` | [data.md](data.md) |
| `SurfaceEmbedding` | `geometry/core.py` | [geometry/core.md](geometry/core.md) |
| `set_backend`, `reset_backend`, `get_backend`, `plot_surface`, `plot_volume_surface`, `plot_points_on_mesh`, `to_pyvista_surface`, `to_pyvista_volume`, `write_points_obj` | `visualize.py` | [visualize.md](visualize.md) |
| `volume_mesh_info`, `tet_mesher`, `cubic_mesher`, `has_tetwild`, `has_cgal_remesher`, `VolumetricMeshInfo` | `processing/volume.py` | [processing/volume.md](processing/volume.md) |
| `check_surface_quality`, `QualityReport`, `remove_isolated_vertices`, `merge_close_vertices`, `raw_surface_cleanup`, `cgal_smooth`, `cgal_isotropic_remesh`, `cgal_repair_self_intersections`, `cgal_simplify` 及结果 dataclass | `processing/surface.py` | [processing/surface.md](processing/surface.md) |
| `geometry`, `volume`（子模块，惰性） | — | [geometry/overview.md](geometry/overview.md) · [volume/overview.md](volume/overview.md) |

注意：`__all__`（30-65 行）包含上述全部符号；`set_backend`/`reset_backend`/`get_backend`/`to_pyvista_*`/`plot_surface`/`plot_volume_surface` 虽被导入但部分未列入 `__all__`，仍可经属性访问。`processing` 的导入放在文件**末尾**（112-132 行）以避开与 `data.py`→`processing` 间的循环依赖。
