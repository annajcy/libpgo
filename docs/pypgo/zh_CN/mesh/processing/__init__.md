# `pypgo/mesh/processing/__init__.py` — 公开面

> 源文件：`pypgo/mesh/processing/__init__.py`（47 行，纯重导出）。子包架构见 [overview.md](overview.md)。

## 定位

把曲面诊断/修复（`surface.py`）与体网格生成（`volume.py`）重导出为扁平公开面。注意 `surface.py` 依赖 `volume.py` 的 `has_cgal_remesher`，故本文件先导 `volume` 再导 `surface`。全部符号同时被提升到 `pypgo.mesh` 顶层（[../\_\_init\_\_.md](../__init__.md)）。

## 导出表

| 符号 | 来源文件 | 对象 |
|---|---|---|
| `QualityReport` | `surface.py` | 五判据质检报告 |
| `check_surface_quality` | `surface.py` | 质检入口 |
| `remove_isolated_vertices` | `surface.py` | 删未引用顶点 |
| `merge_close_vertices` / `MergeCloseVerticesResult` | `surface.py` | 近顶点合并（CGAL） |
| `raw_surface_cleanup` / `RawSurfaceCleanupResult` / `RawSurfaceCleanupReport` / `RawSurfaceCleanupStats` | `surface.py` | 保守退化清理（CGAL） |
| `cgal_smooth` / `cgal_isotropic_remesh` / `cgal_repair_self_intersections` / `cgal_simplify` | `surface.py` | CGAL 重网格族 |
| `tet_mesher` | `volume.py` | 四面体化（TetGen/fTetWild） |
| `cubic_mesher` | `volume.py` | 体素化六面体化 |
| `volume_mesh_info` / `VolumetricMeshInfo` | `volume.py` | 体网格几何汇总 |
| `has_tetwild` / `has_cgal_remesher` | `volume.py` | 可选后端探测 |

`__all__`（27-47 行）与上表一致（按字母序）。
