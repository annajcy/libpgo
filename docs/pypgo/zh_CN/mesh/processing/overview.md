# `pypgo.mesh.processing` — 网格生成、修复与质量诊断子包架构

> 包目录：`pypgo/mesh/processing/`（3 个文件）。上级架构见 [../overview.md](../overview.md)。

## 模块职责

仿真网格的入口质检与制造车间。FEM/IPC 对输入网格有硬性前提——闭合、流形、无自交、无退化元素——本子包提供从"原始扫描/建模产物"到"可仿真体网格"的全部工具：

1. **诊断**（`surface.py`）——`check_surface_quality` 一次性产出退化/短边/非流形/翻转/自交五项判据；
2. **修复**（`surface.py`）——孤立顶点清理、近顶点合并、保守退化清理（`raw_surface_cleanup`）、CGAL 平滑/重网格/自交修复/简化；
3. **生成**（`volume.py`）——闭合曲面 → 四面体网格（TetGen / fTetWild 双后端）或六面体网格（体素化）；
4. **汇总**（`volume.py`）——`volume_mesh_info` 报告顶点/元素数、总体积、质心。

## 理论流水线中的位置

```
原始 TriMeshData
   │ check_surface_quality ──> QualityReport（五判据）
   │ 不干净时：
   │   remove_isolated_vertices / merge_close_vertices / raw_surface_cleanup
   │   cgal_repair_self_intersections / cgal_smooth / cgal_isotropic_remesh / cgal_simplify
   ▼
干净闭合流形曲面
   │ tet_mesher（TetGen/fTetWild）        │ cubic_mesher（体素化）
   ▼                                      ▼
TetMeshData ── volume_mesh_info 核对 ── CubicMeshData
   │ + 材料（../volume/material.md）
   ▼
volume.VolumeMesh → fem.SimulationMesh
```

**为什么质检在前**：`cubic_mesher` 的 C++ 入口对自交/非流形/不闭合直接抛错（`triangleMeshVoxelizer.cpp` 的 `validateTriangleMesh`）；TetGen 对脏输入会失败或产出退化四面体；IPC 接触（[../../contact/overview.md](../../contact/overview.md)）的无穿插不变量要求初始面无自交。

## 可选后端

| 探测函数 | 后端 | 守门的函数 |
|---|---|---|
| `has_cgal_remesher()` | CGAL（编译期 `PYPGO_HAS_CGAL`） | `merge_close_vertices`、`raw_surface_cleanup`、全部 `cgal_*` |
| `has_tetwild()` | fTetWild（编译期 `PGO_TET_MESHER_HAS_TET_WILD`） | `tet_mesher(backend="tetwild")` |

后端缺失时对应函数抛 `RuntimeError`；TetGen 与体素化始终可用。

## 文件 ↔ 职责 主表

| 文件 | 对象 | 文档 |
|---|---|---|
| `surface.py` | `QualityReport` + `check_surface_quality`；修复函数族与结果 dataclass | [surface.md](surface.md) |
| `volume.py` | `tet_mesher` / `cubic_mesher` / `volume_mesh_info` / 后端探测 | [volume.md](volume.md) |
| `__init__.py` | 公开面 | [\_\_init\_\_.md](__init__.md) |

## C++ 引擎对应

| Python | C++ | 位置 |
|---|---|---|
| 自交检测 | `Mesh::TriMeshBVTree::selfIntersectionExact` | `src/core/mesh/boundingVolumeTree.h` |
| `merge_close_vertices` / `raw_surface_cleanup` | `CGALInterface::mergeCloseVertices` / `rawSurfaceCleanup` | `src/core/cgalInterface/cgalInterface.cpp` |
| `cgal_smooth` 等 | `CGALInterface::smoothMesh` / `isotropicRemeshing` / `repairSelfIntersections` / `simplifyMeshGH` | 同上（经 `src/python/pypgo/mesh/remesh_cgal.cpp`） |
| `tet_mesher` | `tet_mesher::generateTetgenMesh` / `generateTetwildMesh` | `src/core/volumetricMeshMeshing/tetgenBackend.cpp`、`tetwildBackend.cpp` |
| `cubic_mesher` | `cubic_mesher::createTriangleMeshCubicMesh` | `src/core/volumetricMeshMeshing/triangleMeshVoxelizer.cpp` |

## 贯穿示例

```python
import pypgo

raw = pypgo.mesh.read_obj("scan.obj")

report = pypgo.mesh.check_surface_quality(raw, short_edge_threshold=1e-5)
if not report.is_clean:
    if pypgo.mesh.has_cgal_remesher():
        raw, _ = pypgo.mesh.cgal_repair_self_intersections(raw)
        raw = pypgo.mesh.raw_surface_cleanup(raw).surface
        raw = pypgo.mesh.cgal_isotropic_remesh(raw, target_edge_length=0.01)

backend = "tetwild" if pypgo.mesh.has_tetwild() else "tetgen"
tets = pypgo.mesh.tet_mesher(raw, backend=backend)
print(pypgo.mesh.volume_mesh_info(tets))
```
