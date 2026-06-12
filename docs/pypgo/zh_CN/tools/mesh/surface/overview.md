# `pypgo.tools.mesh.surface` — 曲面网格 CLI

> 包目录：`pypgo/tools/mesh/surface/`（5 个文件）。上级见 [../overview.md](../overview.md)；被封装的库见 [../../../mesh/processing/surface.md](../../../mesh/processing/surface.md)。

## 模块职责

OBJ 曲面的命令行处理三件套：**质检 → 清理 → 重网格化**。三个工具都是 [`pypgo.mesh`](../../../mesh/overview.md) 曲面处理函数的薄壳；共享的 I/O 与统计助手在 `_common.py`。

## 文件 ↔ 职责 主表

| 文件 | 命令 | 封装的库函数 | 文档 |
|---|---|---|---|
| `quality.py` | `pypgo-surface-quality` | `check_surface_quality` | [quality.md](quality.md) |
| `cleanup.py` | `pypgo-surface-cleanup` | `raw_surface_cleanup`（+可选 `merge_close_vertices`、`cgal_repair_self_intersections`、`remove_isolated_vertices`） | [cleanup.md](cleanup.md) |
| `remesh.py` | `pypgo-surface-remesh` | `cgal_isotropic_remesh` | [remesh.md](remesh.md) |
| `_common.py` | —（共享助手） | `read_obj`/`write_obj` + 平均棱长 | [_common.md](_common.md) |
| `__init__.py` | — | — | [\_\_init\_\_.md](__init__.md) |

## 典型流水线

```bash
# 1. 先看有什么问题（退化三角形、非流形棱、自交…）
pypgo-surface-quality raw.obj -o quality.json

# 2. 保守清理：可选先合并近邻顶点 + 修自交，再做短棱坍缩
pypgo-surface-cleanup -i raw.obj -o clean.obj --json cleanup_report.json \
    --merge-close-vertices --repair-self-intersections

# 3. 各向同性重网格化到目标棱长（绝对值或相对输入平均棱长的倍率）
pypgo-surface-remesh -i clean.obj -o final.obj -l 1.0

# 4. 喂给体网格化
pypgo-tetgen-mesher final.obj out.veg
```

## 交叉链接

- 各算法的数学与参数语义：[../../../mesh/processing/surface.md](../../../mesh/processing/surface.md)
- 网格类型：[../../../mesh/data.md](../../../mesh/data.md)（`TriMeshData`）
- 下游：[../volume/overview.md](../volume/overview.md)（体网格化）
