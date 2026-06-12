# `pypgo/tools/mesh/surface/cleanup.py` — `pypgo-surface-cleanup` CLI

> 源文件：`pypgo/tools/mesh/surface/cleanup.py`（87 行）。模块架构见 [overview.md](overview.md)。

OBJ 曲面的保守清理：可选预处理（合并近邻顶点、修自交）→ 短棱坍缩主循环（[`raw_surface_cleanup`](../../../mesh/processing/surface.md)）→ 可选去孤立顶点，并写 JSON 报告。

## 命令行用法

```bash
pypgo-surface-cleanup -i raw.obj -o clean.obj --json report.json \
    [--expected-components N] [--short-edge-threshold 1e-5] \
    [--max-passes 3] [--max-collapses 10000] \
    [--merge-close-vertices [--eps EPS]] \
    [--repair-self-intersections [--repair-method autorefine|autorefine-only|remove]] \
    [--remove-isolated-vertices] [--dry-run]
# 或 python -m pypgo.tools.mesh.surface.cleanup ...
```

| 参数 | 默认 | 含义（对应库参数） |
|---|---|---|
| `-i/--input-mesh`、`-o/--output-mesh` | 必填 | 输入/输出 OBJ |
| `--json` | 必填 | 输出 JSON 清理报告 |
| `--expected-components` | 输入的连通分量数 | 棱连通分量数预期（清理不应改变拓扑分量） |
| `--short-edge-threshold` | `1e-5` | 短棱判定阈值 |
| `--max-passes` | `3` | 坍缩遍数上限 |
| `--max-collapses` | `10000` | 总坍缩次数上限 |
| `--merge-close-vertices` | 关 | 预处理：`merge_close_vertices(surface, eps=--eps)` |
| `--eps` | 库默认 | 合并距离 |
| `--repair-self-intersections` | 关 | 预处理：`cgal_repair_self_intersections(surface, method=...)` |
| `--repair-method` | `autorefine` | `autorefine` / `autorefine-only` / `remove` |
| `--remove-isolated-vertices` | 关 | 后处理：`remove_isolated_vertices` |
| `--dry-run` | 关 | 只写报告、不写输出网格 |

## 处理流程（as-implemented）

```
read_obj ─▶ [merge_close_vertices] ─▶ [cgal_repair_self_intersections]
        ─▶ raw_surface_cleanup（主循环） ─▶ [remove_isolated_vertices]
        ─▶ write_obj（非 dry-run） + JSON 报告
```

报告 = `result.report.to_dict()` 再合入：`input`/`output`/`dry_run`、两个预处理段（`merge_close_vertices`：`enabled`/`merged_vertices`/`eps`；`repair_self_intersections`：`enabled`/`method`/`all_fixed`）与 `remove_isolated_vertices.enabled`。

各步算法语义（坍缩准则、autorefine 与 remove 的差异、报告字段全表）见 [../../../mesh/processing/surface.md](../../../mesh/processing/surface.md)。

## 交叉链接

- 库函数：[../../../mesh/processing/surface.md](../../../mesh/processing/surface.md)
- 共享助手：[_common.md](_common.md)
- 前置质检：[quality.md](quality.md)；后续重网格化：[remesh.md](remesh.md)
