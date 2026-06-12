# `pypgo/tools/mesh/surface/quality.py` — `pypgo-surface-quality` CLI

> 源文件：`pypgo/tools/mesh/surface/quality.py`（45 行）。模块架构见 [overview.md](overview.md)。

[`pypgo.mesh.check_surface_quality`](../../../mesh/processing/surface.md) 的命令行薄壳：检查 OBJ 曲面质量，输出 JSON 报告（默认 stdout）。

## 命令行用法

```bash
pypgo-surface-quality <input_obj> \
    [--short-edge-threshold 0.0] [--degenerate-area-threshold 1e-12] \
    [-o OUTPUT_JSON]
# 或 python -m pypgo.tools.mesh.surface.quality mesh.obj
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `input_obj`（位置参数） | — | 输入 OBJ |
| `--short-edge-threshold` | `0.0` | 短棱判定阈值（0 = 不检短棱） |
| `--degenerate-area-threshold` | `1e-12` | 退化三角形面积阈值 |
| `-o, --output` | stdout | 输出 JSON 路径 |

## 输出 JSON 字段（`QualityReport` 的序列化）

| 键 | 含义 |
|---|---|
| `is_clean` | 全部检查通过 |
| `degenerate_tris` | 面积低于阈值的三角形数 |
| `short_edges` | 短于阈值的棱数 |
| `non_manifold_edges` | 非流形棱数（关联面数 ≠ 1, 2） |
| `flipped_tris` | 朝向翻转的三角形数 |
| `has_self_intersections` | 是否存在自交 |

各检查的精确定义见 [../../../mesh/processing/surface.md](../../../mesh/processing/surface.md)（`check_surface_quality` / `QualityReport`）。

## 调用链

```
main(argv) ── argparse ──▶ mesh.read_obj ──▶ mesh.check_surface_quality(...) ──▶ JSON
```

## 交叉链接

- 库函数：[../../../mesh/processing/surface.md](../../../mesh/processing/surface.md)
- 发现问题后的修复：[cleanup.md](cleanup.md)
