# `pypgo/tools/mesh/volume/tetgen_mesher.py` — `pypgo-tetgen-mesher` CLI

> 源文件：`pypgo/tools/mesh/volume/tetgen_mesher.py`（34 行）。模块架构见 [overview.md](overview.md)。

[`pypgo.mesh.tet_mesher`](../../../mesh/processing/volume.md) TetGen 后端的命令行薄壳：闭合 OBJ 曲面 → tet `.veg`。要求输入无自交、定向一致（TetGen 对脏网格不鲁棒——脏输入先走 [surface/cleanup](../surface/cleanup.md) 或改用 [fTetWild](ftetwild_mesher.md)）。

## 命令行用法

```bash
pypgo-tetgen-mesher <input_obj> <output_veg> \
    [--command pq1.414] [--E 1e6] [--nu 0.45] [--density 1000.0] \
    [--output-surface SURF_OBJ]
# 或 python -m pypgo.tools.mesh.volume.tetgen_mesher ...
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `input_obj` / `output_veg` | — | 输入闭合曲面 / 输出 `.veg` |
| `--command` | `pq1.414` | TetGen 命令串，原样传给后端（`p`=PLC 网格化、`q1.414`=半径-棱长比质量界；加 `aV` 限体积等，见 TetGen 手册） |
| `--E` / `--nu` / `--density` | `1e6` / `0.45` / `1000.0` | 单一材料参数（[_common.md](_common.md)） |
| `--output-surface` | 无 | 可选：抽 tet 网格表面写 OBJ |

## 调用链

```
main(argv) ── argparse ──▶ read_obj
          ──▶ mesh.tet_mesher(surface, backend="tetgen", config={"command": ...})
          ──▶ _common.write_volume_outputs
```

## 交叉链接

- 库函数与后端配置：[../../../mesh/processing/volume.md](../../../mesh/processing/volume.md)
- 鲁棒替代（容忍脏输入）：[ftetwild_mesher.md](ftetwild_mesher.md)
- 共享收尾：[_common.md](_common.md)
