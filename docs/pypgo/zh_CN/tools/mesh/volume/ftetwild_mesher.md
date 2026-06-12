# `pypgo/tools/mesh/volume/ftetwild_mesher.py` — `pypgo-ftetwild-mesher` CLI

> 源文件：`pypgo/tools/mesh/volume/ftetwild_mesher.py`（42 行）。模块架构见 [overview.md](overview.md)。

[`pypgo.mesh.tet_mesher`](../../../mesh/processing/volume.md) fTetWild 后端的命令行薄壳：闭合 OBJ 曲面 → tet `.veg`。fTetWild 对脏输入（自交、小缝）鲁棒，但输出表面是输入的 $\varepsilon$-近似（不保原顶点），代价是更慢。

## 命令行用法

```bash
pypgo-ftetwild-mesher <input_obj> <output_veg> \
    [--lr 0.05] [--la L_ABS] [--epsr 0.001] [--stop-energy 10.0] [--max-threads 0] \
    [--E 1e6] [--nu 0.45] [--density 1000.0] [--output-surface SURF_OBJ]
# 或 python -m pypgo.tools.mesh.volume.ftetwild_mesher ...
```

| 参数 | 默认 | 含义（fTetWild 语义） |
|---|---|---|
| `--lr` | `0.05` | 相对理想棱长（× 包围盒对角线） |
| `--la` | 无 | 绝对理想棱长；给出时**追加**进 config（同时存在时由后端裁决） |
| `--epsr` | `0.001` | 相对包络容差 $\varepsilon_r$（输出面偏离输入面的允许量，× 包围盒对角线） |
| `--stop-energy` | `10.0` | 优化停止能量（AMIPS；越小质量越好、越慢） |
| `--max-threads` | `0` | 线程数上限；0 = 后端默认 |
| `--E` / `--nu` / `--density` | `1e6` / `0.45` / `1000.0` | 单一材料参数（[_common.md](_common.md)） |
| `--output-surface` | 无 | 可选：抽 tet 网格表面写 OBJ |

## 调用链

```
main(argv) ── argparse ──▶ config = {lr, epsr, stop_energy, max_threads [, la]}
          ──▶ read_obj ──▶ mesh.tet_mesher(surface, backend="tetwild", config=config)
          ──▶ _common.write_volume_outputs
```

## 交叉链接

- 库函数与后端配置全表：[../../../mesh/processing/volume.md](../../../mesh/processing/volume.md)
- 干净输入的快速替代：[tetgen_mesher.md](tetgen_mesher.md)
- 共享收尾：[_common.md](_common.md)
