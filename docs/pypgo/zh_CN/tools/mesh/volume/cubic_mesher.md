# `pypgo/tools/mesh/volume/cubic_mesher.py` — `pypgo-cubic-mesher` CLI

> 源文件：`pypgo/tools/mesh/volume/cubic_mesher.py`（35 行）。模块架构见 [overview.md](overview.md)。

[`pypgo.mesh.cubic_mesher`](../../../mesh/processing/volume.md) 的命令行薄壳：把闭合 OBJ 曲面体素化成 cubic（8 点六面体）`.veg` 网格。产物配 [tools/sim](../../sim/overview.md) 的 `cubic` 命令（formulation `cubic-linear` 或 `cubic-tricubic-hermite`）。

## 命令行用法

```bash
pypgo-cubic-mesher <input_obj> <output_veg> --resolution N \
    [--E 1e6] [--nu 0.45] [--density 1000.0] [--output-surface SURF_OBJ]
# 或 python -m pypgo.tools.mesh.volume.cubic_mesher ...
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `input_obj` | — | 输入闭合 OBJ 曲面 |
| `output_veg` | — | 输出 `.veg` |
| `--resolution` | 必填 | **最短轴**方向的体素数 |
| `--E` / `--nu` / `--density` | `1e6` / `0.45` / `1000.0` | 单一材料参数（见 [_common.md](_common.md)） |
| `--output-surface` | 无 | 可选：抽体素网格表面写 OBJ |

## 调用链

```
main(argv) ── argparse ──▶ read_obj
          ──▶ mesh.cubic_mesher(surface, resolution, E, nu, density)
          ──▶ _common.write_volume_outputs（VegFile 单材料 → write_veg [+ 表面 OBJ]）
```

注意 `cubic_mesher` 库函数直接接收材料参数（与 tet 后端不同），但 CLI 收尾仍统一走 `write_volume_outputs` 重新打材料。体素化语义（内外判定、分辨率换算）见库文档。

## 交叉链接

- 库函数：[../../../mesh/processing/volume.md](../../../mesh/processing/volume.md)
- 共享收尾：[_common.md](_common.md)
- tet 替代路线：[tetgen_mesher.md](tetgen_mesher.md)、[ftetwild_mesher.md](ftetwild_mesher.md)
