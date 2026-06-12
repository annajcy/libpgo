# `pypgo/tools/mesh/volume/msh_converter.py` — `pypgo-msh-converter` CLI

> 源文件：`pypgo/tools/mesh/volume/msh_converter.py`（33 行）。模块架构见 [overview.md](overview.md)。

格式转换薄壳：Gmsh `.msh` tet 网格 → Vega `.veg`（[`read_msh`](../../../mesh/volume/core.md) + 单一材料烘焙）。用于把外部 Gmsh 流水线的结果接入 pypgo。

## 命令行用法

```bash
pypgo-msh-converter <input_msh> <output_veg> \
    [--E 1e6] [--nu 0.45] [--density 1000.0] [--output-surface SURF_OBJ]
# 或 python -m pypgo.tools.mesh.volume.msh_converter ...
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `input_msh` | — | 输入 Gmsh `.msh`（tet 单元） |
| `output_veg` | — | 输出 `.veg` |
| `--E` / `--nu` / `--density` | `1e6` / `0.45` / `1000.0` | 单一材料参数（[_common.md](_common.md)）——`.msh` 不携带材料，必须在此指定 |
| `--output-surface` | 无 | 可选：抽体网格表面写 OBJ |

## 调用链

```
main(argv) ── argparse ──▶ volume.read_msh(input_msh)   # → VolumeMeshData
          ──▶ _common.write_volume_outputs（单材料 VegFile → write_veg [+ 表面 OBJ]）
```

无网格化步骤——纯 I/O 转换。

## 交叉链接

- `read_msh` 与 `.veg` 数据模型：[../../../mesh/volume/core.md](../../../mesh/volume/core.md)
- 材料：[../../../mesh/volume/material.md](../../../mesh/volume/material.md)
- 共享收尾：[_common.md](_common.md)
- 转换后验货：[volume_info.md](volume_info.md)
