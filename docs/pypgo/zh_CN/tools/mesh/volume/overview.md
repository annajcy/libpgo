# `pypgo.tools.mesh.volume` — 体网格 CLI

> 包目录：`pypgo/tools/mesh/volume/`（7 个文件）。上级见 [../overview.md](../overview.md)；被封装的库见 [../../../mesh/processing/volume.md](../../../mesh/processing/volume.md)。

## 模块职责

把闭合 OBJ 曲面（或 Gmsh `.msh`）变成带单一 [`ENuMaterial`](../../../mesh/volume/material.md) 的 Vega `.veg` 体网格，供 [tools/sim](../../sim/overview.md) 的 `mesh.volume` 消费。三个网格化后端 + 一个格式转换 + 一个信息摘要；共享的材料参数与 `.veg` 写出在 `_common.py`。

## 文件 ↔ 职责 主表

| 文件 | 命令 | 后端 / 库函数 | 输出元素 | 文档 |
|---|---|---|---|---|
| `tetgen_mesher.py` | `pypgo-tetgen-mesher` | `tet_mesher(backend="tetgen")` | tet（4 点） | [tetgen_mesher.md](tetgen_mesher.md) |
| `ftetwild_mesher.py` | `pypgo-ftetwild-mesher` | `tet_mesher(backend="tetwild")` | tet（4 点） | [ftetwild_mesher.md](ftetwild_mesher.md) |
| `cubic_mesher.py` | `pypgo-cubic-mesher` | `cubic_mesher`（体素化） | cubic（8 点） | [cubic_mesher.md](cubic_mesher.md) |
| `msh_converter.py` | `pypgo-msh-converter` | `read_msh`（Gmsh → veg） | tet | [msh_converter.md](msh_converter.md) |
| `volume_info.py` | `pypgo-volume-info` | `volume_mesh_info` | —（只读） | [volume_info.md](volume_info.md) |
| `_common.py` | —（共享助手） | 材料参数 + `write_veg`/抽面 | — | [_common.md](_common.md) |
| `__init__.py` | — | — | [\_\_init\_\_.md](__init__.md) |

## 共同约定

四个生成/转换工具共享同一收尾（[`_common.write_volume_outputs`](_common.md)）：

- 材料参数 `--E`（默认 `1e6`）、`--nu`（默认 `0.45`）、`--density`（默认 `1000.0`），打成**单一材料区域**的 `VegFile`；
- 必有位置参数 `output_veg`；可选 `--output-surface` 同时抽取并写出体网格表面 OBJ（正好可作 tools/sim 的 `mesh.surface`）。

## 典型流水线

```bash
# tet 路线（仿真质量好）
pypgo-tetgen-mesher clean.obj bunny.veg --E 5e5 --nu 0.45 --output-surface bunny_surf.obj

# cubic 路线（体素网格，配 cubic-linear / cubic-tricubic-hermite formulation）
pypgo-cubic-mesher clean.obj bunny_cubic.veg --resolution 32 --output-surface bunny_surf.obj

# 外部 Gmsh 结果转入
pypgo-msh-converter model.msh model.veg

# 验货
pypgo-volume-info bunny.veg
```

## 交叉链接

- 网格化算法与配置语义：[../../../mesh/processing/volume.md](../../../mesh/processing/volume.md)
- `.veg`/材料模型：[../../../mesh/volume/material.md](../../../mesh/volume/material.md)、[../../../mesh/volume/core.md](../../../mesh/volume/core.md)
- 上游曲面准备：[../surface/overview.md](../surface/overview.md)
- 下游仿真：[../../sim/overview.md](../../sim/overview.md)
