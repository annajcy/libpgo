# `pypgo/tools/mesh/volume/_common.py` — 体网格 CLI 共享助手（私有）

> 源文件：`pypgo/tools/mesh/volume/_common.py`（26 行）。模块架构见 [overview.md](overview.md)。
>
> `_` 前缀私有模块：被四个网格化/转换入口复用，不进公共 API。

## func `add_material_args(parser)`

给 parser 加统一的材料参数：

| 参数 | 默认 | 含义 |
|---|---|---|
| `--E` | `1e6` | 杨氏模量 $E$（Pa） |
| `--nu` | `0.45` | 泊松比 $\nu$ |
| `--density` | `1000.0` | 密度 $\rho$（kg/m³） |

## func `material_from_args(args)`

```python
material_from_args(args) -> ENuMaterial
```

三个参数打成 [`ENuMaterial`](../../../mesh/volume/material.md)。

## func `write_volume_outputs(mesh_data, args)`

四个工具共同的收尾：

```
VegFile.from_single_material(mesh_data, material)   # 整网格单一材料区域
write_veg(args.output_veg, veg)
if args.output_surface:                              # 可选
    VolumeMesh.from_veg_file(veg).extract_surface_mesh() → write_obj
```

即：体网格 + 单一 $E,\nu,\rho$ 材料 → `.veg`；`--output-surface` 时再抽体网格表面写 OBJ（与体网格拓扑一致，可直接作 [tools/sim](../../sim/overview.md) 的 `mesh.surface` 嵌入面）。多材料区域的 `.veg` 不在 CLI 范围内——用库 API（[../../../mesh/volume/material.md](../../../mesh/volume/material.md)）。

## 交叉链接

- 消费方：[cubic_mesher.md](cubic_mesher.md)、[tetgen_mesher.md](tetgen_mesher.md)、[ftetwild_mesher.md](ftetwild_mesher.md)、[msh_converter.md](msh_converter.md)
- `VegFile`/`VolumeMesh`/`ENuMaterial`：[../../../mesh/volume/core.md](../../../mesh/volume/core.md)、[../../../mesh/volume/material.md](../../../mesh/volume/material.md)
