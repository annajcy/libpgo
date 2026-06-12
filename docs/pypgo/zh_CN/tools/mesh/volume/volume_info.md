# `pypgo/tools/mesh/volume/volume_info.py` — `pypgo-volume-info` CLI

> 源文件：`pypgo/tools/mesh/volume/volume_info.py`（20 行）。模块架构见 [overview.md](overview.md)。

只读检查工具：打印 `.veg` 体网格摘要（[`volume_mesh_info`](../../../mesh/processing/volume.md)）。本组里唯一不写任何文件的命令。

## 命令行用法

```bash
pypgo-volume-info <veg>
# 或 python -m pypgo.tools.mesh.volume.volume_info bunny.veg
```

| 参数 | 含义 |
|---|---|
| `veg`（位置参数） | 输入 `.veg` 文件 |

## 调用链

```
main(argv) ── argparse ──▶ volume.read_veg(veg) ──▶ print(mesh.volume_mesh_info(...))
```

摘要内容（顶点/单元数、单元类型、材料区域、包围盒等）由库函数定义，见 [../../../mesh/processing/volume.md](../../../mesh/processing/volume.md)。

## 交叉链接

- 库函数：[../../../mesh/processing/volume.md](../../../mesh/processing/volume.md)（`volume_mesh_info`）
- `.veg` 读取：[../../../mesh/volume/core.md](../../../mesh/volume/core.md)（`read_veg`）
