# `pypgo/tools/mesh/surface/_common.py` — 曲面 CLI 共享助手（私有）

> 源文件：`pypgo/tools/mesh/surface/_common.py`（40 行）。模块架构见 [overview.md](overview.md)。
>
> `_` 前缀私有模块：被 `cleanup.py` / `remesh.py` 复用，不进公共 API。

## func `add_surface_io_args(parser)`

给 argparse parser 加统一的 I/O 参数：`-i/--input-mesh`（必填，输入 OBJ）与 `-o/--output-mesh`（必填，输出 OBJ）。

## func `read_surface(path)` / `write_surface(path, surface)`

[`pypgo.mesh.read_obj` / `write_obj`](../../../mesh/data.md) 的直传别名，统一 CLI 内的网格 I/O 入口。

## func `write_json(path, payload)`

写 `indent=2` + 结尾换行的 JSON（cleanup 报告用）。

## func `average_triangle_edge_length(surface)`

```python
average_triangle_edge_length(surface: TriMeshData) -> float
```

输入网格全部三角形三条边长的算术平均：

$$\bar\ell=\frac{1}{3T}\sum_{t=1}^{T}\sum_{(i,j)\in\{(0,1),(1,2),(2,0)\}}\big\|\mathbf v_{t,i}-\mathbf v_{t,j}\big\|_2$$

注意按**三角形-边**计数（共享棱被两个三角形各算一次）。无三角形抛 `ValueError`。供 [`remesh.md`](remesh.md) 的 `--edge-length-scale` 把相对倍率换算成绝对目标棱长。

## 交叉链接

- 消费方：[cleanup.md](cleanup.md)、[remesh.md](remesh.md)
- 网格类型：[../../../mesh/data.md](../../../mesh/data.md)
