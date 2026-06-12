# `pypgo/tools/mesh/surface/remesh.py` — `pypgo-surface-remesh` CLI

> 源文件：`pypgo/tools/mesh/surface/remesh.py`（50 行）。模块架构见 [overview.md](overview.md)。

[`pypgo.mesh.cgal_isotropic_remesh`](../../../mesh/processing/surface.md) 的命令行薄壳：CGAL 各向同性重网格化到目标棱长。

## 命令行用法

```bash
pypgo-surface-remesh -i in.obj -o out.obj \
    ( --target-edge-length L | -l/--edge-length-scale S ) \
    [--iterations 10] [-s/--sharp-edge-angle 180.0]
# 或 python -m pypgo.tools.mesh.surface.remesh ...
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `-i/--input-mesh`、`-o/--output-mesh` | 必填 | 输入/输出 OBJ |
| `--target-edge-length` | 互斥二选一 | 绝对目标棱长 $\ell^*$ |
| `-l, --edge-length-scale` | 互斥二选一 | 相对倍率 $s$：$\ell^*=s\cdot\bar\ell$，$\bar\ell$ 为输入的[平均三角形棱长](_common.md)（`-l 1.0` ≈ 保持密度，只改善均匀性） |
| `--iterations` | `10` | 重网格化迭代次数（`num_iter`） |
| `-s, --sharp-edge-angle` | `180.0` | 特征棱二面角阈值（度，`sharp_angle`）；180 = 不保特征 |

两种棱长给法**必须且只能给一个**（argparse 互斥组，`required=True`）。

## 调用链

```
main(argv) ── argparse ──▶ read_obj ──▶ [average_triangle_edge_length × scale]
          ──▶ mesh.cgal_isotropic_remesh(surface, target_edge_length, num_iter, sharp_angle)
          ──▶ write_obj
```

## 交叉链接

- 库函数（CGAL 算法语义）：[../../../mesh/processing/surface.md](../../../mesh/processing/surface.md)
- 平均棱长定义：[_common.md](_common.md)
- 通常先做：[cleanup.md](cleanup.md)
