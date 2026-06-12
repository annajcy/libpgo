# `pypgo/animation/__init__.py` — 包公开面

> 源文件：`pypgo/animation/__init__.py`（20 行）。模块架构见 [overview.md](overview.md)。

纯 re-export，无逻辑。docstring 一句话定位：Alembic 导出、应力 VDB 导出、应力统计、Eigen 二进制 I/O。

## 导出表（12 个符号）

| 符号 | 来源 | 类别 | 文档 |
|---|---|---|---|
| `has_animation_io` | `abc.py` | Alembic 能力探测 | [abc.md](abc.md) |
| `read_u_file` | `abc.py` | 读 `.u` 位移矩阵 | [abc.md](abc.md) |
| `write_u_file` | `abc.py` | 写 `.u` 位移矩阵 | [abc.md](abc.md) |
| `AbcWriter` | `abc.py` | 增量 `.abc` 写出器 | [abc.md](abc.md) |
| `AnimationLoader` | `abc.py` | JSON 配置驱动的批量导出 | [abc.md](abc.md) |
| `dump_animation` | `abc.py` | 一步 JSON → `.abc` | [abc.md](abc.md) |
| `has_stress_vdb_export` | `stress_vdb.py` | OpenVDB 能力探测 | [stress_vdb.md](stress_vdb.md) |
| `StressFieldVDBExporter` | `stress_vdb.py` | von Mises → `.vdb` 序列 | [stress_vdb.md](stress_vdb.md) |
| `dump_stress_vdb` | `stress_vdb.py` | 一步导出（自动探测帧范围） | [stress_vdb.md](stress_vdb.md) |
| `FrameStressStats` | `stress_stats.py` | 单帧统计 dataclass | [stress_stats.md](stress_stats.md) |
| `StressFieldStats` | `stress_stats.py` | 序列统计 dataclass | [stress_stats.md](stress_stats.md) |
| `compute_stress_field_stats` | `stress_stats.py` | 统计计算入口 | [stress_stats.md](stress_stats.md) |

## 用法示例

```python
from pypgo.animation import read_u_file, compute_stress_field_stats

u = read_u_file("out/states/deform0010.u")     # (3n, 1) float64
stats = compute_stress_field_stats("out/stress")
print(stats.frames[-1].p99)
```
