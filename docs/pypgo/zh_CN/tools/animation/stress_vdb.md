# `pypgo/tools/animation/stress_vdb.py` — `pypgo-stress-vdb` CLI

> 源文件：`pypgo/tools/animation/stress_vdb.py`（40 行）。模块架构见 [overview.md](overview.md)。

[`pypgo.animation.dump_stress_vdb`](../../animation/stress_vdb.md) 的命令行薄壳：把仿真输出的逐 tet von Mises 应力 splat 成逐帧 OpenVDB 体积序列。

## 命令行用法

```bash
pypgo-stress-vdb <veg_path> <sim_output> <output_dir> \
    [--prefix vonMises] [--voxel-size 0.0] [--frame-start 0] [--frame-end -1]
# 或
python -m pypgo.tools.animation.stress_vdb assets/bunny.veg out/run out/run/vdb
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `veg_path` | — | 静止构型 tet 网格（`.veg`） |
| `sim_output` | — | 仿真输出目录，须含 `states/`（`deform%04d.u`）与 `stress/`（`von_mises%04d.json`）——即 [tools/sim](../sim/overview.md) 开 `write_states`+`write_stress` 的布局 |
| `output_dir` | — | `.vdb` 输出目录 |
| `--prefix` | `vonMises` | 输出文件名前缀（`{prefix}%04d.vdb`） |
| `--voxel-size` | `0.0` | 体素尺寸；$\le 0$ 自动取静止网格平均棱长的一半 |
| `--frame-start` | `0` | 起始帧（含） |
| `--frame-end` | `-1` | 结束帧（不含）；`-1` 自动探测到第一个缺帧为止 |

成功时打印 `Wrote N frame(s) to <output_dir>` 并返回 0。

## 调用链

```
main(argv) ── argparse ──▶ pypgo.animation.dump_stress_vdb(...)
                              └─▶ StressFieldVDBExporter（C++ stressFieldVDBExporter.cpp：
                                  逐 tet 包围盒体素化 + point-in-tet + max 合成 → FOG_VOLUME .vdb）
```

要求 OpenVDB build（[`has_stress_vdb_export`](../../animation/stress_vdb.md)），否则 `RuntimeError`。

## 交叉链接

- 库函数与 splat 算法细节：[../../animation/stress_vdb.md](../../animation/stress_vdb.md)
- 输入数据的生产者：[../sim/_runners.md](../sim/_runners.md)
- 应力统计（无需 OpenVDB）：[../../animation/stress_stats.md](../../animation/stress_stats.md)
