# `pypgo.tools.animation` — 动画后处理 CLI

> 包目录：`pypgo/tools/animation/`（3 个文件）。工具层架构见 [../overview.md](../overview.md)；被封装的库见 [../../animation/overview.md](../../animation/overview.md)。

## 模块职责

[`pypgo.animation`](../../animation/overview.md) 两条导出管线的命令行封装。两个工具都是"argparse + 一次库函数调用"的薄壳，无自有逻辑。

## 文件 ↔ 职责 主表

| 文件 | 命令 | 封装的库函数 | 文档 |
|---|---|---|---|
| `abc_convert.py` | `pypgo-animation-convert` | [`dump_animation`](../../animation/abc.md) | [abc_convert.md](abc_convert.md) |
| `stress_vdb.py` | `pypgo-stress-vdb` | [`dump_stress_vdb`](../../animation/stress_vdb.md) | [stress_vdb.md](stress_vdb.md) |
| `__init__.py` | — | — | [\_\_init\_\_.md](__init__.md) |

## 流水线位置

```
tools/sim 输出（states/*.u, stress/*.json）
   ├── pypgo-animation-convert  anim.json → 每 mesh 一个 .abc
   └── pypgo-stress-vdb         veg + sim 输出 → vonMises%04d.vdb
```

两者分别要求 Alembic / OpenVDB build（运行时经 [`has_animation_io`](../../animation/abc.md) / [`has_stress_vdb_export`](../../animation/stress_vdb.md) 检查）。

## 贯穿示例

```bash
# 1. 跑一个 dynamic 场景并写状态/应力
pypgo-sim-tet-dynamic --config scene.json --output-dir out/run

# 2. 应力体积序列（Houdini/Blender volume）
pypgo-stress-vdb assets/bunny.veg out/run out/run/vdb

# 3. 自定义 JSON 序列 → Alembic
pypgo-animation-convert anim_config.json -o out/abc
```

## 交叉链接

- 输入数据的生产者：[../sim/overview.md](../sim/overview.md)（`write_states`/`write_stress`/`write_abc`）
- 库层细节（文件格式、splat 算法）：[../../animation/overview.md](../../animation/overview.md)
