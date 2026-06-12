# `pypgo/tools/animation/abc_convert.py` — `pypgo-animation-convert` CLI

> 源文件：`pypgo/tools/animation/abc_convert.py`（26 行）。模块架构见 [overview.md](overview.md)。

[`pypgo.animation.dump_animation`](../../animation/abc.md) 的命令行薄壳：读 JSON 动画配置，把位移序列导出为 Alembic `.abc`（每个 mesh 一个文件）。

## 命令行用法

```bash
pypgo-animation-convert <config> [-o OUTPUT_FOLDER]
# 或
python -m pypgo.tools.animation.abc_convert anim.json -o out/abc
```

| 参数 | 含义 |
|---|---|
| `config`（位置参数） | JSON 动画配置路径（schema 见 [abc.md](../../animation/abc.md) 的 `AnimationLoader` 一节：`meshes[]` 的 `driving-mesh`/`sequence`/`sequence-type`/`sequence-range` 等） |
| `-o, --output-folder` | 输出目录；缺省取配置的 `output-folder` 字段，再缺省取配置文件所在目录 |

## 调用链

```
main(argv) ── argparse ──▶ pypgo.animation.dump_animation(config, output_folder)
                              └─▶ AnimationLoader.load → save_abc（C++ animationLoader.cpp）
```

要求 Alembic build（[`has_animation_io`](../../animation/abc.md)），否则 `RuntimeError`。配置内相对路径按配置文件目录解析。退出码 0；参数错误 2。

## 交叉链接

- 库函数与配置 schema：[../../animation/abc.md](../../animation/abc.md)
- 配套应力工具：[stress_vdb.md](stress_vdb.md)
