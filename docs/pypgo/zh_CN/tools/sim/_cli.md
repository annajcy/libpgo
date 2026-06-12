# `pypgo/tools/sim/_cli.py` — 共享 argparse 层（私有）

> 源文件：`pypgo/tools/sim/_cli.py`（103 行）。模块架构见 [overview.md](overview.md)。
>
> 6 个入口 stub 的公共骨架：建 parser、把 flags 转成点路径覆盖、串起 `load_config → build_scene → run_*`。

## func `build_parser(*, prog, mesh_type, mode)`

按 (mesh_type, mode) 生成 parser。flag 集（全部默认 `None` = "未给出，不覆盖 JSON"）：

| flag | 适用 | 对应配置点路径 |
|---|---|---|
| `--config PATH` | 全部 | （JSON 文件本身） |
| `--volume PATH` | tet/cubic | `mesh.volume` |
| `--surface PATH` | 全部 | `mesh.surface` |
| `--formulation {auto,tet-linear,cubic-linear,cubic-tricubic-hermite}` | tet/cubic | `mesh.formulation` |
| `--output-dir PATH` | 全部 | `output.directory` |
| `--write-surfaces` | 全部 | `output.write_surfaces` |
| `--gravity GX GY GZ` | 全部 | `loads.gravity` |
| `--solver-max-iterations N` | 全部 | `solver.max_iterations` |
| `--solver-gradient-tolerance T` | 全部 | `solver.gradient_tolerance` |
| `--timestep DT` | dynamic | `dynamic.timestep` |
| `--num-steps N` | dynamic | `dynamic.num_steps` |
| `--integrator {implicit_euler,trbdf2}` | dynamic | `dynamic.integrator` |
| `--damping MASS STIFFNESS` | dynamic | `dynamic.damping` |

注意 `--write-surfaces` 用 `action="store_true", default=None`——不给时保持 `None`（不覆盖 JSON），给了才覆盖为 True。其余 output flag（write_states/stress/abc、dump_interval）**没有 CLI 开关**，只能写在 JSON。

## func `_overrides_from_args(args, *, mesh_type, mode) -> dict`

把非 `None` 的 flag 收成 `{点路径: 值}`。路径类 flag 先 `Path(value).resolve()`（**按 CWD 绝对化**）再放入——区别于 JSON 内路径按 JSON 目录解析（见 [_config.md](_config.md)）。

## func `run_cli(*, mesh_type, mode, prog, argv=None) -> int`

入口骨架：

```
parse_args ─▶ load_config(mesh_type, mode, json_path=--config, overrides=flags)
           ─▶ build_scene(cfg) ─▶ run_static / run_dynamic
           ─▶ 打印 "wrote .../summary.json" + 一行结果
```

- 任何 [`ConfigError`](_config.md)（含场景装配期的，如网格读不出、选择器空集）→ `parser.error`：消息进 stderr，**退出码 2**；
- 成功返回 0；结果行：dynamic 打帧数与最终时间，static 打 `converged`/`iterations`。

优先级链条全貌：**内置默认 < JSON（`--config`） < CLI flags**——合并发生在 dict 层（`_set_dotted`），因此 CLI 值同样经过 `_config` 的全部校验。

## 用法示例

```bash
# JSON 给场景，CLI 临时改步数与输出目录（覆盖 JSON 同名字段）
pypgo-sim-cubic-dynamic \
    --config examples/sim_configs/cubic_dynamic_bunny_ipc.json \
    --num-steps 20 --output-dir /tmp/quick

# 不用 JSON，纯 flags 跑一个最小 static（output.directory 必给）
pypgo-sim-tet-static --volume box.veg --surface box.obj \
    --gravity 0 0 -9.8 --output-dir /tmp/box
```

注意 `constraints`/`contact` 等结构化字段没有 CLI flag，只能经 JSON 给；带重力的 static 场景缺约束时存在刚体零空间，一般需要 JSON 配 `fixed` 或 attachment（见 [_config.md](_config.md)）。

## 交叉链接

- 配置 schema 与校验：[_config.md](_config.md)
- 被串起的下游：[_scene.md](_scene.md)、[_runners.md](_runners.md)
- 调用方：6 个入口 stub（[tet_static.md](tet_static.md) 等）与 [batch.md](batch.md)
