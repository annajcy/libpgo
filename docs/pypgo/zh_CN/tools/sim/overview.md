# `pypgo.tools.sim` — 仿真 CLI 套件架构

> 包目录：`pypgo/tools/sim/`（13 个文件）。工具层架构见 [../overview.md](../overview.md)。

## 模块职责

一族 JSON 场景配置驱动的 FEM 仿真命令：**3 种网格类型（tet / cubic / shell）× 2 种模式（static / dynamic）= 6 个入口** + 1 个批量器。把 pypgo 的库模块（[fem](../../fem/overview.md)、[energy](../../energy/overview.md)、[contact](../../contact/overview.md)、[solver](../../solver/overview.md)、[sim](../../sim/overview.md)）编排成端到端可跑的命令行，是整个文档树贯穿示例的"可执行版"。

分层（入口极薄，逻辑全在 `_` 私有模块）：

```
tet_static.py 等 6 个 stub（各 12 行）
   └─▶ _cli.py      参数解析 + 优先级合并（defaults < JSON < CLI flags）
        └─▶ _config.py   JSON schema → 冻结 SimConfig（校验/路径解析）
        └─▶ _scene.py    SimConfig → SceneBundle（网格/能量/约束/接触/重力装配）
        └─▶ _runners.py  run_static / run_dynamic（与网格类型无关的执行循环）
             └─▶ _outputs.py  summary.json / 表面 OBJ 写出
batch.py  按批量 JSON 串行调用上述入口的 main(argv)
```

## 入口 × 配置 × runner 总表

| 命令（console_script / `python -m`） | mesh_type | mode | formulation 取值 | runner | 文档 |
|---|---|---|---|---|---|
| `pypgo-sim-tet-static` / `pypgo.tools.sim.tet_static` | `tet` | static | `auto`=`tet-linear` | `run_static` | [tet_static.md](tet_static.md) |
| `pypgo-sim-tet-dynamic` / `...tet_dynamic` | `tet` | dynamic | `auto`=`tet-linear` | `run_dynamic` | [tet_dynamic.md](tet_dynamic.md) |
| `pypgo-sim-cubic-static` / `...cubic_static` | `cubic` | static | `auto`=`cubic-linear`，或 `cubic-tricubic-hermite` | `run_static` | [cubic_static.md](cubic_static.md) |
| `pypgo-sim-cubic-dynamic` / `...cubic_dynamic` | `cubic` | dynamic | 同上 | `run_dynamic` | [cubic_dynamic.md](cubic_dynamic.md) |
| `pypgo-sim-shell-static` / `...shell_static` | `shell` | static | 固定 KoiterShell | `run_static` | [shell_static.md](shell_static.md) |
| `pypgo-sim-shell-dynamic` / `...shell_dynamic` | `shell` | dynamic | 固定 KoiterShell | `run_dynamic` | [shell_dynamic.md](shell_dynamic.md) |
| `pypgo-sim-batch` / `...batch` | —（批量编排） | — | — | 逐 case 调上述 `main` | [batch.md](batch.md) |

所有入口共享同一 JSON schema（[_config.md](_config.md)）与同一 CLI flag 集（[_cli.md](_cli.md)）；mesh_type/mode 由**命令本身**决定，配置里的可选 `"type"` 字段只做一致性校验。

## 文件 ↔ 职责 主表

| 文件 | 职责 | 关键符号 | 文档 |
|---|---|---|---|
| `_config.py` | JSON/CLI → 冻结配置 + 全部校验 | `SimConfig`、`load_config`、`ConfigError`、各 Section dataclass | [_config.md](_config.md) |
| `_scene.py` | 配置 → 场景装配 | `SceneBundle`、`build_scene`、`resolve_vertex_selector` | [_scene.md](_scene.md) |
| `_runners.py` | static/dynamic 执行循环 + 输出调度 | `run_static`、`run_dynamic` | [_runners.md](_runners.md) |
| `_outputs.py` | 文件写出助手 | `write_summary`、`write_surface` | [_outputs.md](_outputs.md) |
| `_cli.py` | argparse + 优先级合并 + 入口骨架 | `run_cli`、`build_parser` | [_cli.md](_cli.md) |
| `batch.py` | 批量编排（cases/jobs） | `main` | [batch.md](batch.md) |
| 6 个入口 stub | 绑定 (mesh_type, mode) | `main` | 各自文档 |
| `__init__.py` | 公开面 | — | [\_\_init\_\_.md](__init__.md) |

## 物理内容一览（static 与 dynamic 解什么）

static：解总势能驻点（[`NewtonOptimizer`](../../solver/optimizer.md)）

$$\mathbf u^*=\arg\min_{\mathbf u}\ E_{\text{def}}(\mathbf u)+\textstyle\sum E_{\text{att}}+\sum E_{\text{contact}}-\mathbf f_g^\top\mathbf u$$

重力以线性势能 `LinearEnergy(-f_g)` 进目标（$\mathbf f_g$ = formulation 的 `body_force`）；硬固定经 `problem.fix_variables` 消元。

dynamic：[`DynamicSimulation`](../../sim/simulation.md) 逐步隐式积分（`implicit_euler` / `trbdf2`），重力改作外力传入 `sim.step(external_force=...)`，接触的 `begin_step` 由 C++ stepper 派发。细节见 [_runners.md](_runners.md)。

## 示例场景库

`examples/sim_configs/` 收录全部端到端示例（dragon 静态吊挂、box/bunny/dragon IPC 跌落、Hermite 变体、shell 跌落…），每个配置都由 `tests/pypgo/test_sim_cli_examples.py` 覆盖登记；静态示例默认跑通，动态端到端示例需设置 `PYPGO_RUN_DYNAMIC_SIM_CLI_EXAMPLES=1` 后显式复跑。总表与输出 flag 速查见 [examples/sim_configs/README.md](../../../../../examples/sim_configs/README.md)。批量复跑：

```bash
pypgo-sim-batch --config examples/sim_configs/batch.json --job smoke --output-root /tmp/batch
```

## 贯穿示例

```bash
pypgo-sim-tet-dynamic \
    --config examples/sim_configs/tet_dynamic_bunny_ipc.json \
    --num-steps 50 --output-dir /tmp/bunny      # CLI 覆盖 JSON

cat /tmp/bunny/summary.json                     # 收敛与逐帧状态
# 开了 write_states/write_stress 的话，接动画后处理：
pypgo-stress-vdb assets/bunny.veg /tmp/bunny /tmp/bunny/vdb
```

## 交叉链接

- 配置 schema 全表：[_config.md](_config.md)；装配流程：[_scene.md](_scene.md)
- 输出布局（summary/surface/states/stress/abc）：[_outputs.md](_outputs.md)、[_runners.md](_runners.md)
- 后处理：[../animation/overview.md](../animation/overview.md)
- 上游网格准备：[../mesh/overview.md](../mesh/overview.md)
