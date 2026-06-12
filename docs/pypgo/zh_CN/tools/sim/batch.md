# `pypgo/tools/sim/batch.py` — `pypgo-sim-batch` 批量编排器

> 源文件：`pypgo/tools/sim/batch.py`（256 行）。模块架构见 [overview.md](overview.md)。

按批量 JSON **串行**运行多个 sim 入口：每个 case 经 `importlib` 找到入口模块、以 **Python 函数调用** `main(argv)` 执行（不开子进程），收集退出码打 PASS/FAIL 表。

## 批量 JSON schema

```json
{
    "cases": {
        "tet-bunny": { "command": "tet-dynamic",
                       "config": "tet_dynamic_bunny_ipc.json",
                       "args": ["--num-steps", "10"] },
        "shell-drop": { "command": "shell-dynamic",
                        "config": "shell_dynamic_ipc_drop.json" }
    },
    "jobs": {
        "smoke": ["tet-bunny"],
        "all": ["tet-bunny", "shell-drop"]
    }
}
```

| 键 | 含义 |
|---|---|
| `cases.<name>.command` | 6 选 1：`tet-static/tet-dynamic/cubic-static/cubic-dynamic/shell-static/shell-dynamic`（映射到 `pypgo.tools.sim.*` 模块） |
| `cases.<name>.config` | 场景 JSON；相对路径按**批量文件所在目录**解析，须存在 |
| `cases.<name>.args` | 追加的 CLI flags 列表（排在 `--config` 之后，故可覆盖场景 JSON） |
| `jobs.<name>` | case 名列表，或字符串 `"all"`；成员必须已定义 |

job 选择规则（as-implemented）：`--job all` 时若定义了名为 `all` 的 job 用之，否则隐式等于全部 cases；其他 job 名必须存在。

## 命令行用法

```bash
pypgo-sim-batch --config examples/sim_configs/batch.json \
    [--job smoke] [--list] [--output-root /tmp/batch]
# 或 python -m pypgo.tools.sim.batch ...
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `--config` | 必填 | 批量 JSON |
| `--job` | `all` | 要跑的 job |
| `--list` | — | 只打印 cases/jobs 清单后退出（不校验路径） |
| `--output-root` | 无 | 给出时：每个 case 追加 `--output-dir <root>/<case_name>`，并在 root 写 `batch_summary.json` |

每个 case 实际执行的 argv：`["--config", <config>] + args [+ ["--output-dir", <root>/<name>]]`。

## 退出码与产物

| 码 | 含义 |
|---|---|
| 0 | 所有选中 case 通过 |
| 1 | 至少一个 case 失败（批量配置本身有效；单 case 的 `SystemExit`/异常被捕获，批次继续） |
| 2 | 批量配置非法（`ConfigError`）或 argparse 错误 |

`batch_summary.json`（仅 `--output-root` 时）：`{cases: {name: {command, exit_code, passed}}, num_passed, num_failed}`。

## 用法示例

`examples/sim_configs/batch.json` 收录全部示例场景的 cases 与若干 job（见 [README](../../../../../examples/sim_configs/README.md)）：

```bash
pypgo-sim-batch --config examples/sim_configs/batch.json --list
pypgo-sim-batch --config examples/sim_configs/batch.json --job all \
    --output-root /tmp/sim_suite
```

## 交叉链接

- 被编排的入口与其 flags：[overview.md](overview.md)、[_cli.md](_cli.md)
- 每 case 的 `summary.json`：[_outputs.md](_outputs.md)
