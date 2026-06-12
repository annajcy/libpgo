# `pypgo/tools/sim/shell_dynamic.py` — `pypgo-sim-shell-dynamic` 入口

> 源文件：`pypgo/tools/sim/shell_dynamic.py`（12 行）。模块架构见 [overview.md](overview.md)。

入口 stub：把 `(mesh_type="shell", mode="dynamic")` 绑进共享骨架，无其他逻辑。

## 调用链

```
main(argv) ─▶ _cli.run_cli(mesh_type="shell", mode="dynamic", prog="pypgo-sim-shell-dynamic")
              ├─ _config.load_config（dynamic.timestep 必填，见 _config.md）
              ├─ _scene.build_scene（KoiterShell + KoiterStVK，见 _scene.md）
              └─ _runners.run_dynamic（见 _runners.md）
```

## 命令行用法

```bash
pypgo-sim-shell-dynamic --config examples/sim_configs/shell_dynamic_ipc_drop.json
# 或
python -m pypgo.tools.sim.shell_dynamic --config scene.json --timestep 0.005 --num-steps 200
```

shell 特有事项同 [shell_static.md](shell_static.md)（无 `--volume`/`--formulation`；壳材料/质量配置）。dynamic flags 见 [_cli.md](_cli.md)。示例：`shell_dynamic_ipc_drop.json`（IPC 跌落，显式 dhat/kappa），见 [README](../../../../../examples/sim_configs/README.md)。
