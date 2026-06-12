# `pypgo/tools/sim/tet_dynamic.py` — `pypgo-sim-tet-dynamic` 入口

> 源文件：`pypgo/tools/sim/tet_dynamic.py`（12 行）。模块架构见 [overview.md](overview.md)。

入口 stub：把 `(mesh_type="tet", mode="dynamic")` 绑进共享骨架，无其他逻辑。

## 调用链

```
main(argv) ─▶ _cli.run_cli(mesh_type="tet", mode="dynamic", prog="pypgo-sim-tet-dynamic")
              ├─ _config.load_config（dynamic.timestep 必填，见 _config.md）
              ├─ _scene.build_scene（TetLinear formulation，见 _scene.md）
              └─ _runners.run_dynamic（DynamicSimulation 逐步积分，见 _runners.md）
```

## 命令行用法

```bash
pypgo-sim-tet-dynamic --config examples/sim_configs/tet_dynamic_bunny_ipc.json \
    --num-steps 50 --output-dir /tmp/bunny
# 或
python -m pypgo.tools.sim.tet_dynamic --config scene.json
```

可用 flags 见 [_cli.md](_cli.md)（含 `--timestep/--num-steps/--integrator/--damping`）。示例场景：`tet_dynamic_{box,box_sphere,bunny,dragon}_ipc.json`（IPC 跌落族），见 [README](../../../../../examples/sim_configs/README.md)。
