# `pypgo/tools/sim/cubic_dynamic.py` — `pypgo-sim-cubic-dynamic` 入口

> 源文件：`pypgo/tools/sim/cubic_dynamic.py`（12 行）。模块架构见 [overview.md](overview.md)。

入口 stub：把 `(mesh_type="cubic", mode="dynamic")` 绑进共享骨架，无其他逻辑。

## 调用链

```
main(argv) ─▶ _cli.run_cli(mesh_type="cubic", mode="dynamic", prog="pypgo-sim-cubic-dynamic")
              ├─ _config.load_config（dynamic.timestep 必填，见 _config.md）
              ├─ _scene.build_scene（CubicLinear 或 CubicTricubicHermite，见 _scene.md）
              └─ _runners.run_dynamic（见 _runners.md）
```

## 命令行用法

```bash
pypgo-sim-cubic-dynamic --config examples/sim_configs/cubic_dynamic_bunny_ipc.json
# Hermite 变体（与 cubic-linear 场景仅差 mesh.formulation 与步数）
pypgo-sim-cubic-dynamic --config examples/sim_configs/cubic_dynamic_dragon_ipc_hermite.json
# 或
python -m pypgo.tools.sim.cubic_dynamic --config scene.json --num-steps 100
```

可用 flags 见 [_cli.md](_cli.md)。Hermite 场景的硬固定 clamp 每顶点全部 24 个 DOF，软约束须用 `surface_attachments`（[_scene.md](_scene.md)）。示例总表见 [README](../../../../../examples/sim_configs/README.md)。
