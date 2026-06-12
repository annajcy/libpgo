# `pypgo/tools/sim/cubic_static.py` — `pypgo-sim-cubic-static` 入口

> 源文件：`pypgo/tools/sim/cubic_static.py`（12 行）。模块架构见 [overview.md](overview.md)。

入口 stub：把 `(mesh_type="cubic", mode="static")` 绑进共享骨架，无其他逻辑。

## 调用链

```
main(argv) ─▶ _cli.run_cli(mesh_type="cubic", mode="static", prog="pypgo-sim-cubic-static")
              ├─ _config.load_config（见 _config.md）
              ├─ _scene.build_scene（CubicLinear 或 CubicTricubicHermite，见 _scene.md）
              └─ _runners.run_static（见 _runners.md）
```

## 命令行用法

```bash
pypgo-sim-cubic-static --config examples/sim_configs/cubic_static_box_hang.json
# Hermite 变体（24 DOF/顶点；attachments 不可用，改 surface_attachments）
pypgo-sim-cubic-static --config examples/sim_configs/cubic_static_box_hang_hermite.json
# 或
python -m pypgo.tools.sim.cubic_static --config scene.json --formulation cubic-tricubic-hermite
```

可用 flags 见 [_cli.md](_cli.md)。formulation：`auto` = `cubic-linear`，可显式 `cubic-tricubic-hermite`（[../../fem/formulations.md](../../fem/formulations.md)）。要求 `.veg` 为 8 点 cubic 元素。示例总表见 [README](../../../../../examples/sim_configs/README.md)。
