# `pypgo/tools/sim/tet_static.py` — `pypgo-sim-tet-static` 入口

> 源文件：`pypgo/tools/sim/tet_static.py`（12 行）。模块架构见 [overview.md](overview.md)。

入口 stub：把 `(mesh_type="tet", mode="static")` 绑进共享骨架，无其他逻辑。

## 调用链

```
main(argv) ─▶ _cli.run_cli(mesh_type="tet", mode="static", prog="pypgo-sim-tet-static")
              ├─ _config.load_config（defaults < JSON < flags，见 _config.md）
              ├─ _scene.build_scene（TetLinear formulation，见 _scene.md）
              └─ _runners.run_static（势能极小化 + 重力线性势，见 _runners.md）
```

## 命令行用法

```bash
pypgo-sim-tet-static --config examples/sim_configs/tet_static_dragon.json
# 或
python -m pypgo.tools.sim.tet_static --config scene.json --output-dir /tmp/run
```

可用 flags 见 [_cli.md](_cli.md)（static 集：无 `--timestep` 等动态参数）。formulation：`auto` = `tet-linear`。示例场景：`tet_static_dragon.json`（surface_attachments）、`tet_static_box_hang.json`（region 硬固定），见 [README](../../../../../examples/sim_configs/README.md)。
