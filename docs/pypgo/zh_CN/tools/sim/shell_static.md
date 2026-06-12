# `pypgo/tools/sim/shell_static.py` — `pypgo-sim-shell-static` 入口

> 源文件：`pypgo/tools/sim/shell_static.py`（12 行）。模块架构见 [overview.md](overview.md)。

入口 stub：把 `(mesh_type="shell", mode="static")` 绑进共享骨架，无其他逻辑。

## 调用链

```
main(argv) ─▶ _cli.run_cli(mesh_type="shell", mode="static", prog="pypgo-sim-shell-static")
              ├─ _config.load_config（只收 mesh.surface；material 走壳分支，见 _config.md）
              ├─ _scene.build_scene（KoiterShell + KoiterStVK，见 _scene.md）
              └─ _runners.run_static（见 _runners.md）
```

## 命令行用法

```bash
pypgo-sim-shell-static --config shell_scene.json
# 或
python -m pypgo.tools.sim.shell_static --surface cloth.obj --output-dir /tmp/run
```

shell 特有：无 `--volume`/`--formulation` flag（formulation 固定 [KoiterShell](../../fem/formulations.md)）；`material` 段为 `thickness/E_membrane/nu_membrane/mass`（`mass` 二选一：`areal_density` 或 `density`×厚度），见 [_config.md](_config.md)。其余 flags 见 [_cli.md](_cli.md)。
