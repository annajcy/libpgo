# Sim CLI example scenes

Each JSON here is a complete scene for one of the `pypgo-sim-*` CLIs.
Paths inside a config resolve relative to the config file, so these run
from any CWD:

```bash
pypgo-sim-tet-static     --config examples/sim_configs/tet_static_dragon.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_bunny_floor.json
pypgo-sim-cubic-static   --config examples/sim_configs/cubic_static_box_hang.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_ipc.json
pypgo-sim-shell-static   --config examples/sim_configs/shell_static_drape.json
pypgo-sim-shell-dynamic  --config examples/sim_configs/shell_dynamic_floor.json
```

CLI flags override JSON values (defaults < JSON < CLI), e.g. append
`--num-steps 5 --output-dir /tmp/run` for a quick look.
Every config is exercised end-to-end by
`tests/pypgo/test_sim_cli_examples.py`.

Scene-to-feature map:

| config | features exercised |
|---|---|
| tet_static_dragon | static solve, gravity, soft vertex attachment (region selector) |
| tet_dynamic_bunny_floor | dynamics, FloorEnergy, initial velocity |
| cubic_static_box_hang | static solve, hard fixed DOFs (region selector) |
| cubic_dynamic_box_ipc | dynamics, IPC + static obstacle, initial velocity |
| shell_static_drape | Koiter shell static, shell self-weight, clamped edge |
| shell_dynamic_floor | shell dynamics, shell mass matrix, frictional penalty + floor |

Try `"formulation": "cubic-tricubic-hermite"` in the cubic dynamic scene
for the tricubic Hermite variant
(soft attachments are not supported by the Hermite formulation; this scene has none).
