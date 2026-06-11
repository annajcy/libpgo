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
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_squash.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_box_squash.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_sphere_ipc.json
pypgo-sim-shell-dynamic  --config examples/sim_configs/shell_dynamic_ipc_drop.json
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
| cubic_dynamic_box_squash | moving attachment targets, IPC self-contact, no gravity |
| tet_dynamic_box_squash | same as cubic_dynamic_box_squash but tet mesh |
| cubic_dynamic_box_sphere_ipc | box-with-sphere IPC drop onto floor obstacle, self-contact |
| shell_dynamic_ipc_drop | shell IPC drop onto obstacle, explicit dhat/kappa |

Try `"formulation": "cubic-tricubic-hermite"` in the cubic dynamic scene
for the tricubic Hermite variant
(soft attachments are not supported by the Hermite formulation; this scene has none).

## Legacy example/ipc parity

The four scenes above mirror the C++ `example/ipc/` cases:

| new config | legacy case |
|---|---|
| cubic_dynamic_box_squash | `example/ipc/cubic/box-squash` |
| tet_dynamic_box_squash | `example/ipc/tet/box-squash` |
| cubic_dynamic_box_sphere_ipc | `example/ipc/cubic/box-with-sphere-lite` |
| shell_dynamic_ipc_drop | `example/ipc/shell/shell-drop` |

Note: the legacy `ipc-heuristic` mode (automatic dhat/kappa selection) is not exposed
in the Python binding. Use explicit `dhat` and `kappa` values as shown in these configs.
