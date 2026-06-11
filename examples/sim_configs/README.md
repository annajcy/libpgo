# Sim CLI example scenes

Each JSON here is a complete scene for one of the `pypgo-sim-*` CLIs.
Paths inside a config resolve relative to the config file, so these run
from any CWD:

```bash
# static
pypgo-sim-tet-static     --config examples/sim_configs/tet_static_dragon.json
pypgo-sim-tet-static     --config examples/sim_configs/tet_static_box_hang.json
pypgo-sim-cubic-static   --config examples/sim_configs/cubic_static_box_hang.json
pypgo-sim-cubic-static   --config examples/sim_configs/cubic_static_dragon.json
pypgo-sim-shell-static   --config examples/sim_configs/shell_static_drape.json
# dynamic
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_bunny_floor.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_bunny_floor.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_box_ipc.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_ipc.json
pypgo-sim-shell-dynamic  --config examples/sim_configs/shell_dynamic_floor.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_box_squash.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_squash.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_box_sphere_ipc.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_sphere_ipc.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_dragon_ipc.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_dragon_ipc.json
pypgo-sim-shell-dynamic  --config examples/sim_configs/shell_dynamic_ipc_drop.json
```

CLI flags override JSON values (defaults < JSON < CLI), e.g. append
`--num-steps 5 --output-dir /tmp/run` for a quick look.
Every config is exercised end-to-end by
`tests/pypgo/test_sim_cli_examples.py`.

Every volume scene exists in both tet and cubic variants (tet/cubic pairing convention).

Scene-to-feature map:

| config | features exercised |
|---|---|
| tet_static_dragon | static solve, gravity, soft vertex attachment (region selector) |
| cubic_static_dragon | same as tet_static_dragon but cubic mesh |
| tet_static_box_hang | static solve, hard fixed DOFs (region selector), tet mesh |
| cubic_static_box_hang | same as tet_static_box_hang but cubic mesh |
| shell_static_drape | Koiter shell static, shell self-weight, clamped edge |
| tet_dynamic_bunny_floor | dynamics, FloorEnergy, initial velocity |
| cubic_dynamic_bunny_floor | same as tet_dynamic_bunny_floor but cubic mesh |
| tet_dynamic_box_ipc | dynamics, IPC + static obstacle, initial velocity, tet mesh |
| cubic_dynamic_box_ipc | same as tet_dynamic_box_ipc but cubic mesh |
| shell_dynamic_floor | shell dynamics, shell mass matrix, frictional penalty + floor |
| tet_dynamic_box_squash | moving attachment targets, IPC self-contact, no gravity, tet mesh |
| cubic_dynamic_box_squash | same as tet_dynamic_box_squash but cubic mesh |
| tet_dynamic_box_sphere_ipc | box-with-sphere IPC drop onto floor obstacle, self-contact, tet mesh |
| cubic_dynamic_box_sphere_ipc | same as tet_dynamic_box_sphere_ipc but cubic mesh |
| tet_dynamic_dragon_ipc | dragon IPC drop onto dish obstacle, initial lift displacement, tet mesh |
| cubic_dynamic_dragon_ipc | same as tet_dynamic_dragon_ipc but cubic mesh |
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

## Batch runs

`batch.json` groups all ten scenes into named jobs.  Use `pypgo-sim-batch`
to run a subset or all of them in one command:

```bash
# List all cases and jobs without running anything
pypgo-sim-batch --config examples/sim_configs/batch.json --list

# Run the "drops" job (5 dynamic drop cases) and save output under /tmp/sim-batch
pypgo-sim-batch --config examples/sim_configs/batch.json --job drops --output-root /tmp/sim-batch

# Run all 10 cases
pypgo-sim-batch --config examples/sim_configs/batch.json --output-root /tmp/sim-batch-all
```

After a run with `--output-root`, each case writes its `summary.json` to
`<output-root>/<case_name>/` and an overall `batch_summary.json` is written to
`<output-root>/`.

Jobs defined in `batch.json`:

| job | cases |
|---|---|
| all | all 17 cases |
| static | 5 static cases (tet+cubic dragon, tet+cubic box hang, shell drape) |
| dynamic | all 12 dynamic cases |
| squash | cubic_dynamic_box_squash, tet_dynamic_box_squash |
| drops | all IPC/floor drop cases (tet+cubic box ipc, tet+cubic box sphere, tet+cubic bunny, tet+cubic dragon, shell ipc drop, shell floor) |
