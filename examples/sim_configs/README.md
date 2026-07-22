# Sim CLI example scenes

Each JSON here is a complete scene for one of the `pypgo-sim-*` CLIs.
Paths inside a config resolve relative to the config file, so these run
from any CWD:

```bash
# static
pypgo-sim-tet-static     --config examples/sim_configs/tet_static_dragon.json
pypgo-sim-tet-static     --config examples/sim_configs/tet_static_box_hang.json
pypgo-sim-cubic-static   --config examples/sim_configs/cubic_static_box_hang.json
# dynamic (IPC collision cases; volume scenes paired tet/cubic)
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_box_ipc.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_ipc.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_box_sphere_ipc.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_box_sphere_ipc.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_bunny_ipc.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_bunny_ipc.json
pypgo-sim-tet-dynamic    --config examples/sim_configs/tet_dynamic_dragon_ipc.json
pypgo-sim-cubic-dynamic  --config examples/sim_configs/cubic_dynamic_dragon_ipc.json
pypgo-sim-shell-dynamic  --config examples/sim_configs/shell_dynamic_ipc_drop.json
```

CLI flags override JSON values (defaults < JSON < CLI), e.g. append
`--num-steps 5 --output-dir /tmp/run` for a quick look.
Without an output path in either place, a sim CLI writes to `./output` in
the current working directory. The configs in this directory explicitly use
`output/<case>/`, resolved relative to this directory.
Every config is listed by `tests/pypgo/test_sim_cli_examples.py`; full
end-to-end example runs are opt-in with `PYPGO_RUN_SIM_CLI_EXAMPLES=1`.

Output flags (all in the `output` section; dynamic dumps share one `dump_interval`):

| flag | output |
|---|---|
| `write_surfaces` | `surface/surface%04d.obj` per dumped frame (static: `final_surface.obj`) |
| `write_states` | `states/deform%04d.u` Eigen-binary displacement (static: `deform_final.u`) |
| `write_checkpoints` | `checkpoints/state%04d.npz` restart checkpoint per dumped dynamic frame |
| `write_stress` | `stress/von_mises%04d.json` per-element von Mises (static: `von_mises_final.json`) |
| `write_abc` | `animation.abc` Alembic of the dumped surface frames, fps = 1/(dump_interval × timestep); dynamic only |

Dynamic runs can resume from checkpoints by setting `dynamic.resume` to
`"latest"` or to a specific `checkpoints/state%04d.npz` path. `num_steps` is
the total target timestep count, not an additional step count: resuming from a
checkpoint at timestep 280 with `num_steps: 500` advances to timestep 500.
The matching CLI flags are `--write-checkpoints` and `--resume latest|PATH`.

Scene-to-feature map:

| config | features exercised |
|---|---|
| tet_static_dragon | static solve on `dragon_big.veg`, gravity, surface_attachments via `dragon-surface-fixed.txt` (file selector, formulation-portable) |
| tet_static_box_hang | static solve, hard fixed DOFs (region selector), tet mesh |
| cubic_static_box_hang | same as tet_static_box_hang but cubic mesh |
| tet/cubic_dynamic_box_ipc | IPC drop onto dish obstacle, initial velocity (legacy box drop) |
| tet/cubic_dynamic_box_sphere_ipc | box-with-sphere IPC drop, self-contact (legacy box-with-sphere-lite) |
| tet/cubic_dynamic_bunny_ipc | bunny IPC drop onto dish obstacle (legacy bunny-lite-ipc) |
| tet/cubic_dynamic_dragon_ipc | dragon IPC drop, initial lift displacement (legacy dragon-dyn-lite) |
| shell_dynamic_ipc_drop | shell IPC drop onto obstacle, explicit dhat/kappa (legacy shell-drop) |
| cubic_dynamic_box_ipc_hermite | box IPC drop with the `cubic-tricubic-hermite` formulation (24 DOFs/vertex) |
| cubic_dynamic_box_sphere_ipc_hermite | box-with-sphere IPC drop, tricubic Hermite |
| cubic_dynamic_bunny_ipc_hermite | bunny IPC drop, tricubic Hermite |
| cubic_dynamic_dragon_ipc_hermite | dragon IPC drop, tricubic Hermite |
| cubic_static_box_hang_hermite | static box hang, tricubic Hermite (fixed regions clamp all 24 DOFs) |

All IPC drop scenes run to contact with legacy step counts (2000 steps;
dragon 4000) and export `animation.abc` (`write_abc`).

The `*_hermite` scenes are identical to their cubic-linear siblings except for
`mesh.formulation` and `num_steps` (capped at 500 — Hermite + IPC contact costs
~10-30s/step); run the `hermite` batch job to execute all of them. Soft vertex
attachments are not supported by the Hermite formulation (these scenes have
none; `surface_attachments` work fine).

Note: the legacy `ipc-heuristic` mode (automatic dhat/kappa selection) is not exposed
in the Python binding. Use explicit `dhat` and `kappa` values as shown in these configs.

The config schema additionally supports features without a dedicated example scene:
`floor` / `sampled_penalty` contact models, moving attachment targets
(`attachments[].movement`), and region/index vertex selectors — all covered by
unit tests under `tests/pypgo/`. Sampled penalty friction is enabled by adding
`friction_coeff` and `velocity_eps` to a `sampled_penalty` contact entry; omitting
those fields means no friction.

## Batch runs

`batch.json` groups the scenes into named jobs.  Use `pypgo-sim-batch`
to run a subset or all of them in one command:

```bash
# List all cases and jobs without running anything
pypgo-sim-batch --config examples/sim_configs/batch.json --list

# Run the "drops" job; output defaults to ./output/<case_name>
pypgo-sim-batch --config examples/sim_configs/batch.json --job drops

# Run all cases with an explicit output root
pypgo-sim-batch --config examples/sim_configs/batch.json --output-root /tmp/sim-batch-all
```

Each case writes its `summary.json` to
`<output-root>/<case_name>/` and an overall `batch_summary.json` is written to
`<output-root>/`. `<output-root>` defaults to `./output` in the directory where
the batch CLI is launched.

Jobs defined in `batch.json`:

| job | cases |
|---|---|
| all | all 12 cases |
| static | tet_static_dragon, tet/cubic_static_box_hang |
| dynamic / drops | the 9 dynamic IPC collision cases |
