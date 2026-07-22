# Shell IPC drop

This example is the direct Python-API version of
`examples/sim_configs/shell_dynamic_ipc_drop.json`.

It constructs a Koiter StVK shell, assembles mass from density and thickness,
adds gravity and IPC obstacle contact, and advances the state with implicit
backward Euler. Unlike a volume case, shell vertices are the simulation and
contact vertices, so no surface embedding matrix is required.

Run from the repository root:

```bash
python examples/demo/simulation/shell_dynamic_ipc_drop/main.py
```

The scene uses 80 steps at `dt = 0.0005`. Selected surface frames are written
to `examples/demo/simulation/shell_dynamic_ipc_drop/output/`.

Use the JSON scene when checkpointing, stress output, batch execution, or other
CLI-managed output is required.
