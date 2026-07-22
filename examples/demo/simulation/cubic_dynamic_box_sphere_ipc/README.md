# Cubic box-with-sphere IPC drop

This example is the direct Python-API version of
`examples/sim_configs/cubic_dynamic_box_sphere_ipc.json`.

It combines cubic-linear FEM, implicit dynamics, gravity, self-contact, and an
external obstacle. The OBJ surface is embedded in the volume DOFs and reused by
`IPCEnergy`, so both collision response and output operate on the same render
surface.

Run from the repository root:

```bash
python examples/demo/simulation/cubic_dynamic_box_sphere_ipc/main.py
```

The full contact sequence uses 2000 steps at `dt = 0.001`. Edit `NUM_STEPS` and
`DUMP_INTERVAL` at the top of `main.py` for a shorter diagnostic run. Selected
surface frames are written to
`examples/demo/simulation/cubic_dynamic_box_sphere_ipc/output/`.

The corresponding JSON config additionally supports CLI-controlled Alembic,
checkpoint, stress, and summary output. Those features are intentionally left
out here to keep the public simulation API visible.
