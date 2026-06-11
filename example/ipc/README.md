# IPC Example Assets

This directory keeps the IPC meshes, fixed-vertex lists, legacy JSON configs,
and historical animation configs that were previously used by the C++
`runIPCSim` and `convertAnimation` command-line tools.

The C++ CLI tools have been removed. New runnable workflows should use the
Python API:

- `pypgo.tools.sim` for simulation scene construction and dynamic stepping
- `pypgo.fem` for formulation-aware FEM operators
- `pypgo.contact` for IPC, floor, and penalty contact energies
- `pypgo.animation` for Alembic/VDB export

The existing JSON files remain useful as reference data while the Python API
gains full JSON compatibility. For new code, prefer constructing scenes with
Python dataclasses instead of adding new C++ tool config fields.

## Asset Layout

Representative assets live under:

- `shell/shell-hang/`
- `shell/shell-drop/`
- `tet/box-hang/`
- `tet/box-squash/`
- `cubic/box-hang/`
- `cubic/box-squash/`
- `cubic/box-with-sphere/`

Volume cases usually contain:

- `box.obj`: display/contact surface mesh
- `box.veg`: tetrahedral or cubic simulation mesh
- `box-fixed*.txt`: fixed or pulled simulation vertex lists
- `box-ipc.json`: historical C++ CLI config retained for migration reference
- `anim.json`: historical animation conversion config retained for migration reference

Shell cases usually contain:

- `shell.obj`: shell simulation/display mesh
- `shell-fixed*.txt`: fixed or pulled shell vertex lists
- `bottom.obj`: optional external obstacle mesh
- `shell-ipc.json`: historical C++ CLI config retained for migration reference
- `anim.json`: historical animation conversion config retained for migration reference

## Python Direction

Runnable scenes are driven by the `pypgo-sim-*` CLI family (tet/cubic/shell x
static/dynamic) with JSON scene configs; see `examples/sim_configs/` for
runnable examples covering IPC, floor, and frictional penalty contact:

```bash
pypgo-sim-cubic-dynamic \
  --volume example/ipc/cubic/box-hang/box.veg \
  --surface example/ipc/cubic/box-hang/box.obj \
  --output-dir /tmp/box-hang \
  --timestep 0.001 --num-steps 100 \
  --gravity 0 -9.81 0
```

JSON configs support contact models (`ipc` / `floor` / `sampled_penalty` /
`frictional_sampled_penalty`), fixed-vertex selectors (`file` / `indices` /
`region`), soft attachments, and shell scenes. CLI flags override JSON values
(defaults < JSON < CLI). The historical C++ JSON files in this directory
remain reference data only.
