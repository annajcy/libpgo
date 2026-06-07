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

The first Python simulation builder supports volumetric IPC scenes:

```python
import pypgo as pgo
from pypgo.tools import sim as tsim

volume = pgo.mesh.veg.VolumeMesh.from_veg_file(
    pgo.mesh.veg.read_veg("examples/ipc/cubic/box-hang/box.veg")
)
surface = pgo.mesh.read_obj("examples/ipc/cubic/box-hang/box.obj")

spec = tsim.VolumeIPCSimulationSpec(
    volume=volume,
    surface=surface,
    formulation=pgo.fem.LinearCubic(),
    runtime=tsim.RuntimeConfig(
        timestep=0.001,
        num_steps=10,
        gravity=(0.0, -9.81, 0.0),
    ),
    contact=tsim.IPCContactSpec(
        parameters=pgo.contact.IPCParameters(
            dhat=0.002,
            dhat_external=0.1,
            kappa=3000.0,
        ),
    ),
)

runner = tsim.build_volume_ipc_simulation(spec)
frames = runner.run()
deformed_surface = runner.deformed_surface(frames[-1].displacement)
```

Future migration work should add Python JSON loaders and shell/static parity
there, not reintroduce C++ CLI tools.
