# Python API Migration Milestones

## 目标

最终目标是把 `src/tools/sim/runIPCSim` 的完整仿真工作流迁移到 Python-first API：

- Python 负责 config / scene 构建、simulation orchestration、batch runs、output 管理、实验脚本和 PyTorch 生态联动。
- C++ 保留高性能核心：mesh/FEM/contact/energy/solver/time integrator 等数值内核。
- 旧 C-style Python API 不保留；它原来覆盖的能力必须以新的 Python-first API 重新出现。
- 现有 `runIPCSim` CLI 在迁移期保留为行为基准，最终可以降级为 thin wrapper 或被 Python CLI 替代。

## 当前基线

当前 `runIPCSim` 已经拆成可迁移的 C++ core library：

- `runIPCSimCore`
  - app orchestration: `src/tools/sim/runIPCSim/app/*`
  - typed runtime config: `app/config.*`
  - setup: `setup/*`
  - contact backend: `contact/*`
  - solver/static solve: `solver/*`
  - output/session/loop: `app/output.*`, `app/session.*`, `app/loop.*`
- executable:
  - `src/tools/sim/runIPCSim/runIPCSim.cpp`
  - `src/tools/sim/runIPCSim/cli/*`

Python side is intentionally reset to a clean package scaffold:

- `pypgo/__init__.py`
- no `pypgo.legacy`
- no C-style wrapper entrypoints

This makes the next API free to model user workflows instead of mirroring historical C/C++ handles.

## 迁移原则

1. **Python-first surface, C++ kernel underneath.** Public Python API should expose `Mesh`, `Material`, `Energy`, `Simulation`, `Frame`, `Result`, not `create_*` / `destroy_*`.
2. **NumPy is the first data protocol.** Public array-facing APIs should accept NumPy-like inputs, normalize through NumPy shape/dtype rules, and return NumPy arrays or objects that can cheaply produce NumPy arrays.
3. **JSON parity first, programmatic scenes second.** The first Python runner must be able to load existing `runIPCSim` JSON configs and match behavior before we design more expressive scene builders.
4. **No hidden behavior drift.** Every milestone that touches simulation semantics must compare against existing CLI examples/tests.
5. **Interop is layered.** NumPy is required and foundational; SciPy and PyTorch integration should be optional adapters built on top of the same sparse/vector contracts.
6. **Keep C++ bindings narrow.** Bind stable service boundaries and value types; do not expose every internal helper class just because it exists.
7. **Refactor C++ when Python exposes bad boundaries.** If a clean Python API would require binding awkward ownership chains, oversized classes, global side effects, raw pointer lifetimes, or C++ implementation details, first introduce or refactor a C++ service/facade API and bind that instead.

## C++ API Refactoring Policy

Python migration is allowed to drive C++ API refactoring. The rule is not "bind whatever exists"; the rule is "make the native boundary clean enough that Python can stay clean."

Refactor C++ before binding when any of these are true:

- A Python concept would need to expose a long C++ ownership chain, such as `energy -> assembler -> manager -> mesh`.
- A binding would require Python users to call setup functions in a fragile order.
- A public Python object would have to own raw pointers or call `destroy_*`.
- A Python method would need to mirror a large C++ method with unrelated responsibilities.
- A C++ API only works through filesystem side effects when Python should support in-memory objects.
- A C++ function mixes config parsing, object construction, simulation stepping, logging, and output writing.
- A useful Python operation can only be implemented by binding private helper types.

Preferred C++ refactor shapes:

- **Facade/service functions** for workflow boundaries, such as `buildSimulationContext`, `runStaticSolve`, `advanceDynamicStep`, or `writeSimulationFrame`.
- **Value objects** for typed inputs and outputs, such as runtime config, solver result, frame state, output paths, and sparse COO data.
- **RAII ownership roots** that hide internal chains from Python while keeping lifetimes deterministic.
- **Small adapters around existing kernels** instead of exposing full internal classes.
- **Behavior-preserving extraction** with C++ characterization tests before Python bindings depend on the new boundary.

Refactor constraints:

- Keep existing C++ CLI behavior as the parity oracle during migration.
- Prefer additive C++ APIs until Python parity tests are stable.
- Do not refactor numerical kernels just to make bindings prettier; refactor orchestration, ownership, data conversion, and service boundaries first.
- Every C++ refactor that changes a binding boundary should have a C++ test and a Python test.

## Milestone 0: Scaffold and Parity Inventory

**Goal:** Establish the Python package shape and define exactly what "full run sim migration" means.

**Deliverables:**

- Keep the clean `pypgo` package scaffold.
- Create a Python API coverage matrix that maps current C++/tool features to future Python modules.
- Define a run-sim parity suite covering representative shell, tet, cubic, static, dynamic, IPC, floor, obstacle, surface pressure, restart, von Mises, and volume legacy penalty cases. Legacy shell is not part of the main Python run-sim path and will be handled by a separate legacy sim script later.
- Define the first array data contract:
  - positions use `(n, 3)` floating arrays.
  - triangle faces use `(m, 3)` integer arrays.
  - tet elements use `(m, 4)` integer arrays.
  - cubic elements use `(m, 8)` integer arrays.
  - `.veg` files load as volume meshes with an explicit cell type.
  - solver DOF vectors use flat `(3 * n,)` floating arrays.
  - sparse matrices expose COO arrays as `(rows, cols, values)`.
- Decide the target public module names:
  - `pypgo.mesh` — `MeshData`, OBJ I/O, shape factory, mesh data utilities.
  - `pypgo.mesh.geo` — geometry façade and geometry-only algorithms.
  - `pypgo.mesh.veg` — Vega volume mesh, `.veg` I/O, volume materials, sets/regions.
  - `pypgo.sim` — solver-ready `SimulationMesh`, simulation materials/factories, shell spec I/O.
  - `pypgo.fem`
  - `pypgo.energy`
  - `pypgo.contact`
  - `pypgo.sparse`
  - `pypgo.torch`

`pypgo.io` is not part of the target public API. File I/O is domain-scoped instead of centralized: OBJ lives in `pypgo.mesh`, VEG lives in `pypgo.mesh.veg`, and shell specs live in `pypgo.sim`.

**Key files:**

- `plan/python_api_migration/api_coverage.md`
- `plan/python_api_migration/parity_matrix.md`
- `plan/python_api_migration/numpy_data_contract.md`
- `plan/python_api_migration/future_work.md`
- `tests/pypgo/test_package_scaffold.py`

**Exit criteria:**

- Every current `runIPCSim` user-visible feature is either assigned to a Python milestone or explicitly marked out of scope.
- Every old Python wrapper capability is assigned to a new Python-first module.
- The NumPy shape/dtype/copy policy is documented before native bindings are added.

## Milestone 1: Native Binding Foundation

**Goal:** Add focused native extension infrastructure without reviving the old monolithic wrapper. Establish the nanobind pipeline, NumPy conversion helpers, and a first batch of mesh type bindings (`MeshData`, `MeshGeo`, `VolumeMesh`, I/O) to prove the binding architecture is sound.

**Deliverables:**

1. **Private native module `pypgo._core`**, built with nanobind from small binding translation units.
2. **Common conversion helpers:**
   - NumPy array validation for `(n, 3)`, `(m, K)`, flat DOF vectors.
   - dtype normalization: floating → `float64`, index → `int64`; contiguous CPU array check.
   - Eigen dense vector/matrix conversion from/to NumPy arrays.
   - Eigen sparse matrix wrapper that can export NumPy COO arrays.
   - `gil_scoped_release` policy for long-running kernels.
3. **Mesh type bindings** (first batch, proves the architecture):
   - `pgo::Mesh::MeshData<K>` as shared data container (tri K=3, tet K=4, cubic K=8), with `ElementsView<K>`.
   - `TriMeshGeo`, `TetMeshGeo`, `CubicMeshGeo` as geometry façades; explicit `to_mesh_data()` / `from_mesh_data()` bridge.
   - `MaterialSpec` value object.
   - `VolumeMesh` simulation mesh — accepts `TetMeshData | CubicMeshData`, rejects `MeshGeo` / `TriMeshData`.
   - File I/O target: `pypgo.mesh.veg.read_veg/write_veg`, `pypgo.mesh.read_obj/write_obj`, and `pypgo.sim.read_shell/write_shell`; all geometry arrays stay at the `MeshData` boundary.
4. **MeshGeo utility scope decision:** M1 does NOT bind C++ Geo utility functions (normals, areas, distances, sub-mesh, etc.). Geo objects are "data façade + bridge" only. Utility bindings are prioritized for M2/M4 (see table below).
5. **Tests:**
   - Import smoke tests and packaging tests in `tests/pypgo`.
   - C++ tests: `MeshData` construction, `ElementsView` indexing, Geo `toMeshData()` roundtrip.
   - Python tests: dtype acceptance, shape rejection, copy/view behavior, boundary type checks.
6. **Cleanup:** Remove old public names (`TriCellMeshGeo`, `TetCellMeshGeo`, `CubicCellMeshGeo`, `CellMeshType`).

**Recommended binding layout:**

```text
src/python/pypgo/
  CMakeLists.txt
  bindings/
    module.cpp
    mesh_bindings.cpp
    mesh_geo_bindings.cpp
    sparse_bindings.cpp
    energy_bindings.cpp
    solver_bindings.cpp
    sim_bindings.cpp
    ndarray_utils.h
```

**NumPy contract:**

```python
vertices = np.asarray(vertices, dtype=np.float64)
tets     = np.asarray(tets,     dtype=np.int64)
cubes    = np.asarray(cubes,    dtype=np.int64)
```

The public Python layer may accept broader array-like inputs; the private `_core` layer receives validated CPU-contiguous arrays with known dtype and shape.

**Boundary rules (mesh types):**

- `MeshData` is the only intermediate conversion representation.
- `MeshGeo -> MeshData`: `.to_mesh_data()`.  `MeshData -> MeshGeo`: `.from_mesh_data(data)`.
- `VolumeMesh` accepts only `TetMeshData` or `CubicMeshData`.
- I/O returns/accepts `MeshData`; rejects `MeshGeo` façade objects.
- Target `pypgo.__all__` after the M1 mesh pipeline cleanup is `["mesh", "sim", "sparse", "tools"]` plus later milestones' `fem` / `energy` / `contact` modules as they land. `mesh_geo` and `io` are not retained as public modules.

### M1 Progress Status

| Item | Status |
|------|--------|
| `pypgo._core` nanobind module | Done |
| NumPy array validation (`_arrays.py`: `float_matrix`, `index_matrix`) | Done |
| `MeshData<K>` C++ + Python bindings (`TriMeshData`, `TetMeshData`, `CubicMeshData`) | Done |
| `TriMeshGeo` / `TetMeshGeo` / `CubicMeshGeo` C++ + Python bindings | Done |
| `MaterialSpec` + `VolumeMesh` bindings | Done |
| I/O bindings (`read_*_geo` / `write_*_geo` / `VolumeMesh.load`) | Done |
| Old public names removed | Done |
| C++ tests (MeshData, Tri/Tet/CubicMeshGeo) | Done |
| Python tests (mesh types, I/O, boundary checks) | Done |
| Domain-scoped package layout (`pypgo.mesh`, `pypgo.mesh.geo`, `pypgo.mesh.veg`, `pypgo.sim`; remove public `pypgo.io`) | Not done |
| Shell spec I/O (`pypgo.sim.read_shell` / `write_shell`) | Not done |
| Eigen dense vector/matrix ↔ NumPy conversion helpers | Not done |
| Eigen sparse matrix wrapper + COO export | Not done |
| `gil_scoped_release` policy for long-running kernels | Not done |
| `python -m pip install -e . --no-build-isolation` packaging check | Not done |

### M1 MeshGeo Utility Scope Decision

C++ `TriMeshGeo`/`TriMeshRef`、`TetMeshGeo`/`TetMeshRef`、`CubicMeshGeo` 有大量 utility 函数（法向量、面积、重心、距离查询、sub-mesh 提取、拓扑检查等），M1 刻意**一个都不绑**。

**设计原则：**

- M1 的 Geo 对象定位是"数据 façade + `to_mesh_data()` / `from_mesh_data()` 桥"，不是全功能 geometry kernel。
- 能用 NumPy/SciPy 高效做到的（表面积、法向量、重心），不值得绑 C++。
- 绑 C++ 的价值在于：避免数据搬运开销、复用经过生产验证的实现、与 C++ 仿真路径保持一致。

**分层绑定策略：**

| 优先级 | 函数 | 理由 |
|--------|------|------|
| **M2 第一批** | `TetNeighbor` | FEM assembly（Laplacian/Gradient）的前提 |
| | `computeTetDeterminant(tetID)` | FEM 单元质量、刚度矩阵积分 |
| | `computeTetBarycentricWeights(tetID, queryPos)` | FEM 插值、embedding |
| | `computeSurfaceArea()` | Contact、surface pressure 需要 |
| | `computeTriangleNormal(triID)` | Contact、rendering |
| | `getTriangleClosestPoint(triID, queryPos, feature)` | Contact detection |
| **M4+ 再考虑** | `computeMeshVolume`、`computeSolidInertiaParameters`、`getSubTriMesh`、`filterSmallComponents` | 场景构建/后处理时才需要 |
| **不绑** | `mergeMesh`、`triangulatePolygon`、`computeWindingNumber`、validation helpers 等 | Python 侧用 NumPy 就能做 |

**CubicMeshGeo 特别说明：** C++ `CubicMeshGeo` 本身只有存储 + `toMeshData()`，没有额外 utility，不需要特殊处理。

### Exit Criteria

- `import pypgo` and `import pypgo._core` work in the `libpgo` conda environment.
- The package installs with `python -m pip install -e . --no-build-isolation`.
- Native bindings are private; no old C-style functions appear at top level.
- NumPy ⇄ Eigen dense conversion is tested for `(n, 3)` and flat DOF vectors.
- Eigen sparse → COO export is tested.
- `gil_scoped_release` is applied to at least one long-running bound kernel and tested.
- All Progress Status items above are "Done".

## Milestone 2: Simulation Mesh, Sparse, and FEM Building Blocks

**Goal:** 在 M1 的 `MeshData` / `MeshGeo` 基础上恢复 old wrapper 的 sparse/FEM 能力，并沿用 domain-scoped module layout。

**Deliverables:**

- `pypgo.mesh` owns mesh data containers and surface OBJ I/O:
  - `TriMeshData`, `TetMeshData`, `CubicMeshData` own `(n, 3)` vertices and `(m, K)` elements.
  - `read_obj` / `write_obj` accept and return `TriMeshData`.
- `pypgo.mesh.geo` owns geometry façades and geometry algorithms:
  - `TriMeshGeo`, `TetMeshGeo`, `CubicMeshGeo` are typed façades with `to_mesh_data()` / `from_mesh_data()`.
- `pypgo.mesh.veg` owns Vega volume mesh I/O and material/region semantics:
  - `read_veg` returns `VegFile`, not a naked tuple.
  - `write_veg` writes `VegFile`.
  - `VolumeMesh` carries Vega volume semantics and does not expose solver-ready conversion methods.
- `pypgo.sim`
  - `SimulationMesh.create_volumetric(volume_mesh)` converts Vega volume to solver-ready mesh.
  - `SimulationMesh.create_shell(surface, material)` creates solver-ready shell mesh.
  - `read_shell` / `write_shell` persist `.shell.json + .obj` specs and return/accept `(TriMeshData, ShellMaterialLike)`.
- No `pypgo.io` public module is retained.
- `pypgo.sparse.SparseMatrix`
  - `.shape`
  - `.nnz`
  - `.to_coo()` returning NumPy arrays `(rows, cols, values)`
  - `.to_scipy_csr()` when SciPy is installed
  - `.to_torch_sparse_coo()` when PyTorch is installed
- `pypgo.fem`
  - `tet_laplacian(tet_mesh, face_neighbor=False, repeat=1, scale=False)`
  - `tet_gradient(tet_mesh)`
  - `tet_gradient_per_element(tet_mesh)`
  - `tet_biharmonic_gradient(tet_mesh, face_neighbor=True, scale=False)`
- MeshGeo utility bindings — M2 第一批，按 M1 决策表中优先级执行：
  - `TetNeighbor` 类：`getTetNeighbors(tetID)`、`findTetBoundaries()`
  - `TetMeshGeo.compute_tet_determinant(tetID)` → `float`
  - `TetMeshGeo.compute_tet_barycentric_weights(tetID, queryPos)` → `(4,)` array
  - `TriMeshGeo.compute_surface_area()` → `float`
  - `TriMeshGeo.compute_triangle_normal(triID)` → `(3,)` array
  - `TriMeshGeo.get_triangle_closest_point(triID, queryPos, feature)` → `(3,)` array
  - 以上暴露为 `_core` 方法，Python wrapper 提供对应 public API

**Example target API:**

```python
import pypgo as pgo

veg = pgo.mesh.veg.read_veg("box.veg")
mesh_data = veg.mesh_data
assert mesh_data.mesh_type in {pgo.mesh.MeshDataType.Tet, pgo.mesh.MeshDataType.Cubic}

volume = pgo.mesh.veg.VolumeMesh(mesh_data, regions=veg.to_volume_regions())
sim_mesh = pgo.sim.SimulationMesh.create_volumetric(volume)

if mesh_data.mesh_type == pgo.mesh.MeshDataType.Tet:
    tet_geo = pgo.mesh.geo.TetMeshGeo.from_mesh_data(mesh_data)
    L = pgo.fem.tet_laplacian(tet_geo, repeat=3)
    rows, cols, values = L.to_coo()

surface, shell_mat = pgo.sim.read_shell("cloth.shell.json")
shell_sim_mesh = pgo.sim.SimulationMesh.create_shell(surface, shell_mat)
```

**Exit criteria:**

- Old `pypgo.create_tetmeshgeo*` workflows are expressible without `create_*` / `destroy_*`.
- Existing tet matrix smoke coverage is restored under Python-first tests.
- NumPy is the canonical API surface for mesh vertices/elements, FEM vectors, and sparse COO data.
- SciPy and PyTorch adapters are tested as optional integrations, not required imports.
- Simulation mesh construction uses `MeshData` from M1 as the single data source.
- No simulation setup code depends on duplicate Python-only geometry wrappers; this milestone remains a reusable foundation.

## Milestone 3: Energy and Solver API

**Goal:** Make optimization workflows usable from Python before migrating the full simulation loop.

**Deliverables:**

- `pypgo.energy.PotentialEnergy` / energy protocol for solver-facing energy objects; full Python programming/plugin API is deferred to `plan/python_api_migration/future_work.md`.
- `pypgo.energy.EnergySet` for composing weighted energies.
- `pypgo.solver.solve_newton`
- `pypgo.solver.NewtonOptions`
- Python-facing `SolverResult`, `SolveStatus`, and diagnostics.
- Initial contact energies:
  - IPC surface energy
  - floor energy
  - sampled penalty contact energy

**Example target API:**

```python
x = tet.rest_positions_flat()
energy = pgo.energy.EnergySet([elastic, floor])
result = pgo.solver.solve_newton(energy, x, max_iter=50, tol=1e-6)
```

**Exit criteria:**

- A Python test can build a small energy, solve with Newton, and inspect status/iterations.
- Static solve building blocks are available without invoking the C++ CLI.
- Contact/floor energies can be composed with other energies from Python.

## Milestone 4: Python Run-Sim Config and Context Builder

**Goal:** Load existing `runIPCSim` JSON configs in Python and build the same simulation context.

**Deliverables:**

- `pypgo.sim.RunSimConfig`
  - load from JSON path
  - resolve paths relative to config
  - expose typed fields mirroring `RunIPCSimRuntimeConfig`
- `pypgo.sim.SimulationContext`
  - wraps the C++ `IpcSimulationContext`
  - exposes rest positions, surface mesh, mass matrix, energies, contact backend summary
- `pypgo.sim.from_config(path, contact_model="ipc" | "sampled_penalty")`
  - supports shell IPC
  - supports tet/cubic volume IPC
  - supports tet/cubic volume sampled penalty backend
  - does not support legacy shell; legacy shell is reserved for a future standalone legacy sim Python script

**C++ boundary:**

Use existing `runIPCSimCore` service functions where possible:

- `parseRunIPCSimRuntimeConfig`
- `buildRunIPCSimSimulation`
- `buildShellIpcSimulation`
- `buildVolumeIpcSimulation`
- `buildVolumeLegacyPenaltySimulation`

Bind these as private `_core` services, then wrap them with Python classes.

If these functions still expose too much C++ implementation detail, add an intermediate C++ facade before binding. For example, prefer binding a compact `SimulationContextHandle` service that exposes rest state, surface state, mass matrix, backend kind, and energy handles over binding every field of `IpcSimulationContext` directly.

**Exit criteria:**

- Python can load every representative config in `examples/ipc`.
- Python context construction matches CLI setup for dimensions, mesh counts, output paths, and backend selection.
- Any C++ context-building refactor needed for clean Python ownership has C++ parity tests before Python tests rely on it.
- No time stepping is required yet.

## Milestone 5: Static Solve Migration

**Goal:** Implement Python-driven static simulation first, because it has the smallest run loop surface.

**Deliverables:**

- `pypgo.sim.StaticSimulation`
- `pypgo.sim.solve_static(config_or_context, ...)`
- Python output writing for:
  - `states/deform0000.u`
  - `surface/ret0000.obj`
  - optional logs/diagnostics
- Parity tests against `runIPCSim` static cases.

**Why static first:**

Static mode exercises config parsing, context setup, energies, solver, output, and convergence policy without requiring per-frame integrator/session state.

**Exit criteria:**

- Python static solve succeeds for supported attached shell/tet/cubic cases.
- Python static solve rejects expected non-converged gravity-only cases in the same way as the C++ CLI.
- Output layout matches `runIPCSim`.

## Milestone 6: Dynamic Loop Migration

**Goal:** Move dynamic timestep orchestration into Python while keeping C++ integrator kernels.

**Deliverables:**

- `pypgo.sim.DynamicSimulation`
- `simulation.step()`
- `simulation.run(num_steps=None)`
- Python `Frame` object containing:
  - frame index
  - displacement
  - velocity
  - acceleration
  - surface mesh snapshot
  - solver result / diagnostics
- Support for:
  - frame gap
  - restart-from-u
  - dump-deform-every-frame
  - floor motion
  - IPC backend hooks
  - volume legacy penalty backend hooks
  - surface pressure force
  - von Mises stress output

**Exit criteria:**

- Python dynamic runner reproduces the existing CLI output layout.
- Representative shell/tet/cubic dynamic examples run from Python.
- Restart behavior is covered by tests.
- Solver acceptance policy matches current `runIPCSim` dynamic semantics.
- Per-step C++ APIs are clean enough that Python owns orchestration without manually reproducing internal contact/session side effects.

## Milestone 7: Python CLI Replacement

**Goal:** Provide a Python CLI that can replace the C++ executable for normal workflows.

**Deliverables:**

- Console entrypoint:

```bash
pypgo-run-sim examples/ipc/tet/box-hang/box-ipc.json
pypgo-run-sim --legacy examples/path/to/legacy-volume.json
```

- Module entrypoint:

```bash
python -m pypgo.tools.run_sim examples/ipc/tet/box-hang/box-ipc.json
```

- CLI options compatible with current `runIPCSim` where practical:
  - `--legacy`
  - `--log`
  - future Python-only flags for dry run, inspect config, override output, override steps

**Exit criteria:**

- Existing batch scripts can choose the Python runner.
- C++ `runIPCSim` is still available as a reference executable.
- README documents Python as the primary workflow.

## Milestone 8: PyTorch and Research Workflow Integration

**Goal:** Make simulation components usable in ML/research loops.

This milestone builds on the NumPy contracts from Milestones 1 and 2. PyTorch should not replace NumPy as the base API; it should provide tensor adapters for users who need autograd or ML training loops.

**Deliverables:**

- `pypgo.torch.energy_value(energy, x)`
- torch autograd bridge using C++ `func()` and `gradient()`
- sparse matrix conversion to `torch.sparse_coo_tensor`
- CPU tensor to NumPy bridge for mesh/state inputs where zero-copy is safe, and explicit copy where tensor layout/device requires it.
- helpers for optimizing material/contact parameters from Python
- examples showing PGO energy inside a PyTorch loss

**Exit criteria:**

- CPU tensor workflow is tested.
- Optional PyTorch dependency is lazy; `import pypgo` does not require PyTorch.
- Energy-level interop works before attempting solver-level differentiation.

## Milestone 9: Deprecation and Cutover

**Goal:** Make Python the primary run-sim interface while keeping C++ stable for core development.

**Deliverables:**

- Mark C++ `runIPCSim` executable as reference / compatibility path in docs.
- Update examples to show Python-first usage first.
- Keep C++ tests for kernel behavior.
- Keep Python parity tests for workflow behavior.
- Remove or archive stale docs that describe old Python wrapper or C-style API.

**Exit criteria:**

- New users can run documented simulations entirely from Python.
- Existing C++ CLI remains available for debugging and performance comparison.
- The migration has a clear owner for each remaining uncovered feature.

## Future Work

完整 Python programming / plugin API、nanobind trampoline for Python-defined C++ abstract bases、JIT/tracing/compiled user kernels 都不阻塞 M1-M7 的 run-sim 迁移主线。它们统一记录在 `plan/python_api_migration/future_work.md`，M9 之后再单独设计。

## Suggested Execution Order

1. Milestone 0: inventory and parity matrix.
2. Milestone 1: private native binding foundation.
3. Milestone 2: mesh/sparse/FEM.
4. Milestone 3: energy/solver.
5. Milestone 4: config/context builder.
6. Milestone 5: static solve.
7. Milestone 6: dynamic loop.
8. Milestone 7: Python CLI.
9. Milestone 8: PyTorch integration.
10. Milestone 9: deprecation/cutover.

## Current Next Step

M0 的规划文档已经具备：coverage matrix、parity matrix、NumPy data contract、future work 都已成形。

M1 核心实现已完成：`MeshData<K>` 数据容器、`TriMeshData`/`TetMeshData`/`CubicMeshData` Python 类型、`TriMeshGeo`/`TetMeshGeo`/`CubicMeshGeo` façade、`MaterialSpec`、`VolumeMesh`（接受 MeshData）、I/O（收发 MeshData）、旧 public names 移除、C++ 和 Python 测试覆盖。

下一步进入 M2：Sparse Matrix 和 FEM Building Blocks，恢复 sparse/FEM 能力并补齐 geometry queries。
