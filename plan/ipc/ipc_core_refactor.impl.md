# IPC Core Refactor Implementation Record

Source plan:
- `plan/ipc/ipc_core_refactor.plan.md`

## Implemented Scope

Implemented R1-R5 as behavior-preserving refactors.

### R1: Geometry Math Extraction

Added `src/core/contact/ipc/geometry/`:

- `ipcDistancePrimitives.h/.cpp`
- `ipcBarrier.h/.cpp`
- `ipcCCD.h/.cpp`
- `ipcHessianProjection.h/.cpp`

`ipc/core/surfaceIPCCore.h` now includes these geometry headers instead of
declaring all geometry functions inline in the facade header. Function names,
namespaces, and signatures remain in `pgo::Contact::CIPC`.

Focused test:

- `tests/src/core/contact/ipcGeometry_gtest.cpp`

### R2: Broad Phase Directory Migration

Moved the extracted spatial hash implementation into:

- `src/core/contact/ipc/broadPhase/spatialHashGrid.h/.cpp`

No forwarding compatibility header is kept. Repo includes were migrated to the
real IPC path:

- `#include "ipc/broadPhase/spatialHashGrid.h"`

Focused test:

- `tests/src/core/contact/spatialHashGrid_gtest.cpp`

### R3: Surface Topology Extraction

Added `SurfaceIPCTopology` in:

- `src/core/contact/ipc/topology/surfaceIPCTopology.h/.cpp`

`SurfaceIPCCore::setMesh()` now delegates topology, edge, area, and length build
to this value object. The unused `allDOFs_`, `vertexTriAdj_`, `edgeVertAdj_`,
and `buildAdjacency()` path was removed from `SurfaceIPCCore`.

Focused test:

- `tests/src/core/contact/surfaceIPCTopology_gtest.cpp`

### R4: Self Broad Phase Extraction

Added `SurfaceIPCSelfBroadPhase` in:

- `src/core/contact/ipc/broadPhase/surfaceIPCSelfBroadPhase.h/.cpp`

`SurfaceIPCCore::findCollisionPairs()` is now a facade wrapper around the helper.
PT/EE candidate build, filtering, weights, and ordering behavior were preserved.

Focused test:

- `tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest.cpp`

### R5: Max-Step And Assembly Split

Added:

- `src/core/contact/ipc/core/surfaceIPCMaxStep.h/.cpp`
- `src/core/contact/ipc/core/surfaceIPCBarrierAssembler.h/.cpp`

`SurfaceIPCMaxStep` computes the CCD-based contact feasible alpha. Contact clamp
stats and logging remain owned by `SurfaceIPCCore`.

`SurfaceIPCBarrierAssembler` computes energy, gradient, Hessian, and combined
assembly from an already-built active pair set. `SurfaceIPCCore` still owns the
public lifecycle: it builds active pairs, then delegates assembly.

Focused tests:

- `tests/src/core/contact/surfaceIPCMaxStep_gtest.cpp`
- `tests/src/core/contact/surfaceIPCBarrierAssembler_gtest.cpp`

### No-Forwarding IPC Directory Migration

Moved IPC public implementation headers into the `ipc/` tree without top-level
forwarding headers:

- `src/core/contact/ipc/core/surfaceIPCCore.h/.cpp`
- `src/core/contact/ipc/profiling/surfaceIPCProfiling.h`
- `src/core/contact/ipc/geometry/generated/CIPC_autogen.h`
- `src/core/contact/ipc/geometry/generated/CIPC_autogen_ll.h`

Added:

- `src/core/contact/ipc/core/surfaceIPCPairs.h`

`PTPair` and `EEPair` now live in `surfaceIPCPairs.h`, so lower-level IPC
helpers can share pair types without including the `SurfaceIPCCore` facade.
Internal repo includes were updated to the real `ipc/...` paths.

## Intentional Drift / Clarifications

- R5 was implemented in the same refactor batch after R1-R4 tests passed,
  but it remains behavior-preserving and does not introduce prepared-state cache.
- `SurfaceIPCCore` moved to `ipc/core/` as part of the no-forwarding directory
  cleanup.
- `PTPair` and `EEPair` moved to `ipc/core/surfaceIPCPairs.h` to avoid reverse
  dependencies from IPC helpers back into the facade.
- No top-level forwarding headers are retained for the moved IPC files. Existing
  in-repo users were migrated; external include users should include the real
  `ipc/...` headers.

## Validation

Validated with:

```bash
cmake --preset base_no_mkl
cmake --build build/base_no_mkl --target ipcGeometry_gtest surfaceIPCTopology_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCMaxStep_gtest surfaceIPCBarrierAssembler_gtest spatialHashGrid_gtest surfaceIPCCore_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest -j2
./build/base_no_mkl/tests/src/core/contact/ipcGeometry_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCTopology_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCMaxStep_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCBarrierAssembler_gtest
./build/base_no_mkl/tests/src/core/contact/spatialHashGrid_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
cmake --build build/base_no_mkl --target surfaceIPCCore_gtest embeddedSurfaceFloorPotentialEnergy_gtest runIPCSim_gtest -j2
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
git diff --check
```

All listed tests passed.

Additional no-forwarding validation:

```bash
cmake --preset base_no_mkl
cmake --build build/base_no_mkl --target ipcGeometry_gtest surfaceIPCTopology_gtest surfaceIPCSelfBroadPhase_gtest surfaceIPCMaxStep_gtest surfaceIPCBarrierAssembler_gtest spatialHashGrid_gtest surfaceIPCCore_gtest cipcProfiling_gtest cipcPotentialEnergy_gtest embeddedSurfaceIPCPotentialEnergy_gtest -j2
./build/base_no_mkl/tests/src/core/contact/ipcGeometry_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCTopology_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCSelfBroadPhase_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCMaxStep_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCBarrierAssembler_gtest
./build/base_no_mkl/tests/src/core/contact/spatialHashGrid_gtest
./build/base_no_mkl/tests/src/core/contact/surfaceIPCCore_gtest
./build/base_no_mkl/tests/src/core/contact/cipcProfiling_gtest
./build/base_no_mkl/tests/src/core/contact/cipcPotentialEnergy_gtest
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceIPCPotentialEnergy_gtest
cmake --build build/base_no_mkl --target embeddedSurfaceFloorPotentialEnergy_gtest runIPCSim_gtest -j2
./build/base_no_mkl/tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest
./build/base_no_mkl/tests/src/tools/runIPCSim_gtest
test ! -e src/core/contact/surfaceIPCCore.h
test ! -e src/core/contact/surfaceIPCCore.cpp
test ! -e src/core/contact/surfaceIPCProfiling.h
test ! -e src/core/contact/spatialHashGrid.h
test ! -e src/core/contact/CIPC_autogen.h
test ! -e src/core/contact/CIPC_autogen_ll.h
! rg -n '#include "(surfaceIPCCore|surfaceIPCProfiling|spatialHashGrid|CIPC_autogen|CIPC_autogen_ll)\.h"|../../surfaceIPCCore' src tests
git diff --check
```

All listed tests and no-forwarding checks passed.
