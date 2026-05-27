# Phase 2 Implementation Record

Status: implementation complete, pending review.

## Files Created

| File | Purpose |
|------|---------|
| `src/core/contact/ipc/external/obstacleSurface.h` | `ObstacleSurface` class + `makeLinearTrajectorySampler` |
| `src/core/contact/ipc/external/obstacleSurface.cpp` | Implementation: update, uniqueEdge derivation, sampler factory |
| `src/tests/testIPCExternal/testIPCExternal.cpp` | Unit tests: ObstacleSurface, registration, barrier, CCD, wrapper |
| `src/tests/testIPCExternal/CMakeLists.txt` | Build for test executable |

## Files Modified

| File | Changes |
|------|---------|
| `ipc/core/surfaceIPCPairs.h` | Added `ExternalPTPair`, `ExternalTPPair`, `ExternalEEPair` |
| `ipc/core/surfaceIPCCore.h` | Added `dhat_external`, `obstacles_`, `extPTPairs_/extTPPairs_/extEEPairs_`, obstacle APIs |
| `ipc/core/surfaceIPCCore.cpp` | External pair building (broad-phase), external CCD, obstacle registration |
| `ipc/core/surfaceIPCBarrierAssembler.h` | External barrier assembly method declarations |
| `ipc/core/surfaceIPCBarrierAssembler.cpp` | External barrier energy/gradient/hessian/all + scatter helpers |
| `embeddedSurfaceIPCPotentialEnergy.h` | Wrapper obstacle APIs |
| `embeddedSurfaceIPCPotentialEnergy.cpp` | Wrapper forwards to core |
| `tools/runSim/runIPCSimSetup.cpp` | `parseExternalObjects` helper, `ipc-dhat-external`, phase1D→phase2 messages |
| `tools/runSim/runIPCSim.cpp` | `updateObstacleStage` + `invalidatePreparedState` per timestep |
| `contact/CMakeLists.txt` | Added `obstacleSurface.h/.cpp` |
| `tests/CMakeLists.txt` | Added `testIPCExternal` |

## Key Design Decisions

1. **PT split by orientation**: `ExternalPTPair` (dyn vertex × obs triangle) vs `ExternalTPPair` (dyn triangle × obs vertex)
2. **Dynamic-only block scatter**: Three type-specialized helpers (`scatterExternalPTGrad/Hessian`, etc.)
3. **`dhat_external` separate from `dhat`**: Defaults to same value for self-only numerical invariance
4. **PSD-before-crop**: Full 12×12 PSD projection before cropping to dynamic-only block (known conservative; see plan §5.3)
5. **α semantics**: Obstacle endpoint fixed at `current`, α only scales dynamic Newton direction
6. **Stage-aware obstacle motion**: `updateObstacleStage(tStart, tEnd)` sets `previous = sampler(tStart)`, `current = sampler(tEnd)`

## Test Coverage

| Test | What it covers |
|------|----------------|
| `test_obstacleSurface_basic` | Construction, update, unique_edges derivation |
| `test_obstacleSurface_zero_velocity` | Zero-velocity sampler regression baseline |
| `test_surfaceIPCCore_external_register_clear` | addObstacleSurface, clearObstacleSurfaces, self-path invariance |
| `test_surfaceIPCCore_external_static_plane` | Static plane barrier energy/gradient/hessian |
| `test_surfaceIPCCore_external_box_contact` | PT/TP/EE pair coverage, obstacleObjectId validation |
| `test_surfaceIPCCore_external_multi_obstacle` | obstacleObjectId distinguishes same local indices |
| `test_surfaceIPCCore_external_self_equivalence` | External energy vs self-IPC with locked DOFs |
| `test_surfaceIPCCore_external_ccd_kinematic` | Kinematic obstacle CCD |
| `test_surfaceIPCCore_external_ccd_alpha_symmetry` | α scaling invariant |
| `test_embeddedSurfaceIPCPotentialEnergy_external` | Wrapper obstacle APIs |
| `test_embeddedSurfaceIPCPotentialEnergy_wrapper_vs_core` | Wrapper vs core consistency |

## Known Limitations (Deferred)

- PSD-before-crop conservative (Phase 4)
- Per-obstacle `dhat_external` not supported (Phase 4+)
- `obstacleObjectId` only process-stable, not cross-process (Phase 3)
- TRBDF2 integration not included (Phase 3C)
- Friction not included (Phase 3)
