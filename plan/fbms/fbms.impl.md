# FBMS Implementation Record

Source plan: `plan/fbms/fbms.plan.md`

This file records implemented behavior and validation results for completed FBMS phases. Future-phase planning remains in the source plan.

## Current Alignment Snapshot

Status: source plan realigned with repo truth on April 25, 2026.

Current repo truth:

- FBMS asset generation uses `examples/fbms/generate_shell_assets.py` plus `generateFBMSUnionSurface` and `remeshSurface cgal_iso`.
- Generated assets live under `examples/fbms/generated/<job>/<case>/`; the current `g0_b8 + r128_default` directory already contains `union_shell_remesh.obj`, `union_shell.veg`, and `union_shell_tet_surface.obj`.
- `tetMesher` is JSON-only: `tetMesher --config path/to/tetmesh.json`.
- Old `tetMesher tetgen ...` and `tetMesher tetwild ...` subcommands are intentionally rejected.
- TetWild uses the fTetWild C++ API directly, with `PGO_TET_MESHER_USE_TET_WILD` defaulting to `ON`; the default pipeline does not produce or consume `.msh`.
- `runIPCSim` already writes timestep output under `states/`, `surface/`, and `stress/`.
- `runIPCSim` already supports volume-path `output-von-mises`; shell path rejects that flag.
- `runIPCSim` supports multi-floor lower/upper floor penalty prototypes through `floors[]`, including per-frame linear height motion.
- `runIPCSim` still rejects `external-objects`, so formal Case 2 and Case 3 external IPC remain future work.

Next implementation target:

- Phase E / external IPC contact can now use the Phase D floor prototypes as behavior and stress-output baselines.
- `runIPCSim` still rejects `external-objects`, so formal Case 2/3 external IPC remains future work.

## 3.3 SDF Thickening + Union Surface Generator

Status: implemented.

Implemented files:

- `src/tools/fbms/CMakeLists.txt`
- `src/tools/fbms/generateFBMSUnionSurface.cpp`
- `src/tools/CMakeLists.txt`
- `tests/src/tools/generateFBMSUnionSurface_gtest.cpp`
- `tests/src/tools/CMakeLists.txt`

Implemented behavior:

- Adds `generateFBMSUnionSurface` under `src/tools/fbms` when `libiglInterface` is available.
- CLI accepts `--fbms`, `--sphere`, `--fbms-thickness`, `--sphere-thickness`, `--resolution`, `--padding-ratio`, and `--output-surface`.
- Loads FBMS and sphere OBJ meshes as `TriMeshGeo`.
- Computes FBMS field as `unsigned_distance_to_fbms - fbms_thickness / 2` using `libiglInterface::computeDistanceField(..., robust=1, sign=0)`.
- Parses sphere parameters from OBJ header lines `# center = (...)` and `# radius = ...`; falls back to sphere OBJ bbox center plus max vertex distance when header parsing fails.
- Prints sphere parameter source, center, and radius to stdout.
- Builds the grid bbox from the FBMS/sphere bbox union, expanded by `max(fbms_thickness, sphere_thickness) + padding_ratio * bbox_diag`.
- Computes `f_union = min(f_fbms, f_sphere_shell)` where `f_sphere_shell = abs(norm(x - center) - radius) - sphere_thickness / 2`.
- Runs `libiglInterface::computeMarchingCubes(...)` on the union field and writes the raw OBJ surface.
- Prints grid resolution, bbox min/max, raw vertex/face counts, and field min/max.
- Returns an error if inputs are invalid, the field does not cross zero, marching cubes returns an empty surface, or output cannot be written.

Validation:

```bash
cmake -S . -B build/base_no_mkl
cmake --build build/base_no_mkl --target generateFBMSUnionSurface_gtest -j 4
ctest --test-dir build/base_no_mkl -R GenerateFBMSUnionSurfaceCli --output-on-failure
```

Result: passed on April 24, 2026.

Asset smoke test:

```bash
build/base_no_mkl/bin/generateFBMSUnionSurface \
  --fbms examples/fbms/g0_b8/g0_b8_fbms.obj \
  --sphere examples/fbms/g0_b8/g0_b8_fbms_bounding_sphere.obj \
  --fbms-thickness 0.02 \
  --sphere-thickness 0.02 \
  --resolution 128 \
  --padding-ratio 0.08 \
  --output-surface examples/fbms/g0_b8/g0_b8_union_shell_raw.obj
```

Result: passed on April 24, 2026. It produced `144464` vertices, `289040` faces, and a 14 MB raw OBJ. The raw surface bbox was approximately `[-1.00331, -1.00309, -1.00324]` to `[1.00322, 1.00343, 1.00328]`, matching the parsed sphere radius plus half shell thickness.

Plan comparison and drift:

- Matches the planned SDF formulas, bbox expansion rule, header parsing plus bbox fallback, marching-cubes output, and required logging.
- The implementation adds input validation and explicit non-empty/zero-crossing failure modes so bad parameter combinations fail early instead of producing unusable assets.
- The regression test uses a small synthetic open FBMS patch plus header sphere.
- The `g0_b8` resolution `128` asset-generation smoke test passed and wrote the planned raw OBJ artifact. Resolution `256` remains to be run if the next phase wants the higher-resolution TetWild input.

## 3.5 fTetWild Tet Mesher Backend + Generated Surface Export

Status: implemented.

Implemented files:

- `CMakeLists.txt`
- `CMakeModules/third-party/ftetwild.cmake`
- `src/tools/tetMesher/CMakeLists.txt`
- `src/tools/tetMesher/tetMesher.cpp`
- `src/tools/tetMesher/tetMesherBackend.h`
- `src/tools/tetMesher/tetMesherBackend.cpp`
- `src/tools/tetMesher/tetgenBackend.cpp`
- `src/tools/tetMesher/tetwildBackend.cpp`
- `src/tools/tetMesher/tetwildBackendDisabled.cpp`
- `tests/src/tools/CMakeLists.txt`
- `tests/src/tools/tetMesher_gtest.cpp`

Implemented behavior:

- Adds `PGO_TET_MESHER_USE_TET_WILD`, default `ON`.
- When enabled, integrates `wildmeshing/fTetWild` through `FetchContent` at commit `d7d99bb4387a07895b9adce058dc7305f6b6e5ab` and links `tetMesher` against `FloatTetwild`.
- Adds a small CMake compatibility overlay for this fTetWild commit with current libigl predicates headers.
- Refactors `tetMesher` into a JSON-only job-config entry point:
  - `tetMesher --config path/to/tetmesh.json`
  - old `tetMesher tetgen ...` and `tetMesher tetwild ...` subcommand entries are intentionally rejected.
- Keeps backend implementation details out of the shared `tetMesherBackend.h` boundary.
- `tetwild` uses the fTetWild C++ API directly. It does not shell out, does not add a CLI fallback, and does not implement `--keep-msh`.
- Shared save logic writes `.veg`, reload-checks the saved mesh, and supports `output_surface` by extracting the generated tet mesh boundary surface as OBJ.
- Shared JSON fields are `version`, `backend`, `input_mesh`, `output_mesh`, `output_surface`, `print_stats`, and `quiet`; paths are resolved relative to the JSON config file.
- `tetgen.command` stores the TetGen command string.
- `tetwild` supports `lr`, `la`, `epsr`, `stop_energy`, and `max_threads` under the `tetwild` JSON object.
- fTetWild internal scratch surface files are given a deterministic temporary prefix and removed after the API call.

Validation:

```bash
cmake -S . -B build/tetwild_off -DPGO_TET_MESHER_USE_TET_WILD=OFF -DPGO_ENABLE_FULL=ON
cmake --build build/tetwild_off --target tetMesher_gtest -j 4
ctest --test-dir build/tetwild_off -R TetMesherCli --output-on-failure
```

Result: passed on April 25, 2026. The explicit `OFF` build keeps `tetgen` working and reports a clear disabled-backend error for `tetwild`.

```bash
cmake --preset base_no_mkl
cmake --build --preset base_no_mkl_release --target tetMesher_gtest
ctest --test-dir build/base_no_mkl -R TetMesherCli --output-on-failure
```

Result: passed on April 25, 2026. The `ON` build compiles `FloatTetwild`, runs the TetGen regression, runs the TetWild closed-cube `.veg` plus boundary OBJ test, and rejects the old subcommand entry.

Asset smoke tests:

```bash
build/base_no_mkl/bin/tetMesher --config examples/fbms/generated/r128_default/g0_b3/tetmesh.json
```

with `examples/fbms/generated/r128_default/g0_b3/tetmesh.json` equivalent to:

```json
{
  "version": 1,
  "backend": "tetwild",
  "input_mesh": "union_shell_remesh.obj",
  "output_mesh": "union_shell.veg",
  "output_surface": "union_shell_tet_surface.obj",
  "print_stats": true,
  "quiet": true,
  "tetwild": {
    "lr": 0.05,
    "epsr": 0.001,
    "stop_energy": 10,
    "max_threads": 8
  }
}
```

Result: passed again through the JSON config entry on April 25, 2026.

- `.veg`: `11194` vertices, `35273` tets.
- boundary OBJ: `10931` vertices, `21858` faces.
- bbox min: `[-1.0128, -1.01283, -1.01224]`
- bbox max: `[1.01257, 1.01284, 1.01281]`

```bash
build/base_no_mkl/bin/tetMesher --config examples/fbms/generated/r128_default/g0_b8/tetmesh.json
```

with `examples/fbms/generated/r128_default/g0_b8/tetmesh.json` equivalent to the same TetWild JSON body above.

Result: passed on April 25, 2026.

- `.veg`: `14728` vertices, `46682` tets.
- boundary OBJ: `14334` vertices, `28684` faces.
- bbox min: `[-1.01154887629, -1.0114965584, -1.01188750384]`
- bbox max: `[1.01131908692, 1.01214722237, 1.01198672584]`

Plan comparison and drift:

- 3.6 `.msh -> .veg` conversion enhancement was intentionally not implemented in this phase.
- The implemented path is direct C++ API only: surface OBJ to generated tet `.veg`.
- `output_surface` exports the generated tet mesh outer boundary, not a copy of the input surface.

## 4.1 Output Directory Layout and 4.2 von Mises Stress Output

Status: implemented.

Implemented files:

- `src/core/solidDeformationModel/deformationModelAssembler.cpp`
- `src/tools/runSim/runIPCSim.cpp`
- `src/tools/runSim/runIPCSimSetup.h`
- `src/tools/runSim/runIPCSimSetup.cpp`
- `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`
- `tests/src/tools/runIPCSim_gtest.cpp`

Implemented behavior:

- `runIPCSim` now creates `states/`, `surface/`, and `stress/` under the configured output root after clearing or creating the output folder.
- `runIPCSim.log` remains at the output root when `--log` is enabled.
- `restart-from-u=true` now searches restart states under `states/deformXXXX.u`.
- Deformation state is written every timestep to `states/deformXXXX.u`.
- Surface OBJ dumps are written to `surface/retXXXX.obj` on the configured `dump-interval`; the output root no longer receives new root-level `deformXXXX.u` or `retXXXX.obj` timestep files.
- Adds `output-von-mises`; when enabled for a tet/cubic volume simulation, each timestep writes `stress/von_misesXXXX.json`.
- Stress JSON contains `frame`, `time`, `stress_type`, `location`, and `values`. For tet meshes, `location` is `tet_element` and `values[i]` maps to tet element `i`.
- `DeformationModelAssembler::computeVonMisesStresses(...)` now gathers each element's absolute local positions, plastic params, and elastic params, calls `prepareData(...)`, and then calls the element deformation model's `vonMisesStress(...)`.
- Tet elements write their single integration-point stress directly. Cubic elements write the maximum over their integration-point stresses.
- `runIPCSim` stores the simulation plastic/elastic params in `IpcSimulationContext` so timestep stress output uses the same material state as the elastic energy.

Validation:

```bash
cmake --build build/base_no_mkl --target deformationModelAssembler_gtest runIPCSim_gtest -j 4
ctest --test-dir build/base_no_mkl -R 'DeformationModelAssemblerGTest|RunIPCSim' --output-on-failure
```

Result: passed on April 25, 2026. This covered:

- tet assembler von Mises stress is near zero at rest and nonzero under a simple stretch;
- restart reads from `states/`;
- zero-timestep output creates the new semantic subdirectories without timestep files;
- tet/cubic one-step smoke runs write state and surface outputs into subdirectories;
- `dump-interval > 1` keeps state output every step and surface output only on dump frames;
- `output-von-mises=true` writes parseable tet element stress JSON with `values.size()` equal to the tet element count.

Plan comparison and drift:

- Matches the planned output layout and restart path migration.
- Applies the new output directory layout to shell, tet, and cubic `runIPCSim` paths so existing runIPCSim behavior stays consistent across paths.
- Narrows `output-von-mises` to the volume path; shell simulations now reject that flag instead of writing misleading tet-element stress output.
- Extends the assembler implementation to cubic elements by taking the maximum integration-point von Mises stress, while FBMS still relies on the tet path.

## 4.3 Surface Pressure Force / Case 1

Status: implemented.

Implemented files:

- `src/tools/runSim/runIPCSimSetup.h`
- `src/tools/runSim/runIPCSimSetup.cpp`
- `src/tools/runSim/runIPCSim.cpp`
- `src/core/simulation/implicitBackwardEulerTimeIntegrator.cpp`
- `tests/src/tools/runIPCSim_gtest.cpp`
- `examples/fbms/generated/r128_default/g0_b8/g0_b8_case1_pressure-ipc.json` (workspace file under the ignored generated-assets tree)

Implemented behavior:

- Adds volume-path `surface-pressure-force` config parsing:
  - `enabled=false` is a no-op;
  - enabled configs require `center`, `pressure`, and positive `ramp-steps`;
  - enabled shell-path configs throw a clear volume-only error.
- Computes rest-surface vertex areas with `TriMeshRef::computeVertexSurfaceAreas(...)`.
- Builds surface force as `pressure * area_i * normalize(center - x_rest_i)`.
- Skips and warns for surface vertices closer than `1e-12` to the pressure center.
- Projects surface force to simulation DOFs through `surfaceFromSimulationDispMap.transpose()`.
- Stores the projected force and ramp metadata in `IpcSimulationContext`.
- Updates `runIPCSim` external force every frame as `M g + ramp(frame) * f_pressure`, where `ramp(frame) = min(1, (frame + 1) / ramp_steps)`.
- Keeps the no-pressure path equivalent to the previous `M g` external force behavior.
- Treats finite non-converged implicit solves with accepted Newton iterates (`MaxIterations` and `StepTooSmall`) as accepted timesteps; `NonFinite`, `LinearSolveFailed`, and `LineSearchFailed` remain rejected. This matches the intended unconditional-accept behavior for tiny-step pressure smoke runs while still rejecting genuinely invalid solver states.
- Adds Case 1 config at `examples/fbms/generated/r128_default/g0_b8/g0_b8_case1_pressure-ipc.json` with:
  - `fixed-vertices: []`;
  - zero gravity and zero initial velocity;
  - `output-von-mises: true`;
  - `pressure: 1000.0`;
  - `ramp-steps: 20`;
  - `num-timestep: 20` as the Phase C smoke/tuning run.

Validation:

```bash
cmake --build build/base_no_mkl --target implicitBackwardEulerTimeIntegrator_gtest runIPCSim_gtest -j 4
ctest --test-dir build/base_no_mkl -R 'ImplicitBackwardEulerTimeIntegrator|RunIPCSim' --output-on-failure
```

Result: passed on April 26, 2026. This covered:

- `MaxIterations` is accepted by the implicit backward Euler timestep wrapper and advances the timestep;
- enabled volume pressure builds a nonzero simulation-DOF force vector;
- disabled pressure is a no-op;
- `pressure=0` produces a zero projected pressure force;
- enabled shell pressure is rejected;
- one-step tet pressure CLI smoke writes state, surface, and nonzero stress JSON;
- existing finite solver-return cases are accepted by the timestep wrapper and logged with `accepted=true`.

FBMS Case 1 smoke:

```bash
build/base_no_mkl/bin/runIPCSim examples/fbms/generated/r128_default/g0_b8/g0_b8_case1_pressure-ipc.json --log
```

Result: passed on April 26, 2026.

- Wrote `20` state files under `case1_pressure_output/states/`.
- Wrote `20` surface OBJ files under `case1_pressure_output/surface/`.
- Wrote `20` von Mises JSON files under `case1_pressure_output/stress/`.
- Final stress file `von_mises0019.json` has `frame=19`, `time=0.0095`, `location=tet_element`, `45166` values, and all values are nonzero; max stress was approximately `133414.989`.

Plan comparison and drift:

- Matches the planned pressure formula, one-third surface area accumulation, `W^T` projection, frame ramp, positive-inward pressure convention, and empty `fixed-vertices` Case 1 config.
- The source plan already anticipated `num-timestep=20` as Phase C smoke validation even though section 5.1 showed `200` as a longer recommended run; the committed workspace config uses `20`.
- Implementation clarified the integrator-level accept policy required by the smoke run: finite `StepTooSmall` and `MaxIterations` solver returns now advance the timestep instead of throwing after Newton has produced an accepted iterate.
- The Case 1 config is placed under `examples/fbms/generated/...`; the JSON config itself is not ignored, while generated outputs under `*_output` remain ignored.

## 4.5 Floor Penalty Upgrade / Case 2 and 3 Floor Prototypes

Status: implemented.

Implemented files:

- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.h`
- `src/core/contact/embeddedSurfaceFloorPotentialEnergy.cpp`
- `src/tools/runSim/runIPCSimSetup.h`
- `src/tools/runSim/runIPCSimSetup.cpp`
- `src/tools/runSim/runIPCSim.cpp`
- `tests/src/core/contact/embeddedSurfaceFloorPotentialEnergy_gtest.cpp`
- `tests/src/tools/runIPCSim_gtest.cpp`
- `examples/ipc/cubic/box-with-sphere/box-ipc.json`
- `examples/ipc/README.md`
- `examples/fbms/generated/r128_default/g0_b8/g0_b8_case2_squash_floor-prototype-ipc.json`
- `examples/fbms/generated/r128_default/g0_b8/g0_b8_case3_wall_impact_floor-prototype-ipc.json`

Implemented behavior:

- Adds `FloorSide` with `LOWER` and `UPPER`.
- `EmbeddedSurfaceFloorPotentialEnergy` now evaluates the floor penalty with signed side semantics:
  - lower floors penalize `coord < height` and push toward positive axis;
  - upper floors penalize `coord > height` and push toward negative axis.
- Adds `setFloorHeight(double)` and `floorHeight()` so `runIPCSim` can update kinematic floor height between frames.
- Replaces the old top-level `use-floor` / `floor-axis` / `floor-height` / `floor-kappa` schema with `floors[]`.
- `floors[]` accepts zero or more floor objects. Each floor requires:
  - `axis`: `x`, `y`, or `z`;
  - optional `side`: `lower` by default, or `upper`;
  - `kappa`;
  - exactly one of static `height` or linear `motion`.
- Motion floors support `height-start`, `height-end`, `frame-start`, and `frame-end`; heights are linearly interpolated and clamped outside the frame interval.
- Old floor fields are explicitly rejected with a migration error instead of being ignored.
- Shell and volume IPC setup both add every parsed floor as an extra general implicit force model.
- `runIPCSim` updates moving floor heights once per frame before adding the floor models to the timestep.
- Migrates the existing IPC floor example to `floors[]`.
- Adds FBMS Case 2 and Case 3 floor-prototype configs in the generated `g0_b8` asset directory.

Validation:

```bash
cmake --build build/base_no_mkl --target embeddedSurfaceFloorPotentialEnergy_gtest runIPCSim_gtest -j 4
ctest --test-dir build/base_no_mkl -R 'EmbeddedSurfaceFloorPotentialEnergy|RunIPCSim' --output-on-failure
```

Result: passed on April 26, 2026. This covered:

- lower-side behavior remains compatible with the existing reference floor penalty;
- upper-side energy, gradient, and Hessian have the expected sign and curvature;
- `setFloorHeight` changes the next energy evaluation and rejects non-finite heights;
- `floors: []` is accepted;
- missing `axis` / `kappa`, invalid `height` plus `motion`, missing both `height` and `motion`, and legacy top-level floor fields are rejected;
- multiple floors create multiple implicit force models;
- a moving upper tet floor smoke run writes state output and nonzero von Mises stress.

FBMS prototype config setup smoke:

```bash
build/base_no_mkl/bin/runIPCSim /tmp/libpgo-fbms-phase-d-1777181624726/g0_b8_case2_squash_floor-prototype-ipc.json --log
build/base_no_mkl/bin/runIPCSim /tmp/libpgo-fbms-phase-d-1777181624726/g0_b8_case3_wall_impact_floor-prototype-ipc.json --log
```

Result: passed on April 26, 2026 with temporary copies of the two FBMS configs modified to `num-timestep=0` and absolute mesh paths. Both configs loaded the `g0_b8` `.veg` and surface assets, parsed their floor definitions, built the IPC volume setup, and wrote output subdirectories plus `runIPCSim.log`.

Plan comparison and drift:

- Matches the source plan's lower/upper floor semantics, mutable height, multi-floor `floors[]` schema, old-field rejection, and per-frame frozen floor update model.
- Keeps floor prototypes explicitly scoped as prototypes; external IPC contact and friction remain future Phase E/F work.
- The Phase D test smoke uses a compact tet moving-upper-floor regression for CI speed instead of running the full FBMS `g0_b8` 300-frame configs during unit tests.
- Case 2/3 prototype configs are placed under `examples/fbms/generated/...`; the JSON configs themselves are not ignored, while their `*_output` result directories remain ignored.
