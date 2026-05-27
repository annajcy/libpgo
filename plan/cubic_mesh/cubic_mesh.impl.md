# Cubic Mesh Implementation

Source plan: `plan/cubic_mesh.plan.md`  

## Status

Implemented Phase 1.
Implemented Phase 2A.

# Phase 1 Implemented Scope

Phase 1 now provides a minimal `cubicMesher` triangle-mesh to cubic-mesh path:

- input: closed triangle surface mesh in `.obj`
- output: cubic `.veg`
- optional output: extracted surface `.obj`
- test coverage: helper-level and CLI-level GoogleTest under `tests/src/tools/`
- example assets are organized under `examples/cubic/<case>/`
- top-level usage documentation is included in `README.md`

## Implemented Behavior

### Build layout

- Added `src/tools/cubicMesher/`
- Added `src/tools/CMakeLists.txt -> add_subdirectory(cubicMesher)`
- Added internal static library `cubicMesherCore`
- Linked both `cubicMesher` and `cubicMesher_gtest` against `cubicMesherCore`
- Added root-level usage notes for `cubicMesher` in `README.md`

### CLI

`cubicMesher` has no subcommands and accepts:

- required: `--input-mesh`, `--resolution`, `--output-mesh`, `--E`, `--nu`, `--density`
- optional: `--output-surface`

All failures are reported as exceptions in the helper layer and converted to non-zero exit in `main()`.

### Voxelization rules

- `resolution` means the number of cells along the shortest input AABB edge
- `h = min(sx, sy, sz) / resolution`
- `nx = ceil(sx / h)`, `ny = ceil(sy / h)`, `nz = ceil(sz / h)`
- `gridMin = aabb.center() - 0.5 * h * (nx, ny, nz)`
- occupancy rule is `centerInside || triangleAABBOverlap`
- center test uses `PointInsideOutsideQuery`
- boundary compensation uses `BoundingBoxBVTree` candidates plus `whetherTriangleIntersectBoundingBox(...)`

### Input validation

The helper rejects invalid inputs in this order:

1. `selfIntersectionExact`
2. `areTrianglesManifold`
3. `getExteriorEdges`

It also rejects unsupported extensions, invalid material parameters, non-positive resolution, degenerate input AABB, and empty occupied voxel sets.

### Mesh construction

The implementation does not use `CubicMesh::createFromUniformGrid()` followed by vertex remapping.

Instead, it:

- collects occupied voxel `(i, j, k)`
- builds the shared integer lattice corner set
- maps those corners directly into world-space with `gridMin + h * (ix, iy, iz)`
- constructs `CubicMesh(numVertices, vertices, numElements, elements, E, nu, density)`

This keeps `cubeSize` and related cached values consistent with the true world-space element size.

### Surface export

`writeSurfaceMesh()` uses `GenerateSurfaceMesh::computeMesh(..., triangulate=true)` and writes the extracted surface as `TriMeshGeo`.

## Tests And Validation

### GoogleTest layout

- `tests/CMakeLists.txt` now includes `add_subdirectory(src)`
- added `tests/src/CMakeLists.txt`
- added `tests/src/tools/CMakeLists.txt`
- added single mixed test target `cubicMesher_gtest`
- added single mixed test file `tests/src/tools/cubicMesher_gtest.cpp`

### Covered cases

- helper: voxelize a unit cube OBJ at `resolution=2`, expect `27` vertices and `8` elements
- helper: save and reload `.veg`, verify `E`, `nu`, `density`
- helper: write and reload surface `.obj`
- helper: reject open mesh
- helper: reject self-intersecting mesh
- helper: reject non-manifold mesh
- helper: reject `resolution <= 0`
- CLI: execute built `cubicMesher` binary and verify generated assets are readable

### Validation commands

Passed:

```bash
cmake --preset base_no_mkl_debug
cmake --build build/base_no_mkl_debug --target cubicMesher cubicMesher_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R CubicMesher
build/base_no_mkl_debug/bin/cubicMesher --input-mesh examples/cubic/box/box.obj --resolution 4 --output-mesh examples/cubic/box/box.veg --output-surface examples/cubic/box/box-surface.obj --E 10000000 --nu 0.45 --density 1000
```

Observed generated cubic assets:

- `examples/cubic/box/box.veg`: `125` vertices, `64` elements
- `examples/cubic/box/box-surface.obj`: generated successfully from the cubic mesh surface
- `examples/cubic/bunny/bunny.veg`: `resolution=20`, `6084` vertices, `4695` elements
- `examples/cubic/dragon-dyn/dragon.veg`: `resolution=20`, `10495` vertices, `7503` elements
- `examples/cubic/box-with-sphere/box-with-sphere.veg`: `resolution=50`, `57366` vertices, `51845` elements

Asset generation commands and per-case parameters are documented in `examples/cubic/README.md`.

### Documentation updates

- Root `README.md` now includes a `Cubic Mesher` section with build steps, a representative command line, and a link to `examples/cubic/README.md`
- `examples/cubic/README.md` records the exact command lines and parameters used to generate each shipped cubic asset

## Comparison To Source Plan

- The source plan proposed using `createFromUniformGrid()` and then remapping vertices; the implementation intentionally changed this to direct world-space mesh construction to keep cubic mesh internal size caches correct.
- The source plan said tests should live under `tests/src/tools`; the implementation matches that and formalizes it as one mixed gtest target and one mixed gtest file.
- The source plan treated generated assets as a validation artifact under `examples/cubic-box/`; the implementation now keeps cubic assets under `examples/cubic/<case>/` so multiple cubic cases share one consistent layout.
- The source plan did not mention root-level documentation for `cubicMesher`; the implementation adds concise usage documentation in `README.md` and detailed per-asset commands in `examples/cubic/README.md`.

## Phase Boundary

This implementation only completes phase 1. It does not change the phase 2 and phase 3 simulation/FEM integration described in the source plan.

---

# Cubic Mesh Phase 2A.1 Implementation

Source plan: `plan/cubic_mesh.plan.md`  
Target phase: `2A.1 SimulationMesh 加载器`

## Status

Implemented.

## Implemented Scope

Phase `2A.1` now adds a cubic `SimulationMesh` loader path without enabling any cubic FEM model logic:

- added `loadCubicMesh(const VolumetricMeshes::CubicMesh *)`
- converted `CubicMesh` vertex and element data into the flat buffers expected by `SimulationMesh(...)`
- copied per-element ENu material data with the same clone-and-delete lifetime pattern used by `loadTetMesh()`
- added a focused gtest under `tests/src/core/solidDeformationModel/`

## Implemented Behavior

### API changes

- `src/core/solidDeformationModel/simulationMesh.h` now forward-declares `VolumetricMeshes::CubicMesh`
- `src/core/solidDeformationModel/simulationMesh.h` now declares `loadCubicMesh(...)` next to `loadTetMesh(...)`

### Loader behavior

`src/core/solidDeformationModel/simulationMesh.cpp` now implements `loadCubicMesh(...)` by:

- exporting all cubic vertex positions into a flat `std::vector<double>` with `x, y, z` packing
- exporting all cubic element connectivities into a flat `std::vector<int>` with `8` vertex indices per element
- keeping the existing cubic local vertex ordering unchanged
- downcasting each element material to `VolumetricMesh::ENuMaterial`
- creating one `SimulationMeshENuMaterial(E, nu)` copy per element
- assigning `SimulationMeshType::CUBIC`
- deleting the temporary loader-owned material objects immediately after `SimulationMesh` construction, matching the existing `loadTetMesh()` ownership pattern

### Test layout

Added the core gtest structure requested for this phase:

- `tests/src/CMakeLists.txt` now includes `add_subdirectory(core)`
- added `tests/src/core/CMakeLists.txt`
- added `tests/src/core/solidDeformationModel/CMakeLists.txt`
- added `tests/src/core/solidDeformationModel/simulationMesh_gtest.cpp`

The new test target is `simulationMesh_gtest` and it uses `examples/cubic/box/box.veg` as the fixed example asset.

## Tests And Validation

### Covered case

`SimulationMeshGTest.LoadsCubicMeshFromExampleFile` verifies that:

- `loadCubicMesh()` returns a non-null mesh
- the resulting type is `SimulationMeshType::CUBIC`
- vertex count and element count match the source `CubicMesh`
- `getNumElementVertices() == 8`
- the first element's 8 vertex indices match the source mesh
- the first vertex position matches the source mesh
- the first element material is a `SimulationMeshENuMaterial` with the same `E` and `nu` as the source `CubicMesh` material

### Validation commands

Passed:

```bash
cmake --preset base_no_mkl_debug
cmake --build build/base_no_mkl_debug --target simulationMesh_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R SimulationMeshGTest
```

## Comparison To Source Plan

- The implementation matches the source plan's loader boundary exactly: it only performs geometry/material data marshaling into `SimulationMesh(...)`.
- The plan left test placement open after the later clarification; the implementation follows the requested final layout under `tests/src/core/solidDeformationModel/`.
- No phase-2A.2, phase-2A.3, or later cubic FEM work was pulled into this change.

---

# Cubic Mesh Phase 2A.2 Implementation

Source plan: `plan/cubic_mesh.plan.md`  
Target phase: `2A.2 DeformationModelAssembler 动态局部维度`

## Status

Implemented.

## Implemented Scope

Phase `2A.2` now removes the fixed local-size assumptions that were still baked into `DeformationModelAssembler`:

- local element buffers now scale with `simulationMesh->getNumElementVertices() * 3`
- inverse-index caches are now stored with dynamic matrix sizes instead of fixed `24 x 24`
- plastic and elastic parameter scratch buffers now scale with `numPlasticParams` and `numElasticParams`
- regression coverage now exercises both tet `12 DOF` and shell `18 DOF` assembler paths

## Implemented Behavior

### Assembler dynamic sizing

`src/core/solidDeformationModel/deformationModelAssembler.h/cpp` now:

- stores `localDOFs = neleVtx * 3`
- uses `DynamicIndexMatrix` for:
  - `elementKInverseIndices`
  - `element_dfda_InverseIndices`
  - `element_dfdb_InverseIndices`
- sizes each inverse-index cache to the actual local row/column dimensions needed by:
  - Hessian assembly
  - `df/da`
  - `df/db`

### Local buffer changes

The following fixed-size assumptions were removed:

- `ES::V24d localp`
- `ES::V18d localGradx`
- `double localKData[24 * 24]`
- `double plasticParam[20]`
- `double elasticParam[20]`

They are now replaced by dynamic `ES::VXd` or `std::vector<double>` buffers sized from:

- `localDOFs`
- `numPlasticParams`
- `numElasticParams`

This applies to:

- `computeEnergy()`
- `computeGradient()`
- `computeHessian()`
- `compute_df_da()`
- `compute_df_db()`

### Logging / safety cleanup

- `computeGradient()` no longer hardcodes logging of `plasticParam[0..2]`
- sanity checks now iterate over `localDOFs`
- `compute_df_da()` and `compute_df_db()` now short-circuit cleanly when the corresponding parameter count is zero

## Tests And Validation

### Test layout

Added a new core regression target:

- `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`
- `tests/src/core/solidDeformationModel/CMakeLists.txt` now includes `deformationModelAssembler_gtest`

The target uses:

- `examples/torus.veg` for tet regression
- `examples/shell/shell.obj` for shell regression

### Covered cases

`DeformationModelAssemblerGTest.TetAssemblerRegression` verifies that:

- `loadTetMesh()` + `DeformationModelManager(VOLUMETRIC_DOF6, STABLE_NEO)` still assemble correctly
- `computeGradient()`, `computeHessian()`, and `compute_df_da()` produce correctly sized finite outputs

`DeformationModelAssemblerGTest.ShellAssemblerRegression` verifies that:

- `loadShellMesh()` + `DeformationModelManager(SHELL_FF_DOF1, KOITER_STVK)` still assemble correctly
- `computeGradient()`, `computeHessian()`, `compute_df_da()`, and `compute_df_db()` produce correctly sized finite outputs

### Validation commands

Passed:

```bash
cmake --preset base_no_mkl_debug
cmake --build build/base_no_mkl_debug --target deformationModelAssembler_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R DeformationModelAssembler
```

## Comparison To Source Plan

- The implementation matches the plan's main intent: assembly-layer local working dimensions are now driven by mesh topology and parameter counts instead of fixed tet/shell-era stack sizes.
- The plan allowed either dynamic `ES::VXd` or dynamic `ES::MXd`; the implementation uses `ES::VXd` plus `std::vector<double>` backing storage mapped into dynamic `ES::MXd` where matrix views are needed.
- The plan asked to preserve current writeback semantics for `df/da` and `df/db`; the implementation keeps those paths serial and does not introduce new parallel behavior.
- Runtime validation still covers only tet and shell paths. Real `24 DOF` cubic runtime validation remains blocked on later phases because the repo still does not have `CubicMeshDeformationModel`.

# Phase 2A.3 Implemented Scope

Phase 2A.3 now refactors `runSim` volumetric mesh input handling so tet and cubic meshes share the same input validation and preprocessing path, while cubic still stops before FEM initialization.

## Implemented Behavior

### Volume mesh config parsing

Added `src/tools/runSim/runSimVolumeMeshIO.h/cpp` with two helper entry points:

- `parseVolumeMeshInputConfig(const ConfigFileJSON &)`
- `loadValidatedVolumeMesh(const VolumeMeshInputConfig &, double scale)`

The parser now enforces:

- exactly one of `"tet-mesh"` or `"cubic-mesh"` must exist
- both present is rejected
- both missing is rejected

The loader now enforces:

- `VolumetricMesh::getElementType(filename)` is checked before constructing the mesh
- `"tet-mesh"` only accepts `VolumetricMesh::TET`
- `"cubic-mesh"` only accepts `VolumetricMesh::CUBIC`
- mismatched key vs file type throws a descriptive error instead of silently accepting it

### Shared preprocessing path in `runSim`

`src/tools/runSim/runSim.cpp` now:

- loads the volumetric input into `std::unique_ptr<VolumetricMeshes::VolumetricMesh>`
- applies scene `scale` generically through the `VolumetricMesh` base interface
- builds `InterpolationCoordinates::BarycentricCoordinates` from `volumetricMesh.get()`
- builds the volumetric mass matrix from `volumetricMesh.get()`

This means tet and cubic now share the same preprocessing path for:

- volumetric mesh loading
- type validation
- scaling
- surface embedding weights
- mass matrix construction

While validating the shared tet/cubic preprocessing path, implementation exposed an existing dynamic-dimension bug in `src/core/volumetricMesh/generateMassMatrix.cpp`:

- the function used `thread_local ES::MXd elementMass`
- it only resized that cache when `rows() == 0`
- after a tet `4x4` mass-matrix call, a later cubic `8x8` call in the same process could reuse the stale buffer shape and hit Eigen bounds assertions

This phase fixes that repo-truth issue by resizing the thread-local buffer whenever its shape does not match the current `numElementVertices`.

### Tet-only simulation gate remains explicit

After shared preprocessing:

- tet meshes continue into the existing `loadTetMesh()` + `DeformationModelManager` path
- cubic meshes now stop with an explicit runtime error message:
  `cubic volumetric mesh preprocessing is available, but runSim cubic FEM path is not enabled until phases 2B/2C.`

This phase intentionally does not:

- call `loadCubicMesh()` from `runSim`
- initialize a cubic `DeformationModelManager`
- enable any cubic FEM solve path

### Config-relative path resolution and cubic example JSONs

`src/tools/runSim/runSimVolumeMeshIO.h/cpp` now also resolves run-time paths against the config file directory for:

- `"tet-mesh"` / `"cubic-mesh"`
- `"surface-mesh"`
- `"output"`
- `fixed-vertices[].filename`
- `external-objects[].filename`

Absolute paths are preserved. Relative paths are normalized against the directory containing the input JSON config.

This change is now used by `src/tools/runSim/runSim.cpp`, so example configs can be launched from the repo root without relying on the shell working directory matching the config directory.

Added mirrored cubic example configs:

- `examples/cubic/box/box.json`
- `examples/cubic/bunny/bunny.json`
- `examples/cubic/dragon-dyn/dragon.json`
- `examples/cubic/box-with-sphere/box-with-sphere.json`

These configs:

- use `"cubic-mesh"` explicitly
- use the extracted `*-surface.obj` as `surface-mesh`
- mirror the corresponding tet demo parameters where possible
- use unique cubic-specific output paths to avoid clobbering tet example output

This intentionally does not yet switch `examples/cubic/box/box.json` to the later phase-4 static `fixed.txt` design from the source plan. The shipped cubic JSONs are still preprocessing/demo configs that stop at the current cubic FEM gate.

## Tests And Validation

### Test layout

Added a new tools-level regression target:

- `tests/src/tools/runSim_gtest.cpp`
- `tests/src/tools/CMakeLists.txt` now includes `runSim_gtest`

The test target compiles `src/tools/runSim/runSimVolumeMeshIO.cpp` directly and uses existing repo assets:

- `examples/box/box.veg`
- `examples/box/box.obj`
- `examples/cubic/box/box.veg`
- `examples/cubic/box/box.obj`

### Covered cases

`RunSimVolumeMeshIOGTest` verifies:

- legacy `"tet-mesh"` config is still accepted
- new `"cubic-mesh"` config is accepted
- mesh scaling still works through the helper path
- config-relative path resolution works for cubic example fields
- config-relative path resolution remains compatible with legacy tet example fields
- missing volume mesh keys are rejected
- duplicate volume mesh keys are rejected
- tet files bound to `"cubic-mesh"` are rejected
- cubic files bound to `"tet-mesh"` are rejected
- barycentric interpolation and volumetric mass matrix construction both work for tet and cubic meshes through the common preprocessing path

### Validation commands

Passed:

```bash
cmake --preset base_no_mkl_debug
cmake --build build/base_no_mkl_debug --target runSim runSim_gtest simulationMesh_gtest deformationModelAssembler_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "RunSimVolumeMeshIO|SimulationMeshGTest|DeformationModelAssemblerGTest"
```

Additional manual compatibility check passed with a temporary absolute-path config derived from `examples/box/box.json`:

```bash
build/base_no_mkl_debug/bin/runSim <mktemp-generated box_compat.json>
```

Additional cubic smoke tests passed from the repo root and reached the intended unsupported gate without file-not-found failures:

```bash
build/base_no_mkl_debug/bin/runSim examples/cubic/box/box.json
build/base_no_mkl_debug/bin/runSim examples/cubic/dragon-dyn/dragon.json
```

## Comparison To Source Plan

- The implementation follows the source plan's explicit dual-key design and does not introduce a generic `"volume-mesh"` key.
- The common preprocessing path is now shared exactly where the plan requested: loading, scaling, barycentric interpolation, and mass matrix construction.
- The plan allowed cubic preprocessing but required a hard stop before FEM initialization; the implementation does that with an explicit non-zero exit and error message.
- The source plan mentioned only `runSim.cpp`, but the implementation introduced `runSimVolumeMeshIO.h/cpp` to keep config parsing and type validation out of `main()`. This is intentional scope-preserving refactoring, not a behavior expansion.
- The source plan did not call out `GenerateMassMatrix`, but shared tet/cubic preprocessing validation revealed a real cross-mesh `thread_local` resizing bug there. Fixing it was necessary for the planned common preprocessing path to work reliably in one process.
- The source plan only sketched `examples/cubic/box/box.json` as a later static end-to-end target. The implementation deliberately ships a fuller set of cubic example JSONs now, but keeps them aligned with the current dynamic tet demos and the still-disabled cubic FEM runtime.

---

# Config-Relative JSON Path Helper Consolidation

Source plan basis: follow-up consolidation after `2A.3`  
Target scope: unify config-relative JSON path handling across all JSON-config consumers

## Status

Implemented.

## Implemented Scope

This follow-up change moves config-relative path resolution out of the `runSim`-local helper layer and into `ConfigFileJSON`, then reuses that shared behavior across the repo's JSON-driven entry points:

- `src/tools/runSim/runSim.cpp`
- `src/tools/runSim/runShellSim.cpp`
- `src/c/pgo_c.cpp` via `pgo_run_sim_from_config(...)`
- `src/core/animationIO/animationLoader.cpp`

It also updates shipped cubic example JSONs and root-level README examples so config-based flows can be launched from the repo root without `cd examples/...`.

## Implemented Behavior

### Shared helper API

`src/core/configFileJSON/configFileJSON.h/cpp` now:

- records the opened config filename and its parent directory
- exposes `getConfigFilename()` and `getConfigDirectory()`
- exposes `resolvePath(const std::string &)` for raw path strings
- exposes `getResolvedPath(key, forceExistence, default)` for direct key lookup
- applies config-relative resolution to `getVectorPath(...)`
- keeps the older token helpers such as `{work}` unchanged for backward compatibility

Resolution rules are now:

- absolute paths stay absolute
- relative paths are interpreted relative to the JSON file directory
- paths are normalized with `lexically_normal()`

### Consumer adoption

The following call sites now use the shared helper instead of raw config strings or tool-local path logic:

- `runSimVolumeMeshIO.cpp` for `"tet-mesh"`, `"cubic-mesh"`, `"surface-mesh"`, `"output"`, and nested filenames
- `runShellSim.cpp` for `"surface-mesh"`, `"output"`, `fixed-vertices[].filename`, and `external-objects[].filename`
- `pgo_c.cpp` for the same `runSim` config fields inside `pgo_run_sim_from_config(...)`
- `animationLoader.cpp` for `driving-mesh`, `display-mesh`, `sequence`, and cache filenames derived from config content

### Example configs and documentation

The shipped cubic companion configs now use the original input `.obj` files as `surface-mesh`:

- `examples/cubic/box/box.json -> box.obj`
- `examples/cubic/bunny/bunny.json -> bunny.obj`
- `examples/cubic/dragon-dyn/dragon.json -> dragon.obj`
- `examples/cubic/box-with-sphere/box-with-sphere.json -> box-with-sphere.obj`

The extracted `*-surface.obj` files remain generated assets from `cubicMesher`, but they are no longer the default display mesh in the cubic `runSim` configs.

`README.md` examples were also updated so config-based Python entry points are shown from the repo root:

- `python src/python/pypgo/pgo_run_sim.py examples/box/box.json`
- `python src/python/pypgo/pgo_dump_abc.py examples/box/anim.json examples/box/`

## Tests And Validation

### Test layout

Added core-level regression coverage for the shared helper:

- `tests/src/core/configFileJSON_gtest.cpp`
- conditional `tests/src/core/animationLoader_gtest.cpp` when `animationIO` is available

The tests cover:

- config filename and directory tracking
- key-based path resolution on shipped tet, cubic, and shell example JSONs
- raw relative and absolute path normalization
- vector path resolution
- non-regression of legacy token helpers
- config-relative animation driving-mesh and sequence loading

The existing `tests/src/tools/runSim_gtest.cpp` was also updated to use the shared helper-backed `runSimVolumeMeshIO` interface and to expect the cubic examples' original `.obj` surface meshes.

### Validation commands

Passed:

```bash
cmake --build build/base_no_mkl_debug --target runSim runShellSim runSim_gtest configFileJSON_gtest simulationMesh_gtest deformationModelAssembler_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "RunSimVolumeMeshIO|ConfigFileJSON|SimulationMeshGTest|DeformationModelAssemblerGTest"
build/base_no_mkl_debug/bin/runSim examples/cubic/box/box.json
```

If `animationIO` is available in the build, `animationLoader_gtest` also exercises config-relative animation loading.

## Comparison To Earlier 2A.3 Implementation

- `2A.3` originally kept config-relative resolution inside `runSimVolumeMeshIO`; the final implementation moves that behavior into `ConfigFileJSON` so all JSON consumers share one source of truth.
- The earlier cubic JSON pass used extracted `*-surface.obj` display meshes; the final implementation switches those configs to the original input `.obj` files.
- The earlier README examples still assumed `cd examples/...`; the consolidated implementation changes config-based examples to repo-root invocation style.

---

# Phase 2B Implemented Scope

Phase 2B now adds a real cubic volumetric FEM element model and connects it to the existing `SimulationMeshType::CUBIC` deformation-model manager path, while still leaving the user-facing `runSim` cubic runtime gate for phase 2C.

## Implemented Behavior

### CubicMeshDeformationModel core FEM

Added:

- `src/core/solidDeformationModel/cubicMeshDeformationModel.h`
- `src/core/solidDeformationModel/cubicMeshDeformationModel.cpp`

`CubicMeshDeformationModel` now implements the full volumetric `DeformationModel` interface for an 8-node trilinear hex:

- `allocateCacheData()`, `freeCacheData()`, `prepareData()`
- `computeEnergy()`, `compute_dE_dx()`, `compute_d2E_dx2()`
- `compute_d2E_dxda()`, `compute_d2E_dxdb()`
- `vonMisesStress()`, `maxStrain()`
- `enableSPD()`

The implementation follows the planned cubic hex design:

- local vertex order matches the current `CubicMesh` 8-corner order directly
- reference coordinates use `[0, 1]^3`
- quadrature uses fixed `2 x 2 x 2` Gauss points with `1 / 8` weights
- each quadrature point caches its own `DmInv`, `weightDetJ`, `restBm`, and `rest_dFdx`
- `prepareData()` computes `Fref`, `Fe`, `U/V/S`, current `Bm`, and current `dFdx` for all 8 quadrature points
- energy, force, Hessian, plastic mixed derivative, and material mixed derivative all accumulate over the 8 quadrature points

The implementation now also tightens the cache layout to match the refined Phase 2B plan:

- geometry-related cache remains fixed-size:
  - `24` local DOFs
  - `8` quadrature points
  - per-point `9 x 24` `dFdx`
  - per-point `3 x 8` `Bm`
  - per-point `Fref`, `Fe`, `U`, `V`, `S`
- parameter-related cache is now runtime-sized instead of using static capacity limits:
  - `plasticParam`
  - `ddetA_da`
  - `d2detA_da2`
  - `dAInv_dai`
  - flattened `d2AInv_dai_daj`
  - `materialParam`
- `allocateCacheData()` now allocates those parameter buffers once using the model's actual
  `plasticModel->getNumParameters()` and `elasticModel->getNumParameters()`
- `prepareData()` now only fills/reset existing buffers and no longer relies on hardcoded
  `kMaxNumPlasticParams` / `kMaxNumElasticParams` style assumptions

The class also exposes a small cubic-specific inspection surface used by tests and future debugging:

- `computeFe(...)`
- `computeP(...)`
- `computedPdF(...)`
- `computedFdx(...)`
- `computeForceFromP(...)`
- `getWeightDetJ(...)`

For parity with the tet model, the cubic model also implements non-virtual helpers:

- `compute_dE_da()`
- `compute_d2E_da2()`
- `compute_dE_db()`
- `compute_d2E_db2()`
- `compute_d2E_dadb()`

### DeformationModelManager cubic integration

`src/core/solidDeformationModel/deformationModelManager.cpp` now includes a `SimulationMeshType::CUBIC` branch inside `DeformationModelManager::init()`.

That branch now:

- gathers the 8 rest vertices for each cubic element
- reuses the existing volumetric elastic/plastic material selection logic
- instantiates `CubicMeshDeformationModel` per cubic element

This means `loadCubicMesh()` + `DeformationModelManager(VOLUMETRIC_DOF*, volumetric material)` now produce real element FEM objects instead of failing with `unknown mesh element type`.

### Build system registration

`src/core/solidDeformationModel/CMakeLists.txt` now registers `cubicMeshDeformationModel.h/cpp` in the `solidDeformationModel` library.

## Tests And Validation

### Direct cubic element regression tests

Added:

- `tests/src/core/solidDeformationModel/cubicMeshDeformationModel_gtest.cpp`
- `tests/src/core/solidDeformationModel/CMakeLists.txt` target `cubicMeshDeformationModel_gtest`

The new tests cover:

- rest-state `Fe` at all 8 Gauss points is approximately identity when `Fp = I`
- all `weightDetJ` values are positive and sum to the hex volume
- rest and perturbed energy stay finite
- `vonMisesStress()` and `maxStrain()` return 8 finite values
- `compute_dE_dx()` matches centered finite differences of `E(x)`
- `compute_d2E_dx2()` matches centered finite differences of `dE_dx(x)` and remains symmetric
- `compute_d2E_dxda()` matches centered finite differences of `dE_dx(x, a)` with respect to plastic parameters
- `compute_d2E_dxdb()` matches centered finite differences of `dE_dx(x, b)` with respect to material parameters
- two independently allocated caches can be reused across repeated `prepareData()` calls with different
  states and parameters without leaking stale values into later evaluations

The FD test logs maximum absolute and relative errors for the `x`, `a`, and `b` derivative checks.

### Assembler cubic smoke test

Extended:

- `tests/src/core/solidDeformationModel/deformationModelAssembler_gtest.cpp`

`DeformationModelAssemblerGTest.CubicAssemblerSmokeRegression` now verifies:

- `loadCubicMesh()` + `DeformationModelManager(VOLUMETRIC_DOF6, STABLE_NEO)` initialize successfully
- cubic `computeGradient()`, `computeHessian()`, and `compute_df_da()` all produce finite outputs with the expected sizes

### Validation commands

Passed:

```bash
cmake --preset base_no_mkl_debug
cmake --build build/base_no_mkl_debug --target simulationMesh_gtest deformationModelAssembler_gtest cubicMeshDeformationModel_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "SimulationMeshGTest|DeformationModelAssemblerGTest|CubicMeshDeformationModelGTest"
```

The targeted cubic direct-element executable also passed standalone and reported tight FD errors for `dx`, `dxda`, and `dxdb`.

## Comparison To Source Plan

- The source plan asked to mirror the tet model's mixed-derivative structure; the implementation does that per quadrature point and sums over all 8 cubic integration points.
- The source plan treated extra helper access as optional; the implementation adds explicit cubic inspection helpers such as `computeFe()` and `getWeightDetJ()` so the direct-element tests can validate quadrature geometry without reaching into private cache types.
- The refined source plan later clarified that parameter cache should be dynamic while geometry cache stays fixed-size; the implementation now follows that tightened design exactly.
- The source plan deferred `runSim` end-to-end activation to later phases; the implementation keeps that boundary intact. Phase 2B only enables cubic FEM construction inside `DeformationModelManager`, not the `runSim` cubic solve path.

---

# Phase 2C Implemented Scope

Phase 2C now removes the remaining cubic runtime gate in `runSim` and enables the shared volumetric simulation main path for both tet and cubic meshes.

## Implemented Behavior

### Shared volumetric FEM setup helper

Added:

- `src/tools/runSim/runSimFEMSetup.h`
- `src/tools/runSim/runSimFEMSetup.cpp`

`initializeVolumetricSimulation(...)` now centralizes the common runtime initialization path:

- branch on `VolumetricMesh::elementType`
- `TET -> loadTetMesh(...)`
- `CUBIC -> loadCubicMesh(...)`
- construct `DeformationModelManager`
- call `setMesh(...)`, `init(...)`, and `setEnforceSPD(1)`
- construct `DeformationModelAssembler`
- initialize default plastic parameters by querying each element's `PlasticModel3DDeformationGradient`
- collect rest positions
- construct `DeformationModelEnergy` and bind the initialized plastic state

This keeps tet and cubic divergence limited to `VolumetricMesh -> SimulationMesh`, with the downstream runtime path shared.

### `runSim` cubic main-path enablement

`src/tools/runSim/runSim.cpp` now:

- removes the old early exit that blocked `VolumetricMesh::CUBIC`
- replaces the previous tet-only `loadTetMesh(...)` block with the shared helper-backed initialization path
- reuses the returned `simMesh`, `dmm`, `assembler`, `elasticEnergy`, `plasticity`, and `restPosition` for the existing solver/contact/runtime flow

No cubic-only solver branch was added. After initialization, tet and cubic continue through the same `runSim` pipeline.

### Test integration

Updated:

- `tests/src/tools/runSim_gtest.cpp`
- `tests/src/tools/CMakeLists.txt`
- `src/tools/runSim/CMakeLists.txt`

The `runSim_gtest` target now compiles `runSimFEMSetup.cpp` directly and includes a new runtime smoke test:

- `RunSimVolumeMeshIOGTest.InitializesCubicRuntimeMainPath`

That test verifies:

- `examples/cubic/box/box.json` resolves and loads a valid cubic volumetric mesh
- cubic `SimulationMesh` creation succeeds
- `DeformationModelManager` produces cubic deformation models
- `DeformationModelAssembler` initializes successfully
- `DeformationModelEnergy` can create and assemble a finite Hessian
- cubic gradient assembly through the shared main-path objects returns finite values with the expected dimensions

The test now explicitly calls `pgo::Logging::init()` before entering the runtime path. This matches the real `runSim` binary behavior, where logging is initialized in `main()` before deformation-model setup begins.

## Tests And Validation

Passed:

```bash
cmake --preset base_no_mkl_debug -DBUILD_TESTING=ON
cmake --build build/base_no_mkl_debug --target runSim runSim_gtest simulationMesh_gtest deformationModelAssembler_gtest cubicMeshDeformationModel_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "RunSimVolumeMeshIO|SimulationMeshGTest|DeformationModelAssemblerGTest|CubicMeshDeformationModelGTest"
build/base_no_mkl_debug/bin/runSim examples/cubic/box/box.json
```

Observed runtime validation:

- the new cubic runtime gtest passes
- the targeted regression suite passes `18/18`
- the real `runSim` binary no longer exits at cubic initialization
- `runSim examples/cubic/box/box.json` enters the shared deformation/contact/Newton solve loop and advances multiple time steps before manual interruption

## Comparison To Source Plan

- The source plan suggested extracting the shared volumetric initialization block for reuse in tests; the implementation does that via `runSimFEMSetup`.
- The source plan targeted enabling cubic through the existing `runSim` main path instead of adding a parallel solver path; the implementation preserves exactly that boundary.
- The source plan asked for a runtime smoke test in addition to preprocessing coverage; the implementation adds that test and also validates the real binary path directly.

---

# Phase 2C.4 Implemented Scope

Phase 2C.4 now closes the remaining contact-side gap from the source plan: surface embedding expansion inside the contact handlers is no longer hardcoded to tet arity `4`, so cubic surface embeddings now propagate all `8` volumetric weights back into the contact sample interpolation path.

## Implemented Behavior

### Contact embedding arity fix

Updated:

- `src/core/contact/triangleMeshExternalContactHandler.cpp`
- `src/core/contact/triangleMeshSelfContactHandler.cpp`

Both handlers now:

- reject one-sided embedding input where only indices or only weights are provided
- reject malformed embedding arrays with `std::invalid_argument` when:
  - index/weight array sizes differ
  - the surface vertex count is zero
  - the embedding arrays are not divisible by the number of surface vertices
  - the derived embedding arity is non-positive
- derive `embeddingArity = vertexEmbeddingIndices->size() / vertices.size()` once per handler construction
- expand sample-to-volume interpolation entries with `vid * embeddingArity + j` and `j < embeddingArity`

The change is intentionally limited to the surface-embedding expansion path. The legitimate `4`-entry constants used by point-triangle pair bookkeeping in self contact were left untouched.

### Regression coverage

Updated:

- `tests/src/tools/runSim_gtest.cpp`
- `tests/src/tools/CMakeLists.txt`

`runSim_gtest` now links `contact` and includes direct regression checks that compare each handler's `getSampleEmbeddingMatrix()` against `BarycentricCoordinates::generateInterpolationMatrix()` with `subdivideTriangle=1`, so sample rows stay aligned with original surface vertices.

Added coverage:

- tet external contact embedding matches barycentric interpolation
- tet self-contact embedding matches barycentric interpolation
- cubic external contact embedding matches barycentric interpolation
- cubic self-contact embedding matches barycentric interpolation
- malformed external contact embedding arrays throw `std::invalid_argument`
- malformed self-contact embedding arrays throw `std::invalid_argument`

## Tests And Validation

Passed:

```bash
cmake --build build/base_no_mkl_debug --target runSim runSim_gtest
ctest --test-dir build/base_no_mkl_debug --output-on-failure -R "RunSimVolumeMeshIO"
build/base_no_mkl_debug/bin/runSim examples/cubic/box/box.json
```

Observed validation:

- the new tet/cubic external/self contact embedding regression tests pass
- the malformed-input regression tests pass
- the full `RunSimVolumeMeshIO` suite passes with the new contact coverage included
- `runSim examples/cubic/box/box.json` still initializes the cubic runtime path and advances into the shared simulation loop

## Comparison To Source Plan

- The earlier Phase 2C implementation record was accurate for cubic main-path enablement, but it overstated phase completeness because the contact handlers were still truncating cubic embeddings to tet arity. This Phase 2C.4 update closes that remaining gap.
- The source plan asked for a minimal contact-focused regression inside `tests/src/tools/runSim_gtest.cpp`; the implementation follows that directly instead of creating a separate test target.
- No future-phase work was pulled in: `runSim.cpp`, `pgo_c.cpp`, and the point-triangle energy formulations remain unchanged.

## Note

`plan/cubic_mesh.impl.md` remains ignored by the repo's `.gitignore` via `plan/*`, so this phase record is a local implementation log rather than a tracked repository change.
