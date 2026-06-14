# Conda Prefix Centralization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every `find_package(...)` (including `BLAS`) resolve against the active conda environment through a single `CMAKE_PREFIX_PATH` injection, instead of each third-party finder hardcoding its own `$CONDA_PREFIX` hint — while keeping the `PGO_CHECK_CONDA=OFF` standalone (non-conda) source build fully working.

**Architecture:** When `PGO_CHECK_CONDA=ON` and `$CONDA_PREFIX` is set, the top-level `CMakeLists.txt` prepends the conda prefix (`$CONDA_PREFIX`, plus `$CONDA_PREFIX/Library` on Windows) to `CMAKE_PREFIX_PATH` once, cross-platform, before any third-party module runs. Standard `find_package(... CONFIG)` finders (`BLAS`, `Boost`, `TBB`, `MKL`) then locate the conda copies automatically. Special-purpose conda usages that are NOT plain finders — CGAL's Windows GMP/MPFR DLL copy, OpenVDB's Boost hinting, Gmsh's custom `find_path`/`find_library` finder — are left untouched. The two-mode switch (`PGO_CHECK_CONDA`) stays; only the redundant per-finder prefix hints are removed.

**Tech Stack:** CMake (>=3.29), Ninja, conda-forge toolchain, OpenBLAS/MKL, SuiteSparse.

**Scope note:** The BLAS backend policy is unchanged — default `BLA_VENDOR=OpenBLAS`, `BLA_VENDOR=Intel10_64lp` when `PGO_USE_MKL=ON`, OpenBLAS-only on macOS. This plan does NOT touch `BLA_VENDOR`, MKL Pardiso, or `EIGEN_USE_MKL_ALL`. The `openblas` dev package stays in `environment.yml` (BLA_VENDOR=OpenBLAS resolves `libopenblas` by name, which that package provides).

**Branch:** Do this on a dedicated branch (e.g. `codex/conda-prefix-centralization`), separate from the current uncommitted Windows/torch/smoke-test fixes.

---

## Related design decision: one BLAS backend per process

numpy/scipy/pyvista link the conda BLAS **interface** (`libblas`/`liblapack`), whose backend is chosen by the installed variant; the C++ extension links a specific BLAS chosen by `BLA_VENDOR`. These must be the **same** backend, or a single process loads two BLAS implementations (symbol clashes / wrong results / crashes).

- The `pypgo-mkl` conda recipe guarantees this by pinning `libblas * *mkl`; the OpenBLAS recipe pins `libblas * *openblas`.
- **Landed separately as a companion fix (not a task in this plan):** `environment.yml` now pins `libblas=*=*openblas` / `liblapack=*=*openblas`, so the local dev env is deterministically OpenBLAS — matching the C++ `BLA_VENDOR=OpenBLAS`. The README documents the MKL switch (`mkl-devel` + `libblas=*=*mkl` + `liblapack=*=*mkl`), because `mkl-devel` alone leaves numpy on OpenBLAS (the inconsistency this fixes).

**How this interacts with the refactor:** Task 1's central `$CONDA_PREFIX` injection makes `find_package(BLAS)` resolve the conda OpenBLAS (dev files from the `openblas` package); the env variant-pin makes numpy resolve the same backend. Together they guarantee one consistent BLAS across Python and C++. This plan does **not** change `BLA_VENDOR` or the variant pinning — it only centralizes how the conda prefix is discovered.

---

## File Structure

| File | Responsibility | Change |
|------|----------------|--------|
| `CMakeLists.txt` | Top-level config; the single conda-prefix injection point | Modify: replace the macOS-only conda `else()` (currently uncommitted) with a cross-platform central injection |
| `CMakeModules/third-party/boost.cmake` | Find Boost | Modify: drop redundant `CMAKE_PREFIX_PATH` prepend (Phase 2) |
| `CMakeModules/third-party/tbb.cmake` | Find TBB | Modify: drop redundant `TBB_DIR` conda hint (Phase 2) |
| `CMakeModules/third-party/mkl.cmake` | Find MKL | Modify: drop redundant `MKL_DIR` conda hint (Phase 2) |
| `CMakeModules/third-party/cgal.cmake` | CGAL + Windows GMP/MPFR DLL copy | **Unchanged** (conda usage is DLL placement, not a finder) |
| `CMakeModules/third-party/openvdb.cmake` | OpenVDB + Boost hinting | **Unchanged** (entangled Boost detection; low value, high risk) |
| `CMakeModules/third-party/gmsh.cmake` | Custom Gmsh finder | **Unchanged** (uses prefix as `find_path`/`find_library` HINTS, not a CONFIG package) |

**Phasing:** Phase 1 (Task 1) is the core, purely-additive change that fixes BLAS uniformly and establishes the central mechanism. Phase 2 (Tasks 2–4) removes the now-redundant hints incrementally and can be stopped/deferred at any point. Task 5 is the CI gate.

---

## Phase 1 — Central conda prefix injection

### Task 1: Inject `$CONDA_PREFIX` into `CMAKE_PREFIX_PATH` centrally

**Files:**
- Modify: `CMakeLists.txt` (the `if(APPLE)` block around lines 38–71, plus a new block right after it)

- [ ] **Step 1: Remove the macOS-only conda `else()` branch (currently uncommitted)**

In `CMakeLists.txt`, inside the `if(APPLE)` block, the `if(NOT PGO_CHECK_CONDA)` currently has an `else()` that injects the conda prefix. Delete that `else()` branch so the macOS block only handles Homebrew again:

Replace this:

```cmake
    if(PGO_HOMEBREW_PREFIX AND EXISTS "${PGO_HOMEBREW_PREFIX}/include")
      message(STATUS "Adding Homebrew prefix to global search paths: ${PGO_HOMEBREW_PREFIX}")
      include_directories(SYSTEM "${PGO_HOMEBREW_PREFIX}/include")
      link_directories("${PGO_HOMEBREW_PREFIX}/lib")
      list(APPEND CMAKE_PREFIX_PATH "${PGO_HOMEBREW_PREFIX}")
    endif()
  else()
    # Conda build: point standard finders at the active conda environment.
    # Package-specific finders (TBB, Gmsh, OpenVDB, ...) hardcode their own
    # $CONDA_PREFIX hints, but find_package(BLAS) relies on the default search
    # path, which does not include the env when building with the system
    # (non-conda) compiler — causing "Could NOT find BLAS" for the OpenBLAS
    # dev build. Adding the prefix here lets it resolve the conda OpenBLAS.
    if(NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
      message(STATUS "Adding conda prefix to global search paths: $ENV{CONDA_PREFIX}")
      list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
    endif()
  endif()
endif()
```

With this (drop the `else()`):

```cmake
    if(PGO_HOMEBREW_PREFIX AND EXISTS "${PGO_HOMEBREW_PREFIX}/include")
      message(STATUS "Adding Homebrew prefix to global search paths: ${PGO_HOMEBREW_PREFIX}")
      include_directories(SYSTEM "${PGO_HOMEBREW_PREFIX}/include")
      link_directories("${PGO_HOMEBREW_PREFIX}/lib")
      list(APPEND CMAKE_PREFIX_PATH "${PGO_HOMEBREW_PREFIX}")
    endif()
  endif()
endif()
```

- [ ] **Step 2: Add the cross-platform central injection right after the `if(APPLE)` block**

Immediately after the `endif()` that closes `if(APPLE)` (and before `include(CTest)`), add:

```cmake
# Conda builds (PGO_CHECK_CONDA=ON): make standard finders search the active
# conda environment. Per-package finders (Boost, TBB, MKL, ...) historically
# hardcoded their own $CONDA_PREFIX hints, but find_package(BLAS) relies on the
# default search path, which excludes the env when using a non-conda system
# compiler — the cause of "Could NOT find BLAS" in local OpenBLAS dev builds.
# Injecting the prefix once here covers every finder uniformly. The standalone
# (PGO_CHECK_CONDA=OFF) path is unaffected.
if(PGO_CHECK_CONDA AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  if(WIN32)
    list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}/Library" "$ENV{CONDA_PREFIX}")
  else()
    list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  endif()
  message(STATUS "Conda build: prepended $ENV{CONDA_PREFIX} to CMAKE_PREFIX_PATH")
endif()
```

- [ ] **Step 3: Configure-check in a conda env (macOS/OpenBLAS path)**

Prerequisite: the active conda env has the `openblas` dev package (`conda install -n libpgo -c conda-forge openblas`).

Run:

```bash
conda activate libpgo
rm -rf build/base
cmake -S . -B build/base -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DPGO_CHECK_CONDA=ON -DPGO_ENABLE_FULL=ON \
  -DPGO_ENABLE_PYTHON=ON -DPGO_ENABLE_OPENVDB=ON -DPGO_ENABLE_GMSH=ON \
  -DPGO_TET_MESHER_USE_TET_WILD=ON -DPGO_ENABLE_ALEMBIC=ON 2>&1 | tee /tmp/cfg.log
```

Expected: log contains `Conda build: prepended .../envs/libpgo to CMAKE_PREFIX_PATH`, does NOT contain `Could NOT find BLAS`, and ends with `Configuring done` / `Generating done` (no `Configuring incomplete`).

- [ ] **Step 4: Confirm standalone (non-conda) configure still works**

Run:

```bash
rm -rf build/standalone
cmake -S . -B build/standalone -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DPGO_CHECK_CONDA=OFF -DPGO_ENABLE_FULL=ON 2>&1 | tee /tmp/cfg_standalone.log
```

Expected: configure reaches the same dependency-fetch behavior as before this change (it should NOT print the `Conda build: prepended ...` line). It is acceptable if this path requires Homebrew GMP/MPFR as it did before — the point is that this change introduced no regression to the OFF path. Compare against `git stash` baseline if unsure.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt
git commit -m "build: centralize conda prefix injection into CMAKE_PREFIX_PATH

Inject \$CONDA_PREFIX once (cross-platform) when PGO_CHECK_CONDA=ON so every
find_package — including find_package(BLAS) — resolves against the conda env.
Replaces the macOS-only conda branch and fixes 'Could NOT find BLAS' in local
OpenBLAS dev builds. Standalone (non-conda) builds are unaffected."
```

---

## Phase 2 — Remove now-redundant per-finder hints (incremental, optional)

> After Task 1, the conda prefix is already on `CMAKE_PREFIX_PATH`, so the prepend/`_DIR` hints in `boost.cmake`, `tbb.cmake`, and `mkl.cmake` are redundant. Remove them one at a time, re-configuring after each. Stop any time — each task is independent.

### Task 2: Drop the redundant conda prefix prepend in `boost.cmake`

**Files:**
- Modify: `CMakeModules/third-party/boost.cmake`

- [ ] **Step 1: Read the current file** to see the exact conda block.

Run: `sed -n '1,30p' CMakeModules/third-party/boost.cmake`

It contains a guard like `if(NOT PGO_CHECK_CONDA OR "$ENV{CONDA_PREFIX}" STREQUAL "")` that computes `PGO_BOOST_PREFIX` (`$ENV{CONDA_PREFIX}` or `.../Library`) and then `list(PREPEND CMAKE_PREFIX_PATH "${PGO_BOOST_PREFIX}")` before `find_package(Boost CONFIG REQUIRED)`.

- [ ] **Step 2: Remove the `PGO_BOOST_PREFIX` computation and its `list(PREPEND CMAKE_PREFIX_PATH ...)`**, keeping the early-return guard for the non-conda case and the `find_package(Boost CONFIG REQUIRED)` call intact. Do NOT remove any non-conda logic.

- [ ] **Step 3: Configure-check**

Run:
```bash
rm -rf build/base && cmake -S . -B build/base -G Ninja -DPGO_CHECK_CONDA=ON -DPGO_ENABLE_FULL=ON 2>&1 | grep -iE "boost|could not find" | head
```
Expected: Boost is still found (no `Could NOT find Boost`); configure completes.

- [ ] **Step 4: Commit**

```bash
git add CMakeModules/third-party/boost.cmake
git commit -m "build: drop redundant conda prefix prepend in boost.cmake (covered centrally)"
```

### Task 3: Drop the redundant `TBB_DIR` conda hint in `tbb.cmake`

**Files:**
- Modify: `CMakeModules/third-party/tbb.cmake`

- [ ] **Step 1: Remove the conda hint block**

Delete this block (the `find_package(TBB CONFIG REQUIRED)` and the IMPORTED_LOCATION fix-up below it stay):

```cmake
if(PGO_CHECK_CONDA AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  if(WIN32)
    set(CANDIDATE_PATH "$ENV{CONDA_PREFIX}/Library/lib/cmake/TBB")
  else()
    set(CANDIDATE_PATH "$ENV{CONDA_PREFIX}/lib/cmake/TBB")
  endif()

  if(EXISTS "${CANDIDATE_PATH}/TBBConfig.cmake")
    set(TBB_DIR "${CANDIDATE_PATH}")
  endif()
endif()
```

- [ ] **Step 2: Configure-check**

Run:
```bash
rm -rf build/base && cmake -S . -B build/base -G Ninja -DPGO_CHECK_CONDA=ON -DPGO_ENABLE_FULL=ON 2>&1 | grep -iE "tbb lib:|could not find tbb" | head
```
Expected: still prints `tbb lib: .../envs/libpgo/lib/libtbb...`; no `Could NOT find TBB`.

- [ ] **Step 3: Commit**

```bash
git add CMakeModules/third-party/tbb.cmake
git commit -m "build: drop redundant conda TBB_DIR hint in tbb.cmake (covered centrally)"
```

### Task 4: Drop the redundant conda `MKL_DIR` hint in `mkl.cmake`

**Files:**
- Modify: `CMakeModules/third-party/mkl.cmake`

- [ ] **Step 1: Remove only the conda branch of the `MKL_DIR` resolution**

In the block that sets `MKL_DIR`, delete the `if(PGO_CHECK_CONDA AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")` sub-block that points `MKL_DIR` at `$ENV{CONDA_PREFIX}[/Library]/lib/cmake/mkl`. **Keep** the system-oneAPI fallbacks (`/opt/intel/oneapi/...`, `C:/Program Files (x86)/Intel/oneAPI/...`) and the `find_package(MKL CONFIG)` call — those serve the non-conda path.

- [ ] **Step 2: Configure-check (MKL is Linux/Windows only — verify on a non-macOS conda env, or rely on CI)**

On a Linux conda env with `mkl-devel` installed:
```bash
rm -rf build/base && cmake -S . -B build/base -G Ninja -DPGO_CHECK_CONDA=ON -DPGO_USE_MKL=ON -DPGO_ENABLE_FULL=ON 2>&1 | grep -iE "MKL|could not find" | head
```
Expected: `find_package(MKL CONFIG)` still resolves `MKL::MKL` (no `Could NOT find MKL`). If no Linux env is available locally, defer verification to the CI gate (Task 5) — note this in the commit.

- [ ] **Step 3: Commit**

```bash
git add CMakeModules/third-party/mkl.cmake
git commit -m "build: drop redundant conda MKL_DIR hint in mkl.cmake (covered centrally)"
```

---

## Task 5: CI gate (authoritative validation)

**Files:** none (push + observe)

- [ ] **Step 1: Push the branch and open a PR**

```bash
git push -u origin codex/conda-prefix-centralization
gh pr create --fill
```

- [ ] **Step 2: Confirm all 5 Conda Release matrix jobs go green**

The matrix is: `linux-64-pypgo`, `linux-64-pypgo-mkl`, `osx-arm64-pypgo`, `win-64-pypgo`, `win-64-pypgo-mkl`. Watch with:

```bash
gh run watch $(gh run list --branch codex/conda-prefix-centralization --workflow "Conda Release" --limit 1 --json databaseId --jq '.[0].databaseId')
```

Expected: every build job succeeds. Pay special attention to `win-64-pypgo` and `win-64-pypgo-mkl` (the `/Library` prefix path) and `linux-64-pypgo-mkl` (MKL via central prefix after Task 4).

- [ ] **Step 3: If any job regresses**, the central injection or a removed hint did not cover that platform. Re-add the specific hint for the failing finder/platform (Phase 2 removals are individually revertible) and re-push. Phase 1 (Task 1) should not be reverted — it only adds a search path.

---

## Self-Review Notes

- **Spec coverage:** Central injection (Task 1) = the core elegance/BLAS-fix goal. Hint removal (Tasks 2–4) = the "remove scattered hardcoding" goal. Switch retained (no task removes `PGO_CHECK_CONDA`) = the "keep non-conda entry" decision. CGAL/OpenVDB/Gmsh explicitly out of scope (documented in File Structure). BLAS backend policy untouched (scope note).
- **Risk profile:** Task 1 is purely additive (adds a search path) → safe. Tasks 2–4 each remove one redundancy with an immediate configure check and are independently revertible. Task 5 is the cross-platform truth.
- **Local limitation:** macOS can only validate the OpenBLAS path locally; MKL (Task 4) and Windows `/Library` (Task 1 on Windows) are validated in CI.
