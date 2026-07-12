# Tricubic Hermite FEM Experiments

This experiment compares tet-linear, cubic-linear, and cubic-tricubic-Hermite
FEM on conservative r15 bunny and dragon domains under static gravity and
dynamic drop/contact loading.

## Case Matrix

Each study and mode uses the same five formulation cases:

| case | role |
|---|---|
| `tet_ref` | same-domain numerical reference |
| `cubic_linear` | original low-order cubic baseline |
| `cubic_linear_x8` | 2x2x2 comparable-DOF linear refinement |
| `cubic_hermite` | high-order formulation under test |
| `cubic_linear_x27` | 3x3x3 over-resolved linear convergence point |

## Layout

```text
tricubic_hermite_fem/
├── common.py
├── run_experiments.sh
├── run_static.py
├── run_dynamic.py
├── summarize.py
├── assets/
├── mesh/
├── reports/
└── docs/
```

Static and dynamic runners share the same assets and case definitions from
`common.py`. New outputs are written under:

```text
examples/outputs/tricubic_hermite_fem/<study>/<mode>/<case>
```

Legacy output directories are left in place as historical data; they are not
silently reused by the reorganized runners.

## Full Pipeline

The full pipeline generates meshes, tunes tet references by binary search,
runs all 20 study/mode/formulation combinations, and summarizes them:

```bash
CONDA_ENV=libpgo examples/experiments/tricubic_hermite_fem/run_experiments.sh
```

This is an expensive server command. It should not be used as a local smoke
test.

## Individual Runs

```bash
conda run -n libpgo python examples/experiments/tricubic_hermite_fem/run_static.py \
  --study bunny --cases cubic_linear_x27 --force

conda run -n libpgo python examples/experiments/tricubic_hermite_fem/run_dynamic.py \
  --study dragon --cases cubic_hermite --force
```

For dynamic experiments, `--force` starts from timestep zero and is required
for a complete wall-time measurement. Without `--force`, an incomplete run
resumes from its latest checkpoint and records only that resumed segment's
wall time.

## Summaries

Summarize one completed matrix:

```bash
conda run -n libpgo python examples/experiments/tricubic_hermite_fem/summarize.py \
  --study bunny --mode dynamic
```

Summarize all four study/mode matrices:

```bash
conda run -n libpgo python examples/experiments/tricubic_hermite_fem/summarize.py --all
```

Each output root receives `comparison.json`, `comparison.csv`, and
`summary.md`. Dynamic summaries include reference-relative pre-contact,
post-contact, late, and final-frame metrics plus fresh/resumed provenance.

## Mesh Utilities

Mesh utilities live under `mesh/`. For example:

```bash
conda run -n libpgo python examples/experiments/tricubic_hermite_fem/mesh/subdivide_cubic_mesh.py \
  --factor 3 \
  --input examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15.veg \
  --output examples/experiments/tricubic_hermite_fem/assets/veg/cubic/bunny-conservative-r15-subdiv3.veg
```

Generated cubic/tet meshes and conservative boundary surfaces are ignored by
Git. Source OBJ meshes, fixed patches, reports, scripts, and documentation are
tracked.
