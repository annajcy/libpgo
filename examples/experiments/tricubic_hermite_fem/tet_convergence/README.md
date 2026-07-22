# Tet Convergence Experiment

This directory implements the pre-registered static spatial-convergence test
in [`PLAN.md`](PLAN.md).  It does not replace the canonical `tet_ref` manifest
or outputs.

## 1. Generate independent meshes

```bash
MKL_THREADING_LAYER=TBB conda run -n libpgo python -u \
  examples/experiments/tricubic_hermite_fem/tet_convergence/generate_meshes.py \
  --study bunny
```

The command reads the canonical L0 selection, tunes L1/L2 to approximately
2x/4x its actual tet DOFs, and generates the factor-four cubic mesh.  Use
`--levels tet_L1 tet_L2 cubic_linear_x64` to select a subset.

## 2. Run the new static cases

```bash
MKL_THREADING_LAYER=TBB conda run -n libpgo python -u \
  examples/experiments/tricubic_hermite_fem/tet_convergence/run_static.py \
  --study bunny --force
```

Cases run sequentially by default.  They can be launched individually with
`--cases tet_L0 tet_L0_tight tet_L1 tet_L2 cubic_linear_x64`.
`--num-threads N` is an execution-only TBB override; the effective value is
recorded in every case signature.  Omit it to reproduce the canonical 12-thread
setting.

## 3. Analyze against the canonical controls

```bash
conda run -n libpgo python \
  examples/experiments/tricubic_hermite_fem/tet_convergence/analyze.py \
  --study bunny
```

The analysis reuses canonical `cubic_linear`, `cubic_linear_x8`,
`cubic_hermite`, and `cubic_linear_x27` final surfaces.  It writes
`convergence.json`, `distances.csv`, and `report.md` into the independent
convergence output root.

## Serial server run

For a single non-overlapping Bunny-then-Dragon run, use:

```bash
CONDA_BIN=/home/byk/anaconda3/bin/conda NUM_THREADS=21 \
  examples/experiments/tricubic_hermite_fem/tet_convergence/run_serial.sh
```

The script force-runs all five convergence cases for one study, analyzes it,
and only then advances to the next study.  A server operator can wrap this
command in one tmux session; the script itself never creates parallel study
processes.
