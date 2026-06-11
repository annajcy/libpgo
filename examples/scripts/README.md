# pypgo example notebook generators

Each example notebook has one source generator script in this directory:

- `generate_contact_api_demo.py` -> `../contact_api_demo.ipynb`
- `generate_mesh_api_demo.py` -> `../mesh_api_demo.ipynb`
- `generate_numpy_interoperate.py` -> `../numpy_interoperate.ipynb`
- `generate_static_solve_dragon_gravity_demo.py` -> `../static_solve_dragon_gravity_demo.ipynb`
- `generate_tricubic_hermite_demo.py` -> `../tricubic_hermite_demo.ipynb`
- `generate_tricubic_hermite_box_drop_ipc_demo.py` -> `../tricubic_hermite_box_drop_ipc_demo.ipynb`

Edit the generator script, then regenerate the notebook from the repository root:

```bash
conda run -n libpgo python examples/scripts/generate_contact_api_demo.py
conda run -n libpgo python examples/scripts/generate_mesh_api_demo.py
conda run -n libpgo python examples/scripts/generate_numpy_interoperate.py
conda run -n libpgo python examples/scripts/generate_static_solve_dragon_gravity_demo.py
conda run -n libpgo python examples/scripts/generate_tricubic_hermite_box_drop_ipc_demo.py
```

Validate by executing notebooks top to bottom:

```bash
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute examples/contact_api_demo.ipynb --output /tmp/pypgo_contact_api_demo_executed.ipynb --ExecutePreprocessor.timeout=180
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute examples/mesh_api_demo.ipynb --output /tmp/pypgo_mesh_api_demo_executed.ipynb --ExecutePreprocessor.timeout=180
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute examples/numpy_interoperate.ipynb --output /tmp/pypgo_numpy_interoperate_executed.ipynb --ExecutePreprocessor.timeout=120
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute examples/static_solve_dragon_gravity_demo.ipynb --output /tmp/pypgo_static_solve_dragon_gravity_executed.ipynb --ExecutePreprocessor.timeout=1200
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute examples/tricubic_hermite_demo.ipynb --output /tmp/pypgo_tricubic_hermite_demo_executed.ipynb --ExecutePreprocessor.timeout=400
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute examples/tricubic_hermite_box_drop_ipc_demo.ipynb --output /tmp/pypgo_tricubic_hermite_box_drop_ipc_demo_executed.ipynb --ExecutePreprocessor.timeout=400
```

## Experiment scripts

Besides notebook generators, this directory holds reproducible experiment
scripts:

- `dragon_formulation_comparison.py` — static formulation study (tet
  reference vs cubic-linear vs tricubic Hermite on the voxel dragon, identical
  surface-attachment constraints from `assets/fixed/dragon-surface-fixed.txt`).
  Full run ~45 min; `--quick --cases cubic_linear` for a seconds-level
  pipeline check (covered by `tests/pypgo/test_dragon_comparison_script.py`).

```bash
conda run -n libpgo python examples/scripts/dragon_formulation_comparison.py \
    --output-root /tmp/dragon-comparison
```
