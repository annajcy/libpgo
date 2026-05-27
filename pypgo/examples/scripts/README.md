# pypgo example notebook generators

Each example notebook has one source generator script in this directory:

- `generate_mesh_api_demo.py` -> `../mesh_api_demo.ipynb`
- `generate_numpy_interoperate.py` -> `../numpy_interoperate.ipynb`

Edit the generator script, then regenerate the notebook from the repository root:

```bash
conda run -n libpgo python pypgo/examples/scripts/generate_mesh_api_demo.py
conda run -n libpgo python pypgo/examples/scripts/generate_numpy_interoperate.py
```

Validate by executing notebooks top to bottom:

```bash
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute pypgo/examples/mesh_api_demo.ipynb --output /tmp/pypgo_mesh_api_demo_executed.ipynb --ExecutePreprocessor.timeout=180
conda run -n libpgo python -m jupyter nbconvert --to notebook --execute pypgo/examples/numpy_interoperate.ipynb --output /tmp/pypgo_numpy_interoperate_executed.ipynb --ExecutePreprocessor.timeout=120
```
