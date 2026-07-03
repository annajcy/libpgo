#!/usr/bin/env python3
"""Phase 10A validation: run 1 frame from state0500.npz with PGO_PROFILE_DYNAMIC=1."""
import json
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np
import pypgo as pgo
import pypgo.parallel as pp
import pypgo.profiling as _profiling
from pypgo.mesh.volume import read_veg
from pypgo.tools.sim._config import load_config
from pypgo.tools.sim._runners import run_dynamic
from pypgo.tools.sim._scene import build_scene

ASSETS = SCRIPT_DIR / "assets"
OUTPUT_DIR = ROOT / "examples" / "outputs" / "bunny-dynamic-drop-compare-conservative-r15" / "cubic_hermite"
CHECKPOINT = OUTPUT_DIR / "checkpoints" / "state0500.npz"
PROFILE_PATH = Path("/tmp/phase10a_profile.jsonl")

# Clean any prior profile
if PROFILE_PATH.exists():
    PROFILE_PATH.unlink()

# Build same config as dynamic_compare.py
overrides = {
    "mesh.volume": str(ASSETS / "veg" / "cubic" / "bunny-conservative-r15.veg"),
    "mesh.surface": str(ASSETS / "obj" / "bunny.obj"),
    "mesh.formulation": "cubic-tricubic-hermite",
    "material.model": "stable_neo",
    "material.enable_material_max_step": False,
    "loads.gravity": [0.0, -9.81, 0.0],
    "initial_state.velocity": [0.0, 0.0, 0.0],
    "contact": [{
        "model": "ipc",
        "dhat": 0.002,
        "dhat_external": 0.005,
        "kappa": 3000.0,
        "obstacles": [{"mesh": str(ASSETS / "obj" / "bottom.1.obj")}],
    }],
    "solver.max_iterations": 200,
    "solver.gradient_tolerance": 1e-4,
    "dynamic.timestep": 0.001,
    "dynamic.num_steps": 501,  # resume from 500, run 1 step (frame 501)
    "dynamic.integrator": "implicit_euler",
    "dynamic.damping": [0.0, 0.0],
    "dynamic.resume": str(CHECKPOINT),
    "output.directory": str(OUTPUT_DIR),
    "output.write_surfaces": False,
    "output.write_abc": False,
    "output.write_checkpoints": False,
    "output.dump_interval": 10000,  # no dumps during single-frame run
}
pp.set_num_threads(32)

cfg = load_config(mesh_type="cubic", mode="dynamic", overrides=overrides)
print(f"[Phase10A] Config loaded, mesh_type={cfg.mesh_type} formulation={cfg.mesh.formulation}")

# Monkey-patch the profile path to write to our temp file
import pypgo.tools.sim._runners as _runners
_orig_write = _runners._write_dynamic_profile_row
def _patched_write(path, row):
    _orig_write(PROFILE_PATH, row)
_runners._write_dynamic_profile_row = _patched_write

print(f"[Phase10A] Building scene...")
bundle = build_scene(cfg)
print(f"[Phase10A] Scene built: {bundle.num_dofs} DOFs")

print(f"[Phase10A] Running 1 frame from {CHECKPOINT.name}...")
started = time.perf_counter()
# We need PGO_PROFILE_DYNAMIC in env — set it here
import os
os.environ["PGO_PROFILE_DYNAMIC"] = "1"

# Re-import to pick up env
_profiling.set_enabled(True)
_profiling.reset()

summary = run_dynamic(bundle, cfg)
elapsed = time.perf_counter() - started
print(f"[Phase10A] Done in {elapsed:.1f}s")

# Analyze the profile
if PROFILE_PATH.exists():
    with open(PROFILE_PATH) as f:
        lines = f.readlines()
    print(f"\n=== Phase 10A Validation Results ===")
    print(f"Profile rows written: {len(lines)}")
    
    for i, line in enumerate(lines):
        row = json.loads(line)
        sd = row.get("solver_diagnostics", {})
        diag = sd.get("diagnostics", {})
        ni = diag.get("newton_iterations")
        
        print(f"\nRow {i}: frame_index={row.get('frame_index')} "
              f"status={row.get('solver_status')} "
              f"iterations={row.get('solver_iterations')} "
              f"accepted={row.get('accepted')}")
        print(f"  step_wall_seconds={row.get('step_wall_seconds', 'N/A')}")
        
        if ni is not None:
            print(f"  ✅ newton_iterations: {len(ni)} traces recorded")
            for j, trace in enumerate(ni):
                print(f"    trace[{j}]: iter={trace.get('iteration')} "
                      f"energy_before={trace.get('energy_before')} "
                      f"energy_after={trace.get('energy_after')} "
                      f"energy_delta={trace.get('energy_delta')} "
                      f"grad_max_before={trace.get('grad_max_before')} "
                      f"grad_max_after={trace.get('grad_max_after')} "
                      f"grad_reduction_ratio={trace.get('grad_reduction_ratio')} "
                      f"accepted_alpha={trace.get('accepted_alpha')} "
                      f"line_search_iterations={trace.get('line_search_iterations')} "
                      f"line_search_status={trace.get('line_search_status')} "
                      f"factorize_seconds={trace.get('factorize_seconds')} "
                      f"solve_seconds={trace.get('solve_seconds')} "
                      f"iteration_wall_seconds={trace.get('iteration_wall_seconds')} "
                      f"low_value={trace.get('low_value')} "
                      f"symbolic_rebuilt={trace.get('symbolic_rebuilt')}")
        else:
            print(f"  ❌ newton_iterations: NOT PRESENT in diagnostics!")
            print(f"  Available diag keys: {sorted(diag.keys())}")
        
        # Summary fields
        for key in ["newton_convergence_reason", "newton_convergence_reason_name",
                     "newton_convergence_threshold",
                     "newton_low_value_iteration_count", "newton_tiny_step_count",
                     "newton_small_alpha_count", "newton_symbolic_rebuild_count",
                     "newton_worst_progress_iteration", "newton_worst_progress_ratio",
                     "newton_total_factorize_seconds", "newton_total_solve_seconds"]:
            print(f"  {key} = {diag.get(key, 'MISSING')}")
else:
    print(f"❌ Profile file not found at {PROFILE_PATH}")

print(f"\n=== Comparison with Phase 9D baseline ===")
print(f"Phase 9D reference: frame 501, CONVERGED, ~193.5s Newton solve")
# The profile now shows current numbers for comparison
