"""Pipeline test for examples/scripts/dragon_formulation_comparison.py."""

import json
import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "examples" / "scripts" / \
    "dragon_formulation_comparison.py"


def test_comparison_script_quick_linear(tmp_path):
    """--quick --cases cubic_linear exercises the full pipeline in seconds."""
    proc = subprocess.run(
        [sys.executable, str(SCRIPT),
         "--output-root", str(tmp_path),
         "--cases", "cubic_linear",
         "--quick"],
        capture_output=True, text=True, timeout=300,
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    report = json.loads((tmp_path / "report.json").read_text())
    entry = report["cubic_linear"]
    assert entry["num_dofs"] == 3741
    assert entry["max_u"] > 0.0
    assert entry["pin_residual"] < 0.01  # soft pins hold within 10mm even unconverged
    assert "rel_l2_vs_ref" not in entry  # no reference in this subset
    assert (tmp_path / "report.md").exists()
    assert (tmp_path / "cubic_linear" / "final_surface.obj").exists()
