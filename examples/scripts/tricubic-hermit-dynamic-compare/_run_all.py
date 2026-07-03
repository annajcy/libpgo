#!/usr/bin/env python3
"""Run all four cases with checkpointing — tet_ref reuses cache, cubic cases run fresh."""
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parents[2]))

from dynamic_compare import main
raise SystemExit(main(["--cases", "tet_ref", "cubic_linear", "cubic_linear_x8", "cubic_hermite"]))
