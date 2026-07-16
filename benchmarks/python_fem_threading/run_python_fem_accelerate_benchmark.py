#!/usr/bin/env python3
"""Run the pure-Python FEM benchmark with Accelerate SINGLE/MULTI policies."""

from run_python_fem_threading_benchmark import main


if __name__ == "__main__":
    raise SystemExit(main(default_backend="accelerate"))
