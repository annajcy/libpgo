"""Pytest collection controls for optional example-notebook checks."""

from __future__ import annotations

import os


collect_ignore: list[str] = []

if os.environ.get("PYPGO_RUN_NOTEBOOK_TESTS") != "1":
    collect_ignore.extend([
        "test_example_notebooks.py",
        "test_notebook.py",
    ])
