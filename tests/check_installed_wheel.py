#!/usr/bin/env python3
"""Verify imports and build metadata from an installed pypgo wheel."""

from __future__ import annotations

import importlib
import os
from pathlib import Path

import numpy as np


def main() -> int:
    print(f"numpy {np.__version__} from {Path(np.__file__).resolve()}")
    for module_name in (
        "pypgo",
        "pypgo._core",
        "pypgo.energy",
        "pypgo.mesh",
        "pypgo.solver",
    ):
        print(f"import {module_name}")
        importlib.import_module(module_name)

    import pypgo
    import pypgo._core as core

    package_file = Path(pypgo.__file__).resolve()
    workspace_value = os.environ.get("GITHUB_WORKSPACE")
    if workspace_value:
        workspace = Path(workspace_value).resolve()
        if workspace in package_file.parents:
            raise RuntimeError(
                f"import resolved to the source checkout instead of the wheel: {package_file}"
            )

    info = core.build_info()
    if info["module"] != "pypgo._core":
        raise RuntimeError(f"unexpected build_info module: {info!r}")
    print(f"pypgo wheel import OK from {package_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
