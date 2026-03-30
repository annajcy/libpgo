#!/usr/bin/env python3
"""Export local Conan recipes into the cache without building them."""

import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

RECIPES = [
    ("asa", "1.0.0"),
    ("ccd-safe", "1.0.0"),
    ("ccd-exact", "1.0.0"),
    ("autodiff", "1.1.2"),
    ("tetgen", "1.6.0"),
    ("suitesparse", "7.10.3"),
    ("arpack-ng", "3.9.1"),
    ("geogram", "1.9.0"),
    ("gmsh", "4.13.1"),
    ("alembic", "1.8.9"),
]


def export_recipe(name: str, version: str) -> bool:
    recipe_dir = SCRIPT_DIR / name / "all"
    result = subprocess.run(
        [
            "conan",
            "export",
            str(recipe_dir),
            "--name",
            name,
            "--version",
            version,
        ]
    )
    return result.returncode == 0


def main():
    targets = sys.argv[1:] if len(sys.argv) > 1 else [name for name, _ in RECIPES]
    lookup = {name: version for name, version in RECIPES}

    failed = []
    for name in targets:
        version = lookup.get(name)
        if version is None:
            print(f"[SKIP] unknown recipe: {name}")
            continue
        if not export_recipe(name, version):
            failed.append(name)

    if failed:
        print(f"[FAIL] recipe export failed: {', '.join(failed)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
