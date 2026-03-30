#!/usr/bin/env python3
"""逐个 conan create 私有 recipe，验证独立构建。"""

import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

# 按依赖复杂度排序
RECIPES = [
    ("asa",          "1.0.0"),
    ("ccd-safe",     "1.0.0"),
    ("ccd-exact",    "1.0.0"),
    ("autodiff",     "1.1.2"),
    ("tetgen",       "1.6.0"),
    ("suitesparse",  "7.10.3"),
    ("arpack-ng",    "3.9.1"),
    ("geogram",      "1.9.0"),
    ("gmsh",         "4.13.1"),
    ("alembic",      "1.8.9"),
]


def create_recipe(name: str, version: str) -> bool:
    recipe_dir = SCRIPT_DIR / name / "all"
    print(f"\n{'='*60}")
    print(f"  conan create {name}/{version}")
    print(f"{'='*60}")
    result = subprocess.run(
        ["conan", "create", str(recipe_dir), "--build=missing"],
    )
    return result.returncode == 0


def main():
    targets = sys.argv[1:] if len(sys.argv) > 1 else [r[0] for r in RECIPES]
    lookup = {name: ver for name, ver in RECIPES}

    passed, failed = [], []
    for name in targets:
        ver = lookup.get(name)
        if ver is None:
            print(f"[SKIP] unknown recipe: {name}")
            continue
        if create_recipe(name, ver):
            passed.append(name)
        else:
            failed.append(name)

    print(f"\n{'='*60}")
    print(f"  PASSED: {len(passed)}  FAILED: {len(failed)}")
    if failed:
        print(f"  Failed: {', '.join(failed)}")
    print(f"{'='*60}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
