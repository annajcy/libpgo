#!/usr/bin/env python3
"""Verify that one macOS pypgo extension links only system Accelerate BLAS."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess


SYSTEM_ACCELERATE = "/System/Library/Frameworks/Accelerate.framework"
SYSTEM_BLAS = (
    f"{SYSTEM_ACCELERATE}/Versions/A/Frameworks/vecLib.framework/"
    "Versions/A/libBLAS.dylib"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("extension", nargs="+", type=Path)
    args = parser.parse_args()

    if len(args.extension) != 1:
        parser.error(
            f"expected exactly one native extension, found {len(args.extension)}"
        )

    extension = args.extension[0].resolve()
    dependencies = subprocess.check_output(
        ["otool", "-L", str(extension)], text=True
    )
    print(dependencies, end="")

    if SYSTEM_ACCELERATE not in dependencies:
        raise RuntimeError(
            f"{extension} does not link the system Accelerate framework"
        )
    if SYSTEM_BLAS not in dependencies:
        raise RuntimeError(
            f"{extension} does not link Accelerate's BLAS runtime directly"
        )

    unexpected = [
        line.strip()
        for line in dependencies.splitlines()
        if re.search(r"lib(open)?blas|liblapack", line, re.IGNORECASE)
        and SYSTEM_ACCELERATE not in line
    ]
    if unexpected:
        raise RuntimeError(
            f"{extension} links an alternate BLAS/LAPACK runtime:\n"
            + "\n".join(unexpected)
        )

    print(f"OK: {extension} links only system Accelerate BLAS/LAPACK.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
