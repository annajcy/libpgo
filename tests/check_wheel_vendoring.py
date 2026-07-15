#!/usr/bin/env python3
"""Reject repaired wheels that vendor the environment-owned BLAS runtime."""

from __future__ import annotations

import argparse
from pathlib import Path
import zipfile


FORBIDDEN = {
    "accelerate": ("openblas", "libblas", "liblapack"),
    "mkl": (
        "openblas",
        "libblas",
        "blas.dll",
        "liblapack",
        "lapack.dll",
        "libmkl",
        "mkl_",
    ),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--provider", choices=sorted(FORBIDDEN), required=True)
    parser.add_argument("wheel", nargs="+", type=Path)
    args = parser.parse_args()

    if len(args.wheel) != 1:
        parser.error(f"expected exactly one repaired wheel, found {len(args.wheel)}")

    wheel = args.wheel[0].resolve()
    with zipfile.ZipFile(wheel) as archive:
        names = [
            name.lower()
            for name in archive.namelist()
            if ".libs/" in name or ".dylibs/" in name
        ]

    print("Vendored shared libraries:")
    for name in names:
        print(f"  {name}")

    forbidden = FORBIDDEN[args.provider]
    disallowed = [
        name for name in names if any(token in name for token in forbidden)
    ]
    if disallowed:
        raise RuntimeError(
            "Repaired pypgo wheel unexpectedly vendors environment-owned "
            "BLAS/LAPACK libraries:\n" + "\n".join(disallowed)
        )

    print(
        f"OK: {wheel.name} relies on the environment-provided "
        f"{args.provider} BLAS/LAPACK runtime."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
