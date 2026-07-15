#!/usr/bin/env python3
"""Verify that NumPy and optionally pypgo use the system Accelerate runtime."""

from __future__ import annotations

import argparse
import ctypes
from pathlib import Path
import sys


def _loaded_image_paths() -> set[Path]:
    if sys.platform != "darwin":
        raise RuntimeError(
            f"Accelerate runtime verification is unsupported on {sys.platform}"
        )

    process = ctypes.CDLL(None)
    process._dyld_image_count.restype = ctypes.c_uint32
    process._dyld_get_image_name.argtypes = [ctypes.c_uint32]
    process._dyld_get_image_name.restype = ctypes.c_char_p

    paths = set()
    for index in range(process._dyld_image_count()):
        raw_path = process._dyld_get_image_name(index)
        if raw_path:
            paths.add(Path(raw_path.decode(errors="replace")))
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--import-pypgo",
        action="store_true",
        help="also import the installed pypgo native extension",
    )
    args = parser.parse_args()

    import numpy as np

    rng = np.random.default_rng(7)
    left = rng.standard_normal((256, 256))
    right = rng.standard_normal((256, 256))
    product = left @ right
    if not np.isfinite(product).all():
        raise RuntimeError("NumPy GEMM through Accelerate produced non-finite values")

    matrix = left + 256.0 * np.eye(left.shape[0])
    solution = np.linalg.solve(matrix, right[:, 0])
    relative_residual = np.linalg.norm(matrix @ solution - right[:, 0]) / np.linalg.norm(
        right[:, 0]
    )
    if not np.isfinite(relative_residual) or relative_residual > 1.0e-10:
        raise RuntimeError(
            "NumPy LAPACK solve through Accelerate failed: "
            f"relative residual {relative_residual}"
        )

    if args.import_pypgo:
        import pypgo._core  # noqa: F401

    image_paths = _loaded_image_paths()
    image_strings = {str(path) for path in image_paths}
    lower_image_strings = {path.lower() for path in image_strings}

    required_fragments = {
        "conda newaccelerate shim": "libblas_reexport.dylib",
        "system Accelerate framework": "/System/Library/Frameworks/Accelerate.framework/",
        "system Accelerate BLAS": "/vecLib.framework/Versions/A/libBLAS.dylib",
        "system Accelerate LAPACK": "/vecLib.framework/Versions/A/libLAPACK.dylib",
    }
    missing = [
        label
        for label, fragment in required_fragments.items()
        if not any(fragment in path for path in image_strings)
    ]
    if missing:
        relevant = sorted(
            path
            for path in image_strings
            if "accelerate" in path.lower()
            or "blas" in path.lower()
            or "lapack" in path.lower()
        )
        raise RuntimeError(
            "Missing expected Accelerate runtime images "
            f"({', '.join(missing)}). Loaded relevant images: {', '.join(relevant)}"
        )

    openblas_images = sorted(
        path for path in lower_image_strings if "openblas" in path
    )
    if openblas_images:
        raise RuntimeError(
            "NumPy loaded OpenBLAS alongside Accelerate: " + ", ".join(openblas_images)
        )

    print(
        "NumPy system Accelerate runtime verified; "
        f"solve relative residual={relative_residual:.3e}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
