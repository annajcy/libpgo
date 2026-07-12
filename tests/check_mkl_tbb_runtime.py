#!/usr/bin/env python3
"""Verify that NumPy and optionally pypgo load MKL's TBB threading layer."""

from __future__ import annotations

import argparse
import ctypes
import os
from pathlib import Path
import sys


def _loaded_library_names() -> set[str]:
    if sys.platform.startswith("linux"):
        names = set()
        for line in Path("/proc/self/maps").read_text().splitlines():
            path = line.split()[-1]
            if path.startswith("/"):
                names.add(Path(path).name.lower())
        return names

    if sys.platform == "win32":
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
        kernel32.GetModuleHandleW.restype = ctypes.c_void_p
        candidates = {
            "mkl_tbb_thread.dll",
            "mkl_tbb_thread.2.dll",
            "mkl_tbb_thread.3.dll",
            "mkl_intel_thread.dll",
            "mkl_intel_thread.2.dll",
            "mkl_intel_thread.3.dll",
        }
        return {name for name in candidates if kernel32.GetModuleHandleW(name)}

    raise RuntimeError(f"MKL threading-layer verification is unsupported on {sys.platform}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--import-pypgo",
        action="store_true",
        help="also import the installed pypgo native extension",
    )
    args = parser.parse_args()

    configured_layer = os.environ.get("MKL_THREADING_LAYER", "")
    if configured_layer.upper() != "TBB":
        raise RuntimeError(
            "MKL_THREADING_LAYER must be TBB before importing NumPy; "
            f"got {configured_layer!r}"
        )

    import numpy as np

    matrix = np.ones((256, 256), dtype=np.float64)
    np.matmul(matrix, matrix, out=matrix)

    if args.import_pypgo:
        import pypgo._core  # noqa: F401

    libraries = _loaded_library_names()
    tbb_layers = sorted(name for name in libraries if "mkl_tbb_thread" in name)
    intel_layers = sorted(name for name in libraries if "mkl_intel_thread" in name)

    if not tbb_layers:
        raise RuntimeError(
            "NumPy did not load an MKL-TBB threading library. Loaded MKL libraries: "
            + ", ".join(sorted(name for name in libraries if "mkl" in name))
        )
    if intel_layers:
        raise RuntimeError(
            "Intel OpenMP and TBB MKL threading layers are both loaded: "
            + ", ".join(intel_layers + tbb_layers)
        )

    print("MKL-TBB runtime verified:", ", ".join(tbb_layers))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
