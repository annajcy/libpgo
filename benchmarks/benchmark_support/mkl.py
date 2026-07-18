"""Fixtures specific to the supported Linux oneMKL + oneTBB benchmark stack."""

from __future__ import annotations

import os
import re
from collections.abc import Mapping
from pathlib import Path

from benchmark_support.process import checked_output


def mkl_tbb_environment(
    overrides: Mapping[str, str] | None = None,
) -> dict[str, str]:
    environment = os.environ.copy()
    environment["MKL_THREADING_LAYER"] = "TBB"
    if overrides:
        environment.update(overrides)
    return environment


def verify_mkl_tbb_probe_linkage(
    probe: Path, environment: Mapping[str, str] | None = None
) -> str:
    dependencies = checked_output(["ldd", probe], environment)
    lowered = dependencies.lower()
    required = ("libmkl_core", "libmkl_tbb_thread", "libtbb")
    missing = [library for library in required if library not in lowered]
    if missing:
        raise RuntimeError(f"Probe is missing required libraries: {missing}")
    forbidden = ("libiomp5", "libgomp", "libomp.so")
    present = [library for library in forbidden if library in lowered]
    if present:
        raise RuntimeError(f"Probe unexpectedly links OpenMP runtimes: {present}")
    return dependencies


def verify_mkl_tbb_benchmark_linkage(
    executable: Path,
    environment: Mapping[str, str] | None = None,
    *,
    require_dgemm: bool = True,
) -> dict[str, str]:
    dependencies = checked_output(["ldd", executable], environment)
    undefined_symbols = checked_output(["nm", "-D", "-u", executable], environment)
    dependencies_lower = dependencies.lower()

    required = {
        "MKL core": r"libmkl_core",
        "MKL LP64 interface": r"libmkl_(?:intel|gf)_lp64",
        "MKL TBB threading layer": r"libmkl_tbb_thread",
        "oneTBB": r"libtbb",
    }
    for label, pattern in required.items():
        if not re.search(pattern, dependencies_lower):
            raise RuntimeError(f"Benchmark is missing {label} in ldd output.")

    forbidden = ("libiomp5", "libgomp", "libomp.so")
    if any(library in dependencies_lower for library in forbidden):
        raise RuntimeError(
            "Benchmark links an OpenMP runtime instead of a pure MKL-TBB stack."
        )

    dgemm = re.compile(r"(?:^|\s)_?(?:cblas_)?dgemm_?(?:@\S+)?(?:\s|$)", re.I | re.M)
    if require_dgemm and not dgemm.search(undefined_symbols):
        raise RuntimeError("Benchmark does not expose a dynamic DGEMM reference.")
    return {"dependencies": dependencies, "undefined_symbols": undefined_symbols}
