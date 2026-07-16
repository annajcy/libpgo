#!/usr/bin/env python3
"""Compare one Eigen GEMM expression with an Accelerate or MKL BLAS backend."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import statistics
import subprocess
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}
CASE_PATTERN = re.compile(r"^EigenBlas/Gemm/([^/]+)/n_(\d+)(?:/real_time)?$")


@dataclass(frozen=True)
class Provider:
    name: str
    single: str
    wide: str

    @property
    def variants(self) -> tuple[str, str, str]:
        return ("EigenInternal", self.single, self.wide)


PROVIDERS = {
    "accelerate": Provider("accelerate", "AccelerateSingle", "AccelerateMulti"),
    "mkl": Provider("mkl", "MklLocal1", "MklC"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("provider", choices=PROVIDERS)
    parser.add_argument("internal", type=Path)
    parser.add_argument("vendor", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-concurrency", type=int)
    parser.add_argument("--repetitions", type=int, default=7)
    parser.add_argument("--min-time", default="0.05s")
    parser.add_argument("--warmup-time", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=20260714)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument("--checksum-relative-tolerance", type=float, default=1e-10)
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def run(
    command: list[str], environment: dict[str, str] | None = None
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env=environment,
    )


def checked_output(
    command: list[str], environment: dict[str, str] | None = None
) -> str:
    result = run(command, environment)
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(command)}\n{result.stdout}")
    return result.stdout


def benchmark_environment(args: argparse.Namespace) -> dict[str, str]:
    environment = os.environ.copy()
    if args.provider == "mkl" and args.max_concurrency is not None:
        environment["PGO_EIGEN_BLAS_MAX_CONCURRENCY"] = str(args.max_concurrency)
    return environment


def undefined_symbols(executable: Path, provider: str) -> str:
    command = ["nm", "-u", str(executable)]
    if provider == "mkl":
        command.insert(1, "-D")
    return checked_output(command)


def verify_linkage(provider: Provider, internal: Path, vendor: Path) -> dict[str, str]:
    if provider.name == "accelerate":
        dependency_command = ["otool", "-L"]
    else:
        dependency_command = ["ldd"]

    internal_dependencies = checked_output([*dependency_command, str(internal)])
    vendor_dependencies = checked_output([*dependency_command, str(vendor)])
    internal_symbols = undefined_symbols(internal, provider.name)
    vendor_symbols = undefined_symbols(vendor, provider.name)
    gemm_symbol = re.compile(
        r"(?:^|\s)_?(?:cblas_)?dgemm_?(?:@\S+)?(?:\s|$)", re.I | re.M
    )

    if gemm_symbol.search(internal_symbols):
        raise RuntimeError("The Eigen-internal control unexpectedly references DGEMM.")

    if provider.name == "accelerate":
        system_accelerate = "/System/Library/Frameworks/Accelerate.framework"
        system_blas = (
            "/System/Library/Frameworks/Accelerate.framework/Versions/A/Frameworks/"
            "vecLib.framework/Versions/A/libBLAS.dylib"
        )
        if system_accelerate in internal_dependencies:
            raise RuntimeError(
                "The Eigen-internal control unexpectedly links Accelerate."
            )
        if system_accelerate not in vendor_dependencies:
            raise RuntimeError(
                "The Accelerate benchmark does not link system Accelerate."
            )
        if system_blas not in vendor_dependencies:
            raise RuntimeError(
                "The Accelerate benchmark does not link system Accelerate libBLAS."
            )
    else:
        internal_lower = internal_dependencies.lower()
        vendor_lower = vendor_dependencies.lower()
        if "libmkl" in internal_lower:
            raise RuntimeError("The Eigen-internal control unexpectedly links MKL.")
        required = {
            "MKL core": r"libmkl_core",
            "MKL LP64 interface": r"libmkl_(?:intel|gf)_lp64",
            "MKL TBB threading layer": r"libmkl_tbb_thread",
            "oneTBB": r"libtbb",
        }
        for label, pattern in required.items():
            if not re.search(pattern, vendor_lower):
                raise RuntimeError(
                    f"The MKL benchmark is missing {label} in ldd output."
                )
        forbidden = ("libiomp5", "libgomp", "libomp.so")
        if any(library in vendor_lower for library in forbidden):
            raise RuntimeError(
                "The MKL benchmark links an OpenMP runtime instead of a pure TBB stack."
            )

    if not gemm_symbol.search(vendor_symbols):
        raise RuntimeError(f"The {provider.name} benchmark does not reference DGEMM.")

    return {
        "internal_dependencies": internal_dependencies,
        "vendor_dependencies": vendor_dependencies,
        "internal_undefined_symbols": internal_symbols,
        "vendor_undefined_symbols": vendor_symbols,
    }


def discover_cases(
    provider: Provider,
    internal: Path,
    vendor: Path,
    environment: dict[str, str],
) -> dict[int, dict[str, tuple[Path, str]]]:
    cases: dict[int, dict[str, tuple[Path, str]]] = {}
    for executable in (internal, vendor):
        output = checked_output(
            [str(executable), "--benchmark_list_tests"], environment
        )
        for line in output.splitlines():
            name = line.strip()
            match = CASE_PATTERN.fullmatch(name)
            if not match:
                continue
            variant, matrix_n = match.groups()
            if variant in provider.variants:
                cases.setdefault(int(matrix_n), {})[variant] = (executable, name)

    return {
        matrix_n: variants
        for matrix_n, variants in sorted(cases.items())
        if set(variants) == set(provider.variants)
    }


def exact_filter(name: str) -> str:
    return f"^{re.escape(name)}$"


def run_one(
    executable: Path,
    name: str,
    min_time: str,
    warmup_time: float,
    output: Path,
    environment: dict[str, str],
) -> dict[str, Any]:
    command = [
        str(executable),
        f"--benchmark_filter={exact_filter(name)}",
        f"--benchmark_min_time={min_time}",
        f"--benchmark_min_warmup_time={warmup_time:g}",
        "--benchmark_repetitions=1",
        f"--benchmark_out={output}",
        "--benchmark_out_format=json",
    ]
    checked_output(command, environment)
    payload = json.loads(output.read_text())
    rows = [
        row
        for row in payload.get("benchmarks", [])
        if row.get("run_type") != "aggregate"
    ]
    if len(rows) != 1:
        raise RuntimeError(f"Expected one result for {name}, found {len(rows)}")
    row = rows[0]
    scale = TIME_SCALE.get(row["time_unit"])
    if scale is None:
        raise RuntimeError(f"Unsupported time unit {row['time_unit']} for {name}")
    row["wall_seconds"] = row["real_time"] * scale
    return row


def validate_checksums(
    measurements: dict[str, dict[str, Any]],
    matrix_n: int,
    relative_tolerance: float,
) -> None:
    baseline = float(measurements["EigenInternal"].get("checksum", math.nan))
    if not math.isfinite(baseline):
        raise RuntimeError(f"Missing finite EigenInternal checksum for n={matrix_n}")
    for variant, row in measurements.items():
        checksum = float(row.get("checksum", math.nan))
        tolerance = max(1e-12, relative_tolerance * max(abs(checksum), abs(baseline)))
        if not math.isfinite(checksum) or abs(checksum - baseline) > tolerance:
            raise RuntimeError(
                f"Checksum mismatch for n={matrix_n}: "
                f"{variant}={checksum}, EigenInternal={baseline}"
            )


def validate_provider_configuration(
    measurements: dict[str, dict[str, Any]],
    provider: Provider,
    matrix_n: int,
) -> None:
    if provider.name != "mkl":
        return

    configured = round(
        float(measurements[provider.single].get("configured_concurrency", math.nan))
    )
    if configured <= 0:
        raise RuntimeError(f"Missing configured MKL concurrency for n={matrix_n}")
    expected_budgets = {provider.single: 1, provider.wide: configured}
    for variant, expected_budget in expected_budgets.items():
        row = measurements[variant]
        arena = round(float(row.get("arena_concurrency", math.nan)))
        budget = round(float(row.get("mkl_local_thread_budget", math.nan)))
        if arena != configured:
            raise RuntimeError(
                f"{variant} used arena concurrency {arena}; expected {configured}"
            )
        if budget != expected_budget:
            raise RuntimeError(
                f"{variant} used MKL local budget {budget}; expected {expected_budget}"
            )


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def bootstrap_median_ci(
    values: list[float], rng: random.Random, samples: int
) -> list[float]:
    medians = [
        statistics.median(values[rng.randrange(len(values))] for _ in values)
        for _ in range(samples)
    ]
    return [percentile(medians, 0.025), percentile(medians, 0.975)]


def summarize(
    records: list[dict[str, Any]],
    provider: Provider,
    seed: int,
    bootstrap_samples: int,
) -> list[dict[str, Any]]:
    rng = random.Random(seed ^ 0xB1A5)
    by_size: dict[int, list[dict[str, Any]]] = {}
    for record in records:
        by_size.setdefault(record["matrix_n"], []).append(record)

    summary = []
    for matrix_n, blocks in sorted(by_size.items()):
        ratio_names = (
            f"{provider.single}/EigenInternal",
            f"{provider.wide}/EigenInternal",
            f"{provider.wide}/{provider.single}",
        )
        ratios = {
            name: [block["ratios"][name] for block in blocks] for name in ratio_names
        }
        summary.append(
            {
                "matrix_n": matrix_n,
                "repetitions": len(blocks),
                "median_wall_seconds": {
                    variant: statistics.median(
                        block["measurements"][variant]["wall_seconds"]
                        for block in blocks
                    )
                    for variant in provider.variants
                },
                "median_ratios": {
                    key: statistics.median(values) for key, values in ratios.items()
                },
                "bootstrap_95pct_ci": {
                    key: bootstrap_median_ci(values, rng, bootstrap_samples)
                    for key, values in ratios.items()
                },
                "median_extra_threads": {
                    variant: statistics.median(
                        float(block["cold_probes"][variant].get("extra_threads", 0))
                        for block in blocks
                    )
                    for variant in provider.variants
                },
            }
        )
    return summary


def main() -> int:
    args = parse_args()
    provider = PROVIDERS[args.provider]
    if args.repetitions < 1 or args.bootstrap_samples < 1 or args.warmup_time < 0:
        raise SystemExit("repetitions, bootstrap samples, and warm-up must be valid")
    if args.max_concurrency is not None and args.max_concurrency <= 0:
        raise SystemExit("--max-concurrency must be positive")
    if args.provider != "mkl" and args.max_concurrency is not None:
        raise SystemExit("--max-concurrency is only meaningful for the MKL provider")

    internal = args.internal.resolve()
    vendor = args.vendor.resolve()
    for executable in (internal, vendor):
        if not executable.exists():
            raise SystemExit(f"Benchmark executable does not exist: {executable}")

    environment = benchmark_environment(args)
    linkage = verify_linkage(provider, internal, vendor)
    cases = discover_cases(provider, internal, vendor, environment)
    if args.case_limit:
        cases = dict(list(cases.items())[: args.case_limit])
    if not cases:
        raise SystemExit(
            f"No complete Eigen/{provider.name} benchmark cases were found"
        )

    print(f"Verified {provider.name} linkage; matched {len(cases)} matrix size(s).")
    for matrix_n in cases:
        print(f"n={matrix_n}")
    if args.dry_run:
        return 0

    schedule = [
        (repetition, matrix_n)
        for repetition in range(1, args.repetitions + 1)
        for matrix_n in cases
    ]
    randomizer = random.Random(args.seed)
    randomizer.shuffle(schedule)
    records: list[dict[str, Any]] = []

    with tempfile.TemporaryDirectory(prefix=f"pgo-eigen-{provider.name}-") as temp_dir:
        temporary = Path(temp_dir)
        for block_index, (repetition, matrix_n) in enumerate(schedule, start=1):
            variants = list(provider.variants)
            randomizer.shuffle(variants)
            print(
                f"[{block_index}/{len(schedule)}] repetition={repetition} n={matrix_n} "
                f"order={','.join(variants)}",
                flush=True,
            )
            measurements: dict[str, dict[str, Any]] = {}
            cold_probes: dict[str, dict[str, Any]] = {}
            for variant in variants:
                executable, name = cases[matrix_n][variant]
                cold_probes[variant] = run_one(
                    executable,
                    name,
                    "1x",
                    0.0,
                    temporary / f"{block_index}-{variant}-cold.json",
                    environment,
                )
                measurements[variant] = run_one(
                    executable,
                    name,
                    args.min_time,
                    args.warmup_time,
                    temporary / f"{block_index}-{variant}-steady.json",
                    environment,
                )

            validate_checksums(cold_probes, matrix_n, args.checksum_relative_tolerance)
            validate_checksums(measurements, matrix_n, args.checksum_relative_tolerance)
            validate_provider_configuration(cold_probes, provider, matrix_n)
            validate_provider_configuration(measurements, provider, matrix_n)
            internal_time = measurements["EigenInternal"]["wall_seconds"]
            single_time = measurements[provider.single]["wall_seconds"]
            wide_time = measurements[provider.wide]["wall_seconds"]
            records.append(
                {
                    "repetition": repetition,
                    "matrix_n": matrix_n,
                    "order": variants,
                    "measurements": measurements,
                    "cold_probes": cold_probes,
                    "ratios": {
                        f"{provider.single}/EigenInternal": single_time / internal_time,
                        f"{provider.wide}/EigenInternal": wide_time / internal_time,
                        f"{provider.wide}/{provider.single}": wide_time / single_time,
                    },
                }
            )

    summary = summarize(records, provider, args.seed, args.bootstrap_samples)
    payload = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "provider": provider.name,
        "internal_benchmark": str(internal),
        "vendor_benchmark": str(vendor),
        "max_concurrency": args.max_concurrency,
        "seed": args.seed,
        "repetitions": args.repetitions,
        "min_time": args.min_time,
        "warmup_time": args.warmup_time,
        "thread_telemetry_source": "cold_probes",
        "linkage": linkage,
        "records": records,
        "summary": summary,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2) + "\n")

    ratio_names = (
        f"{provider.single}/EigenInternal",
        f"{provider.wide}/EigenInternal",
        f"{provider.wide}/{provider.single}",
    )
    print("\nn  single/internal  wide/internal  wide/single")
    for row in summary:
        ratios = row["median_ratios"]
        print(
            f"{row['matrix_n']:4d}  {ratios[ratio_names[0]]:15.4f}  "
            f"{ratios[ratio_names[1]]:13.4f}  {ratios[ratio_names[2]]:11.4f}"
        )
    print(f"\nWrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
