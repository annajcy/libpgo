#!/usr/bin/env python3
"""Distinguish executor switching from real multi-threaded PARDISO aftermath."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import random
import shlex
import statistics
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1]
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.harness import (  # noqa: E402
    add_benchmark_harness_arguments,
    prepare_benchmark_host,
)
from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
from benchmark_support.schedule import balanced_order, order_configuration  # noqa: E402
from benchmark_support.mkl import (  # noqa: E402
    mkl_tbb_environment,
    verify_mkl_tbb_probe_linkage,
)
from benchmark_support.process import resolve_file  # noqa: E402
from benchmark_support.warmup import (  # noqa: E402
    add_workload_warmup_arguments,
    validate_workload_warmup_arguments,
)


RESULT_PREFIX = "PGO_CUBIC_LINEAR_PARDISO_AFTERMATH_RESULT"
CASES = ("none", "noop1", "noop8", "pardiso1", "pardiso8")
INTEGER_FIELDS = {
    "iteration",
    "configured_global_concurrency",
    "effective_global_concurrency",
    "configured_arena_concurrency",
    "configured_linear_mkl_budget",
    "configured_evaluation_mkl_budget",
    "observed_linear_mkl_budget",
    "observed_evaluation_mkl_budget",
    "observed_evaluation_arena_concurrency",
    "prelude_worker_entries",
    "prelude_peak_workers",
    "mesh_vertices",
    "mesh_elements",
    "dofs",
    "fixed_dofs",
    "reduced_rows",
    "reduced_nnz",
    "configured_warmup_min_operations",
    "actual_warmup_operations",
}
FLOAT_FIELDS = {
    "prelude_seconds",
    "prelude_process_cpu_seconds",
    "evaluation_execute_seconds",
    "evaluation_kernel_seconds",
    "energy",
    "gradient_squared_norm",
    "gradient_max_abs",
    "hessian_abs_sum",
    "hessian_squared_norm",
    "hessian_max_abs",
    "solve_squared_norm",
    "configured_warmup_seconds",
    "actual_warmup_seconds",
}
SIGNATURE_FIELDS = (
    "energy",
    "gradient_squared_norm",
    "gradient_max_abs",
    "hessian_abs_sum",
    "hessian_squared_norm",
    "hessian_max_abs",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("--mesh", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument("--reserved-slots", type=int, default=1)
    parser.add_argument("--measured-iterations", type=int, default=5)
    parser.add_argument("--repetitions", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20260717)
    parser.add_argument("--bootstrap-samples", type=int, default=20000)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--dry-run", action="store_true")
    add_workload_warmup_arguments(
        parser, default_seconds=0.0, default_min_operations=10
    )
    add_benchmark_harness_arguments(parser)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.concurrency <= 0:
        raise ValueError("--concurrency must be positive")
    if args.reserved_slots < 0 or args.reserved_slots >= args.concurrency:
        raise ValueError("--reserved-slots must be in [0, concurrency)")
    validate_workload_warmup_arguments(args)
    if args.measured_iterations <= 0 or args.repetitions <= 0:
        raise ValueError("--measured-iterations and --repetitions must be positive")
    if args.bootstrap_samples <= 0:
        raise ValueError("--bootstrap-samples must be positive")
    if len(set(args.cases)) != len(args.cases):
        raise ValueError("--cases must not contain duplicates")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_record(line: str) -> dict[str, Any] | None:
    if not line.startswith(RESULT_PREFIX + " "):
        return None
    record: dict[str, Any] = {}
    for token in shlex.split(line[len(RESULT_PREFIX) + 1 :]):
        key, value = token.split("=", 1)
        if key in INTEGER_FIELDS:
            record[key] = int(value)
        elif key in FLOAT_FIELDS:
            record[key] = float(value)
        else:
            record[key] = value
    missing = (INTEGER_FIELDS | FLOAT_FIELDS | {"case", "prelude"}) - record.keys()
    if missing:
        raise RuntimeError(f"Probe record is missing fields: {sorted(missing)}")
    return record


def command_for(
    args: argparse.Namespace, probe: Path, mesh: Path, case: str
) -> list[str]:
    command = [
        str(probe),
        f"--case={case}",
        f"--mesh={mesh}",
        f"--concurrency={args.concurrency}",
        f"--reserved-slots={args.reserved_slots}",
        f"--warmup-seconds={args.warmup_seconds}",
        f"--warmup-min-operations={args.warmup_min_operations}",
        f"--measured-iterations={args.measured_iterations}",
    ]
    return command


def median_record(records: list[dict[str, Any]], repetition: int) -> dict[str, Any]:
    first = records[0]
    return {
        "case": first["case"],
        "prelude": first["prelude"],
        "repetition": repetition,
        "prelude_seconds": statistics.median(r["prelude_seconds"] for r in records),
        "prelude_process_cpu_seconds": statistics.median(
            r["prelude_process_cpu_seconds"] for r in records
        ),
        "prelude_cpu_over_wall": statistics.median(
            r["prelude_process_cpu_seconds"] / r["prelude_seconds"]
            for r in records
            if r["prelude_seconds"] > 0.0
        ),
        "prelude_worker_entries": statistics.median(
            r["prelude_worker_entries"] for r in records
        ),
        "prelude_peak_workers": statistics.median(
            r["prelude_peak_workers"] for r in records
        ),
        "evaluation_execute_seconds": statistics.median(
            r["evaluation_execute_seconds"] for r in records
        ),
        "evaluation_kernel_seconds": statistics.median(
            r["evaluation_kernel_seconds"] for r in records
        ),
    }


def close(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-11)


def validate_block(raw_by_case: dict[str, list[dict[str, Any]]]) -> None:
    reference = next(iter(raw_by_case.values()))[0]
    exact_fields = (
        "configured_evaluation_mkl_budget",
        "observed_evaluation_mkl_budget",
        "observed_evaluation_arena_concurrency",
        "mesh_vertices",
        "mesh_elements",
        "dofs",
        "fixed_dofs",
        "reduced_rows",
        "reduced_nnz",
    )
    for case, records in raw_by_case.items():
        if len(records) == 0:
            raise RuntimeError(f"No records for {case}")
        for record in records:
            for field in exact_fields:
                if record[field] != reference[field]:
                    raise RuntimeError(f"{case}: mismatched {field}")
            if record["observed_evaluation_mkl_budget"] != 1:
                raise RuntimeError(f"{case}: evaluation did not observe MKL budget 1")
            for field in SIGNATURE_FIELDS:
                if not close(record[field], reference[field]):
                    raise RuntimeError(
                        f"{case}: correctness signature {field} differs: "
                        f"{record[field]} versus {reference[field]}"
                    )
            expected_linear = {
                "none": -1,
                "noop1": 1,
                "noop8": 8,
                "pardiso1": 1,
                "pardiso8": 8,
            }[case]
            if record["observed_linear_mkl_budget"] != expected_linear:
                raise RuntimeError(
                    f"{case}: observed linear budget "
                    f"{record['observed_linear_mkl_budget']} != {expected_linear}"
                )


def summarize(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for case in CASES:
        selected = [sample for sample in samples if sample["case"] == case]
        if not selected:
            continue
        row: dict[str, Any] = {"case": case, "samples": len(selected)}
        for metric in (
            "prelude_seconds",
            "prelude_process_cpu_seconds",
            "prelude_cpu_over_wall",
            "prelude_worker_entries",
            "prelude_peak_workers",
            "evaluation_execute_seconds",
            "evaluation_kernel_seconds",
        ):
            values = [sample[metric] for sample in selected]
            row[f"median_{metric}"] = statistics.median(values)
            row[f"mad_{metric}"] = statistics.median(
                abs(value - row[f"median_{metric}"]) for value in values
            )
        result.append(row)

    by_case = {row["case"]: row for row in result}
    for row in result:
        reference_name = "pardiso1" if row["case"] == "pardiso8" else "noop1"
        reference = by_case.get(reference_name)
        if reference is not None:
            for metric in ("evaluation_execute_seconds", "evaluation_kernel_seconds"):
                row[f"ratio_{metric}_to_{reference_name}"] = (
                    row[f"median_{metric}"] / reference[f"median_{metric}"]
                )
    return result


def paired_contrasts(
    samples: list[dict[str, Any]], bootstrap_samples: int, seed: int
) -> list[dict[str, Any]]:
    by_key = {(sample["repetition"], sample["case"]): sample for sample in samples}
    repetitions = sorted({sample["repetition"] for sample in samples})
    requested = (
        ("none_over_noop1", "none", "noop1"),
        ("noop8_over_noop1", "noop8", "noop1"),
        ("pardiso1_over_noop1", "pardiso1", "noop1"),
        ("pardiso8_over_noop8", "pardiso8", "noop8"),
        ("pardiso8_over_pardiso1", "pardiso8", "pardiso1"),
    )
    rng = random.Random(seed)
    rows: list[dict[str, Any]] = []
    for contrast, numerator, denominator in requested:
        paired_repetitions = [
            repetition
            for repetition in repetitions
            if (repetition, numerator) in by_key and (repetition, denominator) in by_key
        ]
        if not paired_repetitions:
            continue
        for metric in ("evaluation_execute_seconds", "evaluation_kernel_seconds"):
            ratios = [
                by_key[repetition, numerator][metric]
                / by_key[repetition, denominator][metric]
                for repetition in paired_repetitions
            ]
            differences = [
                by_key[repetition, numerator][metric]
                - by_key[repetition, denominator][metric]
                for repetition in paired_repetitions
            ]
            boot_ratios: list[float] = []
            boot_differences: list[float] = []
            for _ in range(bootstrap_samples):
                indices = [
                    rng.randrange(len(paired_repetitions)) for _ in paired_repetitions
                ]
                boot_ratios.append(
                    statistics.median(ratios[index] for index in indices)
                )
                boot_differences.append(
                    statistics.median(differences[index] for index in indices)
                )
            boot_ratios.sort()
            boot_differences.sort()
            lower = max(0, int(0.025 * bootstrap_samples) - 1)
            upper = min(bootstrap_samples - 1, int(0.975 * bootstrap_samples))
            rows.append(
                {
                    "contrast": contrast,
                    "numerator": numerator,
                    "denominator": denominator,
                    "metric": metric,
                    "paired_samples": len(paired_repetitions),
                    "median_paired_ratio": statistics.median(ratios),
                    "paired_ratio_ci95_low": boot_ratios[lower],
                    "paired_ratio_ci95_high": boot_ratios[upper],
                    "median_paired_difference_seconds": statistics.median(differences),
                    "paired_difference_ci95_low_seconds": boot_differences[lower],
                    "paired_difference_ci95_high_seconds": boot_differences[upper],
                }
            )
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    validate_args(args)
    probe = resolve_file(args.probe, "probe", executable=True)
    mesh = resolve_file(args.mesh, "mesh")
    out = args.out.expanduser().resolve()
    environment = mkl_tbb_environment(
        {
            "MKL_NUM_THREADS": str(args.concurrency),
            "MKL_DYNAMIC": "FALSE",
            "OMP_NUM_THREADS": str(args.concurrency),
            "OMP_DYNAMIC": "FALSE",
        }
    )

    rng = random.Random(args.seed)
    blocks: list[list[tuple[int, str]]] = []
    for repetition in range(args.repetitions):
        block = [
            (repetition, case)
            for case in balanced_order(
                args.cases,
                repetition=repetition,
                seed=args.seed,
                block_key="cubic_linear_pardiso_aftermath",
            )
        ]
        blocks.append(block)
    # Shuffle complete repetition blocks while retaining one sample per case per block.
    rng.shuffle(blocks)
    jobs = [job for block in blocks for job in block]
    order = order_configuration(
        args.repetitions,
        [("cases", args.cases)],
        allow_incomplete=args.allow_incomplete_order_cycle,
    )

    if args.dry_run:
        for repetition, case in jobs:
            print(f"r={repetition} {shlex.join(command_for(args, probe, mesh, case))}")
        return 0

    out.mkdir(parents=True, exist_ok=False)
    linkage = verify_mkl_tbb_probe_linkage(probe, environment)
    host_environment = prepare_benchmark_host(args, workers=args.concurrency)
    raw: list[dict[str, Any]] = []
    samples: list[dict[str, Any]] = []
    pending: dict[int, dict[str, list[dict[str, Any]]]] = {}
    manifest = {
        "runner": runner_manifest(Path(__file__)),
        "schema_version": 1,
        "benchmark": "cubic_linear_pardiso_aftermath",
        "completed": False,
        "valid": True,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "hypothesis": (
            "If noop8/noop1 is flat but pardiso8/pardiso1 slows the following "
            "evaluation=1, actual multi-threaded PARDISO aftermath—not executor/TLS "
            "switching alone—causes the slowdown."
        ),
        "arguments": vars(args)
        | {"probe": str(probe), "mesh": str(mesh), "out": str(out)},
        "mesh_sha256": sha256(mesh),
        "probe_sha256": sha256(probe),
        "environment": {
            key: environment[key]
            for key in (
                "MKL_THREADING_LAYER",
                "MKL_NUM_THREADS",
                "MKL_DYNAMIC",
                "OMP_NUM_THREADS",
                "OMP_DYNAMIC",
            )
        },
        "linkage": linkage,
        "host_environment": host_environment,
        "order": order,
        "raw_measurements": raw,
        "worker_samples": samples,
        "summary": [],
        "paired_contrasts": [],
    }
    artifact = JsonArtifact(
        out / "cubic-linear-pardiso-aftermath.json",
        scheduled_units=len(jobs),
        sort_keys=True,
    )

    for job_index, (repetition, case) in enumerate(jobs, start=1):
        label = f"repetition={repetition}:case={case}"
        artifact.set_active(label)
        with artifact.capture_failures(lambda: manifest):
            command = command_for(args, probe, mesh, case)
            print(
                f"[{job_index}/{len(jobs)}] r={repetition} case={case}",
                flush=True,
            )
            completed = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=environment,
                check=False,
            )
            log_path = out / f"r{repetition:02d}-{case}.log"
            log_path.write_text(completed.stdout, encoding="utf-8")
            if completed.returncode != 0:
                raise RuntimeError(f"{shlex.join(command)}\n{completed.stdout}")
            records = [
                record
                for line in completed.stdout.splitlines()
                if (record := parse_record(line)) is not None
            ]
            if len(records) != args.measured_iterations:
                raise RuntimeError(
                    f"Expected {args.measured_iterations} records, got {len(records)} "
                    f"for r={repetition}, case={case}"
                )
            for record in records:
                record["repetition"] = repetition
                raw.append(record)
            pending.setdefault(repetition, {})[case] = records
            if len(pending[repetition]) == len(args.cases):
                validate_block(pending[repetition])
                samples.extend(
                    median_record(pending[repetition][candidate], repetition)
                    for candidate in args.cases
                )
        artifact.checkpoint(manifest, completed_units=job_index)

    with artifact.capture_failures(lambda: manifest):
        summaries = summarize(samples)
        contrasts = paired_contrasts(samples, args.bootstrap_samples, args.seed)
        manifest.update(
            {
                "completed": True,
                "summary": summaries,
                "paired_contrasts": contrasts,
            }
        )
        artifact.complete(manifest)
    write_csv(out / "raw_measurements.csv", raw)
    write_csv(out / "worker_samples.csv", samples)
    write_csv(out / "summary.csv", summaries)
    write_csv(out / "paired_contrasts.csv", contrasts)
    print(json.dumps(summaries, indent=2))
    print(json.dumps(contrasts, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
