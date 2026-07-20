#!/usr/bin/env python3
"""Calibrate fixed workload-specific warmup time for one benchmark subject."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


BENCHMARKS_ROOT = Path(__file__).resolve().parent
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
from benchmark_support.harness import (  # noqa: E402
    add_benchmark_harness_arguments,
    finish_benchmark_host,
    prepare_benchmark_host,
)
from benchmark_support.process import command_text  # noqa: E402
from benchmark_support.warmup import (  # noqa: E402
    DEFAULT_CALIBRATION_CANDIDATES,
    analyze_warmup_calibration,
    calibration_order,
    parse_warmup_candidates,
)


TIME_SCALE = {"ns": 1e-9, "us": 1e-6, "ms": 1e-3, "s": 1.0}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument(
        "--warmup-candidates",
        type=parse_warmup_candidates,
        default=DEFAULT_CALIBRATION_CANDIDATES,
    )
    parser.add_argument("--warmup-min-operations", type=int, default=1)
    parser.add_argument("--repetitions", type=int, default=12)
    parser.add_argument("--seed", type=int, default=20260720)
    parser.add_argument("--bootstrap-samples", type=int, default=5000)
    parser.add_argument("--median-tolerance", type=float, default=0.01)
    parser.add_argument("--ci-tolerance", type=float, default=0.02)
    parser.add_argument(
        "--result-format",
        choices=("google-json", "key-value", "json-marker"),
        required=True,
    )
    parser.add_argument(
        "--result-marker",
        help="Line prefix for key-value or JSON-marker output.",
    )
    parser.add_argument(
        "--metric",
        default="wall_seconds",
        help="Seconds-valued result field; google-json also accepts real_time.",
    )
    parser.add_argument("--env", action="append", default=[])
    parser.add_argument("--cwd", type=Path, default=Path.cwd())
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "command",
        nargs=argparse.REMAINDER,
        help=(
            "Command template after --. Supported fields: {warmup_seconds}, "
            "{warmup_min_operations}, {result_path}, and {repetition}."
        ),
    )
    add_benchmark_harness_arguments(parser)
    args = parser.parse_args()
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    return args


def validate_args(args: argparse.Namespace) -> None:
    if args.concurrency <= 0:
        raise ValueError("--concurrency must be positive")
    if args.warmup_min_operations < 0:
        raise ValueError("--warmup-min-operations must be nonnegative")
    if args.repetitions <= 0 or args.bootstrap_samples <= 0:
        raise ValueError("--repetitions and --bootstrap-samples must be positive")
    if not 0 < args.median_tolerance <= args.ci_tolerance < 1:
        raise ValueError("require 0 < --median-tolerance <= --ci-tolerance < 1")
    if not args.command:
        raise ValueError("a command template must follow --")
    if args.result_format != "google-json" and not args.result_marker:
        raise ValueError("--result-marker is required for marker result formats")
    for assignment in args.env:
        name, separator, _ = assignment.partition("=")
        if not separator or not name:
            raise ValueError(f"invalid --env assignment: {assignment}")


def render_command(
    template: list[str],
    *,
    warmup_seconds: float,
    warmup_min_operations: int,
    result_path: Path,
    repetition: int,
) -> list[str]:
    values = {
        "warmup_seconds": f"{warmup_seconds:g}",
        "warmup_min_operations": str(warmup_min_operations),
        "result_path": str(result_path),
        "repetition": str(repetition),
    }
    try:
        return [part.format_map(values) for part in template]
    except KeyError as error:
        raise ValueError(f"unknown command-template field: {error.args[0]}") from error


def _marker_lines(stdout: str, marker: str) -> list[str]:
    return [line[len(marker) :].strip() for line in stdout.splitlines() if line.startswith(marker)]


def parse_measurement(
    args: argparse.Namespace,
    *,
    stdout: str,
    result_path: Path,
) -> tuple[float, dict[str, Any]]:
    if args.result_format == "google-json":
        payload = json.loads(result_path.read_text())
        rows = [
            row
            for row in payload.get("benchmarks", [])
            if row.get("run_type") != "aggregate"
        ]
        if len(rows) != 1:
            raise RuntimeError(f"expected one Google Benchmark row, found {len(rows)}")
        row = rows[0]
        if args.metric == "real_time":
            scale = TIME_SCALE.get(row.get("time_unit"))
            if scale is None:
                raise RuntimeError(f"unknown Google Benchmark time unit: {row.get('time_unit')}")
            return float(row["real_time"]) * scale, row
        return float(row[args.metric]), row

    lines = _marker_lines(stdout, args.result_marker)
    if not lines:
        raise RuntimeError(f"command emitted no {args.result_marker} result line")
    if args.result_format == "json-marker":
        payloads = [json.loads(line) for line in lines]
        values = [float(payload[args.metric]) for payload in payloads]
        return statistics.median(values), {"payloads": payloads}

    parsed_lines = []
    values = []
    for line in lines:
        fields = dict(item.split("=", 1) for item in shlex.split(line))
        parsed_lines.append(fields)
        values.append(float(fields[args.metric]))
    return statistics.median(values), {"marker_records": parsed_lines}


def main() -> int:
    args = parse_args()
    validate_args(args)
    schedule, order = calibration_order(
        [args.subject],
        args.warmup_candidates,
        args.repetitions,
        seed=args.seed,
    )
    host = prepare_benchmark_host(
        args,
        workers=args.concurrency,
        dry_run=args.dry_run,
    )
    environment = os.environ.copy()
    for assignment in args.env:
        name, value = assignment.split("=", 1)
        environment[name] = value

    manifest: dict[str, Any] = {
        "runner": runner_manifest(Path(__file__)),
        "protocol": "workload_warmup_plateau_calibration",
        "subject": args.subject,
        "candidates_seconds": list(args.warmup_candidates),
        "warmup_min_operations": args.warmup_min_operations,
        "repetitions": args.repetitions,
        "seed": args.seed,
        "bootstrap_samples": args.bootstrap_samples,
        "median_tolerance": args.median_tolerance,
        "ci_tolerance": args.ci_tolerance,
        "selection_rule": "smallest T equivalent to both 2T and 4T",
        "order": order,
        "command_template": args.command,
        "result_format": args.result_format,
        "result_marker": args.result_marker,
        "metric": args.metric,
        "environment_overrides": dict(item.split("=", 1) for item in args.env),
        "host_environment": host,
    }
    if args.dry_run:
        for repetition, _, candidate in schedule:
            print(
                command_text(
                    render_command(
                        args.command,
                        warmup_seconds=candidate,
                        warmup_min_operations=args.warmup_min_operations,
                        result_path=Path("RESULT.json"),
                        repetition=repetition,
                    )
                )
            )
        return 0

    args.out.parent.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    payload = {"manifest": manifest, "records": records, "summary": []}
    artifact = JsonArtifact(args.out, scheduled_units=len(schedule), sort_keys=True)
    with tempfile.TemporaryDirectory(prefix="pgo-warmup-calibration-") as temp_dir:
        temporary = Path(temp_dir)
        for index, (repetition, subject, candidate) in enumerate(schedule, start=1):
            label = f"r={repetition}:warmup={candidate:g}s"
            artifact.set_active(label)
            result_path = temporary / f"result-{index:04d}.json"
            command = render_command(
                args.command,
                warmup_seconds=candidate,
                warmup_min_operations=args.warmup_min_operations,
                result_path=result_path,
                repetition=repetition,
            )
            print(f"[{index}/{len(schedule)}] {label}", flush=True)
            with artifact.capture_failures(lambda: payload):
                completed = subprocess.run(
                    command,
                    cwd=args.cwd,
                    env=environment,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
                if completed.returncode != 0:
                    raise RuntimeError(
                        f"calibration command exited {completed.returncode}: "
                        f"{command_text(command)}\n{completed.stdout}"
                    )
                measurement, raw_result = parse_measurement(
                    args,
                    stdout=completed.stdout,
                    result_path=result_path,
                )
                records.append(
                    {
                        "subject": subject,
                        "repetition": repetition,
                        "warmup_seconds": candidate,
                        "warmup_min_operations": args.warmup_min_operations,
                        "measurement_seconds": measurement,
                        "command": command,
                        "raw_result": raw_result,
                    }
                )
            artifact.checkpoint(payload, completed_units=len(records))

    payload["summary"] = analyze_warmup_calibration(
        records,
        args.warmup_candidates,
        seed=args.seed,
        bootstrap_samples=args.bootstrap_samples,
        median_tolerance=args.median_tolerance,
        ci_tolerance=args.ci_tolerance,
    )
    finish_benchmark_host(host)
    artifact.complete(payload)
    selected = payload["summary"][0]["selected_warmup_seconds"]
    print(f"selected_warmup_seconds={selected}")
    print(f"Wrote {args.out}")
    return 0 if selected is not None else 3


if __name__ == "__main__":
    raise SystemExit(main())
