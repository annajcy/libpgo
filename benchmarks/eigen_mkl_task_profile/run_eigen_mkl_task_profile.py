#!/usr/bin/env python3
"""Collect isolated VTune profiles for Eigen/oneMKL executor policies."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


BENCHMARKS_ROOT = Path(__file__).resolve().parents[1]
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.conditioning import (  # noqa: E402
    add_benchmark_harness_arguments,
    guard_host_condition,
    precondition_host,
)
from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
from benchmark_support.mkl import (  # noqa: E402
    mkl_tbb_environment,
    verify_mkl_tbb_probe_linkage,
)
from benchmark_support.process import (  # noqa: E402
    checked_output,
    report_output,
    resolve_file,
    resolve_vtune,
)
from benchmark_support.validation import require_positive  # noqa: E402


ALL_POLICIES = (
    "ExecutorDefault",
    "ExecutorLocal1",
    "ExecutorDefaultArena1",
    "ExecutorLocal1Arena1",
    "ExecutorMKLC",
    "ExecutorMKLCArena1",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--vtune", type=Path)
    parser.add_argument(
        "--policies", nargs="+", choices=ALL_POLICIES, default=ALL_POLICIES
    )
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument("--matrix-n", type=int, default=1024)
    parser.add_argument("--warmup-iterations", type=int, default=3)
    parser.add_argument("--profile-iterations", type=int, default=50)
    parser.add_argument(
        "--sudo",
        action="store_true",
        help="Run the VTune collector through sudo when ptrace_scope blocks collection.",
    )
    parser.add_argument("--dry-run", action="store_true")
    add_benchmark_harness_arguments(parser)
    return parser.parse_args()


def probe_command(probe: Path, args: argparse.Namespace, policy: str) -> list[str]:
    return [
        str(probe),
        f"--policy={policy}",
        f"--concurrency={args.concurrency}",
        f"--matrix-n={args.matrix_n}",
        f"--warmup-iterations={args.warmup_iterations}",
        f"--profile-iterations={args.profile_iterations}",
    ]


def main() -> int:
    args = parse_args()
    require_positive(args.concurrency, "--concurrency")
    require_positive(args.matrix_n, "--matrix-n")
    require_positive(args.profile_iterations, "--profile-iterations")
    if args.warmup_iterations < 0:
        raise ValueError("--warmup-iterations must be nonnegative.")

    probe = resolve_file(args.probe, "Probe executable")
    vtune = resolve_vtune(args.vtune)
    output = args.out.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Output path already exists: {output}")

    environment = mkl_tbb_environment()
    linkage = verify_mkl_tbb_probe_linkage(probe, environment)

    commands: list[dict[str, Any]] = []
    for policy in args.policies:
        result_directory = output / f"vtune-{policy.lower()}"
        command = [
            *(["sudo", "env", "MKL_THREADING_LAYER=TBB"] if args.sudo else []),
            str(vtune),
            "-collect",
            "threading",
            "-result-dir",
            str(result_directory),
            "--",
            *probe_command(probe, args, policy),
        ]
        commands.append(
            {
                "policy": policy,
                "command": command,
                "result_directory": str(result_directory),
            }
        )

    if args.dry_run:
        for entry in commands:
            print(shlex.join(entry["command"]))
        return 0

    output.mkdir(parents=True)
    host_preconditioning = precondition_host(args, workers=args.concurrency)
    manifest: dict[str, Any] = {
        "runner": runner_manifest(Path(__file__)),
        "schema_version": 2,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "probe": str(probe),
        "vtune": str(vtune),
        "configuration": {
            "policies": args.policies,
            "concurrency": args.concurrency,
            "matrix_n": args.matrix_n,
            "warmup_iterations": args.warmup_iterations,
            "profile_iterations": args.profile_iterations,
            "sudo": args.sudo,
        },
        "environment": {"MKL_THREADING_LAYER": "TBB"},
        "linkage": linkage,
        "host_preconditioning": host_preconditioning,
        "runs": [],
    }
    artifact = JsonArtifact(output / "manifest.json", scheduled_units=len(commands))

    for index, entry in enumerate(commands, start=1):
        artifact.set_active(f"vtune:{entry['policy']}")
        with artifact.capture_failures(lambda: manifest):
            guard_host_condition(
                args,
                host_preconditioning,
                label=f"vtune:{entry['policy']}",
            )
            result_directory = Path(entry["result_directory"])
            print(f"Running {entry['policy']}...", flush=True)
            result = subprocess.run(
                entry["command"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
                env=environment,
            )
            if args.sudo and result_directory.exists():
                checked_output(
                    [
                        "sudo",
                        "chown",
                        "-R",
                        f"{os.getuid()}:{os.getgid()}",
                        str(result_directory),
                    ],
                    environment,
                )
            log_path = output / f"{entry['policy'].lower()}.log"
            log_path.write_text(result.stdout)
            run = {
                **entry,
                "returncode": result.returncode,
                "log": str(log_path),
            }
            manifest["runs"].append(run)
            if result.returncode != 0:
                raise RuntimeError(
                    f"VTune failed for {entry['policy']}; see {log_path}."
                )

            reports = {
            "summary": [
                str(vtune),
                "-quiet",
                "-report",
                "summary",
                "-result-dir",
                str(result_directory),
                "-report-knob",
                "show-issues=false",
            ],
            "hotspots.csv": [
                str(vtune),
                "-quiet",
                "-report",
                "hotspots",
                "-result-dir",
                str(result_directory),
                "-format=csv",
                "-csv-delimiter=comma",
            ],
            "tasks.csv": [
                str(vtune),
                "-quiet",
                "-report",
                "hotspots",
                "-result-dir",
                str(result_directory),
                "-group-by",
                "task",
                "-format=csv",
                "-csv-delimiter=comma",
            ],
        }
            run["reports"] = {}
            for suffix, report_command in reports.items():
                report_path = output / f"{entry['policy'].lower()}.{suffix}"
                report_path.write_text(report_output(report_command, environment))
                run["reports"][suffix] = str(report_path)
        artifact.checkpoint(manifest, completed_units=index)

    artifact.complete(manifest)
    print(f"Profiles written to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
