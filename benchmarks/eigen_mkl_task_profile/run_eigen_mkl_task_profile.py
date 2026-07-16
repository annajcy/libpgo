#!/usr/bin/env python3
"""Collect isolated VTune profiles for Eigen/oneMKL executor policies."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


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
    return parser.parse_args()


def require_positive(value: int, option: str) -> None:
    if value <= 0:
        raise ValueError(f"{option} must be positive.")


def resolve_vtune(explicit: Path | None) -> Path:
    if explicit is not None:
        candidate = explicit.expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"VTune executable does not exist: {candidate}")
        return candidate

    discovered = shutil.which("vtune")
    if discovered is None:
        raise FileNotFoundError(
            "VTune CLI was not found. Install Intel VTune Profiler or pass --vtune."
        )
    return Path(discovered).resolve()


def checked_output(command: list[str], environment: dict[str, str]) -> str:
    result = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env=environment,
    )
    if result.returncode != 0:
        raise RuntimeError(f"{' '.join(command)}\n{result.stdout}")
    return result.stdout


def report_output(command: list[str], environment: dict[str, str]) -> str:
    result = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env=environment,
    )
    if result.returncode == 0:
        return result.stdout
    if "Empty request output." in result.stdout:
        return ""
    raise RuntimeError(f"{' '.join(command)}\n{result.stdout}")


def verify_linkage(probe: Path, environment: dict[str, str]) -> str:
    dependencies = checked_output(["ldd", str(probe)], environment)
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

    probe = args.probe.expanduser().resolve()
    if not probe.is_file():
        raise FileNotFoundError(f"Probe executable does not exist: {probe}")
    vtune = resolve_vtune(args.vtune)
    output = args.out.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Output path already exists: {output}")

    environment = os.environ.copy()
    environment["MKL_THREADING_LAYER"] = "TBB"
    linkage = verify_linkage(probe, environment)

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
    manifest: dict[str, Any] = {
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
        "runs": [],
    }

    for entry in commands:
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
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        if result.returncode != 0:
            raise RuntimeError(f"VTune failed for {entry['policy']}; see {log_path}.")

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
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    print(f"Profiles written to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
