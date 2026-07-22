"""pypgo-sim-batch — batch orchestrator for the pypgo-sim CLI suite.

Reads a batch JSON, validates it, then runs the specified cases sequentially
via each entry module's ``main(argv)``.  A per-case PASS/FAIL table is printed
at the end.  Each case writes below ``<output-root>/<case_name>`` and a
``batch_summary.json`` is written to ``<output-root>``.  The output root
defaults to ``./output``.

Exit codes:
  0   all selected cases passed
  1   at least one case failed (but batch config itself was valid)
  2   invalid batch config (ConfigError) or argparse error
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from pathlib import Path
from typing import Any

from pypgo.tools.sim._config import ConfigError

# Map command string -> entry-module dotted path
_COMMANDS: dict[str, str] = {
    "tet-static":      "pypgo.tools.sim.tet_static",
    "tet-dynamic":     "pypgo.tools.sim.tet_dynamic",
    "cubic-static":    "pypgo.tools.sim.cubic_static",
    "cubic-dynamic":   "pypgo.tools.sim.cubic_dynamic",
    "shell-static":    "pypgo.tools.sim.shell_static",
    "shell-dynamic":   "pypgo.tools.sim.shell_dynamic",
}


def _load_batch(batch_path: Path) -> dict:
    """Read and return the raw batch JSON payload."""
    try:
        return json.loads(batch_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"cannot read batch config {batch_path}: {exc}") from exc


def _validate_batch(payload: dict, batch_dir: Path, job: str) -> list[tuple[str, dict]]:
    """Validate the batch payload and return the ordered list of (name, case_dict).

    ``case_dict`` has keys: command, config (absolute Path), args (list[str]).
    Raises ConfigError for any validation failure.
    """
    if not isinstance(payload, dict):
        raise ConfigError("batch config root must be a JSON object")

    raw_cases: dict = payload.get("cases", {})
    if not isinstance(raw_cases, dict):
        raise ConfigError("batch 'cases' must be a JSON object")

    jobs: dict = payload.get("jobs", {})
    if not isinstance(jobs, dict):
        raise ConfigError("batch 'jobs' must be a JSON object")

    # Validate every case definition
    for case_name, case in raw_cases.items():
        if not isinstance(case, dict):
            raise ConfigError(f"case {case_name!r} must be a JSON object")
        cmd = case.get("command")
        if cmd not in _COMMANDS:
            raise ConfigError(
                f"case {case_name!r}: unknown command {cmd!r}; "
                f"must be one of {sorted(_COMMANDS)}")
        cfg_raw = case.get("config")
        if not cfg_raw:
            raise ConfigError(f"case {case_name!r}: 'config' is required")
        cfg_path = Path(cfg_raw)
        if not cfg_path.is_absolute():
            cfg_path = batch_dir / cfg_path
        if not cfg_path.exists():
            raise ConfigError(
                f"case {case_name!r}: scene config not found: {cfg_path}")
        args = case.get("args", [])
        if not isinstance(args, list):
            raise ConfigError(f"case {case_name!r}: 'args' must be a list")

    # Validate all jobs (case members must be defined)
    for job_name, members in jobs.items():
        if members == "all":
            continue
        if not isinstance(members, list):
            raise ConfigError(
                f"job {job_name!r}: value must be a list or the string 'all'")
        for m in members:
            if m not in raw_cases:
                raise ConfigError(
                    f"job {job_name!r}: references undefined case {m!r}")

    # Resolve which cases to run
    if job == "all":
        # The implicit "all" job means every case; a defined "all" job wins.
        if "all" in jobs:
            members_spec = jobs["all"]
        else:
            members_spec = "all"
    else:
        if job not in jobs:
            raise ConfigError(
                f"unknown job {job!r}; defined jobs: {sorted(jobs)}")
        members_spec = jobs[job]

    if members_spec == "all":
        selected_names = list(raw_cases.keys())
    else:
        selected_names = list(members_spec)  # already validated above

    # Build resolved case list
    resolved = []
    for case_name in selected_names:
        case = raw_cases[case_name]
        cfg_path = Path(case["config"])
        if not cfg_path.is_absolute():
            cfg_path = batch_dir / cfg_path
        resolved.append((case_name, {
            "command": case["command"],
            "config": cfg_path,
            "args": list(case.get("args", [])),
        }))
    return resolved


def _print_list(payload: dict) -> None:
    """Print cases and jobs to stdout."""
    cases: dict = payload.get("cases", {})
    jobs: dict = payload.get("jobs", {})

    print("Cases:")
    for name, case in cases.items():
        print(f"  {name:40s}  command={case.get('command', '?')}")

    print("\nJobs:")
    for job_name, members in jobs.items():
        if members == "all":
            print(f"  {job_name}: <all cases>")
        else:
            print(f"  {job_name}: {', '.join(members)}")


def _run_case(
    case_name: str,
    case: dict[str, Any],
    output_root: Path,
) -> tuple[int, str]:
    """Run a single case and return (exit_code, command).

    Catches SystemExit and Exception so the batch continues on failure.
    """
    mod = importlib.import_module(_COMMANDS[case["command"]])
    argv: list[str] = ["--config", str(case["config"])] + case["args"]
    argv += ["--output-dir", str(output_root / case_name)]

    try:
        code = mod.main(argv)
        if code is None:
            code = 0
        return int(code), case["command"]
    except SystemExit as exc:
        code = exc.code if exc.code is not None else 0
        if isinstance(code, int):
            return code, case["command"]
        return 1, case["command"]
    except Exception as exc:  # noqa: BLE001
        print(f"  ERROR in case {case_name!r}: {exc}", file=sys.stderr)
        return 1, case["command"]


def main(argv=None) -> int:
    """Entry point for pypgo-sim-batch."""
    parser = argparse.ArgumentParser(
        prog="pypgo-sim-batch",
        description="Run a batch of pypgo-sim cases from a JSON config.",
    )
    parser.add_argument("--config", type=Path, required=True,
                        help="Path to the batch JSON file")
    parser.add_argument("--job", default="all",
                        help="Job name to run (default: 'all')")
    parser.add_argument("--list", action="store_true",
                        help="List cases and jobs then exit")
    parser.add_argument("--output-root", type=Path, default=Path("output"),
                        help="Root directory (default: ./output); each case writes below it")

    args = parser.parse_args(argv)

    # Load raw JSON
    try:
        payload = _load_batch(args.config)
    except ConfigError as exc:
        parser.error(str(exc))

    # --list: no validation of config/command paths needed
    if args.list:
        _print_list(payload)
        return 0

    # Validate and resolve cases
    batch_dir = args.config.resolve().parent
    try:
        cases = _validate_batch(payload, batch_dir, args.job)
    except ConfigError as exc:
        parser.error(str(exc))

    # Run cases sequentially
    results: dict[str, dict] = {}
    col_width = max((len(n) for n, _ in cases), default=10) + 2

    print(f"Running {len(cases)} case(s) (job={args.job!r})")
    for case_name, case in cases:
        print(f"  [{case_name}] {case['command']} ...", flush=True)
        exit_code, command = _run_case(case_name, case, args.output_root)
        passed = (exit_code == 0)
        results[case_name] = {
            "command": command,
            "exit_code": exit_code,
            "passed": passed,
        }

    # Print per-case table
    print()
    print(f"{'Case':{col_width}}  {'Command':20}  Result")
    print("-" * (col_width + 35))
    for name, r in results.items():
        status = "PASS" if r["passed"] else f"FAIL({r['exit_code']})"
        print(f"{name:{col_width}}  {r['command']:20}  {status}")

    num_passed = sum(1 for r in results.values() if r["passed"])
    num_failed = len(results) - num_passed
    print(f"\n{num_passed}/{len(results)} passed.")

    args.output_root.mkdir(parents=True, exist_ok=True)
    summary = {
        "cases": results,
        "num_passed": num_passed,
        "num_failed": num_failed,
    }
    summary_path = args.output_root / "batch_summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    print(f"Wrote {summary_path}")

    return 0 if num_failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
