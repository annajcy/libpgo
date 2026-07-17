#!/usr/bin/env python3
"""Benchmark public Python Newton solves with semantic phase threading policies."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import random
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from workloads import DEFAULT_MESHES, WORKLOADS, build_workload


SCRIPT = Path(__file__).resolve()
ROOT = SCRIPT.parents[2]
RESULT_MARKER = "PYPGO_SOLVER_PHASE_THREADING_RESULT="
REFERENCE_POLICY = "phase_aware"
POLICIES = (
    "uniform_single",
    "uniform_multi",
    "phase_aware",
    "phase_reversed",
    "phase_single_single",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--workloads", nargs="+", choices=WORKLOADS, default=WORKLOADS)
    parser.add_argument("--policies", nargs="+", choices=POLICIES, default=POLICIES)
    parser.add_argument(
        "--tet-mesh", type=Path, default=ROOT / DEFAULT_MESHES["tet_linear"]
    )
    parser.add_argument(
        "--cubic-linear-mesh",
        type=Path,
        default=ROOT / DEFAULT_MESHES["cubic_linear"],
    )
    parser.add_argument(
        "--hermite-mesh",
        type=Path,
        default=ROOT / DEFAULT_MESHES["cubic_tricubic_hermite"],
    )
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument("--reserved-slots", type=int, default=1)
    parser.add_argument("--repetitions", type=int, default=7)
    parser.add_argument("--warmup-solves", type=int, default=1)
    parser.add_argument("--timed-solves", type=int, default=1)
    parser.add_argument("--seed", type=int, default=20260716)
    parser.add_argument("--displacement-scale", type=float, default=1e-4)
    parser.add_argument("--fixed-slab-fraction", type=float, default=0.01)
    parser.add_argument(
        "--elastic-model",
        choices=("stable_neo", "stvk", "linear"),
        default="stable_neo",
    )
    parser.add_argument("--plastic-dofs", type=int, choices=(0, 3, 6), default=6)
    parser.add_argument("--signature-relative-tolerance", type=float, default=1e-9)
    parser.add_argument("--signature-absolute-tolerance", type=float, default=1e-10)
    parser.add_argument("--bootstrap-samples", type=int, default=10000)
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")

    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--workload", choices=WORKLOADS, help=argparse.SUPPRESS)
    parser.add_argument("--policy", choices=POLICIES, help=argparse.SUPPRESS)
    parser.add_argument("--mesh", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--repetition", type=int, default=0, help=argparse.SUPPRESS)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if len(set(args.workloads)) != len(args.workloads):
        raise SystemExit("--workloads must not contain duplicates")
    if len(set(args.policies)) != len(args.policies):
        raise SystemExit("--policies must not contain duplicates")
    if args.concurrency <= 0:
        raise SystemExit("--concurrency must be positive")
    if not 0 <= args.reserved_slots <= args.concurrency:
        raise SystemExit("--reserved-slots must be between zero and concurrency")
    if args.repetitions <= 0:
        raise SystemExit("--repetitions must be positive")
    if args.warmup_solves < 0:
        raise SystemExit("--warmup-solves must be non-negative")
    if args.timed_solves <= 0:
        raise SystemExit("--timed-solves must be positive")
    if not math.isfinite(args.displacement_scale) or args.displacement_scale <= 0.0:
        raise SystemExit("--displacement-scale must be positive")
    if (
        not math.isfinite(args.fixed_slab_fraction)
        or not 0.0 < args.fixed_slab_fraction < 1.0
    ):
        raise SystemExit("--fixed-slab-fraction must be finite and in (0, 1)")
    if (
        not math.isfinite(args.signature_relative_tolerance)
        or args.signature_relative_tolerance < 0.0
    ):
        raise SystemExit("--signature-relative-tolerance must be non-negative")
    if (
        not math.isfinite(args.signature_absolute_tolerance)
        or args.signature_absolute_tolerance < 0.0
    ):
        raise SystemExit("--signature-absolute-tolerance must be non-negative")
    if args.bootstrap_samples < 0:
        raise SystemExit("--bootstrap-samples must be non-negative")
    if args.case_limit < 0:
        raise SystemExit("--case-limit must be non-negative")

    if args.worker:
        if args.workload is None or args.policy is None or args.mesh is None:
            raise SystemExit("worker mode requires --workload, --policy, and --mesh")
        if not args.mesh.is_file():
            raise SystemExit(f"mesh does not exist: {args.mesh}")
    else:
        if args.out is None and not args.dry_run:
            raise SystemExit("controller mode requires --out")
        selected_paths = mesh_paths(args)
        for workload in args.workloads:
            if not selected_paths[workload].is_file():
                raise SystemExit(
                    f"mesh for {workload} does not exist: {selected_paths[workload]}"
                )


def mesh_paths(args: argparse.Namespace) -> dict[str, Path]:
    return {
        "tet_linear": args.tet_mesh,
        "cubic_linear": args.cubic_linear_mesh,
        "cubic_tricubic_hermite": args.hermite_mesh,
    }


def _executor_parameters(policy: str, concurrency: int) -> dict[str, Any]:
    if policy == "uniform_single":
        return {"kind": "uniform", "outer_budget": 1}
    if policy == "uniform_multi":
        return {"kind": "uniform", "outer_budget": concurrency}
    if policy == "phase_aware":
        return {"kind": "phase", "evaluation_budget": 1, "linear_budget": concurrency}
    if policy == "phase_reversed":
        return {"kind": "phase", "evaluation_budget": concurrency, "linear_budget": 1}
    if policy == "phase_single_single":
        return {"kind": "phase", "evaluation_budget": 1, "linear_budget": 1}
    raise ValueError(f"unknown policy: {policy}")


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _iteration_value(iteration: dict[str, Any] | None, name: str) -> Any:
    return None if iteration is None else iteration.get(name)


def _diagnostic_record(result: Any) -> dict[str, Any]:
    diagnostics = result.diagnostics
    iterations = diagnostics.newton_iterations or []
    last_iteration = iterations[-1] if iterations else None
    record = {
        "threading_evaluation_phase_calls": diagnostics.threading_evaluation_phase_calls,
        "threading_linear_solver_phase_calls": diagnostics.threading_linear_solver_phase_calls,
        "threading_evaluation_phase_seconds": diagnostics.threading_evaluation_phase_seconds,
        "threading_linear_solver_phase_seconds": diagnostics.threading_linear_solver_phase_seconds,
        "newton_total_factorize_seconds": diagnostics.newton_total_factorize_seconds,
        "newton_total_solve_seconds": diagnostics.newton_total_solve_seconds,
        "linear_solver_symbolic_rebuild_count": diagnostics.linear_solver_symbolic_rebuild_count,
        "linear_solver_symbolic_reuse_count": diagnostics.linear_solver_symbolic_reuse_count,
        "last_active_system_rows": diagnostics.last_active_system_rows,
        "last_active_system_cols": diagnostics.last_active_system_cols,
        "last_active_system_nnz": diagnostics.last_active_system_nnz,
        "last_line_search_iterations": diagnostics.last_line_search_iterations,
        "total_line_search_iterations": diagnostics.total_line_search_iterations,
        "accepted_alpha": _iteration_value(last_iteration, "accepted_alpha"),
        "line_search_status": _iteration_value(last_iteration, "line_search_status"),
        "func_grad_hessian_seconds": _iteration_value(
            last_iteration, "func_grad_hessian_seconds"
        ),
        "symbolic_analyze_seconds": _iteration_value(
            last_iteration, "symbolic_analyze_seconds"
        ),
        "factorize_seconds": _iteration_value(last_iteration, "factorize_seconds"),
        "solve_seconds": _iteration_value(last_iteration, "solve_seconds"),
        "line_search_seconds": _iteration_value(last_iteration, "line_search_seconds"),
        "iteration_wall_seconds": _iteration_value(
            last_iteration, "iteration_wall_seconds"
        ),
    }
    required_phase_fields = (
        "threading_evaluation_phase_calls",
        "threading_linear_solver_phase_calls",
        "threading_evaluation_phase_seconds",
        "threading_linear_solver_phase_seconds",
    )
    missing = [field for field in required_phase_fields if record[field] is None]
    if missing:
        raise RuntimeError(
            "solver build does not expose required phase diagnostics: "
            + ", ".join(missing)
        )
    non_finite = [
        field
        for field, value in record.items()
        if isinstance(value, float) and not math.isfinite(value)
    ]
    if non_finite:
        raise RuntimeError(
            "solver produced non-finite diagnostics: " + ", ".join(non_finite)
        )
    return record


def _result_signature(result: Any, fixed_dofs: Any) -> dict[str, Any]:
    import numpy as np

    x = np.asarray(result.x, dtype=np.float64)
    if not np.all(np.isfinite(x)):
        raise RuntimeError("solver produced non-finite result DOFs")
    if result.final_objective is not None and not math.isfinite(result.final_objective):
        raise RuntimeError("solver produced a non-finite final objective")
    diagnostics = _diagnostic_record(result)
    fixed_error = np.abs(x[fixed_dofs])
    canonical_x = np.asarray(x, dtype="<f8").tobytes()
    return {
        "status": int(result.status),
        "status_name": result.status.name,
        "converged": bool(result.converged),
        "iterations": int(result.iterations),
        "raw_status_code": int(result.raw_status_code),
        "final_objective": result.final_objective,
        "x_size": int(x.size),
        "x_sum": float(np.sum(x, dtype=np.float64)),
        "x_absolute_sum": float(np.sum(np.abs(x), dtype=np.float64)),
        "x_squared_norm": float(np.sum(x * x, dtype=np.float64)),
        "x_max_abs": float(np.max(np.abs(x), initial=0.0)),
        "x_sha256": hashlib.sha256(canonical_x).hexdigest(),
        "x_values": x.tolist(),
        "fixed_dof_count": int(fixed_dofs.size),
        "fixed_dof_max_abs": float(np.max(fixed_error, initial=0.0)),
        "last_line_search_iterations": diagnostics["last_line_search_iterations"],
        "total_line_search_iterations": diagnostics["total_line_search_iterations"],
        "accepted_alpha": diagnostics["accepted_alpha"],
        "line_search_status": diagnostics["line_search_status"],
        "linear_solver_symbolic_rebuild_count": diagnostics[
            "linear_solver_symbolic_rebuild_count"
        ],
        "linear_solver_symbolic_reuse_count": diagnostics[
            "linear_solver_symbolic_reuse_count"
        ],
        "last_active_system_rows": diagnostics["last_active_system_rows"],
        "last_active_system_cols": diagnostics["last_active_system_cols"],
        "last_active_system_nnz": diagnostics["last_active_system_nnz"],
        "threading_evaluation_phase_calls": diagnostics[
            "threading_evaluation_phase_calls"
        ],
        "threading_linear_solver_phase_calls": diagnostics[
            "threading_linear_solver_phase_calls"
        ],
    }


def worker_main(args: argparse.Namespace) -> int:
    sys.path.insert(0, str(ROOT))
    import gc

    import pypgo
    import pypgo.solver as ps
    from pypgo.parallel import ArenaThreadingExecutor, GlobalTbbControl

    global_control = GlobalTbbControl(args.concurrency)
    workload = build_workload(
        name=args.workload,
        mesh_path=args.mesh,
        seed=args.seed,
        displacement_scale=args.displacement_scale,
        fixed_slab_fraction=args.fixed_slab_fraction,
        elastic_model=args.elastic_model,
        plastic_dofs=args.plastic_dofs,
    )

    parameters = _executor_parameters(args.policy, args.concurrency)
    outer_executor = None
    threading = None
    if parameters["kind"] == "uniform":
        outer_executor = ArenaThreadingExecutor(
            args.concurrency,
            reserved_slots=args.reserved_slots,
            mkl_local_thread_budget=parameters["outer_budget"],
        )
    else:
        evaluation_executor = ArenaThreadingExecutor(
            args.concurrency,
            reserved_slots=args.reserved_slots,
            mkl_local_thread_budget=parameters["evaluation_budget"],
        )
        linear_executor = ArenaThreadingExecutor(
            args.concurrency,
            reserved_slots=args.reserved_slots,
            mkl_local_thread_budget=parameters["linear_budget"],
        )
        threading = ps.NewtonThreadingPolicy(
            evaluation=evaluation_executor,
            linear_solver=linear_executor,
        )

    optimizer = ps.NewtonOptimizer(
        max_iterations=1,
        gradient_tolerance=0.0,
        line_search=ps.Backtrack(),
        damping=ps.NoDamping(),
        sparse_solver=ps.MKLPardiso(),
        threading=threading,
    )

    def solve_once():
        if outer_executor is None:
            return optimizer.solve(workload.problem, workload.x0)
        return outer_executor.execute(
            lambda: optimizer.solve(workload.problem, workload.x0)
        )

    for _ in range(args.warmup_solves):
        solve_once()
    gc.collect()

    solve_wall_seconds: list[float] = []
    timed_diagnostics: list[dict[str, Any]] = []
    timed_signatures: list[dict[str, Any]] = []
    last_result = None
    for solve_index in range(args.timed_solves):
        started = time.perf_counter()
        last_result = solve_once()
        solve_wall_seconds.append(time.perf_counter() - started)
        timed_diagnostics.append(_diagnostic_record(last_result))
        signature = _result_signature(last_result, workload.fixed_dofs)
        if signature["iterations"] != 1:
            raise RuntimeError(
                f"timed solve {solve_index} expected exactly one Newton iteration, "
                f"got {signature['iterations']}"
            )
        if signature["fixed_dof_max_abs"] != 0.0:
            raise RuntimeError(f"fixed DOFs changed during timed solve {solve_index}")
        timed_signatures.append(signature)

    if last_result is None:
        raise RuntimeError("timed solve loop produced no result")
    validate_signatures(
        [
            {"policy": f"{args.policy}[{index}]", "signature": signature}
            for index, signature in enumerate(timed_signatures)
        ],
        args.signature_relative_tolerance,
        args.signature_absolute_tolerance,
    )
    signature = timed_signatures[-1]

    phase_totals = {
        "threading_evaluation_phase_calls": sum(
            int(record["threading_evaluation_phase_calls"] or 0)
            for record in timed_diagnostics
        ),
        "threading_linear_solver_phase_calls": sum(
            int(record["threading_linear_solver_phase_calls"] or 0)
            for record in timed_diagnostics
        ),
        "threading_evaluation_phase_seconds": sum(
            float(record["threading_evaluation_phase_seconds"] or 0.0)
            for record in timed_diagnostics
        ),
        "threading_linear_solver_phase_seconds": sum(
            float(record["threading_linear_solver_phase_seconds"] or 0.0)
            for record in timed_diagnostics
        ),
        "newton_total_factorize_seconds": sum(
            float(record["newton_total_factorize_seconds"] or 0.0)
            for record in timed_diagnostics
        ),
        "newton_total_solve_seconds": sum(
            float(record["newton_total_solve_seconds"] or 0.0)
            for record in timed_diagnostics
        ),
    }
    result = {
        "workload": args.workload,
        "policy": args.policy,
        "policy_parameters": parameters,
        "repetition": args.repetition,
        "concurrency": args.concurrency,
        "reserved_slots": args.reserved_slots,
        "warmup_solves": args.warmup_solves,
        "timed_solves": args.timed_solves,
        "solve_wall_seconds": solve_wall_seconds,
        "total_wall_seconds": float(sum(solve_wall_seconds)),
        "seconds_per_solve": float(sum(solve_wall_seconds) / len(solve_wall_seconds)),
        "phase_totals": phase_totals,
        "timed_diagnostics": timed_diagnostics,
        "timed_signatures": timed_signatures,
        "signature": signature,
        "workload_metadata": workload.metadata,
        "runtime": {
            "python": sys.version,
            "platform": platform.platform(),
            "pypgo": str(Path(pypgo.__file__).resolve()),
            "pypgo_core": str(Path(pypgo._core.__file__).resolve()),
            "pypgo_core_sha256": _file_sha256(Path(pypgo._core.__file__).resolve()),
            "cpu_count": os.cpu_count(),
            "cpu_affinity": (
                sorted(os.sched_getaffinity(0))
                if hasattr(os, "sched_getaffinity")
                else None
            ),
            "environment": {
                name: os.environ.get(name)
                for name in (
                    "MKL_NUM_THREADS",
                    "MKL_DYNAMIC",
                    "MKL_THREADING_LAYER",
                    "OMP_NUM_THREADS",
                    "OMP_DYNAMIC",
                    "OMP_PROC_BIND",
                    "OMP_PLACES",
                    "KMP_AFFINITY",
                )
            },
        },
    }
    global_control.close()
    print(RESULT_MARKER + json.dumps(result, sort_keys=True, allow_nan=False))
    return 0


def worker_environment(concurrency: int) -> tuple[dict[str, str], dict[str, str]]:
    overrides = {
        "MKL_NUM_THREADS": str(concurrency),
        "OMP_NUM_THREADS": str(concurrency),
        "MKL_DYNAMIC": "FALSE",
        "OMP_DYNAMIC": "FALSE",
    }
    env = os.environ.copy()
    env.update(overrides)
    env["PYTHONPATH"] = os.pathsep.join([str(ROOT), env.get("PYTHONPATH", "")]).rstrip(
        os.pathsep
    )
    return env, overrides


def worker_command(
    args: argparse.Namespace,
    workload: str,
    policy: str,
    repetition: int,
    mesh: Path,
) -> list[str]:
    command = [
        sys.executable,
        str(SCRIPT),
        "--worker",
        "--workload",
        workload,
        "--policy",
        policy,
        "--repetition",
        str(repetition),
        "--mesh",
        str(mesh.resolve()),
        "--concurrency",
        str(args.concurrency),
        "--reserved-slots",
        str(args.reserved_slots),
        "--warmup-solves",
        str(args.warmup_solves),
        "--timed-solves",
        str(args.timed_solves),
        "--seed",
        str(args.seed),
        "--displacement-scale",
        str(args.displacement_scale),
        "--fixed-slab-fraction",
        str(args.fixed_slab_fraction),
        "--elastic-model",
        args.elastic_model,
        "--plastic-dofs",
        str(args.plastic_dofs),
        "--signature-relative-tolerance",
        str(args.signature_relative_tolerance),
        "--signature-absolute-tolerance",
        str(args.signature_absolute_tolerance),
    ]
    return command


def parse_worker_result(stdout: str, command: list[str]) -> dict[str, Any]:
    payloads = [
        line[len(RESULT_MARKER) :]
        for line in stdout.splitlines()
        if line.startswith(RESULT_MARKER)
    ]
    if len(payloads) != 1:
        raise RuntimeError(
            f"worker emitted {len(payloads)} result payloads: {' '.join(command)}\n{stdout}"
        )
    return json.loads(payloads[0])


def run_worker(command: list[str], concurrency: int) -> dict[str, Any]:
    env, _ = worker_environment(concurrency)
    completed = subprocess.run(
        command,
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"worker failed with exit code {completed.returncode}: {' '.join(command)}\n"
            f"{completed.stdout}"
        )
    return parse_worker_result(completed.stdout, command)


def _close_numeric(
    left: float | None,
    right: float | None,
    relative_tolerance: float,
    absolute_tolerance: float,
    comparison_scale: float | None = None,
) -> bool:
    if left is None or right is None:
        return left is right
    scale = max(abs(left), abs(right), comparison_scale or 0.0)
    return abs(left - right) <= absolute_tolerance + relative_tolerance * scale


def validate_signatures(
    records: list[dict[str, Any]], relative_tolerance: float, absolute_tolerance: float
) -> None:
    if len(records) < 2:
        return
    reference = records[0]
    reference_signature = reference["signature"]
    exact_fields = (
        "status",
        "status_name",
        "converged",
        "iterations",
        "raw_status_code",
        "x_size",
        "fixed_dof_count",
        "last_line_search_iterations",
        "total_line_search_iterations",
        "line_search_status",
        "linear_solver_symbolic_rebuild_count",
        "linear_solver_symbolic_reuse_count",
        "last_active_system_rows",
        "last_active_system_cols",
        "last_active_system_nnz",
        "threading_evaluation_phase_calls",
        "threading_linear_solver_phase_calls",
    )
    numeric_fields = (
        ("final_objective", None),
        ("x_sum", "x_absolute_sum"),
        ("x_absolute_sum", None),
        ("x_squared_norm", None),
        ("x_max_abs", None),
        ("fixed_dof_max_abs", None),
        ("accepted_alpha", None),
    )
    for record in records[1:]:
        signature = record["signature"]
        for field in exact_fields:
            if signature[field] != reference_signature[field]:
                raise RuntimeError(
                    f"signature field {field} differs between "
                    f"{reference['policy']} and {record['policy']}: "
                    f"{reference_signature[field]} vs {signature[field]}"
                )
        for field, scale_field in numeric_fields:
            comparison_scale = None
            if scale_field is not None:
                comparison_scale = max(
                    abs(float(signature[scale_field])),
                    abs(float(reference_signature[scale_field])),
                )
            if not _close_numeric(
                signature[field],
                reference_signature[field],
                relative_tolerance,
                absolute_tolerance,
                comparison_scale,
            ):
                raise RuntimeError(
                    f"signature field {field} differs between "
                    f"{reference['policy']} and {record['policy']}: "
                    f"{reference_signature[field]} vs {signature[field]}"
                )
        left_values = reference_signature["x_values"]
        right_values = signature["x_values"]
        if len(left_values) != len(right_values):
            raise RuntimeError(
                f"x_values length differs between {reference['policy']} "
                f"and {record['policy']}"
            )
        for dof, (left, right) in enumerate(zip(left_values, right_values)):
            if not _close_numeric(
                float(left),
                float(right),
                relative_tolerance,
                absolute_tolerance,
            ):
                raise RuntimeError(
                    f"result x differs at DOF {dof} between {reference['policy']} "
                    f"and {record['policy']}: {left} vs {right}"
                )


def median_absolute_deviation(values: list[float]) -> float:
    center = statistics.median(values)
    return statistics.median(abs(value - center) for value in values)


def _percentile(sorted_values: list[float], probability: float) -> float:
    if not sorted_values:
        raise ValueError("cannot take a percentile of no values")
    position = probability * (len(sorted_values) - 1)
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def _bootstrap_median_ci(
    values: list[float], samples: int, seed_label: str
) -> tuple[float | None, float | None]:
    if not values or samples <= 0:
        return None, None
    seed = int.from_bytes(hashlib.sha256(seed_label.encode()).digest()[:8], "little")
    rng = random.Random(seed)
    estimates = []
    for _ in range(samples):
        draw = [values[rng.randrange(len(values))] for _ in values]
        estimates.append(statistics.median(draw))
    estimates.sort()
    return _percentile(estimates, 0.025), _percentile(estimates, 0.975)


def summarize(
    records: list[dict[str, Any]], bootstrap_samples: int
) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        grouped.setdefault(record["workload"], []).append(record)

    summaries: list[dict[str, Any]] = []
    for workload, group in sorted(grouped.items()):
        by_policy: dict[str, list[dict[str, Any]]] = {}
        for record in group:
            by_policy.setdefault(record["policy"], []).append(record)
        reference_by_repetition = {
            int(record["repetition"]): record
            for record in by_policy.get(REFERENCE_POLICY, [])
        }
        for policy, policy_records in sorted(by_policy.items()):
            values = [float(record["seconds_per_solve"]) for record in policy_records]
            ratios = []
            for record in policy_records:
                reference = reference_by_repetition.get(int(record["repetition"]))
                if reference is not None:
                    ratios.append(
                        float(record["seconds_per_solve"])
                        / float(reference["seconds_per_solve"])
                    )
            ratio_ci_low, ratio_ci_high = _bootstrap_median_ci(
                ratios, bootstrap_samples, f"{workload}:{policy}:{REFERENCE_POLICY}"
            )
            summaries.append(
                {
                    "workload": workload,
                    "policy": policy,
                    "samples": len(values),
                    "median_seconds_per_solve": statistics.median(values),
                    "mad_seconds_per_solve": median_absolute_deviation(values),
                    "min_seconds_per_solve": min(values),
                    "max_seconds_per_solve": max(values),
                    "paired_ratio_over_phase_aware_median": (
                        statistics.median(ratios) if ratios else None
                    ),
                    "paired_ratio_over_phase_aware_ci95_low": ratio_ci_low,
                    "paired_ratio_over_phase_aware_ci95_high": ratio_ci_high,
                }
            )
    return summaries


def _raw_csv_rows(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for record in records:
        totals = record["phase_totals"]
        metadata = record["workload_metadata"]
        rows.append(
            {
                "workload": record["workload"],
                "policy": record["policy"],
                "repetition": record["repetition"],
                "seconds_per_solve": record["seconds_per_solve"],
                "total_wall_seconds": record["total_wall_seconds"],
                "timed_solves": record["timed_solves"],
                "evaluation_phase_calls": totals["threading_evaluation_phase_calls"],
                "linear_solver_phase_calls": totals[
                    "threading_linear_solver_phase_calls"
                ],
                "evaluation_phase_seconds": totals[
                    "threading_evaluation_phase_seconds"
                ],
                "linear_solver_phase_seconds": totals[
                    "threading_linear_solver_phase_seconds"
                ],
                "factorize_seconds": totals["newton_total_factorize_seconds"],
                "solve_seconds": totals["newton_total_solve_seconds"],
                "num_dofs": metadata["num_dofs"],
                "num_elements": metadata["mesh"]["num_elements"],
                "active_system_nnz": record["signature"]["last_active_system_nnz"],
                "final_objective": record["signature"]["final_objective"],
                "accepted_alpha": record["signature"]["accepted_alpha"],
                "status": record["signature"]["status_name"],
                "iterations": record["signature"]["iterations"],
                "validation_status": record.get("validation_status", "pending"),
            }
        )
    return rows


def _phase_csv_rows(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for record in records:
        for solve_index, diagnostics in enumerate(record["timed_diagnostics"]):
            rows.append(
                {
                    "workload": record["workload"],
                    "policy": record["policy"],
                    "repetition": record["repetition"],
                    "solve_index": solve_index,
                    "solve_wall_seconds": record["solve_wall_seconds"][solve_index],
                    **diagnostics,
                }
            )
    return rows


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    if not rows:
        temporary.write_text("")
        temporary.replace(path)
        return
    with temporary.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def write_checkpoint(
    output: Path,
    manifest: dict[str, Any],
    records: list[dict[str, Any]],
    bootstrap_samples: int,
    run_status: dict[str, Any],
) -> None:
    summary = summarize(records, bootstrap_samples)
    payload = {
        "manifest": manifest,
        "run_status": run_status,
        "measurements": records,
        "summary": summary,
    }
    temporary = output / "python-solver-phase-threading.json.tmp"
    final = output / "python-solver-phase-threading.json"
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    )
    temporary.replace(final)
    _write_csv(output / "runs.csv", _raw_csv_rows(records))
    _write_csv(output / "phase_breakdown.csv", _phase_csv_rows(records))
    _write_csv(output / "summary.csv", summary)


def _git_revision() -> dict[str, Any]:
    def run(*command: str) -> str | None:
        completed = subprocess.run(
            command,
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return completed.stdout.strip() if completed.returncode == 0 else None

    return {
        "commit": run("git", "rev-parse", "HEAD"),
        "branch": run("git", "branch", "--show-current"),
        "dirty": bool(run("git", "status", "--short")),
    }


def controller_main(args: argparse.Namespace) -> int:
    rng = random.Random(args.seed)
    paths = mesh_paths(args)
    blocks = [
        (repetition, workload)
        for repetition in range(args.repetitions)
        for workload in args.workloads
    ]
    rng.shuffle(blocks)
    scheduled: list[tuple[list[str], tuple[int, str]]] = []
    for repetition, workload in blocks:
        policies = list(args.policies)
        rng.shuffle(policies)
        for policy in policies:
            scheduled.append(
                (
                    worker_command(args, workload, policy, repetition, paths[workload]),
                    (repetition, workload),
                )
            )
    requested_case_count = len(scheduled)
    if args.case_limit:
        scheduled = scheduled[: args.case_limit]

    if args.dry_run:
        for command, _ in scheduled:
            print(" ".join(command))
        return 0

    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    _, environment_overrides = worker_environment(args.concurrency)
    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "script": str(SCRIPT),
        "script_sha256": _file_sha256(SCRIPT),
        "workloads_script_sha256": _file_sha256(SCRIPT.with_name("workloads.py")),
        "git": _git_revision(),
        "pure_public_python_workload": True,
        "fresh_process_per_sample": True,
        "serial_execution": True,
        "reference_policy": REFERENCE_POLICY,
        "workloads": list(args.workloads),
        "policies": list(args.policies),
        "mesh_paths": {name: str(path.resolve()) for name, path in paths.items()},
        "concurrency": args.concurrency,
        "reserved_slots": args.reserved_slots,
        "repetitions": args.repetitions,
        "warmup_solves": args.warmup_solves,
        "timed_solves": args.timed_solves,
        "seed": args.seed,
        "displacement_scale": args.displacement_scale,
        "fixed_slab_fraction": args.fixed_slab_fraction,
        "elastic_model": args.elastic_model,
        "plastic_dofs": args.plastic_dofs,
        "bootstrap_samples": args.bootstrap_samples,
        "signature_relative_tolerance": args.signature_relative_tolerance,
        "signature_absolute_tolerance": args.signature_absolute_tolerance,
        "case_limit": args.case_limit,
        "requested_case_count": requested_case_count,
        "scheduled_case_count": len(scheduled),
        "schedule_truncated": len(scheduled) != requested_case_count,
        "actual_schedule": [
            {
                "repetition": block_key[0],
                "workload": block_key[1],
                "policy": command[command.index("--policy") + 1],
            }
            for command, block_key in scheduled
        ],
        "environment_overrides": environment_overrides,
    }

    records: list[dict[str, Any]] = []
    completed_blocks: dict[tuple[int, str], list[dict[str, Any]]] = {}
    expected_per_block: dict[tuple[int, str], int] = {}
    for _, block_key in scheduled:
        expected_per_block[block_key] = expected_per_block.get(block_key, 0) + 1

    run_status: dict[str, Any] = {
        "state": "running",
        "complete": False,
        "valid_so_far": True,
        "validation_error": None,
        "completed_measurements": 0,
        "scheduled_measurements": len(scheduled),
        "validated_blocks": 0,
        "scheduled_blocks": len(expected_per_block),
    }

    total = len(scheduled)
    for index, (command, block_key) in enumerate(scheduled, start=1):
        print(f"[{index}/{total}] {' '.join(command)}", flush=True)
        try:
            record = run_worker(command, args.concurrency)
        except Exception as error:
            run_status.update(
                {
                    "state": "failed",
                    "valid_so_far": False,
                    "validation_error": f"worker failure: {error}",
                }
            )
            write_checkpoint(output, manifest, records, 0, run_status)
            raise
        record["validation_status"] = "pending"
        records.append(record)
        completed_blocks.setdefault(block_key, []).append(record)
        # Keep every completed raw sample even if the block-level correctness
        # check below finds a policy-dependent numerical or solver-path change.
        # Bootstrap resampling is deferred until the end so checkpointing does
        # not heat the controller CPU between timed worker processes.
        run_status["completed_measurements"] = len(records)
        block_records = completed_blocks[block_key]
        if len(block_records) == expected_per_block[block_key]:
            try:
                validate_signatures(
                    block_records,
                    args.signature_relative_tolerance,
                    args.signature_absolute_tolerance,
                )
            except Exception as error:
                for block_record in block_records:
                    block_record["validation_status"] = "invalid"
                run_status.update(
                    {
                        "state": "failed",
                        "valid_so_far": False,
                        "validation_error": f"signature validation failure: {error}",
                    }
                )
                write_checkpoint(output, manifest, records, 0, run_status)
                raise
            for block_record in block_records:
                block_record["validation_status"] = "valid"
            run_status["validated_blocks"] += 1
            write_checkpoint(output, manifest, records, 0, run_status)
        else:
            write_checkpoint(output, manifest, records, 0, run_status)
        print(
            f"  {record['seconds_per_solve'] * 1e3:.3f} ms/solve; "
            f"eval={record['phase_totals']['threading_evaluation_phase_seconds'] * 1e3:.3f} ms; "
            f"linear={record['phase_totals']['threading_linear_solver_phase_seconds'] * 1e3:.3f} ms",
            flush=True,
        )

    run_status.update({"state": "complete", "complete": True})
    write_checkpoint(output, manifest, records, args.bootstrap_samples, run_status)
    print(f"Wrote {output / 'python-solver-phase-threading.json'}")
    return 0


def main() -> int:
    args = parse_args()
    validate_args(args)
    if args.worker:
        return worker_main(args)
    return controller_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
