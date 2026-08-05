#!/usr/bin/env python3
"""Benchmark public Python FEM evaluations under ArenaThreadingExecutor policies."""

from __future__ import annotations

import argparse
import json
import os
import platform
import random
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT = Path(__file__).resolve()
ROOT = SCRIPT.parents[2]
BENCHMARKS_ROOT = SCRIPT.parents[1]
if str(BENCHMARKS_ROOT) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS_ROOT))

from benchmark_support.harness import (  # noqa: E402
    add_benchmark_harness_arguments,
    prepare_benchmark_host,
)
from benchmark_support.artifact import JsonArtifact, runner_manifest  # noqa: E402
from benchmark_support.schedule import balanced_order, order_configuration  # noqa: E402
from benchmark_support.python_worker import (  # noqa: E402
    run_json_worker,
    worker_environment as isolated_worker_environment,
)
from benchmark_support.statistics import median_absolute_deviation  # noqa: E402
from benchmark_support.warmup import (  # noqa: E402
    add_workload_warmup_arguments,
    run_workload_warmup,
    validate_workload_warmup_arguments,
    workload_warmup_configuration,
)


RESULT_MARKER = "PYPGO_FEM_THREADING_RESULT="

FORMULATIONS = (
    "tet_linear",
    "cubic_linear",
    "cubic_tricubic_hermite",
)
OPERATIONS = ("value", "gradient", "hessian")
MKL_POLICIES = (
    "no_executor",
    "budget_0",
    "budget_1",
    "budget_c",
    "arena_1_budget_1",
)
ACCELERATE_POLICIES = (
    "accelerate_single",
    "accelerate_multi",
)
POLICIES = MKL_POLICIES + ACCELERATE_POLICIES
BACKEND_POLICIES = {
    "mkl": MKL_POLICIES,
    "accelerate": ACCELERATE_POLICIES,
}
REFERENCE_POLICIES = {
    "mkl": "budget_1",
    "accelerate": "accelerate_single",
}

FORMULATION_METADATA = {
    "tet_linear": {"mesh_kind": "tet", "local_dofs": 12},
    "cubic_linear": {"mesh_kind": "cubic", "local_dofs": 24},
    "cubic_tricubic_hermite": {"mesh_kind": "cubic", "local_dofs": 192},
}


def parse_args(default_backend: str = "mkl") -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend", choices=tuple(BACKEND_POLICIES), default=default_backend
    )
    parser.add_argument("--out", type=Path)
    parser.add_argument(
        "--tet-mesh",
        type=Path,
        default=ROOT / "examples/assets/veg/tet/box.veg",
    )
    parser.add_argument(
        "--cubic-mesh",
        type=Path,
        default=ROOT / "examples/assets/veg/cubic/box.veg",
    )
    parser.add_argument(
        "--formulations", nargs="+", choices=FORMULATIONS, default=FORMULATIONS
    )
    parser.add_argument(
        "--operations", nargs="+", choices=OPERATIONS, default=OPERATIONS
    )
    parser.add_argument("--policies", nargs="+", choices=POLICIES)
    parser.add_argument("--concurrency", type=int, default=8)
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--min-time", type=float, default=0.5)
    parser.add_argument("--min-iterations", type=int, default=1)
    parser.add_argument("--seed", type=int, default=20260716)
    parser.add_argument("--displacement-scale", type=float, default=1e-4)
    parser.add_argument(
        "--elastic-model",
        choices=("stable_neo", "stvk", "linear"),
        default="stable_neo",
    )
    parser.add_argument("--plastic-dofs", type=int, choices=(0, 3, 6), default=6)
    parser.add_argument("--signature-relative-tolerance", type=float, default=1e-9)
    parser.add_argument("--native-profile", action="store_true")
    parser.add_argument("--case-limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    add_workload_warmup_arguments(
        parser, default_seconds=0.0, default_min_operations=10
    )
    add_benchmark_harness_arguments(parser)

    # The controller starts one fresh worker process for every timed sample.
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--formulation", choices=FORMULATIONS, help=argparse.SUPPRESS)
    parser.add_argument("--operation", choices=OPERATIONS, help=argparse.SUPPRESS)
    parser.add_argument("--policy", choices=POLICIES, help=argparse.SUPPRESS)
    parser.add_argument("--repetition", type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.policies is None:
        args.policies = BACKEND_POLICIES[args.backend]
    return args


def validate_args(args: argparse.Namespace) -> None:
    if args.backend == "accelerate" and sys.platform != "darwin":
        raise SystemExit("backend=accelerate requires macOS")
    invalid_policies = sorted(set(args.policies) - set(BACKEND_POLICIES[args.backend]))
    if invalid_policies:
        raise SystemExit(
            f"policies {invalid_policies} are not valid for backend={args.backend}"
        )
    if args.concurrency <= 0:
        raise SystemExit("--concurrency must be positive")
    if args.repetitions <= 0:
        raise SystemExit("--repetitions must be positive")
    if args.min_time < 0:
        raise SystemExit("--min-time must be non-negative")
    if args.min_iterations <= 0:
        raise SystemExit("--min-iterations must be positive")
    try:
        validate_workload_warmup_arguments(args)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.displacement_scale < 0:
        raise SystemExit("--displacement-scale must be non-negative")
    if args.signature_relative_tolerance < 0:
        raise SystemExit("--signature-relative-tolerance must be non-negative")
    if args.case_limit < 0:
        raise SystemExit("--case-limit must be non-negative")

    for label, path in (("tet", args.tet_mesh), ("cubic", args.cubic_mesh)):
        if not path.is_file():
            raise SystemExit(f"{label} mesh does not exist: {path}")

    if args.worker:
        if args.formulation is None or args.operation is None or args.policy is None:
            raise SystemExit(
                "worker mode requires --formulation, --operation, and --policy"
            )
    elif args.out is None and not args.dry_run:
        raise SystemExit("controller mode requires --out")


def _elastic_model(pf, name: str):
    factories = {
        "stable_neo": pf.StableNeoDefinition,
        "stvk": pf.StVKDefinition,
        "linear": pf.LinearElasticDefinition,
    }
    return factories[name]()


def _formulation(pf, name: str):
    factories = {
        "tet_linear": pf.TetLinear,
        "cubic_linear": pf.CubicLinear,
        "cubic_tricubic_hermite": pf.CubicTricubicHermite,
    }
    return factories[name]()


def _policy_parameters(name: str, concurrency: int) -> dict[str, Any] | None:
    if name == "no_executor":
        return None
    if name == "budget_0":
        return {
            "arena_concurrency": concurrency,
            "mkl_local_thread_budget": 0,
            "accelerate": "single",
        }
    if name == "budget_1":
        return {
            "arena_concurrency": concurrency,
            "mkl_local_thread_budget": 1,
            "accelerate": "single",
        }
    if name == "budget_c":
        return {
            "arena_concurrency": concurrency,
            "mkl_local_thread_budget": concurrency,
            "accelerate": "single",
        }
    if name == "arena_1_budget_1":
        return {
            "arena_concurrency": 1,
            "mkl_local_thread_budget": 1,
            "accelerate": "single",
        }
    if name == "accelerate_single":
        return {
            "arena_concurrency": concurrency,
            "mkl_local_thread_budget": 1,
            "accelerate": "single",
        }
    if name == "accelerate_multi":
        return {
            "arena_concurrency": concurrency,
            "mkl_local_thread_budget": 1,
            "accelerate": "multi",
        }
    raise ValueError(f"unknown policy: {name}")


def _result_signature(result: Any, operation: str) -> dict[str, Any]:
    import numpy as np

    if operation == "value":
        value = float(result)
        return {"kind": "scalar", "value": value}

    if operation == "gradient":
        values = np.asarray(result, dtype=np.float64)
        return {
            "kind": "dense_vector",
            "size": int(values.size),
            "sum": float(np.sum(values, dtype=np.float64)),
            "absolute_sum": float(np.sum(np.abs(values), dtype=np.float64)),
            "squared_norm": float(np.sum(values * values, dtype=np.float64)),
            "max_abs": float(np.max(np.abs(values), initial=0.0)),
        }

    if operation == "hessian":
        rows, cols, values = result.to_coo()
        values = np.asarray(values, dtype=np.float64)
        return {
            "kind": "sparse_matrix",
            "shape": list(result.shape),
            "nnz": int(result.nnz),
            "row_sum": int(np.sum(rows, dtype=np.int64)),
            "col_sum": int(np.sum(cols, dtype=np.int64)),
            "value_sum": float(np.sum(values, dtype=np.float64)),
            "absolute_sum": float(np.sum(np.abs(values), dtype=np.float64)),
            "squared_norm": float(np.sum(values * values, dtype=np.float64)),
            "max_abs": float(np.max(np.abs(values), initial=0.0)),
        }

    raise ValueError(f"unknown operation: {operation}")


def _profile_snapshot(profiling, enabled: bool) -> dict[str, Any] | None:
    if not enabled:
        return None
    return {
        "sections": profiling.snapshot(),
        "counters": profiling.snapshot_counters(),
    }


def worker_main(args: argparse.Namespace) -> int:
    # Keep imports below environment setup in the controller. NumPy and pypgo may
    # themselves load a BLAS implementation during import.
    sys.path.insert(0, str(ROOT))
    import gc

    import numpy as np
    import pypgo
    from pypgo.parallel import (
        AccelerateThreading,
        ArenaThreadingExecutor,
        GlobalTbbControl,
    )

    # Establish the process-wide oneTBB cap before mesh/material/energy
    # construction can initialize or use the scheduler. The worker is a fresh
    # process, so this owner intentionally lives until the result is complete.
    global_control = GlobalTbbControl(args.concurrency)

    import pypgo.fem as pf
    import pypgo.profiling as profiling
    from pypgo.mesh.volume import VolumeMesh, read_veg

    metadata = FORMULATION_METADATA[args.formulation]
    mesh_path = args.tet_mesh if metadata["mesh_kind"] == "tet" else args.cubic_mesh
    veg = read_veg(str(mesh_path))
    volume_mesh = VolumeMesh(veg)
    asset = pf.SimulationImportResult(volume_mesh)
    formulation = _formulation(pf, args.formulation)
    elastic = _elastic_model(pf, args.elastic_model)
    plastic = pf.VolumetricPlasticityDefinition(dofs=args.plastic_dofs)
    def identity_field(field_type, names):
        count = len(names)
        return field_type(
            names,
            pf.ElementwiseParameterLayout(asset.num_elements, count),
            pf.IdentityMaterialChannelMapping(count))

    elastic_fixed = identity_field(
        pf.FixedParameterField, elastic.fixed_channel_names)
    plastic_fixed = identity_field(
        pf.FixedParameterField, plastic.fixed_channel_names)
    elastic_optimizable = identity_field(
        pf.OptimizableParameterField, elastic.optimizable_channel_names)
    plastic_optimizable = identity_field(
        pf.OptimizableParameterField, plastic.optimizable_channel_names)
    if args.plastic_dofs == 6:
        plastic_values = np.tile(
            np.array([1.0, 0.0, 0.0, 1.0, 0.0, 1.0]),
            (asset.num_elements, 1),
        )
    elif args.plastic_dofs == 3:
        plastic_values = np.ones((asset.num_elements, 3), dtype=np.float64)
    else:
        plastic_values = np.empty(0)
    def fixed_values(field):
        return np.asarray(
            pf.project_imported_material_inputs(asset.material_catalog, field),
            dtype=np.float64,
        ).reshape(-1)

    material_binding = pf.MaterialBinding(
        pf.ElasticMaterialBinding(
            elastic,
            pf.FixedMaterialParameters(
                elastic_fixed, fixed_values(elastic_fixed)),
            elastic_optimizable),
        pf.PlasticMaterialBinding(
            plastic,
            pf.FixedMaterialParameters(
                plastic_fixed, fixed_values(plastic_fixed)),
            plastic_optimizable),
        pf.GlobalAxesMaterialFrameField(asset.num_elements),
    )
    material_state = pf.MaterialState(
        np.empty(0, dtype=np.float64),
        np.ascontiguousarray(plastic_values.reshape(-1)))
    energy_operator = pf.DeformationEnergyOperator(
        asset.mesh, material_binding,
        formulation=formulation,
        options=pf.DeformationOptions(
            project_hessian_psd=True,
            enable_material_max_step=False,
        ),
    )
    energy = pf.DeformationPotentialEnergy(
        energy_operator, material_state
    )

    rng = np.random.default_rng(args.seed)
    displacement = np.ascontiguousarray(
        rng.normal(0.0, args.displacement_scale, energy.num_dofs),
        dtype=np.float64,
    )
    displacement_signature = {
        "generator": "numpy.default_rng.normal",
        "seed": args.seed,
        "scale": args.displacement_scale,
        "size": int(displacement.size),
        "nonzero_count": int(np.count_nonzero(displacement)),
        "l2_norm": float(
            np.sqrt(np.sum(displacement * displacement, dtype=np.float64))
        ),
        "max_abs": float(np.max(np.abs(displacement), initial=0.0)),
    }

    if args.operation == "value":
        evaluation_method = energy.value
    elif args.operation == "gradient":
        evaluation_method = energy.gradient
    else:
        evaluation_method = energy.hessian

    def evaluate():
        return evaluation_method(displacement)

    policy_parameters = _policy_parameters(args.policy, args.concurrency)
    executor = None
    if policy_parameters is not None:
        accelerate = {
            "single": AccelerateThreading.SINGLE,
            "multi": AccelerateThreading.MULTI,
        }[policy_parameters["accelerate"]]
        executor = ArenaThreadingExecutor(
            policy_parameters["arena_concurrency"],
            mkl_local_thread_budget=policy_parameters["mkl_local_thread_budget"],
            accelerate=accelerate,
        )

    def invoke(fn):
        if executor is None:
            return fn()
        return executor.execute(fn)

    def timed_loop():
        started = time.perf_counter()
        elapsed = 0.0
        iterations = 0
        last = None
        while iterations < args.min_iterations or elapsed < args.min_time:
            last = evaluate()
            iterations += 1
            elapsed = time.perf_counter() - started
        return elapsed, iterations, last

    _, warmup_result = invoke(
        lambda: run_workload_warmup(
            evaluate,
            minimum_seconds=args.warmup_seconds,
            minimum_operations=args.warmup_min_operations,
        )
    )
    gc.collect()
    if args.native_profile:
        profiling.set_enabled(True)
        profiling.reset()
    outer_started = time.perf_counter()
    elapsed, iterations, last = invoke(timed_loop)
    execute_wall_seconds = time.perf_counter() - outer_started
    native_profile = _profile_snapshot(profiling, args.native_profile)
    if args.native_profile:
        profiling.set_enabled(False)

    if last is None:
        raise RuntimeError("timed loop produced no result")

    result = {
        "formulation": args.formulation,
        "operation": args.operation,
        "policy": args.policy,
        "backend": args.backend,
        "repetition": args.repetition,
        "concurrency": args.concurrency,
        "arena_concurrency": (
            None
            if policy_parameters is None
            else policy_parameters["arena_concurrency"]
        ),
        "mkl_local_thread_budget": (
            None
            if policy_parameters is None
            else policy_parameters["mkl_local_thread_budget"]
        ),
        "accelerate": None
        if policy_parameters is None
        else policy_parameters["accelerate"],
        "mesh": {
            "path": str(mesh_path.resolve()),
            "kind": metadata["mesh_kind"],
            "num_vertices": asset.num_vertices,
            "num_elements": asset.num_elements,
        },
        "material": {
            "elastic_model": args.elastic_model,
            "plastic_dofs": args.plastic_dofs,
            "project_hessian_psd": True,
        },
        "local_dofs": metadata["local_dofs"],
        "num_dofs": energy.num_dofs,
        "displacement_scale": args.displacement_scale,
        "displacement": displacement_signature,
        "warmup": workload_warmup_configuration(args)
        | {
            "actual_seconds": warmup_result.elapsed_seconds,
            "actual_operations": warmup_result.completed_operations,
        },
        "iterations": iterations,
        "wall_seconds": elapsed,
        "execute_wall_seconds": execute_wall_seconds,
        "seconds_per_evaluation": elapsed / iterations,
        "signature": _result_signature(last, args.operation),
        "native_profile": native_profile,
        "runtime": {
            "python": sys.version,
            "platform": platform.platform(),
            "pypgo": str(Path(pypgo.__file__).resolve()),
            "pypgo_core": str(Path(pypgo._core.__file__).resolve()),
            "environment": {
                name: os.environ.get(name)
                for name in (
                    "MKL_NUM_THREADS",
                    "MKL_DYNAMIC",
                    "MKL_THREADING_LAYER",
                    "OMP_NUM_THREADS",
                    "VECLIB_MAXIMUM_THREADS",
                    "VECLIB_DEFAULT_THREAD_COUNT",
                )
            },
        },
    }
    global_control.close()
    print(RESULT_MARKER + json.dumps(result, sort_keys=True))
    return 0


def worker_command(
    args: argparse.Namespace,
    formulation: str,
    operation: str,
    policy: str,
    repetition: int,
) -> list[str]:
    command = [
        sys.executable,
        str(SCRIPT),
        "--worker",
        "--backend",
        args.backend,
        "--formulation",
        formulation,
        "--operation",
        operation,
        "--policy",
        policy,
        "--repetition",
        str(repetition),
        "--tet-mesh",
        str(args.tet_mesh.resolve()),
        "--cubic-mesh",
        str(args.cubic_mesh.resolve()),
        "--concurrency",
        str(args.concurrency),
        "--min-time",
        str(args.min_time),
        "--min-iterations",
        str(args.min_iterations),
        "--warmup-seconds",
        str(args.warmup_seconds),
        "--warmup-min-operations",
        str(args.warmup_min_operations),
        "--seed",
        str(args.seed),
        "--displacement-scale",
        str(args.displacement_scale),
        "--elastic-model",
        args.elastic_model,
        "--plastic-dofs",
        str(args.plastic_dofs),
    ]
    if args.native_profile:
        command.append("--native-profile")
    return command


def worker_environment(
    backend: str, concurrency: int
) -> tuple[dict[str, str], dict[str, str]]:
    overrides: dict[str, str] = {}
    if backend == "mkl":
        overrides = {
            "MKL_NUM_THREADS": str(concurrency),
            "OMP_NUM_THREADS": str(concurrency),
            "MKL_DYNAMIC": "FALSE",
        }
    return isolated_worker_environment(ROOT, overrides), overrides


def run_worker(command: list[str], backend: str, concurrency: int) -> dict[str, Any]:
    env, _ = worker_environment(backend, concurrency)
    return run_json_worker(
        command,
        marker=RESULT_MARKER,
        cwd=ROOT,
        environment=env,
    )


def _close_numeric(
    left: float,
    right: float,
    relative_tolerance: float,
    comparison_scale: float | None = None,
) -> bool:
    scale = max(abs(left), abs(right), comparison_scale or 0.0, 1.0)
    return abs(left - right) <= relative_tolerance * scale


def validate_signatures(
    records: list[dict[str, Any]], relative_tolerance: float
) -> None:
    if len(records) < 2:
        return
    reference = records[0]
    reference_signature = reference["signature"]
    exact_fields = {
        "scalar": (),
        "dense_vector": ("size",),
        "sparse_matrix": ("shape", "nnz", "row_sum", "col_sum"),
    }[reference_signature["kind"]]
    numeric_fields = {
        "scalar": (("value", None),),
        "dense_vector": (
            ("sum", "absolute_sum"),
            ("absolute_sum", None),
            ("squared_norm", None),
            ("max_abs", None),
        ),
        "sparse_matrix": (
            ("value_sum", "absolute_sum"),
            ("absolute_sum", None),
            ("squared_norm", None),
            ("max_abs", None),
        ),
    }[reference_signature["kind"]]

    for record in records[1:]:
        signature = record["signature"]
        if signature["kind"] != reference_signature["kind"]:
            raise RuntimeError(
                "result signature kinds differ across threading policies"
            )
        for field in exact_fields:
            if signature[field] != reference_signature[field]:
                raise RuntimeError(
                    f"signature field {field} differs between {reference['policy']} "
                    f"and {record['policy']}"
                )
        for field, scale_field in numeric_fields:
            comparison_scale = None
            if scale_field is not None:
                comparison_scale = max(
                    abs(float(signature[scale_field])),
                    abs(float(reference_signature[scale_field])),
                )
            if not _close_numeric(
                float(signature[field]),
                float(reference_signature[field]),
                relative_tolerance,
                comparison_scale,
            ):
                raise RuntimeError(
                    f"signature field {field} differs between {reference['policy']} "
                    f"and {record['policy']}: {reference_signature[field]} vs {signature[field]}"
                )


def summarize(
    records: list[dict[str, Any]], reference_policy: str
) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for record in records:
        grouped.setdefault((record["formulation"], record["operation"]), []).append(
            record
        )

    summaries: list[dict[str, Any]] = []
    for (formulation, operation), group in sorted(grouped.items()):
        policy_rows: dict[str, dict[str, Any]] = {}
        by_policy: dict[str, list[dict[str, Any]]] = {}
        for record in group:
            by_policy.setdefault(record["policy"], []).append(record)
        for policy, policy_records in sorted(by_policy.items()):
            values = [
                float(record["seconds_per_evaluation"]) for record in policy_records
            ]
            policy_rows[policy] = {
                "samples": len(values),
                "median_seconds_per_evaluation": statistics.median(values),
                "mad_seconds_per_evaluation": median_absolute_deviation(values),
                "min_seconds_per_evaluation": min(values),
                "max_seconds_per_evaluation": max(values),
            }

        paired_ratios: dict[str, dict[str, Any]] = {}
        reference_by_repetition = {
            int(record["repetition"]): record
            for record in by_policy.get(reference_policy, [])
        }
        for policy, policy_records in sorted(by_policy.items()):
            if policy == reference_policy:
                continue
            ratios = []
            for record in policy_records:
                repetition = int(record["repetition"])
                if repetition not in reference_by_repetition:
                    continue
                reference = reference_by_repetition[repetition]
                ratios.append(
                    float(record["seconds_per_evaluation"])
                    / float(reference["seconds_per_evaluation"])
                )
            if ratios:
                paired_ratios[f"{policy}_over_{reference_policy}"] = {
                    "samples": len(ratios),
                    "median": statistics.median(ratios),
                    "mad": median_absolute_deviation(ratios),
                }

        summaries.append(
            {
                "formulation": formulation,
                "operation": operation,
                "local_dofs": group[0]["local_dofs"],
                "mesh": group[0]["mesh"],
                "policies": policy_rows,
                "paired_time_ratios": paired_ratios,
            }
        )
    return summaries


def checkpoint_payload(
    manifest: dict[str, Any],
    records: list[dict[str, Any]],
    reference_policy: str,
) -> dict[str, Any]:
    return {
        "manifest": manifest,
        "measurements": records,
        "summary": summarize(records, reference_policy),
    }


def controller_main(args: argparse.Namespace) -> int:
    reference_policy = REFERENCE_POLICIES[args.backend]
    rng = random.Random(args.seed)
    blocks = [
        (repetition, formulation, operation)
        for repetition in range(args.repetitions)
        for formulation in args.formulations
        for operation in args.operations
    ]
    rng.shuffle(blocks)

    scheduled: list[tuple[list[str], tuple[int, str, str]]] = []
    for repetition, formulation, operation in blocks:
        policies = balanced_order(
            args.policies,
            repetition=repetition,
            seed=args.seed,
            block_key=f"{args.backend}:{formulation}:{operation}",
        )
        for policy in policies:
            scheduled.append(
                (
                    worker_command(args, formulation, operation, policy, repetition),
                    (repetition, formulation, operation),
                )
            )
    requested_case_count = len(scheduled)
    if args.case_limit:
        scheduled = scheduled[: args.case_limit]

    order = order_configuration(
        args.repetitions,
        [("policies", args.policies)],
        allow_incomplete=args.allow_incomplete_order_cycle,
        schedule_truncated=len(scheduled) != requested_case_count,
    )

    if args.dry_run:
        for command, _ in scheduled:
            print(" ".join(command))
        return 0

    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    host_environment = prepare_benchmark_host(args, workers=args.concurrency)
    _, environment_overrides = worker_environment(args.backend, args.concurrency)
    manifest = {
        "runner": runner_manifest(SCRIPT),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "script": str(SCRIPT),
        "backend": args.backend,
        "reference_policy": reference_policy,
        "pure_python_workload": True,
        "fresh_process_per_sample": True,
        "formulations": list(args.formulations),
        "operations": list(args.operations),
        "policies": list(args.policies),
        "concurrency": args.concurrency,
        "repetitions": args.repetitions,
        "order": order,
        "min_time": args.min_time,
        "min_iterations": args.min_iterations,
        "warmup": workload_warmup_configuration(args),
        "seed": args.seed,
        "displacement_scale": args.displacement_scale,
        "elastic_model": args.elastic_model,
        "plastic_dofs": args.plastic_dofs,
        "native_profile": args.native_profile,
        "host_environment": host_environment,
        "environment_overrides": environment_overrides,
        "tet_mesh": str(args.tet_mesh.resolve()),
        "cubic_mesh": str(args.cubic_mesh.resolve()),
    }

    records: list[dict[str, Any]] = []
    artifact = JsonArtifact(
        output / "python-fem-threading.json",
        scheduled_units=len(scheduled),
        sort_keys=True,
    )
    completed_blocks: dict[tuple[int, str, str], list[dict[str, Any]]] = {}
    total = len(scheduled)
    for index, (command, block_key) in enumerate(scheduled, start=1):
        policy = command[command.index("--policy") + 1]
        label = (
            f"r={block_key[0]}:formulation={block_key[1]}:"
            f"operation={block_key[2]}:policy={policy}"
        )
        artifact.set_active(label)
        with artifact.capture_failures(
            lambda: checkpoint_payload(manifest, records, reference_policy)
        ):
            print(f"[{index}/{total}] {' '.join(command)}", flush=True)
            record = run_worker(command, args.backend, args.concurrency)
            records.append(record)
            completed_blocks.setdefault(block_key, []).append(record)
            block_records = completed_blocks[block_key]
            expected_in_block = sum(1 for _, key in scheduled if key == block_key)
            if len(block_records) == expected_in_block:
                validate_signatures(block_records, args.signature_relative_tolerance)
        artifact.checkpoint(
            checkpoint_payload(manifest, records, reference_policy),
            completed_units=len(records),
        )
        print(
            f"  {record['seconds_per_evaluation'] * 1e3:.3f} ms/eval "
            f"({record['iterations']} iterations)",
            flush=True,
        )

    artifact.complete(checkpoint_payload(manifest, records, reference_policy))
    print(f"Wrote {artifact.path}")
    return 0


def main(default_backend: str = "mkl") -> int:
    args = parse_args(default_backend)
    validate_args(args)
    if args.worker:
        return worker_main(args)
    return controller_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
