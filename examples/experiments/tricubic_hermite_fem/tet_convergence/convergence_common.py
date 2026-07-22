"""Shared paths and case metadata for the independent tet convergence test."""

from __future__ import annotations

import json
import importlib.util
import sys
from pathlib import Path


CONVERGENCE_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = CONVERGENCE_DIR.parent
ROOT = EXPERIMENT_DIR.parents[2]
sys.path.insert(0, str(EXPERIMENT_DIR))


def _load_parent_common():
    spec = importlib.util.spec_from_file_location(
        "tricubic_hermite_fem_parent_common", EXPERIMENT_DIR / "common.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_PARENT = _load_parent_common()
OUTPUT_ROOT = _PARENT.OUTPUT_ROOT
STATIC_SETTINGS = _PARENT.STATIC_SETTINGS
STUDIES = _PARENT.STUDIES
build_cases = _PARENT.build_cases


TET_LEVEL_FACTORS = {"tet_L1": 2.0, "tet_L2": 4.0}
RUN_CASES = (
    "tet_L0",
    "tet_L0_tight",
    "tet_L1",
    "tet_L2",
    "cubic_linear_x64",
)
CANONICAL_CONTROLS = (
    "cubic_linear",
    "cubic_linear_x8",
    "cubic_hermite",
    "cubic_linear_x27",
)


def convergence_output(study_name: str) -> Path:
    if study_name not in STUDIES:
        raise ValueError(f"unknown study: {study_name}")
    return OUTPUT_ROOT / study_name / "tet_convergence"


def mesh_output(study_name: str) -> Path:
    return convergence_output(study_name) / "meshes"


def level_manifest_path(study_name: str, level: str) -> Path:
    if level not in TET_LEVEL_FACTORS:
        raise ValueError(f"unknown tet convergence level: {level}")
    return mesh_output(study_name) / f"{level}.json"


def load_level_manifest(study_name: str, level: str) -> dict:
    path = level_manifest_path(study_name, level)
    if not path.exists():
        raise FileNotFoundError(
            f"missing {level} manifest {path}; run generate_meshes.py first"
        )
    manifest = json.loads(path.read_text())
    if manifest.get("study") != study_name or manifest.get("level") != level:
        raise ValueError(f"invalid convergence manifest: {path}")
    mesh = Path(manifest["candidate_mesh"])
    if not mesh.is_absolute():
        mesh = ROOT / mesh
    if not mesh.exists():
        raise FileNotFoundError(f"manifest points to missing mesh: {mesh}")
    manifest["resolved_mesh"] = str(mesh)
    return manifest


def build_convergence_cases(study_name: str) -> dict:
    study = STUDIES[study_name]
    canonical = build_cases(study)
    if canonical["tet_ref"]["volume"] is None:
        raise FileNotFoundError(
            "canonical tet reference selection is missing; generate it before the "
            "convergence experiment"
        )
    cases = {
        "tet_L0": {
            "mesh_type": "tet",
            "volume": canonical["tet_ref"]["volume"],
            "formulation": "tet-linear",
            "gradient_tolerance": STATIC_SETTINGS["gradient_tolerance"],
            "source": "canonical_tet_ref",
            "manifest": canonical["tet_ref"]["selection"],
        },
        "tet_L0_tight": {
            "mesh_type": "tet",
            "volume": canonical["tet_ref"]["volume"],
            "formulation": "tet-linear",
            "gradient_tolerance": 1e-6,
            "source": "canonical_tet_ref_tolerance_control",
            "manifest": canonical["tet_ref"]["selection"],
        },
    }
    for level in TET_LEVEL_FACTORS:
        manifest = load_level_manifest(study_name, level)
        cases[level] = {
            "mesh_type": "tet",
            "volume": Path(manifest["resolved_mesh"]),
            "formulation": "tet-linear",
            "gradient_tolerance": STATIC_SETTINGS["gradient_tolerance"],
            "source": "independent_tet_convergence_manifest",
            "manifest": manifest,
        }
    x64 = mesh_output(study_name) / f"{study['prefix']}-subdiv4.veg"
    if not x64.exists():
        raise FileNotFoundError(f"missing x64 cubic mesh {x64}; run generate_meshes.py first")
    cases["cubic_linear_x64"] = {
        "mesh_type": "cubic",
        "volume": x64,
        "formulation": "cubic-linear",
        "gradient_tolerance": STATIC_SETTINGS["gradient_tolerance"],
        "source": "factor_four_cubic_subdivision",
        "manifest": None,
    }
    return cases
