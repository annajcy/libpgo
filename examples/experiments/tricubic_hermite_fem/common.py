"""Shared configuration for the tricubic Hermite FEM experiments."""

from __future__ import annotations

import json
from pathlib import Path


EXPERIMENT_DIR = Path(__file__).resolve().parent
ROOT = EXPERIMENT_DIR.parents[2]
ASSETS = EXPERIMENT_DIR / "assets"
OUTPUT_ROOT = ROOT / "examples" / "outputs" / "tricubic_hermite_fem"

FORMULATION_CASES = (
    "tet_ref",
    "cubic_linear",
    "cubic_linear_x8",
    "cubic_hermite",
    "cubic_linear_x27",
)

STUDIES = {
    "bunny": {
        "name": "bunny",
        "title": "Bunny Conservative R15",
        "prefix": "bunny-conservative-r15",
        "surface": ASSETS / "obj" / "bunny.obj",
        "fixed": ASSETS / "fixed" / "bunny-surface-fixed-ear-tip.txt",
        "obstacle": ASSETS / "obj" / "bottom.1.obj",
    },
    "dragon": {
        "name": "dragon",
        "title": "Dragon Conservative R15",
        "prefix": "dragon-conservative-r15",
        "surface": ASSETS / "obj" / "dragon.obj",
        "fixed": ASSETS / "fixed" / "dragon-surface-fixed.txt",
        "obstacle": ASSETS / "obj" / "bottom.1.obj",
    },
}

STATIC_SETTINGS = {
    "attachment_coeff": 1e5,
    "gravity": [0.0, -9.81, 0.0],
    "max_iterations": 300,
    "gradient_tolerance": 1e-5,
    "pin_residual_limit": 1.5e-3,
    "num_threads": 12,
}

DYNAMIC_SETTINGS = {
    "gravity": [0.0, -9.81, 0.0],
    "initial_velocity": [0.0, 0.0, 0.0],
    "timestep": 0.001,
    "num_steps": 800,
    "integrator": "implicit_euler",
    "damping": [0.0, 0.0],
    "dhat": 0.002,
    "dhat_external": 0.005,
    "kappa": 3000.0,
    "max_iterations": 200,
    "gradient_tolerance": 1e-4,
    "dump_interval": 10,
    "num_threads": 12,
}


def tet_reference_selection_path(study: dict) -> Path:
    """Return the atomic selection manifest written by tet tuning."""
    return ASSETS / "veg" / "tet" / f"{study['prefix']}-tet-reference.json"


def _selection_asset_path(identifier: str) -> Path:
    path = Path(identifier)
    return path if path.is_absolute() else EXPERIMENT_DIR / path


def tet_reference_selection(study: dict) -> dict | None:
    """Load the upstream binary-search result, if it has been generated."""
    selection_path = tet_reference_selection_path(study)
    if not selection_path.exists():
        return None
    selection = json.loads(selection_path.read_text())
    if selection.get("study") != study["name"]:
        raise ValueError(f"invalid tet selection study in {selection_path}")
    candidate = _selection_asset_path(selection["candidate_mesh"])
    metadata_path = _selection_asset_path(selection["candidate_metadata"])
    if not candidate.exists() or not metadata_path.exists():
        raise FileNotFoundError(f"tet selection points to missing candidate assets: {selection_path}")
    candidate_metadata = json.loads(metadata_path.read_text())
    if candidate_metadata.get("input_signature") != selection.get("input_signature"):
        raise ValueError(f"tet selection metadata does not match its candidate: {selection_path}")
    return selection


def tet_reference_mesh(selection: dict | None) -> Path | None:
    """Resolve the selected candidate mesh from its manifest."""
    return None if selection is None else _selection_asset_path(selection["candidate_mesh"])


def build_cases(study: dict) -> dict:
    """Return the five formulation cases for one study."""
    prefix = study["prefix"]
    cubic = ASSETS / "veg" / "cubic"
    selection = tet_reference_selection(study)
    return {
        "tet_ref": {
            "mesh_type": "tet",
            "volume": tet_reference_mesh(selection),
            "formulation": "tet-linear",
            "selection": selection,
        },
        "cubic_linear": {
            "mesh_type": "cubic",
            "volume": cubic / f"{prefix}.veg",
            "formulation": "cubic-linear",
        },
        "cubic_linear_x8": {
            "mesh_type": "cubic",
            "volume": cubic / f"{prefix}-subdiv2.veg",
            "formulation": "cubic-linear",
        },
        "cubic_hermite": {
            "mesh_type": "cubic",
            "volume": cubic / f"{prefix}.veg",
            "formulation": "cubic-tricubic-hermite",
        },
        "cubic_linear_x27": {
            "mesh_type": "cubic",
            "volume": cubic / f"{prefix}-subdiv3.veg",
            "formulation": "cubic-linear",
        },
    }


def output_root(study_name: str, mode: str) -> Path:
    """Return the canonical output root for one study and mode."""
    if study_name not in STUDIES:
        raise ValueError(f"unknown study: {study_name}")
    if mode not in {"static", "dynamic"}:
        raise ValueError(f"unknown mode: {mode}")
    return OUTPUT_ROOT / study_name / mode


def asset_id(path: Path) -> str:
    """Return a repo-location-independent identifier for an experiment asset."""
    return path.resolve().relative_to(EXPERIMENT_DIR.resolve()).as_posix()
