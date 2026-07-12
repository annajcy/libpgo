"""Shared configuration for the tricubic Hermite FEM experiments."""

from __future__ import annotations

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
        "tet_a": "2.8768e-9",
        "surface": ASSETS / "obj" / "bunny.obj",
        "fixed": ASSETS / "fixed" / "bunny-surface-fixed-ear-tip.txt",
        "obstacle": ASSETS / "obj" / "bottom.1.obj",
    },
    "dragon": {
        "name": "dragon",
        "title": "Dragon Conservative R15",
        "prefix": "dragon-conservative-r15",
        "tet_a": "1.45885e-7",
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
    "pin_residual_limit": 1e-3,
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


def build_cases(study: dict) -> dict:
    """Return the five formulation cases for one study."""
    prefix = study["prefix"]
    cubic = ASSETS / "veg" / "cubic"
    return {
        "tet_ref": {
            "mesh_type": "tet",
            "volume": ASSETS / "veg" / "tet" / f"{prefix}-tet-a{study['tet_a']}.veg",
            "formulation": "tet-linear",
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
