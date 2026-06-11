"""Output writers for the pypgo-sim CLI family."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from pypgo.mesh import TriMeshData, write_obj


def write_summary(output_dir: Path, payload: dict) -> Path:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "summary.json"
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    return path


def write_surface(path: Path, vertices: np.ndarray, triangles: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_obj(str(path), TriMeshData(np.asarray(vertices, dtype=np.float64),
                                     np.asarray(triangles)))
