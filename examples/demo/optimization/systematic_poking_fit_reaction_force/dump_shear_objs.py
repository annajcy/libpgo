"""Dump a simple-shear Neo-Hookean static solve as OBJ files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.fem as pf
import pypgo.solver as ps

import main as experiment


DEFAULT_SHEAR = 0.8
DEFAULT_OUTPUT_DIR = (
    Path(__file__).resolve().parent
    / "output"
    / "shear_static_solve_obj"
)


def _surface_from_deformed_cubes(
    vertices: np.ndarray,
    elements: np.ndarray,
) -> pgo.mesh.TriMeshData:
    cubes = pgo.mesh.CubicMeshData(vertices, elements)
    volume = pgo.mesh.volume.VolumeMesh(
        cubes,
        pgo.mesh.volume.ENuMaterial(),
    )
    return volume.extract_surface_mesh()


def _write_preview(
    surfaces: list[tuple[str, pgo.mesh.TriMeshData]],
    path: Path,
) -> bool:
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError:
        return False

    figure = plt.figure(figsize=(10, 5))
    plot_vertices = [
        surface.vertices[:, [0, 2, 1]] for _, surface in surfaces
    ]
    all_vertices = np.vstack(plot_vertices)
    center = 0.5 * (all_vertices.min(axis=0) + all_vertices.max(axis=0))
    radius = 0.55 * np.max(np.ptp(all_vertices, axis=0))

    for index, ((label, surface), vertices) in enumerate(
        zip(surfaces, plot_vertices),
        start=1,
    ):
        axes = figure.add_subplot(1, len(surfaces), index, projection="3d")
        triangles = vertices[surface.elements]
        collection = Poly3DCollection(
            triangles,
            facecolor="#6baed6",
            edgecolor="#17365d",
            linewidth=0.7,
            alpha=0.78,
        )
        axes.add_collection3d(collection)
        axes.set(
            xlim=(center[0] - radius, center[0] + radius),
            ylim=(center[1] - radius, center[1] + radius),
            zlim=(center[2] - radius, center[2] + radius),
            xlabel="x",
            ylabel="z",
            zlabel="y (axial)",
            title=label.replace("_", " "),
        )
        axes.set_box_aspect((1.0, 1.0, 1.0))
        axes.view_init(elev=20.0, azim=-55.0)

    figure.suptitle("Target Neo-Hookean simple-shear static solve")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return True


def dump_shear_objs(
    *,
    shear: float,
    grid_size: int,
    output_dir: Path,
) -> dict[str, object]:
    if not np.isfinite(shear):
        raise ValueError("shear must be finite")
    if grid_size < 2:
        raise ValueError("grid_size must be at least two")

    output_dir.mkdir(parents=True, exist_ok=True)
    rest_vertices, elements = experiment.make_cubic_grid(grid_size)
    imported = experiment._make_import(grid_size)
    target_energy = experiment._make_energy(
        imported,
        pf.NeoHookeanDefinition(),
        np.empty(0, dtype=np.float64),
    )
    optimizer = ps.NewtonOptimizer(
        max_iterations=80,
        gradient_tolerance=1.0e-10,
        damping=ps.FixedDamping(),
        line_search=ps.Backtrack(),
    )

    rest_surface = _surface_from_deformed_cubes(rest_vertices, elements)
    rest_path = output_dir / "rest.obj"
    pgo.mesh.write_obj(str(rest_path), rest_surface)

    case = experiment._shear_case(
        rest_vertices,
        shear,
        split="obj_dump",
    )
    state, _ = experiment._solve_case(
        target_energy,
        optimizer,
        case,
        warm_start=None,
    )
    deformed_vertices = (
        rest_vertices + state.displacement.reshape((-1, 3))
    )
    surface = _surface_from_deformed_cubes(deformed_vertices, elements)
    obj_path = output_dir / f"simple_shear_{shear:+.6f}.obj"
    pgo.mesh.write_obj(str(obj_path), surface)

    bbox_min = deformed_vertices.min(axis=0)
    bbox_max = deformed_vertices.max(axis=0)
    preview_path = output_dir / "preview.png"
    preview_written = _write_preview(
        [("rest", rest_surface), ("deformed", surface)],
        preview_path,
    )
    manifest = {
        "material": {
            "model": "NeoHookean",
            "youngs_modulus": experiment.YOUNGS_MODULUS,
            "poisson_ratio": experiment.POISSON_RATIO,
        },
        "mesh": {
            "grid_size": grid_size,
            "num_elements": int(elements.shape[0]),
            "num_vertices": int(rest_vertices.shape[0]),
            "num_displacement_dofs": int(rest_vertices.size),
            "rest_obj": rest_path.name,
        },
        "protocol": "simple_shear",
        "shear": shear,
        "reaction_x": state.reaction,
        "static_iterations": state.iterations,
        "max_free_residual": state.free_residual_max,
        "num_fixed_dofs": int(case.fixed_dofs.size),
        "num_free_dofs": int(case.free_dofs.size),
        "bbox_min": bbox_min.tolist(),
        "bbox_max": bbox_max.tolist(),
        "bbox_extent": (bbox_max - bbox_min).tolist(),
        "obj": obj_path.name,
        "preview": preview_path.name if preview_written else None,
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    return manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--shear",
        type=float,
        default=DEFAULT_SHEAR,
        help="simple-shear amount (default: 0.8, a training case)",
    )
    parser.add_argument("--grid-size", type=int, default=2)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
    )
    args = parser.parse_args(argv)

    manifest = dump_shear_objs(
        shear=args.shear,
        grid_size=args.grid_size,
        output_dir=args.output_dir,
    )
    print(
        f"simple_shear: gamma={manifest['shear']:+.9f}, "
        f"reaction_x={manifest['reaction_x']:.9e}, "
        f"residual={manifest['max_free_residual']:.3e}, "
        f"bbox_extent={manifest['bbox_extent']}"
    )
    print("saved OBJ files ->", args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
