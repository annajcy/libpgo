"""Dump free and confined uniaxial static-solve surfaces as OBJ files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.fem as pf
import pypgo.solver as ps

import examples.experiments.systematic_poking_fit_reaction_force.main as experiment


DEFAULT_STRETCH = 2.0 ** -0.5
DEFAULT_OUTPUT_DIR = (
    Path(__file__).resolve().parent
    / "output"
    / "uniaxial_static_solve_obj"
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


def _solve_protocol(
    energy,
    optimizer,
    rest_vertices: np.ndarray,
    elements: np.ndarray,
    stretch: float,
    *,
    confined: bool,
) -> tuple[pgo.mesh.TriMeshData, dict[str, object]]:
    case = experiment._uniaxial_case(
        rest_vertices,
        stretch,
        confined=confined,
        split="obj_dump",
    )
    state, _ = experiment._solve_case(
        energy,
        optimizer,
        case,
        warm_start=None,
    )
    deformed_vertices = (
        rest_vertices + state.displacement.reshape((-1, 3))
    )
    surface = _surface_from_deformed_cubes(
        deformed_vertices,
        elements,
    )
    bbox_min = deformed_vertices.min(axis=0)
    bbox_max = deformed_vertices.max(axis=0)
    protocol = "confined_uniaxial" if confined else "free_uniaxial"
    metadata = {
        "protocol": protocol,
        "stretch": stretch,
        "reaction": state.reaction,
        "static_iterations": state.iterations,
        "max_free_residual": state.free_residual_max,
        "num_fixed_dofs": int(case.fixed_dofs.size),
        "num_free_dofs": int(case.free_dofs.size),
        "bbox_min": bbox_min.tolist(),
        "bbox_max": bbox_max.tolist(),
        "bbox_extent": (bbox_max - bbox_min).tolist(),
    }
    return surface, metadata


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
    plot_vertices = []
    for _, surface in surfaces:
        # Display the axial y direction vertically.
        plot_vertices.append(surface.vertices[:, [0, 2, 1]])
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

    figure.suptitle("Target Neo-Hookean static solves")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return True


def dump_uniaxial_objs(
    *,
    stretch: float,
    grid_size: int,
    output_dir: Path,
) -> dict[str, object]:
    if not np.isfinite(stretch) or stretch <= 0.0:
        raise ValueError("stretch must be finite and positive")
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
        termination=ps.AbsoluteTermination(abs_tolerance=1.0e-10),
        damping=ps.FixedDamping(),
        line_search=ps.Backtrack(),
    )

    rest_surface = _surface_from_deformed_cubes(
        rest_vertices,
        elements,
    )
    rest_path = output_dir / "rest.obj"
    pgo.mesh.write_obj(str(rest_path), rest_surface)

    surfaces = []
    cases = []
    for confined in (False, True):
        surface, metadata = _solve_protocol(
            target_energy,
            optimizer,
            rest_vertices,
            elements,
            stretch,
            confined=confined,
        )
        protocol = str(metadata["protocol"])
        obj_path = output_dir / f"{protocol}.obj"
        pgo.mesh.write_obj(str(obj_path), surface)
        metadata["obj"] = obj_path.name
        cases.append(metadata)
        surfaces.append((protocol, surface))

    preview_path = output_dir / "preview.png"
    preview_written = _write_preview(surfaces, preview_path)
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
        "stretch": stretch,
        "top_displacement_y": stretch - 1.0,
        "cases": cases,
        "preview": preview_path.name if preview_written else None,
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stretch",
        type=float,
        default=DEFAULT_STRETCH,
        help="prescribed axial stretch (default: an actual 17-knot train case)",
    )
    parser.add_argument("--grid-size", type=int, default=2)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
    )
    args = parser.parse_args(argv)

    manifest = dump_uniaxial_objs(
        stretch=args.stretch,
        grid_size=args.grid_size,
        output_dir=args.output_dir,
    )
    for case in manifest["cases"]:
        print(
            f"{case['protocol']}: "
            f"reaction={case['reaction']:.9e}, "
            f"residual={case['max_free_residual']:.3e}, "
            f"bbox_extent={case['bbox_extent']}"
        )
    print("saved OBJ files ->", args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
