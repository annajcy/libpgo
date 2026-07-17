"""Real FEM workloads for the Python Newton phase-threading benchmark.

This module deliberately imports NumPy and pypgo inside ``build_workload``.
The benchmark controller starts a fresh worker process and configures its
threading environment before this builder loads either runtime.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


WORKLOADS = (
    "tet_linear",
    "cubic_linear",
    "cubic_tricubic_hermite",
)

DEFAULT_MESHES = {
    "tet_linear": "examples/assets/veg/tet/box-with-sphere-big.veg",
    "cubic_linear": "examples/assets/veg/cubic/dragon-center-r16.veg",
    "cubic_tricubic_hermite": "examples/assets/veg/cubic/box.veg",
}

LOCAL_DOFS = {
    "tet_linear": 12,
    "cubic_linear": 24,
    "cubic_tricubic_hermite": 192,
}


@dataclass(slots=True)
class SolverWorkload:
    """Objects retained for repeated public ``NewtonOptimizer.solve`` calls."""

    energy: Any
    problem: Any
    x0: Any
    fixed_dofs: Any
    metadata: dict[str, Any]


def _component_top_anchors(
    np: Any,
    vertices: Any,
    elements: Any,
    slab_fraction: float,
) -> tuple[Any, list[dict[str, Any]]]:
    """Select a top support for every connected component.

    The default tet workload contains two disconnected bodies. A single global
    top slab anchors only the upper body and leaves exact rigid modes in the
    other one, making the Newton system singular. Keep the whole requested top
    slab for each component; only expand it when fewer than three non-collinear
    vertices were selected.
    """

    num_vertices = int(vertices.shape[0])
    parent = list(range(num_vertices))
    component_size = [1] * num_vertices

    def find(vertex: int) -> int:
        while parent[vertex] != vertex:
            parent[vertex] = parent[parent[vertex]]
            vertex = parent[vertex]
        return vertex

    def unite(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root == right_root:
            return
        if component_size[left_root] < component_size[right_root]:
            left_root, right_root = right_root, left_root
        parent[right_root] = left_root
        component_size[left_root] += component_size[right_root]

    for element in np.asarray(elements, dtype=np.int64):
        first = int(element[0])
        for vertex in element[1:]:
            unite(first, int(vertex))

    grouped: dict[int, list[int]] = {}
    for vertex in range(num_vertices):
        grouped.setdefault(find(vertex), []).append(vertex)
    components = sorted(grouped.values(), key=lambda indices: indices[0])

    fixed_components = []
    component_metadata = []
    for component_index, indices in enumerate(components):
        component = np.asarray(indices, dtype=np.int64)
        positions = vertices[component]
        y = positions[:, 1]
        y_range = float(np.max(y) - np.min(y))
        threshold = float(np.max(y) - slab_fraction * y_range)
        selected = component[y >= threshold]
        initial_count = int(selected.size)

        def is_anchored(candidate: Any) -> bool:
            if candidate.size < 3:
                return False
            centered = vertices[candidate] - np.mean(vertices[candidate], axis=0)
            return bool(np.linalg.matrix_rank(centered) >= 2)

        expanded = False
        if not is_anchored(selected):
            order = component[np.lexsort((component, -y))]
            start = max(3, initial_count + 1)
            for count in range(start, int(order.size) + 1):
                candidate = order[:count]
                if is_anchored(candidate):
                    selected = candidate
                    expanded = True
                    break
            else:
                raise RuntimeError(
                    f"component {component_index} has no three non-collinear "
                    "vertices and cannot be rigidly anchored"
                )

        selected = np.sort(selected)
        fixed_components.append(selected)
        component_metadata.append(
            {
                "component": component_index,
                "num_vertices": int(component.size),
                "initial_slab_vertices": initial_count,
                "fixed_vertices": int(selected.size),
                "expanded_to_anchor": expanded,
            }
        )

    return np.unique(np.concatenate(fixed_components)), component_metadata


def _elastic_model(pf: Any, name: str) -> Any:
    factories = {
        "stable_neo": pf.StableNeo,
        "stvk": pf.StVK,
        "linear": pf.LinearElastic,
    }
    return factories[name]()


def _formulation(pf: Any, name: str) -> Any:
    factories = {
        "tet_linear": pf.TetLinear,
        "cubic_linear": pf.CubicLinear,
        "cubic_tricubic_hermite": pf.CubicTricubicHermite,
    }
    return factories[name]()


def build_workload(
    *,
    name: str,
    mesh_path: Path,
    seed: int,
    displacement_scale: float,
    fixed_slab_fraction: float,
    elastic_model: str,
    plastic_dofs: int,
) -> SolverWorkload:
    """Build one perturbed, constrained, real volumetric FEM solve.

    The top slab is fixed to the rest displacement. Fixing at least three
    non-collinear vertices removes rigid-body null modes without adding a
    benchmark-only penalty energy. Every other DOF receives deterministic
    Gaussian noise so the first Newton iteration is non-trivial.
    """

    if name not in WORKLOADS:
        raise ValueError(f"unknown workload: {name}")

    import numpy as np
    import pypgo.fem as pf
    import pypgo.solver as ps
    from pypgo.mesh.volume import VolumeMesh, read_veg

    mesh_path = mesh_path.resolve()
    mesh_sha256 = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
    veg = read_veg(str(mesh_path))
    volume_mesh = VolumeMesh.from_veg_file(veg)
    simulation_mesh = pf.SimulationMesh.create_volumetric(volume_mesh)
    energy = pf.deformation_energy(
        simulation_mesh,
        elastic=_elastic_model(pf, elastic_model),
        elastic_field=pf.ElementwiseField(),
        plastic=pf.VolumetricPlasticity(dofs=plastic_dofs),
        plastic_field=pf.ElementwiseField(),
        formulation=_formulation(pf, name),
        options=pf.DeformationOptions(
            enforce_spd=True,
            enable_material_max_step=False,
        ),
    )

    vertices = np.asarray(volume_mesh.mesh_data.vertices, dtype=np.float64)
    fixed_vertices, component_anchors = _component_top_anchors(
        np,
        vertices,
        volume_mesh.mesh_data.elements,
        fixed_slab_fraction,
    )

    num_vertices = int(volume_mesh.num_vertices)
    if int(energy.num_dofs) % num_vertices != 0:
        raise RuntimeError("energy DOFs are not vertex-aligned")
    dofs_per_vertex = int(energy.num_dofs) // num_vertices
    fixed_dofs = (
        fixed_vertices[:, None] * dofs_per_vertex
        + np.arange(dofs_per_vertex, dtype=np.int64)
    ).ravel()
    if fixed_dofs.size >= int(energy.num_dofs):
        raise RuntimeError(
            "fixed slab selected every DOF; decrease --fixed-slab-fraction"
        )

    rng = np.random.default_rng(seed)
    x0 = np.asarray(energy.zero_state(), dtype=np.float64).copy()
    x0 += rng.normal(0.0, displacement_scale, x0.size)
    x0[fixed_dofs] = 0.0
    x0 = np.ascontiguousarray(x0, dtype=np.float64)

    problem = ps.OptimizationProblem(objective=energy)
    problem.fix_variables(
        fixed_dofs.tolist(),
        np.zeros(fixed_dofs.size, dtype=np.float64),
        num_dofs=x0.size,
    )

    fixed_bytes = np.asarray(fixed_dofs, dtype="<i8").tobytes()
    perturbation = x0.copy()
    perturbation[fixed_dofs] = 0.0
    metadata = {
        "name": name,
        "mesh": {
            "path": str(mesh_path),
            "sha256": mesh_sha256,
            "num_vertices": num_vertices,
            "num_elements": int(simulation_mesh.num_elements),
        },
        "material": {
            "elastic_model": elastic_model,
            "plastic_dofs": plastic_dofs,
            "enforce_spd": True,
            "enable_material_max_step": False,
        },
        "local_element_dofs": LOCAL_DOFS[name],
        "num_dofs": int(energy.num_dofs),
        "dofs_per_vertex": dofs_per_vertex,
        "fixed_boundary": {
            "axis": "y",
            "side": "max",
            "slab_fraction": fixed_slab_fraction,
            "num_components": len(component_anchors),
            "components": component_anchors,
            "num_vertices": int(fixed_vertices.size),
            "num_dofs": int(fixed_dofs.size),
            "dof_sha256": hashlib.sha256(fixed_bytes).hexdigest(),
        },
        "perturbation": {
            "generator": "numpy.default_rng.normal",
            "seed": seed,
            "scale": displacement_scale,
            "nonzero_count": int(np.count_nonzero(perturbation)),
            "l2_norm": float(np.linalg.norm(perturbation)),
            "max_abs": float(np.max(np.abs(perturbation), initial=0.0)),
        },
    }
    return SolverWorkload(
        energy=energy,
        problem=problem,
        x0=x0,
        fixed_dofs=fixed_dofs,
        metadata=metadata,
    )


__all__ = [
    "DEFAULT_MESHES",
    "LOCAL_DOFS",
    "SolverWorkload",
    "WORKLOADS",
    "build_workload",
]
