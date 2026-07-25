"""Match a target shape by optimizing an elementwise plastic field."""

from importlib.util import find_spec
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.fem as pf
import pypgo.solver as ps
import torch


OUTPUT_DIR = Path(__file__).resolve().parent / "output"
GRID_SIZE = 4
NUM_STEPS = 60


def make_cubic_grid(n: int) -> tuple[np.ndarray, np.ndarray]:
    vertex_ids = np.arange((n + 1) ** 3).reshape((n + 1, n + 1, n + 1))
    vertices = np.array(
        [
            [i / n, j / n, k / n]
            for i in range(n + 1)
            for j in range(n + 1)
            for k in range(n + 1)
        ],
        dtype=np.float64,
    )
    elements = np.array(
        [
            [
                vertex_ids[i, j, k],
                vertex_ids[i + 1, j, k],
                vertex_ids[i + 1, j + 1, k],
                vertex_ids[i, j + 1, k],
                vertex_ids[i, j, k + 1],
                vertex_ids[i + 1, j, k + 1],
                vertex_ids[i + 1, j + 1, k + 1],
                vertex_ids[i, j + 1, k + 1],
            ]
            for i in range(n)
            for j in range(n)
            for k in range(n)
        ],
        dtype=np.int64,
    )
    return vertices, elements


def main() -> None:
    torch.set_default_dtype(torch.float64)

    # Build the simulation mesh and its observed surface.
    vertices, elements = make_cubic_grid(GRID_SIZE)
    cubic = pgo.mesh.CubicMeshData(vertices, elements)
    volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(
        cubic,
        pgo.mesh.volume.ENuMaterial(E=1.0e6, nu=0.45),
    )
    surface = volume.extract_surface_mesh()
    surface_ids = np.array(
        [
            np.flatnonzero(np.all(np.isclose(vertices, point), axis=1))[0]
            for point in surface.vertices
        ],
        dtype=np.int64,
    )

    # Shear the rest surface to create the fitting target.
    target_vertices = surface.vertices.copy()
    target_vertices[:, 0] += 0.35 * surface.vertices[:, 1]
    target_vertices[:, 2] += 0.15 * surface.vertices[:, 1]
    target_surface = pgo.mesh.TriMeshData(target_vertices, surface.elements)

    # Use one six-channel plastic tensor per element.
    simulation_mesh = pf.SimulationMesh.create_volumetric(volume)
    energy = pf.deformation_energy(
        simulation_mesh,
        elastic=pf.StVK(),
        plastic=pf.VolumetricPlasticity(dofs=6),
        formulation=pf.CubicLinear(),
        options=pf.DeformationOptions(
            project_hessian_psd=False,
            enable_material_max_step=False,
        ),
    )

    # Clamp the bottom face and expose equilibrium as a PyTorch layer.
    fixed_vertices = np.flatnonzero(np.isclose(vertices[:, 1], 0.0))
    fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3)).ravel()
    layer = pgo.fem.PlasticStaticEquilibriumLayer(
        energy=energy,
        fixed_dofs=fixed_dofs,
        fixed_values=np.zeros(fixed_dofs.size),
        surface_vertices=surface.vertices,
        surface_vertex_ids=surface_ids,
        inner_optimizer=ps.NewtonOptimizer(
            max_iterations=10,
            damping=ps.NoDamping(),
        ),
    )

    # Fit the plastic field through the differentiable equilibrium solve.
    initial_plastic = energy.parameters.plastic_values.copy()
    initial_tensor = torch.as_tensor(initial_plastic.ravel())
    target_tensor = torch.as_tensor(target_vertices)
    plastic = torch.tensor(initial_plastic.ravel(), requires_grad=True)
    optimizer = torch.optim.Adam([plastic], lr=0.01)
    history = []
    best_loss = np.inf
    best_plastic = initial_tensor.numpy().copy()

    for step in range(NUM_STEPS):
        optimizer.zero_grad()
        solved_vertices = layer(plastic)
        residual = solved_vertices - target_tensor
        loss = 0.5 * torch.sum(residual**2)
        loss += 0.5e-4 * torch.sum((plastic - initial_tensor) ** 2)
        loss.backward()

        value = float(loss.detach())
        error = float(torch.linalg.norm(residual).detach())
        if value < best_loss:
            best_loss = value
            best_plastic = plastic.detach().numpy().copy()
        optimizer.step()
        history.append((value, error))

        if step < 5 or (step + 1) % 10 == 0:
            print(f"step {step + 1:3d}  loss={value:.6f}  error={error:.6f}")

    # Re-evaluate and save the best design.
    layer.reset_warm_start()
    optimized_vertices = layer(torch.as_tensor(best_plastic)).detach().numpy().copy()
    optimized_plastic = best_plastic.reshape(initial_plastic.shape)
    optimized_surface = pgo.mesh.TriMeshData(optimized_vertices, surface.elements)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "plastic_shape_match_weights.npz"
    np.savez(
        output_path,
        vertices=vertices,
        elements=elements,
        surface_vertex_ids=surface_ids,
        target_vertices=target_vertices,
        optimized_vertices=optimized_vertices,
        initial_plastic=initial_plastic,
        optimized_plastic=optimized_plastic,
        history=np.asarray(history),
    )

    # Compare the target with the optimized equilibrium shape.
    if find_spec("pyvista") is None:
        print("PyVista is not installed; skipping visualization.")
    else:
        pgo.mesh.plot_surface(
            [target_surface, optimized_surface],
            titles=["target", "optimized"],
            colors=["palegreen", "salmon"],
            show_edges=True,
            window_size=(900, 420),
        )
    final_error = np.linalg.norm(optimized_vertices - target_vertices)
    print(f"best loss: {best_loss:.6f}")
    print(f"final shape error: {final_error:.6f}")
    print("saved results ->", output_path)


if __name__ == "__main__":
    main()
