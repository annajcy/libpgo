"""Recover a shell's membrane-stiffness field from a target shape."""

from importlib.util import find_spec
from pathlib import Path

import numpy as np
import pypgo as pgo
import pypgo.energy as pe
import pypgo.fem as pf
import pypgo.solver as ps
import torch


OUTPUT_DIR = Path(__file__).resolve().parent / "output"
GRID_SIZE = 5
NUM_STEPS = 300


def make_shell_grid(n: int) -> tuple[np.ndarray, np.ndarray]:
    vertex_ids = np.arange((n + 1) ** 2).reshape((n + 1, n + 1))
    vertices = np.array(
        [[i / n, j / n, 0.0] for i in range(n + 1) for j in range(n + 1)],
        dtype=np.float64,
    )
    triangles = np.array(
        [
            triangle
            for i in range(n)
            for j in range(n)
            for triangle in (
                [vertex_ids[i, j], vertex_ids[i + 1, j], vertex_ids[i + 1, j + 1]],
                [vertex_ids[i, j], vertex_ids[i + 1, j + 1], vertex_ids[i, j + 1]],
            )
        ],
        dtype=np.int64,
    )
    return vertices, triangles


def main() -> None:
    torch.set_default_dtype(torch.float64)

    # Build a regular triangular shell grid.
    vertices, triangles = make_shell_grid(GRID_SIZE)
    surface = pgo.mesh.TriMeshData(vertices, triangles)
    simulation_mesh = pf.SimulationMesh.create_shell(
        surface,
        pf.KoiterStVKShellMaterial(
            thickness=1.0e-3,
            E_membrane=2.0e4,
            nu_membrane=0.35,
        ),
    )

    # Create the elementwise shell material field.
    base_material = np.array([2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3])
    initial_elastic = np.tile(base_material, (triangles.shape[0], 1))
    plastic_values = np.ones((triangles.shape[0], 1))
    energy = pf.deformation_energy(
        simulation_mesh,
        elastic=pf.KoiterStVK(),
        elastic_field=pf.ElementwiseField(values=initial_elastic),
        plastic=pf.ShellPlasticity(dofs=1),
        plastic_field=pf.ElementwiseField(values=plastic_values),
        formulation=pf.KoiterShell(),
        options=pf.DeformationOptions(
            enforce_spd=False,
            enable_material_max_step=False,
        ),
    )

    # Apply self-weight and clamp the top edge.
    mass_field = pf.ShellDensityElasticThickness(
        density=1000.0,
        parameter_field=energy.elastic_field,
        channel=4,
    )
    external_load = pf.SelfWeightGravity(
        formulation=pf.KoiterShell(),
        sim_mesh=simulation_mesh,
        mass_field=mass_field,
        acceleration=np.array([0.0, 0.0, -20.0]),
    )
    fixed_vertices = np.flatnonzero(np.isclose(vertices[:, 1], 1.0))
    fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3)).ravel()
    fixed_values = np.zeros(fixed_dofs.size)
    inner_optimizer = ps.NewtonOptimizer(
        max_iterations=80,
        gradient_tolerance=1.0e-9,
        damping=ps.FixedDamping(),
        line_search=ps.Backtrack(),
    )

    # Generate a target from a hidden membrane-stiffness field.
    centers = vertices[triangles].mean(axis=1)
    softness = (1.0 - centers[:, 1]) ** 0.8
    softness *= np.exp(-(((centers[:, 0] - 0.5) / 0.75) ** 2))
    target_elastic = initial_elastic.copy()
    target_elastic[:, 0] *= 1.0 - 0.98 * softness
    energy.set_elastic_values(target_elastic)

    target_objective = pe.EnergySet(
        [(energy, 1.0), (pe.LinearEnergy(-external_load.force()), 1.0)]
    )
    target_problem = ps.OptimizationProblem(objective=target_objective)
    target_problem.fix_variables(
        fixed_dofs.tolist(),
        fixed_values,
        num_dofs=energy.num_dofs,
    )
    target_result = inner_optimizer.solve(target_problem, energy.zero_state())
    target_vertices = vertices + target_result.x.reshape((-1, 3))
    target_vertices[:, 2] += (
        0.08 * (1.0 - vertices[:, 1]) * np.sin(2.0 * np.pi * vertices[:, 0])
    )
    energy.set_elastic_values(initial_elastic)

    # Differentiate the observed surface through static equilibrium.
    layer = pgo.fem.ElasticStaticEquilibriumLayer(
        energy=energy,
        objective_energy=energy,
        external_load=external_load,
        fixed_dofs=fixed_dofs,
        fixed_values=fixed_values,
        surface_vertices=vertices,
        surface_vertex_ids=np.arange(vertices.shape[0]),
        inner_optimizer=inner_optimizer,
    )

    # Parameterize the membrane modulus with a small neural network.
    points = torch.as_tensor(centers[:, :2])
    target_tensor = torch.as_tensor(target_vertices)
    base_tensor = torch.as_tensor(initial_elastic.ravel())
    network = torch.nn.Sequential(
        torch.nn.Linear(2, 8),
        torch.nn.Tanh(),
        torch.nn.Linear(8, 8),
        torch.nn.Tanh(),
        torch.nn.Linear(8, 1),
    ).to(torch.float64)
    with torch.no_grad():
        network[-1].weight.zero_()
        network[-1].bias.zero_()

    optimizer = torch.optim.Adam(network.parameters(), lr=0.05)
    history = []
    best_loss = np.inf
    best_state = {
        key: value.detach().clone() for key, value in network.state_dict().items()
    }
    best_displacement = energy.zero_state()

    # Fit the network with the adjoint gradient from the equilibrium layer.
    for step in range(NUM_STEPS):
        optimizer.zero_grad()
        membrane = 12000.0 + 8000.0 * network(points).squeeze(-1)
        elastic = base_tensor.clone()
        elastic[0::5] = membrane
        solved_vertices = layer(elastic)
        residual = solved_vertices - target_tensor
        loss = 0.5 * torch.sum(residual**2)
        loss += 2.0e-4 * sum(
            torch.sum(parameter**2) for parameter in network.parameters()
        )
        loss.backward()

        value = float(loss.detach())
        error = float(torch.linalg.norm(residual).detach())
        if value < best_loss:
            best_loss = value
            best_state = {
                key: state.detach().clone()
                for key, state in network.state_dict().items()
            }
            best_displacement = layer.last_equilibrium_displacement
        optimizer.step()
        history.append((value, error))

        if step < 5 or (step + 1) % 40 == 0:
            values = membrane.detach()
            print(
                f"step {step + 1:3d}  loss={value:.6f}  error={error:.6f}  "
                f"E=[{float(values.min()):.0f}, {float(values.max()):.0f}]"
            )

    # Re-evaluate and save the best material field.
    network.load_state_dict(best_state)
    with torch.no_grad():
        membrane = 12000.0 + 8000.0 * network(points).squeeze(-1)
        best_elastic = base_tensor.clone()
        best_elastic[0::5] = membrane
    layer.reset_warm_start(best_displacement)
    optimized_vertices = layer(best_elastic).detach().numpy().copy()
    optimized_elastic = best_elastic.numpy().reshape(initial_elastic.shape)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / "elastic_shape_match_weights.npz"
    np.savez(
        output_path,
        vertices=vertices,
        triangles=triangles,
        target_vertices=target_vertices,
        optimized_vertices=optimized_vertices,
        initial_elastic=initial_elastic,
        target_elastic=target_elastic,
        optimized_elastic=optimized_elastic,
        history=np.asarray(history),
    )

    # Compare the target with the optimized equilibrium shape.
    target_surface = pgo.mesh.TriMeshData(target_vertices, triangles)
    optimized_surface = pgo.mesh.TriMeshData(optimized_vertices, triangles)
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

    shape_error = np.linalg.norm(optimized_vertices - target_vertices)
    target_modulus = target_elastic[:, 0]
    optimized_modulus = optimized_elastic[:, 0]
    correlation = (
        np.corrcoef(target_modulus, optimized_modulus)[0, 1]
        if np.std(optimized_modulus) > 0.0
        else np.nan
    )
    print(f"best loss: {best_loss:.6f}")
    print(f"final shape error: {shape_error:.6f}")
    print(f"membrane-stiffness correlation: {correlation:.4f}")
    print("saved results ->", output_path)


if __name__ == "__main__":
    main()
