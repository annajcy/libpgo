import numpy as np
import torch
from pathlib import Path

import pypgo as pgo
import pypgo.energy as pe
import pypgo.fem as fem
import pypgo.solver as solver
from tests.pypgo.material_helpers import direct_assignment


ROOT = Path(__file__).resolve().parents[2]



def make_cubic_case():
    cube = pgo.mesh.CubicMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
                [0.0, 1.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64),
    )
    volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(
        cube, pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45)
    )
    sim = pgo.fem.SimulationAsset.create_volumetric(volume)
    elastic = fem.StVKDefinition()
    plastic = fem.VolumetricPlasticityDefinition(dofs=6)
    assignment = direct_assignment(
        sim, elastic, plastic, fem.ElementwiseParameterLayout, fem.ConstantParameterLayout)
    energy = fem.DeformationEnergy(
        assignment,
        formulation=fem.CubicLinear(),
        options=fem.DeformationOptions(
            project_hessian_psd=False, enable_material_max_step=False
        ),
    )
    return sim, energy


def make_shell_elastic_case():
    surface = pgo.mesh.TriMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64),
    )
    material = pgo.fem.KoiterStVKShellMaterial(
        thickness=1.0e-3,
        E_membrane=2.0e4,
        nu_membrane=0.35,
    )
    sim = pgo.fem.SimulationAsset.create_shell(surface, material)
    elastic = fem.KoiterStVKDefinition()
    plastic = fem.ShellPlasticityDefinition(dofs=1)
    elastic_values = np.array(
        [[2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3]], dtype=np.float64
    )
    plastic_values = np.array([[1.03], [1.02]], dtype=np.float64)
    assignment = direct_assignment(
        sim, elastic, plastic, fem.ConstantParameterLayout, fem.ElementwiseParameterLayout,
        elastic_values, plastic_values)
    energy = fem.DeformationEnergy(
        assignment,
        formulation=fem.KoiterShell(),
        options=fem.DeformationOptions(
            project_hessian_psd=False, enable_material_max_step=False
        ),
    )
    return sim, energy


def test_adjoint_dE_dp_can_be_assembled_directly():
    _, energy = make_cubic_case()
    rest = energy.vertex_rest_positions
    surface = pgo.mesh.TriMeshData(
        rest[[0, 1, 2, 3]],
        np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64),
    )
    target = surface.vertices.copy()
    target[:, 0] *= 1.02

    a0 = energy.optimizable_parameters.plastic_values.ravel()
    energy.optimizable_parameters.set_plastic_values(
        a0.reshape(energy.optimizable_parameters.plastic_values.shape)
    )
    fixed_dofs = np.arange(0, 9, dtype=np.int64)
    fixed_values = np.zeros(9, dtype=np.float64)
    problem = solver.OptimizationProblem(objective=energy)
    problem.fix_variables(fixed_dofs.tolist(), fixed_values, num_dofs=energy.num_dofs)
    inner = solver.NewtonOptimizer(max_iterations=5, damping=solver.NoDamping()).solve(
        problem,
        energy.zero_state(),
    )

    surface_vertex_ids = np.array([0, 1, 2, 3], dtype=np.int64)
    surface_vertices = surface.vertices + inner.x.reshape((-1, 3))[surface_vertex_ids]
    residual = surface_vertices - target
    grad_u = np.zeros(energy.num_dofs, dtype=np.float64)
    np.add.at(grad_u.reshape((-1, 3)), surface_vertex_ids, residual)

    free = np.setdiff1d(np.arange(energy.num_dofs), fixed_dofs)
    hessian = energy.hessian(inner.x).to_dense()
    d2E_dudp = energy.d2E_dudp(inner.x).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(hessian[np.ix_(free, free)], grad_u[free])
    grad_a = -(d2E_dudp.T @ adjoint)

    assert np.isfinite(0.5 * np.dot(residual.ravel(), residual.ravel()))
    assert grad_a.shape == a0.shape
    assert np.all(np.isfinite(grad_a))


def test_static_equilibrium_torch_layer_backward_matches_direct_adjoint():
    _, energy = make_cubic_case()
    rest = energy.vertex_rest_positions
    surface_vertex_ids = np.array([0, 1, 2, 3], dtype=np.int64)
    surface_vertices = rest[surface_vertex_ids]
    target = surface_vertices.copy()
    target[:, 0] *= 1.02

    fixed_dofs = np.arange(0, 9, dtype=np.int64)
    fixed_values = np.zeros(9, dtype=np.float64)
    layer = pgo.fem.PlasticStaticEquilibriumLayer(
        energy=energy,
        fixed_dofs=fixed_dofs,
        fixed_values=fixed_values,
        surface_vertices=surface_vertices,
        surface_vertex_ids=surface_vertex_ids,
        inner_optimizer=solver.NewtonOptimizer(
            max_iterations=5, damping=solver.NoDamping()
        ),
    )

    a0 = energy.optimizable_parameters.plastic_values.ravel()
    plastic_param = torch.tensor(a0, dtype=torch.float64, requires_grad=True)
    target_torch = torch.as_tensor(target, dtype=torch.float64)

    solved_surface = layer(plastic_param)
    loss = 0.5 * torch.sum((solved_surface - target_torch) ** 2)
    loss.backward()

    residual = layer.last_surface_vertices - target
    grad_u = np.zeros(energy.num_dofs, dtype=np.float64)
    np.add.at(grad_u.reshape((-1, 3)), surface_vertex_ids, residual)

    free = np.setdiff1d(np.arange(energy.num_dofs), fixed_dofs)
    hessian = energy.hessian(layer.last_equilibrium_displacement).to_dense()
    d2E_dudp = energy.d2E_dudp(
        layer.last_equilibrium_displacement
    ).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(hessian[np.ix_(free, free)], grad_u[free])
    expected_grad = -(d2E_dudp.T @ adjoint)

    assert solved_surface.shape == target_torch.shape
    assert plastic_param.grad is not None
    assert np.allclose(plastic_param.grad.detach().numpy(), expected_grad)


def test_static_equilibrium_torch_layer_elastic_backward_matches_direct_adjoint():
    _, energy = make_shell_elastic_case()
    rest = energy.vertex_rest_positions
    surface_vertex_ids = np.arange(rest.shape[0], dtype=np.int64)
    surface_vertices = rest[surface_vertex_ids]
    target = surface_vertices.copy()
    target[:, 0] += 0.03 * target[:, 1]
    target[:, 2] += 0.02 * target[:, 1]

    fixed_vertices = np.array([0, 1], dtype=np.int64)
    fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3, dtype=np.int64)).ravel()
    fixed_values = np.zeros(fixed_dofs.size, dtype=np.float64)
    layer = pgo.fem.ElasticStaticEquilibriumLayer(
        energy=energy,
        fixed_dofs=fixed_dofs,
        fixed_values=fixed_values,
        surface_vertices=surface_vertices,
        surface_vertex_ids=surface_vertex_ids,
        inner_optimizer=solver.NewtonOptimizer(
            max_iterations=5, damping=solver.NoDamping()
        ),
    )

    b0 = energy.optimizable_parameters.elastic_values.ravel()
    elastic_param = torch.tensor(b0, dtype=torch.float64, requires_grad=True)
    target_torch = torch.as_tensor(target, dtype=torch.float64)

    solved_surface = layer(elastic_param)
    loss = 0.5 * torch.sum((solved_surface - target_torch) ** 2)
    loss.backward()

    residual = layer.last_surface_vertices - target
    grad_u = np.zeros(energy.num_dofs, dtype=np.float64)
    np.add.at(grad_u.reshape((-1, 3)), surface_vertex_ids, residual)

    free = np.setdiff1d(np.arange(energy.num_dofs), fixed_dofs)
    hessian = energy.hessian(layer.last_equilibrium_displacement).to_dense()
    d2E_dude = energy.d2E_dude(
        layer.last_equilibrium_displacement
    ).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(hessian[np.ix_(free, free)], grad_u[free])
    expected_grad = -(d2E_dude.T @ adjoint)

    assert solved_surface.shape == target_torch.shape
    assert d2E_dude.shape == (energy.num_dofs, energy.num_elastic_dofs)
    assert elastic_param.grad is not None
    assert np.allclose(elastic_param.grad.detach().numpy(), expected_grad)


def test_elastic_static_equilibrium_layer_uses_additional_energy_for_adjoint_hessian():
    _, energy = make_shell_elastic_case()
    rest = energy.vertex_rest_positions
    surface_vertex_ids = np.arange(rest.shape[0], dtype=np.int64)
    surface_vertices = rest[surface_vertex_ids]
    target = surface_vertices.copy()
    target[:, 2] -= 0.03 * target[:, 1]

    extra_stiffness = pe.QuadraticEnergy(
        (
            energy.num_dofs,
            energy.num_dofs,
            np.arange(energy.num_dofs, dtype=np.int64),
            np.arange(energy.num_dofs, dtype=np.int64),
            np.full(energy.num_dofs, 0.01, dtype=np.float64),
        )
    )
    objective = pe.EnergySet([(energy, 1.0), (extra_stiffness, 1.0)])

    fixed_vertices = np.array([0, 1], dtype=np.int64)
    fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3, dtype=np.int64)).ravel()
    fixed_values = np.zeros(fixed_dofs.size, dtype=np.float64)
    layer = pgo.fem.ElasticStaticEquilibriumLayer(
        energy=energy,
        additional_energy=extra_stiffness,
        fixed_dofs=fixed_dofs,
        fixed_values=fixed_values,
        surface_vertices=surface_vertices,
        surface_vertex_ids=surface_vertex_ids,
        inner_optimizer=solver.NewtonOptimizer(
            max_iterations=5, damping=solver.NoDamping()
        ),
    )

    b0 = energy.optimizable_parameters.elastic_values.ravel()
    elastic_param = torch.tensor(b0, dtype=torch.float64, requires_grad=True)
    target_torch = torch.as_tensor(target, dtype=torch.float64)

    solved_surface = layer(elastic_param)
    loss = 0.5 * torch.sum((solved_surface - target_torch) ** 2)
    loss.backward()

    residual = layer.last_surface_vertices - target
    grad_u = np.zeros(energy.num_dofs, dtype=np.float64)
    np.add.at(grad_u.reshape((-1, 3)), surface_vertex_ids, residual)

    free = np.setdiff1d(np.arange(energy.num_dofs), fixed_dofs)
    hessian = objective.hessian(layer.last_equilibrium_displacement).to_dense()
    d2E_dude = energy.d2E_dude(
        layer.last_equilibrium_displacement
    ).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(hessian[np.ix_(free, free)], grad_u[free])
    expected_grad = -(d2E_dude.T @ adjoint)

    assert solved_surface.shape == target_torch.shape
    assert elastic_param.grad is not None
    assert np.allclose(elastic_param.grad.detach().numpy(), expected_grad)


def test_plastic_shape_match_demo_script_covers_inverse_design_path():
    script = (
        ROOT / "examples" / "demo" / "optimization" / "plastic_shape_match" / "main.py"
    )
    source = script.read_text()

    assert "PlasticStaticEquilibriumLayer" in source
    assert "torch.optim.Adam" in source
