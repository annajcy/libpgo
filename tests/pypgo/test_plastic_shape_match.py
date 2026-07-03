import numpy as np
import os
import pytest
import torch
import sys
from pathlib import Path

import pypgo as pgo
import pypgo.energy as pe
import pypgo.fem as fem
import pypgo.solver as solver


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
    sim = pgo.fem.SimulationMesh.create_volumetric(volume)
    energy = fem.deformation_energy(
        sim,
        elastic=fem.StVK(),
        elastic_field=fem.ElementwiseField(),
        plastic=fem.VolumetricPlasticity(dofs=6),
        plastic_field=fem.ConstantField(),
        formulation=fem.CubicLinear(),
        options=fem.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
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
    sim = pgo.fem.SimulationMesh.create_shell(surface, material)
    energy = fem.deformation_energy(
        sim,
        elastic=fem.KoiterStVK(),
        elastic_field=fem.ConstantField(values=np.array([[2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3]], dtype=np.float64)),
        plastic=fem.ShellPlasticity(dofs=1),
        plastic_field=fem.ElementwiseField(values=np.array([[1.03], [1.02]], dtype=np.float64)),
        formulation=fem.KoiterShell(),
        options=fem.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
    )
    return sim, energy


def test_adjoint_plastic_gradient_can_be_assembled_directly():
    _, energy = make_cubic_case()
    rest = energy.rest_position
    surface = pgo.mesh.TriMeshData(
        rest[[0, 1, 2, 3]],
        np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64),
    )
    target = surface.vertices.copy()
    target[:, 0] *= 1.02

    a0 = energy.plastic_field.values.ravel()
    energy.set_plastic_values(a0.reshape(energy.plastic_field.values.shape))
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
    plastic_jacobian = energy.plastic_jacobian(inner.x).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(hessian[np.ix_(free, free)], grad_u[free])
    grad_a = -(plastic_jacobian.T @ adjoint)

    assert np.isfinite(0.5 * np.dot(residual.ravel(), residual.ravel()))
    assert grad_a.shape == a0.shape
    assert np.all(np.isfinite(grad_a))


def test_static_equilibrium_torch_layer_backward_matches_direct_adjoint():
    _, energy = make_cubic_case()
    rest = energy.rest_position
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
        inner_optimizer=solver.NewtonOptimizer(max_iterations=5, damping=solver.NoDamping()),
    )

    a0 = energy.plastic_field.values.ravel()
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
    plastic_jacobian = energy.plastic_jacobian(layer.last_equilibrium_displacement).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(hessian[np.ix_(free, free)], grad_u[free])
    expected_grad = -(plastic_jacobian.T @ adjoint)

    assert solved_surface.shape == target_torch.shape
    assert plastic_param.grad is not None
    assert np.allclose(plastic_param.grad.detach().numpy(), expected_grad)


def test_static_equilibrium_torch_layer_elastic_backward_matches_direct_adjoint():
    _, energy = make_shell_elastic_case()
    rest = energy.rest_position
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
        inner_optimizer=solver.NewtonOptimizer(max_iterations=5, damping=solver.NoDamping()),
    )

    b0 = energy.elastic_field.values.ravel()
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
    elastic_jacobian = energy.elastic_jacobian(layer.last_equilibrium_displacement).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(hessian[np.ix_(free, free)], grad_u[free])
    expected_grad = -(elastic_jacobian.T @ adjoint)

    assert solved_surface.shape == target_torch.shape
    assert elastic_jacobian.shape == (energy.num_dofs, energy.num_elastic_dofs)
    assert elastic_param.grad is not None
    assert np.allclose(elastic_param.grad.detach().numpy(), expected_grad)


def test_elastic_static_equilibrium_layer_uses_objective_energy_for_adjoint_hessian():
    _, energy = make_shell_elastic_case()
    rest = energy.rest_position
    surface_vertex_ids = np.arange(rest.shape[0], dtype=np.int64)
    surface_vertices = rest[surface_vertex_ids]
    target = surface_vertices.copy()
    target[:, 2] -= 0.03 * target[:, 1]

    gravity_force = np.zeros(energy.num_dofs, dtype=np.float64)
    gravity_force[2::3] = -0.02
    gravity_energy = pe.LinearEnergy(-gravity_force)
    objective = pe.EnergySet([(energy, 1.0), (gravity_energy, 1.0)])

    fixed_vertices = np.array([0, 1], dtype=np.int64)
    fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3, dtype=np.int64)).ravel()
    fixed_values = np.zeros(fixed_dofs.size, dtype=np.float64)
    layer = pgo.fem.ElasticStaticEquilibriumLayer(
        energy=energy,
        objective_energy=objective,
        fixed_dofs=fixed_dofs,
        fixed_values=fixed_values,
        surface_vertices=surface_vertices,
        surface_vertex_ids=surface_vertex_ids,
        inner_optimizer=solver.NewtonOptimizer(max_iterations=5, damping=solver.NoDamping()),
    )

    b0 = energy.elastic_field.values.ravel()
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
    elastic_jacobian = energy.elastic_jacobian(layer.last_equilibrium_displacement).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(hessian[np.ix_(free, free)], grad_u[free])
    expected_grad = -(elastic_jacobian.T @ adjoint)

    assert solved_surface.shape == target_torch.shape
    assert elastic_param.grad is not None
    assert np.allclose(elastic_param.grad.detach().numpy(), expected_grad)


@pytest.mark.skipif(
    os.environ.get("PYPGO_RUN_NOTEBOOK_TESTS") != "1",
    reason="example notebook generator checks are opt-in",
)
def test_plastic_shape_match_demo_module_imports():
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    import examples.scripts.notebook_generators.generate_plastic_shape_match_demo as demo

    assert hasattr(demo, "run_demo")
