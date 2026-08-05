import numpy as np

import pypgo as pgo
import pypgo.fem as fem
import pypgo.solver as solver
from tests.pypgo.material_helpers import direct_material


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
    volume = pgo.mesh.volume.VolumeMesh(
        cube, pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45)
    )
    sim = pgo.fem.SimulationImportResult(volume)
    material = direct_material(
        sim,
        fem.StVKDefinition(),
        fem.VolumetricPlasticityDefinition(dofs=6),
        fem.ElementwiseParameterLayout,
        fem.ConstantParameterLayout,
    )
    operator = fem.DeformationEnergyOperator(
        material.mesh,
        material.binding,
        formulation=fem.CubicLinear(),
        options=fem.DeformationOptions(
            project_hessian_psd=False, enable_material_max_step=False
        ),
    )
    energy = fem.DeformationPotentialEnergy(operator, material.state)
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

    initial_plastic = energy.material_state.plastic_values.ravel()
    fixed_dofs = np.arange(0, 9, dtype=np.int64)
    fixed_values = np.zeros(9, dtype=np.float64)
    problem = solver.OptimizationProblem(objective=energy)
    problem.fix_variables(
        fixed_dofs.tolist(), fixed_values, num_dofs=energy.num_dofs
    )
    inner = solver.NewtonOptimizer(
        max_iterations=5, damping=solver.NoDamping()
    ).solve(problem, energy.zero_state())

    surface_vertex_ids = np.array([0, 1, 2, 3], dtype=np.int64)
    surface_vertices = (
        surface.vertices
        + inner.x.reshape((-1, 3))[surface_vertex_ids]
    )
    residual = surface_vertices - target
    grad_u = np.zeros(energy.num_dofs, dtype=np.float64)
    np.add.at(grad_u.reshape((-1, 3)), surface_vertex_ids, residual)

    free = np.setdiff1d(np.arange(energy.num_dofs), fixed_dofs)
    hessian = energy.hessian(inner.x).to_dense()
    adjoint = np.zeros(energy.num_dofs, dtype=np.float64)
    adjoint[free] = np.linalg.solve(
        hessian[np.ix_(free, free)], grad_u[free]
    )
    grad_plastic = -energy.plastic_material_vjp(inner.x, adjoint)

    assert np.isfinite(0.5 * np.dot(residual.ravel(), residual.ravel()))
    assert grad_plastic.shape == initial_plastic.shape
    assert np.all(np.isfinite(grad_plastic))
