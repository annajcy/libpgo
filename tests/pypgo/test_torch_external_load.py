import numpy as np
import pytest

import pypgo as pgo
import pypgo.energy as pe
import pypgo.fem as pf
from tests.pypgo.material_helpers import direct_assignment

torch = pytest.importorskip("torch")

# Layer 1 (physical self-weight force/Jacobian) is covered in
# tests/pypgo/test_mass_fields.py; this file tests the torch layer wiring.


class _DenseJacobian:
    def __init__(self, values):
        self._values = values

    def to_dense(self):
        return self._values


class _ThicknessPointLoad:
    def __init__(self, energy, *, target_dof, parameter_dof, scale):
        self.energy = energy
        self.target_dof = target_dof
        self.parameter_dof = parameter_dof
        self.scale = scale

    @property
    def material_state(self):
        return self.energy.material_state

    def force(self, material_state):
        force = np.zeros(self.energy.num_dofs, dtype=np.float64)
        force[self.target_dof] = (
            self.scale
            * material_state.elastic_values.ravel()[self.parameter_dof]
        )
        return force

    def parameter_jacobian(self, material_state):
        jac = np.zeros((self.energy.num_dofs, self.energy.num_elastic_dofs), dtype=np.float64)
        jac[self.target_dof, self.parameter_dof] = self.scale
        return _DenseJacobian(jac)


def _setup(nx=2, ny=2, external_load="self_weight"):
    def vid(i, j):
        return i * (ny + 1) + j

    vertices = np.array(
        [[i / nx, j / ny, 0.0] for i in range(nx + 1) for j in range(ny + 1)],
        dtype=np.float64,
    )
    triangles = []
    for i in range(nx):
        for j in range(ny):
            triangles.append([vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)])
            triangles.append([vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)])
    triangles = np.asarray(triangles, dtype=np.int64)
    surface = pgo.mesh.TriMeshData(vertices, triangles)
    sim = pf.SimulationMesh(surface)

    base_row = np.array([2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3], dtype=np.float64)
    elastic = np.tile(base_row, (triangles.shape[0], 1))
    plastic = np.ones((triangles.shape[0], 1), dtype=np.float64)
    elastic_config = pf.KoiterStVKDefinition()
    plastic_config = pf.ShellPlasticityDefinition(dofs=1)
    assignment = direct_assignment(
        sim, elastic_config, plastic_config,
        pf.ElementwiseParameterLayout, pf.ElementwiseParameterLayout, elastic, plastic)
    operator = pf.DeformationEnergyOperator(
        assignment,
        formulation=pf.KoiterShell(),
        options=pf.DeformationOptions(project_hessian_psd=False, enable_material_max_step=False),
    )
    energy = pf.DeformationPotentialEnergy(
        operator, assignment.initial_material_state)

    if external_load == "point":
        load = _ThicknessPointLoad(energy, target_dof=2, parameter_dof=4, scale=1e6)
    else:
        areal_density = pf.ShellArealDensity.from_elastic_parameter(
            scale=1000.0,
            parameter=energy.material_state.elastic_field.parameter("thickness"),
        )
        load = pf.SelfWeightGravity(
            formulation=pf.KoiterShell(), mesh=sim, areal_density=areal_density,
            material_state=energy.material_state,
            acceleration=[0.0, 0.0, -20.0])

    fixed_vertices = np.flatnonzero(np.isclose(vertices[:, 1], 1.0)).astype(np.int64)
    fixed_dofs = (3 * fixed_vertices[:, None] + np.arange(3, dtype=np.int64)).ravel()
    layer = pgo.fem.ElasticStaticEquilibriumLayer(
        energy=energy,
        fixed_dofs=fixed_dofs,
        fixed_values=np.zeros(fixed_dofs.size),
        surface_vertices=vertices,
        surface_vertex_ids=np.arange(vertices.shape[0], dtype=np.int64),
        inner_optimizer=pgo.solver.NewtonOptimizer(max_iterations=200, gradient_tolerance=1e-11),
        external_load=load,
    )
    return layer, elastic, vertices


def test_external_load_objective_includes_parameter_current_load():
    layer, elastic, _vertices = _setup(external_load="point")
    state = layer._material_state(elastic.ravel())
    potential = pf.DeformationPotentialEnergy(
        layer.energy.energy_operator, state)
    objective = layer._build_objective(potential, state)
    u = layer.energy.zero_state()
    g0 = objective.gradient(u)

    modified = elastic.copy()
    modified[0, 4] *= 2.0
    state = layer._material_state(modified.ravel())
    potential = pf.DeformationPotentialEnergy(
        layer.energy.energy_operator, state)
    objective = layer._build_objective(potential, state)
    g1 = objective.gradient(u)

    assert g1[2] - g0[2] == pytest.approx(-1e6 * elastic[0, 4])


def test_external_load_jacobian_contributes_to_backward_material_vjp():
    layer, elastic, vertices = _setup(external_load="point")
    del elastic, vertices
    u = layer.energy.zero_state()

    adjoint = np.zeros(layer.energy.num_dofs, dtype=np.float64)
    adjoint[2] = 1.0
    vjp = layer._material_vjp(
        u, layer.energy.material_state, adjoint)
    energy_vjp = layer.energy.elastic_material_vjp(u, adjoint)

    assert vjp[4] == pytest.approx(energy_vjp[4] - 1e6)


def test_external_load_forward_smoke_records_inner_solve_without_displacement_assumption():
    layer, elastic, vertices = _setup(external_load="point")

    solved = layer(torch.tensor(elastic.ravel(), dtype=torch.float64))

    assert solved.shape == vertices.shape
    assert np.all(np.isfinite(solved.detach().numpy()))
    assert layer.last_inner_result.x.shape == (layer.energy.num_dofs,)


def test_external_load_rejected_on_plastic_layer():
    layer, elastic, _vertices = _setup()
    with pytest.raises(ValueError):
        pgo.fem.PlasticStaticEquilibriumLayer(
            energy=layer.energy,
            fixed_dofs=layer.fixed_dofs,
            fixed_values=layer.fixed_values,
            surface_vertices=layer.surface_vertices,
            surface_vertex_ids=layer.surface_vertex_ids,
            external_load=layer.external_load,
        )


def test_external_load_rejects_material_state_from_other_fields():
    layer, _elastic, _vertices = _setup(external_load="point")
    other, _other_elastic, _other_vertices = _setup(external_load="point")
    foreign_load = _ThicknessPointLoad(
        other.energy, target_dof=2, parameter_dof=4, scale=1e6
    )

    with pytest.raises(ValueError, match="same optimizable fields"):
        pgo.fem.ElasticStaticEquilibriumLayer(
            energy=layer.energy,
            fixed_dofs=layer.fixed_dofs,
            fixed_values=layer.fixed_values,
            surface_vertices=layer.surface_vertices,
            surface_vertex_ids=layer.surface_vertex_ids,
            external_load=foreign_load,
        )


def test_equilibrium_layer_rejects_legacy_objective_energy_argument():
    layer, _elastic, _vertices = _setup()
    with pytest.raises(TypeError, match="objective_energy"):
        pgo.fem.ElasticStaticEquilibriumLayer(
            energy=layer.energy,
            fixed_dofs=layer.fixed_dofs,
            fixed_values=layer.fixed_values,
            surface_vertices=layer.surface_vertices,
            surface_vertex_ids=layer.surface_vertex_ids,
            objective_energy=layer.energy,
        )


def test_equilibrium_layer_validates_additional_energy_shape():
    layer, _elastic, _vertices = _setup()
    extra = pe.LinearEnergy(np.zeros(layer.energy.num_dofs - 1))
    with pytest.raises(ValueError, match="additional_energy num_dofs"):
        pgo.fem.ElasticStaticEquilibriumLayer(
            energy=layer.energy,
            additional_energy=extra,
            fixed_dofs=layer.fixed_dofs,
            fixed_values=layer.fixed_values,
            surface_vertices=layer.surface_vertices,
            surface_vertex_ids=layer.surface_vertex_ids,
        )


def test_equilibrium_layer_requires_deformation_energy():
    layer, _elastic, _vertices = _setup()
    generic_energy = pe.LinearEnergy(np.zeros(layer.energy.num_dofs))
    with pytest.raises(TypeError, match="DeformationPotentialEnergy"):
        pgo.fem.ElasticStaticEquilibriumLayer(
            energy=generic_energy,
            fixed_dofs=layer.fixed_dofs,
            fixed_values=layer.fixed_values,
            surface_vertices=layer.surface_vertices,
            surface_vertex_ids=layer.surface_vertex_ids,
        )
