import numpy as np
import pytest

import pypgo as pgo
import pypgo.fem as pf

torch = pytest.importorskip("torch")


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

    def force(self):
        force = np.zeros(self.energy.num_dofs, dtype=np.float64)
        force[self.target_dof] = self.scale * self.energy.elastic_field.values.ravel()[self.parameter_dof]
        return force

    def parameter_jacobian(self):
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
    material = pf.KoiterStVKShellMaterial(thickness=1e-3, E_membrane=2e4, nu_membrane=0.35)
    sim = pf.SimulationMesh.create_shell(surface, material)

    base_row = np.array([2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3], dtype=np.float64)
    elastic = np.tile(base_row, (triangles.shape[0], 1))
    plastic = np.ones((triangles.shape[0], 1), dtype=np.float64)
    energy = pf.deformation_energy(
        sim,
        elastic=pf.KoiterStVK(),
        elastic_field=pf.ElementwiseField(values=elastic),
        plastic=pf.ShellPlasticity(dofs=1),
        plastic_field=pf.ElementwiseField(values=plastic),
        formulation=pf.KoiterShell(),
        options=pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
    )

    if external_load == "point":
        load = _ThicknessPointLoad(energy, target_dof=2, parameter_dof=4, scale=1e6)
    else:
        mass_field = pf.ShellDensityElasticThickness(
            density=1000.0, parameter_field=energy.elastic_field, channel=4)
        load = pf.SelfWeightGravity(
            formulation=pf.KoiterShell(), sim_mesh=sim, mass_field=mass_field,
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


def test_external_load_forward_responds_to_parameter():
    layer, elastic, _vertices = _setup(external_load="point")
    b = torch.tensor(elastic.ravel(), dtype=torch.float64)
    out_uniform = layer(b).detach().numpy().copy()

    modified = elastic.copy()
    modified[0, 4] *= 2.0
    layer.reset_warm_start()
    out_modified = layer(torch.tensor(modified.ravel(), dtype=torch.float64)).detach().numpy().copy()
    assert not np.allclose(out_uniform, out_modified, atol=1e-10)


def test_external_load_gradient_matches_finite_differences():
    layer, elastic, vertices = _setup(external_load="point")
    rng = np.random.default_rng(1)
    R = rng.standard_normal(vertices.shape)

    def loss_np(b_flat):
        layer.reset_warm_start()
        out = layer(torch.tensor(b_flat, dtype=torch.float64))
        return float((out * torch.tensor(R)).sum())

    b0 = elastic.ravel().copy()
    b = torch.tensor(b0, dtype=torch.float64, requires_grad=True)
    layer.reset_warm_start()
    loss = (layer(b) * torch.tensor(R)).sum()
    loss.backward()
    grad = b.grad.numpy()

    # Probe a thickness dof and a membrane-E dof on an interior element
    # (magnitude-scaled FD steps per repo convention).
    num_channels = 5
    for dof in (0 * num_channels + 4, 1 * num_channels + 4, 0 * num_channels + 0):
        h = 1e-6 * max(abs(b0[dof]), 1e-8)
        bp = b0.copy(); bp[dof] += h
        bm = b0.copy(); bm[dof] -= h
        fd = (loss_np(bp) - loss_np(bm)) / (2 * h)
        assert grad[dof] == pytest.approx(fd, rel=2e-3, abs=1e-8), f"dof {dof}"


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
