import numpy as np
import pytest

import pypgo as pgo
import pypgo.fem as pf


def _unit_tet_volume(*, density=2.0):
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2, 3]], dtype=np.int64)
    mesh = pgo.mesh.TetMeshData(vertices, elements)
    material = pgo.mesh.volume.ENuMaterial(density=density, E=1e6, nu=0.45)
    return pgo.mesh.volume.VolumeMesh.create_from_single_material(mesh, material)


def _single_cube_volume(*, density=2.0):
    vertices = np.array(
        [
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64)
    mesh = pgo.mesh.CubicMeshData(vertices, elements)
    material = pgo.mesh.volume.ENuMaterial(density=density, E=1e6, nu=0.45)
    return pgo.mesh.volume.VolumeMesh.create_from_single_material(mesh, material)


def test_tet_mass_matrix_matches_legacy_vega_consistent_mass():
    volume = _unit_tet_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    M_new = pf.TetLinear().mass_matrix(sim_mesh, pf.VolumeDensity(2.0)).to_dense()
    M_legacy = volume._mass_matrix().to_dense()
    np.testing.assert_allclose(M_new, M_legacy, rtol=1e-12, atol=1e-14)


def test_cubic_mass_matrix_matches_legacy_vega_consistent_mass():
    volume = _single_cube_volume(density=3.0)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    M_new = pf.CubicLinear().mass_matrix(sim_mesh, pf.VolumeDensity(3.0)).to_dense()
    # The legacy vega cubic mass uses hardcoded approximate constants (~1e-8 relative
    # error). Our Gauss-2^3 quadrature is analytically exact for the trilinear N^TN
    # integrand. Compare against the analytically correct consistent-mass values:
    # for a unit cube with density rho, M[i,j] = rho * integral_{[0,1]^3} N_i N_j dV.
    # For node 0 (corner): M[0,0] = rho/27 = 3/27 = 1/9.
    M_legacy = volume._mass_matrix().to_dense()
    # Ensure we agree with legacy to the precision of legacy's own truncated constants
    # (~7 significant figures), and that our values are more accurate.
    np.testing.assert_allclose(M_new, M_legacy, rtol=1e-6, atol=1e-7)
    # Verify that M_new is analytically correct (density * unit-cube Gauss integral).
    # rho=3, V=1: M[0,0] = 3 * (1/3)^3 = 3/27 = 1/9
    assert M_new[0, 0] == pytest.approx(1.0 / 9.0, rel=1e-12)


def test_tet_body_force_distributes_total_weight():
    volume = _unit_tet_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    g = np.array([0.0, -9.8, 0.0])
    f = pf.TetLinear().body_force(sim_mesh, g, pf.VolumeDensity(2.0))
    tet_volume = 1.0 / 6.0
    total = f.reshape(-1, 3).sum(axis=0)
    np.testing.assert_allclose(total, 2.0 * tet_volume * g, rtol=1e-12)
    # Linear tet: each vertex carries rho*V/4.
    np.testing.assert_allclose(f.reshape(-1, 3), np.tile(2.0 * tet_volume / 4.0 * g, (4, 1)), rtol=1e-12)


def test_volume_constant_velocity_kinetic_energy_is_exact():
    volume = _single_cube_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    M = pf.CubicLinear().mass_matrix(sim_mesh, pf.VolumeDensity(2.0)).to_dense()
    v = np.array([0.4, -0.2, 0.7])
    qdot = np.tile(v, 8)
    kinetic = 0.5 * qdot @ (M @ qdot)
    assert kinetic == pytest.approx(0.5 * 2.0 * 1.0 * float(v @ v), rel=1e-12)


def test_volume_density_reads_region_density():
    volume = _unit_tet_volume(density=7.5)
    field = pf.volume_density(volume)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    f = pf.TetLinear().body_force(sim_mesh, [0.0, -1.0, 0.0], field)
    np.testing.assert_allclose(f.reshape(-1, 3).sum(axis=0), [0.0, -7.5 / 6.0, 0.0], rtol=1e-12)


def test_multi_element_tet_mass_matrix_accumulates_shared_dofs():
    # Two-element tet mesh: 5 vertices, two tets sharing a triangular face (verts 1,2,3).
    # Tet 0 = [0,1,2,3] (volume 1/6), Tet 1 = [1,2,3,4] (volume 1/3).
    # Both tets have positive volume (oriented consistently), so vega accepts them.
    # The shared face means vertices 1,2,3 receive contributions from both elements;
    # duplicate (i,j) triplets in the assembly must be summed — exercising the
    # setFromTriplets accumulation path that single-element tests cannot cover.
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]],
        dtype=np.float64,
    )
    elements = np.array([[0, 1, 2, 3], [1, 2, 3, 4]], dtype=np.int64)
    mesh = pgo.mesh.TetMeshData(vertices, elements)
    rho = 2.0
    material = pgo.mesh.volume.ENuMaterial(density=rho, E=1e6, nu=0.45)
    volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(mesh, material)
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    M_new = pf.TetLinear().mass_matrix(sim_mesh, pf.VolumeDensity(rho)).to_dense()
    M_legacy = volume._mass_matrix().to_dense()
    np.testing.assert_allclose(M_new, M_legacy, rtol=1e-12, atol=1e-14)


def test_volume_mass_field_type_errors():
    volume = _unit_tet_volume()
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    with pytest.raises(TypeError):
        pf.TetLinear().mass_matrix(sim_mesh, "not a mass field")
    with pytest.raises(TypeError):
        # Old call style: VolumeMesh in place of SimulationMesh.
        pf.TetLinear().mass_matrix(volume, pf.VolumeDensity(1.0))
    with pytest.raises(ValueError):
        pf.VolumeDensity(-1.0)
    with pytest.raises(ValueError):
        # Elementwise size mismatch surfaces as ValueError (C++ invalid_argument).
        pf.TetLinear().mass_matrix(sim_mesh, pf.VolumeDensity(np.array([1.0, 2.0])))


def _shell_grid(nx=2, ny=2):
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
    return surface, vertices, triangles, pf.SimulationMesh.create_shell(surface, material)


def test_shell_body_force_matches_manual_lumped_formula():
    _surface, vertices, triangles, sim = _shell_grid()
    g = np.array([0.0, 0.0, -9.81])
    rho_h = 1.0
    f = pf.KoiterShell().body_force(sim, g, pf.ShellArealDensity(rho_h))

    vertex_area = np.zeros(vertices.shape[0])
    for tri in triangles:
        a, b, c = vertices[tri]
        vertex_area[tri] += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 3.0
    manual = (rho_h * vertex_area[:, None] * g).ravel()
    np.testing.assert_allclose(f, manual, rtol=1e-12, atol=1e-15)


def test_shell_body_force_total_weight():
    _surface, _vertices, _triangles, sim = _shell_grid()
    g = np.array([0.0, 0.0, -9.81])
    f = pf.KoiterShell().body_force(sim, g, pf.ShellDensityThickness(density=1000.0, thickness=1e-3))
    # Unit square shell: total area 1, rho*h = 1.
    np.testing.assert_allclose(f.reshape(-1, 3).sum(axis=0), 1.0 * g, rtol=1e-12)


def test_shell_mass_matrix_row_sums_are_lumped_vertex_masses():
    _surface, vertices, triangles, sim = _shell_grid()
    M = pf.KoiterShell().mass_matrix(sim, pf.ShellArealDensity(2.0)).to_dense()
    vertex_area = np.zeros(vertices.shape[0])
    for tri in triangles:
        a, b, c = vertices[tri]
        vertex_area[tri] += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 3.0
    np.testing.assert_allclose(np.diag(M).reshape(-1, 3), 2.0 * vertex_area[:, None] * np.ones(3), rtol=1e-12)
    np.testing.assert_allclose(M, np.diag(np.diag(M)), atol=1e-15)


def _shell_energy(sim, triangles):
    base_row = np.array([2.0e4, 0.35, 1.0e4, 0.25, 1.0e-3], dtype=np.float64)
    elastic = np.tile(base_row, (triangles.shape[0], 1))
    plastic = np.ones((triangles.shape[0], 1), dtype=np.float64)
    return pf.deformation_energy(
        sim,
        elastic=pf.KoiterStVK(),
        elastic_field=pf.ElementwiseField(values=elastic),
        plastic=pf.ShellPlasticity(dofs=1),
        plastic_field=pf.ElementwiseField(values=plastic),
        formulation=pf.KoiterShell(),
        options=pf.DeformationOptions(enforce_spd=False, enable_material_max_step=False),
    )


def test_shell_elastic_thickness_mass_field_reads_live_values():
    _surface, _vertices, triangles, sim = _shell_grid()
    energy = _shell_energy(sim, triangles)
    field = pf.ShellDensityElasticThickness(density=1000.0, parameter_field=energy.elastic_field, channel=4)
    g = np.array([0.0, 0.0, -9.81])
    ks = pf.KoiterShell()

    f0 = ks.body_force(sim, g, field)
    values = energy.elastic_field.values.copy()
    values[:, 4] *= 2.0  # double the thickness
    energy.set_elastic_values(values)
    f1 = ks.body_force(sim, g, field)
    np.testing.assert_allclose(f1, 2.0 * f0, rtol=1e-12)


def test_shell_body_force_parameter_jacobian_matches_differences():
    _surface, _vertices, triangles, sim = _shell_grid()
    energy = _shell_energy(sim, triangles)
    field = pf.ShellDensityElasticThickness(density=1000.0, parameter_field=energy.elastic_field, channel=4)
    g = np.array([0.0, 0.0, -9.81])
    ks = pf.KoiterShell()

    b0 = energy.elastic_field.values.copy()
    f0 = ks.body_force(sim, g, field)
    J = ks.body_force_parameter_jacobian(sim, g, field).to_dense()
    assert J.shape == (sim.num_vertices * 3, b0.size)

    rng = np.random.default_rng(0)
    db = np.zeros_like(b0)
    db[:, 4] = rng.uniform(-0.5, 0.5, size=b0.shape[0]) * b0[:, 4]
    energy.set_elastic_values(b0 + db)
    f1 = ks.body_force(sim, g, field)
    # f_g is linear in h, so the Jacobian is exact even for finite steps.
    np.testing.assert_allclose(f1 - f0, J @ db.ravel(), rtol=1e-10, atol=1e-14)
    energy.set_elastic_values(b0)


def test_body_force_parameter_jacobian_rejects_fixed_mass_fields():
    _surface, _vertices, _triangles, sim = _shell_grid()
    with pytest.raises(ValueError):
        pf.KoiterShell().body_force_parameter_jacobian(
            sim, [0.0, 0.0, -9.81], pf.ShellArealDensity(1.0))


def test_shell_volume_mass_field_cross_domain_type_errors():
    _surface, _vertices, _triangles, sim = _shell_grid()
    with pytest.raises(TypeError):
        pf.KoiterShell().mass_matrix(sim, pf.VolumeDensity(1000.0))
    volume = _unit_tet_volume()
    sim_mesh = pgo.fem.SimulationMesh.create_volumetric(volume)
    with pytest.raises(TypeError):
        pf.TetLinear().mass_matrix(sim_mesh, pf.ShellArealDensity(1.0))
