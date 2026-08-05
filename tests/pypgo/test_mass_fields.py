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
    return pgo.mesh.volume.VolumeMesh(mesh, material)


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
    return pgo.mesh.volume.VolumeMesh(mesh, material)


def test_tet_mass_matrix_matches_legacy_vega_consistent_mass():
    volume = _unit_tet_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh(volume)
    M_new = pf.TetLinear().mass_matrix(sim_mesh, 2.0).to_dense()
    M_legacy = volume._mass_matrix().to_dense()
    np.testing.assert_allclose(M_new, M_legacy, rtol=1e-12, atol=1e-14)


def test_cubic_mass_matrix_matches_legacy_vega_consistent_mass():
    volume = _single_cube_volume(density=3.0)
    sim_mesh = pgo.fem.SimulationMesh(volume)
    M_new = pf.CubicLinear().mass_matrix(sim_mesh, 3.0).to_dense()
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
    sim_mesh = pgo.fem.SimulationMesh(volume)
    g = np.array([0.0, -9.8, 0.0])
    f = pf.TetLinear().body_force(sim_mesh, g, 2.0)
    tet_volume = 1.0 / 6.0
    total = f.reshape(-1, 3).sum(axis=0)
    np.testing.assert_allclose(total, 2.0 * tet_volume * g, rtol=1e-12)
    # Linear tet: each vertex carries rho*V/4.
    np.testing.assert_allclose(f.reshape(-1, 3), np.tile(2.0 * tet_volume / 4.0 * g, (4, 1)), rtol=1e-12)


def test_volume_constant_velocity_kinetic_energy_is_exact():
    volume = _single_cube_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh(volume)
    M = pf.CubicLinear().mass_matrix(sim_mesh, 2.0).to_dense()
    v = np.array([0.4, -0.2, 0.7])
    qdot = np.tile(v, 8)
    kinetic = 0.5 * qdot @ (M @ qdot)
    assert kinetic == pytest.approx(0.5 * 2.0 * 1.0 * float(v @ v), rel=1e-12)


def test_volume_mesh_exposes_explicit_element_densities():
    volume = _unit_tet_volume(density=7.5)
    assert volume.element_densities == [7.5]
    mesh = pgo.fem.SimulationMesh(volume)
    force = pf.TetLinear().body_force(
        mesh, [0.0, -1.0, 0.0], volume.element_densities
    )
    np.testing.assert_allclose(
        force.reshape(-1, 3).sum(axis=0),
        [0.0, -7.5 / 6.0, 0.0],
        rtol=1e-12,
    )


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
    volume = pgo.mesh.volume.VolumeMesh(mesh, material)
    sim_mesh = pgo.fem.SimulationMesh(volume)
    M_new = pf.TetLinear().mass_matrix(sim_mesh, rho).to_dense()
    M_legacy = volume._mass_matrix().to_dense()
    np.testing.assert_allclose(M_new, M_legacy, rtol=1e-12, atol=1e-14)


def test_explicit_volume_density_validation():
    volume = _unit_tet_volume()
    sim_mesh = pgo.fem.SimulationMesh(volume)
    with pytest.raises((TypeError, ValueError)):
        pf.TetLinear().mass_matrix(sim_mesh, "not a mass field")
    with pytest.raises(TypeError):
        # VolumeMesh is converted explicitly at the FEM boundary.
        pf.TetLinear().mass_matrix(volume, 1.0)
    with pytest.raises(ValueError):
        pf.TetLinear().mass_matrix(sim_mesh, -1.0)
    with pytest.raises(ValueError):
        pf.TetLinear().mass_matrix(sim_mesh, np.array([1.0, 2.0]))


@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_formulations_reject_non_finite_densities(invalid):
    volume = _unit_tet_volume()
    volume_mesh = pgo.fem.SimulationMesh(volume)
    _surface, _vertices, _triangles, shell_mesh = _shell_grid()
    with pytest.raises(ValueError, match="finite"):
        pf.TetLinear().mass_matrix(volume_mesh, invalid)
    with pytest.raises(ValueError, match="finite"):
        pf.KoiterShell().mass_matrix(shell_mesh, invalid)


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
    return surface, vertices, triangles, pf.SimulationMesh(surface)


def test_shell_body_force_matches_manual_lumped_formula():
    _surface, vertices, triangles, sim = _shell_grid()
    g = np.array([0.0, 0.0, -9.81])
    rho_h = 1.0
    f = pf.KoiterShell().body_force(sim, g, rho_h)

    vertex_area = np.zeros(vertices.shape[0])
    for tri in triangles:
        a, b, c = vertices[tri]
        vertex_area[tri] += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 3.0
    manual = (rho_h * vertex_area[:, None] * g).ravel()
    np.testing.assert_allclose(f, manual, rtol=1e-12, atol=1e-15)


def test_shell_body_force_total_weight():
    _surface, _vertices, _triangles, sim = _shell_grid()
    g = np.array([0.0, 0.0, -9.81])
    f = pf.KoiterShell().body_force(sim, g, 1000.0 * 1e-3)
    # Unit square shell: total area 1, rho*h = 1.
    np.testing.assert_allclose(f.reshape(-1, 3).sum(axis=0), 1.0 * g, rtol=1e-12)


def test_shell_mass_matrix_row_sums_are_lumped_vertex_masses():
    _surface, vertices, triangles, sim = _shell_grid()
    M = pf.KoiterShell().mass_matrix(sim, 2.0).to_dense()
    vertex_area = np.zeros(vertices.shape[0])
    for tri in triangles:
        a, b, c = vertices[tri]
        vertex_area[tri] += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 3.0
    np.testing.assert_allclose(np.diag(M).reshape(-1, 3), 2.0 * vertex_area[:, None] * np.ones(3), rtol=1e-12)
    np.testing.assert_allclose(M, np.diag(np.diag(M)), atol=1e-15)


def test_shell_accepts_explicit_elementwise_areal_density():
    _surface, _vertices, triangles, sim = _shell_grid()
    values = np.linspace(1.0, 2.0, triangles.shape[0])
    M = pf.KoiterShell().mass_matrix(sim, values)
    assert M.shape == (3 * sim.num_vertices, 3 * sim.num_vertices)
