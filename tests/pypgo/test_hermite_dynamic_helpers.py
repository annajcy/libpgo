import numpy as np
import pytest

import pypgo as pgo
import pypgo.fem as pf
from tests.pypgo.material_helpers import direct_material
from pypgo.mesh.geometry import BarycentricEmbedding


def _single_cube_volume(*, density=2.0):
    vertices = np.array(
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
    )
    elements = np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64)
    mesh = pgo.mesh.CubicMeshData(vertices, elements)
    material = pgo.mesh.volume.ENuMaterial(density=density, E=1e6, nu=0.45)
    return pgo.mesh.volume.VolumeMesh(mesh, material)


def test_barycentric_embedding_exposes_all_local_corners():
    volume = _single_cube_volume()
    target = np.array([[0.25, 0.5, 0.75]], dtype=np.float64)
    emb = BarycentricEmbedding(target, volume)

    np.testing.assert_array_equal(emb.embedding_indices, np.array([[0, 1, 2, 3, 4, 5, 6, 7]]))
    np.testing.assert_array_equal(emb.embedding_elements, np.array([0]))
    assert emb.embedding_weights.shape == (1, 8)
    assert emb.embedding_weights.sum() == pytest.approx(1.0)


def test_hermite_mass_matrix_has_correct_size_symmetry_and_constant_velocity_energy():
    volume = _single_cube_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh(volume)
    M = pf.CubicTricubicHermite().mass_matrix(sim_mesh, 2.0)

    assert M.shape == (8 * 24, 8 * 24)
    Md = M.to_dense()
    np.testing.assert_allclose(Md, Md.T, atol=1e-12)

    qdot = np.zeros(8 * 24, dtype=np.float64)
    v = np.array([0.4, -0.2, 0.7], dtype=np.float64)
    for vertex in range(8):
        qdot[vertex * 24: vertex * 24 + 3] = v
    kinetic = 0.5 * qdot @ (Md @ qdot)
    expected = 0.5 * 2.0 * 1.0 * float(v @ v)
    assert kinetic == pytest.approx(expected, rel=1e-10, abs=1e-10)


def test_hermite_body_force_has_generalized_derivative_entries_and_correct_total_force():
    volume = _single_cube_volume(density=3.0)
    sim_mesh = pgo.fem.SimulationMesh(volume)
    g = np.array([0.0, -9.8, 0.0], dtype=np.float64)
    f = pf.CubicTricubicHermite().body_force(sim_mesh, g, 3.0)

    assert f.shape == (8 * 24,)
    value_force = np.zeros(3)
    derivative_force_norm = 0.0
    for vertex in range(8):
        value_force += f[vertex * 24: vertex * 24 + 3]
        derivative_force_norm += np.linalg.norm(f[vertex * 24 + 3: vertex * 24 + 24])
    np.testing.assert_allclose(value_force, 3.0 * g, atol=1e-10)
    assert derivative_force_norm > 0.0


def test_hermite_surface_embedding_reproduces_affine_displacement():
    volume = _single_cube_volume()
    points = np.array(
        [[0.25, 0.5, 0.75], [1.0, 0.0, 0.5], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    W = pf.CubicTricubicHermite().surface_embedding_matrix(volume, points)
    assert W.shape == (points.shape[0] * 3, 8 * 24)

    A = np.array([[0.1, 0.2, 0.0], [0.0, -0.1, 0.3], [0.05, 0.0, 0.2]], dtype=np.float64)
    b = np.array([0.3, -0.4, 0.2], dtype=np.float64)
    q = np.zeros(8 * 24, dtype=np.float64)
    vertices = volume.mesh_data.vertices
    for vertex, X in enumerate(vertices):
        base = vertex * 24
        q[base:base + 3] = A @ X + b
        q[base + 3:base + 6] = A @ np.array([1.0, 0.0, 0.0])
        q[base + 6:base + 9] = A @ np.array([0.0, 1.0, 0.0])
        q[base + 9:base + 12] = A @ np.array([0.0, 0.0, 1.0])

    mapped = (W @ q).reshape(-1, 3)
    expected = points @ A.T + b
    np.testing.assert_allclose(mapped, expected, atol=1e-12)


def test_hermite_dynamic_free_fall_uses_24_dofs():
    volume = _single_cube_volume(density=2.0)
    sim_mesh = pgo.fem.SimulationMesh(volume)
    elastic = pf.StableNeoDefinition()
    plastic = pf.VolumetricPlasticityDefinition(dofs=0)
    material = direct_material(
        volume, elastic, plastic,
        None, None,
        np.empty(0), np.empty(0))
    operator = pf.DeformationEnergyOperator(
        material.mesh, material.binding,
        formulation=pf.CubicTricubicHermite(),
    )
    energy = pf.DeformationPotentialEnergy(
        operator, material.state)
    M = pf.CubicTricubicHermite().mass_matrix(sim_mesh, 2.0)
    f = pf.CubicTricubicHermite().body_force(
        sim_mesh, [0.0, -9.8, 0.0], 2.0
    )
    dyn_state = pgo.sim.DynamicState(
        displacement=np.zeros(energy.num_dofs),
        velocity=np.zeros(energy.num_dofs),
        acceleration=np.zeros(energy.num_dofs),
    )
    sim = pgo.sim.DynamicSimulation(mass=M, state=dyn_state, timestep=0.01, energy=energy)
    frame = sim.step(
        external_force=f,
        optimizer=pgo.solver.NewtonOptimizer(
            max_iterations=20,
            termination=pgo.solver.AbsoluteTermination(abs_tolerance=1e-6)),
    )

    assert frame.accepted
    assert sim.num_dofs == 8 * 24
    assert frame.displacement.shape == (8 * 24,)


def test_hermite_mapped_floor_contact_has_hermite_dof_count():
    volume = _single_cube_volume()
    surface = volume.extract_surface_mesh()
    W = pf.CubicTricubicHermite().surface_embedding_matrix(volume, surface.vertices)
    contact_surface = pgo.contact.ContactSurface.embedded(surface.vertices, W)
    floor = pgo.contact.FloorEnergy(
        contact_surface,
        axis="z",
        side="keep_above",
        height=-0.2,
        stiffness=100.0,
    )
    x = np.zeros(volume.num_vertices * 24, dtype=np.float64)

    assert floor.num_dofs == volume.num_vertices * 24
    assert floor.value(x) == pytest.approx(0.0)
