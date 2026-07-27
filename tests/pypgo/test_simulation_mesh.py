import numpy as np
import pytest

import pypgo as pgo


def test_simulation_mesh_create_volumetric_for_tet_and_cubic():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    tet_volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(tet, pgo.mesh.volume.ENuMaterial())
    tet_sim = pgo.fem.SimulationAsset.create_volumetric(tet_volume)
    assert tet_sim.mesh_type == "tet"
    assert tet_sim.num_vertices == 4
    assert tet_sim.num_elements == 1
    assert tet_sim.num_element_vertices == 4

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
    cubic_volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(cube, pgo.mesh.volume.ENuMaterial())
    cubic_sim = pgo.fem.SimulationAsset.create_volumetric(cubic_volume)
    assert cubic_sim.mesh_type == "cubic"
    assert cubic_sim.num_vertices == 8
    assert cubic_sim.num_elements == 1
    assert cubic_sim.num_element_vertices == 8


def test_simulation_mesh_create_volumetric_supports_mooney_rivlin_material():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(
        tet,
        pgo.mesh.volume.MooneyRivlinMaterial(mu01=0.5, mu10=0.3, v1=0.1),
    )
    sim = pgo.fem.SimulationAsset.create_volumetric(volume)
    assert sim.mesh_type == "tet"
    assert sim.num_elements == 1


def test_simulation_mesh_create_shell():
    surface = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    material = pgo.fem.KoiterStVKShellMaterial(
        "cloth", thickness=0.01, E_membrane=2e6, nu_membrane=0.35)
    sim = pgo.fem.SimulationAsset.create_shell(surface, material)
    assert sim.mesh_type == "shell"
    assert sim.num_vertices == 3
    assert sim.num_elements == 1
    assert sim.num_element_vertices == 6


def test_volume_mesh_carries_multiple_material_types():
    """VolumeMesh can carry ENu, MooneyRivlinDefinition, and Orthotropic materials."""
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )

    enu_vol = pgo.mesh.volume.VolumeMesh.create_from_single_material(tet, pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45))
    assert isinstance(enu_vol.material, pgo.mesh.volume.ENuMaterial)
    assert enu_vol.material.E == 1e6

    mr_vol = pgo.mesh.volume.VolumeMesh.create_from_single_material(
        tet, pgo.mesh.volume.MooneyRivlinMaterial(mu01=0.5, mu10=0.3, v1=0.1))
    assert isinstance(mr_vol.material, pgo.mesh.volume.MooneyRivlinMaterial)
    assert mr_vol.material.mu01 == 0.5


def test_cubic_mesh_type_is_topology_metadata():
    """SimulationAsset.mesh_type == \"cubic\" is topology metadata (8-vertex
    hexahedral cell), not formulation metadata. The hex formulation
    (trilinear, future tricubic Hermite) is a separate concept that will
    be introduced in later refactoring tasks."""
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
    cubic_volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(cube, pgo.mesh.volume.ENuMaterial())
    cubic_sim = pgo.fem.SimulationAsset.create_volumetric(cubic_volume)
    assert cubic_sim.mesh_type == "cubic"
    assert cubic_sim.num_element_vertices == 8


def test_simulation_mesh_can_be_reused():
    """A SimulationAsset can be queried multiple times without being consumed."""
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.volume.VolumeMesh.create_from_single_material(tet, pgo.mesh.volume.ENuMaterial())
    sim = pgo.fem.SimulationAsset.create_volumetric(volume)

    # Multiple queries on the same mesh must work.
    assert sim.mesh_type == "tet"
    assert sim.num_vertices == 4
    assert sim.num_elements == 1
    # Second round of queries must return the same values.
    assert sim.mesh_type == "tet"
    assert sim.num_vertices == 4

    # Creating a second SimulationAsset from the same VolumeMesh must work.
    sim2 = pgo.fem.SimulationAsset.create_volumetric(volume)
    assert sim2.mesh_type == "tet"
    assert sim2.num_vertices == 4
    # The first mesh must still be usable.
    assert sim.mesh_type == "tet"
