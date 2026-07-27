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
    tet_volume = pgo.mesh.volume.VolumeMesh(tet, pgo.mesh.volume.ENuMaterial())
    tet_sim = pgo.fem.SimulationImportResult(tet_volume)
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
    cubic_volume = pgo.mesh.volume.VolumeMesh(cube, pgo.mesh.volume.ENuMaterial())
    cubic_sim = pgo.fem.SimulationImportResult(cubic_volume)
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
    volume = pgo.mesh.volume.VolumeMesh(
        tet,
        pgo.mesh.volume.MooneyRivlinMaterial(mu01=0.5, mu10=0.3, v1=0.1),
    )
    sim = pgo.fem.SimulationImportResult(volume)
    assert sim.mesh_type == "tet"
    assert sim.num_elements == 1


def test_simulation_mesh_create_shell():
    surface = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    mesh = pgo.fem.SimulationMesh(surface)
    assert mesh.mesh_type == "shell"
    assert mesh.num_vertices == 3
    assert mesh.num_elements == 1
    assert mesh.num_element_vertices == 6


def test_volume_mesh_carries_multiple_material_types():
    """VolumeMesh can carry ENu, Mooney-Rivlin, and Orthotropic materials."""
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )

    enu_vol = pgo.mesh.volume.VolumeMesh(tet, pgo.mesh.volume.ENuMaterial(E=1e6, nu=0.45))
    assert isinstance(enu_vol.material, pgo.mesh.volume.ENuMaterial)
    assert enu_vol.material.E == 1e6

    mr_vol = pgo.mesh.volume.VolumeMesh(
        tet, pgo.mesh.volume.MooneyRivlinMaterial(mu01=0.5, mu10=0.3, v1=0.1))
    assert isinstance(mr_vol.material, pgo.mesh.volume.MooneyRivlinMaterial)
    assert mr_vol.material.mu01 == 0.5

    rotation = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    orthotropic = pgo.mesh.volume.OrthotropicMaterial(
        E1=3e6,
        E2=2e6,
        E3=1e6,
        nu12=0.2,
        nu23=0.25,
        nu31=0.3,
        G12=0.7e6,
        G23=0.6e6,
        G31=0.5e6,
        rotation=rotation,
    )
    ortho_vol = pgo.mesh.volume.VolumeMesh(tet, orthotropic)
    assert ortho_vol.material == orthotropic
    asset = pgo.fem.SimulationImportResult(ortho_vol)
    record = asset.material_catalog.materials[0]
    assert record.family == "orthotropic"
    assert np.array_equal(
        np.asarray(record.properties["rotation"]).reshape(3, 3),
        rotation,
    )


def test_cubic_mesh_type_is_topology_metadata():
    """SimulationImportResult.mesh_type == \"cubic\" is topology metadata (8-vertex
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
    cubic_volume = pgo.mesh.volume.VolumeMesh(cube, pgo.mesh.volume.ENuMaterial())
    cubic_sim = pgo.fem.SimulationImportResult(cubic_volume)
    assert cubic_sim.mesh_type == "cubic"
    assert cubic_sim.num_element_vertices == 8


def test_simulation_mesh_can_be_reused():
    """A SimulationImportResult can be queried multiple times without being consumed."""
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    volume = pgo.mesh.volume.VolumeMesh(tet, pgo.mesh.volume.ENuMaterial())
    sim = pgo.fem.SimulationImportResult(volume)

    # Multiple queries on the same mesh must work.
    assert sim.mesh_type == "tet"
    assert sim.num_vertices == 4
    assert sim.num_elements == 1
    # Second round of queries must return the same values.
    assert sim.mesh_type == "tet"
    assert sim.num_vertices == 4

    # Creating a second SimulationImportResult from the same VolumeMesh must work.
    sim2 = pgo.fem.SimulationImportResult(volume)
    assert sim2.mesh_type == "tet"
    assert sim2.num_vertices == 4
    # The first mesh must still be usable.
    assert sim.mesh_type == "tet"


def test_legacy_mesh_and_asset_factories_are_removed():
    assert not hasattr(pgo.mesh.volume.VolumeMesh, "create_from_single_material")
    assert not hasattr(pgo.mesh.volume.VolumeMesh, "from_veg_file")
    assert not hasattr(pgo.fem.SimulationImportResult, "create_volumetric")
    assert not hasattr(pgo.fem.SimulationImportResult, "create_shell")


def test_import_result_contains_but_is_not_a_simulation_mesh():
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
             [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    imported = pgo.fem.SimulationImportResult(
        pgo.mesh.volume.VolumeMesh(tet, pgo.mesh.volume.ENuMaterial())
    )

    assert isinstance(imported.mesh, pgo.fem.SimulationMesh)
    assert not isinstance(imported, pgo.fem.SimulationMesh)
    assert imported.material_catalog.materials[0].family == "enu"
