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
    tet_volume = pgo.mesh.veg.VolumeMesh(tet, pgo.mesh.veg.ENuMaterial())
    tet_sim = pgo.sim.SimulationMesh.create_volumetric(tet_volume)
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
    cubic_volume = pgo.mesh.veg.VolumeMesh(cube, pgo.mesh.veg.ENuMaterial())
    cubic_sim = pgo.sim.SimulationMesh.create_volumetric(cubic_volume)
    assert cubic_sim.mesh_type == "cubic"
    assert cubic_sim.num_vertices == 8
    assert cubic_sim.num_elements == 1
    assert cubic_sim.num_element_vertices == 8


def test_simulation_mesh_create_volumetric_rejects_non_enu_material():
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
    volume = pgo.mesh.veg.VolumeMesh(tet, pgo.mesh.veg.MooneyRivlinMaterial(mu01=1.0))
    with pytest.raises(RuntimeError, match="only ENuMaterial"):
        pgo.sim.SimulationMesh.create_volumetric(volume)


def test_simulation_mesh_create_shell():
    surface = pgo.mesh.TriMeshData(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    material = pgo.sim.KoiterStVKShellMaterial(
        "cloth", thickness=0.01, E_membrane=2e6, nu_membrane=0.35)
    sim = pgo.sim.SimulationMesh.create_shell(surface, material)
    assert sim.mesh_type == "shell"
    assert sim.num_vertices == 3
    assert sim.num_elements == 1
    assert sim.num_element_vertices == 6


def test_volume_mesh_carries_multiple_material_types():
    """VolumeMesh can carry ENu, MooneyRivlin, and Orthotropic materials
    before simulation conversion. The simulation mesh factory currently
    only accepts ENu (see test_simulation_mesh_create_volumetric_rejects_non_enu_material),
    but the VolumeMesh layer preserves all material types for future
    material/law refactoring."""
    tet = pgo.mesh.TetMeshData(
        np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )

    enu_vol = pgo.mesh.veg.VolumeMesh(tet, pgo.mesh.veg.ENuMaterial(E=1e6, nu=0.45))
    assert isinstance(enu_vol.material, pgo.mesh.veg.ENuMaterial)
    assert enu_vol.material.E == 1e6

    mr_vol = pgo.mesh.veg.VolumeMesh(
        tet, pgo.mesh.veg.MooneyRivlinMaterial(mu01=0.5, mu10=0.3, v1=0.1))
    assert isinstance(mr_vol.material, pgo.mesh.veg.MooneyRivlinMaterial)
    assert mr_vol.material.mu01 == 0.5

    ortho_vol = pgo.mesh.veg.VolumeMesh(
        tet,
        pgo.mesh.veg.OrthotropicMaterial(
            E1=1e6, E2=1e6, E3=1e6,
            nu12=0.3, nu23=0.3, nu31=0.3,
            G12=0.4e6, G23=0.4e6, G31=0.4e6,
        ),
    )
    assert isinstance(ortho_vol.material, pgo.mesh.veg.OrthotropicMaterial)
    assert ortho_vol.material.E1 == 1e6
    assert ortho_vol.material.G12 == 0.4e6


def test_cubic_mesh_type_is_topology_metadata():
    """SimulationMesh.mesh_type == \"cubic\" is topology metadata (8-vertex
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
    cubic_volume = pgo.mesh.veg.VolumeMesh(cube, pgo.mesh.veg.ENuMaterial())
    cubic_sim = pgo.sim.SimulationMesh.create_volumetric(cubic_volume)
    assert cubic_sim.mesh_type == "cubic"
    assert cubic_sim.num_element_vertices == 8
