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
