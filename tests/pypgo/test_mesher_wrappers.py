import numpy as np
import pytest

import pypgo as pgo
from pypgo.mesh import cubic_mesher, has_tetwild, tet_mesher


def test_cubic_mesher_voxelizes_closed_surface():
    surface = pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 1.0, 1.0))

    cubic = cubic_mesher(surface, resolution=2)

    assert isinstance(cubic, pgo.mesh.CubicMeshData)
    assert cubic.num_vertices == 27
    assert cubic.num_elements == 8
    assert np.allclose(cubic.bbox[0], [0.0, 0.0, 0.0])
    assert np.allclose(cubic.bbox[1], [1.0, 1.0, 1.0])


def test_tet_mesher_tetgen_tetrahedralizes_closed_surface():
    surface = pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 1.0, 1.0))

    tet = tet_mesher(surface, backend="tetgen", config={"command": "pq1.414a0.05"})

    assert isinstance(tet, pgo.mesh.TetMeshData)
    assert tet.num_vertices > 0
    assert tet.num_elements > 0
    assert np.all(tet.elements >= 0)
    assert np.all(tet.elements < tet.num_vertices)


def test_tet_mesher_tetwild_reports_unavailable_when_disabled():
    surface = pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 1.0, 1.0))

    if has_tetwild():
        pytest.skip("tetwild is enabled in this build")

    with pytest.raises(RuntimeError, match="tetwild backend.*available"):
        tet_mesher(surface, backend="tetwild", config={"lr": 0.2})
