import numpy as np
import pytest

from pypgo import implicit
from pypgo.mesh import TriMeshData


def test_grid_spec_validation():
    with pytest.raises((RuntimeError, ValueError)):
        implicit.GridSpec([0, 0, 0], [1, 1, 1], resolution=1)


def test_sphere_field_eval_and_bounds():
    sphere = implicit.SphereField([0, 0, 0], 1.0)

    assert sphere.eval([0, 0, 0]) == pytest.approx(-1.0)
    assert sphere.eval([1, 0, 0]) == pytest.approx(0.0, abs=1e-12)
    assert sphere.eval([2, 0, 0]) == pytest.approx(1.0)

    bmin, bmax = sphere.bounds()
    np.testing.assert_allclose(bmin, [-1, -1, -1])
    np.testing.assert_allclose(bmax, [1, 1, 1])


def test_box_field_eval():
    box = implicit.BoxField([0, 0, 0], [1, 2, 3])

    assert box.eval([0, 0, 0]) == pytest.approx(-1.0)
    assert box.eval([1, 0, 0]) == pytest.approx(0.0)
    assert box.eval([1.5, 0, 0]) > 0.0


def test_grid_field_numpy_zerocopy():
    spec = implicit.GridSpec([-1, -1, -1], [1, 1, 1], resolution=3)
    grid = implicit.SphereField([0, 0, 0], 1.0).sample_to_grid(spec)

    values = grid.values
    assert values.shape == (3, 3, 3)
    assert values.dtype == np.float64

    values[0, 0, 0] = 123.0
    assert grid.eval(spec.bmin) == pytest.approx(123.0)


def test_csg_operators_are_lazy_until_sampling():
    s1 = implicit.SphereField([0, 0, 0], 1.0)
    s2 = implicit.SphereField([0.5, 0, 0], 1.0)

    union = s1 | s2
    diff = s1 - s2

    assert isinstance(union, implicit.ImplicitField)
    assert not isinstance(union, implicit.GridField)
    assert union.eval([0, 0, 0]) == pytest.approx(min(s1.eval([0, 0, 0]), s2.eval([0, 0, 0])))
    assert diff.eval([0, 0, 0]) == pytest.approx(max(s1.eval([0, 0, 0]), -s2.eval([0, 0, 0])))

    grid = union.sample_to_grid(implicit.GridSpec([-2, -2, -2], [2, 2, 2], 8))
    assert isinstance(grid, implicit.GridField)
    assert grid.values.shape == (8, 8, 8)


def test_extract_marching_cubes_requires_materialized_grid():
    with pytest.raises(TypeError, match="GridField"):
        implicit.extract_marching_cubes(implicit.SphereField([0, 0, 0], 1.0))


def test_thicken_mesh_surface_returns_mesh_data():
    mesh = implicit.thicken_mesh_surface(
        TriMeshData(
            [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
            [[0, 1, 2]],
        ),
        thickness=0.1,
        resolution=8,
        padding=0.25,
    )
    assert isinstance(mesh, TriMeshData)
