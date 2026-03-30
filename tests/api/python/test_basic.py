import numpy as np
from pathlib import Path

import pypgo


def test_create_tetmeshgeo_roundtrip():
    vertices = np.array(
        [
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ],
        dtype=np.float32,
    )
    tets = np.array([0, 1, 2, 3], dtype=np.int32)

    tetmesh = pypgo.create_tetmeshgeo(vertices, tets)
    assert pypgo.tetmeshgeo_get_num_vertices(tetmesh) == 4
    assert pypgo.tetmeshgeo_get_num_tets(tetmesh) == 1

    out_vertices = pypgo.tetmeshgeo_get_vertices(tetmesh)
    out_tets = pypgo.tetmeshgeo_get_tets(tetmesh)

    np.testing.assert_allclose(out_vertices, vertices)
    np.testing.assert_array_equal(out_tets, tets)

    pypgo.destroy_tetmeshgeo(tetmesh)


def test_animation_entrypoint_is_available():
    assert hasattr(pypgo, "convert_animation_to_abc")


def test_run_sim_from_config_accepts_deterministic_argument(tmp_path: Path):
    missing_config = tmp_path / "missing.json"

    assert pypgo.run_sim_from_config(str(missing_config)) == 0
    assert pypgo.run_sim_from_config(str(missing_config), deterministic=None) == 0
    assert pypgo.run_sim_from_config(str(missing_config), deterministic=True) == 0
    assert pypgo.run_sim_from_config(str(missing_config), deterministic=False) == 0
