import importlib.util
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = ROOT / "examples" / "experiments" / "tricubic_hermite_fem"
CONVERGENCE = EXPERIMENT / "tet_convergence"
RUN_SERIAL = CONVERGENCE / "run_serial.sh"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_convergence_plan_preregisters_three_result_classes():
    plan = (CONVERGENCE / "PLAN.md").read_text()

    assert "Supports the hypothesis" in plan
    assert "Falsifies/materially weakens it" in plan
    assert "Inconclusive" in plan
    assert "RMS alone never determines" in plan


def test_mesh_tuner_uses_inverse_max_volume_update():
    module = load_module(CONVERGENCE / "generate_meshes.py", "tet_convergence_generate")

    assert module.next_a(1.0, actual_ratio=8.0, target_ratio=4.0) == pytest.approx(2.0)
    assert module.next_a(1.0, actual_ratio=2.0, target_ratio=4.0) == pytest.approx(0.5)
    assert module.relative_ratio_error(4.1, 4.0) == pytest.approx(0.025)


def test_lumped_area_weights_sum_to_surface_area():
    module = load_module(CONVERGENCE / "analyze.py", "tet_convergence_analyze_areas")
    vertices = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    triangles = np.array([[0, 1, 2]], dtype=np.int64)

    weights = module.lumped_vertex_areas(vertices, triangles)

    np.testing.assert_allclose(weights, np.full(3, 1.0 / 3.0))
    assert weights.sum() == pytest.approx(1.0)


def test_interval_decision_has_support_falsify_and_inconclusive_classes():
    module = load_module(CONVERGENCE / "analyze.py", "tet_convergence_analyze_decision")

    small = module.distance_interval(0.1, 0.01)
    large = module.distance_interval(0.3, 0.02)
    overlap = module.distance_interval(0.11, 0.03)

    assert module.classify_smaller(small, large) == "supports"
    assert module.classify_smaller(large, small) == "falsifies"
    assert module.classify_smaller(small, overlap) == "inconclusive"


def test_convergence_output_is_separate_from_canonical_static_output():
    module = load_module(CONVERGENCE / "convergence_common.py", "tet_convergence_common_layout")

    output = module.convergence_output("bunny")

    assert output.name == "tet_convergence"
    assert output.parent.name == "bunny"
    assert output != module.OUTPUT_ROOT / "bunny" / "static"


def test_static_runner_records_optional_thread_override():
    source = (CONVERGENCE / "run_static.py").read_text()

    assert '"--num-threads"' in source
    assert 'SETTINGS["num_threads"] = args.num_threads' in source


def test_serial_runner_orders_bunny_before_dragon_and_never_backgrounds():
    source = RUN_SERIAL.read_text()

    assert "for study in bunny dragon" in source
    assert source.index("run_static.py") < source.index("analyze.py")
    assert "&\n" not in source
