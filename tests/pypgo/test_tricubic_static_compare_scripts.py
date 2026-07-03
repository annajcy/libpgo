import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pypgo as pgo
import pytest
from pypgo.mesh.volume import ENuMaterial, VegFile, read_veg, write_veg


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "examples" / "scripts" / "tricubic-hermit-static-compare"
MESH_QUALITY = SCRIPTS / "mesh_quality.py"
GENERATOR = SCRIPTS / "generate_cubic_mesh.py"
STATIC_COMPARE = SCRIPTS / "static_compare.py"
TUNE_TET = SCRIPTS / "tune_tet_reference.py"
SUBDIVIDE_CUBIC = SCRIPTS / "subdivide_cubic_mesh.py"
DYNAMIC_SCRIPTS = ROOT / "examples" / "scripts" / "tricubic-hermit-dynamic-compare"
DYNAMIC_COMPARE = DYNAMIC_SCRIPTS / "dynamic_compare.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_box_assets(tmp_path):
    surface_path = tmp_path / "box.obj"
    tet_path = tmp_path / "tet.veg"
    cubic_path = tmp_path / "cube.veg"
    material = ENuMaterial(E=1e6, nu=0.45, density=1000.0)
    pgo.mesh.write_obj(str(surface_path), pgo.mesh.create_box(bmin=(0, 0, 0), bmax=(1, 1, 1)))
    tet = pgo.mesh.TetMeshData(
        np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    cube = pgo.mesh.CubicMeshData(
        np.array([
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 1],
            [1, 1, 1],
            [0, 1, 1],
        ], dtype=float),
        np.array([[0, 1, 2, 3, 4, 5, 6, 7]], dtype=np.int64),
    )
    write_veg(str(tet_path), VegFile.from_single_material(tet, material))
    write_veg(str(cubic_path), VegFile.from_single_material(cube, material))
    return surface_path, tet_path, cubic_path


def test_mesh_quality_script_reports_surface_and_volume_ratios(tmp_path):
    surface_path, tet_path, cubic_path = write_box_assets(tmp_path)
    out = tmp_path / "quality.json"

    subprocess.run(
        [
            sys.executable,
            str(MESH_QUALITY),
            "--surface", str(surface_path),
            "--tet", str(tet_path),
            "--cubic", str(cubic_path),
            "--json", str(out),
        ],
        cwd=ROOT,
        check=True,
    )

    data = json.loads(out.read_text())
    assert data["surface"]["is_clean"]
    assert data["tet"]["negative_orientation_count"] == 0
    assert data["tet"]["center_of_mass"] == [0.25, 0.25, 0.25]
    assert data["cubic"]["linear_dofs"] == data["cubic"]["num_vertices"] * 3
    assert data["cubic"]["hermite_dofs"] == data["cubic"]["num_vertices"] * 24
    assert data["ratios"]["cubic_volume_over_tet_volume"] == pytest.approx(6.0)


def test_cubic_mesh_generator_writes_boundary_surface(tmp_path):
    surface_path = tmp_path / "box.obj"
    cubic_path = tmp_path / "box-conservative.veg"
    boundary_path = tmp_path / "box-conservative-surface.obj"
    pgo.mesh.write_obj(str(surface_path), pgo.mesh.create_box(bmin=(0, 0, 0), bmax=(1, 1, 1)))

    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            "--surface", str(surface_path),
            "--case", "dragon",
            "--resolution", "3",
            "--output", str(cubic_path),
            "--boundary-output", str(boundary_path),
        ],
        cwd=ROOT,
        check=True,
    )

    veg = read_veg(str(cubic_path))
    boundary = pgo.mesh.read_obj(str(boundary_path))
    assert veg.mesh_data.num_elements > 0
    assert boundary.num_elements > 0
    assert pgo.mesh.filter_mesh_components(veg.mesh_data, keep_largest=1).num_elements == veg.mesh_data.num_elements


def test_static_compare_cases_are_case_based():
    module = load_module(STATIC_COMPARE, "static_compare")
    dragon = module.build_cases(module.STUDIES["dragon"])
    bunny = module.build_cases(module.STUDIES["bunny"])

    assert set(dragon) == set(module.FORMULATION_CASES)
    assert dragon["tet_ref"]["volume"].name == "dragon-conservative-r15-tet-a1.47683e-7.veg"
    assert bunny["tet_ref"]["volume"].name == "bunny-conservative-r15-tet-a2.89036e-9.veg"
    assert dragon["cubic_linear"]["volume"] == dragon["cubic_hermite"]["volume"]
    assert bunny["cubic_linear_x8"]["volume"].name == "bunny-conservative-r15-subdiv2.veg"


def test_dynamic_compare_cases_use_local_bunny_drop_assets():
    module = load_module(DYNAMIC_COMPARE, "dynamic_compare")
    cases = module.build_cases(module.STUDY)

    assert set(cases) == set(module.FORMULATION_CASES)
    assert module.STUDY["surface"].is_relative_to(DYNAMIC_SCRIPTS)
    assert module.STUDY["obstacle"].name == "bottom.1.obj"
    assert module.STUDY["obstacle"].is_relative_to(DYNAMIC_SCRIPTS)
    assert cases["tet_ref"]["volume"].name == "bunny-conservative-r15-tet-a2.89036e-9.veg"
    assert cases["cubic_linear"]["volume"] == cases["cubic_hermite"]["volume"]
    assert cases["cubic_linear_x8"]["volume"].name == "bunny-conservative-r15-subdiv2.veg"
    assert module.SETTINGS["num_steps"] >= 800


def test_dynamic_compare_tet_reference_uses_default_sparse_solver():
    module = load_module(DYNAMIC_COMPARE, "dynamic_compare")
    cases = module.build_cases(module.STUDY)

    assert "sparse_solver" not in cases["tet_ref"]


def test_dynamic_compare_trajectory_metrics_reports_impact_and_error():
    module = load_module(DYNAMIC_COMPARE, "dynamic_compare")
    rest = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    reference_positions = {
        10: rest + np.array([[0.0, -0.10, 0.0], [0.0, -0.10, 0.0]]),
        20: rest + np.array([[0.0, -0.20, 0.0], [0.0, -0.20, 0.0]]),
    }
    positions = {
        10: rest + np.array([[0.0, -0.05, 0.0], [0.0, -0.10, 0.0]]),
        20: rest + np.array([[0.0, -0.10, 0.0], [0.0, -0.20, 0.0]]),
    }

    metrics = module.trajectory_metrics(positions, reference_positions, rest, obstacle_top_y=0.0)

    assert metrics["matched_frames"] == 2
    assert metrics["impact_frame"] == 10
    assert metrics["impact_time"] == pytest.approx(0.01)
    assert metrics["trajectory_rel_l2_max"] >= metrics["trajectory_rel_l2_mean"] > 0.0
    assert metrics["final_surface_error_p95"] > 0.0


def test_dynamic_compare_writes_abc_by_default(monkeypatch, tmp_path):
    module = load_module(DYNAMIC_COMPARE, "dynamic_compare")
    calls = []

    def fake_run_case(name, study, cases, output_root, force=False, write_abc=False):
        calls.append(write_abc)
        return {"final_timestep_id": module.SETTINGS["num_steps"]}

    monkeypatch.setattr(module, "run_case", fake_run_case)
    monkeypatch.setattr(module, "analyze", lambda *args, **kwargs: None)

    assert module.main(["--cases", "tet_ref", "--output-root", str(tmp_path)]) == 0
    assert calls == [True]


def test_static_compare_metrics_exclude_the_fixed_patch():
    module = load_module(STATIC_COMPARE, "static_compare")
    reference = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 4.0, 0.0]])
    displacement = np.array([[0.2, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 2.0, 0.0]])

    metrics = module.displacement_metrics(displacement, reference, np.array([0]))

    assert metrics["free_surface_rel_l2"] == 0.5
    assert metrics["pin_residual_max"] == 0.2
    assert metrics["all_surface_rel_l2"] != metrics["free_surface_rel_l2"]


def test_static_compare_writes_json_and_csv(tmp_path):
    module = load_module(STATIC_COMPARE, "static_compare")
    surface_path = tmp_path / "surface.obj"
    patch_path = tmp_path / "fixed.txt"
    surface = pgo.mesh.TriMeshData(
        np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    pgo.mesh.write_obj(str(surface_path), surface)
    patch_path.write_text("0\n")

    study = {"surface": surface_path, "patch": patch_path}
    cases = {
        "tet_ref": {"volume": tmp_path / "tet.veg", "formulation": "tet-linear"},
        "cubic_linear": {"volume": tmp_path / "cubic.veg", "formulation": "cubic-linear"},
        "cubic_hermite": {"volume": tmp_path / "cubic.veg", "formulation": "cubic-tricubic-hermite"},
    }
    rest = np.asarray(surface.vertices)
    for name, scale in {"tet_ref": 1.0, "cubic_linear": 0.5, "cubic_hermite": 0.75}.items():
        output = tmp_path / name
        output.mkdir()
        summary = {
            **module._signature(name, cases),
            "converged": True,
            "num_dofs": 1,
            "iterations": 1,
            "wall_seconds": 1.0,
            "volume_ratio": 1.0,
        }
        (output / "summary.json").write_text(json.dumps(summary))
        displacement = np.zeros_like(rest)
        displacement[:, 1] = -0.01 * scale
        displacement[0] = 0.0
        pgo.mesh.write_obj(str(output / "final_surface.obj"), pgo.mesh.TriMeshData(rest + displacement, surface.elements))

    comparison = module.analyze(study, cases, tmp_path)
    rows = {row["case"]: row for row in comparison["cases"]}
    assert rows["cubic_linear"]["free_surface_rel_l2"] == pytest.approx(0.5)
    assert rows["cubic_hermite"]["free_surface_rel_l2"] == pytest.approx(0.25)
    assert (tmp_path / "comparison.json").exists()
    assert (tmp_path / "comparison.csv").exists()


def test_tune_tet_reference_updates_log_space_bracket():
    module = load_module(TUNE_TET, "tune_tet_reference")
    lo, hi = module.update_bracket(1.0, 100.0, 10.0, actual_ratio=5.0, target_ratio=3.0)
    assert (lo, hi) == (10.0, 100.0)
    lo, hi = module.update_bracket(lo, hi, 30.0, actual_ratio=2.0, target_ratio=3.0)
    assert (lo, hi) == (10.0, 30.0)
    assert module.midpoint_a(lo, hi) == pytest.approx((300.0) ** 0.5)


def test_subdivide_cubic_mesh_splits_one_cube_into_eight():
    module = load_module(SUBDIVIDE_CUBIC, "subdivide_cubic_mesh")
    cube = pgo.mesh.CubicMeshData(
        np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ]),
        np.array([[0, 1, 2, 3, 4, 5, 6, 7]]),
    )

    refined = module.subdivide_cubic_data(cube)

    assert refined.num_vertices == 27
    assert refined.num_elements == 8
    assert refined.volume == pytest.approx(cube.volume)
    np.testing.assert_allclose(refined.vertices.min(axis=0), cube.vertices.min(axis=0))
    np.testing.assert_allclose(refined.vertices.max(axis=0), cube.vertices.max(axis=0))
