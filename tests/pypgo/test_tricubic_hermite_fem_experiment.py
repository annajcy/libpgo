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
EXPERIMENT = ROOT / "examples" / "experiments" / "tricubic_hermite_fem"
MESH = EXPERIMENT / "mesh"
COMMON = EXPERIMENT / "common.py"
MESH_QUALITY = MESH / "mesh_quality.py"
GENERATOR = MESH / "generate_cubic_mesh.py"
RUN_STATIC = EXPERIMENT / "run_static.py"
RUN_DYNAMIC = EXPERIMENT / "run_dynamic.py"
SUMMARIZE = EXPERIMENT / "summarize.py"
TUNE_TET = MESH / "tune_tet_reference.py"
SUBDIVIDE_CUBIC = MESH / "subdivide_cubic_mesh.py"
RUN_EXPERIMENTS = EXPERIMENT / "run_experiments.sh"


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
            "--study", "dragon",
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
    module = load_module(RUN_STATIC, "run_static")
    dragon = module.build_cases(module.STUDIES["dragon"])
    bunny = module.build_cases(module.STUDIES["bunny"])

    assert set(dragon) == set(module.FORMULATION_CASES)
    assert dragon["tet_ref"]["volume"] is None
    assert bunny["tet_ref"]["volume"] is None
    assert dragon["cubic_linear"]["volume"] == dragon["cubic_hermite"]["volume"]
    assert bunny["cubic_linear_x8"]["volume"].name == "bunny-conservative-r15-subdiv2.veg"
    assert bunny["cubic_linear_x27"]["volume"].name == "bunny-conservative-r15-subdiv3.veg"


def test_dynamic_compare_cases_use_shared_study_assets():
    module = load_module(RUN_DYNAMIC, "run_dynamic")
    study = module.STUDIES["bunny"]
    cases = module.build_cases(study)

    assert set(cases) == set(module.FORMULATION_CASES)
    assert study["surface"].is_relative_to(EXPERIMENT)
    assert study["obstacle"].name == "bottom.1.obj"
    assert study["obstacle"].is_relative_to(EXPERIMENT)
    assert cases["tet_ref"]["volume"] is None
    assert cases["cubic_linear"]["volume"] == cases["cubic_hermite"]["volume"]
    assert cases["cubic_linear_x8"]["volume"].name == "bunny-conservative-r15-subdiv2.veg"
    assert cases["cubic_linear_x27"]["volume"].name == "bunny-conservative-r15-subdiv3.veg"
    assert module.SETTINGS["num_steps"] >= 800


@pytest.mark.parametrize("entrypoint", [RUN_STATIC, RUN_DYNAMIC])
def test_experiment_entrypoints_use_one_shot_parallel_runtime_api(entrypoint):
    source = entrypoint.read_text()

    assert "pp.GlobalTbbControl(max_concurrency)" in source
    assert "with _parallelism_scope():" in source
    assert "parallel_control =" not in source
    assert "set_worker_limit" not in source


def test_linux_pipeline_selects_mkl_tbb_before_python_launches():
    source = RUN_EXPERIMENTS.read_text()

    assert "export MKL_THREADING_LAYER=${MKL_THREADING_LAYER:-TBB}" in source
    assert source.index("export MKL_THREADING_LAYER") < source.index("RUN=(conda run")


def test_dynamic_compare_tet_reference_uses_default_sparse_solver():
    module = load_module(RUN_DYNAMIC, "run_dynamic")
    cases = module.build_cases(module.STUDIES["bunny"])

    assert "sparse_solver" not in cases["tet_ref"]


def test_dynamic_compare_trajectory_metrics_reports_impact_and_error():
    module = load_module(SUMMARIZE, "summarize_dynamic_metrics")
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
    module = load_module(RUN_DYNAMIC, "run_dynamic_main")
    calls = []

    def fake_run_case(name, study, cases, output_root, force=False, write_abc=False):
        calls.append(write_abc)
        return {"final_timestep_id": module.SETTINGS["num_steps"]}

    monkeypatch.setattr(module, "run_case", fake_run_case)
    monkeypatch.setattr(module, "summarize_dynamic", lambda *args, **kwargs: None)
    monkeypatch.setattr(module, "_parallelism_scope", module.nullcontext)

    assert module.main(["--cases", "tet_ref", "--output-root", str(tmp_path)]) == 0
    assert calls == [True]


def test_static_compare_metrics_exclude_the_fixed_patch():
    module = load_module(SUMMARIZE, "summarize_static_metrics")
    reference = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 4.0, 0.0]])
    displacement = np.array([[0.2, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 2.0, 0.0]])

    metrics = module.static_displacement_metrics(displacement, reference, np.array([0]))

    assert metrics["free_surface_rel_l2"] == 0.5
    assert metrics["pin_residual_max"] == 0.2
    assert metrics["all_surface_rel_l2"] != metrics["free_surface_rel_l2"]


def test_static_summary_writes_json_csv_and_markdown(monkeypatch, tmp_path):
    module = load_module(SUMMARIZE, "summarize_static_outputs")
    surface_path = tmp_path / "surface.obj"
    patch_path = tmp_path / "fixed.txt"
    surface = pgo.mesh.TriMeshData(
        np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float),
        np.array([[0, 1, 2]], dtype=np.int64),
    )
    pgo.mesh.write_obj(str(surface_path), surface)
    patch_path.write_text("0\n")

    monkeypatch.setitem(
        module.STUDIES,
        "bunny",
        {
            "name": "bunny",
            "title": "Synthetic Bunny",
            "surface": surface_path,
            "fixed": patch_path,
        },
    )
    rest = np.asarray(surface.vertices)
    for name, scale in {
        "tet_ref": 1.0,
        "cubic_linear": 0.5,
        "cubic_hermite": 0.75,
    }.items():
        output = tmp_path / name
        output.mkdir()
        summary = {
            "case": name,
            "formulation": name,
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

    comparison = module.summarize_static("bunny", tmp_path)
    rows = {row["case"]: row for row in comparison["cases"]}
    assert rows["cubic_linear"]["free_surface_rel_l2"] == pytest.approx(0.5)
    assert rows["cubic_hermite"]["free_surface_rel_l2"] == pytest.approx(0.25)
    assert (tmp_path / "comparison.json").exists()
    assert (tmp_path / "comparison.csv").exists()
    assert (tmp_path / "summary.md").exists()


def test_tune_tet_reference_updates_log_space_bracket():
    module = load_module(TUNE_TET, "tune_tet_reference")
    lo, hi = module.update_bracket(1.0, 100.0, 10.0, actual_ratio=5.0, target_ratio=3.0)
    assert (lo, hi) == (10.0, 100.0)
    lo, hi = module.update_bracket(lo, hi, 30.0, actual_ratio=2.0, target_ratio=3.0)
    assert (lo, hi) == (10.0, 30.0)
    assert module.midpoint_a(lo, hi) == pytest.approx((300.0) ** 0.5)


def test_tune_tet_reference_publishes_selected_mesh_and_metadata(tmp_path):
    module = load_module(TUNE_TET, "publish_tet_reference")
    candidate = tmp_path / "candidate.veg"
    metadata = tmp_path / "selected.json"
    candidate.write_bytes(b"selected tet mesh")
    input_signature = {"schema_version": 1, "input": "fingerprint"}
    candidate.with_suffix(".meta.json").write_text(
        json.dumps({"input_signature": input_signature})
    )

    selection = module.publish_selection(
        study="bunny",
        candidate=candidate,
        selection_output=metadata,
        a=2.5e-9,
        target_ratio=5.0,
        actual_ratio=4.99,
        vertices=100,
        elements=300,
        input_signature=input_signature,
    )

    assert json.loads(metadata.read_text()) == selection
    assert selection["a"] == 2.5e-9
    assert selection["actual_dof_ratio"] == 4.99
    assert selection["input_signature"] == input_signature


def test_tune_tet_reference_rejects_out_of_tolerance_selection():
    module = load_module(TUNE_TET, "tet_reference_tolerance")

    with pytest.raises(RuntimeError, match="exceeds tolerance"):
        module.require_tolerance((0.2,), tolerance=0.05)


def test_tet_candidate_cache_requires_matching_input_signature(monkeypatch, tmp_path):
    module = load_module(TUNE_TET, "tet_candidate_cache")
    candidate = tmp_path / "candidate.veg"
    tet = pgo.mesh.TetMeshData(
        np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float),
        np.array([[0, 1, 2, 3]], dtype=np.int64),
    )
    calls = []

    def fake_tet_mesher(*args, **kwargs):
        calls.append(kwargs["config"]["command"])
        return tet

    monkeypatch.setattr(module.pgo.mesh, "tet_mesher", fake_tet_mesher)
    signature = {"input": "first"}
    module.generate_or_read(object(), tet.volume, candidate, 1e-6, signature)
    module.generate_or_read(object(), tet.volume, candidate, 1e-6, signature)
    module.generate_or_read(object(), tet.volume, candidate, 1e-6, {"input": "changed"})

    assert calls == ["pq1.414a9.9999999999999995e-07"] * 2


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

    refined_x27 = module.subdivide_cubic_data(cube, factor=3)
    assert refined_x27.num_vertices == 64
    assert refined_x27.num_elements == 27
    assert refined_x27.volume == pytest.approx(cube.volume)


def test_common_output_layout_and_stable_asset_ids():
    module = load_module(COMMON, "tricubic_common")

    assert module.output_root("bunny", "static").as_posix().endswith(
        "examples/outputs/tricubic_hermite_fem/bunny/static"
    )
    assert module.output_root("dragon", "dynamic").as_posix().endswith(
        "examples/outputs/tricubic_hermite_fem/dragon/dynamic"
    )
    assert module.asset_id(module.STUDIES["bunny"]["surface"]) == "assets/obj/bunny.obj"
    assert "tet_a" not in module.STUDIES["bunny"]
    assert module.tet_reference_selection_path(module.STUDIES["dragon"]).name == (
        "dragon-conservative-r15-tet-reference.json"
    )


def test_common_resolves_tet_candidate_from_selection_manifest(monkeypatch, tmp_path):
    module = load_module(COMMON, "tricubic_common_selection")
    assets = tmp_path / "assets"
    tet_dir = assets / "veg" / "tet"
    tet_dir.mkdir(parents=True)
    candidate = tet_dir / "bunny-conservative-r15-tet-a2.5e-9.veg"
    candidate.write_bytes(b"tet")
    candidate_metadata = candidate.with_suffix(".meta.json")
    input_signature = {"schema_version": 1, "input": "fingerprint"}
    candidate_metadata.write_text(json.dumps({"input_signature": input_signature}))
    selection_path = tet_dir / "bunny-conservative-r15-tet-reference.json"
    selection_path.write_text(json.dumps({
        "study": "bunny",
        "candidate_mesh": candidate.relative_to(tmp_path).as_posix(),
        "candidate_metadata": candidate_metadata.relative_to(tmp_path).as_posix(),
        "input_signature": input_signature,
    }))
    monkeypatch.setattr(module, "EXPERIMENT_DIR", tmp_path)
    monkeypatch.setattr(module, "ASSETS", assets)

    cases = module.build_cases({"name": "bunny", "prefix": "bunny-conservative-r15"})

    assert cases["tet_ref"]["volume"] == candidate


def test_dynamic_all_rejects_resumed_wall_time(monkeypatch, tmp_path):
    module = load_module(SUMMARIZE, "summarize_wall_time_gate")
    monkeypatch.setattr(
        module,
        "summarize",
        lambda *args, **kwargs: {"complete": True, "wall_time_comparable": False},
    )

    assert module.main([
        "--study", "bunny", "--mode", "dynamic", "--output-root", str(tmp_path)
    ]) == 2


@pytest.mark.skipif(
    sys.platform == "win32",
    reason="run_experiments.sh is a POSIX shell entrypoint",
)
def test_run_experiments_help_does_not_start_the_pipeline():
    result = subprocess.run(
        [str(RUN_EXPERIMENTS), "--help"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "Usage: run_experiments.sh" in result.stdout
    assert "Step 1/13" not in result.stdout
