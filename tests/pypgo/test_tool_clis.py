import ast
import json
from pathlib import Path

import pytest
import pypgo as pgo
from pypgo.mesh.volume import ENuMaterial, VegFile, read_veg, write_veg
from pypgo.tools.mesh.surface import cleanup as surface_cleanup_cli
from pypgo.tools.mesh.volume import cubic_mesher as cubic_mesher_cli
from pypgo.tools.mesh.volume import tetgen_mesher as tetgen_mesher_cli
from pypgo.tools.mesh.volume import volume_info as volume_info_cli
from pypgo.tools.sim import cubic_dynamic as cubic_dynamic_cli


def test_mesh_cli_volume_info_prints_summary(tmp_path, capsys):
    veg_path = tmp_path / "box.veg"
    cube = pgo.mesh.cubic_mesher(
        pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 1.0, 1.0)),
        resolution=1,
    )
    write_veg(str(veg_path), VegFile.from_single_material(cube, ENuMaterial()))

    ret = volume_info_cli.main([str(veg_path)])

    assert ret == 0
    out = capsys.readouterr().out
    assert "#vtx:" in out
    assert "#elements:" in out


def test_mesh_cli_cubic_mesher_writes_veg(tmp_path):
    obj_path = tmp_path / "box.obj"
    veg_path = tmp_path / "box.veg"
    pgo.mesh.write_obj(str(obj_path), pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 1.0, 1.0)))

    ret = cubic_mesher_cli.main([
        str(obj_path),
        str(veg_path),
        "--resolution",
        "1",
        "--occupancy",
        "center",
    ])

    assert ret == 0
    veg = read_veg(str(veg_path))
    assert veg.mesh_data.num_elements == 1


def test_mesh_cli_tetgen_mesher_writes_veg(tmp_path):
    obj_path = tmp_path / "box.obj"
    veg_path = tmp_path / "box-tet.veg"
    pgo.mesh.write_obj(str(obj_path), pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 1.0, 1.0)))

    ret = tetgen_mesher_cli.main([
        str(obj_path),
        str(veg_path),
        "--command",
        "pq1.414a0.05",
    ])

    assert ret == 0
    veg = read_veg(str(veg_path))
    assert veg.mesh_data.num_elements > 0


def test_mesh_cli_surface_cleanup_can_merge_close_vertices(tmp_path):
    input_obj = tmp_path / "surface.obj"
    output_obj = tmp_path / "surface-merged.obj"
    report_path = tmp_path / "cleanup.json"
    surface = pgo.mesh.TriMeshData(
        [
            [0.0, 0.0, 0.0],
            [1e-7, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        [[0, 2, 3]],
    )
    pgo.mesh.write_obj(str(input_obj), surface)

    ret = surface_cleanup_cli.main([
        "-i",
        str(input_obj),
        "-o",
        str(output_obj),
        "--json",
        str(report_path),
        "--merge-close-vertices",
        "--eps",
        "1e-6",
    ])

    assert ret == 0
    merged = pgo.mesh.read_obj(str(output_obj))
    report = json.loads(report_path.read_text())
    assert merged.num_vertices == 3
    assert merged.num_elements == 1
    assert report["merge_close_vertices"]["merged_vertices"] == 1


def test_mesh_cli_surface_cleanup_writes_report(tmp_path):
    input_obj = tmp_path / "surface.obj"
    output_obj = tmp_path / "surface-clean.obj"
    report_path = tmp_path / "cleanup.json"
    surface = pgo.mesh.TriMeshData(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3], [0, 0, 1]],
    )
    pgo.mesh.write_obj(str(input_obj), surface)

    ret = surface_cleanup_cli.main([
        "-i",
        str(input_obj),
        "-o",
        str(output_obj),
        "--json",
        str(report_path),
        "--expected-components",
        "1",
    ])

    assert ret == 0
    clean = pgo.mesh.read_obj(str(output_obj))
    report = json.loads(report_path.read_text())
    assert clean.num_elements == 4
    assert report["invalid_triangles_before"] == 1
    assert report["invalid_triangles_after"] == 0
    assert report["cleanup_complete"]


def test_sim_cli_cubic_dynamic_runs_zero_steps(tmp_path):
    veg_path = tmp_path / "box.veg"
    obj_path = tmp_path / "box.obj"
    out_dir = tmp_path / "sim"
    surface = pgo.mesh.create_box(bmin=(0.0, 0.0, 0.0), bmax=(1.0, 1.0, 1.0))
    pgo.mesh.write_obj(str(obj_path), surface)
    cube = pgo.mesh.cubic_mesher(surface, resolution=1)
    write_veg(str(veg_path), VegFile.from_single_material(cube, ENuMaterial()))

    ret = cubic_dynamic_cli.main([
        "--volume",
        str(veg_path),
        "--surface",
        str(obj_path),
        "--output-dir",
        str(out_dir),
        "--num-steps",
        "0",
        "--timestep",
        "0.001",
    ])

    assert ret == 0
    summary = json.loads((out_dir / "summary.json").read_text())
    assert summary["num_frames"] == 0
    assert summary["num_dofs"] == cube.num_vertices * 3


def test_setup_declares_console_scripts():
    setup_py = Path("setup.py").read_text()
    tree = ast.parse(setup_py)
    setup_call = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "setup"
    )
    entry_points = None
    for kw in setup_call.keywords:
        if kw.arg == "entry_points":
            entry_points = ast.literal_eval(kw.value)
            break

    assert entry_points is not None
    assert set(entry_points["console_scripts"]) >= {
        "pypgo-volume-info=pypgo.tools.mesh.volume.volume_info:main",
        "pypgo-cubic-mesher=pypgo.tools.mesh.volume.cubic_mesher:main",
        "pypgo-tetgen-mesher=pypgo.tools.mesh.volume.tetgen_mesher:main",
        "pypgo-ftetwild-mesher=pypgo.tools.mesh.volume.ftetwild_mesher:main",
        "pypgo-surface-quality=pypgo.tools.mesh.surface.quality:main",
        "pypgo-surface-remesh=pypgo.tools.mesh.surface.remesh:main",
        "pypgo-surface-cleanup=pypgo.tools.mesh.surface.cleanup:main",
    }
    assert all(
        not script.startswith("pypgo-volume-ipc=")
        for script in entry_points["console_scripts"]
    )
    assert all(not script.startswith("pypgo-surface-remesher=") for script in entry_points["console_scripts"])
    assert all(not script.startswith("pypgo-surface-smooth=") for script in entry_points["console_scripts"])
    assert all(not script.startswith("pypgo-surface-repair=") for script in entry_points["console_scripts"])
    assert all(not script.startswith("pypgo-surface-simplify=") for script in entry_points["console_scripts"])
    assert all(not script.startswith("pypgo-surface-remove-isolated-vertices=") for script in entry_points["console_scripts"])
    assert all(not script.startswith("pypgo-surface-merge-close-vertices=") for script in entry_points["console_scripts"])
    assert all(not script.startswith("pypgo-stress=") for script in entry_points["console_scripts"])
    assert all(not script.startswith("pypgo-tet-mesher=") for script in entry_points["console_scripts"])


def test_sim_cli_tet_dynamic_runs_from_args(tmp_path):
    from pypgo.tools.sim import tet_dynamic

    assets = Path(__file__).resolve().parents[2] / "examples" / "assets"
    out = tmp_path / "out"
    ret = tet_dynamic.main([
        "--volume", str(assets / "veg" / "tet" / "box.veg"),
        "--surface", str(assets / "obj" / "box.obj"),
        "--output-dir", str(out),
        "--timestep", "0.001",
        "--num-steps", "1",
        "--gravity", "0", "-9.81", "0",
    ])
    assert ret == 0
    assert (out / "summary.json").exists()


def test_sim_cli_config_error_exits_2(tmp_path):
    from pypgo.tools.sim import tet_static

    with pytest.raises(SystemExit) as excinfo:
        tet_static.main(["--output-dir", str(tmp_path)])  # missing meshes
    assert excinfo.value.code == 2


def test_setup_declares_sim_console_scripts():
    setup_text = (Path(__file__).resolve().parents[2] / "setup.py").read_text()
    for name in ("shell-static", "shell-dynamic", "cubic-static",
                 "cubic-dynamic", "tet-static", "tet-dynamic"):
        module = name.replace("-", "_")
        assert f"pypgo-sim-{name}=pypgo.tools.sim.{module}:main" in setup_text
    assert "pypgo-sim-batch=pypgo.tools.sim.batch:main" in setup_text
