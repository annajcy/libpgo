"""Smoke tests: every examples/sim_configs/*.json runs through its CLI."""

import json
from pathlib import Path

import pytest

from pypgo.tools.sim import (
    cubic_dynamic, cubic_static, shell_dynamic,
    tet_dynamic, tet_static,
)

CONFIG_DIR = Path(__file__).resolve().parents[2] / "examples" / "sim_configs"

DYNAMIC_CASES = [
    (tet_dynamic, "tet_dynamic_box_ipc.json"),
    (cubic_dynamic, "cubic_dynamic_box_ipc.json"),
    (tet_dynamic, "tet_dynamic_box_sphere_ipc.json"),
    (cubic_dynamic, "cubic_dynamic_box_sphere_ipc.json"),
    (tet_dynamic, "tet_dynamic_bunny_ipc.json"),
    (cubic_dynamic, "cubic_dynamic_bunny_ipc.json"),
    (tet_dynamic, "tet_dynamic_dragon_ipc.json"),
    (cubic_dynamic, "cubic_dynamic_dragon_ipc.json"),
    (shell_dynamic, "shell_dynamic_ipc_drop.json"),
    (cubic_dynamic, "cubic_dynamic_box_ipc_hermite.json"),
    (cubic_dynamic, "cubic_dynamic_box_sphere_ipc_hermite.json"),
    (cubic_dynamic, "cubic_dynamic_bunny_ipc_hermite.json"),
    (cubic_dynamic, "cubic_dynamic_dragon_ipc_hermite.json"),
]

STATIC_CASES = [
    (tet_static, "tet_static_dragon.json"),
    (tet_static, "tet_static_box_hang.json"),
    (cubic_static, "cubic_static_box_hang.json"),
    (cubic_static, "cubic_static_box_hang_hermite.json"),
]


def test_all_example_configs_are_covered():
    covered = {name for _, name in DYNAMIC_CASES + STATIC_CASES}
    on_disk = {p.name for p in CONFIG_DIR.glob("*.json") if p.name != "batch.json"}
    assert covered == on_disk


@pytest.mark.parametrize("module,config", DYNAMIC_CASES,
                         ids=[c for _, c in DYNAMIC_CASES])
def test_dynamic_example_runs(tmp_path, module, config):
    ret = module.main([
        "--config", str(CONFIG_DIR / config),
        "--output-dir", str(tmp_path),
        "--num-steps", "2",
    ])
    assert ret == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["mode"] == "dynamic"
    assert summary["num_frames"] == 2
    assert all(f["accepted"] for f in summary["frames"])
    surface_dir = tmp_path / "surface"
    if summary.get("num_frames") and (surface_dir).exists():
        assert (surface_dir / "surface0000.obj").exists()


@pytest.mark.parametrize("module,config", STATIC_CASES,
                         ids=[c for _, c in STATIC_CASES])
def test_static_example_runs(tmp_path, module, config):
    ret = module.main([
        "--config", str(CONFIG_DIR / config),
        "--output-dir", str(tmp_path),
        "--solver-max-iterations", "5",
    ])
    assert ret == 0
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["mode"] == "static"
    assert summary["iterations"] >= 1
