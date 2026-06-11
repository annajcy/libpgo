"""Tests for pypgo.tools.sim.batch — pypgo-sim-batch orchestrator."""

import json
from pathlib import Path

import pytest

CONFIG_DIR = Path(__file__).resolve().parents[2] / "examples" / "sim_configs"
_VALID_COMMANDS = {
    "tet-static", "tet-dynamic",
    "cubic-static", "cubic-dynamic",
    "shell-static", "shell-dynamic",
}


# ---------------------------------------------------------------------------
# (a) Behavioral: run two real cases with --output-root
# ---------------------------------------------------------------------------

def _write_tmp_batch(tmp_path, cases, jobs=None):
    """Write a batch JSON to tmp_path/batch.json and return its path."""
    payload = {"cases": cases}
    if jobs is not None:
        payload["jobs"] = jobs
    p = tmp_path / "batch.json"
    p.write_text(json.dumps(payload, indent=2))
    return p


def test_batch_two_cases_pass(tmp_path):
    """Run one dynamic + one static real case; assert both pass and outputs exist."""
    from pypgo.tools.sim import batch

    # Use absolute config paths so the batch file location doesn't matter.
    cases = {
        "cubic_box_ipc": {
            "command": "cubic-dynamic",
            "config": str(CONFIG_DIR / "cubic_dynamic_box_ipc.json"),
            "args": ["--num-steps", "2"],
        },
        "tet_static_dragon": {
            "command": "tet-static",
            "config": str(CONFIG_DIR / "tet_static_dragon.json"),
            "args": ["--solver-max-iterations", "3"],
        },
    }
    batch_path = _write_tmp_batch(tmp_path, cases)

    ret = batch.main(["--config", str(batch_path), "--output-root", str(tmp_path)])
    assert ret == 0

    # Per-case summary.json under <root>/<case_name>/
    assert (tmp_path / "cubic_box_ipc" / "summary.json").exists()
    assert (tmp_path / "tet_static_dragon" / "summary.json").exists()

    # batch_summary.json lists both as passed
    bs = json.loads((tmp_path / "batch_summary.json").read_text())
    assert bs["num_passed"] == 2
    assert bs["num_failed"] == 0
    assert bs["cases"]["cubic_box_ipc"]["passed"] is True
    assert bs["cases"]["tet_static_dragon"]["passed"] is True


# ---------------------------------------------------------------------------
# (b) Failure path: nonexistent scene config → exit 2 at validation
# ---------------------------------------------------------------------------

def test_batch_nonexistent_scene_config_exits_2(tmp_path):
    """A case pointing at a missing scene config must fail at validation (exit 2)."""
    from pypgo.tools.sim import batch

    cases = {
        "bad_case": {
            "command": "tet-static",
            "config": str(tmp_path / "does_not_exist.json"),
        }
    }
    batch_path = _write_tmp_batch(tmp_path, cases)

    with pytest.raises(SystemExit) as excinfo:
        batch.main(["--config", str(batch_path)])
    assert excinfo.value.code == 2


# ---------------------------------------------------------------------------
# (c) --list smoke test
# ---------------------------------------------------------------------------

def test_batch_list_returns_0(tmp_path, capsys):
    """--list prints case names and returns 0 without running anything."""
    from pypgo.tools.sim import batch

    cases = {
        "case_alpha": {"command": "tet-static", "config": "/any/path.json"},
        "case_beta": {"command": "shell-dynamic", "config": "/any/other.json"},
    }
    jobs = {"my_job": ["case_alpha"]}
    batch_path = _write_tmp_batch(tmp_path, cases, jobs=jobs)

    # --list must not validate scene config existence
    ret = batch.main(["--config", str(batch_path), "--list"])
    assert ret == 0

    out = capsys.readouterr().out
    assert "case_alpha" in out
    assert "case_beta" in out
    assert "my_job" in out


# ---------------------------------------------------------------------------
# (d) Structural test: shipped examples/sim_configs/batch.json
# ---------------------------------------------------------------------------

def test_shipped_batch_json_structure():
    """Validates the shipped batch.json integrity without running any sim."""
    batch_path = CONFIG_DIR / "batch.json"
    assert batch_path.exists(), "examples/sim_configs/batch.json must exist"

    payload = json.loads(batch_path.read_text())
    cases = payload.get("cases", {})
    jobs = payload.get("jobs", {})

    # Every referenced config exists on disk
    for name, case in cases.items():
        cmd = case.get("command")
        assert cmd in _VALID_COMMANDS, f"case {name!r}: unknown command {cmd!r}"
        cfg_path = Path(case["config"])
        if not cfg_path.is_absolute():
            cfg_path = batch_path.parent / cfg_path
        assert cfg_path.exists(), f"case {name!r}: config {cfg_path} not found"

    # Every job's members reference defined cases
    for job_name, members in jobs.items():
        if members == "all":
            continue
        assert isinstance(members, list)
        for m in members:
            assert m in cases, f"job {job_name!r} references undefined case {m!r}"

    # Every scene JSON (except batch.json itself and README) appears in >= 1 case
    scene_jsons = {
        p.name for p in CONFIG_DIR.glob("*.json") if p.name != "batch.json"
    }
    referenced_configs = set()
    for case in cases.values():
        cfg_path = Path(case["config"])
        if not cfg_path.is_absolute():
            cfg_path = batch_path.parent / cfg_path
        referenced_configs.add(cfg_path.name)
    for scene in scene_jsons:
        assert scene in referenced_configs, (
            f"scene config {scene!r} not covered by any case in batch.json"
        )

    # Spot-check the job names the spec mandates
    assert "all" in jobs
    assert "static" in jobs
    assert "dynamic" in jobs
    assert "squash" in jobs
    assert "drops" in jobs


# ---------------------------------------------------------------------------
# (e) Validation errors → ConfigError → exit 2
# ---------------------------------------------------------------------------

def test_batch_unknown_command_exits_2(tmp_path):
    """An unknown command string in a case must exit 2."""
    from pypgo.tools.sim import batch

    cases = {"bad": {"command": "not-a-command", "config": "/any.json"}}
    batch_path = _write_tmp_batch(tmp_path, cases)

    with pytest.raises(SystemExit) as excinfo:
        batch.main(["--config", str(batch_path)])
    assert excinfo.value.code == 2


def test_batch_unknown_job_exits_2(tmp_path):
    """Requesting a --job not defined in the batch file must exit 2."""
    from pypgo.tools.sim import batch

    cases = {
        "ok_case": {
            "command": "tet-static",
            "config": str(CONFIG_DIR / "tet_static_dragon.json"),
        }
    }
    batch_path = _write_tmp_batch(tmp_path, cases, jobs={"myjob": ["ok_case"]})

    with pytest.raises(SystemExit) as excinfo:
        batch.main(["--config", str(batch_path), "--job", "undefined_job"])
    assert excinfo.value.code == 2


def test_batch_job_undefined_case_member_exits_2(tmp_path):
    """A job listing a case name not defined in 'cases' must exit 2."""
    from pypgo.tools.sim import batch

    cases = {
        "real_case": {
            "command": "tet-static",
            "config": str(CONFIG_DIR / "tet_static_dragon.json"),
        }
    }
    jobs = {"bad_job": ["real_case", "ghost_case"]}
    batch_path = _write_tmp_batch(tmp_path, cases, jobs=jobs)

    with pytest.raises(SystemExit) as excinfo:
        batch.main(["--config", str(batch_path), "--job", "bad_job"])
    assert excinfo.value.code == 2
