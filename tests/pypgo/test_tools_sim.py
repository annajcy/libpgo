from pathlib import Path

from pypgo.tools.sim.volume_ipc import run_volume_ipc

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "legacy" / "cubic" / "box"
_VEG = str(FIXTURE_ROOT / "box.veg")
_OBJ = str(FIXTURE_ROOT / "box.obj")


def test_run_volume_ipc_runs_and_writes_summary(tmp_path):
    out = str(tmp_path / "out")
    n = run_volume_ipc(
        veg_path=_VEG,
        surface_path=_OBJ,
        output_dir=out,
        timestep=0.001,
        num_steps=2,
        gravity=(0.0, -9.81, 0.0),
        dhat=0.005, dhat_external=0.005, kappa=100.0,
        solver_max_iterations=20,
        solver_gradient_tolerance=1e-5,
        write_surfaces=True,
    )
    assert n == 2
    assert Path(out, "summary.json").exists()
    assert Path(out, "surface").is_dir()


def test_run_volume_ipc_with_tricubic_hermite(tmp_path):
    out = str(tmp_path / "out")
    n = run_volume_ipc(
        veg_path=_VEG,
        surface_path=_OBJ,
        output_dir=out,
        timestep=0.001,
        num_steps=1,
        gravity=(0.0, -9.81, 0.0),
        formulation="tricubic-hermite",
        dhat=0.005, dhat_external=0.005, kappa=100.0,
        solver_max_iterations=20,
        solver_gradient_tolerance=1e-5,
    )
    assert n == 1
    assert Path(out, "summary.json").exists()
