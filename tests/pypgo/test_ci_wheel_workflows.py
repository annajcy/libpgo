from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_workflow(name: str) -> str:
    return (ROOT / ".github" / "workflows" / name).read_text()


def test_linux_wheel_repair_excludes_conda_blas_runtime():
    workflow = read_workflow("linux-ci.yml")

    assert "--exclude 'libopenblas*.so*'" in workflow
    assert "--exclude 'libblas*.so*'" in workflow
    assert "--exclude 'liblapack*.so*'" in workflow
    assert "--exclude 'libmkl*.so*'" in workflow


def test_windows_wheel_repair_excludes_conda_blas_runtime():
    workflow = read_workflow("windows-ci.yml")

    assert "$excludedDlls" in workflow
    assert "libopenblas.dll" in workflow
    assert "libblas.dll" in workflow
    assert "liblapack.dll" in workflow
    assert "mkl_rt.2.dll" in workflow
    assert "mkl_core.2.dll" in workflow
