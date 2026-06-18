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


def test_macos_wheel_repair_excludes_conda_blas_and_openmp_runtime():
    workflow = read_workflow("macos-ci.yml")

    assert "-e libopenblas" in workflow
    assert "-e libblas" in workflow
    assert "-e liblapack" in workflow
    assert "-e libmkl" in workflow
    assert "-e libomp" in workflow


def test_windows_wheel_repair_excludes_conda_blas_runtime():
    workflow = read_workflow("windows-ci.yml")

    assert "$excludedDlls" in workflow
    assert "$excludeArgs += @(\"--exclude\", $dll)" in workflow
    assert "libopenblas.dll" in workflow
    assert "libblas.dll" in workflow
    assert "liblapack.dll" in workflow
    assert "mkl_rt.2.dll" in workflow
    assert "mkl_core.2.dll" in workflow


def test_windows_ci_serializes_heavy_conda_jobs():
    workflow = read_workflow("windows-ci.yml")

    assert workflow.count("max-parallel: 1") == 2


def test_conda_release_retries_windows_tool_install():
    workflow = read_workflow("conda-release.yml")

    assert "max-parallel: 2" in workflow
    assert "$attempts = 3" in workflow
    assert "conda install failed on attempt" in workflow
