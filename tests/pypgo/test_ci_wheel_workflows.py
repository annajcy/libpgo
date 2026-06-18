from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_workflow(name: str) -> str:
    return (ROOT / ".github" / "workflows" / name).read_text()


def test_ci_cmake_presets_use_ci_names():
    preset_file = (ROOT / "CMakePresets.json").read_text()
    workflows = "\n".join(
        read_workflow(name)
        for name in (
            "linux-ci.yml",
            "macos-ci.yml",
            "windows-ci.yml",
            "conda-release.yml",
        )
    )

    assert "pypgo-ci" in preset_file
    assert "pypgo-mkl-ci" in preset_file
    assert "pypgo-ci" in workflows
    assert "pypgo-mkl-ci" in workflows
    assert "pypgo-conda" not in preset_file
    assert "pypgo-conda" not in workflows
    assert '"PGO_ENABLE_OPENMP": "OFF"' in preset_file
    assert '"PGO_ENABLE_OPENMP": "ON"' in preset_file


def test_linux_openblas_wheel_is_pypi_friendly_and_mkl_wheel_is_conda_bound():
    workflow = read_workflow("linux-ci.yml")

    assert '"libopenblas=*=*pthreads*"' in workflow
    assert 'python -m venv "${clean_env}"' in workflow
    assert '"${clean_env}/bin/python" -m pip install "${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'if [[ "${PYPGO_WHEEL_PACKAGE}" == "pypgo-mkl" ]]; then' in workflow
    assert "--exclude 'libmkl*.so*'" in workflow
    assert "unexpectedly vendors an OpenMP runtime" in workflow
    assert "-X faulthandler" in workflow


def test_macos_openblas_wheel_is_pypi_friendly():
    workflow = read_workflow("macos-ci.yml")

    assert '"libopenblas=*=*pthreads*"' in workflow
    assert 'python -m venv "${clean_env}"' in workflow
    assert '"${clean_env}/bin/python" -m pip install "${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert "-e libopenblas" not in workflow
    assert "-e libomp" not in workflow


def test_windows_openblas_wheel_is_pypi_friendly_and_mkl_wheel_is_conda_bound():
    workflow = read_workflow("windows-ci.yml")

    assert '"libopenblas=*=*pthreads*"' in workflow
    assert "python -m venv $cleanEnv" in workflow
    assert "python -m pip install $wheel[0].FullName" in workflow
    assert '$env:PYPGO_WHEEL_PACKAGE -eq "pypgo-mkl"' in workflow
    assert "$excludedDlls" in workflow
    assert "$excludeArgs += @(\"--exclude\", $dll)" in workflow
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
