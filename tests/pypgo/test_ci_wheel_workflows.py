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
        )
    )

    assert "pypgo-ci" in preset_file
    assert "pypgo-ci" in workflows
    assert '"PGO_ENABLE_GMSH": "OFF"' in preset_file
    assert '"PGO_ENABLE_OPENVDB": "ON"' in preset_file


def test_linux_wheel_uses_mkl_and_no_openblas_or_openmp():
    workflow = read_workflow("linux-ci.yml")

    assert '"libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel' in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert 'python -m pip install --no-deps "${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'python -m venv "${clean_env}"' not in workflow
    assert "--exclude 'libmkl*.so*'" in workflow
    assert "openmp" not in workflow.lower()
    assert '"openblas"' in workflow
    assert '"libgomp"' in workflow
    assert "-X faulthandler" in workflow


def test_macos_wheel_uses_accelerate_and_no_openblas_or_openmp():
    workflow = read_workflow("macos-ci.yml")

    assert '"libblas=*=*accelerate" "liblapack=*=*accelerate"' in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert 'python -m pip install --no-deps "${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'python -m venv "${clean_env}"' not in workflow
    assert "openmp" not in workflow.lower()
    assert '"openblas"' in workflow
    assert '"libomp"' in workflow


def test_windows_wheel_uses_mkl_and_no_openblas_or_openmp():
    workflow = read_workflow("windows-ci.yml")

    assert 'python=3.12 pip numpy "libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel' in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert "python -m pip install --no-deps $wheel[0].FullName" in workflow
    assert "python -m venv $cleanEnv" not in workflow
    assert "$excludedDlls" in workflow
    assert "$excludeArgs += @(\"--exclude\", $dll)" in workflow
    assert "mkl_rt.2.dll" in workflow
    assert "mkl_core.2.dll" in workflow
    assert "mkl_tbb_thread.3.dll" in workflow
    assert "openmp" not in workflow.lower()
    assert '"openblas"' in workflow
    assert '"libomp"' in workflow
    assert '"vcomp"' in workflow
