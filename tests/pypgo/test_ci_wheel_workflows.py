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
    assert '"PGO_ENABLE_GMSH": "OFF"' in preset_file
    assert '"PGO_ENABLE_GMSH": "ON"' in preset_file
    assert '"PGO_ENABLE_OPENVDB": "ON"' in preset_file


def test_linux_wheels_are_conda_bound():
    workflow = read_workflow("linux-ci.yml")

    assert '"libopenblas=*=*pthreads*"' in workflow
    assert 'if [[ "${PYPGO_WHEEL_PACKAGE}" == "pypgo" ]]; then' in workflow
    assert '"libblas=*=*openblas" "liblapack=*=*openblas" "libopenblas=*=*pthreads*"' in workflow
    assert '"libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel' in workflow
    assert 'python -m pip install --no-deps "${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'python -m venv "${clean_env}"' not in workflow
    assert "--exclude 'libmkl*.so*'" in workflow
    assert "unexpectedly vendors BLAS/LAPACK/OpenMP runtime libraries" in workflow
    assert "-X faulthandler" in workflow


def test_ftetwild_geogram_openmp_follows_pgo_openmp_option():
    cmake = (ROOT / "CMakeModules" / "third-party" / "ftetwild.cmake").read_text()

    assert "pgo_fetch_populate_compat(ftetwild" in cmake
    assert "_libpgo_patch_ftetwild_geogram_openmp" in cmake
    assert "_libpgo_patch_geogram_linux_openmp" in cmake
    assert 'if(${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND PGO_ENABLE_OPENMP)' in cmake


def test_geogram_linux_platform_openmp_follows_pgo_openmp_option():
    cmake = (ROOT / "CMakeModules" / "third-party" / "geogram.cmake").read_text()

    assert "_libpgo_patch_geogram_linux_openmp" in cmake
    assert "GCC_VERSION VERSION_GREATER 4.0 AND PGO_ENABLE_OPENMP" in cmake


def test_macos_openblas_wheel_is_conda_bound():
    workflow = read_workflow("macos-ci.yml")

    assert '"libopenblas=*=*pthreads*"' in workflow
    assert 'conda create -y -p "${clean_env}" python=3.12 pip numpy "libblas=*=*openblas" "liblapack=*=*openblas" "libopenblas=*=*pthreads*"' in workflow
    assert 'python -m pip install --no-deps "${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'python -m venv "${clean_env}"' not in workflow
    assert "-e libopenblas" in workflow
    assert "-e libomp" in workflow


def test_windows_wheels_are_conda_bound():
    workflow = read_workflow("windows-ci.yml")

    assert '"libopenblas=*=*pthreads*"' in workflow
    assert '$env:PYPGO_WHEEL_PACKAGE -eq "pypgo"' in workflow
    assert 'python=3.12 pip numpy "libblas=*=*openblas" "liblapack=*=*openblas" "libopenblas=*=*pthreads*"' in workflow
    assert "python -m pip install --no-deps $wheel[0].FullName" in workflow
    assert "python -m venv $cleanEnv" not in workflow
    assert '$env:PYPGO_WHEEL_PACKAGE -eq "pypgo-mkl"' in workflow
    assert "$excludedDlls" in workflow
    assert "$excludeArgs += @(\"--exclude\", $dll)" in workflow
    assert "mkl_rt.2.dll" in workflow
    assert "mkl_core.2.dll" in workflow
    assert "mkl_tbb_thread.3.dll" in workflow


def test_windows_ci_serializes_heavy_conda_jobs():
    workflow = read_workflow("windows-ci.yml")

    assert workflow.count("max-parallel: 1") == 2


def test_conda_release_retries_windows_tool_install():
    workflow = read_workflow("conda-release.yml")

    assert "max-parallel: 2" in workflow
    assert "$attempts = 3" in workflow
    assert "conda install failed on attempt" in workflow
