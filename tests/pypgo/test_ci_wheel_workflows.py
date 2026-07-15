import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_workflow(name: str) -> str:
    return (ROOT / ".github" / "workflows" / name).read_text()


def test_build_tools_are_system_owned():
    environment = (ROOT / "environment.yml").read_text()
    pyproject = (ROOT / "pyproject.toml").read_text()
    workflows = "\n".join(
        read_workflow(name)
        for name in (
            "linux-ci.yml",
            "macos-ci.yml",
            "windows-ci.yml",
        )
    )

    for package in ("cmake", "ninja", "pkg-config", "gcc", "gxx"):
        assert f"- {package}" not in environment
    assert '"cmake' not in pyproject
    assert '"ninja' not in pyproject
    assert "conda install -y -c conda-forge gcc gxx" not in workflows
    assert "build-essential cmake ninja-build pkg-config" in read_workflow("linux-ci.yml")
    assert "brew install cmake ninja pkg-config" in read_workflow("macos-ci.yml")
    assert "choco install cmake ninja" not in read_workflow("windows-ci.yml")
    assert "ilammy/msvc-dev-cmd" in read_workflow("windows-ci.yml")


def test_ci_reuses_one_portable_full_build_for_tests_and_wheel():
    preset_file = (ROOT / "CMakePresets.json").read_text()
    presets = json.loads(preset_file)
    base_ci = next(
        preset for preset in presets["configurePresets"] if preset["name"] == "base-ci"
    )
    workflows = "\n".join(
        read_workflow(name)
        for name in (
            "linux-ci.yml",
            "macos-ci.yml",
            "windows-ci.yml",
        )
    )

    assert base_ci["inherits"] == ["base"]
    assert base_ci["binaryDir"] == "${sourceDir}/build/base-ci"
    assert base_ci["cacheVariables"]["PGO_NATIVE_OPTIMIZATION"] == "OFF"
    assert base_ci["cacheVariables"]["CMAKE_CXX_SCAN_FOR_MODULES"] == "OFF"
    assert workflows.count("PYPGO_CMAKE_PRESET: base-ci") == 3
    assert workflows.count("cmake --build --preset base-ci") == 3
    assert workflows.count("Build wheel from the tested build tree") == 3


def test_ci_has_one_job_per_platform_and_cancels_stale_runs():
    for name in ("linux-ci.yml", "macos-ci.yml", "windows-ci.yml"):
        workflow = read_workflow(name)
        assert workflow.count("\n  build-test-wheel:") == 1
        assert "matrix." not in workflow
        assert "strategy:" not in workflow
        assert "cancel-in-progress: true" in workflow
        assert workflow.count("python -m pytest -q tests/pypgo") == 1

    windows = read_workflow("windows-ci.yml")
    assert "Free up disk space" not in windows
    assert "C:\\Android" not in windows


def test_linux_wheel_uses_mkl_and_allows_openmp_runtime():
    workflow = read_workflow("linux-ci.yml")
    wheel_policy = (ROOT / "tests" / "check_wheel_vendoring.py").read_text()

    assert '"libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel' in workflow
    assert "MKL_THREADING_LAYER: TBB" in workflow
    assert "tests/check_mkl_tbb_runtime.py" in workflow
    assert "--import-pypgo" in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert '"${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'python -m venv "${clean_env}"' not in workflow
    assert "--exclude 'libmkl*.so*'" in workflow
    assert '"openblas"' in wheel_policy
    assert '"libgomp"' not in workflow
    assert "-X faulthandler" in workflow


def test_macos_numpy_and_native_extension_use_system_accelerate():
    workflow = read_workflow("macos-ci.yml")
    wheel_policy = (ROOT / "tests" / "check_wheel_vendoring.py").read_text()

    for package in ("libblas", "libcblas", "liblapack", "liblapacke"):
        assert f'"{package}=*=*_newaccelerate"' in workflow
    assert '"libblas=*=*accelerate"' not in workflow
    assert "tests/check_accelerate_runtime.py" in workflow
    assert "--import-pypgo" in workflow
    assert "MKL_THREADING_LAYER" not in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert '"${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'python -m venv "${clean_env}"' not in workflow
    assert '"openblas"' in wheel_policy
    assert '"libomp"' not in workflow
    assert workflow.count("tests/check_macos_accelerate_linkage.py") == 2
    linkage_check = (ROOT / "tests" / "check_macos_accelerate_linkage.py").read_text()
    assert "otool" in linkage_check
    assert "does not link the system Accelerate framework" in linkage_check


def test_windows_wheel_uses_mkl_and_allows_openmp_runtime():
    workflow = read_workflow("windows-ci.yml")
    wheel_policy = (ROOT / "tests" / "check_wheel_vendoring.py").read_text()

    assert 'python=3.12 pip numpy "libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel' in workflow
    assert "MKL_THREADING_LAYER: TBB" in workflow
    assert "tests/check_mkl_tbb_runtime.py" in workflow
    assert "--import-pypgo" in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert "python -m pip install --no-deps $wheel[0].FullName" in workflow
    assert "python -m venv $cleanEnv" not in workflow
    assert "$excludedDlls" in workflow
    assert "$excludeArgs += @(\"--exclude\", $dll)" in workflow
    assert 'foreach ($stem in @("mkl_rt", "mkl_core"' in workflow
    assert 'foreach ($suffix in @("", ".2", ".3"))' in workflow
    assert '"openblas"' in wheel_policy
    assert '"libomp"' not in workflow
    assert '"vcomp"' not in workflow
