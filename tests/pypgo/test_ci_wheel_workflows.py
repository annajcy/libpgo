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
    assert "choco install cmake ninja" in read_workflow("windows-ci.yml")


def test_ci_cmake_presets_use_ci_names():
    preset_file = (ROOT / "CMakePresets.json").read_text()
    presets = json.loads(preset_file)
    pypgo_ci = next(preset for preset in presets["configurePresets"] if preset["name"] == "pypgo-ci")
    pypgo_ci_cache = pypgo_ci["cacheVariables"]
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
    assert pypgo_ci_cache["PGO_ENABLE_GMSH"] == "ON"
    assert pypgo_ci_cache["PGO_ENABLE_OPENVDB"] == "ON"


def test_linux_wheel_uses_mkl_and_allows_openmp_runtime():
    workflow = read_workflow("linux-ci.yml")

    assert '"libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel' in workflow
    assert "MKL_THREADING_LAYER: TBB" in workflow
    assert "tests/check_mkl_tbb_runtime.py" in workflow
    assert "--import-pypgo" in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert 'python -m pip install --no-deps "${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'python -m venv "${clean_env}"' not in workflow
    assert "--exclude 'libmkl*.so*'" in workflow
    assert '"openblas"' in workflow
    assert '"libgomp"' not in workflow
    assert "-X faulthandler" in workflow


def test_macos_wheel_uses_accelerate_and_allows_openmp_runtime():
    workflow = read_workflow("macos-ci.yml")

    assert '"libblas=*=*accelerate" "liblapack=*=*accelerate"' in workflow
    assert "MKL_THREADING_LAYER" not in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert 'python -m pip install --no-deps "${GITHUB_WORKSPACE}"/wheelhouse/${PYPGO_WHEEL_DIST}-*.whl' in workflow
    assert 'python -m venv "${clean_env}"' not in workflow
    assert '"openblas"' in workflow
    assert '"libomp"' not in workflow


def test_windows_wheel_uses_mkl_and_allows_openmp_runtime():
    workflow = read_workflow("windows-ci.yml")

    assert 'python=3.12 pip numpy "libblas=*=*mkl" "liblapack=*=*mkl" mkl-devel' in workflow
    assert "MKL_THREADING_LAYER: TBB" in workflow
    assert "tests/check_mkl_tbb_runtime.py" in workflow
    assert "--import-pypgo" in workflow
    assert '"libblas=*=*openblas"' not in workflow
    assert "python -m pip install --no-deps $wheel[0].FullName" in workflow
    assert "python -m venv $cleanEnv" not in workflow
    assert "$excludedDlls" in workflow
    assert "$excludeArgs += @(\"--exclude\", $dll)" in workflow
    assert "mkl_rt.2.dll" in workflow
    assert "mkl_core.2.dll" in workflow
    assert "mkl_tbb_thread.3.dll" in workflow
    assert '"openblas"' in workflow
    assert '"libomp"' not in workflow
    assert '"vcomp"' not in workflow
