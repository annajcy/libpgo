import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read_workflow(name: str) -> str:
    return (ROOT / ".github" / "workflows" / name).read_text()


def test_no_conda_toolchain_anywhere():
    assert not (ROOT / "environment.yml").exists()
    workflows = "\n".join(
        read_workflow(name)
        for name in (
            "linux-ci.yml",
            "macos-ci.yml",
            "windows-ci.yml",
        )
    )
    assert "setup-miniconda" not in workflows
    assert "conda install" not in workflows
    assert "PGO_CHECK_CONDA" not in workflows
    assert "CONDA_PREFIX" not in workflows


def test_uv_is_the_python_toolchain():
    pyproject = (ROOT / "pyproject.toml").read_text()
    assert 'required-version = "==0.11.6"' in pyproject
    assert 'package = false' in pyproject
    assert "cmake==4.3.1" in pyproject
    assert "ninja==1.13.0" in pyproject
    assert "mkl-devel==2025.3.1" in pyproject
    assert (ROOT / "uv.lock").is_file()


def test_presets_keep_full_features_and_portable_wheels():
    presets = json.loads((ROOT / "CMakePresets.json").read_text())
    base = next(p for p in presets["configurePresets"] if p["name"] == "base")
    wheel = next(
        p for p in presets["configurePresets"] if p["name"] == "pypgo-wheel"
    )

    assert base["cacheVariables"]["PGO_ENABLE_GMSH"] == "ON"
    assert base["cacheVariables"]["PGO_ENABLE_OPENVDB"] == "ON"
    assert base["cacheVariables"]["PGO_TET_MESHER_USE_TET_WILD"] == "ON"
    assert base["cacheVariables"]["PGO_RUNTIME_LAYOUT"] == "SOURCE"

    assert wheel["cacheVariables"]["PGO_PORTABLE_BUILD"] == "ON"
    assert wheel["cacheVariables"]["PGO_RUNTIME_LAYOUT"] == "WHEEL"
    assert wheel["cacheVariables"]["PGO_ENABLE_RELEASE_DEBUG_INFO"] == "OFF"
    assert wheel["cacheVariables"]["PGO_ENABLE_GMSH"] == "ON"
    assert wheel["cacheVariables"]["PGO_ENABLE_OPENVDB"] == "ON"
    assert wheel["cacheVariables"]["PGO_TET_MESHER_USE_TET_WILD"] == "ON"


def test_ci_has_four_stage_wheel_pipeline():
    for name in ("linux-ci.yml", "macos-ci.yml", "windows-ci.yml"):
        workflow = read_workflow(name)
        for job in (
            "build-test:",
            "package-candidate:",
            "verify-installed-wheel:",
            "finalize-release:",
        ):
            assert f"\n  {job}" in workflow
        assert "uv sync --locked" in workflow
        assert "pypgo_wheel_" in workflow
        assert "installed_wheel_smoke.py" in workflow
        assert "release_wheel_provenance.py preflight" in workflow


def test_platform_specific_repair_tools():
    linux = read_workflow("linux-ci.yml")
    macos = read_workflow("macos-ci.yml")
    windows = read_workflow("windows-ci.yml")

    assert "quay.io/pypa/manylinux_2_28_x86_64" in linux
    assert "pypgo_wheel_linux.py package" in linux
    assert "macos-26" in macos
    assert "ARCHFLAGS: -arch arm64" in macos
    assert "pypgo_wheel_macos.py package" in macos
    assert "ilammy/msvc-dev-cmd" in windows
    assert "pypgo_wheel_windows.py package" in windows


def test_native_dependency_policy():
    workflows = "\n".join(
        read_workflow(name)
        for name in (
            "linux-ci.yml",
            "macos-ci.yml",
            "windows-ci.yml",
        )
    )
    assert "gmp-devel" in workflows
    assert "mpfr-devel" in workflows
    assert "brew install gmp mpfr libomp tbb" in read_workflow("macos-ci.yml")
    assert "brew install gmsh" not in workflows
    assert "brew install openvdb" not in workflows


def test_non_whitelisted_dependencies_are_fetchcontent_only():
    # Only GMP/MPFR (system) and TBB/MKL (system or uv mkl-devel) are allowed
    # to come from outside FetchContent. Everything else must be pinned and
    # fetched from upstream, even when a system package exists.
    third_party = ROOT / "CMakeModules" / "third-party"
    recipes = {
        "eigen": ("eigen.cmake", "find_package(Eigen3"),
        "boost": ("boost.cmake", "find_package(Boost"),
        "cgal": ("cgal.cmake", "find_package(CGAL CONFIG"),
        "ceres": ("ceres.cmake", "find_package(Ceres"),
        "gmsh": ("gmsh.cmake", "find_package(Gmsh"),
        "openvdb": ("openvdb.cmake", "find_package(OpenVDB"),
        "alembic": ("alembic.cmake", "find_package(Alembic"),
        "imath": ("imath.cmake", "find_package(Imath"),
        "fmt": ("fmt.cmake", "find_package(fmt"),
        "spdlog": ("spdlog.cmake", "find_package(spdlog"),
        "autodiff": ("autodiff.cmake", "find_package(autodiff"),
        "argparse": ("argparse.cmake", "find_package(argparse"),
        "libigl": ("libigl.cmake", "find_package(igl"),
        "geogram": ("geogram.cmake", "find_package(geogram"),
        "nanobind": ("nanobind.cmake", "find_package(nanobind"),
    }
    for name, (filename, forbidden) in recipes.items():
        text = (third_party / filename).read_text()
        assert "FetchContent_Declare(" in text or "pgo_add_third_party(" in text, (
            f"{name} must be built through FetchContent"
        )
        if name == "cgal":
            assert "NO_DEFAULT_PATH" in text
            continue
        assert forbidden not in text, (
            f"{name} must not fall back to a system find_package"
        )

    readme = (ROOT / "README.md").read_text()
    assert "brew install gmp mpfr" in readme
    assert "brew install libomp" in readme
    assert "brew install tbb" in readme or "Homebrew `tbb`" in readme
    assert "brew install gmsh" not in readme
    assert "brew install openvdb" not in readme
