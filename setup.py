"""Setuptools entry point for the Python-first pypgo package."""

from pathlib import Path
import os
import shutil
import subprocess
import sys

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

PACKAGE_NAME = os.environ.get("PYPGO_PACKAGE_NAME", "pypgo").strip()
SUPPORTED_PACKAGE_NAMES = {"pypgo"}
if PACKAGE_NAME not in SUPPORTED_PACKAGE_NAMES:
    raise RuntimeError(
        f"Unsupported PYPGO_PACKAGE_NAME={PACKAGE_NAME!r}; "
        f"expected one of {sorted(SUPPORTED_PACKAGE_NAMES)}."
)
INSTALL_REQUIRES = []


class CMakeExtension(Extension):
    """Placeholder extension built by the CMake preset."""

    def __init__(self, name):
        super().__init__(name, sources=[], py_limited_api=True)


class CMakeBuildExt(build_ext):
    """Build pypgo's native extension through the CMake pypgo preset."""

    preset = os.environ.get("PYPGO_CMAKE_PRESET", "pypgo")
    target = "pypgo_core"

    def build_extension(self, ext):
        source_dir = Path(__file__).resolve().parent

        configure_command = [
            "cmake",
            "--preset",
            self.preset,
            f"-DPython_EXECUTABLE={sys.executable}",
        ]
        subprocess.check_call(configure_command, cwd=source_dir)
        build_command = [
            "cmake",
            "--build",
            "--preset",
            self.preset,
            "--target",
            self.target,
        ]
        build_command.extend(["--parallel", str(self._parallel_jobs())])
        subprocess.check_call(build_command, cwd=source_dir)

        output_path = Path(self.get_ext_fullpath(ext.name)).resolve()
        built_ext = self._find_built_extension(source_dir, output_path.name)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        if built_ext.resolve() != output_path:
            shutil.copy2(built_ext, output_path)

    def _parallel_jobs(self):
        if self.parallel:
            return int(self.parallel)
        # Honor CMAKE_BUILD_PARALLEL_LEVEL so CI can cap concurrent compiles on
        # memory/disk-constrained runners (the Windows hosted runner otherwise dies
        # mid-build at the default cpu_count()). Unset → full cpu_count() as before.
        env_level = os.environ.get("CMAKE_BUILD_PARALLEL_LEVEL", "").strip()
        if env_level.isdigit() and int(env_level) > 0:
            return int(env_level)
        return max(1, os.cpu_count() or 1)

    def _find_built_extension(self, source_dir, expected_name):
        package_dir = source_dir / "pypgo"

        expected_path = package_dir / expected_name
        if expected_path.exists():
            return expected_path

        candidates = sorted(package_dir.glob("_core.*"))
        raise RuntimeError(
            "CMake build completed, but the expected extension "
            f"pypgo/{expected_name} was not found. Found: "
            f"{', '.join(path.name for path in candidates) or 'none'}."
        )


setup(
    name=PACKAGE_NAME,
    version="0.0.4",
    author="Bohan Wang",
    author_email="wangbh11@gmail.com",
    description="libpgo python binding",
    long_description="",
    packages=find_packages(include=["pypgo", "pypgo.*"]),
    ext_modules=[CMakeExtension("pypgo._core")],
    cmdclass={"build_ext": CMakeBuildExt},
    options={"bdist_wheel": {"py_limited_api": "cp312"}},
    entry_points={
        "console_scripts": [
            "pypgo-volume-info=pypgo.tools.mesh.volume.volume_info:main",
            "pypgo-cubic-mesher=pypgo.tools.mesh.volume.cubic_mesher:main",
            "pypgo-tetgen-mesher=pypgo.tools.mesh.volume.tetgen_mesher:main",
            "pypgo-ftetwild-mesher=pypgo.tools.mesh.volume.ftetwild_mesher:main",
            "pypgo-msh-converter=pypgo.tools.mesh.volume.msh_converter:main",
            "pypgo-surface-quality=pypgo.tools.mesh.surface.quality:main",
            "pypgo-surface-remesh=pypgo.tools.mesh.surface.remesh:main",
            "pypgo-surface-cleanup=pypgo.tools.mesh.surface.cleanup:main",
            "pypgo-sim-shell-static=pypgo.tools.sim.shell_static:main",
            "pypgo-sim-shell-dynamic=pypgo.tools.sim.shell_dynamic:main",
            "pypgo-sim-cubic-static=pypgo.tools.sim.cubic_static:main",
            "pypgo-sim-cubic-dynamic=pypgo.tools.sim.cubic_dynamic:main",
            "pypgo-sim-tet-static=pypgo.tools.sim.tet_static:main",
            "pypgo-sim-tet-dynamic=pypgo.tools.sim.tet_dynamic:main",
            "pypgo-sim-batch=pypgo.tools.sim.batch:main",
            "pypgo-animation-convert=pypgo.tools.animation.abc_convert:main",
            "pypgo-stress-vdb=pypgo.tools.animation.stress_vdb:main",
        ],
    },
    zip_safe=False,
    python_requires=">=3.12",
    install_requires=INSTALL_REQUIRES,
    extras_require={
        # Optional torch autograd layers in pypgo.fem (imported lazily).
        # Install torch explicitly in the conda environment when needed.
        "torch": ["torch"],
        # 3D visualization (pypgo.mesh.visualize) and interactive/web rendering.
        # pyvista pulls its own vtk wheel; trame provides the web backend.
        "viz": ["pyvista", "trame", "trame-vtk", "trame-vuetify"],
        # Test / notebook / demo tooling (not needed at runtime).
        # pytest-timeout enforces the per-test timeout in pyproject.toml so a hung
        # solve fails fast (with a traceback) instead of stalling the CI job.
        "dev": ["pytest", "pytest-timeout", "notebook"],
    },
)
