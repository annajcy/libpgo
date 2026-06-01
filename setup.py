"""Setuptools entry point for the Python-first pypgo package."""

from pathlib import Path
import os
import shutil
import subprocess

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel

    class bdist_wheel(_bdist_wheel):
        def get_tag(self):
            _python, _abi, plat = super().get_tag()
            return "cp312", "abi3", plat
except ImportError:
    bdist_wheel = None

install_requires = ["numpy"]


class CMakeExtension(Extension):
    """Placeholder extension built by the CMake preset."""

    def __init__(self, name):
        super().__init__(name, sources=[], py_limited_api=True)


class CMakeBuildExt(build_ext):
    """Build pypgo's native extension through the CMake pypgo preset."""

    preset = "pypgo"
    target = "pypgo_core"

    def build_extension(self, ext):
        source_dir = Path(__file__).resolve().parent

        # Remove stale cpython-specific .so files from both the source package
        # directory and any setuptools build/lib.* staging directories so they
        # don't pollute the abi3 wheel.
        for stale in source_dir.glob("**/pypgo/_core.cpython-*"):
            stale.unlink(missing_ok=True)

        subprocess.check_call(["cmake", "--preset", self.preset], cwd=source_dir)
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
        return max(1, os.cpu_count() or 1)

    def _find_built_extension(self, source_dir, expected_name):
        package_dir = source_dir / "pypgo"

        # Prefer stable ABI build (abi3.so) over cpython-specific builds.
        abi3_candidates = sorted(package_dir.glob("_core.abi3.*"))
        abi3_candidates = [p for p in abi3_candidates if p.suffix in {".so", ".pyd", ".dll", ".dylib"}]
        if abi3_candidates:
            return abi3_candidates[-1]

        expected_path = package_dir / expected_name
        if expected_path.exists():
            return expected_path

        candidates = sorted(package_dir.glob("_core.*"))
        candidates = [
            path
            for path in candidates
            if path.suffix in {".so", ".pyd", ".dll", ".dylib"}
        ]
        if not candidates:
            raise RuntimeError(
                "CMake build completed, but pypgo/_core extension was not found."
            )
        return candidates[-1]


cmdclass = {"build_ext": CMakeBuildExt}
if bdist_wheel is not None:
    cmdclass["bdist_wheel"] = bdist_wheel

setup(
    name="pypgo",
    version="0.0.3",
    author="Bohan Wang",
    author_email="wangbh11@gmail.com",
    description="Python-first libpgo package",
    long_description="",
    packages=find_packages(include=["pypgo", "pypgo.*"]),
    ext_modules=[CMakeExtension("pypgo._core")],
    cmdclass=cmdclass,
    zip_safe=False,
    install_requires=install_requires,
    extras_require={
        "test": ["pytest>=6.0"],
        "examples": ["pyvista[jupyter]"],
    },
    python_requires=">=3.12",
)
