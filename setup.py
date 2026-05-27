"""Setuptools entry point for the Python-first pypgo package."""

from pathlib import Path
import os
import shutil
import subprocess

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

install_requires = ["numpy"]


class CMakeExtension(Extension):
    """Placeholder extension built by the CMake preset."""

    def __init__(self, name):
        super().__init__(name, sources=[])


class CMakeBuildExt(build_ext):
    """Build pypgo's native extension through the CMake python-build preset."""

    preset = "python-build"
    target = "pypgo_core"

    def build_extension(self, ext):
        source_dir = Path(__file__).resolve().parent

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


setup(
    name="pypgo",
    version="0.0.3",
    author="Bohan Wang",
    author_email="wangbh11@gmail.com",
    description="Python-first libpgo package",
    long_description="",
    packages=find_packages(include=["pypgo", "pypgo.*"]),
    ext_modules=[CMakeExtension("pypgo._core")],
    cmdclass={"build_ext": CMakeBuildExt},
    zip_safe=False,
    install_requires=install_requires,
    extras_require={
        "test": ["pytest>=6.0"],
        "examples": ["pyvista[jupyter]"],
    },
    python_requires=">=3.9",
)
