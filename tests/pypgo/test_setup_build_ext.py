import runpy
import shutil
from pathlib import Path

import setuptools


ROOT = Path(__file__).resolve().parents[2]


def load_setup_namespace(monkeypatch):
    setup_calls = []
    monkeypatch.setattr(setuptools, "setup", lambda **kwargs: setup_calls.append(kwargs))
    namespace = runpy.run_path(str(ROOT / "setup.py"))
    assert setup_calls
    return namespace, setup_calls[0]


def test_setup_registers_cmake_build_ext(monkeypatch):
    _namespace, setup_kwargs = load_setup_namespace(monkeypatch)

    assert setup_kwargs["ext_modules"][0].name == "pypgo._core"
    assert "build_ext" in setup_kwargs["cmdclass"]


def test_setup_registers_examples_extra(monkeypatch):
    _namespace, setup_kwargs = load_setup_namespace(monkeypatch)

    assert setup_kwargs["extras_require"]["examples"] == ["pyvista[jupyter]"]


def test_cmake_build_ext_uses_python_build_preset(monkeypatch, tmp_path):
    namespace, setup_kwargs = load_setup_namespace(monkeypatch)
    build_ext_cls = setup_kwargs["cmdclass"]["build_ext"]

    commands = []
    monkeypatch.setattr(
        namespace["subprocess"],
        "check_call",
        lambda command, cwd: commands.append((command, cwd)),
    )

    built_extension = tmp_path / "_core.cpython-314-darwin.so"
    built_extension.write_bytes(b"native extension")

    copied = []
    monkeypatch.setattr(
        shutil,
        "copy2",
        lambda src, dst: copied.append((Path(src), Path(dst))),
    )

    builder = object.__new__(build_ext_cls)
    builder.parallel = 8
    builder._find_built_extension = lambda source_dir, expected_name: built_extension
    builder.get_ext_fullpath = lambda name: str(
        tmp_path / "build" / "lib" / "pypgo" / "_core.cpython-314-darwin.so"
    )

    ext = setup_kwargs["ext_modules"][0]
    builder.build_extension(ext)

    assert commands == [
        (["cmake", "--preset", "python-build"], ROOT),
        (
            [
                "cmake",
                "--build",
                "--preset",
                "python-build",
                "--target",
                "pypgo_core",
                "--parallel",
                "8",
            ],
            ROOT,
        ),
    ]
    assert copied == [
        (
            built_extension,
            tmp_path / "build" / "lib" / "pypgo" / "_core.cpython-314-darwin.so",
        )
    ]


def test_cmake_build_ext_infers_parallel_jobs_by_default(monkeypatch, tmp_path):
    namespace, setup_kwargs = load_setup_namespace(monkeypatch)
    build_ext_cls = setup_kwargs["cmdclass"]["build_ext"]

    commands = []
    monkeypatch.setattr(
        namespace["subprocess"],
        "check_call",
        lambda command, cwd: commands.append((command, cwd)),
    )
    monkeypatch.setattr(namespace["os"], "cpu_count", lambda: 12)

    built_extension = tmp_path / "_core.cpython-314-darwin.so"
    built_extension.write_bytes(b"native extension")

    builder = object.__new__(build_ext_cls)
    builder.parallel = None
    builder._find_built_extension = lambda source_dir, expected_name: built_extension
    builder.get_ext_fullpath = lambda name: str(
        tmp_path / "build" / "lib" / "pypgo" / "_core.cpython-314-darwin.so"
    )

    ext = setup_kwargs["ext_modules"][0]
    builder.build_extension(ext)

    assert commands[1] == (
        [
            "cmake",
            "--build",
            "--preset",
            "python-build",
            "--target",
            "pypgo_core",
            "--parallel",
            "12",
        ],
        ROOT,
    )
