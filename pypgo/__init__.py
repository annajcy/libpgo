"""Python-first package facade for libpgo."""

import os as _os
from importlib import import_module

if _os.name == "nt":
    # Keep the handle alive so bundled runtime DLLs beside the extension remain
    # available to oneMKL/oneTBB runtime loading.
    _dll_directory_handle = _os.add_dll_directory(_os.path.dirname(__file__))

from pypgo.logging import get_log_level, quiet_cpp_logs, set_log_level

__all__ = [
    "animation",
    "contact",
    "constraints",
    "fem",
    "energy",
    "implicit",
    "logging",
    "mesh",
    "parallel",
    "sim",
    "solver",
    "sparse",
    "tools",
    "visualize",
    "get_log_level",
    "quiet_cpp_logs",
    "set_log_level",
]


def __getattr__(name: str):
    if name == "_core":
        module = import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    if name == "__version__":
        try:
            from importlib.metadata import PackageNotFoundError, version

            return version("pypgo")
        except PackageNotFoundError:
            module = import_module(f"{__name__}._core")
            globals()["_core"] = module
            return module.__version__
    if name == "visualize":
        module = import_module(f"{__name__}.mesh.visualize")
        globals()[name] = module
        return module
    if name in __all__:
        module = import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
