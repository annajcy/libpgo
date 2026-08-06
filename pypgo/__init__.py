"""Python-first package facade for libpgo."""

from importlib import import_module

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
    "get_log_level",
    "quiet_cpp_logs",
    "set_log_level",
]


def __getattr__(name: str):
    if name == "_core":
        module = import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    if name in __all__:
        module = import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
