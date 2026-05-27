"""Python-first package facade for libpgo."""

from importlib import import_module

__all__ = ["mesh", "sim", "sparse", "tools"]


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
