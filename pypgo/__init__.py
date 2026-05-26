"""Python-first package facade for libpgo."""

from importlib import import_module

__all__ = ["mesh_geo", "mesh", "io"]


def __getattr__(name: str):
    if name in __all__:
        return import_module(f"{__name__}.{name}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
