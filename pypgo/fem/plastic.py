"""Plastic parametrization wrappers — each holds a persistent C++ PyPlasticModel handle."""

from __future__ import annotations

import pypgo._core as _core


class PlasticModel:
    """Abstract base — owns a C++ PyPlasticModel handle."""

    def __init__(self, core_obj) -> None:
        self._handle = core_obj

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def dofs(self) -> int:
        return self._handle.dofs

    def __repr__(self) -> str:
        return f"{type(self).__name__}(dofs={self.dofs})"


class VolumetricPlasticity(PlasticModel):
    def __init__(self, dofs: int = 6) -> None:
        if dofs not in (0, 3, 6):
            raise ValueError(f"VolumetricPlasticity dofs must be 0, 3, or 6, got {dofs}")
        super().__init__(_core.make_volumetric_plasticity(dofs))


class ShellPlasticity(PlasticModel):
    def __init__(self, dofs: int = 1) -> None:
        if dofs not in (0, 1):
            raise ValueError(f"ShellPlasticity dofs must be 0 or 1, got {dofs}")
        super().__init__(_core.make_shell_plasticity(dofs))
