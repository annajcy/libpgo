"""Plastic model configuration wrappers."""

from __future__ import annotations

import pypgo._core as _core


class PlasticModelConfig:
    """Abstract plastic configuration backed by a shared C++ config object."""

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


class VolumetricPlasticity(PlasticModelConfig):
    def __init__(self, dofs: int = 6) -> None:
        if dofs not in (0, 3, 6):
            raise ValueError(f"VolumetricPlasticity dofs must be 0, 3, or 6, got {dofs}")
        config = {
            0: _core.PyVolumetricPlasticity0Config,
            3: _core.PyVolumetricPlasticity3Config,
            6: _core.PyVolumetricPlasticity6Config,
        }[dofs]()
        super().__init__(config)


class ShellPlasticity(PlasticModelConfig):
    def __init__(self, dofs: int = 1) -> None:
        if dofs not in (0, 1):
            raise ValueError(f"ShellPlasticity dofs must be 0 or 1, got {dofs}")
        config = {
            0: _core.PyShellPlasticity0Config,
            1: _core.PyShellPlasticity1Config,
        }[dofs]()
        super().__init__(config)
