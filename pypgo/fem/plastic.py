"""Plastic model definition wrappers."""

from __future__ import annotations

import pypgo._core as _core


class PlasticModelDefinition:
    """Abstract plastic model definition backed by an immutable C++ definition."""

    def __init__(self, core_obj) -> None:
        if type(self) is PlasticModelDefinition:
            raise TypeError("PlasticModelDefinition is abstract; use a concrete plastic definition")
        if not isinstance(core_obj, _core.PyPlasticModelDefinition):
            raise TypeError("core_obj must be a PyPlasticModelDefinition")
        self._handle = core_obj

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def dofs(self) -> int:
        return self._handle.dofs

    @property
    def num_fixed_channels(self) -> int:
        return int(self._handle.num_fixed_channels)

    @property
    def num_optimizable_channels(self) -> int:
        return int(self._handle.num_optimizable_channels)

    def __repr__(self) -> str:
        return f"{type(self).__name__}(dofs={self.dofs})"


class VolumetricPlasticityDefinition(PlasticModelDefinition):
    def __init__(self, dofs: int = 6) -> None:
        if dofs not in (0, 3, 6):
            raise ValueError(f"VolumetricPlasticityDefinition dofs must be 0, 3, or 6, got {dofs}")
        definition = {
            0: _core.PyVolumetricPlasticity0Definition,
            3: _core.PyVolumetricPlasticity3Definition,
            6: _core.PyVolumetricPlasticity6Definition,
        }[dofs]()
        super().__init__(definition)


class ShellPlasticityDefinition(PlasticModelDefinition):
    def __init__(self, dofs: int = 1) -> None:
        if dofs not in (0, 1):
            raise ValueError(f"ShellPlasticityDefinition dofs must be 0 or 1, got {dofs}")
        definition = {
            0: _core.PyShellPlasticity0Definition,
            1: _core.PyShellPlasticity1Definition,
        }[dofs]()
        super().__init__(definition)
