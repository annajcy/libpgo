"""Elastic model definition wrappers."""

from __future__ import annotations

import pypgo._core as _core


class ElasticModelDefinition:
    """Abstract model definition backed by an immutable C++ definition."""

    def __init__(self, core_obj) -> None:
        if type(self) is ElasticModelDefinition:
            raise TypeError("ElasticModelDefinition is abstract; use a concrete definition such as StableNeoDefinition()")
        if not isinstance(core_obj, _core.PyElasticModelDefinition):
            raise TypeError("core_obj must be a PyElasticModelDefinition")
        self._handle = core_obj

    @property
    def name(self) -> str:
        return self._handle.name

    @property
    def fixed_channel_names(self) -> tuple[str, ...]:
        return tuple(self._handle.fixed_channel_names)

    @property
    def optimizable_channel_names(self) -> tuple[str, ...]:
        return tuple(self._handle.optimizable_channel_names)

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class StableNeoDefinition(ElasticModelDefinition):
    def __init__(self) -> None:
        super().__init__(_core.PyStableNeoDefinition())


class StVKDefinition(ElasticModelDefinition):
    def __init__(self) -> None:
        super().__init__(_core.PyStVKDefinition())


class StVKVolumeDefinition(ElasticModelDefinition):
    def __init__(self) -> None:
        super().__init__(_core.PyStVKVolumeDefinition())


class LinearElasticDefinition(ElasticModelDefinition):
    def __init__(self) -> None:
        super().__init__(_core.PyLinearElasticDefinition())


class MooneyRivlinDefinition(ElasticModelDefinition):
    def __init__(self) -> None:
        super().__init__(_core.PyMooneyRivlinDefinition())


class KoiterStVKDefinition(ElasticModelDefinition):
    def __init__(self) -> None:
        super().__init__(_core.PyKoiterStVKDefinition())
