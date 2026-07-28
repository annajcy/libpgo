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


class NeoHookeanDefinition(ElasticModelDefinition):
    """Classical logarithmic compressible Neo-Hookean material."""

    def __init__(self) -> None:
        super().__init__(_core.PyNeoHookeanDefinition())


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


class SystematicPokingDefinition(ElasticModelDefinition):
    """Paper-final Systematic Poking principal-stretch material.

    The definition owns the spline knot layouts. Its optimizable material
    channels are ``f_dd_0, ..., f_dd_n, lambda``; there are no fixed material
    channels.
    """

    def __init__(
        self,
        stretch_knots,
        stretch_rest_knot_index: int,
        volume_knots,
        volume_rest_knot_index: int,
    ) -> None:
        stretch_knots = tuple(float(x) for x in stretch_knots)
        volume_knots = tuple(float(x) for x in volume_knots)
        stretch_rest_knot_index = int(stretch_rest_knot_index)
        volume_rest_knot_index = int(volume_rest_knot_index)
        super().__init__(_core.PySystematicPokingDefinition(
            list(stretch_knots),
            stretch_rest_knot_index,
            list(volume_knots),
            volume_rest_knot_index,
        ))
        self._stretch_knots = stretch_knots
        self._stretch_rest_knot_index = stretch_rest_knot_index
        self._volume_knots = volume_knots
        self._volume_rest_knot_index = volume_rest_knot_index

    @property
    def stretch_knots(self) -> tuple[float, ...]:
        return self._stretch_knots

    @property
    def stretch_rest_knot_index(self) -> int:
        return self._stretch_rest_knot_index

    @property
    def volume_knots(self) -> tuple[float, ...]:
        return self._volume_knots

    @property
    def volume_rest_knot_index(self) -> int:
        return self._volume_rest_knot_index

    def __repr__(self) -> str:
        return (
            "SystematicPokingDefinition("
            f"stretch_knots={self.stretch_knots!r}, "
            f"stretch_rest_knot_index={self.stretch_rest_knot_index}, "
            f"volume_knots={self.volume_knots!r}, "
            f"volume_rest_knot_index={self.volume_rest_knot_index})"
        )
