"""Elastic law wrappers — each holds a persistent C++ PyElasticModel handle."""

from __future__ import annotations

import pypgo._core as _core


class ElasticModel:
    """Abstract base — owns a C++ PyElasticModel handle."""

    def __init__(self, core_obj) -> None:
        self._handle = core_obj

    @property
    def name(self) -> str:
        return self._handle.name

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class StableNeo(ElasticModel):
    def __init__(self) -> None:
        super().__init__(_core.make_stable_neo())


class StVK(ElasticModel):
    def __init__(self) -> None:
        super().__init__(_core.make_stvk())


class StVKVolume(ElasticModel):
    def __init__(self) -> None:
        super().__init__(_core.make_stvk_vol())


class LinearElastic(ElasticModel):
    def __init__(self) -> None:
        super().__init__(_core.make_linear_elastic())


class MooneyRivlin(ElasticModel):
    def __init__(self) -> None:
        super().__init__(_core.make_mooney_rivlin())


class KoiterStVK(ElasticModel):
    def __init__(self) -> None:
        super().__init__(_core.make_koiter_stvk())
