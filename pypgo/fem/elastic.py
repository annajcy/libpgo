"""Elastic model configuration wrappers."""

from __future__ import annotations

import pypgo._core as _core


class ElasticModelConfig:
    """Abstract model configuration backed by a shared C++ config object."""

    def __init__(self, core_obj) -> None:
        if type(self) is ElasticModelConfig:
            raise TypeError("ElasticModelConfig is abstract; use a concrete config such as StableNeo()")
        if not isinstance(core_obj, _core.PyElasticModelConfig):
            raise TypeError("core_obj must be a PyElasticModelConfig")
        self._handle = core_obj

    @property
    def name(self) -> str:
        return self._handle.name

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class StableNeo(ElasticModelConfig):
    def __init__(self) -> None:
        super().__init__(_core.PyStableNeoConfig())


class StVK(ElasticModelConfig):
    def __init__(self) -> None:
        super().__init__(_core.PyStVKConfig())


class StVKVolume(ElasticModelConfig):
    def __init__(self) -> None:
        super().__init__(_core.PyStVKVolumeConfig())


class LinearElastic(ElasticModelConfig):
    def __init__(self) -> None:
        super().__init__(_core.PyLinearElasticConfig())


class MooneyRivlin(ElasticModelConfig):
    def __init__(self) -> None:
        super().__init__(_core.PyMooneyRivlinConfig())


class KoiterStVK(ElasticModelConfig):
    def __init__(self) -> None:
        super().__init__(_core.PyKoiterStVKConfig())


from dataclasses import dataclass


@dataclass(frozen=True)
class KoiterStVKShellMaterial:
    """Material parameters for the Koiter-St.Venant-Kirchhoff shell model.

    Used by ``SimulationMesh.create_shell`` and shell config I/O.
    """

    name: str = "shell"
    thickness: float = 0.001
    E_membrane: float = 1e6
    nu_membrane: float = 0.4


ShellMaterialLike = KoiterStVKShellMaterial
