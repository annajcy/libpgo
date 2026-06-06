"""Material definitions for volume meshes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar, Literal

import pypgo._core as _core


# ---------------------------------------------------------------------------
# Material data carriers
# ---------------------------------------------------------------------------


@dataclass
class ENuMaterial:
    """Linear isotropic material, represented as a pure Python data carrier."""

    name: str = "defaultMaterial"
    density: float = 1000.0
    E: float = 1e9
    nu: float = 0.45
    type: ClassVar[Literal["enu"]] = "enu"

    @property
    def lam(self) -> float:
        return self.E * self.nu / ((1 + self.nu) * (1 - 2 * self.nu))

    @property
    def mu(self) -> float:
        return self.E / (2 * (1 + self.nu))


MaterialSpec = ENuMaterial


@dataclass
class MooneyRivlinMaterial:
    """Mooney-Rivlin hyperelastic material, represented as Python data."""

    name: str = "mooneyRivlinMaterial"
    density: float = 1000.0
    mu01: float = 0.0
    mu10: float = 0.0
    v1: float = 0.0
    type: ClassVar[Literal["mooney_rivlin"]] = "mooney_rivlin"


MaterialLike = ENuMaterial | MooneyRivlinMaterial


# ---------------------------------------------------------------------------
# Material <-> C++ payload conversion
# ---------------------------------------------------------------------------


def _wrap_material_payload(m) -> MaterialLike:
    if isinstance(m, _core.PyVegENuMaterialPayload):
        return ENuMaterial(m.name, density=m.density, E=m.E, nu=m.nu)
    if isinstance(m, _core.PyVegMooneyRivlinMaterialPayload):
        return MooneyRivlinMaterial(
            m.name, density=m.density, mu01=m.mu01, mu10=m.mu10, v1=m.v1)
    raise RuntimeError(f"Unexpected material payload from _core: {type(m).__name__}")


def _material_to_core_payload(m: MaterialLike):
    if isinstance(m, ENuMaterial):
        return _core.create_enu_material_payload(m.name, m.density, m.E, m.nu)
    if isinstance(m, MooneyRivlinMaterial):
        return _core.create_mooney_rivlin_material_payload(
            m.name, m.density, m.mu01, m.mu10, m.v1)
    raise TypeError(f"unsupported material type: {type(m).__name__}")
