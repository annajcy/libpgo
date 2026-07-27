"""Material definitions for volume meshes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar, Literal

import numpy as np

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




@dataclass
class MooneyRivlinMaterial:
    """Mooney-Rivlin hyperelastic material, represented as Python data."""

    name: str = "mooneyRivlinMaterial"
    density: float = 1000.0
    mu01: float = 0.0
    mu10: float = 0.0
    v1: float = 0.0
    type: ClassVar[Literal["mooney_rivlin"]] = "mooney_rivlin"


@dataclass(eq=False)
class OrthotropicMaterial:
    """Orthotropic material payload with a material-to-reference rotation."""

    name: str = "orthotropicMaterial"
    density: float = 1000.0
    E1: float = 0.0
    E2: float = 0.0
    E3: float = 0.0
    nu12: float = 0.0
    nu23: float = 0.0
    nu31: float = 0.0
    G12: float = 0.0
    G23: float = 0.0
    G31: float = 0.0
    rotation: object = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    type: ClassVar[Literal["orthotropic"]] = "orthotropic"

    def __post_init__(self) -> None:
        values = np.asarray(self.rotation, dtype=np.float64)
        if values.shape != (3, 3):
            raise ValueError("rotation must have shape (3, 3)")
        self.rotation = np.ascontiguousarray(values)

    def __eq__(self, other) -> bool:
        if not isinstance(other, OrthotropicMaterial):
            return NotImplemented
        scalar_names = (
            "name", "density", "E1", "E2", "E3",
            "nu12", "nu23", "nu31", "G12", "G23", "G31",
        )
        return all(getattr(self, name) == getattr(other, name) for name in scalar_names) and bool(
            np.array_equal(self.rotation, other.rotation)
        )


MaterialLike = ENuMaterial | MooneyRivlinMaterial | OrthotropicMaterial


# ---------------------------------------------------------------------------
# Material <-> C++ payload conversion
# ---------------------------------------------------------------------------


def _wrap_material_payload(m) -> MaterialLike:
    if isinstance(m, _core.PyVegENuMaterialPayload):
        return ENuMaterial(m.name, density=m.density, E=m.E, nu=m.nu)
    if isinstance(m, _core.PyVegMooneyRivlinMaterialPayload):
        return MooneyRivlinMaterial(
            m.name, density=m.density, mu01=m.mu01, mu10=m.mu10, v1=m.v1)
    if isinstance(m, _core.PyVegOrthotropicMaterialPayload):
        return OrthotropicMaterial(
            m.name,
            density=m.density,
            E1=m.E1,
            E2=m.E2,
            E3=m.E3,
            nu12=m.nu12,
            nu23=m.nu23,
            nu31=m.nu31,
            G12=m.G12,
            G23=m.G23,
            G31=m.G31,
            rotation=np.asarray(m.R, dtype=np.float64).reshape(3, 3),
        )
    raise RuntimeError(f"Unexpected material payload from _core: {type(m).__name__}")


def _material_to_core_payload(m: MaterialLike):
    if isinstance(m, ENuMaterial):
        return _core._create_enu_material_payload(m.name, m.density, m.E, m.nu)
    if isinstance(m, MooneyRivlinMaterial):
        return _core._create_mooney_rivlin_material_payload(
            m.name, m.density, m.mu01, m.mu10, m.v1)
    if isinstance(m, OrthotropicMaterial):
        return _core._create_orthotropic_material_payload(
            m.name,
            m.density,
            m.E1,
            m.E2,
            m.E3,
            m.nu12,
            m.nu23,
            m.nu31,
            m.G12,
            m.G23,
            m.G31,
            m.rotation.reshape(-1).tolist(),
        )
    raise TypeError(f"unsupported material type: {type(m).__name__}")
