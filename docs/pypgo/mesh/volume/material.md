# `pypgo.mesh.volume.material` — Material Definitions

Pure-Python `@dataclass` carriers for volume mesh materials, plus C++ payload conversion helpers. These dataclasses have no behavior beyond computed properties; simulation code reads them as configuration.

## Material Types

### `ENuMaterial`

Linear isotropic material. Lamé parameters are derived from Young's modulus `E` and Poisson's ratio `ν`.

```python
from pypgo.mesh.volume import ENuMaterial
mat = ENuMaterial("rubber", density=1000.0, E=1e6, nu=0.45)
print(mat.lam, mat.mu)
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `name` | `str` | `"defaultMaterial"` | Identifier |
| `density` | `float` | `1000.0` | Mass density |
| `E` | `float` | `1e9` | Young's modulus |
| `nu` | `float` | `0.45` | Poisson's ratio |
| `type` | `ClassVar` | `"enu"` | Discriminator |

| Property | Formula |
|----------|---------|
| `lam` | $\lambda = \frac{E\nu}{(1+\nu)(1-2\nu)}$ |
| `mu` | $\mu = \frac{E}{2(1+\nu)}$ |

**Linear isotropic elasticity:** The stress-strain relationship for an isotropic material is:

$$\boldsymbol{\sigma} = \lambda \, \text{tr}(\boldsymbol{\varepsilon}) \mathbf{I} + 2\mu \, \boldsymbol{\varepsilon}$$

where $\boldsymbol{\sigma}$ is the Cauchy stress tensor, $\boldsymbol{\varepsilon}$ is the infinitesimal strain tensor, and $\lambda, \mu$ are the Lamé parameters. $E$ (Young's modulus) measures tensile stiffness, while $\nu$ (Poisson's ratio) measures the transverse contraction under uniaxial load ($0 \leq \nu < 0.5$, with $\nu \to 0.5$ for nearly-incompressible materials like rubber).

### `MooneyRivlinMaterial`

Mooney-Rivlin hyperelastic material. Suitable for large-deformation rubber-like materials.

```python
from pypgo.mesh.volume import MooneyRivlinMaterial
mat = MooneyRivlinMaterial("rubber", density=1000.0, mu01=3.0, mu10=4.0, v1=0.2)
```

| Field | Type | Default |
|-------|------|---------|
| `name` | `str` | `"mooneyRivlinMaterial"` |
| `density` | `float` | `1000.0` |
| `mu01` | `float` | `0.0` |
| `mu10` | `float` | `0.0` |
| `v1` | `float` | `0.0` |
| `type` | `ClassVar` | `"mooney_rivlin"` |

**Strain energy density:** The general Mooney-Rivlin strain energy is (`elasticModel3DMooneyRivlin.cpp:114-155`):

$$\Psi = \sum_{p=0}^{N}\sum_{q=0}^{N} C_{pq} (\bar{I}_1-3)^p (\bar{I}_2-3)^q + \sum_{m=1}^{M} \frac{1}{D_{m-1}} (J-1)^{2m}, \quad C_{00}=0$$

where $\bar{I}_1 = J^{-2/3}\,\text{tr}(\mathbf{C})$ and $\bar{I}_2 = J^{-4/3}\,\frac{1}{2}\big(\text{tr}(\mathbf{C})^2 - \text{tr}(\mathbf{C}^2)\big)$ are the isochoric invariants of the right Cauchy-Green deformation tensor $\mathbf{C} = \mathbf{F}^T\mathbf{F}$, and $J = \det(\mathbf{F})$.

For the standard 2-parameter case ($N=1, M=1$): $C_{10} = \mu_{10}$, $C_{01} = \mu_{01}$, and $D_1 = 2/\kappa$ where $\kappa$ is the bulk modulus. The Python `MooneyRivlinMaterial` stores `mu01`, `mu10`, `v1` per the Vega FEM `.veg` file convention; these are converted to $C_{pq}, D_m$ by the simulation layer (`SimulationMeshMooneyRivlinMaterial`).

## Type Aliases

```python
from pypgo.mesh.volume import MaterialSpec, MaterialLike

MaterialSpec = ENuMaterial                           # Default type
MaterialLike = ENuMaterial | MooneyRivlinMaterial
```

---

## Conversion Helpers (private)

| Function | Direction |
|----------|-----------|
| `_wrap_material_payload(core_payload) → MaterialLike` | C++ → Python |
| `_material_to_core_payload(material) → core_payload` | Python → C++ |

These handle the dispatch across the three material types. Not part of the public API.
