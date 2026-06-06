# `pypgo.mesh.volume.core` — Volume Mesh & I/O

## `VegFile` Data Model

In-memory representation of a `.veg` file.

```python
from pypgo.mesh.volume import VegFile, MeshSet, MeshRegion
```

### `MeshSet`

```python
MeshSet(name="allElements", elements=[0, 1, 2, 3])
```

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Set identifier |
| `elements` | `list[int]` | Element indices (auto-sorted, deduplicated) |

### `MeshRegion`

```python
MeshRegion(material_index=0, set_index=1)
```

| Field | Type | Description |
|-------|------|-------------|
| `material_index` | `int` | Index into `VegFile.materials` |
| `set_index` | `int` | Index into `VegFile.sets` |

### `VegFile`

```python
veg = VegFile(
    mesh_data=tet_data,
    materials=[soft, stiff],
    sets=[MeshSet("all", [0,1]), MeshSet("soft", [0]), MeshSet("stiff", [1])],
    regions=[MeshRegion(0, 1), MeshRegion(1, 2)],
)
```

| Field | Type | Description |
|-------|------|-------------|
| `mesh_data` | `TetMeshData \| CubicMeshData` | Mesh geometry |
| `materials` | `list[MaterialLike]` | Material payloads |
| `sets` | `list[MeshSet]` | Element sets |
| `regions` | `list[MeshRegion]` | Region assignments |

| Classmethod | Description |
|-------------|-------------|
| `VegFile.from_single_material(mesh_data, material)` | Convenience for homogeneous meshes |

| Method | Description |
|--------|-------------|
| `first_material()` | Return the sole material (raises if multi-material) |
| `to_volume_regions()` | Expand to `list[(name, material, elements)]` tuples |

---

## `VolumeMesh`

The primary simulation-facing volume mesh. Wraps mesh data + materials + region assignments into a solver-ready object.

### Construction

```python
# Multi-region
volume = VolumeMesh(tet_data, [
    ("soft_region", soft_material, [0, 1, 2]),
    ("stiff_region", stiff_material, [3, 4, 5]),
])

# Single material
volume = VolumeMesh.create_from_single_material(tet_data, material)

# From VegFile
volume = VolumeMesh.from_veg_file(veg_file)
```

Every element must be assigned to exactly one region.

### Properties

| Property | Return | Description |
|----------|--------|-------------|
| `num_vertices` | `int` | Vertex count |
| `num_elements` | `int` | Element count |
| `mesh_type` | `MeshDataType` | `Tet` or `Cubic` |
| `mesh_data` | `TetMeshData \| CubicMeshData` | Underlying geometry |
| `geometry` | same as `mesh_data` | Alias |
| `material` | `MaterialLike` | Single material (raises if multi-material) |
| `material_spec` | same as `material` | Alias |

### Methods

| Method | Return | Description |
|--------|--------|-------------|
| `extract_surface_mesh(triangulate=True)` | `TriMeshData` | Boundary surface |
| `mass_matrix(inflate3dim=True)` | `SparseMatrix` | Consistent mass matrix |
| `to_veg_file()` | `VegFile` | Round-trip to file model |

#### `mass_matrix`

- `inflate3dim=True` (default): shape `(3n, 3n)` — block-diagonal ×3 for displacement DOFs. Standard for solvers and IPC.
- `inflate3dim=False`: shape `(n, n)` — scalar per-vertex mass.

**Consistent mass matrix:** For a volume mesh with $n$ vertices and density $\rho$, the consistent mass matrix $\mathbf{M} \in \mathbb{R}^{n \times n}$ is assembled from element-level contributions:

$$\mathbf{M}^{(e)}_{ij} = \rho \int_{\Omega_e} N_i(\mathbf{x}) \, N_j(\mathbf{x}) \, d\mathbf{x}$$

where $N_i$ are the finite element shape functions. For a uniform tetrahedron, the element mass matrix has diagonal entries $\frac{\rho V_e}{10}$ and off-diagonal entries $\frac{\rho V_e}{20}$. The 3-D inflated form is $\mathbf{M}_{3D} = \mathbf{M} \otimes \mathbf{I}_3$, where $\otimes$ denotes the Kronecker product.

---

## `.veg` I/O

```python
from pypgo.mesh.volume import read_veg, write_veg

veg = read_veg("mesh.veg")       # → VegFile
write_veg("output.veg", veg)     # full round-trip
```

Preserves mesh data, materials, sets, and regions.

---

## Region Validation (private)

`_validate_and_split_regions(regions, num_elements)` ensures:
- Every region has a unique name
- Every element is assigned to exactly one region
- All element indices are in `[0, num_elements)`
