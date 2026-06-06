# `pypgo.mesh.geo.core` — Geometry Facades

Thin Python wrappers around C++ geometry objects. These classes delegate storage and heavy computation to the C++ layer. They are stable glue code that changes infrequently.

## `TriMeshGeo`

Triangle surface mesh geometry facade. Provides NumPy-based derived properties (face areas, normals).

```python
from pypgo.mesh.geo import TriMeshGeo
geo = TriMeshGeo(vertices, triangles)
# or:
geo = TriMeshGeo.from_mesh_data(tri_data)
```

| Property | Return | Description |
|----------|--------|-------------|
| `vertices` | `(N, 3) float64` | Vertex positions |
| `triangles` | `(M, 3) int64` | Triangle indices |
| `num_vertices` | `int` | Vertex count |
| `num_triangles` | `int` | Triangle count |
| `face_areas` | `(M,) float64` | Per-face area (0.5 × |cross(e1, e2)|) |
| `face_normals` | `(M, 3) float64` | Per-face unit normals |
| `vertex_normals` | `(N, 3) float64` | Area-weighted per-vertex normals |

| Method | Description |
|--------|-------------|
| `tri_vtx_id(tri_id, local_id)` | Global vertex index for `(tri, local_vertex)` |
| `to_mesh_data()` | Convert back to `TriMeshData` |

**Derived quantities:** For a triangle with vertices $\mathbf{v}_0, \mathbf{v}_1, \mathbf{v}_2$:

$$\text{area} = \frac{1}{2} \left\| (\mathbf{v}_1 - \mathbf{v}_0) \times (\mathbf{v}_2 - \mathbf{v}_0) \right\|$$

$$\mathbf{n}_{\text{face}} = \frac{(\mathbf{v}_1 - \mathbf{v}_0) \times (\mathbf{v}_2 - \mathbf{v}_0)}{\left\| \cdots \right\|}$$

Per-vertex normals are area-weighted averages of incident face normals:

$$\mathbf{n}_v = \sum_{f \ni v} A_f \cdot \mathbf{n}_f \;\; \big/ \;\; \left\| \sum_{f \ni v} A_f \cdot \mathbf{n}_f \right\|$$

---

## `TetMeshGeo`

Tetrahedral mesh geometry facade.

```python
from pypgo.mesh.geo import TetMeshGeo
geo = TetMeshGeo(vertices, tets)
geo = TetMeshGeo.from_mesh_data(tet_data)
```

| Property | Return | Description |
|----------|--------|-------------|
| `vertices` | `(N, 3) float64` | Vertex positions |
| `tets` | `(M, 4) int64` | Tet indices |
| `num_vertices` | `int` | Vertex count |
| `num_tets` | `int` | Tet count |

---

## `CubicMeshGeo`

Hexahedral mesh geometry facade.

```python
from pypgo.mesh.geo import CubicMeshGeo
geo = CubicMeshGeo(vertices, cubes)
geo = CubicMeshGeo.from_mesh_data(cubic_data)
```

| Property | Return | Description |
|----------|--------|-------------|
| `vertices` | `(N, 3) float64` | Vertex positions |
| `cubes` | `(M, 8) int64` | Cube indices |
| `num_vertices` | `int` | Vertex count |
| `num_cubes` | `int` | Cube count |

---

## `BarycentricEmbedding`

Barycentric interpolation from a `VolumeMesh` onto arbitrary target locations. Finds the containing element and computes barycentric coordinates for each target point.

```python
from pypgo.mesh.geo import BarycentricEmbedding
emb = BarycentricEmbedding(target_locations, volume_mesh)
```

| Property | Type | Description |
|----------|------|-------------|
| `num_target_locations` | `int` | Number of query points |
| `num_element_vertices` | `int` | Vertices per element (4 tet / 8 cubic) |
| `embedding_indices` | `(K, V) int64` | Per-target element vertex indices |
| `embedding_weights` | `(K, V) float64` | Per-target barycentric weights |
| `embedding_elements` | `(K,) int64` | Which element contains each target |
| `interpolation_matrix` | `SparseMatrix` | Shape `(3K, 3N)` |

| Method | Description |
|--------|-------------|
| `deform(volume_disp)` | Apply `(3N,)` or `(N, 3)` displacement to targets → `(K, 3)` |
| `interpolation_matrix_coo()` | Return `(rows, cols, values)` COO triplets |

**Barycentric interpolation:** For a target point $\mathbf{p}$ inside element $e$ with vertices $\mathbf{x}_0, \ldots, \mathbf{x}_{V-1}$, the barycentric weights $\lambda_i$ satisfy:

$$\mathbf{p} = \sum_{i=0}^{V-1} \lambda_i \mathbf{x}_i, \quad \sum_{i=0}^{V-1} \lambda_i = 1, \quad \lambda_i \geq 0$$

The deformed position under displacement field $\mathbf{u}$ is:

$$\mathbf{p}' = \sum_{i=0}^{V-1} \lambda_i (\mathbf{x}_i + \mathbf{u}_i) = \mathbf{p} + \sum_{i=0}^{V-1} \lambda_i \mathbf{u}_i$$

The interpolation matrix $\mathbf{W} \in \mathbb{R}^{3K \times 3N}$ encodes $\lambda_i$ such that $\text{vec}(\mathbf{u}_{\text{target}}) = \mathbf{W} \cdot \text{vec}(\mathbf{u}_{\text{volume}})$.

---

## `SurfaceEmbedding`

Convenience wrapper around `BarycentricEmbedding` for the common case where target locations are vertices of a surface mesh embedded inside a volume mesh.

```python
from pypgo.mesh import SurfaceEmbedding
emb = SurfaceEmbedding(surface_mesh, volume_mesh)
```

| Property | Type | Description |
|----------|------|-------------|
| `rest_surface` | `TriMeshData` | Original surface mesh |
| `interpolation_matrix` | `SparseMatrix` | Shape `(3S, 3V)` |

| Method | Description |
|--------|-------------|
| `displacement(volume_disp)` | Interpolated surface displacements → `(S, 3)` |
| `deform(volume_disp)` | Deformed `TriMeshData` preserving original topology |

---

## `surface_to_volume_interpolation_matrix`

Free function wrapping `BarycentricEmbedding`:

```python
from pypgo.mesh.geo import surface_to_volume_interpolation_matrix
W = surface_to_volume_interpolation_matrix(surface_mesh, volume_mesh)
surf_disp = (W @ vol_disp.ravel()).reshape(-1, 3)
```
