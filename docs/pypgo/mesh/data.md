# `pypgo.mesh.data` — Mesh Data Containers

## Overview

`data.py` is the **foundation layer** of `pypgo.mesh`. It defines the three mesh data types and their shared base class. These are pure data carriers — they store vertices and element topology, but have no geometric interpretation (that lives in `geo`). Critically, `data.py` has **zero internal dependencies** on the rest of `pypgo.mesh`, so every other module can safely import from it without circular import risk.

## Class Hierarchy

```
_MeshDataBase                          # Shared interface (not public)
├── TriMeshData(_MeshDataBase)         # Triangle surface mesh
├── TetMeshData(_MeshDataBase)         # Tetrahedral volume mesh
└── CubicMeshData(_MeshDataBase)       # Hexahedral volume mesh
```

## API Reference

---

### `TriMeshData`

Triangle surface mesh data container.

**Constructor:**

```python
TriMeshData(vertices, elements)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `vertices` | array-like | `(N, 3)` vertex positions |
| `elements` | array-like | `(M, 3)` triangle indices |

**Properties:**

| Property | Return | Description |
|----------|--------|-------------|
| `vertices` | `np.ndarray` (N, 3) float64 | Vertex positions |
| `elements` | `np.ndarray` (M, 3) int64 | Triangle indices |
| `num_vertices` | `int` | Vertex count |
| `num_elements` | `int` | Triangle count |
| `mesh_type` | `MeshDataType` | Always `MeshDataType.Tri` |
| `bbox` | `tuple[ndarray, ndarray]` | (min_corner, max_corner) |

**Methods:**

| Method | Description |
|--------|-------------|
| `element_vtx_id(element_id, local_vertex_id)` | Global vertex index for element-local vertex |
| `take_elements(indices)` | Return new mesh with subset of elements |
| `TriMeshData.concatenate(meshes)` | Concatenate multiple meshes (classmethod) |

---

### `TetMeshData`

Tetrahedral volume mesh data container.

**Constructor:**

```python
TetMeshData(vertices, elements)  # elements shape (M, 4)
```

**Properties (in addition to TriMeshData properties):**

| Property | Return | Description |
|----------|--------|-------------|
| `volume` | `float` | Total volume (sum of tet volumes) |
| `center_of_mass` | `np.ndarray` (3,) | Volume-weighted center of mass |

**Volume computation:** For a tetrahedron with vertices $\mathbf{v}_0, \mathbf{v}_1, \mathbf{v}_2, \mathbf{v}_3$, the signed volume is:

$$V = \frac{1}{6} \left| \det\begin{pmatrix} \mathbf{v}_1 - \mathbf{v}_0 & \mathbf{v}_2 - \mathbf{v}_0 & \mathbf{v}_3 - \mathbf{v}_0 \end{pmatrix} \right|$$

The center of mass is the volume-weighted average of element centroids:

$$\mathbf{c} = \frac{\sum_i V_i \cdot \bar{\mathbf{v}}_i}{\sum_i V_i}, \quad \bar{\mathbf{v}}_i = \frac{1}{4}\sum_{j=0}^3 \mathbf{v}_{ij}$$

---

### `CubicMeshData`

Hexahedral (cubic) volume mesh data container.

**Constructor:**

```python
CubicMeshData(vertices, elements)  # elements shape (M, 8)
```

**Properties:** Same as `TetMeshData` (`volume`, `center_of_mass`).

**Cube-to-tet decomposition:** Each hexahedron (8 vertices) is split into 5 tetrahedra:

$$\begin{aligned}
T_0 &= (0,1,3,4) \\
T_1 &= (1,2,3,6) \\
T_2 &= (1,3,4,6) \\
T_3 &= (1,4,5,6) \\
T_4 &= (3,4,6,7)
\end{aligned}$$

where numbers refer to local vertex indices. Volume and center of mass are computed over the combined set of decomposed tetrahedra.

---

### `MeshDataType`

Enum re-exported from `_core`:

```python
from pypgo.mesh import MeshDataType
# MeshDataType.Tri, MeshDataType.Tet, MeshDataType.Cubic
```

## Usage Examples

```python
import numpy as np
from pypgo.mesh import TriMeshData, TetMeshData, CubicMeshData

# Triangle surface mesh
tri = TriMeshData(
    [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]],
    [[0, 1, 2], [1, 3, 2]],
)
print(tri.bbox)          # (array([0., 0., 0.]), array([1., 1., 0.]))

# Tetrahedral volume mesh
tet = TetMeshData(
    [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
    [[0, 1, 2, 3]],
)
print(tet.volume)        # 0.166666...
print(tet.center_of_mass)  # [0.25 0.25 0.25]

# Subset and concatenate
subset = tri.take_elements([0])
merged = TriMeshData.concatenate([tri, subset])
```

## Input Validation

All constructors validate inputs through `float_matrix` / `index_matrix` (see [`_arrays.md`](../_arrays.md)):
- Vertices are converted to C-contiguous `float64`
- Elements are converted to non-negative `int64` with bounds checking
- Descriptive `ValueError` / `TypeError` on invalid input

## Internal Helpers

These are not part of the public API but are used across the mesh package:

| Function | Purpose |
|----------|---------|
| `_array_from_core(core_obj, method, cols, dtype)` | Extract numpy array from C++ object |
| `_wrap_mesh_data_core(core_obj)` | Wrap C++ mesh data into Python type |
| `_tet_volumes(points)` | Compute volumes for an array of tets |
