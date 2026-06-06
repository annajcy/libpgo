# `pypgo.mesh.geo.algorithms` — Geometry Algorithms

Pure-Python algorithms that operate on mesh data types. These have no dependency on the C++ geometry facades in `core.py`.

## Connected Components

All functions operate on `TriMeshData`. Edge connectivity shares full edges; vertex connectivity also merges components touching at a single vertex (pinch points).

### `triangle_component_ids`

```python
from pypgo.mesh.geo import triangle_component_ids
ids, sizes = triangle_component_ids(tri_data)
```

Returns `(component_ids, component_sizes)`:
- `component_ids` — `(n_triangles,)` int64, `triID → componentID`
- `component_sizes` — `(n_components,)` int64, unsorted

### `connected_components_by_edge`

```python
from pypgo.mesh.geo import connected_components_by_edge
groups = connected_components_by_edge(tri_data)
```

Returns `list[np.ndarray]` — each element is an int64 array of triangle indices belonging to one component (descending size order).

### `connected_components_by_vertex`

Same as above but using vertex connectivity. On well-formed meshes the two are equivalent; they differ at pinch points.

### `filter_small_components`

```python
from pypgo.mesh.geo import filter_small_components
clean = filter_small_components(tri_data, min_triangles=10, keep_largest=-1)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tri_data` | `TriMeshData` | (required) | Input mesh |
| `min_triangles` | `int` | (required) | Drop components with fewer triangles |
| `keep_largest` | `int` | `-1` | If >0, keep only the N largest after threshold |

Returns a new `TriMeshData` with isolated vertices removed.

### `get_outer_component`

```python
from pypgo.mesh.geo import get_outer_component
outer = get_outer_component(tri_data)
```

Finds the topmost triangle by y-coordinate, performs vertex-connected BFS, and returns that shell. Useful for isolating the outer surface of nested (shell/cavity) meshes.

### `split_components`

```python
from pypgo.mesh.geo import split_components
parts = split_components(tri_data)
```

Convenience: returns `list[TriMeshData]` — each component as a standalone mesh with compacted vertices, ordered by descending triangle count.

---

## Minimum Bounding Sphere

### `minimum_bounding_sphere`

Welzl's randomized algorithm (seed=0 for reproducibility).

```python
from pypgo.mesh.geo import minimum_bounding_sphere
center, radius = minimum_bounding_sphere(mesh_data, padding=0.0)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mesh_data` | `TriMeshData \| TetMeshData \| CubicMeshData` | (required) | Any mesh with `.vertices` and `.num_vertices` |
| `padding` | `float` | `0.0` | Radius multiplier `(1 + padding)` |

Returns `(center, radius)` where `center` is `(3,) float64` and `radius` is `float`.

### Internal Helpers (private)

| Function | Purpose |
|----------|---------|
| `_mbs_contains(center, radius, p)` | Point-in-sphere test |
| `_mbs_from_2(a, b)` | Sphere from 2 points |
| `_mbs_from_3(a, b, c)` | Sphere from 3 points (or None if collinear) |
| `_mbs_from_4(a, b, c, d)` | Sphere from 4 points (or None if coplanar) |
| `_mbs_from_boundary(boundary)` | Minimal sphere enclosing boundary set |
