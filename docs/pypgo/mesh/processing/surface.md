# `pypgo.mesh.processing.surface` — Surface Repair & Diagnostics

## Quality Diagnostics

### `check_surface_quality`

Comprehensive surface mesh defect report.

```python
from pypgo.mesh import check_surface_quality, QualityReport
report = check_surface_quality(mesh, short_edge_threshold=1e-6, degenerate_area_threshold=1e-12)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tri_data` | `TriMeshData` | (required) | Surface to inspect |
| `short_edge_threshold` | `float` | `0.0` | Edges shorter than this are flagged |
| `degenerate_area_threshold` | `float` | `1e-12` | Triangles with area ≤ threshold are degenerate |

**Returns `QualityReport`:**

| Field | Type | Description |
|-------|------|-------------|
| `is_clean` | `bool` | `True` if no defects |
| `degenerate_tris` | `list[int]` | Zero-area (collinear/duplicate-vertex) triangle indices |
| `short_edges` | `list[tuple[int, int]]` | Edge pairs with length < threshold |
| `non_manifold_edges` | `list[tuple[int, int]]` | Unordered edges shared by >2 triangles |
| `flipped_tris` | `list[int]` | Triangles with inconsistent winding vs. neighbour |
| `has_self_intersections` | `bool` | Any non-adjacent intersecting triangles |

**Flipped triangle detection:** For two adjacent triangles sharing edge $(u, v)$, if both triangles list the edge in the same direction $(u \to v)$, their winding is inconsistent — one must be flipped. The algorithm groups edges by unordered pair $\{u, v\}$ and flags any face that duplicates an already-seen directed edge $(u, v)$.

**Self-intersection detection:** Delegates to C++ (`_core.check_self_intersections`).

**Private helpers** (not public API):

| Function | Purpose |
|----------|---------|
| `_edge_records` | Build `(key, directed_edge, face_id)` records |
| `_short_edges` | Filter edges by length |
| `_flipped_tris` | Detect inconsistent winding |

---

## Surface Cleanup

### `remove_isolated_vertices`

```python
from pypgo.mesh import remove_isolated_vertices
clean = remove_isolated_vertices(tri_data)
```

Removes vertices not referenced by any triangle. Pure mesh operation — no CGAL dependency. Returns `TriMeshData`.

### `merge_close_vertices`

```python
from pypgo.mesh import merge_close_vertices, MergeCloseVerticesResult
result = merge_close_vertices(tri_data, eps=1e-5)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tri_data` | `TriMeshData` | (required) | Input surface |
| `eps` | `float \| None` | `None` | Merge distance (auto if None) |

Requires CGAL. Returns `MergeCloseVerticesResult`:

| Field | Type | Description |
|-------|------|-------------|
| `surface` | `TriMeshData` | Cleaned mesh |
| `merged_vertices` | `int` | Number of vertices merged |
| `eps` | `float` | Actual merge distance used |

### `raw_surface_cleanup`

Conservative degenerate-triangle removal preserving topology.

```python
from pypgo.mesh import raw_surface_cleanup, RawSurfaceCleanupResult
result = raw_surface_cleanup(
    mesh,
    expected_components=1,
    short_edge_threshold=1e-5,
    max_passes=3,
    max_collapses=10000,
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tri_data` | `TriMeshData` | (required) | Input surface |
| `expected_components` | `int \| None` | `None` | Expected component count (-1 = auto) |
| `short_edge_threshold` | `float` | `1e-5` | Collapse threshold |
| `max_passes` | `int` | `3` | Maximum cleanup passes |
| `max_collapses` | `int` | `10000` | Maximum edge collapses |

Requires CGAL. Returns `RawSurfaceCleanupResult(surface, report)` where `report` is a `RawSurfaceCleanupReport` with before/after stats and per-step acceptance/rejection counts.

**Algorithm** (`cgalInterface.cpp:cleanupRawSurfaceMesh`):

```
1. Compute initial stats (vertices, triangles, invalid faces, components, manifold)
   → RawSurfaceCleanupReport.before

2. For each pass (up to max_passes):
   ├─ Collect all degenerate/invalid triangles
   │  Degenerate = hasDegenerateGeometry (zero area, collapsed edges, etc.)
   │
   ├─ If none found → break (cleanup done)
   │
   ├─ Deletion step:
   │  ├─ Build candidate: remove invalid triangles + compact vertices
   │  ├─ Acceptance gate:
   │  │  (a) candidate.invalidTriangles < current.invalidTriangles  ("did it help?")
   │  │  (b) topologyGatePasses: manifold AND correct component count AND
   │  │      zero boundary/non-manifold edges                        ("did it break topology?")
   │  ├─ Accept → mesh = candidate, acceptedDeletions++
   │  └─ Reject → rejectedByInvalidCount++ or rejectedByTopology++
   │
   ├─ Collapse step (if acceptedCollapses < maxCollapses):
   │  ├─ Find short edges (< threshold) on invalid triangles
   │  ├─ Build candidate: collapse edges + compact
   │  ├─ Same acceptance gate as deletion step
   │  ├─ Accept → mesh = candidate, acceptedCollapses++
   │  └─ Reject → rejectedByTopology++
   │
   └─ If no change this pass → break

3. Compute final stats → RawSurfaceCleanupReport.after
   topologyPreserved = topologyGatePasses(after, expectedComponents)
   cleanupComplete   = (after.invalidTriangles == 0)
```

The topology gate is conservative: it rejects any change that would break manifoldness, alter the component count, or introduce boundary/non-manifold edges. This means the algorithm will leave the mesh unchanged rather than risk corruption.

---

## CGAL Surface Operations

All require `has_cgal_remesher() == True`. All accept and return `TriMeshData`.

### `cgal_smooth`

```python
from pypgo.mesh import cgal_smooth
smoothed = cgal_smooth(mesh, num_iter=10, sharp_angle=60.0)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_iter` | `int` | `10` | Smoothing iterations |
| `sharp_angle` | `float` | `180.0` | Feature edge threshold (degrees) |

Edges with dihedral angle > `sharp_angle` are treated as feature edges and kept fixed.

### `cgal_isotropic_remesh`

```python
from pypgo.mesh import cgal_isotropic_remesh
remeshed = cgal_isotropic_remesh(mesh, target_edge_length=0.01, num_iter=10, sharp_angle=180.0)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `target_edge_length` | `float` | (required) | Target uniform edge length (absolute) |
| `num_iter` | `int` | `10` | Remeshing iterations |
| `sharp_angle` | `float` | `180.0` | Feature edge threshold |

### `cgal_simplify`

```python
from pypgo.mesh import cgal_simplify
simple = cgal_simplify(mesh, target_ratio=0.1)
```

Collapses edges until approximately `target_ratio` of original edges remain.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `target_ratio` | `float` | (required) | Fraction of edges to keep (0–1) |

### `cgal_repair_self_intersections`

```python
from pypgo.mesh import cgal_repair_self_intersections
repaired, all_fixed = cgal_repair_self_intersections(mesh, method="autorefine")
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `method` | `str` | `"autorefine"` | Repair strategy (see below) |

**Method descriptions** (from `cgalInterface.cpp`):

| Method | CGAL function | Behavior |
|--------|--------------|----------|
| `"autorefine"` (default) | `autorefine_and_remove_self_intersections` | **Two-pass:** first splits intersecting triangles via edge refinement, then volumetrically removes any remaining self-intersections. Most thorough, handles complex cases. |
| `"autorefine-only"` | `autorefine` | **Refinement only:** splits intersecting triangles but does NOT perform volumetric removal. Returns `all_fixed=False` if any self-intersections remain after refinement. |
| `"remove"` | `remove_self_intersections` | **Volumetric removal only:** removes self-intersections via CGAL's corefinement-based approach without prior edge refinement. Faster for simple cases where triangles are large enough. |

Returns `(repaired_mesh, all_fixed)` where `all_fixed` is `True` when no self-intersections remain after repair.
