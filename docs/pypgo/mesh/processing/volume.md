# `pypgo.mesh.processing.volume` — Volume Mesh Generation

## Backend Availability

Build-time feature checks — these reflect CMake configuration, not runtime detection.

```python
from pypgo.mesh import has_tetwild, has_cgal_remesher

if has_tetwild():
    tets = tet_mesher(surface, backend="tetwild")

if has_cgal_remesher():
    smoothed = cgal_smooth(surface)
```

| Function | CMake Flag | Description |
|----------|-----------|-------------|
| `has_tetwild()` | `PGO_ENABLE_TETWILD` | Wild tetrahedralization available |
| `has_cgal_remesher()` | `PGO_ENABLE_CGAL` | CGAL surface processing available |

---

## `cubic_mesher`

Voxelize a closed triangle surface into a `CubicMeshData`.

```python
from pypgo.mesh import read_obj, cubic_mesher
cubes = cubic_mesher(read_obj("bunny.obj"), resolution=32)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tri_data` | `TriMeshData` | (required) | Watertight input surface |
| `resolution` | `int` | (required) | Grid resolution per axis |
| `E` | `float` | `1e6` | Material Young's modulus |
| `nu` | `float` | `0.45` | Material Poisson's ratio |
| `density` | `float` | `1000.0` | Material density |

Returns `CubicMeshData`.

---

## `tet_mesher`

Tetrahedralize a closed triangle surface.

```python
from pypgo.mesh import read_obj, tet_mesher
tets = tet_mesher(read_obj("bunny.obj"), backend="tetgen")
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tri_data` | `TriMeshData` | (required) | Watertight input surface |
| `backend` | `str` | `"tetgen"` | `"tetgen"` or `"tetwild"` |
| `config` | `dict \| None` | `None` | Backend-specific options |

**tetgen backend** — `config` keys:

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `command` | `str` | `"pq1.414"` | tetgen command-line switches |

**tetwild backend** — `config` keys:

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `lr` | `float` | `0.05` | Edge length ratio |
| `la` | `float` | `0.0` | Edge length (absolute; if set, overrides `lr`) |
| `epsr` | `float` | `0.001` | Envelope ratio |
| `stop_energy` | `float` | `10.0` | Stopping energy threshold |
| `max_threads` | `int` | `0` | Thread count (0 = auto) |

Returns `TetMeshData`.

---

## `VolumetricMeshInfo` & `volume_mesh_info`

Geometric summary of a volume mesh.

```python
from pypgo.mesh import volume_mesh_info
info = volume_mesh_info(mesh)  # accepts TetMeshData, CubicMeshData, VolumeMesh, VegFile
```

| Field | Type | Description |
|-------|------|-------------|
| `num_vertices` | `int` | Vertex count |
| `num_elements` | `int` | Element count |
| `num_element_vertices` | `int` | Vertices per element (4 or 8) |
| `total_volume` | `float` | Sum of element volumes |
| `center_of_mass` | `(3,) float64` | Volume-weighted centroid |

`str(info)` produces a formatted multi-line report.
