# `pypgo._arrays` — NumPy Array Validation

## Overview

`_arrays` is pypgo's **internal guard module**. It provides two lightweight functions that validate and convert numpy arrays at the Python → C++ boundary. These sit at the entry point of every mesh constructor, ensuring arrays passed to the C++ binding layer have the correct `dtype`, shape, memory layout, and value range.

## Motivation

- **Strict C++ boundary requirements:** pypgo's C++ backend expects C-contiguous `float64` (vertex positions) and `int64` (topology indices) arrays. Python callers may pass `float32`, lists, non-contiguous views, or other types.
- **Fail early, fail clearly:** Catching malformed arrays in Python with descriptive errors is far better than hitting a segfault or cryptic error deep in C++.
- **Minimal dependencies:** pypgo's only hard dependency is numpy. For two 15-line functions, pulling in pydantic or another validation framework would be overkill.

## Comparison with pydantic / dataclass

| Dimension | `_arrays` | pydantic | dataclass |
|-----------|-----------|----------|-----------|
| Validates | numpy arrays (dtype, shape, C-contiguity, value range) | Python structured data (dict, str, int, etc.) | Nothing (type annotations only) |
| Dependencies | numpy only | `pydantic` + `pydantic-core` (Rust) | stdlib |
| Runtime overhead | Minimal (O(1) attr checks + optional O(n) range scan) | Moderate (deserialization + model instantiation) | Near zero |
| Use case | C++ boundary conversion layer | API input validation, config parsing | Data containers |

## API Reference

---

### `float_matrix(name: str, value, columns: int) -> np.ndarray`

Converts arbitrary input to a `float64` 2-D C-contiguous numpy array and validates the column count.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `name` | `str` | Variable name, used in error messages |
| `value` | array-like | Input data (list, ndarray, tensor, etc.) |
| `columns` | `int` | Expected number of columns |

**Validation steps:**

1. Convertible via `np.ascontiguousarray(value, dtype=np.float64)`
2. Must be 2-D (`ndim == 2`)
3. `shape[1]` must equal `columns`

**Returns:** `(N, columns)` `float64` C-contiguous array.

**Raises:**
- `ValueError` — not convertible to float64, or shape mismatch

**Usage sites:**

```python
# pypgo/mesh/geo.py — TriMeshGeo, TetMeshGeo, CubicMeshGeo constructors
v_arr = float_matrix("vertices", vertices, 3)  # (N_v, 3) float64
```

---

### `index_matrix(name: str, value, columns: int, *, num_vertices: int) -> np.ndarray`

Converts input to an `int64` 2-D C-contiguous numpy array and validates column count, non-negativity, and vertex index bounds.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `name` | `str` | Variable name, used in error messages |
| `value` | array-like | Input data |
| `columns` | `int` | Expected number of columns (indices per element) |
| `num_vertices` | `int` | Total vertex count, used for range validation |

**Validation steps:**

1. Input dtype must be an integer subtype (`np.issubdtype`)
2. Convertible via `np.ascontiguousarray(value, dtype=np.int64)`
3. Must be 2-D, `shape[1]` must equal `columns`
4. All values ≥ 0 (no negative indices)
5. All values < `num_vertices` (no out-of-bounds indices)

> **Note:** Range validation is skipped when `arr.size == 0` (empty topology — no elements).

**Returns:** `(M, columns)` `int64` C-contiguous array.

**Raises:**
- `TypeError` — non-integer dtype or not convertible to int64
- `ValueError` — shape mismatch, negative indices, or out-of-range indices

**Usage sites:**

```python
# pypgo/mesh/geo.py — all mesh constructors
t_arr = index_matrix("triangles", triangles, 3, num_vertices=v_arr.shape[0])  # tri faces
t_arr = index_matrix("tets", tets, 4, num_vertices=v_arr.shape[0])           # tetrahedra
c_arr = index_matrix("cubes", cubes, 8, num_vertices=v_arr.shape[0])         # hexahedra
```

## Call Chain

```
User code
  │  vertices (list / np.float32 / whatever)
  ▼
float_matrix("vertices", vertices, 3)
  │  convert + validate
  ▼
(N, 3) np.float64 C-contiguous
  │  .ravel().tolist()
  ▼
_core.create_tri_mesh_geo(...)  # C++ constructor
```

`index_matrix` follows the same pattern for `triangles` / `tets` / `cubes`.

## Internal Status

The module is named with a leading underscore (`_arrays`), marking it as an internal implementation detail, not part of the public API. External users should interact through high-level interfaces like `TriMeshGeo(vertices, triangles)` and should not call `float_matrix` / `index_matrix` directly.
