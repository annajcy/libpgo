import numpy as np

def float_vector(name: str, value) -> np.ndarray:
    """Validate and return a contiguous float64 1-D array."""
    arr = np.asarray(value, dtype=np.float64, order="C")
    if arr.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {arr.shape}")
    return arr

def vec3(name: str, value) -> np.ndarray:
    """Validate and return a contiguous float64 3-vector (flat input is reshaped)."""
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be a 3-element array")
    return np.ascontiguousarray(arr)

def vec3_finite(name: str, value) -> np.ndarray:
    """Validate and return a float64 3-vector, additionally requiring finite values."""
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must contain finite values")
    return arr

def vec3_list(name: str, value) -> list[float]:
    """Validate a 3-vector and return it as a Python list of floats."""
    arr = np.ascontiguousarray(value, dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {arr.shape}")
    return arr.tolist()

def int_vector(name: str, value) -> np.ndarray:
    """Validate and return a contiguous int64 1-D array."""
    arr = np.asarray(value, dtype=np.int64, order="C")
    if arr.ndim != 1:
        raise ValueError(f"{name} must be 1-D, got shape {arr.shape}")
    return arr

def float_matrix(name: str, value, columns: int) -> np.ndarray:
    try:
        arr = np.ascontiguousarray(value, dtype=np.float64)
    except (ValueError, TypeError) as e:
        raise ValueError(f"Cannot convert {name} to float64 array: {e}") from e

    if arr.ndim != 2 or arr.shape[1] != columns:
        raise ValueError(f"{name} must be a 2D array with {columns} columns, got shape {arr.shape}")
    return arr

def index_matrix(name: str, value, columns: int, *, num_vertices: int) -> np.ndarray:
    val_arr = np.asarray(value)
    if not np.issubdtype(val_arr.dtype, np.integer):
        raise TypeError(f"{name} must contain integer types, got dtype {val_arr.dtype}")

    try:
        arr = np.ascontiguousarray(value, dtype=np.int64)
    except (ValueError, TypeError) as e:
        raise TypeError(f"Cannot convert {name} to int64 array: {e}") from e

    if arr.ndim != 2 or arr.shape[1] != columns:
        raise ValueError(f"{name} must be a 2D array with {columns} columns, got shape {arr.shape}")

    if arr.size > 0:
        if np.any(arr < 0):
            raise ValueError(f"{name} cannot contain negative indices")
        if np.any(arr >= num_vertices):
            raise ValueError(f"{name} contains out-of-range index (vertex count is {num_vertices})")

    return arr


# -- scalar guards ----------------------------------------------------------

def finite_scalar(name: str, value: float) -> float:
    value = float(value)
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value

def positive_scalar(name: str, value: float) -> float:
    value = finite_scalar(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value

def nonnegative_scalar(name: str, value: float) -> float:
    value = finite_scalar(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return value


# -- named array validators -------------------------------------------------

def vertex_array(name: str, value) -> np.ndarray:
    """Validate and return a contiguous float64 (n, 3) vertex array."""
    arr = np.asarray(value, dtype=np.float64, order="C")
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"{name} must have shape (n, 3), got {arr.shape}")
    return arr

def triangle_array(name: str, value) -> np.ndarray:
    """Validate and return a contiguous int64 (m, 3) triangle-index array."""
    arr = np.asarray(value, dtype=np.int64, order="C")
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"{name} must have shape (m, 3)")
    if arr.size and np.min(arr) < 0:
        raise ValueError(f"{name} must contain non-negative vertex indices")
    return arr
