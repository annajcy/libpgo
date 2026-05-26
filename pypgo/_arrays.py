import numpy as np

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
