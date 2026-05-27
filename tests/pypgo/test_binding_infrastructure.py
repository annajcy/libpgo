import gc
import threading
import time

import numpy as np
import pytest

import pypgo as pgo


def test_dense_vector_roundtrip_and_zero_copy_observation():
    values = np.array([1.0, 2.0, 3.5], dtype=np.float64)

    out = pgo._core._test_vector_roundtrip(values)
    assert isinstance(out, np.ndarray)
    assert out.dtype == np.float64
    assert np.allclose(out, values)

    assert pgo._core._test_vector_sum(values) == pytest.approx(6.5)
    values[1] = 10.0
    assert pgo._core._test_vector_sum(values) == pytest.approx(14.5)


def test_dense_matrix_roundtrip_copy_fallback_and_fixed_shape_errors():
    matrix = np.arange(12.0, dtype=np.float64).reshape(3, 4)
    sliced = matrix[:, ::2]

    assert pgo._core._test_matrix_sum(matrix) == pytest.approx(66.0)
    matrix[1, 2] = 100.0
    assert pgo._core._test_matrix_sum(matrix) == pytest.approx(160.0)
    matrix[1, 2] = 6.0

    out = pgo._core._test_matrix_roundtrip(sliced)
    assert isinstance(out, np.ndarray)
    assert out.dtype == np.float64
    assert np.allclose(out, sliced)

    owned = pgo._core._test_matrix_roundtrip(matrix)
    del matrix
    gc.collect()
    assert np.allclose(owned, np.arange(12.0, dtype=np.float64).reshape(3, 4))

    with pytest.raises(ValueError, match="expected shape"):
        pgo._core._test_fixed_3x3_trace(np.ones((2, 3), dtype=np.float64))


def test_gil_is_released_during_core_kernel():
    counter = 0
    running = True

    def worker():
        nonlocal counter
        while running:
            counter += 1
            time.sleep(0)

    thread = threading.Thread(target=worker)
    thread.start()
    try:
        before = counter
        pgo._core._test_gil_released_sleep_ms(150)
        after = counter
    finally:
        running = False
        thread.join(timeout=1.0)

    assert after > before
