#include <nanobind/nanobind.h>

#include "core.h"

namespace nb = nanobind;

void init_dense_bindings(nb::module_ &m)
{
  m.def("_test_vector_sum", &pgo::vector_sum);
  m.def("_test_vector_roundtrip", &pgo::vector_roundtrip);
  m.def("_test_matrix_sum", &pgo::matrix_sum);
  m.def("_test_matrix_roundtrip", &pgo::matrix_roundtrip);
  m.def("_test_fixed_3x3_trace", &pgo::fixed_3x3_trace);
  m.def("_test_gil_released_sleep_ms", &pgo::gil_released_sleep_ms);
}
