#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>

#include "core.h"

namespace nb = nanobind;

void init_parallel_bindings(nb::module_ &m)
{
  m.def("_parallel_initialize", &pgo::parallelInitialize, nb::arg("max_concurrency") = nb::none());
}
