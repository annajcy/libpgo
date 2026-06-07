#include <nanobind/nanobind.h>

#include "core.h"

namespace nb = nanobind;

void init_parallel_bindings(nb::module_ &m)
{
  m.def("_parallel_get_num_threads", &pgo::getNumThreads);
  m.def("_parallel_set_num_threads", &pgo::setNumThreads, nb::arg("num_threads"));
  m.def("_parallel_reset_num_threads", &pgo::resetNumThreads);
}
