#include <nanobind/nanobind.h>

#include "core.h"

namespace nb = nanobind;

void init_profiling_bindings(nb::module_ &m)
{
  m.def("_profiling_set_enabled", &pgo::setProfilingEnabled, nb::arg("enabled"));
  m.def("_profiling_is_enabled", &pgo::isProfilingEnabled);
  m.def("_profiling_reset", &pgo::resetProfiling);
  m.def("_profiling_stats", &pgo::profilingStats);
  m.def("_profiling_counter_stats", &pgo::profilingCounterStats);
}
