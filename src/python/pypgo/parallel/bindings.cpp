#include <nanobind/nanobind.h>

#include "core.h"

namespace nb = nanobind;

void init_parallel_bindings(nb::module_ &m)
{
  m.def("_parallel_get_num_threads", &pgo::getNumThreads);
  m.def("_parallel_set_num_threads", &pgo::setNumThreads, nb::arg("num_threads"));
  m.def("_parallel_reset_num_threads", &pgo::resetNumThreads);
  m.def("_parallel_runtime_info", &pgo::runtimeInfo);
  m.def("_parallel_supports_cpu_affinity_limit", &pgo::supportsCpuAffinityLimit);
  m.def("_parallel_get_cpu_affinity_limit", &pgo::getCpuAffinityLimit);
  m.def("_parallel_set_cpu_affinity_limit", &pgo::setCpuAffinityLimit, nb::arg("num_cpus"));
  m.def("_parallel_reset_cpu_affinity_limit", &pgo::resetCpuAffinityLimit);
}
