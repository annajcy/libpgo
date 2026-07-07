#include <nanobind/nanobind.h>

#include "core.h"

namespace nb = nanobind;

void init_parallel_bindings(nb::module_ &m)
{
  m.def("_parallel_get_worker_limit", &pgo::getWorkerLimit);
  m.def("_parallel_set_worker_limit", &pgo::setWorkerLimit, nb::arg("num_workers"));
  m.def("_parallel_reset_worker_limit", &pgo::resetWorkerLimit);
  m.def("_parallel_runtime_info", &pgo::runtimeInfo);
  m.def("_parallel_supports_cpu_affinity_limit", &pgo::supportsCpuAffinityLimit);
  m.def("_parallel_get_cpu_affinity_limit", &pgo::getCpuAffinityLimit);
  m.def("_parallel_set_cpu_affinity_limit", &pgo::setCpuAffinityLimit, nb::arg("num_cpus"));
  m.def("_parallel_reset_cpu_affinity_limit", &pgo::resetCpuAffinityLimit);
}
