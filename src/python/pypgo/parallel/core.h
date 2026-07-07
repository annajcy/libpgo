#pragma once

#include <nanobind/nanobind.h>

namespace pgo
{

nanobind::object getWorkerLimit();
void resetWorkerLimit();
void setWorkerLimit(int numWorkers);
nanobind::dict runtimeInfo();
bool supportsCpuAffinityLimit();
nanobind::object getCpuAffinityLimit();
void resetCpuAffinityLimit();
void setCpuAffinityLimit(int numCpus);

}  // namespace pgo
