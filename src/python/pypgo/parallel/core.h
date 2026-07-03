#pragma once

#include <nanobind/nanobind.h>

namespace pgo
{

nanobind::object getNumThreads();
void resetNumThreads();
void setNumThreads(int numThreads);
nanobind::dict runtimeInfo();
bool supportsCpuAffinityLimit();
nanobind::object getCpuAffinityLimit();
void resetCpuAffinityLimit();
void setCpuAffinityLimit(int numCpus);

}  // namespace pgo
