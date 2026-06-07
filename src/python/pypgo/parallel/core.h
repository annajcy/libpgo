#pragma once

#include <nanobind/nanobind.h>

namespace pgo
{

nanobind::object getNumThreads();
void resetNumThreads();
void setNumThreads(int numThreads);

}  // namespace pgo
