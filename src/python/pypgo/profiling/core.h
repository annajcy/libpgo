#pragma once

#include <nanobind/nanobind.h>

namespace pgo
{

void setProfilingEnabled(bool enabled);
bool isProfilingEnabled();
void resetProfiling();
nanobind::dict profilingStats();
nanobind::list profilingCounterStats();

}  // namespace pgo
