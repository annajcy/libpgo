#pragma once

#include <nanobind/nanobind.h>

#include <optional>

namespace pgo
{

int parallelDefaultConcurrency();
nanobind::dict parallelInitialize(std::optional<int> maxConcurrency);
nanobind::dict parallelRuntimeInfo();

}  // namespace pgo
