#pragma once

#include <nanobind/nanobind.h>

#include <optional>

namespace pgo
{

int parallelInitialize(std::optional<int> maxConcurrency);

}  // namespace pgo
