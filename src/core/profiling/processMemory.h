#pragma once

#include <cstdint>
#include <string_view>

namespace pgo::Profiling
{

struct ProcessMemoryUsage
{
  std::uint64_t residentBytes = 0;
  std::uint64_t peakResidentBytes = 0;
};

ProcessMemoryUsage processMemoryUsage();
ProcessMemoryUsage recordProcessMemoryProfileCounters(std::string_view name);

}  // namespace pgo::Profiling
