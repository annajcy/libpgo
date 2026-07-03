#include "processMemory.h"

#include "scopedProfileSection.h"

#include <string>

#if defined(__APPLE__)
#  include <mach/mach.h>
#  include <sys/resource.h>
#elif defined(__linux__)
#  include <cstdio>
#  include <sys/resource.h>
#  include <unistd.h>
#elif defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#  include <psapi.h>
#endif

namespace pgo::Profiling
{

ProcessMemoryUsage processMemoryUsage()
{
  ProcessMemoryUsage usage;

#if defined(__APPLE__)
  mach_task_basic_info_data_t taskInfo{};
  mach_msg_type_number_t taskInfoCount = MACH_TASK_BASIC_INFO_COUNT;
  if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
        reinterpret_cast<task_info_t>(&taskInfo), &taskInfoCount) == KERN_SUCCESS) {
    usage.residentBytes = static_cast<std::uint64_t>(taskInfo.resident_size);
  }

  rusage resourceUsage{};
  if (getrusage(RUSAGE_SELF, &resourceUsage) == 0)
    usage.peakResidentBytes = static_cast<std::uint64_t>(resourceUsage.ru_maxrss);
#elif defined(__linux__)
  if (FILE *statm = std::fopen("/proc/self/statm", "r")) {
    unsigned long long totalPages = 0;
    unsigned long long residentPages = 0;
    if (std::fscanf(statm, "%llu %llu", &totalPages, &residentPages) == 2) {
      const long pageSize = sysconf(_SC_PAGESIZE);
      if (pageSize > 0)
        usage.residentBytes = static_cast<std::uint64_t>(residentPages) * static_cast<std::uint64_t>(pageSize);
    }
    std::fclose(statm);
  }

  rusage resourceUsage{};
  if (getrusage(RUSAGE_SELF, &resourceUsage) == 0)
    usage.peakResidentBytes = static_cast<std::uint64_t>(resourceUsage.ru_maxrss) * 1024u;
#elif defined(_WIN32)
  PROCESS_MEMORY_COUNTERS counters{};
  counters.cb = sizeof(counters);
  if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) != FALSE) {
    usage.residentBytes = static_cast<std::uint64_t>(counters.WorkingSetSize);
    usage.peakResidentBytes = static_cast<std::uint64_t>(counters.PeakWorkingSetSize);
  }
#endif

  return usage;
}

ProcessMemoryUsage recordProcessMemoryProfileCounters(std::string_view name)
{
  const ProcessMemoryUsage usage = processMemoryUsage();
  const std::string prefix(name);
  recordProfileCounter(prefix + ".resident_bytes", usage.residentBytes);
  recordProfileCounter(prefix + ".peak_resident_bytes", usage.peakResidentBytes);
  return usage;
}

}  // namespace pgo::Profiling
