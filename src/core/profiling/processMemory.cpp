#include "processMemory.h"

#include "scopedProfileSection.h"

#include <string>

#if defined(__APPLE__)
#  include <mach/mach.h>
#  include <sys/resource.h>
#elif defined(__linux__)
#  include <cstddef>
#  include <cstdio>
#  include <cstring>
#elif defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#  include <psapi.h>
#endif

namespace pgo::Profiling
{

#if defined(__linux__)
namespace
{

bool parseProcStatusMemoryLine(const char *line, const char *key, std::uint64_t &bytes)
{
  const std::size_t keyLength = std::strlen(key);
  if (std::strncmp(line, key, keyLength) != 0 || line[keyLength] != ':')
    return false;

  unsigned long long valueKiB = 0;
  if (std::sscanf(line + keyLength + 1, "%llu kB", &valueKiB) != 1)
    return false;

  bytes = static_cast<std::uint64_t>(valueKiB) * 1024u;
  return true;
}

}  // namespace
#endif

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
  if (FILE *status = std::fopen("/proc/self/status", "r")) {
    char line[256];
    while (std::fgets(line, sizeof(line), status)) {
      parseProcStatusMemoryLine(line, "VmRSS", usage.residentBytes);
      parseProcStatusMemoryLine(line, "VmHWM", usage.peakResidentBytes);
    }
    std::fclose(status);
  }

  if (usage.peakResidentBytes > 0 && usage.residentBytes > usage.peakResidentBytes)
    usage.peakResidentBytes = usage.residentBytes;
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
