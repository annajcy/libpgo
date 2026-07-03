#include "threadRuntime.h"

#include "scopedProfileSection.h"

#include <string>

#if defined(__linux__)
#  include <dirent.h>
#endif

namespace pgo::Profiling
{
namespace
{
std::string threadRuntimeCounterName(std::string_view name)
{
  std::string counterName("thread_runtime.");
  counterName.append(name.data(), name.size());
  counterName.append(".nlwp");
  return counterName;
}
}  // namespace

std::optional<std::uint64_t> currentProcessThreadCount()
{
#if defined(__linux__)
  DIR *dir = opendir("/proc/self/task");
  if (!dir)
    return std::nullopt;

  std::uint64_t count = 0;
  while (dirent *entry = readdir(dir)) {
    if (entry->d_name[0] == '.')
      continue;
    count++;
  }

  closedir(dir);
  return count;
#else
  return std::nullopt;
#endif
}

void recordThreadRuntimeSample(std::string_view name)
{
  const std::optional<std::uint64_t> threadCount = currentProcessThreadCount();
  if (!threadCount.has_value())
    return;

  recordProfileCounter(threadRuntimeCounterName(name), *threadCount);
}

ScopedThreadRuntimePhase::ScopedThreadRuntimePhase(std::string_view name)
  : name_(name)
{
  if (!isProfilingEnabled())
    return;

  active_ = true;
  recordThreadRuntimeSample(name_);
}

ScopedThreadRuntimePhase::~ScopedThreadRuntimePhase()
{
  if (!active_)
    return;

  recordThreadRuntimeSample(name_);
}

}  // namespace pgo::Profiling
