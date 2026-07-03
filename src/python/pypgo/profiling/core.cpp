#include "core.h"

#include "scopedProfileSection.h"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace pgo
{

void setProfilingEnabled(bool enabled)
{
  Profiling::setProfilingEnabled(enabled);
}

bool isProfilingEnabled()
{
  return Profiling::isProfilingEnabled();
}

void resetProfiling()
{
  Profiling::resetProfileStatistics();
}

namespace
{
nb::dict profileStatToDict(const Profiling::ProfileStat &stat)
{
  nb::dict item;
  item["name"] = nb::str(stat.name.c_str());
  item["local_name"] = nb::str(stat.localName.c_str());
  item["path"] = nb::str(stat.path.c_str());
  item["call_count"] = nb::int_(stat.callCount);
  item["total_seconds"] = nb::float_(stat.totalSeconds);
  item["max_seconds"] = nb::float_(stat.maxSeconds);
  item["total_cpu_seconds"] = nb::float_(stat.totalCpuSeconds);
  item["max_cpu_seconds"] = nb::float_(stat.maxCpuSeconds);
  item["children_seconds"] = nb::float_(stat.childrenSeconds);
  item["self_seconds"] = nb::float_(stat.selfSeconds);
  item["children_cpu_seconds"] = nb::float_(stat.childrenCpuSeconds);
  item["self_cpu_seconds"] = nb::float_(stat.selfCpuSeconds);
  item["avg_cpu_cores"] = nb::float_(
    stat.totalSeconds > 0.0 ? stat.totalCpuSeconds / stat.totalSeconds : 0.0);
  item["max_cpu_cores"] = nb::float_(
    stat.maxSeconds > 0.0 ? stat.maxCpuSeconds / stat.maxSeconds : 0.0);
  item["children_avg_cpu_cores"] = nb::float_(
    stat.childrenSeconds > 0.0 ? stat.childrenCpuSeconds / stat.childrenSeconds : 0.0);
  item["self_avg_cpu_cores"] = nb::float_(
    stat.selfSeconds > 0.0 ? stat.selfCpuSeconds / stat.selfSeconds : 0.0);
  nb::list children;
  for (const Profiling::ProfileStat &child : stat.children)
    children.append(profileStatToDict(child));
  item["children"] = children;
  return item;
}
}  // namespace

nb::dict profilingStats()
{
  return profileStatToDict(Profiling::snapshotProfileStatistics());
}

nb::list profilingCounterStats()
{
  nb::list out;
  for (const Profiling::ProfileCounterStat &stat : Profiling::snapshotProfileCounterStatistics()) {
    nb::dict item;
    item["name"] = nb::str(stat.name.c_str());
    item["sample_count"] = nb::int_(stat.sampleCount);
    item["total"] = nb::int_(stat.total);
    item["max"] = nb::int_(stat.max);
    item["avg"] = nb::float_(stat.avg);
    out.append(item);
  }
  return out;
}

}  // namespace pgo
