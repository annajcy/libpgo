#include "scopedProfileSection.h"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#  include <sys/resource.h>
#endif

namespace pgo::Profiling
{
namespace
{
struct AggregatedProfileStat
{
  std::string name;
  std::string localName;
  std::string path;
  std::string parentKey;
  std::uint64_t callCount = 0;
  double totalSeconds = 0.0;
  double maxSeconds = 0.0;
  double totalCpuSeconds = 0.0;
  double maxCpuSeconds = 0.0;
  double childrenSeconds = 0.0;
  double childrenCpuSeconds = 0.0;
};

struct AggregatedProfileCounterStat
{
  std::uint64_t sampleCount = 0;
  std::uint64_t total = 0;
  std::uint64_t max = 0;
};

struct ActiveProfileFrame
{
  std::string key;
  std::string parentKey;
  std::string localName;
  std::string displayName;
  double childSeconds = 0.0;
  double childCpuSeconds = 0.0;
};

std::atomic<bool> gProfilingEnabled{ false };
std::mutex gProfileMutex;
std::unordered_map<std::string, AggregatedProfileStat> gProfileStats;
std::unordered_map<std::string, AggregatedProfileCounterStat> gProfileCounterStats;
thread_local std::vector<ActiveProfileFrame> tProfileStack;

std::string joinDisplayName(std::string_view parentName, std::string_view localName)
{
  if (parentName.empty())
    return std::string(localName);
  std::string name(parentName);
  name.push_back('.');
  name.append(localName);
  return name;
}

std::string joinInternalKey(std::string_view parentKey, std::string_view localName)
{
  if (parentKey.empty())
    return std::string(localName);
  std::string key(parentKey);
  key.push_back('\x1f');
  key.append(localName);
  return key;
}

double currentProcessCpuSeconds()
{
#if defined(__unix__) || defined(__APPLE__)
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0)
    return 0.0;

  constexpr double usecToSeconds = 1.0e-6;
  const double userSeconds = static_cast<double>(usage.ru_utime.tv_sec) +
    static_cast<double>(usage.ru_utime.tv_usec) * usecToSeconds;
  const double systemSeconds = static_cast<double>(usage.ru_stime.tv_sec) +
    static_cast<double>(usage.ru_stime.tv_usec) * usecToSeconds;
  return userSeconds + systemSeconds;
#else
  return 0.0;
#endif
}

void recordProfileSample(const ActiveProfileFrame &frame,
  double seconds, double cpuSeconds)
{
  std::lock_guard<std::mutex> lock(gProfileMutex);
  AggregatedProfileStat &stat = gProfileStats[frame.key];
  stat.name = frame.displayName;
  stat.localName = frame.localName;
  stat.path = frame.displayName;
  stat.parentKey = frame.parentKey;
  stat.callCount += 1;
  stat.totalSeconds += seconds;
  stat.maxSeconds = std::max(stat.maxSeconds, seconds);
  stat.totalCpuSeconds += cpuSeconds;
  stat.maxCpuSeconds = std::max(stat.maxCpuSeconds, cpuSeconds);
  stat.childrenSeconds += frame.childSeconds;
  stat.childrenCpuSeconds += frame.childCpuSeconds;
}

template<typename Stat>
void sortByName(std::vector<Stat> &stats)
{
  std::sort(stats.begin(), stats.end(),
    [](const Stat &lhs, const Stat &rhs) {
      return lhs.name < rhs.name;
    });
}

ProfileStat makeProfileStat(const AggregatedProfileStat &stat)
{
  const double selfSeconds = std::max(0.0, stat.totalSeconds - stat.childrenSeconds);
  const double selfCpuSeconds = std::max(0.0, stat.totalCpuSeconds - stat.childrenCpuSeconds);
  return ProfileStat{
    .name = stat.name,
    .localName = stat.localName,
    .path = stat.path,
    .callCount = stat.callCount,
    .totalSeconds = stat.totalSeconds,
    .maxSeconds = stat.maxSeconds,
    .totalCpuSeconds = stat.totalCpuSeconds,
    .maxCpuSeconds = stat.maxCpuSeconds,
    .childrenSeconds = stat.childrenSeconds,
    .selfSeconds = selfSeconds,
    .childrenCpuSeconds = stat.childrenCpuSeconds,
    .selfCpuSeconds = selfCpuSeconds,
    .children = {},
  };
}

ProfileStat buildProfileTree()
{
  std::unordered_map<std::string, ProfileStat> nodes;
  nodes.reserve(gProfileStats.size());
  std::unordered_map<std::string, std::vector<std::string>> childKeysByParent;
  childKeysByParent.reserve(gProfileStats.size());

  for (const auto &[key, stat] : gProfileStats) {
    nodes.emplace(key, makeProfileStat(stat));
    childKeysByParent[stat.parentKey].push_back(key);
  }

  for (auto &[_, childKeys] : childKeysByParent) {
    std::sort(childKeys.begin(), childKeys.end(),
      [&nodes](const std::string &lhs, const std::string &rhs) {
        return nodes.at(lhs).name < nodes.at(rhs).name;
      });
  }

  auto buildNode = [&](auto &&self, const std::string &key) -> ProfileStat {
    ProfileStat node = nodes.at(key);
    auto childIt = childKeysByParent.find(key);
    if (childIt != childKeysByParent.end()) {
      node.children.reserve(childIt->second.size());
      for (const std::string &childKey : childIt->second)
        node.children.push_back(self(self, childKey));
    }
    return node;
  };

  ProfileStat root;
  root.name = "root";
  root.localName = "root";
  root.path = "";
  root.callCount = gProfileStats.empty() ? 0u : 1u;

  auto topIt = childKeysByParent.find("");
  if (topIt != childKeysByParent.end()) {
    root.children.reserve(topIt->second.size());
    for (const std::string &childKey : topIt->second) {
      ProfileStat child = buildNode(buildNode, childKey);
      root.totalSeconds += child.totalSeconds;
      root.maxSeconds = std::max(root.maxSeconds, child.maxSeconds);
      root.totalCpuSeconds += child.totalCpuSeconds;
      root.maxCpuSeconds = std::max(root.maxCpuSeconds, child.maxCpuSeconds);
      root.children.push_back(std::move(child));
    }
  }

  root.childrenSeconds = root.totalSeconds;
  root.childrenCpuSeconds = root.totalCpuSeconds;
  root.selfSeconds = 0.0;
  root.selfCpuSeconds = 0.0;
  return root;
}
}  // namespace

void setProfilingEnabled(bool enabled)
{
  gProfilingEnabled.store(enabled, std::memory_order_relaxed);
}

bool isProfilingEnabled()
{
  return gProfilingEnabled.load(std::memory_order_relaxed);
}

void resetProfileStatistics()
{
  std::lock_guard<std::mutex> lock(gProfileMutex);
  gProfileStats.clear();
  gProfileCounterStats.clear();
  tProfileStack.clear();
}

ProfileStat snapshotProfileStatistics()
{
  std::lock_guard<std::mutex> lock(gProfileMutex);
  return buildProfileTree();
}

void recordProfileCounter(std::string_view name, std::uint64_t value)
{
  if (!isProfilingEnabled())
    return;

  std::lock_guard<std::mutex> lock(gProfileMutex);
  AggregatedProfileCounterStat &stat = gProfileCounterStats[std::string(name)];
  stat.sampleCount += 1;
  stat.total += value;
  stat.max = std::max(stat.max, value);
}

std::vector<ProfileCounterStat> snapshotProfileCounterStatistics()
{
  std::lock_guard<std::mutex> lock(gProfileMutex);

  std::vector<ProfileCounterStat> snapshot;
  snapshot.reserve(gProfileCounterStats.size());
  for (const auto &[name, stat] : gProfileCounterStats) {
    snapshot.push_back(ProfileCounterStat{
      .name = name,
      .sampleCount = stat.sampleCount,
      .total = stat.total,
      .max = stat.max,
      .avg = stat.sampleCount > 0u ?
        static_cast<double>(stat.total) / static_cast<double>(stat.sampleCount) :
        0.0,
    });
  }

  sortByName(snapshot);

  return snapshot;
}

ScopedProfileSection::ScopedProfileSection(std::string_view name)
{
  if (!isProfilingEnabled())
    return;

  ActiveProfileFrame frame;
  frame.localName = std::string(name);
  if (!tProfileStack.empty()) {
    frame.parentKey = tProfileStack.back().key;
    frame.displayName = joinDisplayName(tProfileStack.back().displayName, name);
  }
  else {
    frame.displayName = std::string(name);
  }
  frame.key = joinInternalKey(frame.parentKey, name);

  start_ = Clock::now();
  startCpuSeconds_ = currentProcessCpuSeconds();
  stackIndex_ = tProfileStack.size();
  tProfileStack.push_back(std::move(frame));
  active_ = true;
}

ScopedProfileSection::~ScopedProfileSection()
{
  if (!active_)
    return;

  const std::chrono::duration<double> elapsed = Clock::now() - start_;
  const double cpuElapsed = std::max(0.0, currentProcessCpuSeconds() - startCpuSeconds_);

  if (tProfileStack.size() <= stackIndex_)
    return;

  ActiveProfileFrame frame = std::move(tProfileStack.back());
  tProfileStack.pop_back();

  const double seconds = elapsed.count();
  recordProfileSample(frame, seconds, cpuElapsed);
  if (!tProfileStack.empty()) {
    tProfileStack.back().childSeconds += seconds;
    tProfileStack.back().childCpuSeconds += cpuElapsed;
  }
}

}  // namespace pgo::Profiling
