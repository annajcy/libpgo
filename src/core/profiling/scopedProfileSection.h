#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace pgo::Profiling
{

struct ProfileStat
{
  std::string name;
  std::string localName;
  std::string path;
  std::uint64_t callCount = 0;
  double totalSeconds = 0.0;
  double maxSeconds = 0.0;
  double totalCpuSeconds = 0.0;
  double maxCpuSeconds = 0.0;
  double childrenSeconds = 0.0;
  double selfSeconds = 0.0;
  double childrenCpuSeconds = 0.0;
  double selfCpuSeconds = 0.0;
  std::vector<ProfileStat> children;
};

struct ProfileCounterStat
{
  std::string name;
  std::uint64_t sampleCount = 0;
  std::uint64_t total = 0;
  std::uint64_t max = 0;
  double avg = 0.0;
};

void setProfilingEnabled(bool enabled);
bool isProfilingEnabled();

void resetProfileStatistics();
ProfileStat snapshotProfileStatistics();
void recordProfileCounter(std::string_view name, std::uint64_t value);
std::vector<ProfileCounterStat> snapshotProfileCounterStatistics();

class ScopedProfileSection
{
public:
  explicit ScopedProfileSection(std::string_view name);
  ~ScopedProfileSection();

  ScopedProfileSection(const ScopedProfileSection &) = delete;
  ScopedProfileSection &operator=(const ScopedProfileSection &) = delete;
  ScopedProfileSection(ScopedProfileSection &&) = delete;
  ScopedProfileSection &operator=(ScopedProfileSection &&) = delete;

private:
  using Clock = std::chrono::steady_clock;

  Clock::time_point start_;
  double startCpuSeconds_ = 0.0;
  std::size_t stackIndex_ = 0;
  bool active_ = false;
};

}  // namespace pgo::Profiling
