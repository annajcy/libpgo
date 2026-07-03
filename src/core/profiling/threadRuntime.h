#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace pgo::Profiling
{

std::optional<std::uint64_t> currentProcessThreadCount();
void recordThreadRuntimeSample(std::string_view name);

class ScopedThreadRuntimePhase
{
public:
  explicit ScopedThreadRuntimePhase(std::string_view name);
  ~ScopedThreadRuntimePhase();

  ScopedThreadRuntimePhase(const ScopedThreadRuntimePhase &) = delete;
  ScopedThreadRuntimePhase &operator=(const ScopedThreadRuntimePhase &) = delete;
  ScopedThreadRuntimePhase(ScopedThreadRuntimePhase &&) = delete;
  ScopedThreadRuntimePhase &operator=(ScopedThreadRuntimePhase &&) = delete;

private:
  std::string_view name_;
  bool active_ = false;
};

}  // namespace pgo::Profiling
