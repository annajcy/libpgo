#pragma once

#include <optional>

namespace pgo::parallel {

enum class NestedKernelPolicy {
  Suppress,
  Inherit,
};

struct Options {
  int grainSize = 0;
  NestedKernelPolicy nestedKernelPolicy = NestedKernelPolicy::Suppress;
};

struct RuntimeInfo {
  std::optional<int> workerLimit;
  std::optional<int> cpuAffinityLimit;
  std::optional<int> currentCpuAffinityCpus;
  std::optional<int> tbbMaxAllowedParallelism;
};

// Process-wide pgo worker limit. Configuration changes must not overlap native work.
void setWorkerLimit(std::optional<int> numWorkers);
std::optional<int> workerLimit();
RuntimeInfo runtimeInfo();

bool supportsCpuAffinityLimit();
void setCpuAffinityLimit(std::optional<int> numCpus);
std::optional<int> cpuAffinityLimit();

class ScopedWorkerLimit {
public:
  explicit ScopedWorkerLimit(std::optional<int> numWorkers);
  ~ScopedWorkerLimit();

  ScopedWorkerLimit(const ScopedWorkerLimit &) = delete;
  ScopedWorkerLimit &operator=(const ScopedWorkerLimit &) = delete;

private:
  std::optional<int> previousNumWorkers_;
};

class ScopedCpuAffinityLimit {
public:
  explicit ScopedCpuAffinityLimit(std::optional<int> numCpus);
  ~ScopedCpuAffinityLimit();

  ScopedCpuAffinityLimit(const ScopedCpuAffinityLimit &) = delete;
  ScopedCpuAffinityLimit &operator=(const ScopedCpuAffinityLimit &) = delete;

private:
  std::optional<int> previousNumCpus_;
};

}  // namespace pgo::parallel
