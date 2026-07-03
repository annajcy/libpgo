#pragma once

#include <memory>
#include <optional>

namespace pgo::parallel {

enum class Backend {
  Auto,
  Serial,
  TBB,
  OpenMP,
};

enum class NestedKernelPolicy {
  Suppress,
  Inherit,
};

struct Options {
  Backend backend = Backend::Auto;
  int grainSize = 0;
  NestedKernelPolicy nestedKernelPolicy = NestedKernelPolicy::Suppress;
};

bool isBackendAvailable(Backend backend);

struct RuntimeInfo {
  std::optional<int> threadLimit;
  std::optional<int> cpuAffinityLimit;
  std::optional<int> currentCpuAffinityCpus;
  std::optional<int> tbbMaxAllowedParallelism;
  std::optional<int> eigenNumThreads;
  std::optional<int> openMPMaxThreads;
  std::optional<int> mklMaxThreads;
  std::optional<int> mklEffectiveThreadLimit;
  std::optional<int> mklPardisoMaxThreads;
  std::optional<int> openBLASNumThreads;
};

// Process-wide runtime thread limit. Configuration changes must not overlap native work.
void setThreadLimit(std::optional<int> numThreads);
std::optional<int> threadLimit();
RuntimeInfo runtimeInfo();

bool supportsCpuAffinityLimit();
void setCpuAffinityLimit(std::optional<int> numCpus);
std::optional<int> cpuAffinityLimit();

class ScopedMklThreadLimit {
public:
  explicit ScopedMklThreadLimit(std::optional<int> numThreads);
  ~ScopedMklThreadLimit();

  ScopedMklThreadLimit(const ScopedMklThreadLimit &) = delete;
  ScopedMklThreadLimit &operator=(const ScopedMklThreadLimit &) = delete;

private:
  std::shared_ptr<void> threadingControl_;
  int previousNumThreads_ = 0;
  bool shouldRestore_ = false;
};

class ScopedNestedThreadLimit {
public:
  ScopedNestedThreadLimit();
  ~ScopedNestedThreadLimit();

  ScopedNestedThreadLimit(const ScopedNestedThreadLimit &) = delete;
  ScopedNestedThreadLimit &operator=(const ScopedNestedThreadLimit &) = delete;

private:
  int previousOpenMPThreads_ = 0;
  int previousMKLThreads_ = 0;
  bool shouldRestoreOpenMP_ = false;
  bool shouldRestoreMKL_ = false;
};

class ScopedThreadLimit {
public:
  explicit ScopedThreadLimit(std::optional<int> numThreads);
  ~ScopedThreadLimit();

  ScopedThreadLimit(const ScopedThreadLimit &) = delete;
  ScopedThreadLimit &operator=(const ScopedThreadLimit &) = delete;

private:
  std::optional<int> previousNumThreads_;
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
