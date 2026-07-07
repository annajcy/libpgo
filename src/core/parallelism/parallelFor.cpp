#include "parallelism/parallelFor.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <vector>

#if defined(__linux__) || defined(_WIN32)
#define PGO_PARALLELISM_HAS_CPU_AFFINITY 1
#endif

#ifdef __linux__
#include <dirent.h>
#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#ifdef PGO_PARALLELISM_HAS_TBB
#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#endif

#ifdef PGO_PARALLELISM_HAS_MKL
#include <mkl.h>
#endif

#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
#include <Accelerate/Accelerate.h>
#endif

namespace pgo::parallel {
namespace {

std::mutex &runtimeMutex()
{
  static std::mutex mutex;
  return mutex;
}

using CpuList = std::vector<int>;

void validatePositive(std::optional<int> value, const char *name)
{
  if (value.has_value() && *value <= 0)
    throw std::invalid_argument(std::string(name) + " must be positive or unset.");
}

#ifdef PGO_PARALLELISM_HAS_CPU_AFFINITY

CpuList limitCpuList(const CpuList &cpus, int limit)
{
  CpuList limited = cpus;
  std::sort(limited.begin(), limited.end());
  if (static_cast<int>(limited.size()) > limit)
    limited.resize(static_cast<std::size_t>(limit));
  return limited;
}

#endif

#ifdef __linux__

CpuList currentThreadAffinity()
{
  cpu_set_t mask;
  CPU_ZERO(&mask);
  if (sched_getaffinity(0, sizeof(mask), &mask) != 0)
    throw std::system_error(errno, std::generic_category(), "sched_getaffinity");

  CpuList cpus;
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (CPU_ISSET(cpu, &mask))
      cpus.push_back(cpu);
  }
  return cpus;
}

cpu_set_t makeCpuSet(const CpuList &cpus)
{
  if (cpus.empty())
    throw std::invalid_argument("CPU affinity set must not be empty.");

  cpu_set_t mask;
  CPU_ZERO(&mask);
  for (int cpu : cpus) {
    if (cpu < 0 || cpu >= CPU_SETSIZE)
      throw std::runtime_error("CPU id exceeds CPU_SETSIZE.");
    CPU_SET(cpu, &mask);
  }
  return mask;
}

void setTaskAffinity(pid_t tid, const CpuList &cpus)
{
  cpu_set_t mask = makeCpuSet(cpus);
  if (sched_setaffinity(tid, sizeof(mask), &mask) != 0) {
    if (errno == ESRCH)
      return;
    throw std::system_error(errno, std::generic_category(), "sched_setaffinity");
  }
}

bool parseTid(const char *name, pid_t &tid)
{
  char *end = nullptr;
  errno = 0;
  const long value = std::strtol(name, &end, 10);
  if (errno != 0 || end == name || *end != '\0' || value <= 0)
    return false;
  tid = static_cast<pid_t>(value);
  return true;
}

void setCurrentProcessAffinity(const CpuList &cpus)
{
  DIR *dir = opendir("/proc/self/task");
  if (!dir) {
    setTaskAffinity(0, cpus);
    return;
  }

  while (dirent *entry = readdir(dir)) {
    pid_t tid = 0;
    if (parseTid(entry->d_name, tid))
      setTaskAffinity(tid, cpus);
  }

  closedir(dir);
}

void setCurrentProcessAffinityNoThrow(const CpuList &cpus)
{
  try {
    setCurrentProcessAffinity(cpus);
  }
  catch (...) {
  }
}

#endif

#ifdef _WIN32

CpuList currentThreadAffinity()
{
  DWORD_PTR processMask = 0;
  DWORD_PTR systemMask = 0;
  if (!GetProcessAffinityMask(GetCurrentProcess(), &processMask, &systemMask))
    throw std::system_error(static_cast<int>(GetLastError()), std::system_category(), "GetProcessAffinityMask");

  CpuList cpus;
  constexpr int kBits = static_cast<int>(sizeof(DWORD_PTR) * 8);
  for (int cpu = 0; cpu < kBits; ++cpu) {
    if ((processMask & (DWORD_PTR{1} << cpu)) != 0)
      cpus.push_back(cpu);
  }
  return cpus;
}

DWORD_PTR makeCpuMask(const CpuList &cpus)
{
  if (cpus.empty())
    throw std::invalid_argument("CPU affinity set must not be empty.");

  DWORD_PTR mask = 0;
  constexpr int kBits = static_cast<int>(sizeof(DWORD_PTR) * 8);
  for (int cpu : cpus) {
    if (cpu < 0 || cpu >= kBits)
      throw std::runtime_error("CPU id exceeds process affinity mask width.");
    mask |= DWORD_PTR{1} << cpu;
  }
  return mask;
}

void setCurrentProcessAffinity(const CpuList &cpus)
{
  if (!SetProcessAffinityMask(GetCurrentProcess(), makeCpuMask(cpus)))
    throw std::system_error(static_cast<int>(GetLastError()), std::system_category(), "SetProcessAffinityMask");
}

#endif

struct RuntimeState {
  std::optional<int> workerLimit;
#ifdef PGO_PARALLELISM_HAS_CPU_AFFINITY
  CpuList initialCpuAffinity = currentThreadAffinity();
  std::optional<int> cpuAffinityLimit;
#endif
#ifdef __linux__
  std::jthread cpuAffinityWatchdog;
#endif
#ifdef PGO_PARALLELISM_HAS_TBB
  std::unique_ptr<tbb::global_control> tbbControl;
#endif
};

RuntimeState &runtimeState()
{
  static RuntimeState state;
  return state;
}

void validateNumWorkers(std::optional<int> numWorkers)
{
  validatePositive(numWorkers, "parallel numWorkers");
}

void validateNumCpus(std::optional<int> numCpus)
{
  validatePositive(numCpus, "parallel numCpus");
}

#ifdef __linux__
void stopCpuAffinityWatchdog(RuntimeState &state)
{
  if (!state.cpuAffinityWatchdog.joinable())
    return;
  state.cpuAffinityWatchdog.request_stop();
  state.cpuAffinityWatchdog = std::jthread();
}

void startCpuAffinityWatchdog(RuntimeState &state, CpuList cpus)
{
  state.cpuAffinityWatchdog = std::jthread([cpus = std::move(cpus)](std::stop_token stopToken) {
    while (!stopToken.stop_requested()) {
      setCurrentProcessAffinityNoThrow(cpus);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });
}
#endif

[[maybe_unused]] void runSerial(int begin, int end, const detail::ChunkBody &body)
{
  if (begin < end)
    body(begin, end);
}

void warnIfInheritingAccelerateThreading()
{
#if defined(PGO_PARALLELISM_HAS_TBB) && defined(PGO_PARALLELISM_HAS_ACCELERATE_THREADING)
  if (BLASGetThreading() == BLAS_THREADING_SINGLE_THREADED)
    return;

  static std::once_flag warningOnce;
  std::call_once(warningOnce, [] {
    std::cerr << "Warning: pgo::parallel::NestedKernelPolicy::Inherit leaves Accelerate threading enabled inside TBB workers; "
                 "this can oversubscribe CPU threads on macOS. Use Suppress unless this call site is measured safe.\n";
  });
#endif
}

template<class Fn>
void runNestedKernelPolicy(const Options &options, Fn &&fn)
{
  if (options.nestedKernelPolicy == NestedKernelPolicy::Suppress) {
    class ScopedNestedKernelSuppressor {
    public:
      ScopedNestedKernelSuppressor()
      {
#ifdef PGO_PARALLELISM_HAS_MKL
        previousMKLThreads_ = mkl_set_num_threads_local(1);
        shouldRestoreMKL_ = true;
#endif
#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
        previousAccelerateThreading_ = static_cast<int>(BLASGetThreading());
        shouldRestoreAccelerate_ = BLASSetThreading(BLAS_THREADING_SINGLE_THREADED) == 0;
#endif
      }

      ~ScopedNestedKernelSuppressor()
      {
#ifdef PGO_PARALLELISM_HAS_ACCELERATE_THREADING
        if (shouldRestoreAccelerate_)
          BLASSetThreading(static_cast<BLAS_THREADING>(previousAccelerateThreading_));
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
        if (shouldRestoreMKL_)
          mkl_set_num_threads_local(previousMKLThreads_);
#endif
      }

    private:
      [[maybe_unused]] int previousMKLThreads_ = 0;
      [[maybe_unused]] int previousAccelerateThreading_ = 0;
      [[maybe_unused]] bool shouldRestoreMKL_ = false;
      [[maybe_unused]] bool shouldRestoreAccelerate_ = false;
    } nestedKernelSuppressor;

    fn();
    return;
  }

  warnIfInheritingAccelerateThreading();
  fn();
}

#ifdef PGO_PARALLELISM_HAS_TBB
void runTBB(int begin, int end, const Options &options, const detail::ChunkBody &body)
{
  const int grainSize = options.grainSize > 0 ? options.grainSize : 1;
  tbb::parallel_for(tbb::blocked_range<int>(begin, end, grainSize),
    [&](const tbb::blocked_range<int> &range) {
      runNestedKernelPolicy(options, [&] {
        body(range.begin(), range.end());
      });
    });
}
#endif

}  // namespace

void setWorkerLimit(std::optional<int> numWorkers)
{
  validateNumWorkers(numWorkers);
  std::lock_guard<std::mutex> lock(runtimeMutex());
  RuntimeState &state = runtimeState();
  if (state.workerLimit == numWorkers)
    return;

#ifdef PGO_PARALLELISM_HAS_TBB
  std::unique_ptr<tbb::global_control> nextTBBControl;
  if (numWorkers.has_value()) {
    nextTBBControl = std::make_unique<tbb::global_control>(
      tbb::global_control::max_allowed_parallelism, static_cast<std::size_t>(*numWorkers));
  }
#endif

#ifdef PGO_PARALLELISM_HAS_TBB
  state.tbbControl = std::move(nextTBBControl);
#endif
  state.workerLimit = numWorkers;
}

std::optional<int> workerLimit()
{
  std::lock_guard<std::mutex> lock(runtimeMutex());
  return runtimeState().workerLimit;
}

RuntimeInfo runtimeInfo()
{
  RuntimeInfo info;

  {
    std::lock_guard<std::mutex> lock(runtimeMutex());
    RuntimeState &state = runtimeState();
    info.workerLimit = state.workerLimit;
#ifdef PGO_PARALLELISM_HAS_CPU_AFFINITY
    info.cpuAffinityLimit = state.cpuAffinityLimit;
#endif
  }

#ifdef PGO_PARALLELISM_HAS_CPU_AFFINITY
  info.currentCpuAffinityCpus = static_cast<int>(currentThreadAffinity().size());
#endif
#ifdef PGO_PARALLELISM_HAS_TBB
  info.tbbMaxAllowedParallelism = static_cast<int>(
    tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism));
#endif
  return info;
}

bool supportsCpuAffinityLimit()
{
#ifdef PGO_PARALLELISM_HAS_CPU_AFFINITY
  return true;
#else
  return false;
#endif
}

void setCpuAffinityLimit(std::optional<int> numCpus)
{
  validateNumCpus(numCpus);

#ifndef PGO_PARALLELISM_HAS_CPU_AFFINITY
  if (numCpus.has_value())
    throw std::runtime_error("CPU affinity limits are only supported on Linux and Windows.");
  return;
#else
  std::lock_guard<std::mutex> lock(runtimeMutex());
  RuntimeState &state = runtimeState();
  if (state.cpuAffinityLimit == numCpus)
    return;

#ifdef __linux__
  stopCpuAffinityWatchdog(state);
#endif

  const CpuList target = numCpus.has_value() ? limitCpuList(state.initialCpuAffinity, *numCpus) : state.initialCpuAffinity;
  setCurrentProcessAffinity(target);

  state.cpuAffinityLimit = numCpus;
#ifdef __linux__
  if (numCpus.has_value() && target.size() < state.initialCpuAffinity.size())
    startCpuAffinityWatchdog(state, target);
#endif
#endif
}

std::optional<int> cpuAffinityLimit()
{
#ifndef PGO_PARALLELISM_HAS_CPU_AFFINITY
  return std::nullopt;
#else
  std::lock_guard<std::mutex> lock(runtimeMutex());
  return runtimeState().cpuAffinityLimit;
#endif
}

ScopedWorkerLimit::ScopedWorkerLimit(std::optional<int> numWorkers)
  : previousNumWorkers_(workerLimit())
{
  setWorkerLimit(numWorkers);
}

ScopedWorkerLimit::~ScopedWorkerLimit()
{
  setWorkerLimit(previousNumWorkers_);
}

ScopedCpuAffinityLimit::ScopedCpuAffinityLimit(std::optional<int> numCpus)
  : previousNumCpus_(cpuAffinityLimit())
{
  setCpuAffinityLimit(numCpus);
}

ScopedCpuAffinityLimit::~ScopedCpuAffinityLimit()
{
  setCpuAffinityLimit(previousNumCpus_);
}

namespace detail {

void parallelForChunks(int begin, int end, const Options &options, const ChunkBody &body)
{
  if (begin >= end)
    return;

#ifdef PGO_PARALLELISM_HAS_TBB
  runTBB(begin, end, options, body);
#else
  runNestedKernelPolicy(options, [&] {
    runSerial(begin, end, body);
  });
#endif
}

}  // namespace detail
}  // namespace pgo::parallel
