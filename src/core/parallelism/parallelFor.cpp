#include "parallelism/parallelFor.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <vector>

#ifdef __linux__
#include <dirent.h>
#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#ifdef PGO_PARALLELISM_HAS_TBB
#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#endif

#ifdef PGO_PARALLELISM_HAS_EIGEN
#include <Eigen/Core>
#endif

#ifdef PGO_PARALLELISM_HAS_OPENMP
#include <omp.h>
#endif

#ifdef PGO_PARALLELISM_HAS_MKL
#include <mkl.h>
#endif

#ifdef PGO_PARALLELISM_HAS_OPENBLAS
extern "C" int openblas_get_num_threads();
extern "C" void openblas_set_num_threads(int numThreads);
#endif

namespace pgo::parallel {
namespace {

std::mutex &runtimeMutex()
{
  static std::mutex mutex;
  return mutex;
}

std::atomic<bool> &nestedThreadLimitEnabled()
{
  static std::atomic<bool> enabled = false;
  return enabled;
}

using CpuList = std::vector<int>;

void validatePositive(std::optional<int> value, const char *name)
{
  if (value.has_value() && *value <= 0)
    throw std::invalid_argument(std::string(name) + " must be positive or unset.");
}

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

CpuList limitCpuList(const CpuList &cpus, int limit)
{
  CpuList limited = cpus;
  std::sort(limited.begin(), limited.end());
  if (static_cast<int>(limited.size()) > limit)
    limited.resize(static_cast<std::size_t>(limit));
  return limited;
}

#endif

struct RuntimeState {
  std::optional<int> numThreads;
#ifdef __linux__
  CpuList initialCpuAffinity = currentThreadAffinity();
  std::optional<int> cpuAffinityLimit;
  std::jthread cpuAffinityWatchdog;
#endif
#ifdef PGO_PARALLELISM_HAS_TBB
  std::unique_ptr<tbb::global_control> tbbControl;
#endif
#ifdef PGO_PARALLELISM_HAS_EIGEN
  int initialEigenThreads = Eigen::nbThreads();
#endif
#ifdef PGO_PARALLELISM_HAS_OPENMP
  int initialOpenMPThreads = omp_get_max_threads();
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
  int initialMKLThreads = mkl_get_max_threads();
  int initialMKLPardisoThreads = mkl_domain_get_max_threads(MKL_DOMAIN_PARDISO);
#endif
#ifdef PGO_PARALLELISM_HAS_OPENBLAS
  int initialOpenBLASThreads = openblas_get_num_threads();
#endif
};

RuntimeState &runtimeState()
{
  static RuntimeState state;
  return state;
}

void validateNumThreads(std::optional<int> numThreads)
{
  validatePositive(numThreads, "parallel numThreads");
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

Backend autoBackend()
{
#ifdef PGO_PARALLELISM_HAS_TBB
  return Backend::TBB;
#elif defined(PGO_PARALLELISM_HAS_OPENMP)
  return Backend::OpenMP;
#else
  return Backend::Serial;
#endif
}

Backend resolveBackend(Backend backend)
{
  if (backend == Backend::Auto)
    return autoBackend();
  return backend;
}

void runSerial(int begin, int end, const detail::ChunkBody &body)
{
  if (begin < end)
    body(begin, end);
}

template<class Fn>
void runNestedKernelPolicy(const Options &options, Fn &&fn)
{
  if (options.nestedKernelPolicy == NestedKernelPolicy::Suppress) {
    ScopedNestedThreadLimit nestedThreadLimit;
    fn();
    return;
  }

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

void runOpenMP(int begin, int end, const Options &options, const detail::ChunkBody &body)
{
#ifdef PGO_PARALLELISM_HAS_OPENMP
  const int grainSize = options.grainSize > 0 ? options.grainSize : 1;
#pragma omp parallel for schedule(static, grainSize)
  for (int i = begin; i < end; i += grainSize) {
    const int chunkEnd = std::min(i + grainSize, end);
    runNestedKernelPolicy(options, [&] {
      body(i, chunkEnd);
    });
  }
#else
  (void)begin;
  (void)end;
  (void)options;
  (void)body;
  throw std::runtime_error("OpenMP parallel backend is not available in this build.");
#endif
}

}  // namespace

bool isBackendAvailable(Backend backend)
{
  switch (backend) {
    case Backend::Auto:
    case Backend::Serial:
      return true;
    case Backend::TBB:
#ifdef PGO_PARALLELISM_HAS_TBB
      return true;
#else
      return false;
#endif
    case Backend::OpenMP:
#ifdef PGO_PARALLELISM_HAS_OPENMP
      return true;
#else
      return false;
#endif
  }

  return false;
}

void setThreadLimit(std::optional<int> numThreads)
{
  validateNumThreads(numThreads);
  std::lock_guard<std::mutex> lock(runtimeMutex());
  RuntimeState &state = runtimeState();
  if (state.numThreads == numThreads)
    return;

#ifdef PGO_PARALLELISM_HAS_TBB
  std::unique_ptr<tbb::global_control> nextTBBControl;
  if (numThreads.has_value()) {
    nextTBBControl = std::make_unique<tbb::global_control>(
      tbb::global_control::max_allowed_parallelism, static_cast<std::size_t>(*numThreads));
  }
#endif

#ifdef PGO_PARALLELISM_HAS_OPENMP
  omp_set_num_threads(numThreads.value_or(state.initialOpenMPThreads));
#endif
#ifdef PGO_PARALLELISM_HAS_EIGEN
  Eigen::setNbThreads(numThreads.value_or(state.initialEigenThreads));
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
  const int mklThreads = numThreads.value_or(state.initialMKLThreads);
  const int mklPardisoThreads = numThreads.value_or(state.initialMKLPardisoThreads);
  mkl_set_num_threads(mklThreads);
  mkl_domain_set_num_threads(mklThreads, MKL_DOMAIN_ALL);
  mkl_domain_set_num_threads(mklPardisoThreads, MKL_DOMAIN_PARDISO);
#endif
#ifdef PGO_PARALLELISM_HAS_OPENBLAS
  openblas_set_num_threads(numThreads.value_or(state.initialOpenBLASThreads));
#endif
#ifdef PGO_PARALLELISM_HAS_TBB
  state.tbbControl = std::move(nextTBBControl);
#endif
  state.numThreads = numThreads;
  nestedThreadLimitEnabled().store(numThreads.has_value() && *numThreads > 1, std::memory_order_release);
}

std::optional<int> threadLimit()
{
  std::lock_guard<std::mutex> lock(runtimeMutex());
  return runtimeState().numThreads;
}

RuntimeInfo runtimeInfo()
{
  RuntimeInfo info;

  {
    std::lock_guard<std::mutex> lock(runtimeMutex());
    RuntimeState &state = runtimeState();
    info.threadLimit = state.numThreads;
#ifdef __linux__
    info.cpuAffinityLimit = state.cpuAffinityLimit;
#endif
  }

#ifdef __linux__
  info.currentCpuAffinityCpus = static_cast<int>(currentThreadAffinity().size());
#endif
#ifdef PGO_PARALLELISM_HAS_TBB
  info.tbbMaxAllowedParallelism = static_cast<int>(
    tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism));
#endif
#ifdef PGO_PARALLELISM_HAS_EIGEN
  info.eigenNumThreads = Eigen::nbThreads();
#endif
#ifdef PGO_PARALLELISM_HAS_OPENMP
  info.openMPMaxThreads = omp_get_max_threads();
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
  info.mklMaxThreads = mkl_get_max_threads();
  #if defined(PGO_PARALLELISM_MKL_TBB_THREADING) && defined(PGO_PARALLELISM_HAS_TBB)
  info.mklEffectiveThreadLimit = static_cast<int>(
    tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism));
  #else
  info.mklEffectiveThreadLimit = info.mklMaxThreads;
  #endif
  info.mklPardisoMaxThreads = mkl_domain_get_max_threads(MKL_DOMAIN_PARDISO);
#endif
#ifdef PGO_PARALLELISM_HAS_OPENBLAS
  info.openBLASNumThreads = openblas_get_num_threads();
#endif

  return info;
}

bool supportsCpuAffinityLimit()
{
#ifdef __linux__
  return true;
#else
  return false;
#endif
}

void setCpuAffinityLimit(std::optional<int> numCpus)
{
  validateNumCpus(numCpus);

#ifndef __linux__
  if (numCpus.has_value())
    throw std::runtime_error("CPU affinity limits are only supported on Linux.");
  return;
#else
  std::lock_guard<std::mutex> lock(runtimeMutex());
  RuntimeState &state = runtimeState();
  if (state.cpuAffinityLimit == numCpus)
    return;

  stopCpuAffinityWatchdog(state);

  const CpuList target = numCpus.has_value() ? limitCpuList(state.initialCpuAffinity, *numCpus) : state.initialCpuAffinity;
  setCurrentProcessAffinity(target);

  state.cpuAffinityLimit = numCpus;
  if (numCpus.has_value() && target.size() < state.initialCpuAffinity.size())
    startCpuAffinityWatchdog(state, target);
#endif
}

std::optional<int> cpuAffinityLimit()
{
#ifndef __linux__
  return std::nullopt;
#else
  std::lock_guard<std::mutex> lock(runtimeMutex());
  return runtimeState().cpuAffinityLimit;
#endif
}

ScopedThreadLimit::ScopedThreadLimit(std::optional<int> numThreads)
  : previousNumThreads_(threadLimit())
{
  setThreadLimit(numThreads);
}

ScopedThreadLimit::~ScopedThreadLimit()
{
  setThreadLimit(previousNumThreads_);
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

ScopedMklThreadLimit::ScopedMklThreadLimit(std::optional<int> numThreads)
{
  validateNumThreads(numThreads);
#ifdef PGO_PARALLELISM_HAS_MKL
  if (numThreads.has_value()) {
  #if defined(PGO_PARALLELISM_MKL_TBB_THREADING) && defined(PGO_PARALLELISM_HAS_TBB)
    threadingControl_ = std::make_shared<tbb::global_control>(
      tbb::global_control::max_allowed_parallelism, static_cast<std::size_t>(*numThreads));
  #else
    previousNumThreads_ = mkl_set_num_threads_local(*numThreads);
    shouldRestore_ = true;
  #endif
  }
#else
  (void)numThreads;
#endif
}

ScopedMklThreadLimit::~ScopedMklThreadLimit()
{
#ifdef PGO_PARALLELISM_HAS_MKL
#if !defined(PGO_PARALLELISM_MKL_TBB_THREADING) || !defined(PGO_PARALLELISM_HAS_TBB)
  if (shouldRestore_)
    mkl_set_num_threads_local(previousNumThreads_);
#endif
#endif
}

ScopedNestedThreadLimit::ScopedNestedThreadLimit()
{
  if (!nestedThreadLimitEnabled().load(std::memory_order_acquire))
    return;

#ifdef PGO_PARALLELISM_HAS_OPENMP
  previousOpenMPThreads_ = omp_get_max_threads();
  if (previousOpenMPThreads_ != 1) {
    omp_set_num_threads(1);
    shouldRestoreOpenMP_ = true;
  }
#endif
#ifdef PGO_PARALLELISM_HAS_MKL
  previousMKLThreads_ = mkl_set_num_threads_local(1);
  shouldRestoreMKL_ = true;
#endif
}

ScopedNestedThreadLimit::~ScopedNestedThreadLimit()
{
#ifdef PGO_PARALLELISM_HAS_MKL
  if (shouldRestoreMKL_)
    mkl_set_num_threads_local(previousMKLThreads_);
#endif
#ifdef PGO_PARALLELISM_HAS_OPENMP
  if (shouldRestoreOpenMP_)
    omp_set_num_threads(previousOpenMPThreads_);
#endif
}

namespace detail {

void parallelForChunks(int begin, int end, const Options &options, const ChunkBody &body)
{
  if (begin >= end)
    return;

  if (options.backend == Backend::Serial) {
    runSerial(begin, end, body);
    return;
  }

  switch (resolveBackend(options.backend)) {
    case Backend::Auto:
    case Backend::Serial:
      runSerial(begin, end, body);
      return;
    case Backend::TBB:
#ifdef PGO_PARALLELISM_HAS_TBB
      runTBB(begin, end, options, body);
      return;
#else
      throw std::runtime_error("TBB parallel backend is not available in this build.");
#endif
    case Backend::OpenMP:
      runOpenMP(begin, end, options, body);
      return;
  }
}

}  // namespace detail
}  // namespace pgo::parallel
