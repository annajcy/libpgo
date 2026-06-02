#include "parallelism/parallelFor.h"

#include <algorithm>
#include <mutex>
#include <optional>
#include <stdexcept>

#ifdef PGO_PARALLELISM_HAS_TBB
#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#endif

namespace pgo::parallel {
namespace {

std::mutex &runtimeMutex()
{
  static std::mutex mutex;
  return mutex;
}

std::optional<int> &runtimeDefaultNumThreads()
{
  static std::optional<int> numThreads;
  return numThreads;
}

void validateNumThreads(std::optional<int> numThreads)
{
  if (numThreads.has_value() && *numThreads <= 0)
    throw std::invalid_argument("parallel numThreads must be positive or unset.");
}

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

#ifdef PGO_PARALLELISM_HAS_TBB
void runTBB(int begin, int end, const Options &options, const detail::ChunkBody &body)
{
  const int grainSize = options.grainSize > 0 ? options.grainSize : 1;
  auto run = [&]() {
    tbb::parallel_for(tbb::blocked_range<int>(begin, end, grainSize),
      [&](const tbb::blocked_range<int> &range) {
        body(range.begin(), range.end());
      });
  };

  if (options.numThreads > 1) {
    tbb::global_control control(tbb::global_control::max_allowed_parallelism,
      static_cast<std::size_t>(options.numThreads));
    run();
  }
  else {
    run();
  }
}
#endif

void runOpenMP(int begin, int end, const Options &options, const detail::ChunkBody &body)
{
#ifdef PGO_PARALLELISM_HAS_OPENMP
  const int grainSize = options.grainSize > 0 ? options.grainSize : 1;
  if (options.numThreads > 1) {
#pragma omp parallel for schedule(static, grainSize) num_threads(options.numThreads)
    for (int i = begin; i < end; i += grainSize) {
      const int chunkEnd = std::min(i + grainSize, end);
      body(i, chunkEnd);
    }
  }
  else {
#pragma omp parallel for schedule(static, grainSize)
    for (int i = begin; i < end; i += grainSize) {
      const int chunkEnd = std::min(i + grainSize, end);
      body(i, chunkEnd);
    }
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

void setDefaultNumThreads(std::optional<int> numThreads)
{
  validateNumThreads(numThreads);
  std::lock_guard<std::mutex> lock(runtimeMutex());
  runtimeDefaultNumThreads() = numThreads;
}

std::optional<int> defaultNumThreads()
{
  std::lock_guard<std::mutex> lock(runtimeMutex());
  return runtimeDefaultNumThreads();
}

ScopedThreadLimit::ScopedThreadLimit(std::optional<int> numThreads)
  : previousNumThreads_(defaultNumThreads())
{
  setDefaultNumThreads(numThreads);
}

ScopedThreadLimit::~ScopedThreadLimit()
{
  setDefaultNumThreads(previousNumThreads_);
}

namespace detail {

void parallelForChunks(int begin, int end, const Options &options, const ChunkBody &body)
{
  if (begin >= end)
    return;

  Options effectiveOptions = options;
  if (effectiveOptions.numThreads == 0) {
    if (const auto defaultThreads = defaultNumThreads(); defaultThreads.has_value())
      effectiveOptions.numThreads = *defaultThreads;
  }

  if (effectiveOptions.numThreads == 1 || effectiveOptions.backend == Backend::Serial) {
    runSerial(begin, end, body);
    return;
  }

  switch (resolveBackend(effectiveOptions.backend)) {
    case Backend::Auto:
    case Backend::Serial:
      runSerial(begin, end, body);
      return;
    case Backend::TBB:
#ifdef PGO_PARALLELISM_HAS_TBB
      runTBB(begin, end, effectiveOptions, body);
      return;
#else
      throw std::runtime_error("TBB parallel backend is not available in this build.");
#endif
    case Backend::OpenMP:
      runOpenMP(begin, end, effectiveOptions, body);
      return;
  }
}

}  // namespace detail
}  // namespace pgo::parallel
