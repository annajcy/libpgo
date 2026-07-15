#pragma once

#include "accelerateThreading.h"
#include "parallelControl.h"

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/partitioner.h>

namespace pgo::parallel
{
namespace detail
{

inline std::size_t resolveGrainSize(std::optional<int> grainSize)
{
  if (!grainSize.has_value())
    return 1;
  if (*grainSize <= 0)
    throw std::invalid_argument("Parallel grain size must be positive when specified.");
  return static_cast<std::size_t>(*grainSize);
}

}  // namespace detail

template<class Index, class Fn, class Partitioner = tbb::auto_partitioner>
void parallelForChunks(Index begin, Index end, Fn &&fn,
  std::optional<int> grainSize = std::nullopt, Partitioner partitioner = {})
{
  static_assert(std::is_integral_v<Index>, "pgo parallel loop indexes must be integral.");
  const std::size_t resolvedGrainSize = detail::resolveGrainSize(grainSize);
  if (begin >= end)
    return;

  withGlobalTbbConcurrency([&] {
    tbb::parallel_for(
      tbb::blocked_range<Index>(begin, end, resolvedGrainSize),
      [&](const tbb::blocked_range<Index> &range) {
        detail::setAccelerateSingleThreading();
        fn(range.begin(), range.end());
      },
      partitioner);
  });
}

template<class Index, class Partitioner, class Fn>
void parallelForChunks(Index begin, Index end, const Partitioner &partitioner, Fn &&fn)
{
  parallelForChunks(begin, end, std::forward<Fn>(fn), std::nullopt, partitioner);
}

template<class Index, class Fn, class Partitioner = tbb::auto_partitioner>
void parallelFor(Index begin, Index end, Fn &&fn,
  std::optional<int> grainSize = std::nullopt, Partitioner partitioner = {})
{
  parallelForChunks(begin, end, [&](Index chunkBegin, Index chunkEnd) {
      for (Index i = chunkBegin; i < chunkEnd; ++i)
        fn(i); }, grainSize, std::move(partitioner));
}

template<class Index, class Partitioner, class Fn>
void parallelFor(Index begin, Index end, const Partitioner &partitioner, Fn &&fn)
{
  parallelFor(begin, end, std::forward<Fn>(fn), std::nullopt, partitioner);
}

}  // namespace pgo::parallel
