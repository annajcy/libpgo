#pragma once

#include "parallelFor.h"

#include <optional>
#include <type_traits>
#include <utility>

#include <tbb/parallel_reduce.h>
#include <tbb/partitioner.h>

namespace pgo::parallel
{
namespace detail
{

template<class Index, class Value, class RangeFn, class JoinFn, class Partitioner>
Value reduce(Index begin, Index end, std::size_t grainSize, Value identity, RangeFn &rangeFn,
  JoinFn &joinFn, const Partitioner &partitioner)
{
  return withGlobalTbbConcurrency([&]() -> Value {
    return tbb::parallel_reduce(
      tbb::blocked_range<Index>(begin, end, grainSize), identity,
      [&](const tbb::blocked_range<Index> &chunk, Value local) {
        setAccelerateSingleThreading();
        return rangeFn(chunk.begin(), chunk.end(), std::move(local));
      },
      [&](Value left, Value right) {
        setAccelerateSingleThreading();
        return joinFn(std::move(left), std::move(right));
      },
      partitioner);
  });
}

}  // namespace detail

template<class Index, class Value, class RangeFn, class JoinFn,
  class Partitioner = tbb::auto_partitioner>
Value parallelReduce(Index begin, Index end, Value identity, RangeFn &&rangeFn, JoinFn &&joinFn,
  std::optional<int> grainSize = std::nullopt, Partitioner partitioner = {})
{
  static_assert(std::is_integral_v<Index>, "pgo reduction indexes must be integral.");
  static_assert(std::is_copy_constructible_v<Value>,
    "pgo reduction values must be copy constructible.");
  const std::size_t resolvedGrainSize = detail::resolveGrainSize(grainSize);
  if (begin >= end)
    return identity;

  return detail::reduce(begin, end, resolvedGrainSize, std::move(identity), rangeFn, joinFn, partitioner);
}

template<class Index, class Value, class Partitioner, class RangeFn, class JoinFn>
Value parallelReduce(Index begin, Index end, Value identity, const Partitioner &partitioner,
  RangeFn &&rangeFn, JoinFn &&joinFn)
{
  return parallelReduce(begin, end, std::move(identity), std::forward<RangeFn>(rangeFn),
    std::forward<JoinFn>(joinFn), std::nullopt, partitioner);
}

}  // namespace pgo::parallel
