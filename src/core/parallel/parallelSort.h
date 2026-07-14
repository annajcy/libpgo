#pragma once

#include "parallelControl.h"

#include <functional>
#include <iterator>
#include <type_traits>
#include <utility>

#include <tbb/parallel_sort.h>

namespace pgo::parallel
{

template<class RandomIt, class Compare>
void parallelSort(RandomIt first, RandomIt last, Compare compare)
{
  using Category = typename std::iterator_traits<RandomIt>::iterator_category;
  static_assert(std::is_base_of_v<std::random_access_iterator_tag, Category>,
    "pgo parallelSort requires random-access iterators.");

  if (last - first < 2)
    return;
  withGlobalTbbConcurrency([&] {
    tbb::parallel_sort(first, last, std::move(compare));
  });
}

template<class RandomIt>
void parallelSort(RandomIt first, RandomIt last)
{
  using Value = typename std::iterator_traits<RandomIt>::value_type;
  parallelSort(first, last, std::less<Value>{});
}

}  // namespace pgo::parallel
