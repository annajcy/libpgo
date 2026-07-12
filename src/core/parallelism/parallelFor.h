#pragma once

#include "parallelExecutor.h"
#include "parallelOptions.h"
#include "parallelRuntime.h"

#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pgo::parallel
{

namespace detail
{

using ChunkBody = std::function<void(int begin, int end)>;

void parallelForChunks(int begin, int end, const Options &options, const ChunkBody &body);
void parallelForChunks(
  const ParallelExecutor &executor, int begin, int end, const Options &options, const ChunkBody &body);

}  // namespace detail

template<class Fn>
void parallelFor(const ParallelExecutor &executor, int begin, int end, const Options &options, Fn &&fn)
{
  detail::parallelForChunks(executor, begin, end, options,
    [&](int chunkBegin, int chunkEnd) {
      for (int i = chunkBegin; i < chunkEnd; ++i)
        fn(i);
    });
}

template<class Fn>
void parallelFor(const ParallelExecutor &executor, int begin, int end, Fn &&fn)
{
  parallelFor(executor, begin, end, Options{}, std::forward<Fn>(fn));
}

template<class Fn>
void parallelFor(int begin, int end, const Options &options, Fn &&fn)
{
  detail::parallelForChunks(begin, end, options,
    [&](int chunkBegin, int chunkEnd) {
      for (int i = chunkBegin; i < chunkEnd; ++i)
        fn(i);
    });
}

template<class Fn>
void parallelFor(int begin, int end, Fn &&fn)
{
  parallelFor(begin, end, Options{}, std::forward<Fn>(fn));
}

template<class Fn>
void parallelFor3D(
  const ParallelExecutor &executor, int nx, int ny, int nz, const Options &options, Fn &&fn)
{
  if (nx <= 0 || ny <= 0 || nz <= 0)
    return;

  const long long total = static_cast<long long>(nx) * static_cast<long long>(ny) * static_cast<long long>(nz);
  if (total > std::numeric_limits<int>::max())
    throw std::runtime_error("parallelFor3D range exceeds int indexing capacity.");

  parallelFor(executor, 0, static_cast<int>(total), options, [&](int index) {
    const int x = index % nx;
    const int y = (index / nx) % ny;
    const int z = index / (nx * ny);
    fn(x, y, z);
  });
}

template<class Fn>
void parallelFor3D(const ParallelExecutor &executor, int nx, int ny, int nz, Fn &&fn)
{
  parallelFor3D(executor, nx, ny, nz, Options{}, std::forward<Fn>(fn));
}

template<class Fn>
void parallelFor3D(int nx, int ny, int nz, const Options &options, Fn &&fn)
{
  if (nx <= 0 || ny <= 0 || nz <= 0)
    return;

  const long long total = static_cast<long long>(nx) * static_cast<long long>(ny) * static_cast<long long>(nz);
  if (total > std::numeric_limits<int>::max())
    throw std::runtime_error("parallelFor3D range exceeds int indexing capacity.");

  parallelFor(0, static_cast<int>(total), options, [&](int index) {
    const int x = index % nx;
    const int y = (index / nx) % ny;
    const int z = index / (nx * ny);
    fn(x, y, z);
  });
}

template<class Fn>
void parallelFor3D(int nx, int ny, int nz, Fn &&fn)
{
  parallelFor3D(nx, ny, nz, Options{}, std::forward<Fn>(fn));
}

}  // namespace pgo::parallel
