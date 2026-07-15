#pragma once

#include <memory>

namespace pgo::benchmark_helpers
{

// A deterministic scalar-only control. It deliberately avoids Eigen and every
// BLAS/LAPACK entry point, while retaining per-outer-task independent output.
class NoBlasWorkload
{
public:
  NoBlasWorkload(int outerTasks, int matrixN);
  ~NoBlasWorkload();

  NoBlasWorkload(const NoBlasWorkload &) = delete;
  NoBlasWorkload &operator=(const NoBlasWorkload &) = delete;

  void run(int taskIndex);
  double checksum() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pgo::benchmark_helpers
