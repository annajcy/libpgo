#pragma once

#include <memory>

namespace pgo::benchmark_helpers
{

// A deterministic Eigen-only GEMM control. It deliberately avoids every
// BLAS/LAPACK entry point while retaining the same matrix-multiply expression.
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
