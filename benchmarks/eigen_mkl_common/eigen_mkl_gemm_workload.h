#pragma once

#include <memory>

namespace pgo::benchmark_helpers
{

class EigenMklGemmWorkload
{
public:
  EigenMklGemmWorkload(int outerTasks, int matrixN);
  ~EigenMklGemmWorkload();

  EigenMklGemmWorkload(const EigenMklGemmWorkload &) = delete;
  EigenMklGemmWorkload &operator=(const EigenMklGemmWorkload &) = delete;

  void run(int taskIndex);
  double checksum() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pgo::benchmark_helpers
