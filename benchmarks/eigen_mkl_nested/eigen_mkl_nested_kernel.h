#pragma once

#include <memory>

namespace pgo::benchmark_helpers
{

class NestedEigenMklWorkload
{
public:
  NestedEigenMklWorkload(int outerTasks, int matrixN);
  ~NestedEigenMklWorkload();

  NestedEigenMklWorkload(const NestedEigenMklWorkload &) = delete;
  NestedEigenMklWorkload &operator=(const NestedEigenMklWorkload &) = delete;

  void run(int taskIndex);
  double checksum() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pgo::benchmark_helpers
