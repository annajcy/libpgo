#pragma once

#include <memory>

namespace pgo::benchmark_helpers
{

class NestedEigenAccelerateWorkload
{
public:
  NestedEigenAccelerateWorkload(int outerTasks, int matrixN);
  ~NestedEigenAccelerateWorkload();

  NestedEigenAccelerateWorkload(const NestedEigenAccelerateWorkload &) = delete;
  NestedEigenAccelerateWorkload &operator=(const NestedEigenAccelerateWorkload &) = delete;

  void run(int taskIndex);
  double checksum() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pgo::benchmark_helpers
