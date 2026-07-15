#pragma once

#include <memory>

namespace pgo::benchmark_helpers
{

class EigenGemmWorkspace
{
public:
  explicit EigenGemmWorkspace(int matrixN);
  ~EigenGemmWorkspace();

  EigenGemmWorkspace(const EigenGemmWorkspace &) = delete;
  EigenGemmWorkspace &operator=(const EigenGemmWorkspace &) = delete;

  void run();
  double checksum() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pgo::benchmark_helpers
