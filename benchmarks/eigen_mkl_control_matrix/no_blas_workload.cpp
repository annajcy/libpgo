#include "no_blas_workload.h"

#include <cstddef>
#include <numeric>
#include <vector>

namespace pgo::benchmark_helpers
{

class NoBlasWorkload::Impl
{
public:
  Impl(int outerTasks, int matrixN): input_(elementCount(matrixN))
  {
    for (std::size_t i = 0; i < input_.size(); ++i) {
      const auto value = static_cast<unsigned int>((i * 17 + 43) % 257);
      input_[i] = (static_cast<double>(value) - 128.0) / 257.0;
    }

    outputs_.resize(static_cast<std::size_t>(outerTasks));
    for (auto &output : outputs_)
      output.assign(input_.size(), 0.0);
  }

  void run(int taskIndex)
  {
    auto &output = outputs_[static_cast<std::size_t>(taskIndex)];
    const double offset = static_cast<double>(taskIndex + 1) * 1e-4;
    for (std::size_t i = 0; i < output.size(); ++i)
      output[i] = 0.625 * input_[i] + offset;
  }

  double checksum() const
  {
    double result = 0.0;
    for (const auto &output : outputs_)
      result += std::accumulate(output.begin(), output.end(), 0.0);
    return result;
  }

private:
  static std::size_t elementCount(int matrixN)
  {
    const auto size = static_cast<std::size_t>(matrixN);
    return size * size;
  }

  std::vector<double> input_;
  std::vector<std::vector<double>> outputs_;
};

NoBlasWorkload::NoBlasWorkload(int outerTasks, int matrixN):
  impl_(std::make_unique<Impl>(outerTasks, matrixN))
{
}

NoBlasWorkload::~NoBlasWorkload() = default;

void NoBlasWorkload::run(int taskIndex)
{
  impl_->run(taskIndex);
}

double NoBlasWorkload::checksum() const
{
  return impl_->checksum();
}

}  // namespace pgo::benchmark_helpers
