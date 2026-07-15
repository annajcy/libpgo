#include "no_blas_workload.h"

#include <Eigen/Dense>

#if defined(EIGEN_USE_MKL_ALL) || defined(EIGEN_USE_BLAS)
#  error "The NoBlas control target must compile Eigen without a BLAS backend."
#endif

#if !defined(EIGEN_DONT_PARALLELIZE)
#  error "The NoBlas control must keep Eigen's internal parallel layer disabled."
#endif

#include <vector>

namespace pgo::benchmark_helpers
{

class NoBlasWorkload::Impl
{
public:
  Impl(int outerTasks, int matrixN): left_(matrixN, matrixN), right_(matrixN, matrixN)
  {
    initialize(left_, 1);
    initialize(right_, 2);

    outputs_.reserve(static_cast<std::size_t>(outerTasks));
    for (int task = 0; task < outerTasks; ++task)
      outputs_.emplace_back(Eigen::MatrixXd::Zero(matrixN, matrixN));
  }

  void run(int taskIndex)
  {
    auto &output = outputs_[static_cast<std::size_t>(taskIndex)];
    output.noalias() = left_ * right_;
  }

  double checksum() const
  {
    double result = 0.0;
    for (const auto &output : outputs_)
      result += output.sum();
    return result;
  }

private:
  static void initialize(Eigen::MatrixXd &matrix, int seed)
  {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
        const auto value = static_cast<unsigned int>(
          (row * 17 + column * 29 + seed * 43) % 257);
        matrix(row, column) = (static_cast<double>(value) - 128.0) / 257.0;
      }
    }
  }

  Eigen::MatrixXd left_;
  Eigen::MatrixXd right_;
  std::vector<Eigen::MatrixXd> outputs_;
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
