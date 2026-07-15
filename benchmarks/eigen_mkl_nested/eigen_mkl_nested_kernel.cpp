#include "eigen_mkl_nested_kernel.h"

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

#if !defined(EIGEN_USE_MKL_ALL)
#  error "The nested MKL benchmark must inherit EIGEN_USE_MKL_ALL from Eigen3::Eigen."
#endif

#if !defined(EIGEN_MKL_NO_DIRECT_CALL)
#  error "The nested MKL benchmark requires EIGEN_MKL_NO_DIRECT_CALL."
#endif

#if !defined(EIGEN_DONT_PARALLELIZE)
#  error "Eigen's own parallel layer must remain disabled; oneMKL-TBB owns the inner kernel."
#endif

namespace pgo::benchmark_helpers
{

class NestedEigenMklWorkload::Impl
{
public:
  Impl(int outerTasks, int matrixN):
    left_(matrixN, matrixN), right_(matrixN, matrixN)
  {
    initialize(left_, 1);
    initialize(right_, 2);

    outputs_.reserve(static_cast<std::size_t>(outerTasks));
    for (int task = 0; task < outerTasks; ++task)
      outputs_.emplace_back(Eigen::MatrixXd::Zero(matrixN, matrixN));
  }

  void run(int taskIndex)
  {
    outputs_[static_cast<std::size_t>(taskIndex)].noalias() = left_ * right_;
  }

  double checksum() const
  {
    double result = 0.0;
    for (const Eigen::MatrixXd &output : outputs_)
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
        matrix(row, column) =
          (static_cast<double>(value) - 128.0) / 257.0;
      }
    }
  }

  Eigen::MatrixXd left_;
  Eigen::MatrixXd right_;
  std::vector<Eigen::MatrixXd> outputs_;
};

NestedEigenMklWorkload::NestedEigenMklWorkload(int outerTasks, int matrixN):
  impl_(std::make_unique<Impl>(outerTasks, matrixN))
{
}

NestedEigenMklWorkload::~NestedEigenMklWorkload() = default;

void NestedEigenMklWorkload::run(int taskIndex)
{
  impl_->run(taskIndex);
}

double NestedEigenMklWorkload::checksum() const
{
  return impl_->checksum();
}

}  // namespace pgo::benchmark_helpers
