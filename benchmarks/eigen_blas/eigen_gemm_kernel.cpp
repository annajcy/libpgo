#include "eigen_gemm_kernel.h"

#include <Eigen/Dense>

#if (defined(PGO_EIGEN_BLAS_INTERNAL) + defined(PGO_EIGEN_BLAS_ACCELERATE) + \
  defined(PGO_EIGEN_BLAS_MKL)) != 1
#  error "Select exactly one Eigen GEMM benchmark backend."
#endif

#if defined(PGO_EIGEN_BLAS_INTERNAL) && \
  (defined(EIGEN_USE_BLAS) || defined(EIGEN_USE_MKL_ALL))
#  error "The Eigen-internal control must not inherit a BLAS backend."
#endif

#if defined(PGO_EIGEN_BLAS_ACCELERATE) && !defined(EIGEN_USE_BLAS)
#  error "The Accelerate benchmark must inherit EIGEN_USE_BLAS from Eigen3::Eigen."
#endif

#if defined(PGO_EIGEN_BLAS_MKL) && !defined(EIGEN_USE_MKL_ALL)
#  error "The MKL benchmark must inherit EIGEN_USE_MKL_ALL from Eigen3::Eigen."
#endif

#if defined(PGO_EIGEN_BLAS_MKL) && !defined(EIGEN_MKL_NO_DIRECT_CALL)
#  error "The MKL benchmark requires EIGEN_MKL_NO_DIRECT_CALL for a comparable DGEMM path."
#endif

#if !defined(EIGEN_DONT_PARALLELIZE)
#  error "All benchmark backends must keep Eigen's own parallel layer disabled."
#endif

namespace pgo::benchmark_helpers
{

class EigenGemmWorkspace::Impl
{
public:
  explicit Impl(int matrixN): left_(matrixN, matrixN), right_(matrixN, matrixN), output_(matrixN, matrixN)
  {
    initialize(left_, 1);
    initialize(right_, 2);
    output_.setZero();
  }

  void run()
  {
    // This is intentionally the only numerical kernel in the benchmark family. Backend selection
    // happens solely through the Eigen compile definitions attached to each executable.
    output_.noalias() = left_ * right_;
  }

  double checksum() const
  {
    return output_.sum();
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
  Eigen::MatrixXd output_;
};

EigenGemmWorkspace::EigenGemmWorkspace(int matrixN): impl_(std::make_unique<Impl>(matrixN))
{
}

EigenGemmWorkspace::~EigenGemmWorkspace() = default;

void EigenGemmWorkspace::run()
{
  impl_->run();
}

double EigenGemmWorkspace::checksum() const
{
  return impl_->checksum();
}

}  // namespace pgo::benchmark_helpers
