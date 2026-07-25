#pragma once

#include <Eigen/Eigenvalues>

#include <stdexcept>

namespace pgo::SolidDeformationModel
{

// Return the closest symmetric positive-semidefinite matrix obtained by
// clamping the negative eigenvalues of the symmetric part to zero.
template<class MatrixLike>
typename MatrixLike::PlainObject projectSymmetricPSD(const MatrixLike &input)
{
  using Matrix = typename MatrixLike::PlainObject;
  static_assert(Matrix::RowsAtCompileTime == Matrix::ColsAtCompileTime,
    "PSD projection requires a square matrix");

  if (!input.allFinite())
    throw std::invalid_argument("cannot project a non-finite Hessian");

  const Matrix symmetric = 0.5 * (input + input.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix> solver(symmetric);
  if (solver.info() != Eigen::Success)
    throw std::runtime_error("failed to eigendecompose Hessian for PSD projection");

  const auto eigenvalues = solver.eigenvalues().cwiseMax(typename Matrix::Scalar(0));
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

}  // namespace pgo::SolidDeformationModel
