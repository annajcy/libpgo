/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#include "material/elastic/elasticModelInvariantBasedMaterial.h"
#include "material/elastic/invariantBasedMaterial.h"
#include "material/elastic/invariantBasedMaterialStVK.h"



#include "EigenSupport.h"
#include "meshLinearAlgebra.h"

#include <array>
#include <cstring>
#include <iostream>

using namespace pgo::SolidDeformationModel;
namespace ES = pgo::EigenSupport;

pgo::SolidDeformationModel::ElasticModelInvariantBasedMaterial::ElasticModelInvariantBasedMaterial(
  std::unique_ptr<InvariantBasedMaterial> invMat)
  : invariantBasedMaterial_(std::move(invMat))
{
}

double ElasticModelInvariantBasedMaterial::compute_psi(std::span<const double>, const SpectralState &state) const
{
  const auto &S = state.stretches;
  const ES::V3d lambda2 = S.cwiseProduct(S);
  ES::V3d invariants;
  invariants[0] = lambda2.sum();
  invariants[1] = lambda2.squaredNorm();
  invariants[2] = lambda2.prod();

  return invariantBasedMaterial_->compute_psi(invariants);
}

ES::M3d ElasticModelInvariantBasedMaterial::compute_P(std::span<const double>, const SpectralState &state) const
{
  const auto &S = state.stretches;
  const ES::V3d lambda2 = S.cwiseProduct(S);
  ES::V3d invariants;
  invariants[0] = lambda2.sum();
  invariants[1] = lambda2.squaredNorm();
  invariants[2] = lambda2.prod();

  const ES::V3d dPsidI = invariantBasedMaterial_->compute_dpsi_dI(invariants);

  // PDiag = [ dI / dlambda ]^T * dPsidI

  ES::M3d matM;
  matM <<
    2.0 * S[0], 2.0 * S[1], 2.0 * S[2],
    4.0 * S[0] * S[0] * S[0],
    4.0 * S[1] * S[1] * S[1],
    4.0 * S[2] * S[2] * S[2],
    2.0 * S[0] * lambda2[1] * lambda2[2],
    2.0 * S[1] * lambda2[0] * lambda2[2],
    2.0 * S[2] * lambda2[0] * lambda2[1];
  const ES::V3d PDiag = matM.transpose() * dPsidI;

  // This is the 1st equation in p3 section 5 of [Irving 04]
  // P = Us[el] * diag(pHat) * trans(Vs[el])
  return state.U * PDiag.asDiagonal() * state.V.transpose();
}

// set the renumbering indices for conversion from Teran's order to row-major order
constexpr std::array<int, 9> rowMajorMatrixToTeran = { 0, 3, 5, 4, 1, 7, 6, 8, 2 };
constexpr std::array<int, 9> teranToRowMajorMatrix = { 0, 4, 8, 1, 3, 2, 6, 5, 7 };

// {i,j,m,n} goes from 0 to 2 inclusively
// converts 3x3x3x3 tensor indices to 9x9 row-major matrix indices
inline int tensor9x9Index(int i, int j, int m, int n)
{
  int rowIndex_in9x9Matrix = rowMajorMatrixToTeran[3 * i + j];
  int columnIndex_in9x9Matrix = rowMajorMatrixToTeran[3 * m + n];
  return (9 * rowIndex_in9x9Matrix + columnIndex_in9x9Matrix);
}

inline double gammaValue(
  int i, int j,
  const ES::V3d &sigma,
  const ES::V3d &invariants,
  const ES::V3d &gradient,
  const ES::V6d &hessian)
{
  const ES::V3d gammaI(
    2.0 * sigma[i],
    4.0 * sigma[i] * sigma[i] * sigma[i],
    2.0 * invariants[2] / sigma[i]);
  const ES::V3d gammaJ(
    2.0 * sigma[j],
    4.0 * sigma[j] * sigma[j] * sigma[j],
    2.0 * invariants[2] / sigma[j]);
  const ES::V3d product(
    gammaJ[0] * hessian[0] + gammaJ[1] * hessian[1] + gammaJ[2] * hessian[2],
    gammaJ[0] * hessian[1] + gammaJ[1] * hessian[3] + gammaJ[2] * hessian[4],
    gammaJ[0] * hessian[2] + gammaJ[1] * hessian[4] + gammaJ[2] * hessian[5]);

  return gammaI.dot(product) +
    4.0 * invariants[2] * gradient[2] / (sigma[i] * sigma[j]);
}

namespace pgo
{
namespace SolidDeformationModel
{
// on the "B" matrix in Teran's paper, Section 8
// Bij is the entry in row i and column j of 2x2 matrix B (which is symmetric)
inline void fixPositiveIndefiniteness(double &B11, double &B12)
{
  Vec2d eigenValues(B11 - B12, B11 + B12);

  bool hasNegativeEigenValues = false;
  for (int i = 0; i < 2; i++) {
    if (eigenValues[i] < 0)
      hasNegativeEigenValues = true;
  }

  if (hasNegativeEigenValues) {
    if (eigenValues[0] < 0)
      eigenValues[0] = 0;

    if (eigenValues[1] < 0)
      eigenValues[1] = 0;
    B11 = 0.5 * (eigenValues[0] + eigenValues[1]);
    B12 = 0.5 * (-eigenValues[0] + eigenValues[1]);
  }
}

// Project the material tangent to PSD when requested by the displacement
// Hessian assembly path.
// on the "A" matrix in Teran's paper, Section 8
// Aij is the entry in row i and column j of 3x3 matrix A (which is symmetric)
inline void fixPositiveIndefiniteness(double &A11, double &A12, double &A13, double &A22, double &A23, double &A33)
{
  Eigen::Matrix3d mat;
  mat << A11, A12, A13,
    A12, A22, A23,
    A13, A23, A33;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(mat);
  Eigen::Vector3d eigenValues = eig.eigenvalues();
  Eigen::Matrix3d eigenVectors = eig.eigenvectors();

  bool hasNegativeEigenValues = false;
  for (int i = 0; i < 3; i++) {
    if (eigenValues[i] < 0) {
      hasNegativeEigenValues = true;
      eigenValues[i] = 0;
    }
  }

  if (hasNegativeEigenValues) {
    Eigen::Matrix3d fixedMat = eigenVectors * eigenValues.asDiagonal() * eigenVectors.transpose();
    A11 = fixedMat(0, 0);
    A22 = fixedMat(1, 1);
    A33 = fixedMat(2, 2);
    A12 = fixedMat(0, 1);
    A13 = fixedMat(0, 2);
    A23 = fixedMat(1, 2);
  }
}

}  // namespace SolidDeformationModel
}  // namespace pgo

ES::M9d ElasticModelInvariantBasedMaterial::compute_dPdF_impl(std::span<const double>, const SpectralState &state, bool project) const
{
  const auto &S = state.stretches;
  const ES::V3d sigmaSquare = S.cwiseProduct(S);
  const double sigma1square = sigmaSquare[0];
  const double sigma2square = sigmaSquare[1];
  const double sigma3square = sigmaSquare[2];

  ES::V3d invariants;
  invariants[0] = sigmaSquare.sum();
  invariants[1] = sigmaSquare.squaredNorm();
  invariants[2] = sigmaSquare.prod();

  // double E[3];
  // E[0] = 0.5 * (Fhats[el][0] * Fhats[el][0] - 1);
  // E[1] = 0.5 * (Fhats[el][1] * Fhats[el][1] - 1);
  // E[2] = 0.5 * (Fhats[el][2] * Fhats[el][2] - 1);

  const ES::V3d gradient = invariantBasedMaterial_->compute_dpsi_dI(invariants);

  /*
    in order (11,12,13,22,23,33)
    | 11 12 13 |   | 0 1 2 |
    | 21 22 23 | = | 1 3 4 |
    | 31 32 33 |   | 2 4 5 |
  */
  const ES::V6d hessian = invariantBasedMaterial_->compute_d2psi_dI2(invariants);

  //// modify hessian to compute correct values if in the inversion handling regime
  // if (clamped & 1) // first S was clamped (in inversion handling)
  //{
  //   hessian[0] = hessian[1] = hessian[2] = 0.0;
  // }

  // if (clamped & 2) // second S was clamped (in inversion handling)
  //{
  //   hessian[1] = hessian[3] = hessian[4] = 0.0;
  // }

  // if (clamped & 4) // third S was clamped (in inversion handling)
  //{
  //   hessian[0] = hessian[1] = hessian[2] = hessian[4] = hessian[5] = 0.0;
  // }

  double alpha11 = 2.0 * gradient[0] + 8.0 * sigma1square * gradient[1];
  double alpha22 = 2.0 * gradient[0] + 8.0 * sigma2square * gradient[1];
  double alpha33 = 2.0 * gradient[0] + 8.0 * sigma3square * gradient[1];
  double alpha12 = 2.0 * gradient[0] + 4.0 * (sigma1square + sigma2square) * gradient[1];
  double alpha13 = 2.0 * gradient[0] + 4.0 * (sigma1square + sigma3square) * gradient[1];
  double alpha23 = 2.0 * gradient[0] + 4.0 * (sigma2square + sigma3square) * gradient[1];

  double beta11 = 4.0 * sigma1square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma1square;
  double beta22 = 4.0 * sigma2square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma2square;
  double beta33 = 4.0 * sigma3square * gradient[1] - (2.0 * invariants[2] * gradient[2]) / sigma3square;
  double beta12 = 4.0 * S[0] * S[1] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (S[0] * S[1]);
  double beta13 = 4.0 * S[0] * S[2] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (S[0] * S[2]);
  double beta23 = 4.0 * S[1] * S[2] * gradient[1] - (2.0 * invariants[2] * gradient[2]) / (S[1] * S[2]);

  double gamma11 = gammaValue(0, 0, S, invariants, gradient, hessian);
  double gamma22 = gammaValue(1, 1, S, invariants, gradient, hessian);
  double gamma33 = gammaValue(2, 2, S, invariants, gradient, hessian);
  double gamma12 = gammaValue(0, 1, S, invariants, gradient, hessian);
  double gamma13 = gammaValue(0, 2, S, invariants, gradient, hessian);
  double gamma23 = gammaValue(1, 2, S, invariants, gradient, hessian);

  double x1111, x2222, x3333;
  double x2211, x3311, x3322;
  double x2121, x3131, x3232;
  double x2112, x3113, x3223;

  x1111 = alpha11 + beta11 + gamma11;
  x2222 = alpha22 + beta22 + gamma22;
  x3333 = alpha33 + beta33 + gamma33;

  x2211 = gamma12;
  x3311 = gamma13;
  x3322 = gamma23;

  x2121 = alpha12;
  x3131 = alpha13;
  x3232 = alpha23;

  x2112 = beta12;
  x3113 = beta13;
  x3223 = beta23;

  if (project) {
    fixPositiveIndefiniteness(x1111, x2211, x3311, x2222, x3322, x3333);
    fixPositiveIndefiniteness(x2121, x2112);
    fixPositiveIndefiniteness(x3131, x3113);
    fixPositiveIndefiniteness(x3232, x3223);
  }

  ES::M9d dPdF_atFhat = ES::M9d::Zero();
  auto setTeranEntry = [&](int i, int j, int m, int n, double value) {
    const int index = tensor9x9Index(i, j, m, n);
    dPdF_atFhat(index / 9, index % 9) = value;
  };
  setTeranEntry(0, 0, 0, 0, x1111);
  setTeranEntry(0, 0, 1, 1, x2211);
  setTeranEntry(0, 0, 2, 2, x3311);

  setTeranEntry(1, 1, 0, 0, x2211);
  setTeranEntry(1, 1, 1, 1, x2222);
  setTeranEntry(1, 1, 2, 2, x3322);

  setTeranEntry(2, 2, 0, 0, x3311);
  setTeranEntry(2, 2, 1, 1, x3322);
  setTeranEntry(2, 2, 2, 2, x3333);

  setTeranEntry(0, 1, 0, 1, x2121);
  setTeranEntry(0, 1, 1, 0, x2112);

  setTeranEntry(1, 0, 0, 1, x2112);
  setTeranEntry(1, 0, 1, 0, x2121);

  setTeranEntry(0, 2, 0, 2, x3131);
  setTeranEntry(0, 2, 2, 0, x3113);

  setTeranEntry(2, 0, 0, 2, x3113);
  setTeranEntry(2, 0, 2, 0, x3131);

  setTeranEntry(1, 2, 1, 2, x3232);
  setTeranEntry(1, 2, 2, 1, x3223);

  setTeranEntry(2, 1, 1, 2, x3223);
  setTeranEntry(2, 1, 2, 1, x3232);

  /*
          | P_00 P_01 P_02 |        | F_00 F_01 F_02 |
    if P= | P_10 P_11 P_12 | and F= | F_10 F_11 F_12 |
          | P_20 P_21 P_22 |        | F_20 F_21 F_22 |

    | dP_00/dF_00  dP_00/dF_01 dP_00/dF_02 dP_00/dF_10 ... dP00/dF_22 |
    | dP_01/dF_00  dP_01/dF_01 dP_01/dF_02 dP_01/dF_10 ... dP01/dF_22 |
    | dP_02/dF_00  dP_02/dF_01 dP_02/dF_02 dP_02/dF_10 ... dP02/dF_22 |
    | dP_10/dF_00  dP_10/dF_01 dP_10/dF_02 dP_10/dF_10 ... dP10/dF_22 |
    |                               ...                               |
    | dP_22/dF_00  dP_22/dF_01 dP_22/dF_02 dP_22/dF_10 ... dP22/dF_22 |
   */

  const ES::M3d &U = state.U;
  const ES::M3d &V = state.V;
  ES::M3d UT = U.transpose();
  ES::M3d VT = V.transpose();

  /*
    U->print();
    V->print();
    UT.print();
    VT.print();
  */

  ES::M9d dPdF = ES::M9d::Zero();

  ES::V9d eiejVector = ES::V9d::Zero();

  for (int column = 0; column < 9; column++) {
    eiejVector[column] = 1.0;
    ES::M3d ei_ej = asMat3d(eiejVector.data());
    ES::M3d ut_eiej_v = UT * ei_ej * V;

    ES::V9d ut_eiej_v_TeranVector;  // in Teran order
      ut_eiej_v_TeranVector[rowMajorMatrixToTeran[0]] = ut_eiej_v(0, 0);
    ut_eiej_v_TeranVector[rowMajorMatrixToTeran[1]] = ut_eiej_v(0, 1);
    ut_eiej_v_TeranVector[rowMajorMatrixToTeran[2]] = ut_eiej_v(0, 2);
    ut_eiej_v_TeranVector[rowMajorMatrixToTeran[3]] = ut_eiej_v(1, 0);
    ut_eiej_v_TeranVector[rowMajorMatrixToTeran[4]] = ut_eiej_v(1, 1);
    ut_eiej_v_TeranVector[rowMajorMatrixToTeran[5]] = ut_eiej_v(1, 2);
    ut_eiej_v_TeranVector[rowMajorMatrixToTeran[6]] = ut_eiej_v(2, 0);
    ut_eiej_v_TeranVector[rowMajorMatrixToTeran[7]] = ut_eiej_v(2, 1);
    ut_eiej_v_TeranVector[rowMajorMatrixToTeran[8]] = ut_eiej_v(2, 2);

    ES::V9d dPdF_resultVector;
    for (int innerRow = 0; innerRow < 9; innerRow++) {
      double tempResult = 0.0;
      for (int innerColumn = 0; innerColumn < 9; innerColumn++) {
        tempResult += dPdF_atFhat(innerRow, innerColumn) *
          ut_eiej_v_TeranVector[innerColumn];
      }
      dPdF_resultVector[teranToRowMajorMatrix[innerRow]] = tempResult;
    }
    ES::M3d dPdF_resultMatrix = asMat3d(dPdF_resultVector.data());
    ES::M3d u_dpdf_vt = U * dPdF_resultMatrix * VT;

    dPdF.data()[column + 0] = u_dpdf_vt(0, 0);
    dPdF.data()[column + 9] = u_dpdf_vt(0, 1);
    dPdF.data()[column + 18] = u_dpdf_vt(0, 2);
    dPdF.data()[column + 27] = u_dpdf_vt(1, 0);
    dPdF.data()[column + 36] = u_dpdf_vt(1, 1);
    dPdF.data()[column + 45] = u_dpdf_vt(1, 2);
    dPdF.data()[column + 54] = u_dpdf_vt(2, 0);
    dPdF.data()[column + 63] = u_dpdf_vt(2, 1);
    dPdF.data()[column + 72] = u_dpdf_vt(2, 2);
    // reset
    eiejVector[column] = 0.0;
  }

  // for (int i = 0; i < 9; i++) {
  //   for (int j = 0; j < 9; j++) {
  //     std::cout << dPdF[i * 9 + j] << ',';
  //   }
  //   std::cout << std::endl;
  // }

  Eigen::PermutationMatrix<9> perm;
  perm.indices()[0] = 0;
  perm.indices()[1] = 3;
  perm.indices()[2] = 6;
  perm.indices()[3] = 1;
  perm.indices()[4] = 4;
  perm.indices()[5] = 7;
  perm.indices()[6] = 2;
  perm.indices()[7] = 5;
  perm.indices()[8] = 8;

  return perm.transpose() * dPdF * perm;
}

ES::M9d ElasticModelInvariantBasedMaterial::compute_dPdF(
  std::span<const double> param, const SpectralState &state) const
{
  return compute_dPdF_impl(param, state, false);
}

ES::M9d ElasticModelInvariantBasedMaterial::compute_dPdF_psd(
  std::span<const double> param, const SpectralState &state) const
{
  return compute_dPdF_impl(param, state, true);
}


#include <stdexcept>
#include <string>

namespace pgo::SolidDeformationModel {
namespace {
}
MaterialChannelSchema InvariantStVKDefinition::optimizableChannelSchema() const { return {}; }
MaterialChannelSchema InvariantStVKDefinition::fixedChannelSchema() const { static constexpr std::array<std::string_view, 3> names{"E", "nu", "J"}; return MaterialChannelSchema(names); }
std::unique_ptr<ElasticModel> InvariantStVKDefinition::createModelFromFixed(std::span<const double> values, const MaterialFrame &) const
{
  if (values.size() != 3) throw std::invalid_argument("inv_stvk requires fixed channels E, nu, J");
  return std::make_unique<ElasticModelInvariantBasedMaterial>(
    std::make_unique<InvariantBasedMaterialStVK>(values[0], values[1], values[2]));
}
}  // namespace pgo::SolidDeformationModel
