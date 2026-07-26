#include "material/elastic/deformationGradient/spectral/isotropicSpectralTangent.h"

#include "deformation/hessianProjection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
namespace ES = EigenSupport;

const double kEqualityTolerance =
  64.0 * std::sqrt(std::numeric_limits<double>::epsilon());
constexpr double kSymmetryTolerance = 1e-10;
constexpr double kInvSqrtTwo = 0.707106781186547524400844362104849039;

constexpr std::array<std::array<int, 2>, 3> kPairs = {{
  {{0, 1}}, {{0, 2}}, {{1, 2}}
}};

bool closeStretch(double a, double b)
{
  return std::abs(a - b) <=
    kEqualityTolerance * std::max({1.0, std::abs(a), std::abs(b)});
}

void validateStretchInputs(
  const ES::V3d &s,
  const ES::V3d &dpsi_ds,
  const ES::M3d &d2psi_ds2)
{
  if (!s.allFinite() || !dpsi_ds.allFinite() || !d2psi_ds2.allFinite())
    throw std::invalid_argument(
      "isotropic spectral tangent requires finite stretch derivatives");
  if ((s.array() <= 0.0).any())
    throw std::domain_error(
      "isotropic spectral tangent requires positive principal stretches");

  const ES::M3d antisymmetric = d2psi_ds2 - d2psi_ds2.transpose();
  const double scale = std::max(1.0, d2psi_ds2.cwiseAbs().maxCoeff());
  if (antisymmetric.cwiseAbs().maxCoeff() > kSymmetryTolerance * scale)
    throw std::invalid_argument(
      "principal-stretch Hessian must be symmetric");
}

ES::V9d vec(const ES::M3d &M)
{
  ES::V9d result;
  result = Eigen::Map<const ES::V9d>(M.data());
  return result;
}

ES::M3d spectralBasis(
  const ES::M3d &U,
  const ES::M3d &V,
  int i,
  int j)
{
  return U.col(i) * V.col(j).transpose();
}

void projectPairSymmetry(
  int i,
  int j,
  ES::V3d &p,
  ES::M3d &H)
{
  const int k = 3 - i - j;
  const double pAverage = 0.5 * (p(i) + p(j));
  const double diagonalAverage = 0.5 * (H(i, i) + H(j, j));
  const double pairAverage = 0.5 * (H(i, j) + H(j, i));
  const double crossAverage = 0.5 * (H(i, k) + H(j, k));

  p(i) = pAverage;
  p(j) = pAverage;
  H(i, i) = diagonalAverage;
  H(j, j) = diagonalAverage;
  H(i, j) = pairAverage;
  H(j, i) = pairAverage;
  H(i, k) = crossAverage;
  H(j, k) = crossAverage;
  H(k, i) = crossAverage;
  H(k, j) = crossAverage;
}

}  // namespace

IsotropicSpectralTangentBlocks IsotropicSpectralTangent::compute_dPdF_blocks(
  const ES::V3d &s,
  const ES::V3d &dpsi_ds,
  const ES::M3d &d2psi_ds2)
{
  validateStretchInputs(s, dpsi_ds, d2psi_ds2);

  ES::V3d p = dpsi_ds;
  ES::M3d H = 0.5 * (d2psi_ds2 + d2psi_ds2.transpose());

  const bool equal01 = closeStretch(s(0), s(1));
  const bool equal02 = closeStretch(s(0), s(2));
  const bool equal12 = closeStretch(s(1), s(2));

  IsotropicSpectralTangentBlocks blocks;

  if (equal01 && equal02 && equal12) {
    const double sAverage = s.mean();
    const double pAverage = p.mean();
    const double diagonalAverage = H.diagonal().mean();
    const double offDiagonalAverage =
      (H(0, 1) + H(1, 0) + H(0, 2) + H(2, 0) + H(1, 2) + H(2, 1)) / 6.0;

    blocks.d2psi_ds2.setConstant(offDiagonalAverage);
    blocks.d2psi_ds2.diagonal().setConstant(diagonalAverage);
    blocks.beta.setConstant(diagonalAverage - offDiagonalAverage);
    blocks.alpha.setConstant(pAverage / sAverage);
    return blocks;
  }

  if (equal01)
    projectPairSymmetry(0, 1, p, H);
  if (equal02)
    projectPairSymmetry(0, 2, p, H);
  if (equal12)
    projectPairSymmetry(1, 2, p, H);

  blocks.d2psi_ds2 = H;
  for (int pair = 0; pair < 3; pair++) {
    const int i = kPairs[pair][0];
    const int j = kPairs[pair][1];
    if (closeStretch(s(i), s(j))) {
      blocks.beta(pair) = 0.5 * (H(i, i) + H(j, j)) - H(i, j);
    }
    else {
      blocks.beta(pair) = (p(i) - p(j)) / (s(i) - s(j));
    }
    blocks.alpha(pair) = (p(i) + p(j)) / (s(i) + s(j));
  }
  return blocks;
}

IsotropicSpectralTangentBlocks
IsotropicSpectralTangent::project_dPdF_blocks_psd(
  const IsotropicSpectralTangentBlocks &blocks)
{
  IsotropicSpectralTangentBlocks projected = blocks;
  projected.d2psi_ds2 = projectSymmetricPSD(blocks.d2psi_ds2);
  projected.beta = blocks.beta.cwiseMax(0.0);
  projected.alpha = blocks.alpha.cwiseMax(0.0);
  return projected;
}

ES::M9d IsotropicSpectralTangent::assemble_dPdF(
  const ES::M3d &U,
  const ES::M3d &V,
  const IsotropicSpectralTangentBlocks &blocks)
{
  ES::M9d result = ES::M9d::Zero();
  Eigen::Matrix<double, 9, 3> diagonalBasis =
    Eigen::Matrix<double, 9, 3>::Zero();
  for (int i = 0; i < 3; i++)
    diagonalBasis.col(i) = vec(spectralBasis(U, V, i, i));
  result.noalias() +=
    diagonalBasis * blocks.d2psi_ds2 * diagonalBasis.transpose();

  for (int pair = 0; pair < 3; pair++) {
    const int i = kPairs[pair][0];
    const int j = kPairs[pair][1];
    ES::M3d symmetric = ES::M3d::Zero();
    symmetric(i, j) = kInvSqrtTwo;
    symmetric(j, i) = kInvSqrtTwo;
    ES::M3d skew = ES::M3d::Zero();
    skew(i, j) = kInvSqrtTwo;
    skew(j, i) = -kInvSqrtTwo;

    const ES::V9d symmetricBasis = vec(U * symmetric * V.transpose());
    const ES::V9d skewBasis = vec(U * skew * V.transpose());
    result.noalias() += blocks.beta(pair) *
      (symmetricBasis * symmetricBasis.transpose());
    result.noalias() += blocks.alpha(pair) *
      (skewBasis * skewBasis.transpose());
  }
  return result;
}

ES::M3d IsotropicSpectralTangent::apply_dPdF(
  const ES::M3d &U,
  const ES::M3d &V,
  const IsotropicSpectralTangentBlocks &blocks,
  const ES::M3d &dF)
{
  const ES::M3d A = U.transpose() * dF * V;
  ES::M3d B = ES::M3d::Zero();
  B.diagonal() = blocks.d2psi_ds2 * A.diagonal();

  for (int pair = 0; pair < 3; pair++) {
    const int i = kPairs[pair][0];
    const int j = kPairs[pair][1];
    ES::M3d symmetric = ES::M3d::Zero();
    symmetric(i, j) = kInvSqrtTwo;
    symmetric(j, i) = kInvSqrtTwo;
    ES::M3d skew = ES::M3d::Zero();
    skew(i, j) = kInvSqrtTwo;
    skew(j, i) = -kInvSqrtTwo;

    const double symmetricMode =
      (A(i, j) + A(j, i)) * kInvSqrtTwo;
    const double skewMode =
      (A(i, j) - A(j, i)) * kInvSqrtTwo;
    B += blocks.beta(pair) * symmetricMode * symmetric;
    B += blocks.alpha(pair) * skewMode * skew;
  }
  return U * B * V.transpose();
}

}  // namespace pgo::SolidDeformationModel
