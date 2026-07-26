#include "material/elastic/deformationGradient/spectral/elasticModel3DIsotropicPrincipalStretch.h"

#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace
{
namespace ES = EigenSupport;

ES::V3d mapStretch(const double S[3])
{
  return ES::V3d(S[0], S[1], S[2]);
}

void validatePositiveStretch(const ES::V3d &s)
{
  if (!s.allFinite())
    throw std::invalid_argument(
      "isotropic principal-stretch material requires finite stretches");
  if ((s.array() <= 0.0).any())
    throw std::domain_error(
      "isotropic principal-stretch material requires positive stretches");
}

}  // namespace

double ElasticModel3DIsotropicPrincipalStretch::compute_psi(
  const double *param,
  const double[9],
  const double[9],
  const double[9],
  const double S[3]) const
{
  const ES::V3d s = mapStretch(S);
  validatePositiveStretch(s);
  const double energy = compute_psi_s(param, s);
  if (!std::isfinite(energy))
    throw std::invalid_argument(
      "isotropic principal-stretch material returned non-finite energy");
  return energy;
}

void ElasticModel3DIsotropicPrincipalStretch::compute_P(
  const double *param,
  const double[9],
  const double UIn[9],
  const double VIn[9],
  const double S[3],
  double POut[9]) const
{
  const ES::V3d s = mapStretch(S);
  validatePositiveStretch(s);
  const ES::M3d U = Eigen::Map<const ES::M3d>(UIn);
  const ES::M3d V = Eigen::Map<const ES::M3d>(VIn);
  const ES::V3d p = compute_dpsi_ds(param, s);
  if (!p.allFinite())
    throw std::invalid_argument(
      "isotropic principal-stretch material returned non-finite stress");
  Eigen::Map<ES::M3d> pOutMap(POut);
  pOutMap = U * p.asDiagonal() * V.transpose();
}

void ElasticModel3DIsotropicPrincipalStretch::compute_dPdF(
  const double *param,
  const double[9],
  const double UIn[9],
  const double VIn[9],
  const double S[3],
  double dPdFOut[81]) const
{
  const ES::V3d s = mapStretch(S);
  validatePositiveStretch(s);
  const ES::M3d U = Eigen::Map<const ES::M3d>(UIn);
  const ES::M3d V = Eigen::Map<const ES::M3d>(VIn);
  const ES::V3d p = compute_dpsi_ds(param, s);
  const ES::M3d H = compute_d2psi_ds2(param, s);
  const auto blocks = IsotropicSpectralTangent::compute_dPdF_blocks(s, p, H);
  Eigen::Map<ES::M9d> dPdFMap(dPdFOut);
  dPdFMap = IsotropicSpectralTangent::assemble_dPdF(U, V, blocks);
}

void ElasticModel3DIsotropicPrincipalStretch::compute_dPdF_psd(
  const double *param,
  const double[9],
  const double UIn[9],
  const double VIn[9],
  const double S[3],
  double dPdFOut[81]) const
{
  const ES::V3d s = mapStretch(S);
  validatePositiveStretch(s);
  const ES::M3d U = Eigen::Map<const ES::M3d>(UIn);
  const ES::M3d V = Eigen::Map<const ES::M3d>(VIn);
  const ES::V3d p = compute_dpsi_ds(param, s);
  const ES::M3d H = compute_d2psi_ds2(param, s);
  const auto exact = IsotropicSpectralTangent::compute_dPdF_blocks(s, p, H);
  const auto projected =
    IsotropicSpectralTangent::project_dPdF_blocks_psd(exact);
  Eigen::Map<ES::M9d> dPdFMap(dPdFOut);
  dPdFMap = IsotropicSpectralTangent::assemble_dPdF(U, V, projected);
}

}  // namespace pgo::SolidDeformationModel
