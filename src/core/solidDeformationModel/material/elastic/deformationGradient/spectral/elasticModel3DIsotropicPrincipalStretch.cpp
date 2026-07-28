#include "material/elastic/deformationGradient/spectral/elasticModel3DIsotropicPrincipalStretch.h"

#include <cmath>
#include <stdexcept>

namespace pgo::SolidDeformationModel
{
namespace ES = EigenSupport;

void ElasticModel3DIsotropicPrincipalStretch::validatePositiveStretches(
  const ES::V3d &s)
{
  if (!s.allFinite())
    throw std::invalid_argument(
      "isotropic principal-stretch material requires finite stretches");
  if ((s.array() <= 0.0).any())
    throw std::domain_error(
      "isotropic principal-stretch material requires positive stretches");
}

double ElasticModel3DIsotropicPrincipalStretch::compute_psi(
  std::span<const double> param,
  const SpectralState &state) const
{
  const ES::V3d &s = state.stretches;
  validatePositiveStretches(s);
  const double energy = compute_psi_s(param, s);
  if (!std::isfinite(energy))
    throw std::invalid_argument(
      "isotropic principal-stretch material returned non-finite energy");
  return energy;
}

ES::M3d ElasticModel3DIsotropicPrincipalStretch::compute_P(
  std::span<const double> param,
  const SpectralState &state) const
{
  const ES::V3d &s = state.stretches;
  validatePositiveStretches(s);
  const ES::V3d p = compute_dpsi_ds(param, s);
  if (!p.allFinite())
    throw std::invalid_argument(
      "isotropic principal-stretch material returned non-finite stress");
  return state.U * p.asDiagonal() * state.V.transpose();
}

ES::M9d ElasticModel3DIsotropicPrincipalStretch::compute_dPdF(
  std::span<const double> param,
  const SpectralState &state) const
{
  const ES::V3d &s = state.stretches;
  validatePositiveStretches(s);
  const ES::V3d p = compute_dpsi_ds(param, s);
  const ES::M3d H = compute_d2psi_ds2(param, s);
  const auto blocks = IsotropicSpectralTangent::compute_dPdF_blocks(s, p, H);
  return IsotropicSpectralTangent::assemble_dPdF(state.U, state.V, blocks);
}

ES::M9d ElasticModel3DIsotropicPrincipalStretch::compute_dPdF_psd(
  std::span<const double> param,
  const SpectralState &state) const
{
  const ES::V3d &s = state.stretches;
  validatePositiveStretches(s);
  const ES::V3d p = compute_dpsi_ds(param, s);
  const ES::M3d H = compute_d2psi_ds2(param, s);
  const auto exact = IsotropicSpectralTangent::compute_dPdF_blocks(s, p, H);
  const auto projected =
    IsotropicSpectralTangent::project_dPdF_blocks_psd(exact);
  return IsotropicSpectralTangent::assemble_dPdF(state.U, state.V, projected);
}

}  // namespace pgo::SolidDeformationModel
