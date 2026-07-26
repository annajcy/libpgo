#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "material/elastic/deformationGradient/spectral/isotropicSpectralTangent.h"

#include <span>

namespace pgo::SolidDeformationModel
{

class ElasticModel3DIsotropicPrincipalStretch
  : public ElasticModel3DDeformationGradient
{
public:
  ~ElasticModel3DIsotropicPrincipalStretch() override = default;

  double compute_psi(std::span<const double> param,
    const SpectralState &state) const final;

  EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const final;

  EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const final;

  EigenSupport::M9d compute_dPdF_psd(std::span<const double> param,
    const SpectralState &state) const final;

protected:
  virtual double compute_psi_s(
    std::span<const double> param,
    const EigenSupport::V3d &s) const = 0;

  virtual EigenSupport::V3d compute_dpsi_ds(
    std::span<const double> param,
    const EigenSupport::V3d &s) const = 0;

  virtual EigenSupport::M3d compute_d2psi_ds2(
    std::span<const double> param,
    const EigenSupport::V3d &s) const = 0;
};

}  // namespace pgo::SolidDeformationModel
