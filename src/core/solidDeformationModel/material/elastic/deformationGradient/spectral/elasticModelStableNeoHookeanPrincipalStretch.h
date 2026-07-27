#pragma once

#include "material/elastic/deformationGradient/spectral/elasticModel3DIsotropicPrincipalStretch.h"

#include <span>

namespace pgo::SolidDeformationModel
{

// Stable Neo-Hookean written directly as an isotropic principal-stretch
// energy. The public (mu, lambda) convention matches
// ElasticModelStableNeoHookeanMaterial.
class ElasticModelStableNeoHookeanPrincipalStretch final
  : public ElasticModel3DIsotropicPrincipalStretch
{
public:
  ElasticModelStableNeoHookeanPrincipalStretch(double mu, double lambda);

  int getNumParameters() const override { return 0; }

  void setMaterial(double mu, double lambda);

protected:
  double compute_psi_s(
    std::span<const double> param,
    const EigenSupport::V3d &s) const override;

  EigenSupport::V3d compute_dpsi_ds(
    std::span<const double> param,
    const EigenSupport::V3d &s) const override;

  EigenSupport::M3d compute_d2psi_ds2(
    std::span<const double> param,
    const EigenSupport::V3d &s) const override;

private:
  double mu_ = 0.0;
  double lambda_ = 0.0;
  double ratio_ = 0.0;
};

class StableNeoPrincipalStretchDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override
  {
    return "stable_neo_principal_stretch";
  }

  MaterialChannelSchema optimizableChannelSchema() const override;

  MaterialFrameRequirement frameRequirement() const override
  {
    return MaterialFrameRequirement::None;
  }
  MaterialChannelSchema fixedChannelSchema() const override;
  std::unique_ptr<ElasticModel> createModelFromFixed(std::span<const double>, const MaterialFrame &) const override;

};

}  // namespace pgo::SolidDeformationModel
