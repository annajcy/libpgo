/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModelStableNeoHookeanMaterial : public ElasticModel3DDeformationGradient
{
public:
  ElasticModelStableNeoHookeanMaterial(double mu, double lambda);
  virtual ~ElasticModelStableNeoHookeanMaterial();

  int getNumParameters() const override { return 0; }

  double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M9d compute_dPdF_psd(std::span<const double> param,
    const SpectralState &state) const override;

  void setMaterial(double mu_, double lambda_);

protected:
  double _mu, _lambda, _ratio;

private:
  EigenSupport::M9d compute_dPdF_impl(std::span<const double> param,
    const SpectralState &state, bool project) const;
};
class StableNeoDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "stable_neo"; }
  MaterialChannelSchema optimizableChannelSchema() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  MaterialChannelSchema fixedChannelSchema() const override;
  std::unique_ptr<ElasticModel> createModelFromFixed(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
