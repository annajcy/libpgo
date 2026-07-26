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
class ElasticModelLinearMaterial : public ElasticModel3DDeformationGradient
{
public:
  ElasticModelLinearMaterial(double mu_, double lambda_):
    mu(mu_), lambda(lambda_) {}
  virtual ~ElasticModelLinearMaterial() {}

  int getNumParameters() const override { return 0; }

  double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;

  EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;

  EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;

protected:
  double mu, lambda;
};
class LinearElasticConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "linear"; }
  std::span<const std::string_view> parameterChannelNames() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
