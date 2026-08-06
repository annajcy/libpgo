#pragma once

#include "material/elastic/elasticModel3DDeformationGradient.h"
#include "material/elastic/elasticModelDefinition.h"

namespace pgo::SolidDeformationModel
{

/// Classical logarithmic compressible Neo-Hookean material:
///
///   psi(F) = mu/2 (tr(F^T F) - 3)
///          - mu log(J)
///          + lambda/2 log(J)^2,
///   J = det(F) > 0.
class ElasticModel3DNeoHookean final
  : public ElasticModel3DDeformationGradient
{
public:
  ElasticModel3DNeoHookean(double mu, double lambda);

  int getNumParameters() const override { return 0; }

  double compute_psi(
    std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M3d compute_P(
    std::span<const double> param,
    const SpectralState &state) const override;
  EigenSupport::M9d compute_dPdF(
    std::span<const double> param,
    const SpectralState &state) const override;

  double mu() const { return mu_; }
  double lambda() const { return lambda_; }

private:
  double validateAndComputeLogJ(
    std::span<const double> param,
    const SpectralState &state) const;

  double mu_ = 0.0;
  double lambda_ = 0.0;
};

class NeoHookeanDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override
  {
    return "neo_hookean";
  }

  int numFixedChannels() const override { return 2; }
  int numOptimizableChannels() const override { return 0; }
  std::unique_ptr<ElasticModel> createModel(
    std::span<const double> fixedChannels,
    const MaterialFrame &materialFrame) const override;
};

}  // namespace pgo::SolidDeformationModel
