/*
author: Bohan Wang
copyright to USC,MIT,NUS
*/

#pragma once

#include "material/elastic/elasticModelDefinition.h"

#include "material/elastic/elasticModel3DDeformationGradient.h"

namespace pgo
{
namespace SolidDeformationModel
{
class ElasticModelVolumeMaterial : public ElasticModel3DDeformationGradient
{
public:
  ElasticModelVolumeMaterial(double coeff = 1.0):
    scale(coeff) {}
  virtual ~ElasticModelVolumeMaterial() {}

  int getNumParameters() const override { return 0; }

  double compute_psi(std::span<const double> param,
    const SpectralState &state) const override;

  EigenSupport::M3d compute_P(std::span<const double> param,
    const SpectralState &state) const override;

  EigenSupport::M9d compute_dPdF(std::span<const double> param,
    const SpectralState &state) const override;

  void setCoeff(double v) { scale = v; }

protected:
  double scale;
};
class VolumePenaltyDefinition final : public ElasticModelDefinition
{
public:
  std::string_view id() const override { return "volume"; }
  int numOptimizableChannels() const override { return 0; }
  int numFixedChannels() const override { return 1; }
  std::unique_ptr<ElasticModel> createModel(std::span<const double>, const MaterialFrame &) const override;
private:
};
}  // namespace SolidDeformationModel
}  // namespace pgo
