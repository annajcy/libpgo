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
class VolumePenaltyConfig final : public ElasticModelConfig
{
public:
  std::string_view id() const override { return "volume"; }
  std::span<const std::string_view> parameterChannelNames() const override;
  MaterialFrameRequirement frameRequirement() const override { return MaterialFrameRequirement::None; }
  void initializeDefaultElementChannels(const SimulationMesh &, int, std::span<double>) const override;
private:
  std::unique_ptr<ElasticModel> createModel(const SimulationMesh &, int, const MaterialFrame &) const override;
};
}  // namespace SolidDeformationModel
}  // namespace pgo
